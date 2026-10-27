use anyhow::Result;
use futuresdr::blocks::{MessageSink, VectorSink};
use futuresdr::num_complex::Complex32;
use futuresdr::prelude::*;
use futuresdr::runtime::dev::prelude::*;
use futuresdr::runtime::mocker::{Mocker, Reader, Writer};
use futuresdr_ble::*;

/// Finite test source that also terminates for an empty input vector.
#[derive(Block)]
struct TestSource {
    #[output]
    output: DefaultCpuWriter<Complex32>,
    samples: Vec<Complex32>,
    offset: usize,
}
impl TestSource {
    fn new(samples: Vec<Complex32>) -> Self {
        Self {
            output: Default::default(),
            samples,
            offset: 0,
        }
    }
}
impl Kernel for TestSource {
    async fn work(
        &mut self,
        io: &mut WorkIo,
        _mo: &mut MessageOutputs,
        _meta: &BlockMeta,
    ) -> Result<()> {
        let n = self
            .output
            .slice()
            .len()
            .min(self.samples.len() - self.offset);
        self.output.slice()[..n].copy_from_slice(&self.samples[self.offset..self.offset + n]);
        self.offset += n;
        self.output.produce(n);
        io.finished = self.offset == self.samples.len();
        Ok(())
    }
}

/// Mocker input that stays live across set() calls, then explicitly reaches EOF.
#[derive(Default, Debug)]
struct LiveReader<T: CpuSample> {
    reader: Reader<T>,
    eof: bool,
    window_reads: usize,
}
impl<T: CpuSample> BufferReader for LiveReader<T> {
    type Inbox = BlockInbox;
    fn buffer_requirements(&self) -> futuresdr::runtime::buffer::dev::BufferRequirements {
        self.reader.buffer_requirements()
    }
    fn raise_buffer_requirements(
        &mut self,
        r: futuresdr::runtime::buffer::dev::BufferRequirements,
    ) {
        self.reader.raise_buffer_requirements(r);
    }
    fn init(
        &mut self,
        b: futuresdr::runtime::BlockId,
        p: futuresdr::runtime::PortIndex,
        i: BlockInbox,
    ) {
        self.reader.init(b, p, i);
    }
    fn validate(&self) -> std::result::Result<(), futuresdr::runtime::Error> {
        self.reader.validate()
    }
    async fn notify_finished(&mut self) {}
    fn finish(&mut self) {
        self.eof = true;
    }
    fn finished(&self) -> bool {
        self.eof
    }
    fn block_id(&self) -> futuresdr::runtime::BlockId {
        self.reader.block_id()
    }
    fn port_id(&self) -> futuresdr::runtime::PortIndex {
        self.reader.port_id()
    }
}
impl<T: CpuSample> CpuBufferReader for LiveReader<T> {
    type Item = T;
    fn slice(&mut self) -> &[T] {
        self.window_reads += 1;
        self.reader.slice()
    }
    fn slice_with_tags(&mut self) -> (&[T], &[ItemTag]) {
        self.window_reads += 1;
        self.reader.slice_with_tags()
    }
    fn consume(&mut self, n: usize) {
        self.reader.consume(n);
    }
    fn max_contiguous_items(&self) -> usize {
        self.reader.max_contiguous_items()
    }
}

type StreamResult = (Vec<Complex32>, Vec<ItemTag>, Vec<Vec<Pmt>>);

type Demod = BleGfskDemodBlock<LiveReader<Complex32>, Writer<f32>>;
type Sync = BleSymbolSyncBlock<LiveReader<f32>, Writer<Complex32>>;
type Decoder = BlePacketDecoderBlock<LiveReader<Complex32>>;

fn append<T>(output: &mut Vec<T>, tags: &mut Vec<ItemTag>, data: Vec<T>, incoming: Vec<ItemTag>) {
    let offset = output.len();
    output.extend(data);
    tags.extend(incoming.into_iter().map(|tag| ItemTag {
        index: tag.index + offset,
        tag: tag.tag,
    }));
}

fn streaming_samples(
    samples: &[Complex32],
    input_tags: &[ItemTag],
    chunk: usize,
    capacity: usize,
) -> Result<StreamResult> {
    let taps = gfsk_rx_taps(2e6)?;
    let flush_calls = taps.len() + 1;
    let mut demod = Mocker::new(Demod::new(BlePhy::Le1M, 2e6, 37, taps)?);
    let mut sync = Mocker::new(Sync::new(2.0)?);
    let mut decoder = Mocker::new(Decoder::new(
        BlePhy::Le1M,
        2e6,
        37,
        BlePacketDecoderOptions {
            wireshark_output: true,
            ..Default::default()
        },
    )?);
    let mut symbols = Vec::new();
    let mut symbol_tags = Vec::new();
    for start in (0..samples.len()).step_by(chunk) {
        let end = (start + chunk).min(samples.len());
        let tags = input_tags
            .iter()
            .filter(|t| t.index >= start && t.index < end)
            .map(|t| ItemTag {
                index: t.index - start,
                tag: t.tag.clone(),
            })
            .collect();
        demod
            .input()
            .reader
            .set_with_tags(samples[start..end].to_vec(), tags);
        loop {
            demod.output().reserve(capacity);
            demod.run();
            let (data, tags) = demod.output().take();
            sync.input().reader.set_with_tags(data, tags);
            drain_sync(
                &mut sync,
                &mut decoder,
                capacity,
                &mut symbols,
                &mut symbol_tags,
            );
            if demod.input().slice().is_empty() {
                break;
            }
        }
    }
    demod.input().eof = true;
    // Enough calls to flush the causal FIR even with a one-item output buffer.
    for _ in 0..flush_calls {
        demod.output().reserve(capacity);
        demod.run();
        let (data, tags) = demod.output().take();
        let empty = data.is_empty();
        sync.input().reader.set_with_tags(data, tags);
        drain_sync(
            &mut sync,
            &mut decoder,
            capacity,
            &mut symbols,
            &mut symbol_tags,
        );
        if empty {
            break;
        }
    }
    sync.input().eof = true;
    sync.output().reserve(capacity);
    sync.run();
    let (data, tags) = sync.output().take();
    decoder
        .input()
        .reader
        .set_with_tags(data.clone(), tags.clone());
    decoder.run();
    append(&mut symbols, &mut symbol_tags, data, tags);
    decoder.input().eof = true;
    decoder.run();
    decoder.deinit();
    decoder.run();
    Ok((symbols, symbol_tags, decoder.take_messages()))
}

fn drain_sync(
    sync: &mut Mocker<Sync>,
    decoder: &mut Mocker<Decoder>,
    capacity: usize,
    symbols: &mut Vec<Complex32>,
    symbol_tags: &mut Vec<ItemTag>,
) {
    while !sync.input().slice().is_empty() {
        sync.output().reserve(capacity);
        sync.run();
        let (data, tags) = sync.output().take();
        decoder
            .input()
            .reader
            .set_with_tags(data.clone(), tags.clone());
        decoder.run();
        append(symbols, symbol_tags, data, tags);
    }
}

#[test]
fn continuous_chain_preserves_results_under_chunking_and_backpressure() -> Result<()> {
    let samples = ble_sim::generate_ble_packet_samples(2, 4, 37);
    let reference = streaming_samples(&samples, &[], samples.len(), samples.len())?;
    assert_eq!(reference.2[1].len(), 5); // four packets and Finished
    for (chunk, capacity) in [(137, 31), (1, 1), (4093, 7)] {
        let result = streaming_samples(&samples, &[], chunk, capacity)?;
        assert_eq!(result, reference, "chunk={chunk} capacity={capacity}");
    }
    assert!(
        reference
            .0
            .iter()
            .all(|point| point.im == 0.0 && point.re.is_finite())
    );
    assert!(
        reference
            .0
            .iter()
            .any(|point| point.re.abs() > 0.1 && (point.re.abs() - 1.0).abs() > 0.01)
    );
    assert_eq!(
        reference
            .1
            .iter()
            .filter(|tag| matches!(tag.tag, Tag::Id(_)))
            .count(),
        reference.0.len()
    );
    Ok(())
}

#[test]
fn tagged_retunes_keep_channel_and_source_clock_aligned() -> Result<()> {
    let mut samples = Vec::new();
    let mut tags = Vec::new();
    for channel in [37, 38, 39] {
        let mut metadata = BleStreamMetadata::new(BlePhy::Le1M, 2e6, channel)?;
        metadata.sample_index = samples.len() as u64;
        tags.push(ItemTag {
            index: samples.len(),
            tag: metadata.to_tag(),
        });
        tags.push(ItemTag {
            index: samples.len() + 1,
            tag: Tag::String(format!("channel-{channel}")),
        });
        samples.extend(ble_sim::generate_ble_packet_samples(2, 3, channel));
    }
    let (_, output_tags, messages) = streaming_samples(&samples, &tags, 137, 17)?;
    assert_eq!(messages[1].len(), 10);
    for (packets, channel) in messages[1][..9].chunks(3).zip([37, 38, 39]) {
        assert!(
            packets
                .iter()
                .all(|p| matches!(p, Pmt::String(s) if s.contains(&format!("ch={channel} "))))
        );
    }
    let metadata: Vec<_> = output_tags
        .iter()
        .filter_map(|t| BleStreamMetadata::from_tag(&t.tag))
        .collect();
    assert_eq!(metadata.len(), 3);
    assert_eq!(
        metadata.iter().map(|m| m.channel_index).collect::<Vec<_>>(),
        [37, 38, 39]
    );
    assert!(
        metadata
            .iter()
            .all(|m| m.sample_delay == 0.0 && m.discontinuity)
    );
    for channel in [37, 38, 39] {
        assert!(
            output_tags
                .iter()
                .any(|t| t.tag == Tag::String(format!("channel-{channel}")))
        );
    }
    let ids: Vec<_> = output_tags
        .iter()
        .filter_map(|t| match t.tag {
            Tag::Id(id) => Some(id),
            _ => None,
        })
        .collect();
    assert!(ids.windows(2).all(|ids| ids[0] <= ids[1]));
    Ok(())
}

#[test]
fn runtime_fanout_exposes_the_exact_predecision_symbol_stream_and_drains_eof() -> Result<()> {
    for packet_count in [0, 40] {
        let mut samples = ble_sim::generate_ble_packet_samples(2, packet_count, 37);
        while samples.last().is_some_and(|s| s.norm_sqr() == 0.0) {
            samples.pop();
        }
        let mut fg = Flowgraph::new();
        let source = fg.add(TestSource::new(samples))?;
        let demod_block: BleGfskDemodBlock =
            BleGfskDemodBlock::new(BlePhy::Le1M, 2e6, 37, gfsk_rx_taps(2e6)?)?;
        let sync_block: BleSymbolSyncBlock = BleSymbolSyncBlock::new(2.0)?;
        let decoder_block: BlePacketDecoderBlock = BlePacketDecoderBlock::new(
            BlePhy::Le1M,
            2e6,
            37,
            BlePacketDecoderOptions {
                wireshark_output: true,
                ..Default::default()
            },
        )?;
        let demod = fg.add(demod_block)?;
        let sync = fg.add(sync_block)?;
        let decoder = fg.add(decoder_block)?;
        let plot = fg.add(VectorSink::<Complex32>::new(1024))?;
        let packets = fg.add(MessageSink::new())?;
        let wireshark = fg.add(MessageSink::new())?;
        connect!(fg, source > demod > sync > decoder);
        connect!(fg, sync > plot);
        fg.message(decoder, "packets", packets, "in")?;
        fg.message(decoder, "wireshark", wireshark, "in")?;
        let fg = Runtime::new().run(fg)?;
        assert_eq!(fg.block(&packets)?.received(), packet_count as u64);
        assert_eq!(fg.block(&wireshark)?.received(), packet_count as u64);
        assert!(fg.block(&plot)?.items().iter().all(|p| p.im == 0.0));
    }
    Ok(())
}

#[test]
fn continuous_receiver_decodes_noisy_iq_with_carrier_offset() -> Result<()> {
    for (channel, cfo_hz) in [(37, -150_000.0), (38, 0.0), (39, 150_000.0)] {
        let mut seed = 0x1234_5678u32;
        let mut noise = || {
            let mut sum = 0.0;
            for _ in 0..12 {
                seed ^= seed << 13;
                seed ^= seed >> 17;
                seed ^= seed << 5;
                sum += seed as f64 / u32::MAX as f64 - 0.5;
            }
            (sum * 0.015) as f32
        };
        let samples: Vec<_> = ble_sim::generate_ble_packet_samples(2, 4, channel)
            .into_iter()
            .enumerate()
            .map(|(index, sample)| {
                let phase = 0.47 + 2.0 * std::f64::consts::PI * cfo_hz * index as f64 / 2e6;
                sample * Complex32::from_polar(1.0, phase as f32) + Complex32::new(noise(), noise())
            })
            .collect();
        let tag = ItemTag {
            index: 0,
            tag: BleStreamMetadata::new(BlePhy::Le1M, 2e6, channel)?.to_tag(),
        };
        let (_, _, messages) = streaming_samples(&samples, &[tag], 137, 31)?;
        assert_eq!(messages[1].len(), 5, "channel={channel} CFO={cfo_hz}");
        assert!(
            messages[1][..4]
                .iter()
                .all(|p| matches!(p, Pmt::String(s) if s.contains(&format!("ch={channel} "))))
        );
    }
    Ok(())
}

#[test]
fn timing_recovery_decodes_fractional_clock_offsets_without_restart() -> Result<()> {
    let bits = ble_protocol::generate_ble_packet_bits(37);
    for clock_error in [-0.008, -0.002, 0.002, 0.008] {
        let actual_sps = 2.0 * (1.0 + clock_error);
        let count = ((bits.len() + 8) as f64 * actual_sps).ceil() as usize;
        let frequency: Vec<_> = (0..count)
            .map(|index| {
                let t = index as f64 / actual_sps - 0.37;
                let center = t.floor() as isize;
                ((center - 3)..=(center + 3))
                    .map(|symbol| {
                        let bit = if symbol < 0 {
                            0
                        } else {
                            bits.get(symbol as usize).copied().unwrap_or(0)
                        };
                        let sign = if bit == 0 { -1.0 } else { 1.0 };
                        sign * (((t - symbol as f64) / 0.22).tanh()
                            - ((t - symbol as f64 - 1.0) / 0.22).tanh())
                            * 0.5
                    })
                    .sum::<f64>() as f32
            })
            .collect();
        let mut sync = Mocker::new(Sync::new(2.0)?);
        let mut decoder = Mocker::new(Decoder::new(BlePhy::Le1M, 2e6, 37, Default::default())?);
        let mut samples = Vec::new();
        let mut tags = Vec::new();
        for chunk in frequency.chunks(17) {
            sync.input().reader.set(chunk.to_vec());
            drain_sync(&mut sync, &mut decoder, 7, &mut samples, &mut tags);
        }
        let messages = decoder.take_messages();
        assert_eq!(messages[1].len(), 1, "clock_error={clock_error}");
        assert!(
            samples
                .iter()
                .any(|p| p.re.abs() > 0.1 && p.re.abs() < 0.95)
        );
    }
    Ok(())
}

#[test]
fn frequency_control_tags_the_next_sample_and_discards_old_filter_history() -> Result<()> {
    let mut demod = Mocker::new(Demod::new(BlePhy::Le1M, 2e6, 37, vec![0.5, 0.5])?);
    for invalid in [
        Pmt::F64(f64::NAN),
        Pmt::F64(2403e6),
        Pmt::String("38".into()),
    ] {
        assert_eq!(demod.post("frequency", invalid)?, Pmt::InvalidValue);
        assert_eq!(demod.post("frequency", Pmt::Null)?, Pmt::F64(2402e6));
    }
    demod.input().reader.set(vec![Complex32::new(1.0, 0.0); 7]);
    demod.output().reserve(7);
    demod.run();
    demod.output().take();
    assert_eq!(demod.post("frequency", Pmt::F64(2426e6))?, Pmt::Ok);
    demod.input().reader.set(vec![Complex32::new(0.0, 0.0); 4]);
    demod.output().reserve(4);
    demod.run();
    let (samples, tags) = demod.output().take();
    assert_eq!(samples, [0.0; 4]);
    let metadata = BleStreamMetadata::from_tag(&tags[0].tag).unwrap();
    assert_eq!(tags[0].index, 0);
    assert_eq!(metadata.channel_index, 38);
    assert_eq!(metadata.sample_index, 7);
    assert!(metadata.discontinuity);
    Ok(())
}

#[test]
fn resampling_maps_tags_to_a_symbol_at_or_after_the_tagged_input() -> Result<()> {
    let mut sync = Mocker::new(Sync::new(2.0)?);
    sync.input().reader.set_with_tags(
        vec![1.0, 0.0, -0.5, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0],
        vec![ItemTag {
            index: 5,
            tag: Tag::String("marker".into()),
        }],
    );
    sync.output().reserve(10);
    sync.run();
    let (_, tags) = sync.output().take();
    let marker = tags
        .iter()
        .find(|tag| tag.tag == Tag::String("marker".into()))
        .unwrap();
    let sample = tags
        .iter()
        .find_map(|tag| match tag.tag {
            Tag::Id(sample) if tag.index == marker.index => Some(sample),
            _ => None,
        })
        .unwrap();
    assert!(sample >= 5);
    Ok(())
}

#[test]
fn buffer_windows_are_acquired_per_work_call_instead_of_per_sample() -> Result<()> {
    // Studio's dynamic native reader materializes tags on a window acquisition.
    // This guards the interface-call count, independently of the cheap Mocker
    // implementation, so a sample loop cannot silently reintroduce that cost.
    let mut demod = Mocker::new(Demod::new(BlePhy::Le1M, 2e6, 37, gfsk_rx_taps(2e6)?)?);
    demod
        .input()
        .reader
        .set(vec![Complex32::new(0.4, 0.2); 2048]);
    demod.output().reserve(2048);
    demod.run();
    assert!(demod.input().window_reads <= 3);
    let (frequency, tags) = demod.output().take();
    let mut sync = Mocker::new(Sync::new(2.0)?);
    sync.input().reader.set_with_tags(frequency, tags);
    sync.output().reserve(2048);
    sync.run();
    assert!(sync.input().window_reads <= 3);
    let (symbols, tags) = sync.output().take();
    let mut decoder = Mocker::new(Decoder::new(BlePhy::Le1M, 2e6, 37, Default::default())?);
    decoder.input().reader.set_with_tags(symbols, tags);
    decoder.run();
    assert!(decoder.input().window_reads <= 3);
    Ok(())
}
