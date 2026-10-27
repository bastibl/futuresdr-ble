//! Continuous packet synchronization and BLE link-layer decoding.
use crate::ble_connection::{
    ConnectionDecodeContext, SharedConnectionTable, format_connection_summary,
};
use crate::ble_connection_follower::format_planned_events;
use crate::ble_connection_report::{format_control_summary, format_tracking_summary};
use crate::ble_data::{BleDataPacket, format_data_packet_summary};
use crate::ble_data_stream::DataStreamDetector;
use crate::ble_detector::{DetectorEvent, DetectorStats, PacketDetector};
use crate::ble_protocol::{BlePacket, BlePhy, format_packet_summary};
use crate::ble_stream::BleStreamMetadata;
use crate::{ble_wireshark, diagnostics};
use futuresdr::num_complex::Complex32;
use futuresdr::runtime::dev::prelude::*;

#[derive(Clone, Copy, Debug, Default)]
pub struct BlePacketDecoderOptions {
    pub print_packets: bool,
    pub wireshark_output: bool,
    pub follow_connections: bool,
}

/// Consume continuous pre-decision `Complex32(value, 0)` soft symbols.
///
/// Metadata arrives in `ble` stream tags, and `Tag::Id` supplies each symbol's
/// original IQ sample index. Packet detection and partial PDUs survive work
/// calls; a tagged discontinuity resets them before the tagged symbol.
/// Only decoded packets leave through PMTs (`packets` and `wireshark`). Tap
/// the input symbol stream itself for the exact constellation before slicing.
#[derive(Block)]
#[message_outputs(wireshark, packets)]
pub struct BlePacketDecoderBlock<I = DefaultCpuReader<Complex32>>
where
    I: CpuBufferReader<Item = Complex32>,
{
    #[input]
    input: I,
    metadata: BleStreamMetadata,
    sample_index: u64,
    symbol_count: u64,
    options: BlePacketDecoderOptions,
    detector: PacketDetector,
    previous_stats: DetectorStats,
    data_detector: DataStreamDetector,
    contexts: Vec<ConnectionDecodeContext>,
    refresh_contexts: bool,
    data_packets: u64,
    nonfinite_symbols: u64,
    connections: SharedConnectionTable,
    work_input: Vec<Complex32>,
}

impl<I> BlePacketDecoderBlock<I>
where
    I: CpuBufferReader<Item = Complex32>,
{
    pub fn new(
        phy: BlePhy,
        sample_rate_hz: f64,
        channel_index: u8,
        options: BlePacketDecoderOptions,
    ) -> Result<Self> {
        Ok(Self {
            input: I::default(),
            metadata: BleStreamMetadata::new(phy, sample_rate_hz, channel_index)?,
            sample_index: 0,
            symbol_count: 0,
            options,
            detector: PacketDetector::with_packet_output(0, channel_index, false),
            previous_stats: DetectorStats::default(),
            data_detector: DataStreamDetector::default(),
            contexts: Vec::new(),
            refresh_contexts: true,
            data_packets: 0,
            nonfinite_symbols: 0,
            connections: SharedConnectionTable::default(),
            work_input: Vec::new(),
        })
    }

    pub fn with_connections(mut self, connections: SharedConnectionTable) -> Self {
        self.connections = connections;
        self
    }

    fn reset_search(&mut self) {
        self.previous_stats.merge(self.detector.stats());
        self.detector = PacketDetector::with_packet_output(0, self.metadata.channel_index, false);
        self.data_detector.reset();
        self.contexts.clear();
        self.refresh_contexts = true;
    }

    fn apply_metadata(&mut self, metadata: BleStreamMetadata) -> Result<()> {
        metadata.validate()?;
        let reset = metadata.discontinuity
            || metadata.channel_index != self.metadata.channel_index
            || metadata.sample_rate_hz != self.metadata.sample_rate_hz;
        self.metadata = metadata;
        self.sample_index = metadata.sample_index;
        if reset {
            self.reset_search();
        }
        Ok(())
    }

    fn symbol_end_sample(&self) -> u64 {
        self.sample_index.saturating_add(
            (0.5 * self.metadata.sample_rate_hz / self.metadata.phy.symbol_rate_hz()).round()
                as u64,
        )
    }

    fn process_symbol(&mut self, value: f32) -> Option<DecodedPacket> {
        if !value.is_finite() {
            self.nonfinite_symbols += 1;
            self.reset_search();
            return None;
        }
        let bit = u8::from(value > 0.0);
        if self.metadata.channel_index >= 37 {
            return match self.detector.process_symbol(bit) {
                DetectorEvent::ValidPacket { packet } => {
                    Some(DecodedPacket::Advertising(packet, self.symbol_end_sample()))
                }
                DetectorEvent::CrcRejected { reason } => {
                    self.detector.add_crc_reject();
                    if self.options.print_packets && diagnostics::enabled() {
                        println!("BLE candidate rejected: {reason}");
                    }
                    None
                }
                _ => None,
            };
        }
        if self.refresh_contexts || self.symbol_count.is_multiple_of(64) {
            self.contexts = if self.options.follow_connections {
                self.connections
                    .decode_contexts_for_window(
                        self.metadata.channel_index,
                        self.sample_index,
                        self.sample_index,
                        (2048.0 * self.metadata.sample_rate_hz / self.metadata.phy.symbol_rate_hz())
                            as u64,
                    )
                    .contexts
            } else {
                self.connections.decode_contexts()
            };
            self.contexts
                .retain(|context| context.phy == self.metadata.phy);
            self.refresh_contexts = false;
        }
        self.data_detector
            .process(
                bit,
                self.symbol_end_sample(),
                self.metadata.channel_index,
                &self.contexts,
            )
            .map(|(packet, aa_end)| DecodedPacket::Data(packet, aa_end))
    }

    async fn post_packet(&mut self, packet: DecodedPacket, mo: &mut MessageOutputs) -> Result<()> {
        let sample_rate = self.metadata.sample_rate_hz.round() as u64;
        match packet {
            DecodedPacket::Advertising(packet, end_sample) => {
                self.connections
                    .observe_packet_at(&packet, end_sample, sample_rate);
                let summary = format_packet_summary(&packet);
                if self.options.print_packets {
                    println!("BLE packet: {summary}");
                }
                if self.options.wireshark_output {
                    mo.post(
                        "wireshark",
                        Pmt::Blob(ble_wireshark::packet_to_udp_payload(&packet)),
                    )
                    .await?;
                }
                mo.post("packets", Pmt::String(summary)).await?;
            }
            DecodedPacket::Data(packet, aa_end) => {
                let event = self
                    .connections
                    .observe_data_packet_at(&packet, aa_end, sample_rate);
                self.data_packets += 1;
                let summary = format_data_packet_summary(&packet);
                if self.options.print_packets {
                    if let Some(event) = event {
                        println!(
                            "BLE data packet: {summary} event{}={} hop_ch={}",
                            if event.counter_exact { "" } else { "_phase" },
                            event.event_counter,
                            event.channel_index
                        );
                    } else {
                        println!("BLE data packet: {summary}");
                    }
                }
                if self.options.wireshark_output {
                    mo.post(
                        "wireshark",
                        Pmt::Blob(ble_wireshark::data_packet_to_udp_payload(&packet)),
                    )
                    .await?;
                }
                mo.post("packets", Pmt::String(summary)).await?;
            }
        }
        Ok(())
    }

    fn print_stats(&self) {
        let mut stats = self.previous_stats;
        stats.merge(self.detector.stats());
        if self.metadata.channel_index >= 37 {
            println!(
                "BLE summary: ch={} mode=continuous packets={} connect_ind={} crc_passes={} crc_checks={} crc_pass_rate={:.1}%",
                self.metadata.channel_index,
                stats.packets,
                stats.connect_ind_packets,
                stats.packets,
                stats.pdu_attempts(),
                stats.crc_pass_rate()
            );
        } else {
            let checks = self.data_packets + self.data_detector.crc_rejects;
            println!(
                "BLE summary: ch={} mode=continuous data_packets={} crc_passes={} crc_checks={} crc_pass_rate={:.1}%",
                self.metadata.channel_index,
                self.data_packets,
                self.data_packets,
                checks,
                if checks == 0 {
                    0.0
                } else {
                    100.0 * self.data_packets as f64 / checks as f64
                }
            );
        }
        let debug = self.options.print_packets && diagnostics::enabled();
        if debug {
            println!(
                "BLE detector diagnostics: ch={} symbols={} aa_candidates={} data_aa_candidates={} crc_rejects={} invalid_symbols={}",
                self.metadata.channel_index,
                self.symbol_count,
                stats.aa_candidates,
                self.data_detector.aa_candidates,
                stats.crc_rejects + self.data_detector.crc_rejects,
                self.nonfinite_symbols
            );
        }
        if self.connections.finish_reporter() {
            if debug {
                for connection in self.connections.snapshot() {
                    println!(
                        "BLE connection: ch={} {}",
                        connection.first_channel,
                        format_connection_summary(&connection)
                    );
                }
                for plan in self.connections.follow_plans(8) {
                    println!(
                        "BLE follow plan: aa=0x{:08X} next_events=[{}]",
                        plan.access_address,
                        format_planned_events(&plan.events)
                    );
                }
            }
            let summary = self.connections.tracking_summary();
            if self.options.follow_connections || summary.connect_ind_seen > 0 {
                println!("BLE tracking summary: {}", format_tracking_summary(summary));
            }
            if let Some(control) = format_control_summary(summary) {
                println!("BLE control summary: {control}");
            }
        }
    }
}

enum DecodedPacket {
    Advertising(BlePacket, u64),
    Data(BleDataPacket, u64),
}

impl<I> Kernel for BlePacketDecoderBlock<I>
where
    I: CpuBufferReader<Item = Complex32>,
{
    async fn work(
        &mut self,
        io: &mut WorkIo,
        mo: &mut MessageOutputs,
        _meta: &BlockMeta,
    ) -> Result<()> {
        let (input, input_tags) = self.input.slice_with_tags();
        let n = input.len().min(4096);
        self.work_input.clear();
        self.work_input.extend_from_slice(&input[..n]);
        let mut tags: Vec<_> = input_tags
            .iter()
            .filter(|tag| tag.index < n)
            .cloned()
            .collect();
        tags.sort_by_key(|tag| tag.index);
        let mut tags = tags.into_iter().peekable();
        for index in 0..n {
            while tags.peek().is_some_and(|tag| tag.index == index) {
                let tag = tags.next().unwrap();
                if let Some(metadata) = BleStreamMetadata::from_tag(&tag.tag) {
                    self.apply_metadata(*metadata)?;
                }
                if let Tag::Id(sample_index) = tag.tag {
                    self.sample_index = sample_index;
                }
            }
            let value = self.work_input[index].re;
            if let Some(packet) = self.process_symbol(value) {
                self.post_packet(packet, mo).await?;
            }
            self.symbol_count += 1;
            self.sample_index +=
                (self.metadata.sample_rate_hz / self.metadata.phy.symbol_rate_hz()).round() as u64;
        }
        self.input.consume(n);
        io.finished = self.input.finished() && self.input.slice().is_empty();
        io.call_again = !io.finished && !self.input.slice().is_empty();
        Ok(())
    }

    async fn deinit(&mut self, mo: &mut MessageOutputs, _meta: &BlockMeta) -> Result<()> {
        if self.options.wireshark_output {
            mo.post("wireshark", Pmt::Finished).await?;
        }
        mo.post("packets", Pmt::Finished).await?;
        self.print_stats();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use futuresdr::runtime::mocker::{Mocker, Reader};

    fn frame_bytes(channel: u8, pdu_type: u8, payload: &[u8]) -> Vec<u8> {
        let mut pdu = vec![pdu_type, payload.len() as u8];
        pdu.extend_from_slice(payload);
        let crc = crate::ble_phy::crc24_ble(&pdu, crate::ble_phy::BLE_ADV_CRC_INIT);
        pdu.extend_from_slice(&crc.to_le_bytes()[..3]);
        crate::ble_phy::apply_whitening(channel, &pdu).unwrap()
    }

    fn feed(
        decoder: &mut Mocker<BlePacketDecoderBlock<Reader<Complex32>>>,
        channel: u8,
        start: u64,
        access_address: u32,
        pdu: &[u8],
    ) {
        let bytes = [0xaa]
            .into_iter()
            .chain(access_address.to_le_bytes())
            .chain(pdu.iter().copied());
        let symbols: Vec<_> = bytes
            .flat_map(|byte| {
                (0..8).map(move |bit| {
                    Complex32::new(if byte & (1 << bit) == 0 { -0.8 } else { 1.2 }, 0.0)
                })
            })
            .collect();
        for (chunk_index, chunk) in symbols.chunks(17).enumerate() {
            let offset = chunk_index * 17;
            let mut tags: Vec<_> = (0..chunk.len())
                .map(|i| ItemTag {
                    index: i,
                    tag: Tag::Id(start + (offset + i) as u64 * 2 + 1),
                })
                .collect();
            if offset == 0 {
                let mut metadata = BleStreamMetadata::new(BlePhy::Le1M, 2e6, channel).unwrap();
                metadata.sample_index = start;
                tags.insert(
                    0,
                    ItemTag {
                        index: 0,
                        tag: metadata.to_tag(),
                    },
                );
            }
            decoder.input().set_with_tags(chunk.to_vec(), tags);
            decoder.run();
        }
    }

    #[test]
    fn tagged_source_clock_drives_connection_following_and_control_updates() {
        let table = SharedConnectionTable::default();
        let mut decoder = Mocker::new(
            BlePacketDecoderBlock::<Reader<Complex32>>::new(
                BlePhy::Le1M,
                2e6,
                37,
                BlePacketDecoderOptions {
                    follow_connections: true,
                    wireshark_output: true,
                    print_packets: false,
                },
            )
            .unwrap()
            .with_connections(table.clone()),
        );
        let access_address: u32 = 0xa1b2_c3d4;
        let mut request = vec![0xaa; 6];
        request.extend([0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
        request.extend(access_address.to_le_bytes());
        request.extend([0x56, 0x34, 0x12, 2]);
        request.extend(4u16.to_le_bytes());
        request.extend(24u16.to_le_bytes());
        request.extend(0u16.to_le_bytes());
        request.extend(200u16.to_le_bytes());
        request.extend([0xff, 0xff, 0xff, 0xff, 0x1f, (3 << 5) | 12]);
        let request_pdu = frame_bytes(37, 5, &request);
        let request_bits = (1 + 4 + request_pdu.len()) * 8;
        feed(
            &mut decoder,
            37,
            100_000 - request_bits as u64 * 2,
            crate::ble_phy::BLE_ACCESS_ADDRESS,
            &request_pdu,
        );
        assert_eq!(table.tracking_summary().connect_ind_seen, 1);
        let payload: Vec<_> = (0..251).map(|i| (i * 73 + 19) as u8).collect();
        let data = crate::ble_data::build_data_pdu_crc(12, 0x123456, 0x02, &payload);
        feed(&mut decoder, 12, 112_500, access_address, &data);
        assert_eq!(decoder.data_packets, 1);
        assert_eq!(table.tracking_summary().locked_connections, 1);
        let terminate = crate::ble_data::build_data_pdu_crc(24, 0x123456, 0x03, &[0x02, 0x13]);
        feed(&mut decoder, 24, 172_500, access_address, &terminate);
        assert_eq!(table.tracking_summary().connections, 0);
        assert_eq!(decoder.data_packets, 2);
        let messages = decoder.take_messages();
        assert_eq!(messages[0].len(), 3);
        assert_eq!(messages[1].len(), 3);
        assert!(matches!(&messages[1][1], Pmt::String(s) if s.contains("payload_len=251")));
    }
}
