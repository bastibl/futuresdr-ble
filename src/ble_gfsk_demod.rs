//! Continuous causal receive filtering and GFSK frequency demodulation.
use crate::ble_protocol::BlePhy;
use crate::ble_stream::BleStreamMetadata;
use anyhow::{Result, ensure};
use futuresdr::num_complex::Complex32;
use futuresdr::runtime::dev::prelude::*;

/// IQ stream → normalized frequency stream, with tagged BLE metadata.
///
/// Every input sample is processed. Filter history, previous phase and the two
/// frequency-level estimates survive work calls. `frequency` changes channel
/// at the next consumed item, resetting DSP and tagging the discontinuity.
#[derive(Block)]
#[message_inputs(frequency)]
pub struct BleGfskDemodBlock<I = DefaultCpuReader<Complex32>, O = DefaultCpuWriter<f32>>
where
    I: CpuBufferReader<Item = Complex32>,
    O: CpuBufferWriter<Item = f32>,
{
    #[input]
    input: I,
    #[output]
    output: O,
    metadata: BleStreamMetadata,
    pending_metadata: bool,
    next_sample_index: u64,
    taps: Vec<f32>,
    history: Vec<Complex32>,
    head: usize,
    previous: Complex32,
    high: f32,
    low: f32,
    frequencies: [f32; 16],
    frequency_count: usize,
    frequency_head: usize,
    tail_remaining: Option<usize>,
    started: bool,
}

impl<I, O> BleGfskDemodBlock<I, O>
where
    I: CpuBufferReader<Item = Complex32>,
    O: CpuBufferWriter<Item = f32>,
{
    pub fn new(
        phy: BlePhy,
        sample_rate_hz: f64,
        channel_index: u8,
        taps: Vec<f32>,
    ) -> Result<Self> {
        let metadata = BleStreamMetadata::new(phy, sample_rate_hz, channel_index)?;
        ensure!(
            !taps.is_empty() && taps.iter().all(|v| v.is_finite()),
            "receive filter taps must be finite and nonempty"
        );
        let mut block = Self {
            input: I::default(),
            output: O::default(),
            metadata,
            pending_metadata: true,
            next_sample_index: 0,
            history: vec![Complex32::new(0.0, 0.0); taps.len()],
            taps,
            head: 0,
            previous: Complex32::new(0.0, 0.0),
            high: 0.0,
            low: 0.0,
            frequencies: [0.0; 16],
            frequency_count: 0,
            frequency_head: 0,
            tail_remaining: None,
            started: false,
        };
        block.reset_dsp();
        Ok(block)
    }

    fn reset_dsp(&mut self) {
        self.history.fill(Complex32::new(0.0, 0.0));
        self.head = 0;
        self.previous = Complex32::new(0.0, 0.0);
        let deviation =
            (0.5 * self.metadata.phy.symbol_rate_hz() / self.metadata.sample_rate_hz) as f32;
        self.high = deviation;
        self.low = -deviation;
        self.frequency_count = 0;
        self.frequency_head = 0;
    }

    fn filter_demodulate(&mut self, sample: Complex32) -> f32 {
        self.history[self.head] = sample;
        let mut filtered = Complex32::new(0.0, 0.0);
        for (lag, &tap) in self.taps.iter().enumerate() {
            filtered += self.history[(self.head + self.taps.len() - lag) % self.taps.len()] * tap;
        }
        self.head = (self.head + 1) % self.taps.len();
        let product = filtered * self.previous.conj();
        self.previous = filtered;
        if product.norm_sqr() == 0.0 {
            return 0.0;
        }
        let frequency = product.arg() * std::f32::consts::FRAC_1_PI;
        if !frequency.is_finite() {
            return 0.0;
        }
        let nominal =
            (0.5 * self.metadata.phy.symbol_rate_hz() / self.metadata.sample_rate_hz) as f32;
        // Short rolling quantiles acquire both tones after noise without a
        // decision threshold that can lock onto only one of the two levels.
        // Retain the estimate during long equal-bit runs. No samples are gated.
        if frequency.abs() < 0.95 {
            self.frequencies[self.frequency_head] = frequency;
            self.frequency_head = (self.frequency_head + 1) % self.frequencies.len();
            self.frequency_count = (self.frequency_count + 1).min(self.frequencies.len());
            if self.frequency_count == self.frequencies.len()
                && self.frequency_head.is_multiple_of(4)
            {
                let mut sorted = self.frequencies;
                sorted.sort_unstable_by(f32::total_cmp);
                let low = sorted[2];
                let high = sorted[13];
                let span = high - low;
                if span >= nominal * 0.5
                    && span <= nominal * 3.0
                    && ((high + low) * 0.5).abs() <= nominal * 0.9
                {
                    self.high = high;
                    self.low = low;
                }
            }
        }
        let deviation = ((self.high - self.low) * 0.5).clamp(nominal * 0.5, nominal * 2.0);
        (frequency - (self.high + self.low) * 0.5) / deviation
    }

    fn output_metadata(&self) -> BleStreamMetadata {
        BleStreamMetadata {
            sample_index: self.next_sample_index,
            sample_delay: self.metadata.sample_delay + (self.taps.len() - 1) as f64 * 0.5 + 0.5,
            ..self.metadata
        }
    }

    async fn frequency(
        &mut self,
        _io: &mut WorkIo,
        _mo: &mut MessageOutputs,
        _meta: &BlockMeta,
        p: Pmt,
    ) -> Result<Pmt> {
        let frequency = match p {
            Pmt::Null => {
                return Ok(Pmt::F64(crate::ble_phy::frequency_hz_from_channel_index(
                    self.metadata.channel_index,
                )?));
            }
            Pmt::F64(value) => value,
            _ => return Ok(Pmt::InvalidValue),
        };
        let Some(channel) =
            (0..40).find(|&c| crate::ble_phy::frequency_hz_from_channel_index(c) == Ok(frequency))
        else {
            return Ok(Pmt::InvalidValue);
        };
        if channel != self.metadata.channel_index {
            self.metadata.channel_index = channel;
            self.metadata.discontinuity = true;
            self.pending_metadata = true;
            self.reset_dsp();
        }
        Ok(Pmt::Ok)
    }
}

impl<I, O> Kernel for BleGfskDemodBlock<I, O>
where
    I: CpuBufferReader<Item = Complex32>,
    O: CpuBufferWriter<Item = f32>,
{
    async fn work(
        &mut self,
        io: &mut WorkIo,
        _mo: &mut MessageOutputs,
        _meta: &BlockMeta,
    ) -> Result<()> {
        let n = self
            .input
            .slice()
            .len()
            .min(self.output.slice().len())
            .min(4096);
        let mut tags: Vec<_> = self
            .input
            .slice_with_tags()
            .1
            .iter()
            .filter(|t| t.index < n)
            .cloned()
            .collect();
        tags.sort_by_key(|tag| tag.index);
        let mut tags = tags.into_iter().peekable();
        let mut output_tags = Vec::new();
        for index in 0..n {
            while tags.peek().is_some_and(|tag| tag.index == index) {
                let tag = tags.next().unwrap();
                if let Some(metadata) = BleStreamMetadata::from_tag(&tag.tag) {
                    metadata.validate()?;
                    ensure!(
                        metadata.sample_rate_hz == self.metadata.sample_rate_hz,
                        "sample-rate changes require rebuilding the receive filter"
                    );
                    let reset = metadata.discontinuity
                        || metadata.channel_index != self.metadata.channel_index
                        || metadata.sample_delay != self.metadata.sample_delay
                        || metadata.sample_index != self.next_sample_index;
                    self.metadata = *metadata;
                    self.metadata.discontinuity = reset;
                    self.next_sample_index = metadata.sample_index;
                    self.pending_metadata = true;
                    if reset {
                        self.reset_dsp();
                    }
                } else {
                    output_tags.push(ItemTag {
                        index,
                        tag: tag.tag.clone(),
                    });
                }
            }
            let mut sample = self.input.slice()[index];
            if !sample.re.is_finite() || !sample.im.is_finite() {
                self.reset_dsp();
                self.metadata.discontinuity = true;
                self.pending_metadata = true;
                sample = Complex32::new(0.0, 0.0);
            }
            if self.pending_metadata {
                output_tags.push(ItemTag {
                    index,
                    tag: self.output_metadata().to_tag(),
                });
                self.pending_metadata = false;
            }
            let value = self.filter_demodulate(sample);
            self.output.slice()[index] = value;
            self.next_sample_index += 1;
        }
        if n > 0 {
            self.started = true;
        }
        {
            let (_, mut tags) = self.output.slice_with_tags();
            for tag in output_tags {
                tags.add_tag(tag.index, tag.tag);
            }
        }
        self.input.consume(n);
        self.output.produce(n);
        if self.input.finished() && self.input.slice().is_empty() {
            let remaining = *self.tail_remaining.get_or_insert(if self.started {
                self.taps.len() - 1
            } else {
                0
            });
            let tail_produced = remaining.min(self.output.slice().len());
            for index in 0..tail_produced {
                self.output.slice()[index] = self.filter_demodulate(Complex32::new(0.0, 0.0));
                self.next_sample_index += 1;
            }
            self.output.produce(tail_produced);
            self.tail_remaining = Some(remaining - tail_produced);
            io.finished = remaining == tail_produced;
        }
        io.call_again = !io.finished
            && !self.output.slice().is_empty()
            && (!self.input.slice().is_empty() || self.tail_remaining.is_some_and(|v| v > 0));
        Ok(())
    }
}

/// Channel-selection filter for the BLE 1M GFSK frequency discriminator.
pub fn gfsk_rx_taps(sample_rate_hz: f64) -> Result<Vec<f32>> {
    anyhow::ensure!(
        sample_rate_hz.is_finite() && sample_rate_hz >= 2e6,
        "BLE 1M requires at least 2 MS/s"
    );
    Ok(futuredsp::firdes::kaiser::lowpass::<f32>(
        650e3 / sample_rate_hz,
        150e3 / sample_rate_hz,
        0.001,
    ))
}
