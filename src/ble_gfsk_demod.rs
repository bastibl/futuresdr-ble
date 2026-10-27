//! Continuous causal receive filtering and GFSK frequency demodulation.
use crate::ble_protocol::BlePhy;
use crate::ble_stream::BleStreamMetadata;
use anyhow::{Result, ensure};
use futuresdr::num_complex::Complex32;
use futuresdr::runtime::dev::prelude::*;

const LEVEL_TRACKING_GAIN: f32 = 0.02;
const ACQUISITION_POWER_VARIANCE: f32 = 0.2;
const TRACKING_POWER_VARIANCE: f32 = 0.35;
const UNLOCK_WINDOWS: usize = 4;

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
    powers: [f32; 16],
    estimator_confident: bool,
    incoherent_windows: usize,
    frequency_head: usize,
    tail_remaining: Option<usize>,
    started: bool,
    work_input: Vec<Complex32>,
    work_output: Vec<f32>,
    nominal_deviation: f32,
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
            history: vec![Complex32::new(0.0, 0.0); 2 * taps.len()],
            taps,
            head: 0,
            previous: Complex32::new(0.0, 0.0),
            high: 0.0,
            low: 0.0,
            frequencies: [0.0; 16],
            frequency_count: 0,
            powers: [0.0; 16],
            estimator_confident: false,
            incoherent_windows: 0,
            frequency_head: 0,
            tail_remaining: None,
            started: false,
            work_input: Vec::new(),
            work_output: Vec::new(),
            nominal_deviation: (0.5 * phy.symbol_rate_hz() / sample_rate_hz) as f32,
        };
        block.reset_dsp();
        Ok(block)
    }

    fn reset_dsp(&mut self) {
        self.history.fill(Complex32::new(0.0, 0.0));
        self.head = 0;
        self.previous = Complex32::new(0.0, 0.0);
        self.high = self.nominal_deviation;
        self.low = -self.nominal_deviation;
        self.frequency_count = 0;
        self.estimator_confident = false;
        self.incoherent_windows = 0;
        self.frequency_head = 0;
    }

    fn filter_demodulate(&mut self, sample: Complex32) -> f32 {
        let length = self.taps.len();
        // Mirrored history gives a contiguous newest-to-oldest window. Keep the
        // original accumulation order, without an integer division per tap.
        self.history[self.head] = sample;
        self.history[self.head + length] = sample;
        let window = &self.history[self.head + 1..self.head + length + 1];
        let mut filtered = Complex32::new(0.0, 0.0);
        for (&sample, &tap) in window.iter().rev().zip(&self.taps) {
            filtered += sample * tap;
        }
        self.head += 1;
        if self.head == length {
            self.head = 0;
        }
        let product = filtered * self.previous.conj();
        self.previous = filtered;
        if product.norm_sqr() == 0.0 {
            return 0.0;
        }
        let frequency = product.arg() * std::f32::consts::FRAC_1_PI;
        if !frequency.is_finite() {
            return 0.0;
        }
        let nominal = self.nominal_deviation;
        // Acquire from both tones of a constant-envelope signal. Noise has a
        // much higher relative power variance and must not train the slicer.
        // Once acquired, track gently: replacing the levels from every eight
        // bits turns payload imbalance and noise into artificial carrier drift.
        // Confidence only controls the estimator; every sample is output.
        if frequency.abs() < 0.95 {
            self.frequencies[self.frequency_head] = frequency;
            self.powers[self.frequency_head] = filtered.norm_sqr();
            self.frequency_head = (self.frequency_head + 1) % self.frequencies.len();
            self.frequency_count = (self.frequency_count + 1).min(self.frequencies.len());
            if self.frequency_count == self.frequencies.len()
                && self.frequency_head.is_multiple_of(4)
            {
                let mean_power = self.powers.iter().sum::<f32>() / 16.0;
                let power_variance = self
                    .powers
                    .iter()
                    .map(|p| (p - mean_power).powi(2))
                    .sum::<f32>()
                    / 16.0;
                let squared_power = mean_power * mean_power;
                let coherent = power_variance < TRACKING_POWER_VARIANCE * squared_power;
                if !coherent {
                    self.incoherent_windows = (self.incoherent_windows + 1).min(UNLOCK_WINDOWS);
                    // Hysteresis prevents one noisy window from retriggering
                    // fast acquisition in the middle of a weak packet. A large
                    // power step can immediately invalidate an old estimate.
                    if self.incoherent_windows == UNLOCK_WINDOWS || power_variance > squared_power {
                        self.estimator_confident = false;
                    }
                } else {
                    self.incoherent_windows = 0;
                }
                if coherent {
                    let mut sorted = self.frequencies;
                    sorted.sort_unstable_by(f32::total_cmp);
                    let low = sorted[2];
                    let high = sorted[13];
                    let span = high - low;
                    if span >= nominal
                        && span <= nominal * 3.0
                        && ((high + low) * 0.5).abs() <= nominal * 0.9
                    {
                        let gain = if self.estimator_confident {
                            LEVEL_TRACKING_GAIN
                        } else if power_variance < ACQUISITION_POWER_VARIANCE * squared_power {
                            1.0
                        } else {
                            0.0
                        };
                        self.high += gain * (high - self.high);
                        self.low += gain * (low - self.low);
                        self.estimator_confident |= gain > 0.0;
                    }
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
        // A slice is a window acquisition, not necessarily a cheap pointer
        // lookup (Studio's dynamic readers also materialize tags here).
        let capacity = self.output.slice().len();
        let (input, input_tags) = self.input.slice_with_tags();
        let n = input.len().min(capacity).min(4096);
        self.work_input.clear();
        self.work_input.extend_from_slice(&input[..n]);
        self.work_output.resize(n, 0.0);
        let mut tags: Vec<_> = input_tags
            .iter()
            .filter(|tag| tag.index < n)
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
            let mut sample = self.work_input[index];
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
            self.work_output[index] = value;
            self.next_sample_index += 1;
        }
        if n > 0 {
            self.started = true;
        }
        self.input.consume(n);
        if self.input.finished() && self.input.slice().is_empty() {
            let remaining = *self.tail_remaining.get_or_insert(if self.started {
                self.taps.len() - 1
            } else {
                0
            });
            let tail_produced = remaining.min(capacity - n);
            for _ in 0..tail_produced {
                let value = self.filter_demodulate(Complex32::new(0.0, 0.0));
                self.work_output.push(value);
                self.next_sample_index += 1;
            }
            self.tail_remaining = Some(remaining - tail_produced);
            io.finished = remaining == tail_produced;
        }
        let produced = self.work_output.len();
        {
            let (output, mut tags) = self.output.slice_with_tags();
            output[..produced].copy_from_slice(&self.work_output);
            for tag in output_tags {
                tags.add_tag(tag.index, tag.tag);
            }
        }
        self.output.produce(produced);
        io.call_again = !io.finished
            && capacity > produced
            && (!self.input.slice().is_empty() || self.tail_remaining.is_some_and(|n| n > 0));
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

#[cfg(test)]
mod tests {
    use super::*;
    use futuresdr::runtime::mocker::{Reader, Writer};

    #[test]
    fn mirrored_fir_matches_original_accumulation_across_wraps_and_reset() {
        let taps = vec![0.15, -0.3, 0.25, 0.6, -0.05, 0.35, 0.1];
        let mut demod = BleGfskDemodBlock::<Reader<Complex32>, Writer<f32>>::new(
            BlePhy::Le1M,
            2e6,
            37,
            taps.clone(),
        )
        .unwrap();
        let mut history = vec![Complex32::new(0.0, 0.0); taps.len()];
        let mut head = 0;
        for index in 0..257 {
            if index == 103 {
                demod.reset_dsp();
                history.fill(Complex32::new(0.0, 0.0));
                head = 0;
            }
            let sample = Complex32::new((index as f32 * 0.37).sin(), (index as f32 * 0.19).cos());
            history[head] = sample;
            let mut expected = Complex32::new(0.0, 0.0);
            for (lag, &tap) in taps.iter().enumerate() {
                expected += history[(head + taps.len() - lag) % taps.len()] * tap;
            }
            head = (head + 1) % taps.len();
            demod.filter_demodulate(sample);
            assert_eq!(demod.previous, expected);
        }
    }
}
