//! Continuous interpolating Mueller and Müller timing recovery.
use crate::ble_stream::BleStreamMetadata;
use anyhow::{Result, ensure};
use futuresdr::num_complex::Complex32;
use futuresdr::runtime::dev::prelude::*;

const TIMING_GAIN: f64 = 0.175;
const RATE_GAIN: f64 = 0.25 * TIMING_GAIN * TIMING_GAIN;
const MAX_RATE_ERROR: f64 = 0.001;

/// Normalized frequency samples → one real soft symbol per symbol interval.
///
/// `output` contains `Complex32(value, 0)` immediately before bit decisions;
/// connect the same stream to the packet decoder and a constellation viewer.
/// State survives work calls. BLE metadata and other tags are mapped to the
/// first symbol at or after their input item. Every output symbol has a
/// `Tag::Id` giving its delay-compensated original IQ sample index.
#[derive(Block)]
pub struct BleSymbolSyncBlock<I = DefaultCpuReader<f32>, O = DefaultCpuWriter<Complex32>>
where
    I: CpuBufferReader<Item = f32>,
    O: CpuBufferWriter<Item = Complex32>,
{
    #[input]
    input: I,
    #[output]
    output: O,
    nominal: f64,
    period: f64,
    next_position: f64,
    input_index: u64,
    previous_input: Option<f32>,
    previous_symbol: Option<f64>,
    metadata: Option<BleStreamMetadata>,
    pending_tags: Vec<(u64, Tag)>,
}

impl<I, O> BleSymbolSyncBlock<I, O>
where
    I: CpuBufferReader<Item = f32>,
    O: CpuBufferWriter<Item = Complex32>,
{
    pub fn new(samples_per_symbol: f64) -> Result<Self> {
        ensure!(
            samples_per_symbol.is_finite() && samples_per_symbol >= 2.0,
            "timing recovery requires at least 2 samples per symbol"
        );
        Ok(Self {
            input: I::default(),
            output: O::default(),
            nominal: samples_per_symbol,
            period: samples_per_symbol,
            next_position: 0.0,
            input_index: 0,
            previous_input: None,
            previous_symbol: None,
            metadata: None,
            pending_tags: Vec::new(),
        })
    }

    fn apply_metadata(&mut self, mut metadata: BleStreamMetadata) -> Result<()> {
        metadata.validate()?;
        let reset = metadata.discontinuity
            || metadata.sample_index != self.input_index
            || self.metadata.is_none_or(|old| {
                old.channel_index != metadata.channel_index
                    || old.sample_rate_hz != metadata.sample_rate_hz
                    || old.sample_delay != metadata.sample_delay
            });
        self.nominal = metadata.sample_rate_hz / metadata.phy.symbol_rate_hz();
        if reset {
            self.period = self.nominal;
            self.next_position = metadata.sample_index as f64;
            self.previous_input = None;
            self.previous_symbol = None;
            self.pending_tags.clear();
        } else {
            self.next_position += metadata.sample_index as f64 - self.input_index as f64;
        }
        self.input_index = metadata.sample_index;
        metadata.discontinuity = reset;
        self.metadata = Some(metadata);
        self.pending_tags
            .push((metadata.sample_index, metadata.to_tag()));
        Ok(())
    }

    fn symbol(&mut self, value: f32) -> Option<(f32, u64, f64)> {
        let index = self.input_index as f64;
        self.input_index += 1;
        let previous = self.previous_input.replace(value);
        if self.next_position > index {
            return None;
        }
        let value = match previous {
            Some(previous) => {
                let fraction = (self.next_position - (index - 1.0)).clamp(0.0, 1.0) as f32;
                previous + fraction * (value - previous)
            }
            None => value,
        };
        let position = self.next_position;
        let sample_delay = self.metadata.map_or(0.0, |m| m.sample_delay);
        let source_index = (self.next_position - sample_delay).round().max(0.0) as u64;
        let current = f64::from(value).clamp(-1.0, 1.0);
        let error = self
            .previous_symbol
            .map(|last| last.signum() * current - current.signum() * last)
            .unwrap_or(0.0)
            .clamp(-1.0, 1.0);
        self.period = (self.period + RATE_GAIN * error).clamp(
            self.nominal * (1.0 - MAX_RATE_ERROR),
            self.nominal * (1.0 + MAX_RATE_ERROR),
        );
        self.next_position += self.period + TIMING_GAIN * error;
        self.previous_symbol = Some(current);
        Some((value, source_index, position))
    }
}

impl<I, O> Kernel for BleSymbolSyncBlock<I, O>
where
    I: CpuBufferReader<Item = f32>,
    O: CpuBufferWriter<Item = Complex32>,
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
        let mut produced = 0;
        let mut output_tags = Vec::new();
        for index in 0..n {
            while tags.peek().is_some_and(|tag| tag.index == index) {
                let tag = tags.next().unwrap();
                if let Some(metadata) = BleStreamMetadata::from_tag(&tag.tag) {
                    self.apply_metadata(*metadata)?;
                } else {
                    self.pending_tags.push((self.input_index, tag.tag.clone()));
                }
            }
            let input = self.input.slice()[index];
            ensure!(
                input.is_finite(),
                "symbol timing recovery requires finite frequency samples"
            );
            if let Some((value, sample_index, position)) = self.symbol(input) {
                self.output.slice()[produced] = Complex32::new(value, 0.0);
                let mut awaiting = Vec::new();
                for (tag_index, tag) in self.pending_tags.drain(..) {
                    if tag_index as f64 > position {
                        awaiting.push((tag_index, tag));
                        continue;
                    }
                    let tag = if let Some(metadata) = BleStreamMetadata::from_tag(&tag) {
                        BleStreamMetadata {
                            sample_index,
                            sample_delay: 0.0,
                            ..*metadata
                        }
                        .to_tag()
                    } else {
                        tag
                    };
                    output_tags.push(ItemTag {
                        index: produced,
                        tag,
                    });
                }
                self.pending_tags = awaiting;
                output_tags.push(ItemTag {
                    index: produced,
                    tag: Tag::Id(sample_index),
                });
                produced += 1;
            }
        }
        {
            let (_, mut tags) = self.output.slice_with_tags();
            for tag in output_tags {
                tags.add_tag(tag.index, tag.tag);
            }
        }
        self.input.consume(n);
        self.output.produce(produced);
        io.finished = self.input.finished() && self.input.slice().is_empty();
        io.call_again =
            !io.finished && !self.input.slice().is_empty() && !self.output.slice().is_empty();
        Ok(())
    }
}
