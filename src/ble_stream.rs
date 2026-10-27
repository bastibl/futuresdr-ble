//! Metadata for the continuous BLE sample and symbol streams.
use crate::ble_protocol::BlePhy;
use anyhow::{Result, bail};
use futuresdr::runtime::dev::Tag;

pub const BLE_STREAM_TAG: &str = "ble";

/// Configuration and source clock at the tagged stream item.
///
/// `sample_index` is measured in original per-channel IQ samples. Demodulated
/// samples retain that clock and add the causal FIR/discriminator delay.
/// Symbol streams compensate the delay and carry a `Tag::Id` on each symbol
/// with its rounded original IQ sample index. `discontinuity` resets DSP and
/// partial packet state at exactly the tagged item.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BleStreamMetadata {
    pub phy: BlePhy,
    pub channel_index: u8,
    pub sample_rate_hz: f64,
    pub sample_index: u64,
    pub sample_delay: f64,
    pub discontinuity: bool,
}

impl BleStreamMetadata {
    pub fn new(phy: BlePhy, sample_rate_hz: f64, channel_index: u8) -> Result<Self> {
        let metadata = Self {
            phy,
            channel_index,
            sample_rate_hz,
            sample_index: 0,
            sample_delay: 0.0,
            discontinuity: true,
        };
        metadata.validate()?;
        Ok(metadata)
    }

    pub fn validate(&self) -> Result<()> {
        if self.phy != BlePhy::Le1M {
            bail!("only the BLE 1M PHY is supported");
        }
        crate::ble_phy::frequency_hz_from_channel_index(self.channel_index)?;
        if !self.sample_rate_hz.is_finite() || self.sample_rate_hz < 2.0 * self.phy.symbol_rate_hz()
        {
            bail!("BLE sample rate must be finite and at least 2 samples per symbol");
        }
        if !self.sample_delay.is_finite() || self.sample_delay < 0.0 {
            bail!("invalid BLE sample delay");
        }
        Ok(())
    }

    pub fn to_tag(self) -> Tag {
        Tag::NamedAny(BLE_STREAM_TAG.to_owned(), Box::new(self))
    }

    pub fn from_tag(tag: &Tag) -> Option<&Self> {
        match tag {
            Tag::NamedAny(name, value) if name == BLE_STREAM_TAG => value.downcast_ref(),
            _ => None,
        }
    }
}
