use std::path::PathBuf;

use anyhow::Result;
use anyhow::bail;
use clap::Parser;

use crate::ble_protocol;

#[derive(Parser, Debug)]
#[command(version)]
pub(crate) struct Args {
    #[arg(short, long, default_value = "")]
    pub(crate) args: String,
    #[arg(short, long, value_delimiter = ',', default_value = "37")]
    pub(crate) channels: Vec<u8>,
    #[arg(short, long, default_value_t = 2.0e6)]
    pub(crate) sample_rate: f64,
    #[arg(short, long, default_value_t = 30.0)]
    pub(crate) gain: f64,
    #[arg(long)]
    pub(crate) antenna: Option<String>,
    #[arg(long, default_value_t = false)]
    pub(crate) follow_connections: bool,
    #[arg(long, default_value_t = false)]
    pub(crate) quiet: bool,
    #[arg(long, default_value_t = false)]
    pub(crate) wireshark: bool,
    #[arg(long, default_value = "127.0.0.1:55556")]
    pub(crate) wireshark_addr: String,
    #[arg(long, default_value_t = false)]
    pub(crate) simulate: bool,
    #[arg(long, default_value_t = 1)]
    pub(crate) simulate_packets: usize,
    #[arg(long)]
    pub(crate) iq_file: Option<PathBuf>,
    #[arg(long)]
    pub(crate) duration: Option<f64>,
}

impl Args {
    pub(crate) fn is_multi_channel(&self) -> bool {
        let mut channels = self.channels.clone();
        channels.sort_unstable();
        channels.dedup();
        channels.len() > 1
    }
}

pub(crate) fn validate(args: &Args) -> Result<()> {
    if args.simulate && args.iq_file.is_some() {
        bail!("--simulate and --iq-file cannot be used together");
    }
    if args.follow_connections && !args.is_multi_channel() {
        bail!("--follow-connections requires at least two --channels values");
    }
    if !args.sample_rate.is_finite() || args.sample_rate < 2.0e6 {
        bail!("--sample-rate must be finite and at least 2 MS/s");
    }

    Ok(())
}

pub(crate) fn selected_channels(args: &Args) -> Result<Vec<u8>> {
    let mut channels = args.channels.clone();

    channels.sort_unstable();
    channels.dedup();

    for &channel in &channels {
        ble_protocol::frequency_hz_from_channel_index(channel)?;
    }

    Ok(channels)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_to_advertising_channel_37() {
        let args = Args::try_parse_from(["sniffer"]).unwrap();
        assert_eq!(selected_channels(&args).unwrap(), [37]);
        validate(&args).unwrap();
    }

    #[test]
    fn multi_channel_following_uses_continuous_receiver() {
        let args = Args::try_parse_from(["sniffer", "--channels", "37,0", "--follow-connections"])
            .unwrap();
        assert!(args.is_multi_channel());
        assert_eq!(selected_channels(&args).unwrap(), [0, 37]);
        validate(&args).unwrap();
    }

    #[test]
    fn removes_mode_and_squelch_flags() {
        for flags in [
            ["--decoder", "burst"],
            ["--decoder", "continuous"],
            ["--squelch-db", "10"],
            ["--squelch-margin-db", "13"],
        ] {
            assert!(Args::try_parse_from(["sniffer", flags[0], flags[1]]).is_err());
        }
    }

    #[test]
    fn rejects_insufficient_sample_rate_and_single_channel_following() {
        let args = Args::try_parse_from(["sniffer", "--sample-rate", "1e6"]).unwrap();
        assert!(validate(&args).is_err());
        let args = Args::try_parse_from(["sniffer", "--follow-connections"]).unwrap();
        assert!(validate(&args).is_err());
    }
}
