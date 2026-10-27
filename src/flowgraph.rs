use crate::ble_connection::SharedConnectionTable;
use crate::channelizer::{
    BLE_CHANNEL_SPACING_HZ, center_frequency_for_channels, pfb_channelizer_taps,
    pfb_channels_from_sample_rate, pfb_port_for_frequency,
};
use crate::cli::Args;
use crate::{ble_protocol, ble_sim};
use anyhow::{Result, bail};
use futuresdr::blocks::seify::Builder;
use futuresdr::blocks::{BlobToUdp, FileSource, NullSink, PfbChannelizer, VectorSource};
use futuresdr::num_complex::Complex32;
use futuresdr::prelude::*;
use futuresdr::runtime::{BlockId, BlockRef};
use futuresdr_ble::{
    BleGfskDemodBlock, BlePacketDecoderBlock, BlePacketDecoderOptions, BlePhy, BleSymbolSyncBlock,
    gfsk_rx_taps,
};

pub(crate) fn build_single_channel_flowgraph(
    fg: &mut Flowgraph,
    args: &Args,
    sample_rate: f64,
) -> Result<()> {
    let channel = args.channels[0];
    let frequency = ble_protocol::frequency_hz_from_channel_index(channel)?;
    println!(
        "Using BLE channel {channel} ({:.3} MHz), continuous streaming receiver",
        frequency / 1e6
    );
    let connections = SharedConnectionTable::with_monitored_channels(&[channel]);
    let (demod, packets) = add_receiver(fg, args, sample_rate, channel, connections)?;
    if args.simulate {
        let ratio = sample_rate / BlePhy::Le1M.symbol_rate_hz();
        if (ratio - ratio.round()).abs() > 1e-9 {
            bail!("the deterministic IQ simulator requires an integer samples-per-symbol rate");
        }
        println!("Simulation mode: Testing full GFSK streaming DSP chain");
        let src = fg.add(VectorSource::<Complex32>::new(
            ble_sim::generate_ble_packet_samples(ratio as usize, args.simulate_packets, channel),
        ))?;
        connect!(fg, src > demod);
    } else if let Some(iq_file) = &args.iq_file {
        println!(
            "File mode: Reading interleaved f32 IQ samples from {}",
            iq_file.display()
        );
        let src = fg.add(FileSource::<Complex32>::new(iq_file, false))?;
        connect!(fg, src > demod);
    } else {
        let src = fg.add(
            Builder::new(args.args.clone())?
                .frequency(frequency)
                .sample_rate(sample_rate)
                .gain(args.gain)
                .antenna(args.antenna.clone())
                .build_source()?,
        )?;
        connect!(fg, src.outputs[0] > demod);
    }
    connect_wireshark(fg, packets, args)
}

pub(crate) fn build_multi_channel_flowgraph(
    fg: &mut Flowgraph,
    args: &Args,
    sample_rate: f64,
    channels: &[u8],
) -> Result<()> {
    let pfb_channels = pfb_channels_from_sample_rate(sample_rate)?;
    let center = center_frequency_for_channels(channels)?;
    let channel_rate = sample_rate / pfb_channels as f64;
    if (channel_rate - BLE_CHANNEL_SPACING_HZ).abs() > 1.0 {
        bail!("multi-channel output rate must be 2 MS/s per channel");
    }
    println!(
        "Multi-channel continuous mode: center={:.3} MHz sample_rate={:.3} MS/s channels={channels:?}",
        center / 1e6,
        sample_rate / 1e6
    );
    let connections = SharedConnectionTable::with_monitored_channels(channels);
    let src = fg.add(
        Builder::new(args.args.clone())?
            .frequency(center)
            .sample_rate(sample_rate)
            .gain(args.gain)
            .antenna(args.antenna.clone())
            .build_source()?,
    )?;
    let channelizer_block: PfbChannelizer =
        PfbChannelizer::new(pfb_channels, &pfb_channelizer_taps(pfb_channels), 1.0);
    let channelizer = fg.add(channelizer_block)?;
    connect!(fg, src.outputs[0] > channelizer);
    let mut selected = Vec::new();
    for &channel in channels {
        let frequency = ble_protocol::frequency_hz_from_channel_index(channel)?;
        selected.push((
            channel,
            pfb_port_for_frequency(frequency, center, pfb_channels)?,
        ));
    }
    for port in 0..pfb_channels {
        if !selected.iter().any(|(_, selected)| *selected == port) {
            let sink = fg.add(NullSink::<Complex32>::new())?;
            fg.stream_dyn(channelizer, format!("outputs[{port}]"), sink, "input")?;
        }
    }
    for (channel, port) in selected {
        let (demod, packets) = add_receiver(fg, args, channel_rate, channel, connections.clone())?;
        fg.stream_dyn(channelizer, format!("outputs[{port}]"), demod, "input")?;
        connect_wireshark(fg, packets, args)?;
    }
    Ok(())
}

fn add_receiver(
    fg: &mut Flowgraph,
    args: &Args,
    sample_rate: f64,
    channel: u8,
    connections: SharedConnectionTable,
) -> Result<(BlockRef<BleGfskDemodBlock>, BlockRef<BlePacketDecoderBlock>)> {
    let samples_per_symbol = sample_rate / BlePhy::Le1M.symbol_rate_hz();
    let taps = gfsk_rx_taps(sample_rate)?;
    let demod = fg.add(BleGfskDemodBlock::new(
        BlePhy::Le1M,
        sample_rate,
        channel,
        taps,
    )?)?;
    let symbols_block: BleSymbolSyncBlock = BleSymbolSyncBlock::new(samples_per_symbol)?;
    let symbols = fg.add(symbols_block)?;
    let packets = fg.add(
        BlePacketDecoderBlock::new(
            BlePhy::Le1M,
            sample_rate,
            channel,
            BlePacketDecoderOptions {
                print_packets: !args.quiet,
                wireshark_output: args.wireshark,
                follow_connections: args.follow_connections,
            },
        )?
        .with_connections(connections),
    )?;
    connect!(fg, demod > symbols > packets);
    Ok((demod, packets))
}

fn connect_wireshark(fg: &mut Flowgraph, source: impl Into<BlockId>, args: &Args) -> Result<()> {
    if args.wireshark {
        let udp = fg.add(BlobToUdp::new(&args.wireshark_addr))?;
        fg.message(source.into(), "wireshark", udp, "in")?;
        println!(
            "Streaming BLE LL packets to Wireshark UDP endpoint {}.",
            args.wireshark_addr
        );
    }
    Ok(())
}
