//! Bluetooth LE receive blocks, protocol decoding, and deterministic IQ fixtures.

pub mod ble_ad;
pub mod ble_channel_selection;
pub mod ble_connection;
pub mod ble_connection_follower;
mod ble_connection_report;
pub mod ble_control;
pub mod ble_data;
mod ble_detector;
pub mod ble_packet_decoder;
pub mod ble_phy;
pub mod ble_protocol;
pub mod ble_sim;
pub mod ble_symbol_sync;
mod ble_timing_recovery;
pub mod ble_wireshark;
pub mod channelizer;
mod diagnostics;

mod ble_data_stream;
pub mod ble_gfsk_demod;
pub mod ble_stream;

pub use ble_gfsk_demod::{BleGfskDemodBlock, gfsk_rx_taps};
pub use ble_packet_decoder::{BlePacketDecoderBlock, BlePacketDecoderOptions};
pub use ble_protocol::BlePhy;
pub use ble_stream::{BLE_STREAM_TAG, BleStreamMetadata};
pub use ble_symbol_sync::BleSymbolSyncBlock;
