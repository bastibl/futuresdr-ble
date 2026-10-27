//! Incremental connection-specific data PDU detection on the symbol stream.
use crate::ble_connection::ConnectionDecodeContext;
use crate::ble_data::{BLE_MAX_DATA_PAYLOAD_LEN, BleDataPacket, parse_data_pdu};

#[derive(Default)]
pub(crate) struct DataStreamDetector {
    shift: u32,
    capture: Option<DataCapture>,
    pub(crate) aa_candidates: u64,
    pub(crate) crc_rejects: u64,
}
struct DataCapture {
    context: ConnectionDecodeContext,
    inverted: bool,
    bits: Vec<u8>,
    needed: Option<usize>,
    aa_end_sample: u64,
}

impl DataStreamDetector {
    pub(crate) fn reset(&mut self) {
        self.shift = 0;
        self.capture = None;
    }

    pub(crate) fn process(
        &mut self,
        bit: u8,
        sample_index: u64,
        channel: u8,
        contexts: &[ConnectionDecodeContext],
    ) -> Option<(BleDataPacket, u64)> {
        if let Some(capture) = &mut self.capture {
            capture.bits.push(bit ^ u8::from(capture.inverted));
            if capture.bits.len() == 16 {
                let header = bits_to_bytes(&capture.bits);
                let header = crate::ble_phy::apply_whitening(channel, &header).ok()?;
                let length = header[1] as usize;
                if length > BLE_MAX_DATA_PAYLOAD_LEN {
                    self.crc_rejects += 1;
                    self.reset();
                    return None;
                }
                capture.needed = Some((2 + length + 3) * 8);
            }
            if capture.needed == Some(capture.bits.len()) {
                let capture = self.capture.take().unwrap();
                let bytes = bits_to_bytes(&capture.bits);
                self.shift = 0;
                match parse_data_pdu(
                    channel,
                    capture.context.phy,
                    capture.context.access_address,
                    capture.context.crc_init,
                    &bytes,
                ) {
                    Ok(packet) => return Some((packet, capture.aa_end_sample)),
                    Err(_) => self.crc_rejects += 1,
                }
            }
            return None;
        }
        self.shift = (self.shift >> 1) | ((bit as u32) << 31);
        if let Some(context) = contexts
            .iter()
            .find(|c| self.shift == c.access_address || self.shift == !c.access_address)
        {
            self.aa_candidates += 1;
            self.capture = Some(DataCapture {
                context: *context,
                inverted: self.shift != context.access_address,
                bits: Vec::with_capacity(16),
                needed: None,
                aa_end_sample: sample_index,
            });
        }
        None
    }
}

fn bits_to_bytes(bits: &[u8]) -> Vec<u8> {
    bits.as_chunks::<8>()
        .0
        .iter()
        .map(|bits| bits.iter().enumerate().fold(0, |v, (i, b)| v | (b << i)))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ble_phy::BlePhy;

    #[test]
    fn decodes_maximum_data_pdu_incrementally_with_both_polarities() {
        let context = ConnectionDecodeContext {
            access_address: 0xa1b2_c3d4,
            crc_init: 0x123456,
            phy: BlePhy::Le1M,
        };
        let payload: Vec<_> = (0..251).map(|i| (i * 73 + 19) as u8).collect();
        let pdu = crate::ble_data::build_data_pdu_crc(12, context.crc_init, 0x02, &payload);
        let bytes: Vec<_> = context
            .access_address
            .to_le_bytes()
            .into_iter()
            .chain(pdu)
            .collect();
        let bits: Vec<_> = bytes
            .into_iter()
            .flat_map(|byte| (0..8).map(move |bit| (byte >> bit) & 1))
            .collect();
        for inverted in [0, 1] {
            let mut detector = DataStreamDetector::default();
            let mut packets = Vec::new();
            let mut index = 0;
            for chunk in bits.chunks(17) {
                for &bit in chunk {
                    if let Some(packet) =
                        detector.process(bit ^ inverted, 1000 + index * 2, 12, &[context])
                    {
                        packets.push(packet);
                    }
                    index += 1;
                }
            }
            assert_eq!(packets.len(), 1);
            assert_eq!(packets[0].0.payload, payload);
            assert_eq!(packets[0].1, 1062);
            assert_eq!(detector.aa_candidates, 1);
            assert_eq!(detector.crc_rejects, 0);
        }
    }

    #[test]
    fn discontinuity_discards_partial_data_and_crc_failures_do_not_emit_packets() {
        let context = ConnectionDecodeContext {
            access_address: 0xa1b2_c3d4,
            crc_init: 0x123456,
            phy: BlePhy::Le1M,
        };
        let mut pdu = crate::ble_data::build_data_pdu_crc(12, context.crc_init, 0x02, &[1, 2, 3]);
        pdu[2] ^= 1;
        let bits: Vec<_> = context
            .access_address
            .to_le_bytes()
            .into_iter()
            .chain(pdu)
            .flat_map(|byte| (0..8).map(move |bit| (byte >> bit) & 1))
            .collect();
        let mut detector = DataStreamDetector::default();
        assert!(
            bits.iter()
                .enumerate()
                .all(|(i, &bit)| detector.process(bit, i as u64, 12, &[context]).is_none())
        );
        assert_eq!(detector.crc_rejects, 1);
        for &bit in &bits[..45] {
            detector.process(bit, 0, 12, &[context]);
        }
        detector.reset();
        assert!(
            bits[45..]
                .iter()
                .all(|&bit| detector.process(bit, 0, 12, &[context]).is_none())
        );
    }
}
