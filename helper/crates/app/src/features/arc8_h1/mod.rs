//! ARC8-H1 opt-in read-only diagnostics. No independent HID handles or OTA commands.
mod evidence;
pub(crate) mod offline;
pub(crate) mod protocol;

use evidence::EvidenceLedger;

use std::collections::VecDeque;

use crate::hid_log::decode::{self, DecodedData, EventSource, TrackerEvent};
use crate::hid_log::{self, Direction, Interface, Record, TraceItem};
use protocol::valid_event;

const MAX_ROWS: usize = 256;
const POLL_BUDGET: usize = 1024;

#[derive(Clone, Debug)]
pub(crate) struct EventRow {
    pub receiver_key: String,
    pub receiver_label: String,
    pub connection_id: u64,
    pub elapsed_us: u64,
    pub event: TrackerEvent,
}

impl EventRow {
    pub fn synthetic_unknown(&self) -> bool {
        self.event.source == EventSource::Receiver && self.event.outcome == 5
    }

    pub fn describe(&self) -> String {
        let e = &self.event;
        let kind = match e.kind {
            1 => "IMU_ZRO",
            2 => "ACCEL_POSES",
            3 => "MAG_MANUAL",
            4 => "MAG_ONLINE",
            5 => "GYRO_SENS",
            6 => "TCAL_BOOT",
            7 => "TCAL_RUNTIME",
            0x20 => "TRACKER_REST",
            0x21 => "FUSION_REST",
            0x30 => "POWER",
            0x31 => "BUTTON",
            _ => "UNKNOWN",
        };
        let source = if e.source == EventSource::Tracker {
            "Tracker"
        } else {
            "Receiver"
        };
        let authority = if self.synthetic_unknown() {
            "SYNTHETIC UNKNOWN (not success)"
        } else if e.snapshot {
            "snapshot (not a new operation)"
        } else {
            "event (best effort)"
        };
        format!(
            "{source} #{id} · {kind} · {action} · {outcome} · {phase} · detail={detail} · op={op} seq={seq} · {authority}",
            id = e.tracker_id,
            action = e.action_name().unwrap_or("UNKNOWN"),
            outcome = e.outcome_name().unwrap_or("UNKNOWN"),
            phase = e.phase_name().unwrap_or("UNKNOWN"),
            detail = e
                .detail_name()
                .map(str::to_owned)
                .unwrap_or_else(|| e.detail.to_string()),
            op = e.operation_id,
            seq = e.sequence,
        )
    }
}

#[derive(Default)]
pub(crate) struct State {
    subscription: Option<hid_log::Subscription>,
    pub rows: VecDeque<EventRow>,
    pub dropped: u64,
    pub invalid: u64,
    pub error: Option<String>,
    pub evidence: EvidenceLedger,
}

impl State {
    pub fn activate(&mut self) {
        if self.subscription.is_none() {
            self.subscription = Some(hid_log::subscribe());
        }
    }

    pub fn deactivate(&mut self) {
        if let Some(mut subscriber) = self.subscription.take() {
            subscriber.stop();
        }
    }

    pub fn clear(&mut self) {
        self.rows.clear();
        self.dropped = 0;
        self.invalid = 0;
        self.error = None;
        self.evidence.clear();
    }

    pub fn poll(&mut self) -> bool {
        let mut changed = false;
        for _ in 0..POLL_BUDGET {
            let Some(item) = self
                .subscription
                .as_mut()
                .and_then(hid_log::Subscription::try_recv)
            else {
                break;
            };
            match item {
                TraceItem::Gap { count } => {
                    self.dropped = self.dropped.saturating_add(count);
                    changed = true;
                }
                TraceItem::Record(record) => {
                    changed |= self.ingest(&record);
                }
            }
        }
        changed
    }

    fn ingest(&mut self, record: &Record) -> bool {
        if record.source.interface != Interface::Main
            || record.source.vendor_id != 0x1209
            || record.source.product_id != 0x7690
            || !matches!(
                &record.data,
                hid_log::RecordData::Report {
                    direction: Direction::In,
                    ..
                }
            )
        {
            return false;
        }
        let mut changed = false;
        for decoded in decode::decode(record) {
            if let DecodedData::Event(event) = decoded.data {
                if !valid_event(&event) {
                    self.invalid = self.invalid.saturating_add(1);
                    changed = true;
                    continue;
                }
                if self.rows.iter().any(|row| {
                    let old = &row.event;
                    row.receiver_key == record.source.receiver_key
                        && row.connection_id == record.source.connection_id
                        && old.tracker_id == event.tracker_id
                        && old.boot_id == event.boot_id
                        && old.sequence == event.sequence
                        && old.operation_id == event.operation_id
                        && old.kind == event.kind
                        && old.action == event.action
                        && old.outcome == event.outcome
                        && old.source == event.source
                }) {
                    continue;
                }
                if self.rows.len() == MAX_ROWS {
                    self.rows.pop_front();
                }
                let row = EventRow {
                    receiver_key: record.source.receiver_key.clone(),
                    receiver_label: record.source.label.clone(),
                    connection_id: record.source.connection_id,
                    elapsed_us: record.elapsed_us,
                    event,
                };
                self.evidence.observe(&row);
                self.rows.push_back(row);
                changed = true;
            }
        }
        changed
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::ReceiverEndpoint;
    use crate::hid_log::{RecordData, ReportBytes, Source};
    use std::sync::Arc;

    fn event_record(direction: Direction, mut wire: [u8; 16]) -> Record {
        // 16-byte ARC8 tracker event packed into a 64-byte HID input report.
        if wire[0] == 0 {
            wire[0] = 251;
        }
        let mut report = [0u8; 64];
        report[0..16].copy_from_slice(&wire);
        let receiver = ReceiverEndpoint {
            key: "hid:test".to_owned(),
            path: b"arc8-hid-test".to_vec(),
            label: "Test candidate".to_owned(),
            serial_number: None,
        };
        Record {
            sequence: 1,
            elapsed_us: 1_000_000,
            source: Arc::new(Source::main(&receiver)),
            data: RecordData::Report {
                direction,
                bytes: ReportBytes::new(&report).unwrap(),
                transferred: Some(64),
                error: None,
            },
        }
    }

    fn sample_event() -> [u8; 16] {
        let mut wire = [0u8; 16];
        wire[0] = 251;
        wire[2] = 225; // tracker-side event
        wire[3] = 0x14; // END / SUCCESS
        wire[4] = 2;
        wire[5..9].copy_from_slice(&0x12_34_56_78u32.to_le_bytes());
        wire[9..11].copy_from_slice(&10u16.to_le_bytes());
        wire[11..13].copy_from_slice(&17u16.to_le_bytes());
        wire[13] = 1; // IMU ZRO
        wire[14] = 18; // STORAGE phase, NOT a persisted ACK
        wire
    }

    #[test]
    fn only_inbound_valid_events_are_displayed_once() {
        let mut state = State::default();
        let inbound = event_record(Direction::In, sample_event());
        assert!(state.ingest(&inbound));
        assert_eq!(state.rows.len(), 1);
        assert!(!state.ingest(&inbound));
        assert_eq!(state.rows.len(), 1);
        assert!(!state.ingest(&event_record(Direction::Out, sample_event())));
        assert_eq!(state.rows.len(), 1);
        assert!(state.rows[0].describe().contains("op=17"));
        assert!(!state.rows[0].describe().contains("NVS persisted"));
    }

    #[test]
    fn rejects_invalid_event_payload_from_legacy_decoder() {
        let mut state = State::default();
        let mut event = sample_event();
        event[5..9].fill(0); // invalid boot nonce
        assert!(state.ingest(&event_record(Direction::In, event)));
        assert_eq!(state.rows.len(), 0);
        assert_eq!(state.invalid, 1);
    }

    #[test]
    fn synthetic_unknown_is_not_presented_as_success() {
        let mut wire = sample_event();
        wire[2] = 226;
        wire[3] = 0x54; // receiver synthesized END / UNKNOWN
        let mut state = State::default();
        assert!(state.ingest(&event_record(Direction::In, wire)));
        assert!(state.rows[0].synthetic_unknown());
        assert!(state.rows[0].describe().contains("not success"));
    }
}
