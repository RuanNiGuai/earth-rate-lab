//! ARC8-H2 bounded, read-only evidence ledger for v1 Tracker Events.
//!
//! Critical boundary: v1 event notifications cannot attest ARC7 archive NVS
//! persistence. Even tracker-origin END/SUCCESS/STORAGE remains unverified.
//! No HID sender, calibration actuator, persistence or system clock is used here.
use std::collections::VecDeque;

use super::EventRow;
use crate::hid_log::decode::EventSource;

const MAX_OPERATIONS: usize = 96;

/// Receiver-local tracker number and a boot nonce are not globally unique.
/// Scope an observation to the *actual* HID connection generation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct OperationKey {
    pub receiver_key: String,
    pub connection_id: u64,
    pub tracker_id: u8,
    pub boot_id: u32,
    pub operation_id: u16,
    pub kind: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ExecutionEvidence {
    Seen,
    Accepted,
    Running,
    EndedSuccessUnverified,
    Failed,
    Cancelled,
    Skipped,
    Unknown,
}

impl ExecutionEvidence {
    pub fn label(self) -> &'static str {
        match self {
            Self::Seen => "事件已观察，执行情况未知",
            Self::Accepted => "Tracker 报告已接受，尚未完成",
            Self::Running => "Tracker 报告进行中，非完成证明",
            Self::EndedSuccessUnverified => "Tracker 报告 END/SUCCESS；NVS 未验证",
            Self::Failed => "Tracker 报告失败",
            Self::Cancelled => "Tracker 报告取消",
            Self::Skipped => "Tracker 报告跳过",
            Self::Unknown => "接收器报告未知 / 连接可能中断",
        }
    }
    fn terminal(self) -> bool {
        matches!(
            self,
            Self::EndedSuccessUnverified | Self::Failed | Self::Cancelled | Self::Skipped
        )
    }
}

#[derive(Clone, Debug)]
pub(crate) struct OperationEvidence {
    pub key: OperationKey,
    pub receiver_label: String,
    pub latest_elapsed_us: u64,
    pub latest_seq: u16,
    pub latest_phase: u8,
    pub latest_detail: u8,
    pub state: ExecutionEvidence,
    pub observed_packets: u32,
    pub missing_sequences: u32,
    pub synthetic_unknown_seen: bool,
    pub storage_phase_observed: bool,
}

impl OperationEvidence {
    pub fn summary(&self) -> String {
        let e = self;
        format!(
            "{} · 会话 {} · Tracker #{} · boot {:08x} · kind {} · op {} · {} · phase={} detail={} · seq={} · 缺失序号≥{} · NVS: 未验证{}",
            e.receiver_label,
            e.key.connection_id,
            e.key.tracker_id,
            e.key.boot_id,
            e.key.kind,
            e.key.operation_id,
            e.state.label(),
            e.latest_phase,
            e.latest_detail,
            e.latest_seq,
            e.missing_sequences,
            if e.synthetic_unknown_seen {
                " · 接收器曾报告 UNKNOWN"
            } else {
                ""
            }
        )
    }
}

#[derive(Default)]
pub(crate) struct EvidenceLedger {
    pub operations: VecDeque<OperationEvidence>,
    pub evicted: u64,
    pub out_of_order: u64,
}

impl EvidenceLedger {
    pub fn clear(&mut self) {
        self.operations.clear();
        self.evicted = 0;
        self.out_of_order = 0;
    }

    /// Called only for already-validated, deduplicated inbound H1 event rows.
    /// Does not interpret generic calibration STEP detail as ARC7 pose counts.
    pub fn observe(&mut self, row: &EventRow) {
        let e = &row.event;
        if !e.is_calibration() || e.operation_id == 0 || e.snapshot {
            return;
        }
        let key = OperationKey {
            receiver_key: row.receiver_key.clone(),
            connection_id: row.connection_id,
            tracker_id: e.tracker_id,
            boot_id: e.boot_id,
            operation_id: e.operation_id,
            kind: e.kind,
        };
        if let Some(pos) = self.operations.iter().position(|entry| entry.key == key) {
            let item = &mut self.operations[pos];
            let delta = e.sequence.wrapping_sub(item.latest_seq);
            if delta == 0 || delta >= 0x8000 {
                self.out_of_order = self.out_of_order.saturating_add(1);
                return;
            }
            if delta > 1 {
                item.missing_sequences =
                    item.missing_sequences.saturating_add(u32::from(delta - 1));
            }
            item.latest_seq = e.sequence;
            item.latest_elapsed_us = row.elapsed_us;
            item.latest_phase = e.phase;
            item.latest_detail = e.detail;
            item.observed_packets = item.observed_packets.saturating_add(1);
            if e.source == EventSource::Receiver {
                if e.outcome == 5 {
                    // Receiver-generated timeout, not tracker confirmation.
                    item.synthetic_unknown_seen = true;
                    item.state = ExecutionEvidence::Unknown;
                }
            } else if !item.state.terminal() {
                item.storage_phase_observed |= e.phase == 18;
                if !item.synthetic_unknown_seen {
                    // Never silently recover an unknown operation.
                    item.state = execution_state(e.action, e.outcome);
                }
            }
            // Keep most recently observed operations at the front; bounded.
            if let Some(entry) = self.operations.remove(pos) {
                self.operations.push_front(entry);
            }
            return;
        }
        if self.operations.len() == MAX_OPERATIONS {
            self.operations.pop_back();
            self.evicted = self.evicted.saturating_add(1);
        }
        let synthetic_unknown = e.source == EventSource::Receiver && e.outcome == 5;
        let state = if synthetic_unknown {
            ExecutionEvidence::Unknown
        } else if e.source == EventSource::Tracker {
            execution_state(e.action, e.outcome)
        } else {
            ExecutionEvidence::Seen
        };
        self.operations.push_front(OperationEvidence {
            key,
            receiver_label: row.receiver_label.clone(),
            latest_elapsed_us: row.elapsed_us,
            latest_seq: e.sequence,
            latest_phase: e.phase,
            latest_detail: e.detail,
            state,
            observed_packets: 1,
            missing_sequences: 0,
            synthetic_unknown_seen: synthetic_unknown,
            storage_phase_observed: e.source == EventSource::Tracker && e.phase == 18,
        });
    }
}

fn execution_state(action: u8, outcome: u8) -> ExecutionEvidence {
    match (action, outcome) {
        (1, 0) => ExecutionEvidence::Accepted,
        (2 | 3, 0) => ExecutionEvidence::Running,
        (4, 1) => ExecutionEvidence::EndedSuccessUnverified,
        (4, 2) | (5, 2) => ExecutionEvidence::Failed,
        (4, 3) => ExecutionEvidence::Cancelled,
        (4, 4) => ExecutionEvidence::Skipped,
        _ => ExecutionEvidence::Seen,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hid_log::decode::TrackerEvent;

    fn row(
        connection_id: u64,
        boot_id: u32,
        seq: u16,
        action: u8,
        outcome: u8,
        phase: u8,
        source: EventSource,
    ) -> EventRow {
        EventRow {
            receiver_key: "hid:fixture".into(),
            receiver_label: "ARC8 candidate".into(),
            connection_id,
            elapsed_us: u64::from(seq) * 1000,
            event: TrackerEvent {
                source,
                snapshot: false,
                tracker_id: 3,
                boot_id,
                sequence: seq,
                operation_id: 27,
                kind: 1,
                automatic_flag: false,
                action,
                outcome,
                phase,
                detail: 0,
            },
        }
    }

    #[test]
    fn ack_then_success_does_not_prove_nvs() {
        let mut l = EvidenceLedger::default();
        l.observe(&row(1, 42, 1, 1, 0, 1, EventSource::Tracker));
        assert_eq!(l.operations[0].state, ExecutionEvidence::Accepted);
        l.observe(&row(1, 42, 2, 3, 0, 6, EventSource::Tracker));
        l.observe(&row(1, 42, 3, 4, 1, 18, EventSource::Tracker));
        assert_eq!(
            l.operations[0].state,
            ExecutionEvidence::EndedSuccessUnverified
        );
        assert!(l.operations[0].storage_phase_observed);
        assert!(l.operations[0].summary().contains("NVS: 未验证"));
    }

    #[test]
    fn receiver_unknown_is_not_success_and_does_not_auto_recover() {
        let mut l = EvidenceLedger::default();
        l.observe(&row(1, 42, 1, 3, 0, 4, EventSource::Tracker));
        l.observe(&row(1, 42, 2, 4, 5, 18, EventSource::Receiver));
        l.observe(&row(1, 42, 3, 4, 1, 18, EventSource::Tracker));
        assert_eq!(l.operations[0].state, ExecutionEvidence::Unknown);
        assert!(l.operations[0].synthetic_unknown_seen);
    }

    #[test]
    fn same_slot_after_reconnect_or_reboot_never_merges() {
        let mut l = EvidenceLedger::default();
        l.observe(&row(1, 42, 1, 1, 0, 1, EventSource::Tracker));
        l.observe(&row(2, 42, 1, 1, 0, 1, EventSource::Tracker));
        l.observe(&row(2, 43, 1, 1, 0, 1, EventSource::Tracker));
        assert_eq!(l.operations.len(), 3);
    }

    #[test]
    fn dedup_out_of_order_and_wrapping_sequence() {
        let mut l = EvidenceLedger::default();
        l.observe(&row(1, 42, 65534, 1, 0, 1, EventSource::Tracker));
        l.observe(&row(1, 42, 0, 3, 0, 4, EventSource::Tracker));
        assert_eq!(l.operations[0].missing_sequences, 1);
        l.observe(&row(1, 42, 65535, 4, 1, 18, EventSource::Tracker));
        assert_eq!(l.operations[0].state, ExecutionEvidence::Running);
        assert_eq!(l.out_of_order, 1);
    }

    #[test]
    fn generic_event_cannot_claim_arc7_stop_progress() {
        let mut l = EvidenceLedger::default();
        let mut e = row(1, 42, 1, 3, 0, 7, EventSource::Tracker);
        e.event.detail = 5;
        l.observe(&e);
        assert!(!l.operations[0].summary().contains("5/5"));
        assert!(l.operations[0].summary().contains("NVS: 未验证"));
    }

    #[test]
    fn history_is_bounded() {
        let mut l = EvidenceLedger::default();
        for i in 1..=120_u32 {
            l.observe(&row(i as u64, i, 1, 1, 0, 1, EventSource::Tracker));
        }
        assert_eq!(l.operations.len(), MAX_OPERATIONS);
        assert_eq!(l.evicted, 24);
    }
}
