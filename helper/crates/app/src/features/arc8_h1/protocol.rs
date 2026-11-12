//! Strict ARC8-B tracker-event validation, mirrored from tracker_event_protocol.h.
//! ACK/QUERY/SUBSCRIBE are owned by the existing receiver_events::Watch.
use crate::hid_log::decode::{EventSource, TrackerEvent};

pub(crate) fn valid_event(event: &TrackerEvent) -> bool {
    let synthetic_unknown = event.source == EventSource::Receiver && event.outcome == 5;
    if event.tracker_id >= 16 || event.boot_id == 0 {
        return false;
    }
    if (1..=7).contains(&event.kind) {
        if event.operation_id == 0 || !(1..=5).contains(&event.action) {
            return false;
        }
        return match event.action {
            1..=3 => event.outcome == 0,
            4 => (1..=4).contains(&event.outcome) || synthetic_unknown,
            5 => event.outcome == 2,
            _ => false,
        };
    }
    // Automatic origin is meaningful for calibration only, never status/lifecycle.
    if event.automatic_flag || event.operation_id != 0 {
        return false;
    }
    let normal = event.outcome == 0;
    match event.kind {
        0x20 => {
            event.action == 6
                && event.phase <= 2
                && event.detail <= 4
                && (normal || synthetic_unknown)
        }
        0x21 => {
            event.action == 6
                && event.phase <= 3
                && event.detail <= 2
                && (normal || synthetic_unknown)
        }
        0x30 => {
            event.action == 7
                && normal
                && (1..=7).contains(&event.phase)
                && (if matches!(event.phase, 1 | 6) {
                    event.detail <= 2
                } else {
                    event.detail == 0
                })
        }
        0x31 => event.action == 7 && normal && event.phase == 1 && event.detail != 0,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn event(opcode: u8, kind: u8, action: u8, outcome: u8, phase: u8, detail: u8) -> TrackerEvent {
        let mut b = [0u8; 16];
        b[0] = 251;
        b[2] = opcode;
        b[3] = (outcome << 4) | action;
        b[4] = 3;
        b[5..9].copy_from_slice(&0x1234_5678_u32.to_le_bytes());
        b[9..11].copy_from_slice(&7_u16.to_le_bytes());
        if kind <= 7 {
            b[11..13].copy_from_slice(&4_u16.to_le_bytes());
        }
        b[13] = kind;
        b[14] = phase;
        b[15] = detail;
        TrackerEvent::parse(&b).expect("expected legacy event")
    }

    #[test]
    fn validates_calibration_and_rest_events() {
        assert!(valid_event(&event(225, 1, 4, 1, 18, 0)));
        assert!(valid_event(&event(225, 0x20, 6, 0, 1, 0)));
        assert!(valid_event(&event(226, 1, 4, 5, 18, 0)));
        assert!(!valid_event(&event(225, 1, 4, 5, 18, 0)));
        assert!(!valid_event(&event(225, 0x21, 7, 0, 1, 0)));
        assert!(!valid_event(&event(225, 0x30, 7, 0, 8, 0)));
        assert!(!valid_event(&event(225, 0x31, 7, 0, 1, 0)));
    }

    #[test]
    fn rejects_false_identity_and_operation() {
        let mut e = event(225, 1, 2, 0, 5, 1);
        e.boot_id = 0;
        assert!(!valid_event(&e));
        e.boot_id = 5;
        e.tracker_id = 16;
        assert!(!valid_event(&e));
        e.tracker_id = 1;
        e.operation_id = 0;
        assert!(!valid_event(&e));
    }

    #[test]
    fn rest_synthetic_unknown_is_never_a_success() {
        let e = event(226, 0x20, 6, 5, 2, 3);
        assert!(valid_event(&e));
        assert_eq!(e.outcome_name(), Some("UNKNOWN"));
    }
}
