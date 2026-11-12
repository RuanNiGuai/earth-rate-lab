use super::*;
use crate::test_support;
use gpui_kit::TestAppContext;
use std::{sync::mpsc, time::Duration};

#[test]
fn repeated_installation_is_labeled_as_readback_without_rewrite() {
    let mut result = json!({"installation":{"state":"APPLY_READBACK_VERIFIED","written_slots":[]}});
    assert_eq!(completion_label(&result), "fleet.already_installed");
    result["installation"]["written_slots"] = json!([3]);
    assert_eq!(completion_label(&result), "fleet.installed");
    result["installation"]["state"] = json!("ROLLBACK_READBACK_VERIFIED");
    assert_eq!(completion_label(&result), "fleet.complete");
}

#[test]
fn guided_capture_order_requires_each_accepted_predecessor() {
    let mut workflow = json!({"captures":{}});
    assert!(capture_allowed(&workflow, "x"));
    assert!(!capture_allowed(&workflow, "y"));
    workflow["captures"]["x"] = json!({"accepted":true});
    assert!(capture_allowed(&workflow, "y"));
    assert!(!capture_allowed(&workflow, "poses"));
    workflow["captures"]["y"] = json!({"accepted":true});
    workflow["captures"]["z"] = json!({"accepted":false});
    assert!(!capture_allowed(&workflow, "poses"));
    workflow["captures"]["z"]["accepted"] = json!(true);
    assert!(capture_allowed(&workflow, "poses"));
    workflow["captures"]["x"]["accepted"] = json!(false);
    assert!(!capture_allowed(&workflow, "poses"));
    assert!(!capture_allowed(&workflow, "unknown"));
}

#[gpui_kit::test]
fn verified_session_is_retained_and_recheck_preserves_imports(cx: &mut TestAppContext) {
    let runtime = crate::app_runtime::AppRuntime::new().unwrap();
    let _ui = test_support::UiTestSession::new(cx);
    let executor = runtime.executor();
    let cx = cx.add_empty_window();
    let workbench = cx.new_window_entity(move |window, cx| {
        crate::application::test_support::offline_workbench(
            window,
            cx,
            crate::sessions::test_support::display_manager(),
            executor,
        )
    });
    // Include the result shape emitted by the older toolkit: no output/reused.
    for result in [
        json!({"action":"init", "state":"SESSION_READY", "session":"data/fleet-one"}),
        json!({"action":"init", "state":"SESSION_READY", "session":"data/fleet-one", "reused":true}),
    ] {
        let reused = result["reused"] == true;
        workbench.update(cx, |this, _| {
            this.fleet
                .paths
                .insert("recording", "imported.jsonl".into());
            this.fleet.reviewed = Some(("reviewed.json".into(), "hash".into()));
            this.fleet.job = Some(
                Job::start(move |_, _| {
                    Ok(Completion {
                        result,
                        log: "operation/output.log".into(),
                    })
                })
                .unwrap(),
            );
        });
        for _ in 0..100 {
            if workbench.update(cx, |this, _| {
                this.poll_fleet();
                this.fleet.job.is_none()
            }) {
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        workbench.update(cx, |this, _| {
            assert!(!this.fleet.busy());
            assert_eq!(this.fleet.paths["session"], PathBuf::from("data/fleet-one"));
            assert_eq!(this.fleet.paths.contains_key("recording"), reused);
            assert_eq!(this.fleet.reviewed.is_some(), reused);
            let result = &this.fleet.completion.as_ref().unwrap().result;
            assert_eq!(
                completion_label(result),
                if reused {
                    "fleet.session_reused"
                } else {
                    "fleet.session_created"
                }
            );
            assert_ne!(
                progress_text(&json!({"stage":"starting", "action":"init"})),
                progress_text(&json!({"stage":"starting", "action":"record"}))
            );
            let checking = progress_text(
                &json!({"stage":"checking", "round":2, "rounds":3, "checked":7, "total":10}),
            );
            assert!(checking.contains("2/3") && checking.contains("7/10"));
        });
    }
}

#[gpui_kit::test]
fn rejected_plan_clears_prior_write_admission_and_retains_review(cx: &mut TestAppContext) {
    let runtime = crate::app_runtime::AppRuntime::new().unwrap();
    let _ui = test_support::UiTestSession::new(cx);
    let executor = runtime.executor();
    let cx = cx.add_empty_window();
    let workbench = cx.new_window_entity(move |window, cx| {
        crate::application::test_support::offline_workbench(
            window,
            cx,
            crate::sessions::test_support::display_manager(),
            executor,
        )
    });
    workbench.update(cx, |this, _| {
        this.fleet.reviewed = Some(("old-plan.json".into(), "old-hash".into()));
        this.fleet.job = Some(
            Job::start(|_, _| {
                Ok(Completion {
                    result: json!({"action":"plan", "state":"PLAN_REJECTED",
                        "installable":false, "session":"data/fleet",
                        "failures":["#5 Earth RMS 0.00329 > 0.001 degrees/s"],
                        "workflow":{"artifacts":{"result":"fit.json","review":"review.json"}}}),
                    log: "review/output.log".into(),
                })
            })
            .unwrap(),
        );
    });
    for _ in 0..100 {
        if workbench.update(cx, |this, _| {
            this.poll_fleet();
            this.fleet.job.is_none()
        }) {
            break;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    workbench.update(cx, |this, _| {
        assert!(!this.fleet.busy());
        assert!(this.fleet.reviewed.is_none());
        assert!(this.fleet.error.is_none());
        assert_eq!(this.fleet.workflow["artifacts"]["result"], "fit.json");
        let result = &this.fleet.completion.as_ref().unwrap().result;
        assert_eq!(completion_label(result), "fleet.wizard.plan_rejected");
        assert_eq!(result["failures"].as_array().unwrap().len(), 1);
    });
}

#[gpui_kit::test]
fn cancellation_keeps_device_leases_until_worker_joins(cx: &mut TestAppContext) {
    let runtime = crate::app_runtime::AppRuntime::new().unwrap();
    let _ui = test_support::UiTestSession::new(cx);
    let executor = runtime.executor();
    let cx = cx.add_empty_window();
    let workbench = cx.new_window_entity(move |window, cx| {
        crate::application::test_support::offline_workbench(
            window,
            cx,
            crate::sessions::test_support::display_manager(),
            executor,
        )
    });
    let (release, wait) = mpsc::channel();
    let receiver = ReceiverEndpoint {
        key: "fleet-test".into(),
        path: b"fake-hid".to_vec(),
        label: "Test".into(),
        serial_number: Some("TEST".into()),
    };
    workbench.update(cx, |this, cx| {
        this.fleet
            .paths
            .insert("session", "data/existing-session".into());
        this.monitor =
            crate::discovery::ReceiverMonitor::fixture(receiver.clone(), 0, "010203040506", true);
        this.fleet.hid = Some(this.monitor.reserve(&receiver).unwrap());
        this.fleet
            .ports
            .push(this.manager.reserve("fixture-port", None).unwrap());
        this.fleet.job = Some(
            Job::start(move |_, _| {
                wait.recv().unwrap();
                Err("stopped".into())
            })
            .unwrap(),
        );
        this.fleet_stop(cx);
        this.poll_fleet();
        assert!(this.fleet.device_busy());
        assert!(this.monitor.is_reserved(&receiver));
        assert!(this.manager.reserve("fixture-port", None).is_err());
        assert!(this.joint_check_device_idle().is_err());
    });
    release.send(()).unwrap();
    for _ in 0..100 {
        let done = workbench.update(cx, |this, _| {
            this.poll_fleet();
            this.fleet.job.is_none()
        });
        if done {
            break;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    workbench.update(cx, |this, _| {
        assert!(this.fleet.job.is_none());
        assert!(this.fleet.completion.is_none());
        assert!(this.fleet.error.as_ref().unwrap().contains("stopped"));
        assert_eq!(
            this.fleet.paths["session"],
            PathBuf::from("data/existing-session")
        );
        assert!(!this.fleet.device_busy());
        assert!(!this.monitor.is_reserved(&receiver));
        assert!(this.joint_check_device_idle().is_ok());
        let lease = this.manager.reserve("fixture-port", None).unwrap();
        this.manager.release(&lease).unwrap();
    });
}

#[gpui_kit::test]
fn missing_identity_and_shutdown_do_not_start_hardware_jobs(cx: &mut TestAppContext) {
    let runtime = crate::app_runtime::AppRuntime::new().unwrap();
    let _ui = test_support::UiTestSession::new(cx);
    let executor = runtime.executor();
    let cx = cx.add_empty_window();
    let workbench = cx.new_window_entity(move |window, cx| {
        crate::application::test_support::offline_workbench(
            window,
            cx,
            crate::sessions::test_support::display_manager(),
            executor,
        )
    });
    workbench.update(cx, |this, cx| {
        this.fleet.paths.insert("toolkit", "missing-toolkit".into());
        this.fleet.paths.insert("session", "missing-session".into());
        this.fleet_start("record", true, cx);
        assert!(this.fleet.error.is_some());
        assert!(!this.fleet.busy());
        this.fleet.error = None;
        this.shutting_down = true;
        this.fleet_start("record", true, cx);
        this.fleet_choose("toolkit", true, cx);
        assert!(this.fleet.error.is_none());
        assert!(!this.fleet.busy());
    });
}

#[test]
fn recommended_step_respects_rejected_or_missing_captures() {
    let mut w = json!({});
    assert_eq!(recommended_step(&w), 0);
    w["preflight"] = json!({});
    assert_eq!(recommended_step(&w), 1);
    w["captures"] = json!({"x":{"accepted":true},"y":{"accepted":true},"z":{"accepted":true}});
    assert_eq!(recommended_step(&w), 2);
    w["captures"]["poses"] = json!({"accepted":true});
    assert_eq!(recommended_step(&w), 3);
    w["artifacts"] = json!({"result":"result.json"});
    assert_eq!(recommended_step(&w), 4);
    w["captures"]["z"]["accepted"] = json!(false);
    assert_eq!(recommended_step(&w), 1);
}

#[gpui_kit::test]
fn comparison_failure_revokes_write_admission_before_worker_runs(cx: &mut TestAppContext) {
    let runtime = crate::app_runtime::AppRuntime::new().unwrap();
    let _ui = test_support::UiTestSession::new(cx);
    let executor = runtime.executor();
    let cx = cx.add_empty_window();
    let workbench = cx.new_window_entity(move |window, cx| {
        crate::application::test_support::offline_workbench(
            window,
            cx,
            crate::sessions::test_support::display_manager(),
            executor,
        )
    });
    workbench.update(cx, |this, cx| {
        this.fleet.paths.remove("session");
        this.fleet.reviewed = Some(("old-plan.json".into(), "hash".into()));
        this.fleet.comparison = json!({"changed_devices":10});
        this.fleet_start("compare", false, cx);
        assert!(this.fleet.error.is_some());
        assert!(this.fleet.reviewed.is_none());
        assert!(this.fleet.comparison.is_null());
        assert!(!this.fleet.busy());
    });
}

#[test]
fn comparison_presents_rest_and_motion_with_field_decisions() {
    let report = json!({"metrics":{"selected":{"held_rest_deg_per_min":{"rms":0.194},
        "held_closure_deg":{"rms":11.311}}},"decisions":[{"slot":2,"scale_updated":true,
        "values":{"scale":[1.01,1.02,1.03]},"turn_rms_deg":{"old":14.0,"new":2.0}}]});
    let lines = comparison_lines(&report);
    assert!(
        lines
            .iter()
            .any(|s| s.contains("0.1940 °/min") && s.contains("11.3110°"))
    );
    assert!(
        lines
            .iter()
            .any(|s| s.contains("#2") && s.contains("14.000° → 2.000°"))
    );
    assert!(comparison_lines(&Value::Null).is_empty());
}
