use super::{FeaturePage, Workbench};
use crate::{
    discovery::ReceiverReservation,
    domain::{ReceiverEndpoint, console_port_allowed},
    features::fleet_calibration::service::{self, Completion, Request},
    i18n::tr,
    sessions::SerialReservation,
    update::Job,
};
use gpui_kit::component::{
    ActiveTheme, Disableable, Sizable,
    button::{Button, ButtonVariants},
    checkbox::Checkbox,
    input::{Input, InputState},
};
use gpui_kit::{
    AnyElement, Context, Entity, FontWeight, PathPromptOptions, Task, Window, div, prelude::*,
};
use serde_json::{Value, json};
use std::{collections::BTreeMap, path::PathBuf};

#[cfg(test)]
#[path = "tests.rs"]
mod tests;

fn completion_label(result: &Value) -> &'static str {
    if result["installation"]["state"] == "APPLY_READBACK_VERIFIED" {
        if result["installation"]["written_slots"]
            .as_array()
            .is_some_and(|v| v.is_empty())
        {
            "fleet.already_installed"
        } else {
            "fleet.installed"
        }
    } else if result["state"] == "PREFLIGHT_FAILED" {
        "fleet.wizard.preflight_failed"
    } else if result["state"] == "CAPTURE_REJECTED" {
        "fleet.wizard.rejected"
    } else if result["state"] == "PLAN_REJECTED" {
        "fleet.wizard.plan_rejected"
    } else if result["action"]
        .as_str()
        .is_some_and(|a| a.starts_with("workflow-record-"))
    {
        "fleet.wizard.recorded"
    } else if result["action"] == "init" && result["state"] == "SESSION_READY" {
        if result["reused"] == true {
            "fleet.session_reused"
        } else {
            "fleet.session_created"
        }
    } else if result["transport_pass"] == false {
        "fleet.incomplete"
    } else {
        "fleet.complete"
    }
}

fn capture_allowed(workflow: &Value, kind: &str) -> bool {
    let kinds = ["x", "y", "z", "poses"];
    kinds.iter().position(|k| *k == kind).is_some_and(|index| {
        kinds[..index]
            .iter()
            .all(|k| workflow["captures"][*k]["accepted"] == true)
    })
}

fn default_session_parent() -> PathBuf {
    directories::ProjectDirs::from("org", "SlimeNRF", "SlimeNRF Helper")
        .map(|d| d.data_local_dir().join("calibration/sessions"))
        .unwrap_or_else(|| PathBuf::from("calibration-sessions"))
}

fn comparison_lines(comparison: &Value) -> Vec<String> {
    if !comparison.is_object() {
        return Vec::new();
    }
    let mut lines = vec![tr("fleet.comparison.metrics")];
    for key in ["old", "selected", "stable_numeric", "adaptive_rest"] {
        let row = &comparison["metrics"][key];
        if let (Some(rest), Some(motion)) = (
            row["held_rest_deg_per_min"]["rms"].as_f64(),
            row["held_closure_deg"]["rms"].as_f64(),
        ) {
            lines.push(format!(
                "{} · {:.4} °/min · {:.4}°",
                tr(&format!("fleet.comparison.{key}")),
                rest,
                motion
            ));
        }
    }
    lines.push(tr("fleet.comparison.limits"));
    lines.push(tr("fleet.comparison.retained"));
    if let Some(rows) = comparison["decisions"].as_array() {
        for row in rows {
            lines.push(format!(
                "#{} · {} · S={} · RMS {:.3}° → {:.3}°",
                row["slot"],
                tr(if row["scale_updated"] == true {
                    "fleet.comparison.update"
                } else {
                    "fleet.comparison.keep"
                }),
                row["values"]["scale"],
                row["turn_rms_deg"]["old"].as_f64().unwrap_or(f64::NAN),
                row["turn_rms_deg"]["new"].as_f64().unwrap_or(f64::NAN)
            ));
        }
    }
    lines
}

fn workflow_button(
    action: &'static str,
    hardware: bool,
    enabled: bool,
    cx: &mut Context<Workbench>,
) -> Button {
    Button::new(format!("wizard-{action}"))
        .label(tr(&format!(
            "fleet.wizard.action.{}",
            action.replace('-', "_")
        )))
        .disabled(!enabled)
        .on_click(cx.listener(move |this, _, _, cx| this.fleet_start(action, hardware, cx)))
}

fn progress_text(progress: &Value) -> String {
    let stage = progress["stage"].as_str().unwrap_or("starting");
    if stage == "checking" {
        format!(
            "{} {}/{} · {} {}/{}",
            tr("fleet.check_round"),
            progress["round"],
            progress["rounds"],
            tr("fleet.checked"),
            progress["checked"],
            progress["total"]
        )
    } else if stage == "temperature" {
        format!(
            "{} · {}/{}",
            tr("workbench.temperature_progress"),
            progress["completed"],
            progress["total"]
        )
    } else if stage == "snapshot" {
        format!(
            "{} · {}/{}",
            tr("fleet.stage.snapshot"),
            progress["completed"],
            progress["total"]
        )
    } else if stage == "starting" && progress["action"] == "record" {
        tr("fleet.stage.record_preparing")
    } else {
        tr(&format!("fleet.stage.{stage}"))
    }
}

pub(super) struct FleetUiState {
    paths: BTreeMap<&'static str, PathBuf>,
    latitude: Entity<InputState>,
    build: Entity<InputState>,
    receiver: Option<ReceiverEndpoint>,
    confirmed: bool,
    reviewed: Option<(PathBuf, String)>,
    comparison: Value,
    chooser: Option<Task<()>>,
    pub(super) job: Option<Job<Completion>>,
    pending: Option<Request>,
    hid: Option<ReceiverReservation>,
    ports: Vec<SerialReservation>,
    completion: Option<Completion>,
    error: Option<String>,
    advanced: bool,
    details: bool,
    step: Option<usize>,
    pub(super) legacy_tools: bool,
    pub(super) single_capture: bool,
    workflow: Value,
    capture_kind: Option<&'static str>,
}
impl FleetUiState {
    pub(super) fn new(window: &mut Window, cx: &mut Context<Workbench>) -> Self {
        Self {
            paths: BTreeMap::from([("session", default_session_parent())]),
            latitude: cx.new(|cx| InputState::new(window, cx).placeholder("-89 .. 89")),
            build: cx.new(|cx| InputState::new(window, cx).placeholder("OTA info / build")),
            receiver: None,
            confirmed: false,
            reviewed: None,
            comparison: Value::Null,
            chooser: None,
            job: None,
            pending: None,
            hid: None,
            ports: Vec::new(),
            completion: None,
            error: None,
            advanced: false,
            details: false,
            step: None,
            legacy_tools: false,
            single_capture: false,
            workflow: Value::Null,
            capture_kind: None,
        }
    }
    pub(super) fn busy(&self) -> bool {
        self.job.is_some() || self.pending.is_some() || self.chooser.is_some() || self.device_busy()
    }
    pub(super) fn device_busy(&self) -> bool {
        self.hid.is_some() || !self.ports.is_empty()
    }
    pub(super) fn shutdown(&mut self) -> Option<Job<Completion>> {
        self.pending = None;
        if let Some(job) = &self.job {
            job.cancel();
        }
        self.job.take()
    }
}

impl Workbench {
    fn fleet_choose(&mut self, key: &'static str, directory: bool, cx: &mut Context<Self>) {
        if self.shutting_down || self.fleet.busy() {
            return;
        }
        let prompt = tr(&format!("fleet.path.{key}"));
        let dialog = cx.prompt_for_paths(PathPromptOptions {
            files: !directory,
            directories: directory,
            multiple: false,
            prompt: Some(prompt.into()),
        });
        self.fleet.chooser = Some(cx.spawn(async move |this, cx| {
            let result = dialog.await;
            let _ = this.update(cx, |this, cx| {
                this.fleet.chooser = None;
                if this.shutting_down {
                    return;
                }
                match result {
                    Ok(Ok(Some(paths))) => {
                        if let Some(path) = paths.into_iter().next() {
                            if key == "session" {
                                this.fleet.workflow = Value::Null;
                                this.fleet
                                    .paths
                                    .retain(|k, _| matches!(*k, "toolkit" | "python"));
                            }
                            this.fleet.paths.insert(key, path);
                            this.fleet.confirmed = false;
                            this.fleet.reviewed = None;
                            this.fleet.completion = None;
                            this.fleet.error = None;
                        }
                    }
                    Ok(Ok(None)) => {}
                    Ok(Err(e)) => this.fleet.error = Some(e.to_string()),
                    Err(e) => this.fleet.error = Some(e.to_string()),
                }
                cx.notify();
            });
        }));
    }

    fn fleet_release(&mut self) {
        // Called only before a worker starts or after Job::poll joins it.
        for lease in self.fleet.ports.drain(..) {
            if let Err(e) = self.manager.release(&lease) {
                self.fleet.error = Some(e);
            }
        }
        if let Some(lease) = self.fleet.hid.take()
            && let Err(e) = self.monitor.release(&lease)
        {
            self.fleet.error = Some(e);
        }
    }

    fn fleet_start(&mut self, action: &'static str, hardware: bool, cx: &mut Context<Self>) {
        if self.shutting_down || self.fleet.busy() {
            return;
        }
        self.fleet.error = None;
        self.fleet.completion = None;
        self.fleet.capture_kind = action.strip_prefix("workflow-record-");
        if matches!(
            action,
            "compare"
                | "plan"
                | "prepare"
                | "solve"
                | "snapshot"
                | "record"
                | "workflow-open"
                | "workflow-resume"
        ) {
            self.fleet.reviewed = None;
        }
        if matches!(
            action,
            "compare"
                | "prepare"
                | "solve"
                | "snapshot"
                | "record"
                | "workflow-open"
                | "workflow-resume"
                | "workflow-solve"
                | "init"
        ) || self.fleet.capture_kind.is_some()
        {
            self.fleet.comparison = Value::Null;
        }
        if action == "workflow-solve" || self.fleet.capture_kind.is_some() || action == "init" {
            self.fleet.reviewed = None;
            self.fleet
                .paths
                .retain(|k, _| !matches!(*k, "input" | "result" | "plan"));
            if let Some(workflow) = self.fleet.workflow.as_object_mut() {
                workflow.insert("artifacts".into(), json!({}));
                if action == "init" {
                    workflow.remove("preflight");
                }
                if let Some(kind) = self.fleet.capture_kind
                    && let Some(captures) =
                        workflow.get_mut("captures").and_then(Value::as_object_mut)
                {
                    captures.remove(kind);
                }
            }
        }
        let result = (|| {
            let toolkit = self.fleet.paths.get("toolkit").cloned().unwrap_or_default();
            let session = self
                .fleet
                .paths
                .get("session")
                .ok_or_else(|| tr("fleet.path.session"))?;
            if matches!(action, "plan" | "apply" | "rollback" | "reference-new")
                && !self.fleet.confirmed
            {
                return Err(tr("fleet.confirm"));
            }
            let mut payload = json!({"schema":"HELPER_FLEET1", "action":action, "session":session,
                "confirmed":self.fleet.confirmed, "guided":!self.fleet.advanced, "build":self.fleet.build.read(cx).value().trim()});
            if action == "plan" && !self.fleet.advanced {
                payload["selection_mode"] = json!("validated-parts-v1");
            }
            if matches!(action, "prepare" | "workflow-solve") {
                let entered = self.fleet.latitude.read(cx).value();
                let latitude: f64 = if entered.trim().is_empty() {
                    self.fleet.workflow["latitude"]
                        .as_f64()
                        .ok_or_else(|| tr("fleet.latitude"))?
                } else {
                    entered.trim().parse().map_err(|_| tr("fleet.latitude"))?
                };
                if !latitude.is_finite() || latitude.abs() >= 89.0 {
                    return Err(tr("fleet.latitude"));
                }
                payload["latitude"] = json!(latitude);
            }
            for key in [
                "recording",
                "snapshots",
                "clocks",
                "segments",
                "input",
                "result",
                "plan",
            ] {
                if let Some(path) = self.fleet.paths.get(key) {
                    payload[key] = json!(path);
                }
            }
            if matches!(action, "apply" | "rollback") {
                let (path, hash) = self
                    .fleet
                    .reviewed
                    .as_ref()
                    .ok_or_else(|| tr("fleet.review_required"))?;
                payload["plan"] = json!(path);
                payload["plan_sha256"] = json!(hash);
            }
            let request = Request {
                toolkit,
                python: self
                    .fleet
                    .paths
                    .get("python")
                    .cloned()
                    .unwrap_or_else(|| "python".into()),
                payload,
                hardware,
            };
            if !hardware {
                self.fleet.job = Some(Job::start(move |cancel, progress| {
                    service::run(request, cancel, progress)
                })?);
                return Ok(());
            }
            if self.inventory.serial_enumeration_failed
                || self.inventory_error.is_some()
                || self.capture.busy()
                || self.calibration.session.busy()
                || self.flash.busy()
                || self.joint.device_busy()
            {
                return Err(tr("fleet.busy"));
            }
            if self.fleet.receiver.is_none() && self.inventory.receivers.len() == 1 {
                self.fleet.receiver = self.inventory.receivers.first().cloned();
            }
            let receiver = self
                .fleet
                .receiver
                .clone()
                .ok_or_else(|| tr("fleet.receiver"))?;
            self.flash_check_receiver(&receiver.key)?;
            let observation = self
                .monitor
                .observation_for(&receiver)
                .ok_or_else(|| tr("fleet.identities"))?;
            let mut targets = BTreeMap::new();
            for tracker in &observation.trackers {
                let address = tracker
                    .address
                    .as_deref()
                    .and_then(crate::domain::normalize_address)
                    .filter(|a| a != "000000000000")
                    .ok_or_else(|| tr("fleet.identities"))?;
                if tracker.receiver_key != receiver.key
                    || targets.insert(tracker.slot, address).is_some()
                {
                    return Err(tr("fleet.identities"));
                }
            }
            if targets.keys().copied().collect::<Vec<_>>() != (0..10).collect::<Vec<_>>() {
                return Err(tr("fleet.identities"));
            }
            let serial = receiver
                .serial_number
                .as_ref()
                .filter(|s| !s.is_empty())
                .ok_or_else(|| tr("fleet.identities"))?;
            let mut request = request;
            request.payload["receiver_serial"] = json!(serial);
            request.payload["targets"] = json!(targets);
            // Freeze all console ports before reserve clears passive identity data.
            // The fleet operation excludes other device operations until it joins.
            let ports: Vec<_> = self
                .inventory
                .ports
                .iter()
                .filter(|p| console_port_allowed(p, &self.inventory))
                .cloned()
                .collect();
            if !ports.iter().any(|p| {
                p.serial_number.as_ref() == Some(serial)
                    && p.vid == Some(0x1209)
                    && p.pid == Some(0x7690)
            }) {
                return Err(tr("fleet.identities"));
            }
            for port in &ports {
                self.flash_check_port(&port.port_name)?;
            }
            self.fleet.hid = Some(self.monitor.reserve(&receiver)?);
            for port in ports {
                let generation = self.manager.session(&port.port_name).map(|s| s.generation);
                self.fleet
                    .ports
                    .push(self.manager.reserve(&port.port_name, generation)?);
            }
            self.fleet.pending = Some(request);
            Ok::<(), String>(())
        })();
        if let Err(e) = result {
            self.fleet_release();
            self.fleet.error = Some(e);
        }
        self.fleet.confirmed = false;
        cx.notify();
    }

    fn fleet_stop(&mut self, cx: &mut Context<Self>) {
        if let Some(job) = &self.fleet.job {
            job.cancel();
        }
        if self.fleet.pending.take().is_some() {
            self.fleet_release();
        }
        cx.notify();
    }

    pub(super) fn poll_fleet(&mut self) -> bool {
        let mut changed = false;
        if self.fleet.pending.is_some() {
            let ready = (|| {
                let mut ready = self
                    .monitor
                    .reservation_ready(self.fleet.hid.as_ref().expect("pending receiver lease"))?;
                for lease in &self.fleet.ports {
                    ready &= self.manager.reservation_ready(lease)?;
                }
                Ok::<bool, String>(ready)
            })();
            match ready {
                Ok(false) => {}
                Ok(true) => {
                    let request = self.fleet.pending.take().expect("pending operation");
                    match Job::start(move |cancel, progress| {
                        service::run(request, cancel, progress)
                    }) {
                        Ok(job) => self.fleet.job = Some(job),
                        Err(e) => {
                            self.fleet.error = Some(e);
                            self.fleet_release();
                        }
                    }
                    changed = true;
                }
                Err(e) => {
                    self.fleet.pending = None;
                    self.fleet.error = Some(e);
                    self.fleet_release();
                    changed = true;
                }
            }
        }
        if let Some(job) = &mut self.fleet.job {
            let previous = job.progress.as_ref().map(|p| p.message.clone());
            let outcome = job.poll();
            changed |= previous != job.progress.as_ref().map(|p| p.message.clone());
            if let Some(outcome) = outcome {
                self.fleet.job = None;
                self.fleet_release();
                match outcome {
                    Ok(done) if done.result["utility"] == true => {
                        self.fleet.completion = Some(done);
                    }
                    Ok(done) => {
                        self.fleet.step = None;
                        if done.result["comparison"].is_object() {
                            self.fleet.comparison = done.result["comparison"].clone();
                        }
                        if done.result["workflow"].is_object() {
                            self.fleet.workflow = done.result["workflow"].clone();
                        }
                        if let (Some(path), Some(hash)) = (
                            done.result["reviewed_plan"]["path"].as_str(),
                            done.result["reviewed_plan"]["sha256"].as_str(),
                        ) {
                            self.fleet.reviewed = Some((path.into(), hash.to_owned()));
                        } else if done.result["reused"] != true
                            && !matches!(done.result["action"].as_str(), Some("verify" | "audit"))
                        {
                            self.fleet.reviewed = None;
                        }
                        if let Some(path) = done.result["session"].as_str() {
                            self.fleet.paths.insert("session", path.into());
                        }
                        // New results use the backend's accepted artifact index;
                        // imported overrides cannot silently shadow the next stage.
                        if done.result["action"] != "init" || done.result["reused"] != true {
                            self.fleet.paths.retain(|k, _| {
                                matches!(
                                    *k,
                                    "toolkit" | "python" | "session" | "clocks" | "segments"
                                )
                            });
                        }
                        self.fleet.completion = Some(done);
                    }
                    Err(e) => self.fleet.error = Some(e),
                }
                changed = true;
            }
        }
        changed
            && matches!(
                self.active_page,
                FeaturePage::Arc8H1 | FeaturePage::RawCapture | FeaturePage::Temperature
            )
    }

    pub(super) fn fleet_panel(&self, cx: &mut Context<Self>) -> AnyElement {
        let state = &self.fleet;
        let busy = state.busy();
        let mut panel = div()
            .w_full()
            .min_w_0()
            .flex()
            .flex_col()
            .gap_2()
            .p_3()
            .rounded_md()
            .border_1()
            .border_color(cx.theme().border)
            .child(div().text_lg().font_weight(FontWeight::SEMIBOLD).child(tr(
                match self.active_page {
                    FeaturePage::RawCapture => "workbench.capture_title",
                    FeaturePage::Temperature => "workbench.temperature_title",
                    _ => "fleet.title",
                },
            )))
            .child(
                div()
                    .text_sm()
                    .whitespace_normal()
                    .child(tr(match self.active_page {
                        FeaturePage::RawCapture => "workbench.capture_help",
                        FeaturePage::Temperature => "workbench.temperature_help",
                        _ => "workbench.joint_help",
                    })),
            );
        if let Some(job) = &state.job {
            let p = job
                .progress
                .as_ref()
                .and_then(|p| serde_json::from_str::<Value>(&p.message).ok())
                .unwrap_or(Value::Null);
            panel = panel.child(
                div()
                    .font_weight(FontWeight::SEMIBOLD)
                    .whitespace_normal()
                    .child(progress_text(&p)),
            );
            if p["counts"].is_object() {
                panel = panel.child(div().text_sm().whitespace_normal().child(format!(
                    "{}: {}",
                    tr("fleet.counts"),
                    p["counts"]
                )));
            }
            if let Some(ages) = p["last_seen_age"].as_object() {
                let stalled = ages
                    .iter()
                    .filter_map(|(slot, age)| {
                        age.as_f64()
                            .filter(|age| *age > 1.0)
                            .map(|age| format!("#{slot}: {age:.1}s"))
                    })
                    .collect::<Vec<_>>();
                if !stalled.is_empty() {
                    panel = panel.child(
                        div()
                            .text_sm()
                            .whitespace_normal()
                            .text_color(cx.theme().warning)
                            .child(format!(
                                "{} {}",
                                tr("workbench.stalled"),
                                stalled.join(" · ")
                            )),
                    );
                }
            }
            if let Some(rows) = p["temperatures"].as_array() {
                for row in rows {
                    panel = panel.child(div().text_sm().child(format!(
                        "#{} · {} °C · {} {}",
                        row["slot"],
                        row["temperature_c"],
                        row["points"],
                        tr("workbench.points")
                    )));
                }
            }
            if let Some(faces) = p["faces"].as_object().filter(|_| p["ready"] != true) {
                let labels = faces
                    .iter()
                    .map(|(slot, face)| format!("#{slot} {}", face.as_str().unwrap_or("?")))
                    .collect::<Vec<_>>()
                    .join(" · ");
                panel = panel.child(div().text_sm().whitespace_normal().child(format!(
                    "{} {}",
                    tr("fleet.wizard.faces"),
                    labels
                )));
            }
        }
        if let Some(done) = state.completion.as_ref().filter(|d| {
            let action = d.result["action"].as_str().unwrap_or_default();
            match self.active_page {
                FeaturePage::RawCapture => action == "utility-record",
                FeaturePage::Temperature => action.starts_with("utility-temperature-"),
                _ => d.result["utility"] != true,
            }
        }) {
            panel = panel.child(
                div()
                    .text_sm()
                    .font_weight(FontWeight::SEMIBOLD)
                    .whitespace_normal()
                    .child(tr(completion_label(&done.result))),
            );
            if done.result["state"] == "SESSION_READY"
                && let Some(session) = done.result["session"].as_str()
            {
                let path = PathBuf::from(session).join("session.json");
                panel = panel.child(
                    Button::new("fleet-open-session")
                        .label(tr("fleet.open_session"))
                        .on_click(cx.listener(move |_, _, _, cx| cx.reveal_path(&path))),
                );
            }
            if done.result["devices"].is_array() || done.result["direction_checks"].is_array() {
                panel = panel.child(
                    Checkbox::new("fleet-result-details")
                        .label(tr("workbench.details"))
                        .checked(state.details)
                        .on_click(cx.listener(|this, value, _, cx| {
                            this.fleet.details = *value;
                            cx.notify();
                        })),
                );
            }
            if state.details
                && let Some(rows) = done.result["devices"].as_array()
            {
                for row in rows {
                    panel = panel.child(div().text_xs().whitespace_normal().child(format!(
                        "#{} · Earth RMS {} / {} °/s · g-sens RMS {} / {} °/s",
                        row["slot"],
                        row["earth_train_rms_dps"],
                        row["earth_holdout_rms_dps"],
                        row["gsens_train_rms_dps"],
                        row["gsens_holdout_rms_dps"]
                    )));
                }
            }
            if done.result["final_return"]["present"] == true {
                let returned = &done.result["final_return"];
                panel = panel.child(div().text_sm().whitespace_normal().child(format!(
                    "{} {:.3}° · {}",
                    tr("fleet.wizard.return_result"),
                    returned["orientation_closure_deg"].as_f64().unwrap_or(f64::NAN),
                    tr(if returned["passed"] == true {
                        "fleet.wizard.return_passed"
                    } else {
                        "fleet.wizard.return_review"
                    })
                )));
            }
            if state.details
                && let Some(rows) = done.result["direction_checks"].as_array()
            {
                panel = panel.child(
                    div()
                        .text_sm()
                        .whitespace_normal()
                        .child(tr("fleet.wizard.direction_result")),
                );
                for row in rows {
                    let difference = row["relative_difference"]
                        .as_array()
                        .map(|v| v.iter().filter_map(Value::as_f64).fold(0.0_f64, f64::max) * 100.0)
                        .unwrap_or_default();
                    panel = panel.child(div().text_xs().whitespace_normal().child(format!(
                        "#{} · Δ max {:.3}% · {}",
                        row["slot"],
                        difference,
                        tr(if row["passed"] == true {
                            "fleet.wizard.passed"
                        } else {
                            "fleet.wizard.direction_review"
                        })
                    )));
                    if state.advanced {
                        panel = panel.child(div().text_xs().whitespace_normal().child(format!(
                            "+ {} · − {}",
                            row["positive_scale_candidate"], row["negative_scale_candidate"]
                        )));
                    }
                }
            }
            if let Some(rows) = done.result["reviewed_plan"]["devices"].as_array() {
                panel = panel.child(
                    div()
                        .text_sm()
                        .whitespace_normal()
                        .child(tr("fleet.review_values")),
                );
                for row in rows {
                    panel = panel.child(div().text_xs().whitespace_normal().child(format!(
                        "#{} · S={} · M={} °/s/g · E={} °/s",
                        row["slot"],
                        row["values"]["scale"],
                        row["values"]["gsens_dps_per_g"],
                        row["values"]["earth_dps"]
                    )));
                }
            }
            let path = done.log.clone();
            panel = panel
                .child(
                    Button::new("fleet-log")
                        .label(tr("fleet.log"))
                        .on_click(cx.listener(move |_, _, _, cx| cx.reveal_path(&path))),
                )
                .child(
                    div().text_xs().whitespace_normal().child(
                        done.result["output"]
                            .as_str()
                            .or_else(|| done.result["session"].as_str())
                            .unwrap_or_default()
                            .to_owned(),
                    ),
                );
            if let Some(failures) = done.result["failures"].as_array() {
                for failure in failures {
                    panel = panel.child(
                        div()
                            .text_xs()
                            .whitespace_normal()
                            .child(failure.as_str().unwrap_or_default().to_owned()),
                    );
                }
            }
        }
        if let Some(error) = &state.error {
            panel = panel.child(
                div()
                    .text_sm()
                    .whitespace_normal()
                    .text_color(cx.theme().danger)
                    .child(error.clone()),
            );
        }
        if busy {
            panel = panel.child(
                Button::new("fleet-stop")
                    .label(tr("fleet.stop"))
                    .warning()
                    .disabled(state.job.is_none() && state.pending.is_none())
                    .on_click(cx.listener(|this, _, _, cx| this.fleet_stop(cx))),
            );
        }
        if state.job.is_some() || state.pending.is_some() {
            if let Some(kind) = state.capture_kind {
                panel = panel.child(div().text_sm().whitespace_normal().child(tr(
                    if kind == "poses" {
                        "fleet.wizard.poses_help"
                    } else {
                        "fleet.wizard.axis_help"
                    },
                )));
            }
            return panel
                .child(
                    div()
                        .text_xs()
                        .whitespace_normal()
                        .child(tr("fleet.stop_help")),
                )
                .into_any_element();
        }
        if matches!(
            self.active_page,
            FeaturePage::RawCapture | FeaturePage::Temperature
        ) {
            return panel
                .child(self.fleet_utility_controls(cx))
                .into_any_element();
        }
        panel = panel.child(self.fleet_guided_controls(cx)).child(
            Checkbox::new("fleet-advanced")
                .label(tr("fleet.wizard.advanced"))
                .checked(state.advanced)
                .disabled(busy)
                .on_click(cx.listener(|this, value, _, cx| {
                    this.fleet.advanced = *value;
                    cx.notify();
                })),
        );
        if !state.advanced {
            return panel.into_any_element();
        }
        for (key, directory) in [("toolkit", true), ("python", false), ("session", true)] {
            panel =
                panel
                    .child(
                        Button::new(format!("fleet-path-{key}"))
                            .label(tr(&format!("fleet.path.{key}")))
                            .disabled(busy)
                            .on_click(cx.listener(move |this, _, _, cx| {
                                this.fleet_choose(key, directory, cx)
                            })),
                    )
                    .child(
                        div().text_xs().whitespace_normal().child(
                            state.paths.get(key).map_or_else(
                                || tr("fleet.unselected"),
                                |p| p.display().to_string(),
                            ),
                        ),
                    );
        }
        panel = panel.child(div().text_sm().child(tr("fleet.receiver")));
        for (i, receiver) in self.inventory.receivers.iter().enumerate() {
            let receiver = receiver.clone();
            let selected = state.receiver.as_ref().is_some_and(|r| {
                r.key == receiver.key
                    && r.path == receiver.path
                    && r.serial_number == receiver.serial_number
            });
            panel = panel.child(
                Button::new(format!("fleet-receiver-{i}"))
                    .label(format!(
                        "{}{}",
                        if selected { "✓ " } else { "" },
                        receiver.label
                    ))
                    .disabled(busy)
                    .on_click(cx.listener(move |this, _, _, cx| {
                        this.fleet.receiver = Some(receiver.clone());
                        this.fleet.confirmed = false;
                        cx.notify();
                    })),
            );
        }
        panel = panel
            .child(div().text_sm().child(tr("fleet.build")))
            .child(Input::new(&state.build).small().disabled(busy))
            .child(div().text_sm().child(tr("fleet.latitude")))
            .child(Input::new(&state.latitude).small().disabled(busy));
        for (key, directory) in [
            ("recording", false),
            ("snapshots", true),
            ("clocks", false),
            ("segments", false),
            ("input", false),
            ("result", false),
            ("plan", false),
        ] {
            panel = panel.child(
                div()
                    .flex()
                    .flex_wrap()
                    .gap_2()
                    .child(
                        Button::new(format!("fleet-path-{key}"))
                            .label(tr(&format!("fleet.path.{key}")))
                            .disabled(busy)
                            .on_click(cx.listener(move |this, _, _, cx| {
                                this.fleet_choose(key, directory, cx)
                            })),
                    )
                    .child(
                        div().text_xs().whitespace_normal().child(
                            state
                                .paths
                                .get(key)
                                .map_or_else(|| tr("fleet.latest"), |p| p.display().to_string()),
                        ),
                    ),
            );
        }
        panel = panel.child(
            Checkbox::new("fleet-reviewed")
                .label(tr("fleet.confirm"))
                .checked(state.confirmed)
                .disabled(busy)
                .on_click(cx.listener(|this, value, _, cx| {
                    this.fleet.confirmed = *value;
                    cx.notify();
                })),
        );
        for (action, hardware) in [
            ("init", true),
            ("record", true),
            ("audit", false),
            ("snapshot", true),
            ("prepare", false),
            ("solve", false),
            ("plan", false),
            ("review", false),
            ("apply", true),
            ("verify", true),
            ("rollback", true),
        ] {
            panel = panel.child(
                div()
                    .flex()
                    .flex_col()
                    .gap_1()
                    .child(
                        Button::new(format!("fleet-{action}"))
                            .label(tr(&format!("fleet.action.{action}")))
                            .disabled(busy)
                            .on_click(cx.listener(move |this, _, _, cx| {
                                this.fleet_start(action, hardware, cx)
                            })),
                    )
                    .child(
                        div()
                            .text_xs()
                            .whitespace_normal()
                            .child(tr(&format!("fleet.help.{action}"))),
                    ),
            );
        }
        panel = panel
            .child(
                div()
                    .font_weight(FontWeight::SEMIBOLD)
                    .child(tr("fleet.archive")),
            )
            .child(
                div()
                    .text_xs()
                    .whitespace_normal()
                    .child(tr("fleet.archive_help")),
            );
        let mut actions = div().flex().flex_wrap().gap_2();
        for action in [
            "on",
            "off",
            "status",
            "storage",
            "reference-new",
            "reference-confirm",
            "temperature",
            "scale-x-start",
            "scale-x-stop",
            "scale-y-start",
            "scale-y-stop",
            "scale-z-start",
            "scale-z-stop",
            "cancel",
        ] {
            actions = actions.child(
                Button::new(format!("fleet-archive-{action}"))
                    .label(tr(&format!("fleet.batch.{}", action.replace('-', "_"))))
                    .disabled(busy)
                    .on_click(
                        cx.listener(move |this, _, _, cx| this.fleet_start(action, true, cx)),
                    ),
            );
        }
        panel = panel.child(actions);
        panel.into_any_element()
    }

    fn fleet_guided_controls(&self, cx: &mut Context<Self>) -> AnyElement {
        let state = &self.fleet;
        let idle = !state.busy();
        let initialized = state.workflow["preflight"].is_object();
        let complete = ["x", "y", "z", "poses"]
            .iter()
            .all(|k| state.workflow["captures"][*k]["accepted"] == true);
        let solved = state.workflow["artifacts"]["result"].is_string();
        let step = state
            .step
            .unwrap_or_else(|| recommended_step(&state.workflow));
        let mut tabs = div().flex().flex_wrap().gap_2();
        for (i, key) in ["setup", "axes", "poses", "solve", "install"]
            .into_iter()
            .enumerate()
        {
            tabs = tabs.child(
                Button::new(format!("workflow-step-{i}"))
                    .label(format!(
                        "{}{}",
                        if i == step { "▸ " } else { "" },
                        tr(&format!("fleet.wizard.{key}"))
                    ))
                    .disabled(!idle)
                    .on_click(cx.listener(move |this, _, _, cx| {
                        this.fleet.step = Some(i);
                        cx.notify();
                    })),
            );
        }
        let mut panel = div().flex().flex_col().gap_3().min_w_0().child(tabs);
        let key = [
            "setup_help",
            "axis_help",
            "poses_help",
            "solve_help",
            "install_help",
        ][step];
        panel = panel.child(
            div()
                .text_sm()
                .whitespace_normal()
                .child(tr(&format!("fleet.wizard.{key}"))),
        );
        match step {
            0 => {
                panel = panel.child(self.fleet_receiver_controls(cx));
                panel = panel.child(
                    div()
                        .flex()
                        .flex_wrap()
                        .gap_2()
                        .child(workflow_button("init", true, idle, cx))
                        .child(workflow_button("workflow-resume", false, idle, cx)),
                );
                if initialized {
                    panel = panel.child(
                        Button::new("wizard-new")
                            .label(tr("fleet.wizard.new_session"))
                            .disabled(!idle)
                            .on_click(cx.listener(|this, _, _, cx| {
                                this.fleet
                                    .paths
                                    .retain(|k, _| matches!(*k, "toolkit" | "python"));
                                this.fleet.paths.insert("session", default_session_parent());
                                this.fleet.workflow = Value::Null;
                                this.fleet.reviewed = None;
                                this.fleet.confirmed = false;
                                this.fleet_start("init", true, cx);
                            })),
                    );
                }
            }
            1 => {
                for (kind, action) in [
                    ("x", "workflow-record-x"),
                    ("y", "workflow-record-y"),
                    ("z", "workflow-record-z"),
                ] {
                    panel = panel.child(
                        div()
                            .flex()
                            .flex_wrap()
                            .gap_2()
                            .child(workflow_button(
                                action,
                                true,
                                idle && initialized && capture_allowed(&state.workflow, kind),
                                cx,
                            ))
                            .child(div().text_sm().child(tr(
                                if state.workflow["captures"][kind]["accepted"] == true {
                                    "fleet.wizard.recorded_short"
                                } else {
                                    "fleet.wizard.pending"
                                },
                            ))),
                    );
                }
            }
            2 => {
                panel = panel
                    .child(workflow_button(
                        "workflow-record-poses",
                        true,
                        idle && initialized && capture_allowed(&state.workflow, "poses"),
                        cx,
                    ))
                    .child(div().text_sm().child(tr(if complete {
                        "fleet.wizard.recorded_short"
                    } else {
                        "fleet.wizard.pending"
                    })));
            }
            3 => {
                panel = panel
                    .child(div().text_sm().child(tr("fleet.latitude")))
                    .child(Input::new(&state.latitude).small().disabled(!idle));
                if let Some(value) = state.workflow["latitude"].as_f64() {
                    panel = panel.child(
                        div()
                            .text_xs()
                            .child(format!("{} {value}?", tr("fleet.wizard.last_latitude"))),
                    );
                }
                panel = panel.child(workflow_button(
                    "workflow-solve",
                    true,
                    idle && complete,
                    cx,
                ));
            }
            _ => {
                panel = panel
                    .child(
                        div()
                            .text_sm()
                            .whitespace_normal()
                            .child(tr("fleet.comparison.help")),
                    )
                    .child(workflow_button("compare", false, idle && solved, cx));
                for line in comparison_lines(&state.comparison) {
                    panel = panel.child(div().text_sm().whitespace_normal().child(line));
                }
                if matches!(
                    state.workflow["installation"]["state"].as_str(),
                    Some("APPLY_READBACK_VERIFIED" | "VERIFY_READBACK_VERIFIED")
                ) {
                    panel = panel.child(
                        div()
                            .text_sm()
                            .whitespace_normal()
                            .child(tr("fleet.installed")),
                    );
                }
                panel = panel
                    .child(
                        Checkbox::new("wizard-confirm")
                            .label(tr(if state.reviewed.is_some() {
                                "fleet.wizard.confirm_write"
                            } else {
                                "fleet.wizard.confirm"
                            }))
                            .checked(state.confirmed)
                            .disabled(!idle || !solved)
                            .on_click(cx.listener(|this, value, _, cx| {
                                this.fleet.confirmed = *value;
                                cx.notify();
                            })),
                    )
                    .child(
                        div()
                            .flex()
                            .flex_wrap()
                            .gap_2()
                            .child(workflow_button(
                                "plan",
                                false,
                                idle && solved
                                    && state.comparison["changed_devices"]
                                        .as_u64()
                                        .is_some_and(|n| n > 0)
                                    && state.confirmed,
                                cx,
                            ))
                            .child(workflow_button(
                                "apply",
                                true,
                                idle && state.reviewed.is_some() && state.confirmed,
                                cx,
                            ))
                            .child(workflow_button(
                                "verify",
                                true,
                                idle && state.reviewed.is_some(),
                                cx,
                            )),
                    );
            }
        }
        if step > 0 && !initialized {
            panel = panel.child(
                div()
                    .text_sm()
                    .whitespace_normal()
                    .child(tr("workbench.prepare_first")),
            );
        }
        if let Some(path) = state.paths.get("session") {
            let path = path.clone();
            panel = panel.child(
                Button::new("wizard-folder")
                    .label(tr("fleet.open_session"))
                    .on_click(cx.listener(move |_, _, _, cx| cx.reveal_path(&path))),
            );
        }
        panel.into_any_element()
    }

    fn fleet_receiver_controls(&self, cx: &mut Context<Self>) -> AnyElement {
        let mut panel = div()
            .flex()
            .flex_col()
            .gap_2()
            .child(div().text_sm().child(tr("fleet.receiver")));
        if self.inventory.receivers.is_empty() {
            panel = panel.child(
                div()
                    .text_sm()
                    .whitespace_normal()
                    .child(tr("workbench.no_receiver")),
            );
        }
        for (i, receiver) in self.inventory.receivers.iter().enumerate() {
            let receiver = receiver.clone();
            let selected = self
                .fleet
                .receiver
                .as_ref()
                .is_some_and(|r| r.key == receiver.key)
                || (self.fleet.receiver.is_none() && self.inventory.receivers.len() == 1);
            panel = panel.child(
                Button::new(format!("workspace-receiver-{i}"))
                    .label(format!(
                        "{}{}",
                        if selected { "✓ " } else { "" },
                        receiver.label
                    ))
                    .disabled(self.fleet.busy())
                    .on_click(cx.listener(move |this, _, _, cx| {
                        this.fleet.receiver = Some(receiver.clone());
                        this.fleet.reviewed = None;
                        this.fleet.confirmed = false;
                        cx.notify();
                    })),
            );
        }
        panel.into_any_element()
    }

    fn fleet_utility_controls(&self, cx: &mut Context<Self>) -> AnyElement {
        let temperature = self.active_page == FeaturePage::Temperature;
        let mut panel = div()
            .flex()
            .flex_col()
            .gap_3()
            .min_w_0()
            .child(self.fleet_receiver_controls(cx));
        let actions: &[(&str, &str)] = if temperature {
            &[
                ("utility-temperature-status", "workbench.temperature_status"),
                ("utility-temperature-start", "workbench.temperature_start"),
                ("utility-temperature-stop", "workbench.temperature_stop"),
                (
                    "utility-temperature-coverage",
                    "workbench.temperature_coverage",
                ),
            ]
        } else {
            &[("utility-record", "workbench.capture_start")]
        };
        for &(action, label) in actions {
            panel = panel.child(
                Button::new(format!("tool-{action}"))
                    .label(tr(label))
                    .disabled(self.fleet.busy())
                    .on_click(
                        cx.listener(move |this, _, _, cx| this.fleet_start(action, true, cx)),
                    ),
            );
        }
        if temperature {
            panel = panel.child(
                Button::new("temperature-usb-plot")
                    .label(tr("workbench.temperature_plot"))
                    .on_click(cx.listener(|this, _, _, cx| this.open_temperature_window(cx))),
            );
        }
        if let Some(done) = &self.fleet.completion {
            if temperature && let Some(rows) = done.result["temperatures"].as_array() {
                for row in rows {
                    panel = panel.child(div().text_sm().whitespace_normal().child(format!(
                        "#{} · {} °C · {} {}",
                        row["slot"],
                        row["temperature_c"],
                        row["points"],
                        tr("workbench.points")
                    )));
                }
            }
            if temperature && let Some(rows) = done.result["coverage"]["devices"].as_array() {
                for row in rows {
                    panel = panel.child(div().text_sm().whitespace_normal().child(format!(
                        "#{} · {}/140 · {} {} °C",
                        row["slot"],
                        row["points"],
                        tr("workbench.missing"),
                        row["missing_bucket_intervals_c"]
                    )));
                }
            }
            if done.result["utility"] == true
                && let Some(path) = done.result["output"].as_str()
            {
                let path = PathBuf::from(path);
                panel = panel.child(
                    Button::new("tool-output")
                        .label(tr("workbench.open_output"))
                        .on_click(cx.listener(move |_, _, _, cx| cx.reveal_path(&path))),
                );
            }
        }
        panel.into_any_element()
    }
}

fn recommended_step(workflow: &Value) -> usize {
    if !workflow["preflight"].is_object() {
        0
    } else if ["x", "y", "z"]
        .iter()
        .any(|k| workflow["captures"][*k]["accepted"] != true)
    {
        1
    } else if workflow["captures"]["poses"]["accepted"] != true {
        2
    } else if !workflow["artifacts"]["result"].is_string() {
        3
    } else {
        4
    }
}
