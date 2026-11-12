use super::{FeaturePage, Workbench};
use crate::{
    domain::{SerialEndpoint, SerialPortKind},
    features::joint_calibration::{
        device,
        service::{self, Candidate, Completion, Snapshot},
    },
    i18n::tr,
    sessions::SerialReservation,
    update::{Job, Progress},
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
use std::path::PathBuf;

pub(super) struct JointUiState {
    latitude: Entity<InputState>,
    confirmed: bool,
    install_confirmed: bool,
    recording: Option<PathBuf>,
    snapshot: Option<Snapshot>,
    candidate: Option<Candidate>,
    endpoint: Option<SerialEndpoint>,
    chooser: Option<Task<()>>,
    pub(super) job: Option<Job<Completion>>,
    lease: Option<SerialReservation>,
    pending: Option<(SerialEndpoint, Option<Candidate>)>,
    output: Option<PathBuf>,
    notice: Option<String>,
    error: Option<String>,
}
impl JointUiState {
    pub(super) fn new(window: &mut Window, cx: &mut Context<Workbench>) -> Self {
        Self {
            latitude: cx.new(|cx| InputState::new(window, cx).placeholder("-85 .. 85")),
            confirmed: false,
            install_confirmed: false,
            recording: None,
            snapshot: None,
            candidate: None,
            endpoint: None,
            chooser: None,
            job: None,
            lease: None,
            pending: None,
            output: None,
            notice: None,
            error: None,
        }
    }
    fn busy(&self) -> bool {
        self.job.is_some() || self.pending.is_some() || self.chooser.is_some()
    }
    pub(super) fn device_busy(&self) -> bool {
        self.lease.is_some()
    }
    pub(super) fn shutdown(&mut self) -> Option<Job<Completion>> {
        self.pending = None;
        if let Some(job) = &self.job {
            job.cancel();
        }
        self.job.take()
    }
    fn invalidate(&mut self) {
        self.candidate = None;
        self.install_confirmed = false;
        self.notice = None;
        self.error = None;
    }
}
impl Workbench {
    pub(super) fn joint_check_device_idle(&self) -> Result<(), String> {
        if self.joint.device_busy() || self.fleet.device_busy() {
            Err(tr("joint.busy"))
        } else {
            Ok(())
        }
    }
    fn joint_choose(&mut self, snapshot: bool, cx: &mut Context<Self>) {
        if self.shutting_down || self.joint.busy() {
            return;
        }
        let dialog = cx.prompt_for_paths(PathPromptOptions {
            files: true,
            directories: false,
            multiple: false,
            prompt: Some(
                tr(if snapshot {
                    "joint.choose_snapshot"
                } else {
                    "joint.choose_recording"
                })
                .into(),
            ),
        });
        self.joint.chooser = Some(cx.spawn(async move |this, cx| {
            let chosen = dialog.await;
            let _ = this.update(cx, |this, cx| {
                this.joint.chooser = None;
                if this.shutting_down {
                    return;
                }
                match chosen {
                    Ok(Ok(Some(paths))) => {
                        if let Some(path) = paths.into_iter().next() {
                            this.joint.invalidate();
                            if snapshot {
                                this.joint.snapshot = None;
                                match Job::start(move |_, _| {
                                    service::import_snapshot(&path)
                                        .map(|s| Completion::Snapshot(s, path))
                                }) {
                                    Ok(job) => this.joint.job = Some(job),
                                    Err(e) => this.joint.error = Some(e),
                                }
                            } else {
                                this.joint.recording = Some(path);
                            }
                        }
                    }
                    Ok(Ok(None)) => {}
                    Ok(Err(e)) => this.joint.error = Some(e.to_string()),
                    Err(e) => this.joint.error = Some(e.to_string()),
                }
                cx.notify();
            });
        }));
        cx.notify();
    }
    fn joint_solve(&mut self, cx: &mut Context<Self>) {
        if self.shutting_down || self.joint.busy() {
            return;
        }
        self.joint.invalidate();
        let result = (|| {
            let path = self
                .joint
                .recording
                .clone()
                .ok_or_else(|| tr("joint.choose_recording"))?;
            let snapshot = self
                .joint
                .snapshot
                .clone()
                .ok_or_else(|| tr("joint.need_snapshot"))?;
            let latitude = self
                .joint
                .latitude
                .read(cx)
                .value()
                .trim()
                .parse::<f64>()
                .map_err(|_| tr("joint.latitude_error"))?;
            let confirmed = self.joint.confirmed;
            Job::start(move |cancel, progress| {
                progress(Progress::stage("joint.progress.solving"));
                service::solve(path, snapshot, latitude, confirmed, cancel)
            })
        })();
        match result {
            Ok(job) => self.joint.job = Some(job),
            Err(e) => self.joint.error = Some(e),
        }
        cx.notify();
    }
    fn joint_device(&mut self, install: bool, cx: &mut Context<Self>) {
        if self.shutting_down || self.joint.busy() {
            return;
        }
        self.joint.error = None;
        self.joint.notice = None;
        let result = (|| {
            if self.capture.busy() || self.calibration.session.busy() || self.flash.busy() {
                return Err(tr("joint.other_busy"));
            }
            let endpoint = self
                .joint
                .endpoint
                .clone()
                .ok_or_else(|| tr("joint.select_device"))?;
            self.console_check_port(&endpoint.port_name)?;
            self.flash_check_port(&endpoint.port_name)?;
            for receiver in &self.inventory.receivers {
                self.flash_check_receiver(&receiver.key)?;
            }
            // Freeze a positive USB endpoint before reserve drains the console.
            service::check(
                endpoint
                    .serial_number
                    .as_deref()
                    .is_some_and(|s| !s.is_empty()),
                "Missing USB serial",
            )?;
            let candidate = if install {
                service::check(self.joint.install_confirmed, &tr("joint.confirm_install"))?;
                let candidate = self
                    .joint
                    .candidate
                    .clone()
                    .ok_or_else(|| tr("joint.solve"))?;
                service::check(
                    candidate.solution.installable,
                    "Candidate holdout gates failed",
                )?;
                service::check(
                    endpoint.serial_number.as_deref() == Some(&candidate.snapshot.device.serial)
                        && endpoint.vid == Some(candidate.snapshot.device.vid)
                        && endpoint.pid == Some(candidate.snapshot.device.pid),
                    "Selected USB device differs from candidate",
                )?;
                Some(candidate)
            } else {
                None
            };
            let generation = self
                .manager
                .session(&endpoint.port_name)
                .map(|s| s.generation);
            let lease = self.manager.reserve(&endpoint.port_name, generation)?;
            self.joint.lease = Some(lease);
            self.joint.pending = Some((endpoint, candidate));
            if !install {
                self.joint.snapshot = None;
                self.joint.invalidate();
            }
            Ok::<(), String>(())
        })();
        if let Err(e) = result {
            self.joint.error = Some(e);
        }
        cx.notify();
    }
    fn joint_cancel(&mut self) {
        if let Some(job) = &self.joint.job {
            job.cancel();
        }
        if self.joint.pending.take().is_some()
            && let Some(lease) = self.joint.lease.take()
            && let Err(e) = self.manager.release(&lease)
        {
            self.joint.error = Some(e);
        }
    }
    pub(super) fn poll_joint(&mut self) -> bool {
        let mut changed = false;
        if self.joint.pending.is_some() {
            let ready = self
                .manager
                .reservation_ready(self.joint.lease.as_ref().expect("pending lease"));
            match ready {
                Ok(false) => {}
                Ok(true) => {
                    let (endpoint, candidate) =
                        self.joint.pending.take().expect("pending operation");
                    let result = self
                        .manager
                        .reserved_console_source(self.joint.lease.as_ref().unwrap())
                        .and_then(|source| {
                            Job::start(move |cancel, progress| {
                                device::run(source, endpoint, candidate, cancel, progress)
                            })
                        });
                    match result {
                        Ok(job) => self.joint.job = Some(job),
                        Err(e) => {
                            self.joint.error = Some(e);
                            if let Some(lease) = self.joint.lease.take() {
                                let _ = self.manager.release(&lease);
                            }
                        }
                    }
                    changed = true;
                }
                Err(e) => {
                    self.joint.pending = None;
                    self.joint.error = Some(e);
                    if let Some(lease) = self.joint.lease.take() {
                        let _ = self.manager.release(&lease);
                    }
                    changed = true;
                }
            }
        }
        if let Some(job) = &mut self.joint.job {
            let old = job.progress.as_ref().map(|p| p.message.clone());
            let result = job.poll();
            changed |= old != job.progress.as_ref().map(|p| p.message.clone());
            if let Some(result) = result {
                // poll joined the actual worker before releasing exclusion.
                self.joint.job = None;
                if let Some(lease) = self.joint.lease.take()
                    && let Err(e) = self.manager.release(&lease)
                {
                    self.joint.error = Some(e);
                }
                match result {
                    Ok(Completion::Snapshot(snapshot, path)) => {
                        self.joint.snapshot = Some(snapshot);
                        self.joint.output = Some(path);
                        self.joint.notice = Some(tr("joint.inputs_ready"));
                    }
                    Ok(Completion::Candidate(candidate, path)) => {
                        self.joint.notice = Some(tr(if candidate.solution.installable {
                            "joint.qualified"
                        } else {
                            "joint.rejected"
                        }));
                        self.joint.candidate = Some(*candidate);
                        self.joint.output = Some(path);
                    }
                    Ok(Completion::Installed(path)) => {
                        self.joint.notice = Some(tr("joint.installed"));
                        self.joint.output = Some(path);
                        self.joint.install_confirmed = false;
                    }
                    Err(e) => self.joint.error = Some(e),
                }
                changed = true;
            }
        }
        changed && self.active_page == FeaturePage::Arc8H1
    }
    pub(super) fn joint_panel(&self, cx: &mut Context<Self>) -> AnyElement {
        let state = &self.joint;
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
            .child(
                div()
                    .text_lg()
                    .font_weight(FontWeight::SEMIBOLD)
                    .child(tr("joint.title")),
            )
            .child(div().text_sm().whitespace_normal().child(tr("joint.intro")))
            .child(
                div()
                    .text_xs()
                    .whitespace_normal()
                    .child(tr("joint.preparation")),
            )
            .child(
                Button::new("joint-recording")
                    .label(tr("joint.choose_recording"))
                    .disabled(busy)
                    .on_click(cx.listener(|this, _, _, cx| this.joint_choose(false, cx))),
            )
            .child(
                div().text_xs().whitespace_normal().child(
                    state
                        .recording
                        .as_ref()
                        .map_or_else(|| tr("joint.no_file"), |p| p.display().to_string()),
                ),
            )
            .child(
                div()
                    .font_weight(FontWeight::SEMIBOLD)
                    .child(tr("joint.select_device")),
            );
        for (index, endpoint) in self
            .inventory
            .ports
            .iter()
            .filter(|e| {
                e.kind == SerialPortKind::Usb
                    && crate::domain::console_port_allowed(e, &self.inventory)
                    && e.serial_number.is_some()
            })
            .enumerate()
        {
            let endpoint = endpoint.clone();
            let selected = state
                .endpoint
                .as_ref()
                .is_some_and(|e| e.port_name == endpoint.port_name);
            panel = panel.child(
                Button::new(format!("joint-device-{index}"))
                    .label(format!(
                        "{}{} · {}",
                        if selected { "✓ " } else { "" },
                        endpoint.port_name,
                        endpoint.serial_number.as_deref().unwrap_or_default()
                    ))
                    .disabled(busy)
                    .on_click(cx.listener(move |this, _, _, cx| {
                        this.joint.endpoint = Some(endpoint.clone());
                        this.joint.install_confirmed = false;
                        cx.notify();
                    })),
            );
        }
        panel = panel.child(
            div()
                .flex()
                .flex_wrap()
                .gap_2()
                .child(
                    Button::new("joint-read")
                        .label(tr("joint.read_inputs"))
                        .disabled(busy || state.endpoint.is_none())
                        .on_click(cx.listener(|this, _, _, cx| this.joint_device(false, cx))),
                )
                .child(
                    Button::new("joint-import")
                        .label(tr("joint.choose_snapshot"))
                        .disabled(busy)
                        .on_click(cx.listener(|this, _, _, cx| this.joint_choose(true, cx))),
                ),
        );
        if let Some(snapshot) = &state.snapshot {
            panel = panel.child(div().text_xs().whitespace_normal().child(format!(
                "USB {} · RF {} · {}",
                snapshot.device.serial, snapshot.device.rf_address, snapshot.device.build
            )));
        }
        panel = panel
            .child(div().text_sm().child(tr("joint.latitude")))
            .child(Input::new(&state.latitude).small().disabled(busy))
            .child(
                Checkbox::new("joint-reference")
                    .label(tr("joint.reference"))
                    .checked(state.confirmed)
                    .disabled(busy)
                    .on_click(cx.listener(|this, value, _, cx| {
                        this.joint.confirmed = *value;
                        this.joint.invalidate();
                        cx.notify();
                    })),
            )
            .child(
                Button::new("joint-solve")
                    .label(tr("joint.solve"))
                    .primary()
                    .disabled(
                        busy || state.recording.is_none()
                            || state.snapshot.is_none()
                            || !state.confirmed,
                    )
                    .on_click(cx.listener(|this, _, _, cx| this.joint_solve(cx))),
            );
        if let Some(candidate) = &state.candidate {
            let s = &candidate.solution;
            panel=panel.child(div().text_sm().whitespace_normal().child(format!("{}: {:.6}° · {}: {} · {}: {}\nEarth RMS {:.9} / {:.9} ≤ 0.001 dps\nGSens RMS {:.9} / {:.9} ≤ 0.006 dps\nScale {:?}\nEarth {:?}\nGSens {:?}",tr("joint.latitude"),candidate.inputs.latitude_deg,tr("joint.samples"),s.audit.unique_samples,tr("joint.poses"),s.poses.len(),s.earth_fit_rms_dps,s.earth_holdout_rms_dps,s.gsens_fit_rms_dps,s.gsens_holdout_rms_dps,s.coefficients.scale,s.coefficients.earth_dps,s.coefficients.gsens_dps_per_g)))
                .child(Checkbox::new("joint-install-confirm").label(tr("joint.confirm_install")).checked(state.install_confirmed).disabled(busy||!s.installable).on_click(cx.listener(|this,value,_,cx|{this.joint.install_confirmed = *value;cx.notify();})))
                .child(Button::new("joint-install").label(tr("joint.install")).disabled(busy||!s.installable||!state.install_confirmed||state.endpoint.is_none()).on_click(cx.listener(|this,_,_,cx|this.joint_device(true,cx))));
        }
        if let Some(job) = &state.job {
            panel = panel.child(
                div().text_sm().child(
                    job.progress
                        .as_ref()
                        .map_or_else(|| tr("joint.progress.waiting"), |p| tr(&p.message)),
                ),
            );
        }
        if state.job.is_some() || state.pending.is_some() {
            panel = panel.child(
                Button::new("joint-cancel")
                    .label(tr("joint.cancel"))
                    .on_click(cx.listener(|this, _, _, cx| {
                        this.joint_cancel();
                        cx.notify();
                    })),
            );
        }
        if let Some(text) = &state.notice {
            panel = panel.child(div().text_sm().whitespace_normal().child(text.clone()));
        }
        if let Some(text) = &state.error {
            panel = panel.child(
                div()
                    .text_sm()
                    .whitespace_normal()
                    .text_color(cx.theme().danger)
                    .child(text.clone()),
            );
        }
        if let Some(path) = &state.output {
            panel = panel.child(
                div()
                    .text_xs()
                    .whitespace_normal()
                    .child(path.display().to_string()),
            );
        }
        panel
            .child(
                div()
                    .text_xs()
                    .whitespace_normal()
                    .child(tr("joint.limits")),
            )
            .into_any_element()
    }
}
