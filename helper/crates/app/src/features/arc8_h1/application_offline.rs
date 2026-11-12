//! ARC8 desktop offline calibration UI. Reuses the shipped H2/H3 Python+C
//! implementation through an explicit, bounded background job. No device writes.
use super::{FeaturePage, Workbench};
use crate::features::arc8_h1::offline::{self, Completion, Request};
use gpui_kit::component::{
    ActiveTheme, Disableable, Sizable,
    button::Button,
    input::{Input, InputState},
};
use gpui_kit::{
    AnyElement, Context, Entity, FontWeight, PathPromptOptions, Task, Window, div, prelude::*,
};
use std::{
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    thread,
};

#[derive(Clone, Copy)]
enum SourceFile {
    Log,
    Capture,
    Candidate,
}

pub(super) struct OfflineUiState {
    uid: Entity<InputState>,
    receiver_key: Entity<InputState>,
    note: Entity<InputState>,
    log: Option<PathBuf>,
    capture: Option<PathBuf>,
    candidate: Option<PathBuf>,
    chooser: Option<Task<()>>,
    receiver: Option<mpsc::Receiver<Completion>>,
    worker: Option<thread::JoinHandle<()>>,
    cancel: Arc<AtomicBool>,
    running: Option<&'static str>,
    notice: Option<String>,
    error: Option<String>,
}

impl OfflineUiState {
    pub(super) fn new(window: &mut Window, cx: &mut Context<Workbench>) -> Self {
        Self {
            uid: cx
                .new(|cx| InputState::new(window, cx).placeholder("真实 Tracker UID（非槽位号）")),
            receiver_key: cx
                .new(|cx| InputState::new(window, cx).placeholder("接收器唯一标识 / 配对上下文")),
            note: cx
                .new(|cx| InputState::new(window, cx).placeholder("本次动作、参考夹具及数据来源")),
            log: None,
            capture: None,
            candidate: None,
            chooser: None,
            receiver: None,
            worker: None,
            cancel: Arc::new(AtomicBool::new(false)),
            running: None,
            notice: None,
            error: None,
        }
    }

    fn busy(&self) -> bool {
        self.receiver.is_some() || self.chooser.is_some()
    }

    fn start(&mut self, request: Request) {
        if self.busy() {
            return;
        }
        self.error = None;
        self.notice = None;
        let (tx, rx) = mpsc::sync_channel(1);
        self.running = Some(match &request {
            Request::Parse { .. } => "正在校验四组归档与双层 CRC…",
            Request::Solve { .. } => "正在调用原 ARC7 C 求解器…",
            Request::Verify { .. } => "正在校验候选包 SHA-256…",
        });
        self.receiver = Some(rx);
        self.cancel.store(false, Ordering::Release);
        let cancel = self.cancel.clone();
        self.worker = Some(thread::spawn(move || {
            let _ = tx.send(offline::run(request, &cancel));
        }));
    }

    pub(super) fn shutdown(&mut self) -> Option<thread::JoinHandle<()>> {
        self.cancel.store(true, Ordering::Release);
        self.receiver = None;
        self.worker.take()
    }

    fn poll(&mut self) -> bool {
        let message = self.receiver.as_ref().map(mpsc::Receiver::try_recv);
        let completion = match message {
            Some(Ok(completion)) => completion,
            Some(Err(mpsc::TryRecvError::Disconnected)) => {
                self.receiver = None;
                self.running = None;
                if let Some(worker) = self.worker.take() {
                    let _ = worker.join();
                }
                self.error = Some("后台任务异常退出，没有输出结果".into());
                return true;
            }
            _ => return false,
        };
        self.receiver = None;
        self.running = None;
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
        match completion.result {
            Ok(message) => {
                match completion.step {
                    "H3 归档解析" => self.capture = completion.output,
                    "H2 联合求解" => self.candidate = completion.output,
                    _ => {}
                }
                self.notice = Some(message);
                self.error = None;
            }
            Err(error) => {
                self.error = Some(error);
                self.notice = None;
            }
        }
        true
    }
}

impl Workbench {
    pub(in crate::application) fn poll_arc8_offline(&mut self) -> bool {
        let changed = self.arc8_offline.poll();
        changed && self.active_page == FeaturePage::Arc8H1
    }

    fn arc8_choose_source(&mut self, which: SourceFile, cx: &mut Context<Self>) {
        if self.shutting_down || self.arc8_offline.busy() {
            return;
        }
        let dialog = cx.prompt_for_paths(PathPromptOptions {
            files: true,
            directories: false,
            multiple: false,
            prompt: Some("选择 ARC8 文件（不会向设备发送命令）".into()),
        });
        self.arc8_offline.chooser = Some(cx.spawn(async move |this, cx| {
            let chosen = dialog.await;
            let _ = this.update(cx, |this, cx| {
                this.arc8_offline.chooser = None;
                if this.shutting_down {
                    return;
                }
                match chosen {
                    Ok(Ok(Some(paths))) => {
                        if let Some(path) = paths.into_iter().next() {
                            match which {
                                SourceFile::Log => {
                                    this.arc8_offline.log = Some(path);
                                    this.arc8_offline.capture = None;
                                    this.arc8_offline.candidate = None;
                                }
                                SourceFile::Capture => {
                                    this.arc8_offline.capture = Some(path);
                                    this.arc8_offline.candidate = None;
                                }
                                SourceFile::Candidate => this.arc8_offline.candidate = Some(path),
                            }
                            this.arc8_offline.error = None;
                            this.arc8_offline.notice = None;
                        }
                    }
                    Ok(Ok(None)) => {}
                    Ok(Err(error)) => this.arc8_offline.error = Some(error.to_string()),
                    Err(error) => this.arc8_offline.error = Some(error.to_string()),
                }
                cx.notify();
            });
        }));
        cx.notify();
    }

    fn arc8_start_parse(&mut self, cx: &mut Context<Self>) {
        if self.shutting_down || self.arc8_offline.busy() {
            return;
        }
        let Some(log) = self.arc8_offline.log.clone() else {
            return;
        };
        let uid = self.arc8_offline.uid.read(cx).value().to_string();
        let receiver_key = self.arc8_offline.receiver_key.read(cx).value().to_string();
        let note = self.arc8_offline.note.read(cx).value().to_string();
        let directory = log
            .parent()
            .unwrap_or(std::path::Path::new("."))
            .to_path_buf();
        let dialog = cx.prompt_for_new_path(&directory, Some("arc8-archive-verified.json"));
        self.arc8_offline.chooser = Some(cx.spawn(async move |this, cx| {
            let chosen = dialog.await;
            let _ = this.update(cx, |this, cx| {
                this.arc8_offline.chooser = None;
                if this.shutting_down {
                    return;
                }
                match chosen {
                    Ok(Ok(Some(output))) => this.arc8_offline.start(Request::Parse {
                        log,
                        output,
                        tracker_uid: uid,
                        receiver_key,
                        note,
                    }),
                    Ok(Err(error)) => this.arc8_offline.error = Some(error.to_string()),
                    Err(error) => this.arc8_offline.error = Some(error.to_string()),
                    _ => {}
                }
                cx.notify();
            });
        }));
        cx.notify();
    }

    fn arc8_start_solve(&mut self, cx: &mut Context<Self>) {
        if self.shutting_down || self.arc8_offline.busy() {
            return;
        }
        let Some(capture) = self.arc8_offline.capture.clone() else {
            return;
        };
        let directory = capture
            .parent()
            .unwrap_or(std::path::Path::new("."))
            .to_path_buf();
        let dialog = cx.prompt_for_new_path(&directory, Some("arc8-cal-candidate.json"));
        self.arc8_offline.chooser = Some(cx.spawn(async move |this, cx| {
            let chosen = dialog.await;
            let _ = this.update(cx, |this, cx| {
                this.arc8_offline.chooser = None;
                if this.shutting_down {
                    return;
                }
                match chosen {
                    Ok(Ok(Some(output))) => {
                        this.arc8_offline.start(Request::Solve { capture, output })
                    }
                    Ok(Err(error)) => this.arc8_offline.error = Some(error.to_string()),
                    Err(error) => this.arc8_offline.error = Some(error.to_string()),
                    _ => {}
                }
                cx.notify();
            });
        }));
        cx.notify();
    }

    fn arc8_start_verify(&mut self, cx: &mut Context<Self>) {
        if self.shutting_down || self.arc8_offline.busy() {
            return;
        }
        if let Some(bundle) = self.arc8_offline.candidate.clone() {
            self.arc8_offline.start(Request::Verify { bundle });
            cx.notify();
        }
    }

    pub(super) fn arc8_offline_panel(&self, cx: &mut Context<Self>) -> AnyElement {
        let offline = &self.arc8_offline;
        let busy = offline.busy() || self.shutting_down;
        let path_display = |path: &Option<PathBuf>| -> String {
            path.as_ref().map_or("尚未选择".to_owned(), |value| {
                value.display().to_string()
            })
        };
        let mut box_ = div().w_full().min_w_0().flex().flex_col().gap_3().p_3()
            .rounded_md().border_1().border_color(cx.theme().border)
            .child(div().text_lg().font_weight(FontWeight::SEMIBOLD).child("ARC8 H2/H3 · 电脑端联合校准工作台"))
            .child(div().text_sm().whitespace_normal().child(
                "已连接真实离线工具：ARC8X1 四组归档解码及 CRC 校验 → ARC7 原 C 求解器 → 候选包 SHA-256 校验。所有操作只处理电脑文件，绝不写 Tracker NVS。"
            ))
            .child(div().text_sm().text_color(cx.theme().muted_foreground).child(
                "需要运行时 Python 3；求解还需要 GCC/Clang。EXE 包含原始 C 求解器和 Python 工具，不依赖源码目录。可通过 ARC8_PYTHON 指定解释器。"
            ))
            .child(div().flex().flex_wrap().gap_2()
                .child(Button::new("arc8-ui-open-raw").small().label("打开原始数据采集")
                    .on_click(cx.listener(|this, _, window, cx| {
                        if this.set_feature_page(FeaturePage::RawCapture, window, cx) {
                            this.page_focus.focus(window, cx);
                            this.refresh_capture_sources(cx);
                            cx.notify();
                        }
                    })))
                .child(Button::new("arc8-ui-open-replay").small().label("打开 VQF 回放")
                    .on_click(cx.listener(|this, _, window, cx| {
                        if this.set_feature_page(FeaturePage::VqfReplay, window, cx) {
                            this.page_focus.focus(window, cx);
                            cx.notify();
                        }
                    })))
                .child(Button::new("arc8-ui-open-hidlog").small().label("打开完整 HID 日志")
                    .on_click(cx.listener(|this, _, window, cx| {
                        if this.set_feature_page(FeaturePage::HidLog, window, cx) {
                            this.page_focus.focus(window, cx);
                            cx.notify();
                        }
                    }))))
            .child(div().font_weight(FontWeight::SEMIBOLD).child("① 导出日志 → H2 结构化归档"))
            .child(Button::new("arc8-ui-select-log").label("选择 Tracker ARC8X1 控制台日志")
                .disabled(busy).on_click(cx.listener(|this, _, _, cx| {
                    this.arc8_choose_source(SourceFile::Log, cx);
                })))
            .child(div().text_xs().whitespace_normal().child(path_display(&offline.log)))
            .child(div().text_xs().child("设备 UID（人工核对；不能只填 Tracker 槽位号）"))
            .child(Input::new(&offline.uid).small().disabled(busy))
            .child(div().text_xs().child("接收器唯一标识 / 会话标识（人工填写，尚非固件认证）"))
            .child(Input::new(&offline.receiver_key).small().disabled(busy))
            .child(div().text_xs().child("采集来源、动作和参考标注"))
            .child(Input::new(&offline.note).small().disabled(busy))
            .child(Button::new("arc8-ui-parse").label("校验并生成归档 JSON（只读）")
                .disabled(busy || offline.log.is_none()).on_click(cx.listener(|this, _, _, cx| {
                    this.arc8_start_parse(cx);
                })))
            .child(div().font_weight(FontWeight::SEMIBOLD).child("② ARC7 同源 PC 联合求解"))
            .child(Button::new("arc8-ui-select-capture").label("选择已有完整归档 JSON")
                .disabled(busy).on_click(cx.listener(|this, _, _, cx| {
                    this.arc8_choose_source(SourceFile::Capture, cx);
                })))
            .child(div().text_xs().whitespace_normal().child(path_display(&offline.capture)))
            .child(Button::new("arc8-ui-solve").label("求解并保存未应用候选参数")
                .disabled(busy || offline.capture.is_none()).on_click(cx.listener(|this, _, _, cx| {
                    this.arc8_start_solve(cx);
                })))
            .child(div().font_weight(FontWeight::SEMIBOLD).child("③ 候选包完整性检查"))
            .child(Button::new("arc8-ui-select-bundle").label("选择已有候选参数 JSON")
                .disabled(busy).on_click(cx.listener(|this, _, _, cx| {
                    this.arc8_choose_source(SourceFile::Candidate, cx);
                })))
            .child(div().text_xs().whitespace_normal().child(path_display(&offline.candidate)))
            .child(Button::new("arc8-ui-verify").label("检查结构 / 摘要（不是设备验签）")
                .disabled(busy || offline.candidate.is_none()).on_click(cx.listener(|this, _, _, cx| {
                    this.arc8_start_verify(cx);
                })));
        if let Some(text) = offline.running {
            box_ = box_.child(div().text_sm().child(text));
        }
        if let Some(message) = &offline.notice {
            box_ = box_.child(
                div()
                    .text_sm()
                    .whitespace_normal()
                    .child(format!("成功：{message}")),
            );
        }
        if let Some(error) = &offline.error {
            box_ = box_.child(
                div()
                    .text_sm()
                    .text_color(cx.theme().danger)
                    .whitespace_normal()
                    .child(format!("拒绝：{error}")),
            );
        }
        box_ = box_.child(div().text_sm().whitespace_normal().child(
            "尚未实现/验收：十设备统一开始与时钟对齐、H3 无线归档、全量高频数据录制、校准参数暂存提交、NVS 读回和回退。此页不提供这些操作的假按钮。"
        ));
        box_.into_any_element()
    }
}
