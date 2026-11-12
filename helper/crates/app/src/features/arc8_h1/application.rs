//! Native ARC8-H1 page on the upstream receiver monitor and traced HID hub.
//! Joint reconstruction plus the legacy archive and receiver diagnostics.
use super::{FeaturePage, Workbench};
use crate::components::page_scroll::page_scroll;
use crate::receiver_events::EventWatchStatus;
use gpui_kit::component::{ActiveTheme, button::Button};
use gpui_kit::{AnyElement, Context, FontWeight, div, prelude::*};

fn status_text(status: EventWatchStatus) -> String {
    match status {
        EventWatchStatus::Disabled => "未订阅 / Disabled".into(),
        EventWatchStatus::Checking => "正在查询 v1 能力 / Checking".into(),
        EventWatchStatus::Active => "v1 事件订阅有效 / Active".into(),
        EventWatchStatus::Paused => "独占作业期间暂停 / Paused".into(),
        EventWatchStatus::Unsupported(message) => format!("不支持 / Unsupported: {message}"),
        EventWatchStatus::Error(message) => format!("错误 / Error: {message}"),
    }
}

impl Workbench {
    pub(in crate::application) fn poll_arc8_h1(&mut self) -> bool {
        let changed = self.arc8_h1.poll();
        changed && self.active_page == FeaturePage::Arc8H1
    }

    pub(super) fn arc8_h1_page(&self, cx: &mut Context<Self>) -> AnyElement {
        let mut content = div()
            .w_full()
            .min_w_0()
            .flex()
            .flex_col()
            .gap_3()
            .p_4()
            .child(
                div()
                    .text_lg()
                    .font_weight(FontWeight::SEMIBOLD)
                    .child(crate::i18n::tr("app.navigation.arc8_h1")),
            );

        content = content.child(self.fleet_panel(cx));
        content = content.child(
            gpui_kit::component::checkbox::Checkbox::new("legacy-tools")
                .label(crate::i18n::tr("workbench.legacy_tools"))
                .checked(self.fleet.legacy_tools)
                .on_click(cx.listener(|this, value, _, cx| {
                    this.fleet.legacy_tools = *value;
                    cx.notify();
                })),
        );
        if !self.fleet.legacy_tools {
            return page_scroll("arc8-h1-page", &self.page_focus, content).into_any_element();
        }
        content = content.child(self.joint_panel(cx));
        content = content.child(self.arc8_offline_panel(cx));

        let receivers = &self.inventory.receivers;
        content = content.child(
            div()
                .font_weight(FontWeight::SEMIBOLD)
                .child(format!("HID 接收器候选 · {}", receivers.len())),
        );
        if receivers.is_empty() {
            content = content.child(
                div()
                    .text_sm()
                    .text_color(cx.theme().muted_foreground)
                    .child("未发现主 HID 接口；检查接收器 USB 连接及系统权限。"),
            );
        }
        for (index, receiver) in receivers.iter().enumerate() {
            let observation = self.monitor.observation(&receiver.key);
            let watching = self.monitor.events_enabled(&receiver.key);
            let status = status_text(self.monitor.events_status(&receiver.key));
            let receiver_copy = receiver.clone();
            let count = observation.map_or(0, |obs| obs.trackers.len());
            let mut receiver_box = div()
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
                        .font_weight(FontWeight::SEMIBOLD)
                        .child(receiver.label.clone()),
                )
                .child(div().text_xs().whitespace_normal().child(format!(
                    "USB serial: {} · 已观察到 {} 个 tracker · {}",
                    receiver.serial_number.as_deref().unwrap_or("未提供 / unknown"),
                    count, status
                )))
                .child(
                    Button::new(format!("arc8-h1-watch-{index}"))
                        .label(if watching {
                            "停止事件订阅"
                        } else {
                            "查询事件能力并订阅（224）"
                        })
                        .on_click(cx.listener(move |this, _, _, cx| {
                            // The existing receiver reader owns both this command and ACKs.
                            this.arc8_h1.error = this
                                .monitor
                                .set_events_enabled(&receiver_copy, !watching)
                                .err();
                            cx.notify();
                        })),
                );
            if let Some(observation) = observation {
                if let Some(error) = &observation.error {
                    receiver_box =
                        receiver_box.child(div().text_xs().child(format!("读取错误: {error}")));
                }
                for tracker in &observation.trackers {
                    let addr = tracker.address.as_deref().unwrap_or("未知");
                    receiver_box =
                        receiver_box.child(div().text_xs().whitespace_normal().child(format!(
                            "Tracker #{} · 地址 {} · {} · {} TPS · 仅已观察到",
                            tracker.slot,
                            addr,
                            if tracker.online {
                                "最近有传感器数据"
                            } else {
                                "暂无近期传感器数据"
                            },
                            observation.tps(tracker.slot)
                        )));
                }
            }
            content = content.child(receiver_box);
        }
        content = content.child(
            div()
                .flex()
                .items_center()
                .gap_2()
                .flex_wrap()
                .child(div().font_weight(FontWeight::SEMIBOLD).child(format!(
                    "事件观察（最近 {} 条） · 丢失 {} · 拒绝无效 {}",
                    self.arc8_h1.rows.len(),
                    self.arc8_h1.dropped,
                    self.arc8_h1.invalid
                )))
                .child(
                    Button::new("arc8-h1-clear")
                        .label("清空显示")
                        .on_click(cx.listener(|this, _, _, cx| {
                            this.arc8_h1.clear();
                            cx.notify();
                        })),
                )
                .child(
                    Button::new("arc8-h1-open-hidlog")
                        .label("打开完整 HID 日志")
                        .on_click(cx.listener(|this, _, window, cx| {
                            if this.set_feature_page(FeaturePage::HidLog, window, cx) {
                                this.page_focus.focus(window, cx);
                                cx.notify();
                            }
                        })),
                ),
        );
        if let Some(error) = &self.arc8_h1.error {
            content = content.child(
                div()
                    .text_sm()
                    .text_color(cx.theme().danger)
                    .child(error.clone()),
            );
        }
        if self.arc8_h1.rows.is_empty() {
            content = content.child(
                div()
                    .text_sm()
                    .text_color(cx.theme().muted_foreground)
                    .child("暂无事件。需要接收器和追踪器都支持事件回传，且先启用订阅。"),
            );
        }
        // H2: best-effort evidence only, never a device persistent-commit acknowledgement.
        content = content.child(div().font_weight(FontWeight::SEMIBOLD).child(format!(
            "H2 操作证据台账（最近 {} 个） · 淘汰 {} · 乱序/重复 {}",
            self.arc8_h1.evidence.operations.len(),
            self.arc8_h1.evidence.evicted,
            self.arc8_h1.evidence.out_of_order
        )));
        content = content.child(div().text_sm().whitespace_normal().child(
            "后续联合校准拟在 PC 端使用 ARC7 同源求解器。当前 Tracker 尚无经认证的校准包写入/重启读回接口；所有记录均不得标为 NVS 已验证。"
        ));
        for operation in self.arc8_h1.evidence.operations.iter().take(24) {
            content = content.child(
                div()
                    .w_full()
                    .min_w_0()
                    .p_2()
                    .rounded_md()
                    .border_1()
                    .border_color(cx.theme().border)
                    .child(
                        div()
                            .text_xs()
                            .whitespace_normal()
                            .child(operation.summary()),
                    ),
            );
        }
        for row in self.arc8_h1.rows.iter().rev().take(80) {
            content = content.child(
                div()
                    .w_full()
                    .min_w_0()
                    .p_2()
                    .rounded_md()
                    .border_1()
                    .border_color(cx.theme().border)
                    .child(div().text_xs().whitespace_normal().child(format!(
                        "{} · {:.3}s · {}",
                        row.receiver_label,
                        row.elapsed_us as f64 / 1_000_000.,
                        row.describe()
                    ))),
            );
        }
        page_scroll("arc8-h1-page", &self.page_focus, content).into_any_element()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn status_is_explicitly_noncommittal() {
        assert!(status_text(EventWatchStatus::Checking).contains("查询"));
        assert!(status_text(EventWatchStatus::Active).contains("订阅"));
    }
}
