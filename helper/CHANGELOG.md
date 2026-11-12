# Changelog

[简体中文](CHANGELOG.zh-CN.md)

All notable changes to SlimeNRF Helper are documented in this file.

The format follows [Keep a Changelog 1.1.0](https://keepachangelog.com/en/1.1.0/).
Version numbers follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed

- Repeating a reviewed calibration write now verifies and skips devices already at the exact target snapshot. Fleet preflight, partial-state rejection and rollback of this attempt remain enforced; saved results distinguish readback from reboot verification.
- Wireless updates now support the ARC8 OTA v2 profile, including compact tracker information, 44-byte payloads, relay protocol checks and longer erase acknowledgement. Accepted tracker activation is shown as awaiting manual reboot, with relay cleanup; it is not reported as verified boot. OTA v1 remains supported.
- Guided joint calibration now compares old and candidate scales using all ten recordings, displays rest and motion-return metrics, and creates field-specific plans that retain backed-up Earth and g-sensitivity values. Unused candidate diagnostics no longer block validated scales. Recomparison revokes stale plans, and resume restores reviewed plans. Stable low-pass and adaptive-bias comparisons are desktop replays only; the latter is not recommended because of slow-motion counterexamples.
- Joint calibration now reports measured values and limits and saves a review when plan quality gates fail, instead of only showing a traceback. Resuming restores the review; stale plan admission is cleared while recordings and solver results remain intact. Failed candidates still cannot be installed.
- Bundled temperature controls now verify the reported automatic-collection switch after command completion; the status field is no longer mislabeled as compensation enablement.
- Bundled calibration telemetry no longer interrupts capture when Windows temporarily denies replacement of a progress file. Longer stationary recordings retain motion-based clock alignment. The guide recommends longer Earth dwells and an optional final return to the original face and heading; ten-rest captures remain compatible.
- Ten-device session setup now shows verification rounds, tracker counts and an explicit completion message with the next recording step. Repeating setup reuses and verifies the existing session instead of creating nested folders; recording preparation retains its separate stillness/READY prompt.
- Joint input reading also accepts the reviewed `06ce018fa6d1_wom3_t02_on` repair build, which retains the deterministic temperature LUT protocol. Unknown successor builds remain rejected; identity, data and installation checks are unchanged.

### Added

- Reorganized joint calibration into one expanded workflow step, with optional result details and collapsed legacy diagnostics. Capture now offers single-tracker full-rate recording and ten-tracker reliable F04 recording as equal modes. The temperature navigation entry now runs confirmed wireless collection controls and table coverage audits; independent tool runs preserve calibration artifacts. Wireless OTA guidance points to companion firmware candidates.

- Guided ten-device calibration with a bundled PC component, automatic session/file management, orientation-aware READY, three continuous forward/reverse 20-turn captures and ten continuous Earth/g-sensitivity poses. All four recordings enter automatic clock alignment, segmentation and joint fitting with separate accelerations and lever arms. Signed scale candidates, temperature/speed/rest diagnostics and rejected captures are retained for firmware research; large direction differences block a single-scale install plan. Advanced imports and legacy archives remain available. Device reservations last through worker cleanup; no new tracker firmware is required for this workflow.
- Added ARC, Earth, g-sensitivity and diagnostic command buttons with USB/wireless routing and bilingual descriptions. Temperature observation collection is documented as opt-in each boot, independently of applying a stored compensation table.

- Joint Scale / Earth / GSens reconstruction in the ARC8 workbench, computed natively from a raw recording, stored automatic temperature data and three 20-turn Scale records. Direct USB installation backs up coefficients, verifies readback after a tracker reboot and attempts restoration on failure or cancellation. See the user guide for preparation and limits.
- ARC8 H1–H3 native offline workbench: import verified ARC8X1 console exports, run the pinned ARC7 PC solver, validate candidate bundles, and open existing raw capture/replay views. No device calibration writes or claims of NVS verification.

- Experimental ARC8 H2 read-only per-operation event evidence ledger and a separate PC-side ARC7 calibration candidate solver. These legacy archive tools remain separate from joint reconstruction and do not install their candidate bundles.

## [0.4.3] - 2026-10-30

### Fixed

- Wireless-update tracker cards keep their address and firmware details on the enclosing card background instead of showing a mismatched inner fill when selected.

## [0.4.2] - 2026-10-30

### Added

- Raw capture previews show the latest valid raw acceleration magnitude in g, separately from the X/Y/Z components; missing observations are not treated as zero.

### Fixed

- Capture plots distinguish signed Y-axis ticks from sensor readings and clarify acceleration, dimensionless gyro-integral quaternion, angular-rate, and magnetic-field component units and meanings.

## [0.4.1] - 2026-10-30

### Fixed

- CDC raw recording initializes the receiver's command console before START, avoiding an endless wait for samples on firmware that initializes the data interface only after the console opens. Existing terminal connections are reused; otherwise the console is opened temporarily and released before recording starts.

## [0.4.0] - 2026-10-30

### Changed

- Internal application source is organized by capability, colocating models, services, views and tests. Workbench-owned adapters remain explicit application orchestration; standalone replay and analysis-window owners retain their own state and completion handling. This is an ownership refactor, not a measured performance improvement.
- Device-command continuation and background completion handling no longer depend on rendering or the active page. HID presentation is initialized through explicit page activation, and replay preset menus prepare their snapshots only when opened.

### Fixed

- Command/history tabs, firmware source choices and capture source cards support semantic keyboard activation as well as pointer input. OTA Retry no longer also changes tracker selection.
- Playback-speed changes account for elapsed time at the previous speed before applying the new value, keeping keyboard and menu changes consistent without restarting an unchanged speed.
- UF2 preset controls retain their identity when device lists reorder, and distinct named variants sharing a firmware resource remain independently keyboard-accessible, without changing volume or device authorization checks.

## [0.3.2] - 2026-10-29

### Fixed

- Fixed a stack-overflow crash in Windows debug builds when connecting serial terminals or displaying controls for multiple connected ports.

## [0.3.1] - 2026-10-29

### Changed

- Source builds now require Rust 1.99 or newer, aligning the declared minimum with the pinned development toolchain's minor release.
- Tracker-card temperatures sit below TPS at the right of the hardware ID row. Compact plug and full-battery icons stay beside the tighter battery reading. In narrow cards, the battery reading truncates instead of wrapping online and selection states; hover shows the complete reading and firmware-state explanations.

### Fixed

- Windows portable updates rename official versioned executables to the new release's filename while preserving custom names and refusing destination collisions. This takes effect when updating from a version containing the fix.

## [0.3.0] - 2026-10-29

### Added

- Command-list search matches command text, names, and descriptions in the current language.
- Both wired and wireless online firmware lists support filename and variant search without changing selected firmware.

### Changed

- UF2 and OpenDFU updates show a graphical progress bar, transferred bytes, and percentage; unknown totals remain indeterminate, and 100% waits for the final operation result.
- Receiver self-update uses a three-second activation-observation budget instead of a prolonged reconnect wait. Missing confirmation ends with **Transfer complete; activation not confirmed**, preserving the available diagnostic evidence.
- Tracker cards place TPS at the upper right and combine battery percentage and voltage below, alongside temperature and firmware-reported power/full indicators. Compact layouts wrap the readings, with hover explanations for encoded telemetry.

### Fixed

- Wireless firmware-information query failures appear only on the affected device card, without duplicate warnings in the top results area.
- Builds from the exact matching release tag display the version without a commit hash, including prereleases. Development builds and modified local release checkouts retain their build identity.

## [0.2.0] - 2026-10-28

### Added

- A debug-only developer gallery with isolated synthetic controls and terminal scenes, plus documented device diagnosis and native UI smoke workflows.
- Terminal tabs can filter displayed lines by log level while retaining untagged command output; Copy all and log export remain unfiltered.
- The mouse wheel can switch log levels over the terminal filter control and recall command history over the input without sending commands.
- All platforms can check GitHub releases and open a newer version's release page. Official Windows portable builds can also download a matching integrity-verified update and restart to install it. Stable and prerelease channels are selectable; automatic checks do not download or install updates.

### Changed

- RSSI analysis uses a bar chart with an automatically scaled dBm axis, subtle **No sample** markers, and immediate hover details. Actual channel positions and current/recommended markers are preserved. The chart fits the available window height so its explanation remains visible at normal window sizes.
- The displayed application version includes the checkout's short commit ID and marks tracked local changes as dirty; source archives fall back to the base version or a validated GitHub Actions SHA. Release versions, tags, asset names, and update comparisons remain unchanged.
- Device lists use a flatter, more compact layout with a rounded summary bar: receiver serial numbers and online/total tracker counts share a wrapping metadata row, with the redundant command-console heading and outer list border and padding removed.
- Device-list toolbar actions use compact icon buttons with localized hover descriptions.
- Direct tracker commands now show the **Information** group first; the remaining command groups keep their existing order.
- The terminal Send button uses a larger return-key icon button aligned with the command input. Send all appears directly below it in the existing lower toolbar only when multiple eligible command ports are connected, without reserving extra vertical space.
- Selected terminal text is copied with keyboard shortcuts instead of a separate toolbar button.
- The root is now a virtual Cargo workspace, with the desktop application's source and build support in `crates/app` alongside the existing recording core. Root development commands and resource path semantics are unchanged.
- The terminal model and firmware-image parsing and content validation are isolated in internal workspace libraries. GUI presentation, hardware access, device authorization, and application build resources remain in the desktop app; the new libraries can be built and tested independently of its build script. This is an internal boundary change, not a new user-facing feature.
- Developer test scripts use cargo-nextest, with doctests available as a separate action; quality scripts run both. Windows MSVC builds use the Rust toolchain's bundled LLD linker, and Linux builds can opt into mold.
- Release builds use optimization level 2, fat LTO, one codegen unit, and symbol stripping.
- Main, license notices, RSSI analysis, and temperature calibration analysis windows now use theme-aware GPUI Kit title bars instead of system title-bar styling, while preserving window controls and shutdown cleanup.
- Application builds, development checks, packaging, CI, and the independent firmware oracle no longer require Python. One internal Rust tools package shares the frozen-input notice renderer with application builds; its maintenance CLI remains separate from the desktop runtime.
- Repeated Windows build-environment setup no longer accumulates static-CRT flags or Visual Studio tool and compiler search paths, avoiding unnecessary Cargo cache invalidation from nested developer terminals.
- Debug builds, runs, and tests can continue when license notices need review, with a Cargo warning and a pending-review message in the copyright page instead of a stale inventory. Release builds and packaging still require current reviewed notices.
- Windows packaging and CI now publish only the UPX-compressed executable under the normal filename, without a separate uncompressed download or `-upx` suffix. Compression uses `--best --lzma` instead of exhaustive `--ultra-brute` search.

### Fixed

- Moving the pointer after moving or resizing the window no longer unexpectedly selects terminal text; an existing selection is preserved.
- Resuming terminal Follow immediately shows the latest line even without new output, while preserving deliberate scrolling and selection.
- Tracker cards retain equal widths at fractional display scaling, and their content no longer shifts when activity changes.
- Settings navigation remains scrollable and keyboard-accessible when the available window height is reduced or interface text is enlarged.
- Settings and other scrollable pages keep scrollbar tracks and pointer hit areas fixed to their viewports, so thumb positions and drag ranges remain aligned with the visible content.

## [0.1.1] - 2026-10-26

### Added

- Project repository links in the **About** section of **Settings** for SlimeNRF Helper, the receiver firmware, the tracker firmware, and the bootloader.
- CI packaging and prerelease automation for separate normal and UPX-compressed Windows x64 executables and a Linux x64 AppImage, all without bundled firmware.

### Changed

- License notices are generated during application compilation and package staging instead of tracked as a generated source file. Application builds, checks, tests, editor analysis, and packaging now require Python 3.11+; running the application and building only the recording core do not.

### Fixed

- Existing terminal text now follows dark and light theme changes for default and basic ANSI colors. Extended indexed colors and RGB colors retain their specified values.
- Windows UPX packaging no longer passes the unsupported `--exact` option. Maximum compression remains enabled.

## [0.1.0]

Initial standalone application version, documented from the repository's initial source snapshot.

### Added

- Receiver and tracker discovery, independent serial sessions, tabbed terminals, command history, log export, and confirmation for destructive device commands.
- UF2, OpenDFU, receiver-based ESB OTA, and advanced legacy BLE DFU updates, with local, bundled, and supported remote firmware sources.
- HID diagnostics, raw sensor capture, offline VQF analysis, synchronized plots, and an orientation preview.
- Gyroscope calibration guides, temperature calibration analysis with SVG export, and RSSI channel analysis.
- English and Simplified Chinese interfaces, light and dark themes, and Windows and Linux packaging with full-firmware and no-firmware build modes.

[Unreleased]: https://github.com/jitingcn/SlimeNRF-Helper/compare/v0.4.3...HEAD
[0.4.3]: https://github.com/jitingcn/SlimeNRF-Helper/compare/v0.4.2...v0.4.3
[0.4.2]: https://github.com/jitingcn/SlimeNRF-Helper/compare/v0.4.1...v0.4.2
[0.4.1]: https://github.com/jitingcn/SlimeNRF-Helper/compare/v0.4.0...v0.4.1
[0.4.0]: https://github.com/jitingcn/SlimeNRF-Helper/compare/v0.3.2...v0.4.0
[0.3.2]: https://github.com/jitingcn/SlimeNRF-Helper/compare/v0.3.1...v0.3.2
[0.3.1]: https://github.com/jitingcn/SlimeNRF-Helper/compare/v0.3.0...v0.3.1
[0.3.0]: https://github.com/jitingcn/SlimeNRF-Helper/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/jitingcn/SlimeNRF-Helper/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/jitingcn/SlimeNRF-Helper/releases/tag/v0.1.1
[0.1.0]: https://github.com/jitingcn/SlimeNRF-Helper/tree/c1534d42a08b71e2bf9acdf0c33892766493080e
