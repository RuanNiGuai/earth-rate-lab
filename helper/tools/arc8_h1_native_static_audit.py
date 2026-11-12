#!/usr/bin/env python3
"""Static-only ARC8 H1 integration checks. NOT a Rust compile or device test."""
from pathlib import Path
import sys
import yaml

root = Path(__file__).resolve().parents[1]
app = root / 'crates/app/src'
checks = []

def check(label, ok):
    checks.append((label, bool(ok)))
    print(('PASS' if ok else 'FAIL'), label)

def content(path):
    return (app / path).read_text(encoding='utf-8')

feature = content('features/arc8_h1/mod.rs')
protocol = content('features/arc8_h1/protocol.rs')
ui = content('features/arc8_h1/application.rs')
nav = content('application/navigation.rs')
main = content('application/mod.rs')
pump = content('application/pump.rs')
receiver = content('receiver_events.rs')
discovery = content('discovery.rs')
check('Feature and UI module registered',
      'pub(crate) mod arc8_h1;' in content('features/mod.rs')
      and 'mod arc8_h1;' in main and 'FeaturePage::Arc8H1' in main)
check('Navigation and polling integrated',
      'nav-arc8-h1' in nav and 'poll_arc8_h1()' in pump)
check('Existing HID reader reused',
      'set_events_enabled' in ui and 'HidApi' not in feature + ui + protocol
      and 'hid_log::subscribe()' in feature)
check('Underlying 224 event protocol and monitor retained',
      'const OPCODE: u8 = 224' in receiver and 'fn read_receiver_reports(' in discovery)
check('No ARC8-H1 OTA/mutation command paths',
      all(token not in ui + feature + protocol for token in
          ['boot_request_upgrade(', 'OTA_BEGIN', 'flash_area_write(',
           'send_calibration(', 'HidDevice::open(']))
check('Bounded event queue and polling',
      'const MAX_ROWS: usize = 256;' in feature
      and 'const POLL_BUDGET: usize = 1024;' in feature)
check('ARC8 event validation & synthetic unknown labeling',
      'valid_event(&event)' in feature and 'synthetic_unknown' in feature
      and 'synthetic_unknown' in protocol and 'event.boot_id == 0' in protocol)
check('Lifecycle cleans up UI trace subscription',
      'self.arc8_h1.deactivate()' in nav
      and 'self.arc8_h1.deactivate()' in content('application/lifecycle.rs'))
check('UI distinguishes event success from NVS persisted',
      'END SUCCESS 不等于 NVS' in ui and 'NVS persisted' not in ui)
for locale in ('en','zh-CN'):
    data = yaml.safe_load((root/'locales'/f'{locale}.yml').read_text(encoding='utf-8'))
    check(f'{locale} navigation label',
          bool(data['app']['navigation'].get('arc8_h1')))
print(f'{sum(ok for _,ok in checks)}/{len(checks)} static checks passed. Rust build NOT tested.')
sys.exit(0 if all(ok for _,ok in checks) else 1)
