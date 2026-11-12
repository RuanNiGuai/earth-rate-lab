from pathlib import Path
import json
import tempfile
import unittest
import subprocess
import sys
from types import SimpleNamespace
from example import generate
from inputs import configure, load


class InputTests(unittest.TestCase):
    def test_synthetic_input_parses_and_corruption_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = generate(Path(tmp)/'fixture')
            backend = SimpleNamespace()
            c = configure(path, backend)
            data = load()
            self.assertEqual(len(data), 10)
            self.assertEqual(backend.LATITUDE_DEG, 45)
            self.assertEqual(c['dataset_kind'], 'synthetic')
            result = subprocess.run([sys.executable, str(Path(__file__).with_name('make_plan.py')),
                '--config', str(path), '--result', str(path.parent/'absent-result.json'),
                '--session', str(path.parent/'absent-session'), '--out', str(path.parent/'plan'),
                '--reference-confirmed', '--authorization', 'Synthetic trial', '--trial'],
                capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Synthetic data cannot be installed', result.stderr)
            self.assertFalse((path.parent/'plan').exists())
            target = path.parent/'snapshot-2.bin'
            target.write_bytes(target.read_bytes()[:-1]+b'!')
            with self.assertRaisesRegex(ValueError, 'hash mismatch'):
                configure(path, backend)

    def test_wrong_fleet_or_overlapping_segments_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = generate(Path(tmp)/'fixture')
            good = json.loads(path.read_text())
            c = dict(good);c['devices'] = good['devices'][:-1]
            path.write_text(json.dumps(c))
            with self.assertRaisesRegex(ValueError, 'slots'):
                configure(path, SimpleNamespace())
            good['static_segments_s'][1] = good['static_segments_s'][0]
            path.write_text(json.dumps(good))
            with self.assertRaisesRegex(ValueError, 'ordered'):
                configure(path, SimpleNamespace())
