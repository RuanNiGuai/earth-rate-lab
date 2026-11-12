"""Run the published offline checks. Never opens trackers or receivers."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    suites = [('tools', 'test_calwire*.py'), ('tools', 'test_batch_*.py'),
              ('tools/jointcal', 'test_*.py'), ('tests', 'test_*.py'),
              ('integration/tests', 'test_calwire.py')]
    for directory, pattern in suites:
        subprocess.run([sys.executable, '-B', '-X', 'utf8', '-m', 'unittest',
                        'discover', '-s', directory, '-p', pattern, '-v'], cwd=ROOT, check=True)


if __name__ == '__main__':
    main()
