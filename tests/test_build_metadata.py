"""Exercise the actual CMake metadata filter without an SDK or hardware."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class MetadataTests(unittest.TestCase):
    def test_display_urls_and_c_strings(self):
        cmake = shutil.which('cmake')
        if cmake is None:
            sibling = Path(sys.executable).parent/'cmake.exe'
            if sibling.is_file():
                cmake = str(sibling)
        self.assertIsNotNone(cmake, 'Install CMake to run the complete offline checks')
        source = ROOT/'integration/tracker/cmake/display_metadata.cmake'
        self.assertEqual(source.read_bytes(), (ROOT/'integration/receiver/cmake/display_metadata.cmake').read_bytes())
        cases = [
            ('https://github.com/example/project.git', 'https://github.com/example/project'),
            ('git@github.com:example/project.git', 'https://github.com/example/project'),
            ('ssh://git@github.com/example/project.git', 'https://github.com/example/project'),
            ('D:/private/checkout', 'unknown'),
            ('/private/checkout', 'unknown'),
            ('https://user:password@github.com/example/project', 'unknown'),
            ('https://github.com/example/project?token=secret', 'unknown'),
            ('', 'unknown'),
        ]
        base = ROOT/'.local/native-tests'
        base.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=base) as td:
            temp = Path(td)
            for url, expected in cases:
                script = temp/'metadata.cmake'
                output = temp/'result.txt'
                script.write_text(
                    f'include("{source.as_posix()}")\n'
                    f'set(GIT_REPO_URL [==[{url}]==])\n'
                    'set(GIT_BRANCH [==[branch;name]==])\n'
                    'set(GIT_AUTHOR [==[A "quoted" \\ name]==])\n'
                    'sanitize_display_metadata()\n'
                    f'file(WRITE "{output.as_posix()}" "${{GIT_REPO_URL}}\n${{GIT_BRANCH}}\n${{GIT_AUTHOR}}")\n',
                    encoding='utf8')
                subprocess.run([cmake, '-P', str(script)], check=True, capture_output=True)
                self.assertEqual(output.read_text().splitlines(),
                                 [expected, 'branch name', 'A \\"quoted\\" \\\\ name'])
