"""Exercise publication diffs against real local Git objects."""
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from audit_upstream import archived, audit, compare


class AuditTests(unittest.TestCase):
    def test_audit_pins_both_sides_and_ignores_unpublished_edits(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            commits = {}
            for key, files in {
                'project': {'integration/tracker/source.c': b'published\r\n',
                            'integration/tracker/vendor/file': b'dependency',
                            'tools/unrelated.py': b'not firmware'},
                'upstream': {'source.c': b'upstream\n', 'vendor/file': b'other'},
            }.items():
                repo = base / key
                repo.mkdir()
                def git(*args):
                    return subprocess.check_output(['git', '-C', str(repo), *args], stderr=subprocess.PIPE)
                git('init', '-q')
                (repo / '.gitattributes').write_text('* -text\n')
                for name, data in files.items():
                    target = repo / name
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_bytes(data)
                git('add', '.')
                git('-c', 'user.name=Fixture', '-c', 'user.email=fixture@example.invalid',
                    '-c', 'commit.gpgsign=false', 'commit', '-qm', 'fixture')
                commits[key] = git('rev-parse', 'HEAD').decode().strip()
                for name in files:
                    (repo / name).write_bytes(b'unpublished edit')
            pins = {'checked_at': '2026-11-13',
                    'project': {'commit': commits['project']},
                    'repositories': [{'key': 'tracker', 'url': 'https://example.invalid/tracker',
                                      'branch': 'main', 'head': commits['upstream'],
                                      'local_path': 'integration/tracker',
                                      'exclude': ['vendor', '.gitattributes']}]}
            result = audit(pins, base / 'project', {'tracker': base / 'upstream'})
            self.assertEqual(result['project']['commit'], commits['project'])
            row = result['repositories'][0]
            self.assertEqual((row['project_files'], row['upstream_files']), (1, 1))
            self.assertEqual(row['versus_upstream']['counts'], {'added': 0, 'modified': 1, 'removed': 0})
            source = row['versus_upstream']['files'][0]
            self.assertEqual(source['local_sha256'], hashlib.sha256(b'published\n').hexdigest())
            self.assertEqual(source['upstream_sha256'], hashlib.sha256(b'upstream\n').hexdigest())
            self.assertNotIn('versus_base', row)

    def test_added_removed_modified_and_binary_counts(self):
        before = {'same': b'a\n', 'edit': b'a\nb\n', 'gone': b'x\n', 'image': b'\xff'}
        after = {'same': b'a\n', 'edit': b'a\nc\nd\n', 'new': b'z\n', 'image': b'\xfe'}
        result = compare(before, after)
        self.assertEqual(result['counts'], {'added': 1, 'modified': 2, 'removed': 1})
        files = {row['path']: row for row in result['files']}
        self.assertNotIn('same', files)
        self.assertEqual((files['edit']['added_lines'], files['edit']['removed_lines']), (2, 1))
        self.assertEqual(files['edit']['local_sha256'], hashlib.sha256(after['edit']).hexdigest())
        self.assertTrue(files['image']['binary'])
        self.assertNotIn('upstream_sha256', files['new'])
        self.assertNotIn('local_sha256', files['gone'])

    def test_archive_is_pinned_normalized_and_excludes_subtree(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            def git(*args):
                return subprocess.check_output(['git', '-C', str(root), *args], stderr=subprocess.PIPE)
            git('init', '-q')
            (root / '.gitattributes').write_text('* -text\n')
            (root / 'source').write_bytes(b'first\r\n')
            (root / 'vendor').mkdir()
            (root / 'vendor' / 'file').write_bytes(b'dependency')
            git('add', '.')
            git('-c', 'user.name=Fixture', '-c', 'user.email=fixture@example.invalid',
                '-c', 'commit.gpgsign=false', 'commit', '-qm', 'fixture')
            revision = git('rev-parse', 'HEAD').decode().strip()
            (root / 'source').write_bytes(b'uncommitted edit')
            snapshot = archived(root, revision, ['vendor'])
            self.assertEqual(snapshot['source'], b'first\n')
            self.assertNotIn('vendor/file', snapshot)
            self.assertEqual((root / 'source').read_bytes(), b'uncommitted edit')


if __name__ == '__main__':
    unittest.main()
