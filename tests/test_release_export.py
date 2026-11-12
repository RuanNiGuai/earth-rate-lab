from pathlib import Path
import json
import sys
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from release_export import export


class ReleaseTests(unittest.TestCase):
    def test_explicit_component_manifest_keeps_reviewed_hidden_files_and_rejects_escape(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'source';root.mkdir();self.fixture(root,[])
            listing=root/'config/helper-files.json';listing.write_text(json.dumps(['.github/test.yml']))
            spec=dict(files=[],trees=[],manifests=[dict(root='helper',files='config/helper-files.json')])
            (root/'config/publish.json').write_text(json.dumps(spec))
            child=root/'helper/.github/test.yml';child.parent.mkdir(parents=True);child.write_text('reviewed')
            out=Path(tmp)/'out';export(root,out)
            self.assertEqual((out/'helper/.github/test.yml').read_text(),'reviewed')
            listing.write_text(json.dumps(['../../outside.txt']))
            (Path(tmp)/'outside.txt').write_text('private')
            with self.assertRaisesRegex(ValueError,'escape'):export(root)

    def fixture(self, root, files):
        (root/'config').mkdir()
        (root/'config/publish.json').write_text(json.dumps(dict(files=files,trees=[])))

    def test_secret_refused_before_destination_created(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'source';root.mkdir();self.fixture(root,['accidental.txt'])
            (root/'accidental.txt').write_bytes(b'-----BEGIN '+b'EC PRIVATE KEY-----\nsecret')
            out=Path(tmp)/'out'
            with self.assertRaisesRegex(ValueError,'Secret'):
                export(root,out)
            self.assertFalse(out.exists())

    def test_unpublished_link_and_parent_escape_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'source';root.mkdir();self.fixture(root,['README.md'])
            (root/'README.md').write_text('[report](.local/hidden.md)')
            with self.assertRaisesRegex(ValueError,'Unpublished'):
                export(root)
            (Path(tmp)/'outside.txt').write_text('private')
            (root/'config/publish.json').write_text(json.dumps(dict(files=['../outside.txt'],trees=[])))
            with self.assertRaisesRegex(ValueError,'escape'):
                export(root)

    def test_existing_destination_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'source';root.mkdir();self.fixture(root,['README.md'])
            (root/'README.md').write_text('source')
            out=Path(tmp)/'out';out.mkdir();(out/'keep.txt').write_text('keep')
            with self.assertRaises(FileExistsError):
                export(root,out)
            self.assertEqual((out/'keep.txt').read_text(),'keep')
