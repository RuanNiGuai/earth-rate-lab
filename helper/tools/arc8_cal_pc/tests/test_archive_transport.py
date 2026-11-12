"""ARC8-H3 wire/PC end-to-end tests using the *actual* native C wire encoder.

The data produced by wire_fixture.c is SYNTHETIC and only tests serialization;
it is not used as calibration quality proof or a real device observation.
"""
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import calibration as cal
import archive_transport as wire

TRACKER = ROOT.parents[2] / 'tracker_receiver' / 'tracker' / 'ncs' / 'tracker'
# The test bundle can be copied into another location, in which case a source
# checkout may be provided explicitly with ARC8_TRACKER_ROOT.
import os
if 'ARC8_TRACKER_ROOT' in os.environ:
    TRACKER = Path(os.environ['ARC8_TRACKER_ROOT'])

class ArchiveWireTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cc=os.environ.get('CC') or shutil.which('gcc') or shutil.which('clang')
        if not cc or not TRACKER.joinpath('src/sensor/cal_archive_wire.c').exists():
            raise unittest.SkipTest('native compiler or tracker sources unavailable')
        cls.tmp=tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        src = TRACKER / 'src/sensor'
        fixture = ROOT / 'tests' / 'native' / 'wire_fixture.c'
        if not fixture.exists():
            raise unittest.SkipTest('native wire fixture absent')
        exe=Path(cls.tmp.name)/('arc8_fixture.exe' if os.name=='nt' else 'arc8_fixture')
        cmd=[cc,'-std=c11','-O1','-I',str(src),str(fixture),
             str(src/'cal_archive_wire.c'),str(src/'cal_archive_core.c'),'-o',str(exe)]
        if sys.platform!='darwin': cmd.append('-lm')
        subprocess.run(cmd,check=True,capture_output=True,text=True)
        cls.fixture=subprocess.run([str(exe)],check=True,capture_output=True,text=True).stdout.splitlines()

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls,'tmp'): cls.tmp.cleanup()

    def test_c_producer_python_decoder_roundtrip(self):
        ref, d=wire.decode_lines(['unrelated console boot line']+self.fixture)
        self.assertEqual(ref,129)
        self.assertAlmostEqual(d['tcal']['point'][0]['gyro'][0],0.0125,places=7)
        self.assertEqual(d['scale']['axis'][0]['turns'],20)
        self.assertEqual(d['gsens']['pose'][0]['quat'][0],1.)
        result=wire.make_capture(self.fixture,'TRACKER-OPERATOR-129','RX-OPERATOR-123','TEST FIXTURE, NOT REAL DATA')
        self.assertEqual(result['schema'],cal.CAPTURE_SCHEMA)
        self.assertEqual(cal.assemble(result)['earth'].count,5)

    def test_missing_done_rejected(self):
        with self.assertRaisesRegex(ValueError,'DONE'):
            wire.decode_lines(self.fixture[:-1])

    def test_missing_chunk_rejected(self):
        data=self.fixture.copy()
        next(i for i,x in enumerate(data) if x.startswith('ARC8X1 DATA tcal'))
        data.pop(next(i for i,x in enumerate(data) if x.startswith('ARC8X1 DATA tcal')))
        with self.assertRaisesRegex(ValueError,'DATA frame'):
            wire.decode_lines(data)

    def test_duplicate_chunk_rejected(self):
        data=self.fixture.copy()
        i=next(i for i,x in enumerate(data) if x.startswith('ARC8X1 DATA tcal'))
        data.insert(i+1,data[i])
        with self.assertRaisesRegex(ValueError,'DATA frame'):
            wire.decode_lines(data)

    def test_corrupt_chunk_crc_rejected(self):
        data=self.fixture.copy()
        i=next(i for i,x in enumerate(data) if x.startswith('ARC8X1 DATA tcal'))
        parts=data[i].split()
        parts[4]=('f' if parts[4][0]!='f' else '0')+parts[4][1:]
        data[i]=' '.join(parts)
        with self.assertRaisesRegex(ValueError,'CRC32'):
            wire.decode_lines(data)

    def test_mixed_reference_rejected(self):
        data=self.fixture.copy()
        i=next(i for i,x in enumerate(data) if x.startswith('ARC8X1 BEGIN scale'))
        parts=data[i].split(); parts[5]='130'; data[i]=' '.join(parts)
        with self.assertRaisesRegex(ValueError,'mixed archive reference'):
            wire.decode_lines(data)

    def test_duplicate_export_rejected(self):
        with self.assertRaisesRegex(ValueError,'second export'):
            wire.decode_lines(self.fixture+self.fixture)

    def test_incomplete_dataset_rejected(self):
        data=[x for x in self.fixture if not x.startswith('ARC8X1 BEGIN gsens')
              and not x.startswith('ARC8X1 DATA gsens')
              and not x.startswith('ARC8X1 END gsens')]
        with self.assertRaisesRegex(ValueError,'premature DONE'):
            wire.decode_lines(data)

    def test_inner_record_checksum_detected(self):
        data=self.fixture.copy()
        # Modify one chunk, and update BEGIN transport CRC so only the inner
        # ARC7 record CRC detects the alteration.
        start=next(i for i,x in enumerate(data) if x.startswith('ARC8X1 BEGIN tcal'))
        end=next(i for i,x in enumerate(data) if x.startswith('ARC8X1 END tcal'))
        chunks=[i for i in range(start+1,end) if data[i].startswith('ARC8X1 DATA tcal')]
        original=b''.join(bytes.fromhex(data[i].split()[4]) for i in chunks)
        tampered=bytearray(original); tampered[24]^=1
        cursor=0
        for i in chunks:
            parts=data[i].split(); n=len(parts[4])//2
            parts[4]=tampered[cursor:cursor+n].hex();data[i]=' '.join(parts);cursor+=n
        begin=data[start].split();begin[4]=f'{zlib.crc32(tampered):08x}'
        data[start]=' '.join(begin)
        with self.assertRaisesRegex(ValueError,'inner ARC7 record CRC'):
            wire.decode_lines(data)

    def test_cli_no_partial_output_on_corrupted_log(self):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
            td=Path(td);log=td/'fixture.log';out=td/'capture.json'
            log.write_text('\n'.join(self.fixture[:-1]),encoding='utf-8')
            result=wire.main(['parse-log',str(log),'--tracker-uid','OPERATOR-129',
                              '--receiver-key','OPERATOR-RX','--note','synthetic test',
                              '--out',str(out)])
            self.assertEqual(result,2)
            self.assertFalse(out.exists())

    def test_cli_valid_capture_then_reject_overwrite(self):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
            td=Path(td);log=td/'fixture.log';out=td/'capture.json'
            log.write_text('\n'.join(self.fixture)+'\n',encoding='utf-8')
            args=['parse-log',str(log),'--tracker-uid','OPERATOR-129',
                  '--receiver-key','OPERATOR-RX','--note','synthetic test', '--out',str(out)]
            self.assertEqual(wire.main(args),0)
            saved=json.loads(out.read_text(encoding='utf-8'))
            self.assertEqual(saved['target']['reference_id'],129)
            self.assertEqual(wire.main(args),2)

    def test_bad_generation_and_oversized_line_rejected(self):
        data=self.fixture.copy()
        i=next(i for i,x in enumerate(data) if x.startswith('ARC8X1 BEGIN tcal'))
        parts=data[i].split();parts[6]='0';data[i]=' '.join(parts)
        with self.assertRaisesRegex(ValueError,'out-of-range'):
            wire.decode_lines(data)
        with self.assertRaisesRegex(ValueError,'oversized'):
            wire.decode_lines(['x'*321]+self.fixture)

if __name__=='__main__': unittest.main()
