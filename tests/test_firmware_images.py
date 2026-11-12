import struct
import sys
from pathlib import Path
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from firmware_images import uf2_encode,uf2_validate,TRACKER_BASE,TRACKER_LIMIT


class ImageTests(unittest.TestCase):
    def test_application_round_trip_and_address_tampering(self):
        binary=struct.pack('<I',0x96F3B83D)+bytes(range(256))*3
        uf2=uf2_encode(binary);uf2_validate(uf2,binary)
        damaged=bytearray(uf2);struct.pack_into('<I',damaged,12,0xEE000)
        with self.assertRaises(ValueError):uf2_validate(bytes(damaged),binary)
        damaged=bytearray(uf2);damaged[32]^=1
        with self.assertRaises(ValueError):uf2_validate(bytes(damaged),binary)

    def test_unsigned_and_oversized_payload_rejected(self):
        for binary in [b'unsigned',struct.pack('<I',0x96F3B83D)+bytes(TRACKER_LIMIT-TRACKER_BASE)]:
            with self.assertRaises(ValueError):uf2_encode(binary)
