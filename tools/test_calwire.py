import struct
import unittest
import zlib
from calwire import packet,response,unpack_snapshot

class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.req=packet(2,'112233445566',123,1,26)
    def reply(self,payload=b'123',status=0):
        p=bytearray(self.req);p[0]=0x27;p[16]=len(payload)+4;p[17]=status
        p[18:22]=self.req[44:48];p[22:22+len(payload)]=payload
        struct.pack_into('<I',p,44,zlib.crc32(p[:44]));return bytes(p)
    def test_valid_and_little_endian_identity(self):
        self.assertEqual(self.req[8:14],bytes.fromhex('665544332211'))
        self.assertEqual(response(self.reply(),self.req),b'123')
    def test_reject_every_corrupted_byte(self):
        for i in range(48):
            p=bytearray(self.reply());p[i]^=1
            with self.assertRaises(ValueError):response(p,self.req)
    def test_reject_stale_identity_even_with_valid_crc(self):
        for i in (1,3,4,8,14):
            p=bytearray(self.reply());p[i]^=1;struct.pack_into('<I',p,44,zlib.crc32(p[:44]))
            with self.assertRaises(ValueError):response(p,self.req)
    def test_nonzero_status_never_success(self):
        for st in range(1,9):
            with self.assertRaises(ValueError):response(self.reply(status=st),self.req)
    def test_request_bounds(self):
        for kwargs in [dict(slot=32),dict(token=0),dict(offset=65536),dict(data=bytes(27))]:
            args=dict(slot=2,address='112233445566',token=1,op=0);args.update(kwargs)
            with self.assertRaises(ValueError):packet(**args)
    def test_truncated_snapshot(self):
        for blob in (b'',b'123',b'\x01\x00\xff\xff'):
            with self.assertRaises(ValueError):unpack_snapshot(blob)

if __name__=='__main__':unittest.main()
