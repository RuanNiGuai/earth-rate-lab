"""Real cryptographic packaging tests; needs the frozen SDK's imgtool."""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import ec

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from firmware_images import verify_signature


class SignatureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        base = ROOT/'.local/native-tests'
        base.mkdir(parents=True, exist_ok=True)
        cls.temp = tempfile.TemporaryDirectory(dir=base)
        cls.addClassCleanup(cls.temp.cleanup)
        cls.directory = Path(cls.temp.name)
        key = ec.generate_private_key(ec.SECP256R1())
        cls.private = cls.directory/'ephemeral-private.pem'
        cls.private.write_bytes(key.private_bytes(serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8, serialization.NoEncryption()))
        cls.public = cls.directory/'public.pem'
        cls.public.write_bytes(key.public_key().public_bytes(serialization.Encoding.PEM,
            serialization.PublicFormat.SubjectPublicKeyInfo))
        cls.wrong = cls.directory/'wrong-public.pem'
        cls.wrong.write_bytes(ec.generate_private_key(ec.SECP256R1()).public_key().public_bytes(
            serialization.Encoding.PEM, serialization.PublicFormat.SubjectPublicKeyInfo))
        payload = cls.directory/'payload.bin'
        payload.write_bytes(bytes(range(256))*4)
        cls.binary = cls.directory/'signed.bin'
        subprocess.run([sys.executable, str(IMGTOOL), 'sign', '-k', str(cls.private),
            '--version', '1.0.0', '--header-size', '0x200', '--pad-header', '--align', '4',
            '--slot-size', '0x6f000', str(payload), str(cls.binary)], check=True, capture_output=True)

    def test_public_key_verifies(self):
        result = verify_signature(IMGTOOL, self.public, self.binary)
        self.assertEqual(result['signature'], 'ECDSA-P256-SHA256')

    def test_private_key_refused(self):
        with self.assertRaisesRegex(ValueError, 'public-only'):
            verify_signature(IMGTOOL, self.private, self.binary)

    def test_wrong_key_rejected(self):
        with self.assertRaisesRegex(ValueError, 'verification failed'):
            verify_signature(IMGTOOL, self.wrong, self.binary)

    def check_tamper(self, offset):
        damaged = bytearray(self.binary.read_bytes())
        damaged[offset] ^= 1
        path = self.directory/'damaged.bin'
        path.write_bytes(damaged)
        with self.assertRaisesRegex(ValueError, 'verification failed'):
            verify_signature(IMGTOOL, self.public, path)

    def test_modified_payload_rejected(self):
        self.check_tamper(0x200+3)

    def test_modified_signature_rejected(self):
        self.check_tamper(-1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--imgtool', type=Path, required=True)
    IMGTOOL = parser.parse_args().imgtool.resolve()
    unittest.main(argv=[sys.argv[0]], verbosity=2)
