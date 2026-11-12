"""Package a verified application image. No private key or legacy rollback path is needed."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
from firmware_images import uf2_encode, uf2_validate, receiver_validate, verify_signature


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build-dir', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--imgtool', type=Path)
    p.add_argument('--public-key', type=Path)
    a = p.parse_args()
    identity = json.loads((a.build_dir/'source-identity.json').read_text())
    files = {}
    if identity['target'] == 'tracker':
        if a.public_key is None or a.imgtool is None:
            raise ValueError('Tracker packaging requires --public-key and --imgtool')
        if b'PRIVATE KEY' in a.public_key.read_bytes():
            raise ValueError('Verification requires a public-only key')
        binary = a.build_dir/'zephyr/zephyr.signed.bin'
        identity['signature_verification'] = verify_signature(a.imgtool, a.public_key, binary)
        files['tracker.signed.bin'] = binary.read_bytes()
        files['tracker.signed.uf2'] = uf2_encode(files['tracker.signed.bin'])
        uf2_validate(files['tracker.signed.uf2'], files['tracker.signed.bin'])
        files['signing-public.pem'] = a.public_key.read_bytes()
    else:
        z = a.build_dir/'receiver/zephyr'
        files['receiver.uf2'] = (z/'zephyr.uf2').read_bytes()
        receiver_validate(files['receiver.uf2'], z/'zephyr.hex')
        files['receiver.hex'] = (z/'zephyr.hex').read_bytes()
    a.out.mkdir(parents=True, exist_ok=False)
    for name, data in files.items():
        (a.out/name).write_bytes(data)
    identity['files'] = {k:dict(bytes=len(v), sha256=hashlib.sha256(v).hexdigest()) for k,v in files.items()}
    identity['deployed'] = False
    (a.out/'manifest.json').write_text(json.dumps(identity, indent=2), encoding='utf8')


if __name__ == '__main__':
    main()
