"""Application-only image encoding and address/byte verification."""
import struct
from pathlib import Path
import sys
from intelhex import IntelHex
TRACKER_BASE=0xF000
TRACKER_LIMIT=0x7E000
RECEIVER_BASE=0x1000
RECEIVER_LIMIT=0xEE000
FAMILY=0xADA52840


def verify_signature(imgtool_path, public_key, binary):
    """Use the frozen MCUboot verifier with its missing P-256 public-key entry.

    The pinned imgtool accepts ECDSA256P1 but omits ECDSA256P1Public from
    ALLOWED_KEY_SHA. Supply exactly that SHA-256 mapping; hash and signature
    verification still run in upstream Image.verify. Never load a private key.
    """
    public_key = Path(public_key)
    if b'PRIVATE KEY' in public_key.read_bytes():
        raise ValueError('Verification requires a public-only key')
    sys.path.insert(0, str(Path(imgtool_path).resolve().parent))
    from imgtool import image, keys
    key = keys.load(str(public_key))
    if type(key) is not keys.ECDSA256P1Public:
        raise ValueError('This board requires an ECDSA P-256 public key')
    if hasattr(image, 'ALLOWED_KEY_SHA'):
        image.ALLOWED_KEY_SHA.setdefault(keys.ECDSA256P1Public, ['256'])
    result, version, digest, _ = image.Image.verify(str(binary), key)
    if result != image.VerifyResult.OK or digest is None:
        raise ValueError('MCUboot signature verification failed: '+str(result))
    return dict(version=list(version), image_digest=digest.hex(), signature='ECDSA-P256-SHA256')


def uf2_encode(binary):
    if binary[:4] != struct.pack('<I', 0x96F3B83D):
        raise ValueError('missing MCUboot header; unsigned input refused')
    count = (len(binary) + 255) // 256
    if not count or TRACKER_BASE + count * 256 > TRACKER_LIMIT:
        raise ValueError('image exceeds primary slot')
    out = bytearray()
    for number in range(count):
        piece = binary[number*256:(number+1)*256]
        out.extend(struct.pack('<8I', 0x0A324655, 0x9E5D5157, 0x2000,
                               TRACKER_BASE + number*256, 256, number, count, FAMILY))
        out.extend(piece.ljust(256, b'\xff').ljust(476, b'\0'))
        out.extend(struct.pack('<I', 0x0AB16F30))
    return bytes(out)


def uf2_validate(data, binary):
    if not data or len(data) % 512:
        raise ValueError('bad UF2 length')
    blocks = len(data) // 512
    if blocks != (len(binary) + 255) // 256:
        raise ValueError('UF2 block count does not match image size')
    recovered = bytearray()
    for i in range(blocks):
        block = data[i*512:(i+1)*512]
        expected = (0x0A324655, 0x9E5D5157, 0x2000, TRACKER_BASE+i*256, 256, i, blocks, FAMILY)
        if struct.unpack_from('<8I', block) != expected:
            raise ValueError('UF2 block metadata/address mismatch')
        if struct.unpack_from('<I', block, 508)[0] != 0x0AB16F30:
            raise ValueError('UF2 end marker mismatch')
        recovered.extend(block[32:288])
    if TRACKER_BASE + len(recovered) > TRACKER_LIMIT or bytes(recovered) != binary.ljust(blocks*256, b'\xff'):
        raise ValueError('UF2 does not exactly match signed primary-slot image')


def receiver_validate(u, hexfile):
    h=IntelHex(str(hexfile))
    if h.minaddr()!=RECEIVER_BASE or h.maxaddr()>=RECEIVER_LIMIT or not u or len(u)%512:
        raise ValueError('receiver application address/size invalid')
    count=len(u)//512
    if count!=(h.maxaddr()-RECEIVER_BASE+256)//256: raise ValueError('receiver UF2 block count mismatch')
    addresses=set(h.addresses())
    for i in range(count):
        block=u[i*512:(i+1)*512]
        expected=(0x0a324655,0x9e5d5157,0x2000,RECEIVER_BASE+256*i,256,i,count,FAMILY)
        if struct.unpack_from('<8I',block)!=expected or struct.unpack_from('<I',block,508)[0]!=0x0ab16f30:
            raise ValueError('receiver UF2 block metadata mismatch')
        if RECEIVER_BASE+256*(i+1)>RECEIVER_LIMIT: raise ValueError('receiver UF2 overlaps NVS/bootloader')
        for j,b in enumerate(block[32:288]):
            addr=RECEIVER_BASE+256*i+j
            if b!=(h[addr] if addr in addresses else 0):
                raise ValueError('receiver UF2 differs from linked HEX or its zero-filled gaps')


