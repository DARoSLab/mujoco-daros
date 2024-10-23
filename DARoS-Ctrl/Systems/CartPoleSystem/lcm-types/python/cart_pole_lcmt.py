"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class cart_pole_lcmt(object):
    __slots__ = ["link1_pos", "link2_pos"]

    __typenames__ = ["float", "float"]

    __dimensions__ = [[3], [3]]

    def __init__(self):
        self.link1_pos = [ 0.0 for dim0 in range(3) ]
        self.link2_pos = [ 0.0 for dim0 in range(3) ]

    def encode(self):
        buf = BytesIO()
        buf.write(cart_pole_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>3f', *self.link1_pos[:3]))
        buf.write(struct.pack('>3f', *self.link2_pos[:3]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != cart_pole_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return cart_pole_lcmt._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = cart_pole_lcmt()
        self.link1_pos = struct.unpack('>3f', buf.read(12))
        self.link2_pos = struct.unpack('>3f', buf.read(12))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if cart_pole_lcmt in parents: return 0
        tmphash = (0xd4aaa707e5c69bc8) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if cart_pole_lcmt._packed_fingerprint is None:
            cart_pole_lcmt._packed_fingerprint = struct.pack(">Q", cart_pole_lcmt._get_hash_recursive([]))
        return cart_pole_lcmt._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", cart_pole_lcmt._get_packed_fingerprint())[0]

