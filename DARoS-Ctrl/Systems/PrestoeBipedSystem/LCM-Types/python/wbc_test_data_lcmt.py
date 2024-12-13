"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class wbc_test_data_lcmt(object):
    __slots__ = ["contact_est", "Fr_des", "Fr", "body_ori_cmd", "body_pos_cmd", "body_vel_cmd", "body_ang_vel_cmd", "body_rpy_cmd", "com_pos_cmd", "vG_cmd", "vGdot_cmd", "h_cmd", "body_pos", "body_vel", "body_ori", "body_ang_vel", "body_rpy", "com_pos", "vG", "vGdot", "h", "foot_pos_cmd", "foot_vel_cmd", "foot_acc_cmd", "foot_acc_numeric", "foot_pos", "foot_vel", "foot_local_pos", "foot_local_vel", "jpos_cmd", "jvel_cmd", "jacc_cmd", "jpos", "jvel", "vision_loc"]

    __typenames__ = ["int32_t", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "float"]

    __dimensions__ = [[4], [12], [12], [4], [3], [3], [3], [3], [3], [6], [6], [3], [3], [3], [4], [3], [3], [3], [6], [6], [3], [12], [12], [12], [12], [12], [12], [12], [12], [16], [16], [16], [16], [16], [3]]

    def __init__(self):
        self.contact_est = [ 0 for dim0 in range(4) ]
        self.Fr_des = [ 0.0 for dim0 in range(12) ]
        self.Fr = [ 0.0 for dim0 in range(12) ]
        self.body_ori_cmd = [ 0.0 for dim0 in range(4) ]
        self.body_pos_cmd = [ 0.0 for dim0 in range(3) ]
        self.body_vel_cmd = [ 0.0 for dim0 in range(3) ]
        self.body_ang_vel_cmd = [ 0.0 for dim0 in range(3) ]
        self.body_rpy_cmd = [ 0.0 for dim0 in range(3) ]
        self.com_pos_cmd = [ 0.0 for dim0 in range(3) ]
        self.vG_cmd = [ 0.0 for dim0 in range(6) ]
        self.vGdot_cmd = [ 0.0 for dim0 in range(6) ]
        self.h_cmd = [ 0.0 for dim0 in range(3) ]
        self.body_pos = [ 0.0 for dim0 in range(3) ]
        self.body_vel = [ 0.0 for dim0 in range(3) ]
        self.body_ori = [ 0.0 for dim0 in range(4) ]
        self.body_ang_vel = [ 0.0 for dim0 in range(3) ]
        self.body_rpy = [ 0.0 for dim0 in range(3) ]
        self.com_pos = [ 0.0 for dim0 in range(3) ]
        self.vG = [ 0.0 for dim0 in range(6) ]
        self.vGdot = [ 0.0 for dim0 in range(6) ]
        self.h = [ 0.0 for dim0 in range(3) ]
        self.foot_pos_cmd = [ 0.0 for dim0 in range(12) ]
        self.foot_vel_cmd = [ 0.0 for dim0 in range(12) ]
        self.foot_acc_cmd = [ 0.0 for dim0 in range(12) ]
        self.foot_acc_numeric = [ 0.0 for dim0 in range(12) ]
        self.foot_pos = [ 0.0 for dim0 in range(12) ]
        self.foot_vel = [ 0.0 for dim0 in range(12) ]
        self.foot_local_pos = [ 0.0 for dim0 in range(12) ]
        self.foot_local_vel = [ 0.0 for dim0 in range(12) ]
        self.jpos_cmd = [ 0.0 for dim0 in range(16) ]
        self.jvel_cmd = [ 0.0 for dim0 in range(16) ]
        self.jacc_cmd = [ 0.0 for dim0 in range(16) ]
        self.jpos = [ 0.0 for dim0 in range(16) ]
        self.jvel = [ 0.0 for dim0 in range(16) ]
        self.vision_loc = [ 0.0 for dim0 in range(3) ]

    def encode(self):
        buf = BytesIO()
        buf.write(wbc_test_data_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>4i', *self.contact_est[:4]))
        buf.write(struct.pack('>12f', *self.Fr_des[:12]))
        buf.write(struct.pack('>12f', *self.Fr[:12]))
        buf.write(struct.pack('>4f', *self.body_ori_cmd[:4]))
        buf.write(struct.pack('>3f', *self.body_pos_cmd[:3]))
        buf.write(struct.pack('>3f', *self.body_vel_cmd[:3]))
        buf.write(struct.pack('>3f', *self.body_ang_vel_cmd[:3]))
        buf.write(struct.pack('>3f', *self.body_rpy_cmd[:3]))
        buf.write(struct.pack('>3f', *self.com_pos_cmd[:3]))
        buf.write(struct.pack('>6f', *self.vG_cmd[:6]))
        buf.write(struct.pack('>6f', *self.vGdot_cmd[:6]))
        buf.write(struct.pack('>3f', *self.h_cmd[:3]))
        buf.write(struct.pack('>3f', *self.body_pos[:3]))
        buf.write(struct.pack('>3f', *self.body_vel[:3]))
        buf.write(struct.pack('>4f', *self.body_ori[:4]))
        buf.write(struct.pack('>3f', *self.body_ang_vel[:3]))
        buf.write(struct.pack('>3f', *self.body_rpy[:3]))
        buf.write(struct.pack('>3f', *self.com_pos[:3]))
        buf.write(struct.pack('>6f', *self.vG[:6]))
        buf.write(struct.pack('>6f', *self.vGdot[:6]))
        buf.write(struct.pack('>3f', *self.h[:3]))
        buf.write(struct.pack('>12f', *self.foot_pos_cmd[:12]))
        buf.write(struct.pack('>12f', *self.foot_vel_cmd[:12]))
        buf.write(struct.pack('>12f', *self.foot_acc_cmd[:12]))
        buf.write(struct.pack('>12f', *self.foot_acc_numeric[:12]))
        buf.write(struct.pack('>12f', *self.foot_pos[:12]))
        buf.write(struct.pack('>12f', *self.foot_vel[:12]))
        buf.write(struct.pack('>12f', *self.foot_local_pos[:12]))
        buf.write(struct.pack('>12f', *self.foot_local_vel[:12]))
        buf.write(struct.pack('>16f', *self.jpos_cmd[:16]))
        buf.write(struct.pack('>16f', *self.jvel_cmd[:16]))
        buf.write(struct.pack('>16f', *self.jacc_cmd[:16]))
        buf.write(struct.pack('>16f', *self.jpos[:16]))
        buf.write(struct.pack('>16f', *self.jvel[:16]))
        buf.write(struct.pack('>3f', *self.vision_loc[:3]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != wbc_test_data_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return wbc_test_data_lcmt._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = wbc_test_data_lcmt()
        self.contact_est = struct.unpack('>4i', buf.read(16))
        self.Fr_des = struct.unpack('>12f', buf.read(48))
        self.Fr = struct.unpack('>12f', buf.read(48))
        self.body_ori_cmd = struct.unpack('>4f', buf.read(16))
        self.body_pos_cmd = struct.unpack('>3f', buf.read(12))
        self.body_vel_cmd = struct.unpack('>3f', buf.read(12))
        self.body_ang_vel_cmd = struct.unpack('>3f', buf.read(12))
        self.body_rpy_cmd = struct.unpack('>3f', buf.read(12))
        self.com_pos_cmd = struct.unpack('>3f', buf.read(12))
        self.vG_cmd = struct.unpack('>6f', buf.read(24))
        self.vGdot_cmd = struct.unpack('>6f', buf.read(24))
        self.h_cmd = struct.unpack('>3f', buf.read(12))
        self.body_pos = struct.unpack('>3f', buf.read(12))
        self.body_vel = struct.unpack('>3f', buf.read(12))
        self.body_ori = struct.unpack('>4f', buf.read(16))
        self.body_ang_vel = struct.unpack('>3f', buf.read(12))
        self.body_rpy = struct.unpack('>3f', buf.read(12))
        self.com_pos = struct.unpack('>3f', buf.read(12))
        self.vG = struct.unpack('>6f', buf.read(24))
        self.vGdot = struct.unpack('>6f', buf.read(24))
        self.h = struct.unpack('>3f', buf.read(12))
        self.foot_pos_cmd = struct.unpack('>12f', buf.read(48))
        self.foot_vel_cmd = struct.unpack('>12f', buf.read(48))
        self.foot_acc_cmd = struct.unpack('>12f', buf.read(48))
        self.foot_acc_numeric = struct.unpack('>12f', buf.read(48))
        self.foot_pos = struct.unpack('>12f', buf.read(48))
        self.foot_vel = struct.unpack('>12f', buf.read(48))
        self.foot_local_pos = struct.unpack('>12f', buf.read(48))
        self.foot_local_vel = struct.unpack('>12f', buf.read(48))
        self.jpos_cmd = struct.unpack('>16f', buf.read(64))
        self.jvel_cmd = struct.unpack('>16f', buf.read(64))
        self.jacc_cmd = struct.unpack('>16f', buf.read(64))
        self.jpos = struct.unpack('>16f', buf.read(64))
        self.jvel = struct.unpack('>16f', buf.read(64))
        self.vision_loc = struct.unpack('>3f', buf.read(12))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if wbc_test_data_lcmt in parents: return 0
        tmphash = (0x6f239618628e6816) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if wbc_test_data_lcmt._packed_fingerprint is None:
            wbc_test_data_lcmt._packed_fingerprint = struct.pack(">Q", wbc_test_data_lcmt._get_hash_recursive([]))
        return wbc_test_data_lcmt._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", wbc_test_data_lcmt._get_packed_fingerprint())[0]
