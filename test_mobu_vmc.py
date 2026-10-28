import math
import struct

import mobu_vmc as m

I = (0, 0, 0, 1)
h = math.sqrt(0.5)
turn90 = (0, h, 0, h)  # 90 degrees about Y


def close(a, b):
    return all(abs(x - y) < 1e-5 for x, y in zip(a, b))


def decode(msg):
    addr = msg[:msg.index(b"\0")].decode()
    i = (len(addr) + 4) & ~3
    tags = msg[i:msg.index(b"\0", i)].decode()
    i += (len(tags) + 4) & ~3
    args = []
    for t in tags[1:]:
        if t == "s":
            e = msg.index(b"\0", i)
            args.append(msg[i:e].decode())
            i += (e - i + 4) & ~3
        else:
            args.append(struct.unpack(">" + t, msg[i:i + 4])[0])
            i += 4
    return addr, tags, args


def bones(msgs):
    return {a[0]: a[1:] for ad, _, a in map(decode, msgs) if ad == "/VMC/Ext/Bone/Pos"}


# stance: every bone 10 cm above its parent, hips at (5, 100, 20) cm
stance = {}
for b, _, p in m.BONES:
    pos = (5, 100, 20) if p is None else (stance[p][1][0], stance[p][1][1] + 10, stance[p][1][2])
    stance[b] = (I, pos)

# identity pose: identity locals, hips in metres with x flipped, 10 cm offsets
now = dict(stance)
msgs = m.frame_messages(now, stance, 1.5)
out = bones(msgs)
assert len(out) == 21
assert close(out["Hips"], (-0.05, 1.0, 0.2, 0, 0, 0, 1))
assert close(out["Spine"], (0, 0.1, 0, 0, 0, 0, 1))
assert all(close(v[3:], (0, 0, 0, 1)) for v in out.values())
assert decode(msgs[0])[2] == ["root", 0, 0, 0, 0, 0, 0, 1]
assert decode(msgs[-2]) == ("/VMC/Ext/OK", ",i", [1])
assert abs(decode(msgs[-1])[2][0] - 1.5) < 1e-6

# parent turned 90 degrees, child turned with it: child local is identity
now = dict(stance)
now["Hips"] = (turn90, stance["Hips"][1])
now["Spine"] = (turn90, stance["Spine"][1])
out = bones(m.frame_messages(now, stance, 0))
assert close(out["Hips"][3:], m.unity_rot(turn90))
assert close(out["Spine"][3:], (0, 0, 0, 1))

# OSC bytes
bundle = m.osc_bundle(msgs)
assert bundle.startswith(b"#bundle\0") and len(bundle) % 4 == 0
assert all(len(x) % 4 == 0 for x in msgs)
assert b",sfffffff" in bundle
print("ok")
