"""Stream MotionBuilder Actors to a VMC receiver. MotionBuilder 2026 only."""
import socket
import struct
import time

# (VMC bone, FBSkeletonNodeId name, parent VMC bone)
BONES = [("Hips", "Hips", None)]
for _s in ("Left", "Right"):
    BONES += [
        (_s + "UpperLeg", _s + "Hip", "Hips"),
        (_s + "LowerLeg", _s + "Knee", _s + "UpperLeg"),
        (_s + "Foot", _s + "Ankle", _s + "LowerLeg"),
        (_s + "Toes", _s + "Foot", _s + "Foot"),
    ]
BONES += [("Spine", "Waist", "Hips"), ("Chest", "Chest", "Spine")]
for _s in ("Left", "Right"):
    BONES += [
        (_s + "Shoulder", _s + "Collar", "Chest"),
        (_s + "UpperArm", _s + "Shoulder", _s + "Shoulder"),
        (_s + "LowerArm", _s + "Elbow", _s + "UpperArm"),
        (_s + "Hand", _s + "Wrist", _s + "LowerArm"),
    ]
BONES += [("Neck", "Neck", "Chest"), ("Head", "Head", "Neck")]


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def unity_pos(p):
    return (-p[0] / 100, p[1] / 100, p[2] / 100)


def unity_rot(q):
    return (q[0], -q[1], -q[2], q[3])


def _pad(b):
    return b + b"\0" * (4 - len(b) % 4)


def osc_message(address, tags, *args):
    out = _pad(address.encode()) + _pad(b"," + tags.encode())
    for t, a in zip(tags, args):
        if t == "s":
            out += _pad(a.encode())
        elif t == "f":
            out += struct.pack(">f", a)
        elif t == "i":
            out += struct.pack(">i", a)
    return out


def osc_bundle(messages):
    out = b"#bundle\0" + struct.pack(">Q", 1)
    for m in messages:
        out += struct.pack(">i", len(m)) + m
    return out


def frame_messages(now, stance, t):
    """now, stance: {bone: (global_quat_mobu, global_pos_mobu_cm)}. Returns OSC messages."""
    delta = {b: qmul(now[b][0], qconj(stance[b][0])) for b in now}
    msgs = [osc_message("/VMC/Ext/Root/Pos", "sfffffff", "root", 0, 0, 0, 0, 0, 0, 1)]
    for bone, _, parent in BONES:
        if parent is None:
            local = delta[bone]
            pos = unity_pos(now[bone][1])
        else:
            local = qmul(qconj(delta[parent]), delta[bone])
            pos = unity_pos(tuple(c - p for c, p in zip(stance[bone][1], stance[parent][1])))
        msgs.append(osc_message("/VMC/Ext/Bone/Pos", "sfffffff", bone, *pos, *unity_rot(local)))
    msgs.append(osc_message("/VMC/Ext/OK", "i", 1))
    msgs.append(osc_message("/VMC/Ext/T", "f", t))
    return msgs


# --- MotionBuilder glue ---

_state = {"sock": None, "actors": [], "callback": None, "t0": 0.0}


def _read(skel_state):
    import pyfbsdk as fb
    out = {}
    for bone, node, _ in BONES:
        m = fb.FBMatrix()
        skel_state.GetNodeMatrix(getattr(fb.FBSkeletonNodeId, "kFBSkeleton%sIndex" % node), m)
        q = fb.FBVector4d()
        fb.FBMatrixToQuaternion(q, m)
        out[bone] = ((q[0], q[1], q[2], q[3]), (m[12], m[13], m[14]))
    return out


def start(targets):
    import pyfbsdk as fb
    stop()
    found = {a.Name: a for a in fb.FBSystem().Scene.Actors}
    actors = []
    for name, host, port in targets:
        if name not in found:
            print("mobu_vmc: Actor '%s' not found, skipped" % name)
            continue
        actors.append((found[name], (host, port), _read(found[name].GetDefaultSkeletonState())))
    _state["sock"] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _state["actors"] = actors
    _state["t0"] = time.time()

    def on_sync(control, event):
        t = time.time() - _state["t0"]
        for actor, addr, stance in _state["actors"]:
            now = _read(actor.GetCurrentSkeletonState())
            try:
                _state["sock"].sendto(osc_bundle(frame_messages(now, stance, t)), addr)
            except OSError:
                pass

    _state["callback"] = on_sync
    fb.FBEvaluateManager().OnSynchronizationEvent.Add(on_sync)
    print("mobu_vmc: streaming %d Actor(s)" % len(actors))


def stop():
    if _state["callback"]:
        import pyfbsdk as fb
        fb.FBEvaluateManager().OnSynchronizationEvent.Remove(_state["callback"])
        _state["callback"] = None
    if _state["sock"]:
        _state["sock"].close()
        _state["sock"] = None
    _state["actors"] = []
