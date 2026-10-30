"""Stream MotionBuilder Actors and Characters to a VMC receiver. MotionBuilder 2026 only."""
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

_state = {"sock": None, "sources": [], "callback": None, "t0": 0.0}


def _pose(m):
    import pyfbsdk as fb
    q = fb.FBVector4d()
    fb.FBMatrixToQuaternion(q, m)
    return (q[0], q[1], q[2], q[3]), (m[12], m[13], m[14])


def _read(get_matrix):
    out = {}
    for bone, node, parent in BONES:
        m = get_matrix(node)
        out[bone] = _pose(m) if m else out[parent]
    return out


def _actor_reader(skel_state):
    import pyfbsdk as fb

    def get(node):
        m = fb.FBMatrix()
        skel_state.GetNodeMatrix(getattr(fb.FBSkeletonNodeId, "kFBSkeleton%sIndex" % node), m)
        return m
    return get


def _character_now(character):
    import pyfbsdk as fb

    def get(node):
        model = character.GetModel(getattr(fb.FBBodyNodeId, "kFB%sNodeId" % node))
        if not model:
            return None
        m = fb.FBMatrix()
        model.GetMatrix(m)
        return m
    return get


def _character_stance(character):
    import pyfbsdk as fb

    def get(node):
        node_id = getattr(fb.FBBodyNodeId, "kFB%sNodeId" % node)
        if not character.GetModel(node_id):
            return None
        m = fb.FBMatrix()
        character.GetTransformOffset(node_id, m)
        return m
    return get


def start(targets):
    import pyfbsdk as fb
    stop()
    scene = fb.FBSystem().Scene
    found = {("Actor", a.Name): a for a in scene.Actors}
    found.update({("Character", c.Name): c for c in scene.Characters})
    sources = []
    for kind, name, host, port in targets:
        source = found.get((kind, name))
        if not source:
            print("mobu_vmc: %s '%s' not found, skipped" % (kind, name))
            continue
        if kind == "Actor":
            now = lambda a=source: _read(_actor_reader(a.GetCurrentSkeletonState()))
            stance = _read(_actor_reader(source.GetDefaultSkeletonState()))
        else:
            now = lambda c=source: _read(_character_now(c))
            stance = _read(_character_stance(source))
        sources.append((now, (host, port), stance))
    _state["sock"] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _state["sources"] = sources
    _state["t0"] = time.time()

    def on_sync(control, event):
        t = time.time() - _state["t0"]
        for read_now, addr, stance in _state["sources"]:
            now = read_now()
            try:
                _state["sock"].sendto(osc_bundle(frame_messages(now, stance, t)), addr)
            except OSError:
                pass

    _state["callback"] = on_sync
    fb.FBEvaluateManager().OnSynchronizationEvent.Add(on_sync)
    print("mobu_vmc: streaming %d source(s)" % len(sources))


def stop():
    if _state["callback"]:
        import pyfbsdk as fb
        fb.FBEvaluateManager().OnSynchronizationEvent.Remove(_state["callback"])
        _state["callback"] = None
    if _state["sock"]:
        _state["sock"].close()
        _state["sock"] = None
    _state["sources"] = []


# --- Panel ---

try:
    import pyfbsdk as fb
    import pyfbsdk_additions as fba
except ImportError:  # plain CPython has no pyfbsdk
    fb = None

TOOL_NAME = "VMC Sender"

if fb:
    def _label(text):
        label = fb.FBLabel()
        label.Caption = text
        return label

    class VmcSenderTool(fb.FBTool):
        def __init__(self):
            fb.FBTool.__init__(self, TOOL_NAME)
            fba.FBAddTool(self)
            self.rows = []

            x = fb.FBAddRegionParam(0, fb.FBAttachType.kFBAttachLeft, "")
            y = fb.FBAddRegionParam(0, fb.FBAttachType.kFBAttachTop, "")
            w = fb.FBAddRegionParam(0, fb.FBAttachType.kFBAttachRight, "")
            h = fb.FBAddRegionParam(0, fb.FBAttachType.kFBAttachBottom, "")
            self.AddRegion("main", "main", x, y, w, h)
            vbox = fba.FBVBoxLayout()
            self.SetControl("main", vbox)

            host_row = fba.FBHBoxLayout()
            host_row.Add(_label("Host"), 40)
            self.host = fb.FBEdit()
            self.host.Text = "127.0.0.1"
            host_row.AddRelative(self.host)
            vbox.Add(host_row, 25)

            self.rows_box = fba.FBVBoxLayout()
            vbox.AddRelative(self.rows_box)

            button_row = fba.FBHBoxLayout()
            refresh = fb.FBButton()
            refresh.Caption = "Refresh"
            refresh.OnClick.Add(self._refresh)
            self.toggle = fb.FBButton()
            self.toggle.OnClick.Add(self._toggle)
            button_row.Add(refresh, 80)
            button_row.AddRelative(None)
            button_row.Add(self.toggle, 80)
            vbox.Add(button_row, 25)

            self.status = _label("")
            vbox.Add(self.status, 25)
            self._sync()

            self.StartSizeX = 300
            self.StartSizeY = 220
            self.OnShow.Add(self._on_show)

        def _on_show(self, control, event):
            if event.Shown:
                if not self.rows:
                    self._refresh()
                self._sync()

        def _refresh(self, control=None, event=None):
            self.rows_box.RemoveAll()
            self.rows = []
            scene = fb.FBSystem().Scene
            sources = [("Actor", a.Name) for a in scene.Actors] + [("Character", c.Name) for c in scene.Characters]
            for i, (kind, name) in enumerate(sources):
                send = fb.FBButton()
                send.Style = fb.FBButtonStyle.kFBCheckbox
                send.Caption = "%s  (%s)" % (name, kind)
                send.State = True
                port = fb.FBEditNumber()
                port.Precision = 0
                port.Min = 1
                port.Max = 65535
                port.Value = 39540 + i
                row = fba.FBHBoxLayout()
                row.AddRelative(send)
                row.Add(_label("port"), 35)
                row.Add(port, 60)
                self.rows_box.Add(row, 25)
                self.rows.append((kind, name, send, port))

        def _toggle(self, control, event):
            if _state["callback"]:
                stop()
            else:
                # resolve once: sendto would look a host name up every frame
                host = socket.gethostbyname(self.host.Text.strip())
                start([(kind, name, host, round(port.Value)) for kind, name, send, port in self.rows if send.State])
            self._sync()

        def _sync(self):
            on = bool(_state["callback"])
            self.toggle.Caption = "Stop" if on else "Start"
            self.status.Caption = "Streaming %d source(s)" % len(_state["sources"]) if on else "Stopped"

    if TOOL_NAME not in fba.FBToolList:
        VmcSenderTool()
