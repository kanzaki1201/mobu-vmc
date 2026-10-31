# mobu-vmc

Streams MotionBuilder Actors and Characters to a VMC receiver over UDP.
Each source sends one VMC frame per MotionBuilder frame: Root, 21 body bones (no fingers), OK and time.

MotionBuilder 2026 only (Python 3.11).

## Install

Copy `mobu_vmc.py` to `%USERPROFILE%\Documents\MB\2026\config\PythonStartup`.
Restart MotionBuilder and open Python Tools > VMC Sender.

Set the receiver host, tick the Actors and Characters to send, set their ports (39540 and up), and press Start.

## Console

In the MotionBuilder Python Editor:

```python
import sys; sys.path.append(r"<path to repo>")
import mobu_vmc
mobu_vmc.start([("Actor", "Actor", "127.0.0.1", 39540), ("Character", "Character", "127.0.0.1", 39541)])
mobu_vmc.stop()
```

Each entry is `("Actor" or "Character", name, host, port)`. Use one port per source.

## Receiver notes

- Root stays at the origin. The Hips bone carries the world position.
- The Actor stance or the Character characterization pose must be a T-pose. It is read once at `start()` and used as the rest pose.
- A Character bone with no model takes its parent's pose.
- Units are converted from centimetres to metres, and from right-handed to left-handed axes.

## Test

```
python test_mobu_vmc.py
```

## License

MIT
