# mobu-vmc

Streams MotionBuilder Actors to a VMC receiver over UDP.
Each Actor sends one VMC frame per MotionBuilder frame: Root, 21 body bones (no fingers), OK and time.

MotionBuilder 2026 only (Python 3.11).

## Usage

In the MotionBuilder Python Editor:

```python
import sys; sys.path.append(r"<path to repo>")
import mobu_vmc
mobu_vmc.start([("Actor", "127.0.0.1", 39540)])
mobu_vmc.stop()
```

Each entry is `(actor name, host, port)`. Use one port per Actor.
Avoid port 39539, the Rokoko Studio default.

## Receiver notes

- Root stays at the origin. The Hips bone carries the world position.
- The Actor stance must be a T-pose. The stance is read once at `start()` and used as the rest pose.
- Units are converted from centimetres to metres, and from right-handed to left-handed axes.

## Test

```
python test_mobu_vmc.py
```

## License

MIT
