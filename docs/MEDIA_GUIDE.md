# Media guide: screenshots and animation videos

A visual repo is far easier to read. Capture these in order. Everything takes about an hour.

## Folders

| Type | Put in | Format |
|---|---|---|
| Screenshots, diagrams | `docs/images/` | PNG |
| Animations | `docs/videos/` | MP4 (full quality) **and** GIF (for the README) |

## Shot list

### A. Simulink screenshots (3)

1. **`simulink_model.png`**: open `F104_FC1_sim.slx`, press *Ctrl+Shift+F* (fit to view), and capture the whole diagram. Make sure signal names and the State-Space block are readable.
2. **`simulink_scopes_FC1.png`**: run the FC1 free response and capture the scopes (β, p, r, φ). The growing Dutch roll oscillation is the point.
3. **`animation_still_FC1.png`**: pause the 3D animation at about t = 8 s, when the bank angle is large, and capture the window.

### B. Animation videos (3 required, 1 optional)

Record each at 1080p, 20 s of simulation (the simulation time in the problem statement), screen only.

| Video | Condition and input | What to look for |
|---|---|---|
| `FC1_free_response` | FC1, β(0) = 2°, no input | slowly *growing* wag and wing rock (unstable Dutch roll) |
| `FC2_aileron_pulse` | FC2, 2° aileron for 1 s | quick roll, rate dies out, bank angle stays (roll subsidence) |
| `FC3_rudder_pulse` | FC3, 2° rudder for 1 s | yaw with induced roll (cross-coupling), well-damped but oscillatory |
| `comparison_side_by_side` (optional) | all three, same input | the speed-dependent change in frequency and damping |

**Tips for a clean recording**

* Close other windows and hide the Simulink toolbar clutter; show only the 3D view.
* Start recording first, then press Run; stop right when the simulation ends. Trim the dead time at the start.
* Use the screen recorder built into your OS or OBS Studio. Save as MP4.
* Keep each video under about 30 MB. If larger, lower the resolution or frame rate.

## Making a GIF for inline README display

GitHub shows GIFs inline from the repo, but not MP4 files. With [ffmpeg](https://ffmpeg.org) installed:

```bash
ffmpeg -i docs/videos/FC1_free_response.mp4 \
  -vf "fps=12,scale=720:-1:flags=lanczos" -loop 0 docs/videos/FC1_free_response.gif
```

Aim for GIFs under 10 MB (lower `fps` or `scale` if needed).

## Embedding in the README

```markdown
![FC1 free response: unstable Dutch roll](docs/videos/FC1_free_response.gif)

[Full-quality video](docs/videos/FC1_free_response.mp4)
```

To embed a playable MP4 instead, edit the README on github.com and drag the file into the editor; GitHub uploads it and inserts a link that plays inline.

## GitHub size limits

* Files over 50 MB trigger a warning and files over 100 MB are blocked. Keep videos small.
* If you need larger videos, attach them to a GitHub *Release* or host them elsewhere and link them.

## Optional extra figures

* A short annotated version of `results/figures/python/eigenvalue_map.png` with labels on the Dutch roll pair, roll and spiral roots.
* One frame sequence (4 stills) of the FC1 free response showing the aircraft rocking and the bank angle building.
