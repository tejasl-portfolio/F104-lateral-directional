"""
Explanatory (teaching) figures used in the README -> docs/images/
  axes_and_states.png     body axes, beta, psi, r, p, phi
  mode_signatures.png     what each mode looks like when excited on its own (FC2)
  derivatives_vs_q.png    how the stability derivatives scale with dynamic pressure
"""
import os
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, FancyArrowPatch, Arc, Circle
from f104_reference import FC, build, RHO, D2R

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "images")
os.makedirs(out, exist_ok=True)


# ---------------------------------------------------------------- 1. axes and states
def arrow(ax, p0, p1, c="k", lw=2, ms=14, rad=0.0):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=ms, lw=lw, color=c,
                                 connectionstyle=f"arc3,rad={rad}"))

fig, (a, b) = plt.subplots(1, 2, figsize=(12, 5.6))
# --- top view
a.set_aspect("equal"); a.axis("off"); a.set_xlim(-3.2, 3.8); a.set_ylim(-3.0, 4.0)
body = Polygon([[0, 3], [.22, 2.2], [.22, -2.3], [-.22, -2.3], [-.22, 2.2]], fc="#c9d3e3", ec="k")
wings = Polygon([[.22, .4], [1.5, .1], [1.5, -.35], [.22, -.5], [-.22, -.5], [-1.5, -.35], [-1.5, .1], [-.22, .4]],
                fc="#c9d3e3", ec="k")
tail = Polygon([[.22, -1.9], [.9, -2.1], [.9, -2.4], [-.9, -2.4], [-.9, -2.1], [-.22, -1.9]], fc="#c9d3e3", ec="k")
for p in (tail, wings, body): a.add_patch(p)
beta = 25 * D2R
arrow(a, (0, 0), (0, 3.4), "tab:blue"); a.text(.12, 3.5, "x (nose)", color="tab:blue", fontsize=11)
arrow(a, (0, 0), (2.6, 0), "tab:blue"); a.text(2.7, -.05, "y (right wing)", color="tab:blue", fontsize=11)
Vx, Vy = 3.4 * np.sin(beta), 3.4 * np.cos(beta)
arrow(a, (0, 0), (Vx, Vy), "tab:red", lw=2.5); a.text(Vx + .1, Vy + .05, "V (flight path)", color="tab:red", fontsize=11)
a.add_patch(Arc((0, 0), 2.4, 2.4, theta1=90 - 25, theta2=90, color="tab:red", lw=2))
a.text(.3, 1.35, r"$\beta$", color="tab:red", fontsize=16)
arrow(a, (1.3, -1.0), (-1.3, -1.0), "tab:green", lw=2.5, rad=-0.55)
a.text(-1.9, -2.75, r"$r$ = yaw rate (tail swings left, nose right)", color="tab:green", fontsize=11)
a.set_title("Top view: sideslip $\\beta$ and yaw rate $r$\n(heading $\\psi$ = integral of $r/\\cos\\theta_e$)", fontsize=12)
# --- rear view
b.set_aspect("equal"); b.axis("off"); b.set_xlim(-3.2, 3.4); b.set_ylim(-2.6, 3.4)
phi = 20 * D2R
c, s = np.cos(phi), np.sin(phi)
rot = lambda x, y: (x * c + y * s, -x * s + y * c)   # +phi: right wing (+y) goes down, fin leans right
b.plot([-2.8, 2.8], [0, 0], "k--", lw=1); b.text(-2.8, .12, "horizon", fontsize=10)
wl, wr = rot(-2.3, 0), rot(2.3, 0); ft = rot(0, 1.6)
b.plot([wl[0], wr[0]], [wl[1], wr[1]], color="#34507a", lw=6, solid_capstyle="round", zorder=1)
b.plot([0, ft[0]], [0, ft[1]], color="#34507a", lw=5, solid_capstyle="round", zorder=1)
b.add_patch(Circle((0, 0), .32, fc="#c9d3e3", ec="k", zorder=3))
arrow(b, (0.35, 0), (2.9, 0), "tab:blue"); b.text(2.95, -.1, "y", color="tab:blue", fontsize=11)
arrow(b, (0, -.35), (0, -2.1), "tab:blue"); b.text(.12, -2.3, "z (down)", color="tab:blue", fontsize=11)
b.add_patch(Arc((0, 0), 4.2, 4.2, theta1=-20, theta2=0, color="tab:red", lw=2.5))
b.text(2.25, -.85, r"$\phi$ (bank)", color="tab:red", fontsize=14)
arrow(b, (-1.1, 2.3), (1.1, 2.3), "tab:green", lw=2.5, rad=-0.55)
b.text(-1.55, 3.0, r"$p$ = roll rate (right wing down)", color="tab:green", fontsize=11)
b.set_title("Rear view: bank angle $\\phi$ and roll rate $p$", fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(out, "axes_and_states.png"), dpi=140); plt.close(fig)

# ---------------------------------------------------------------- 2. mode signatures (FC2)
A, B, Ve = build(FC["FC2"])
w, V = np.linalg.eig(A)
iD = np.argmax(w.imag); iR = np.argmin(np.where(abs(w.imag) < 1e-9, w.real, 0)); iS = [i for i in range(4) if abs(w[i].imag) < 1e-9 and i != iR][0]
sys0 = signal.StateSpace(A, B, np.eye(4), np.zeros((4, 2)))
def pure(i, t, real_only=True):
    x0 = np.real(V[:, i]) if abs(w[i].imag) < 1e-9 else np.real(V[:, i])
    x0 = x0 / np.max(np.abs(x0)) * 5 * D2R
    _, y, _ = signal.lsim(sys0, np.zeros((len(t), 2)), t, X0=x0)
    return y / D2R
labels = [r"$\beta$ (deg)", r"$p$ (deg/s)", r"$r$ (deg/s)", r"$\phi$ (deg)"]
cols = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
fig, axs = plt.subplots(1, 3, figsize=(14, 4.4))
for ax, i, T, ttl in [(axs[0], iD, np.linspace(0, 6, 1200), "Dutch roll  (complex pair)\nyaw / sideslip wag, wings rock"),
                      (axs[1], iR, np.linspace(0, 1.2, 600), "Roll subsidence  (fast real root)\nroll rate dies out in ~tau"),
                      (axs[2], iS, np.linspace(0, 300, 1500), "Spiral  (slow real root)\nbank angle drifts over minutes")]:
    y = pure(i, T)
    for k in range(4): ax.plot(T, y[:, k], color=cols[k], label=labels[k])
    ax.set_title(ttl, fontsize=11); ax.set_xlabel("t (s)"); ax.grid(alpha=.3)
axs[0].legend(ncol=2, fontsize=9)
fig.suptitle("Each mode excited on its own (initial state = that mode's eigenvector), FC2 (M = 0.8)", y=1.02)
fig.tight_layout(); fig.savefig(os.path.join(out, "mode_signatures.png"), dpi=140, bbox_inches="tight"); plt.close(fig)

# ---------------------------------------------------------------- 3. derivatives vs q
qs = [0.5 * RHO * (fc["M"] * 1116.44) ** 2 for fc in FC.values()]
ders = {"l_beta": "lb", "n_beta": "nb", "l_p": "lp", "n_r": "nr", "l_xi (aileron)": "lxi", "l_zeta (rudder)": "lze"}
fig, ax = plt.subplots(figsize=(7.5, 5))
for lab, key in ders.items():
    ax.loglog(qs, [abs(fc[key]) for fc in FC.values()], "o-", label=f"|{lab}|")
ax.loglog(qs, [q / qs[0] * 4 for q in qs], "k:", label="slope 1 (proportional to q)")
ax.set_xlabel("dynamic pressure q (lbf/ft$^2$)"); ax.set_ylabel("|derivative|")
ax.set_title("Most derivatives grow roughly in proportion to q\n(FC1 -> FC3: q is 18x larger)")
ax.grid(alpha=.3, which="both"); ax.legend(fontsize=8)
fig.tight_layout(); fig.savefig(os.path.join(out, "derivatives_vs_q.png"), dpi=140); plt.close(fig)
print("explainer figures written to", out)
