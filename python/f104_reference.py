"""
F-104A lateral-directional dynamics, Team 15 (three sea-level flight conditions).

Independent Python reference for the MATLAB/Simulink model.
State  x = [beta, p, r, phi]^T      Input u = [aileron, rudder] (rad)
Run from anywhere:  python python/f104_reference.py
Writes figures to results/figures/python/ and prints the mode table.
"""
import os
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

G, A0, RHO = 32.2, 1116.44, 0.00238
D2R = np.pi / 180

# concise derivatives from the problem statement (lb = l_beta, nb = n_beta, ...)
FC = {
    "FC1": dict(M=0.257, th=2.30, yv=-0.178, lb=-20.9, nb=2.68, lp=-1.38, np_=-0.0993,
                lr=1.16, nr=-0.157, yxi=0, lxi=4.76, nxi=0.266, yze=0.0317, lze=5.35, nze=-0.923),
    "FC2": dict(M=0.800, th=2.00, yv=-0.452, lb=-146.0, nb=13.6, lp=-4.64, np_=-0.188,
                lr=3.67, nr=-0.498, yxi=0, lxi=49.6, nxi=3.51, yze=0.0719, lze=41.5, nze=-7.07),
    "FC3": dict(M=1.100, th=1.00, yv=-0.791, lb=-363.0, nb=42.7, lp=-7.12, np_=-0.341,
                lr=7.17, nr=-1.06, yxi=0, lxi=81.5, nxi=6.5, yze=0.0621, lze=57.6, nze=-8.72),
}


def build(fc):
    """Return A, B, Ve for one flight condition."""
    th = fc["th"] * D2R
    Ve = fc["M"] * A0
    A = np.array([
        [fc["yv"], np.sin(th), -np.cos(th), G * np.cos(th) / Ve],
        [fc["lb"], fc["lp"],   fc["lr"],    0],
        [fc["nb"], fc["np_"],  fc["nr"],    0],
        [0,        1,          np.tan(th),  0]])
    B = np.array([[fc["yxi"], fc["yze"]],
                  [fc["lxi"], fc["lze"]],
                  [fc["nxi"], fc["nze"]],
                  [0, 0]])
    return A, B, Ve


def classify_modes(eigs):
    """Dutch roll = complex pair; roll = most negative real root; spiral = other real root."""
    cplx = eigs[np.abs(eigs.imag) > 1e-9]
    real = np.sort(eigs[np.abs(eigs.imag) <= 1e-9].real)
    dr = cplx[cplx.imag > 0][0]
    return dict(dutch=dr, zeta=-dr.real / abs(dr), wn=abs(dr), roll=real[0], spiral=real[1])


def simulate(A, B, case, t):
    """Linear simulation; returns [beta p r phi] in deg, deg/s."""
    sys = signal.StateSpace(A, B, np.eye(4), np.zeros((4, 2)))
    if case == "free":
        _, y, _ = signal.lsim(sys, np.zeros((len(t), 2)), t, X0=[2 * D2R, 0, 0, 0])
    else:
        u = np.zeros((len(t), 2))
        u[t < 1.0, 0 if case == "aileron" else 1] = 2 * D2R
        _, y, _ = signal.lsim(sys, u, t, interp=False)
    return y / D2R


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    figdir = os.path.join(here, "..", "results", "figures", "python")
    os.makedirs(figdir, exist_ok=True)
    t = np.linspace(0, 20, 4001)
    res = {}
    for name, fc in FC.items():
        A, B, Ve = build(fc)
        q = 0.5 * RHO * Ve**2
        m = classify_modes(np.linalg.eigvals(A))
        res[name] = dict(M=fc["M"], Ve=Ve, q=q, **m)
        print(f"{name}: Ve={Ve:7.1f} ft/s  q={q:7.1f} psf | Dutch {m['dutch']:.4f} "
              f"zeta={m['zeta']:.3f} wn={m['wn']:.2f} | roll {m['roll']:.3f} "
              f"(tau={-1/m['roll']:.3f}s) | spiral {m['spiral']:+.5f}")

        fig, ax = plt.subplots(4, 3, figsize=(12, 9), sharex=True)
        for j, (case, title) in enumerate([("free", "Free response, beta0 = 2 deg"),
                                           ("aileron", "Aileron pulse 2 deg, 1 s"),
                                           ("rudder", "Rudder pulse 2 deg, 1 s")]):
            y = simulate(A, B, case, t)
            for i, lab in enumerate(["beta (deg)", "p (deg/s)", "r (deg/s)", "phi (deg)"]):
                ax[i, j].plot(t, y[:, i]); ax[i, j].grid(alpha=.3)
                if j == 0: ax[i, j].set_ylabel(lab)
            ax[0, j].set_title(title); ax[3, j].set_xlabel("t (s)")
        fig.suptitle(f"F-104A {name}  (M = {fc['M']})"); fig.tight_layout()
        fig.savefig(os.path.join(figdir, f"{name}_time_histories.png"), dpi=130); plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 5))
    for (name, r), c in zip(res.items(), ["tab:blue", "tab:orange", "tab:green"]):
        pts = [r["dutch"], np.conj(r["dutch"]), r["roll"], r["spiral"]]
        ax.scatter([p.real for p in pts], [p.imag for p in pts], c=c, label=name, s=45)
    ax.axvline(0, color="k", lw=.8); ax.set_xlabel("Real (1/s)"); ax.set_ylabel("Imag (rad/s)")
    ax.set_title("Eigenvalues: Dutch roll pair, roll, spiral"); ax.grid(alpha=.3); ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(figdir, "eigenvalue_map.png"), dpi=130); plt.close(fig)

    M = [r["M"] for r in res.values()]; Q = [r["q"] for r in res.values()]
    fig, axs = plt.subplots(2, 3, figsize=(12, 6.5))
    for row, (x, xl) in enumerate([(M, "Mach"), (Q, "q (lbf/ft^2)")]):
        axs[row, 0].plot(x, [r["wn"] for r in res.values()], "o-"); axs[row, 0].set_ylabel("Dutch roll wn (rad/s)")
        axs[row, 1].plot(x, [r["zeta"] for r in res.values()], "o-"); axs[row, 1].set_ylabel("Dutch roll zeta")
        axs[row, 2].plot(x, [-1 / r["roll"] for r in res.values()], "o-"); axs[row, 2].set_ylabel("Roll tau (s)")
        for a in axs[row]: a.set_xlabel(xl); a.grid(alpha=.3)
    fig.suptitle("Mode trends (only 3 points: lines are guides)"); fig.tight_layout()
    fig.savefig(os.path.join(figdir, "mode_trends.png"), dpi=130); plt.close(fig)
