"""Generate the final report's res_*.png figures (run from any directory).

Uses captures.py for parsing/calibration windows and fusion.py for all alert
replays and scores. No source CSV, parameter, or firmware file is modified.
Printed tables supply the chapter's durations and hardware-validation figures.
"""
from __future__ import annotations

from dataclasses import replace

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
from matplotlib.ticker import FuncFormatter
import numpy as np
import pandas as pd

import captures as C
import fusion as F

OUT = C.ROOT / "report" / "figures"
SUBJECTS = ["Andre-Monteiro", "Felipe-godoi", "Gustavo-Socce", "Rafael-Herleng",
            "Ryan-Akira", "Thiago-Fita", "Thiago-M", "Vinicius-Beleti"]
NAMES = ["André", "Felipe", "Gustavo", "Rafael", "Ryan", "T. Fita", "T. M.", "Vinicius"]
TAGS = ["1 Ereto", "2 Curvado à frente", "3 Cabeça à frente", "4 Escorregado",
        "5 Inclinado à esquerda", "6 Inclinado à direita", "7 Reclinado",
        "8 Perna E sobre D", "9 Perna D sobre E"]
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                     "axes.spines.top": False, "axes.spines.right": False,
                     "savefig.dpi": 200, "axes.formatter.useoffset": False})
DECIMAL = FuncFormatter(lambda v, _: f"{v:g}".replace(".", ","))


def save(fig: plt.Figure, name: str) -> None:
    fig.savefig(OUT / f"res_{name}.png", bbox_inches="tight", facecolor="white",
                metadata={"Software": "analysis/report_figures.py"})
    plt.close(fig)
    print(f"figure: report/figures/res_{name}.png", flush=True)


def seat_indices(x: F.Inputs) -> tuple[np.ndarray, np.ndarray]:
    """Display indices defined in fusion.py, without reimplementing decisions."""
    f, b = x.f, x.base
    with np.errstate(divide="ignore", invalid="ignore"):
        rear = (f[:, F.SRL] + f[:, F.SRR]) / (b[:, F.SRL] + b[:, F.SRR])
        lr = ((f[:, F.SFL] + f[:, F.SRL] - f[:, F.SFR] - f[:, F.SRR]) / f[:, :4].sum(1)
              - (b[:, F.SFL] + b[:, F.SRL] - b[:, F.SFR] - b[:, F.SRR]) / b[:, :4].sum(1))
    rear[~x.valid], lr[~x.valid] = np.nan, np.nan
    return rear, lr


def channels(xs: list[F.Inputs]) -> pd.DataFrame:
    rows = []
    for x in xs:
        rear, lr = seat_indices(x)
        mask = x.valid & x.run.rows.settled.to_numpy()
        rows.append(pd.DataFrame({"subject": x.run.subject, "tag": x.run.rows.tag[mask],
                                  "dp": x.dp[mask], "dr": x.dr[mask],
                                  "rear": rear[mask], "lr": lr[mask]}))
    return pd.concat(rows, ignore_index=True)


def matrix(d: pd.DataFrame, value: str) -> np.ndarray:
    return (d.groupby(["tag", "subject"])[value].median().unstack("subject")
            .reindex(index=range(1, 10), columns=SUBJECTS).to_numpy())


def heatmap(ax, values, title, *, norm, fmt, cmap="Greys", rows=True):
    cm = plt.get_cmap(cmap).copy()
    cm.set_bad("#e8e8e8")
    im = ax.imshow(np.ma.masked_invalid(values), cmap=cm, norm=norm, aspect="auto")
    ax.set_title(title, fontsize=11, pad=12)
    ax.set_xticks(range(8), NAMES, rotation=45, ha="right")
    ax.set_yticks(range(9), TAGS if rows else [""] * 9)
    ax.tick_params(length=0)
    for i in range(9):
        for j in range(8):
            v = values[i, j]
            text = "—" if np.isnan(v) else fmt(v).replace(".", ",")
            if text in ("-0,00", "-0,0"):
                text = text[1:]
            rgba = im.cmap(im.norm(v)) if np.isfinite(v) else (1, 1, 1, 1)
            lum = .2126 * rgba[0] + .7152 * rgba[1] + .0722 * rgba[2]
            ax.text(j, i, text, ha="center", va="center", fontsize=10,
                    color="white" if lum < .48 else "black")
    return im


def channel_figures(d: pd.DataFrame):
    fig, axs = plt.subplots(2, 1, figsize=(8.4, 9.4), layout="constrained")
    norm = TwoSlopeNorm(vmin=-40, vcenter=0, vmax=40)
    for ax, key, title, rows in zip(axs, ["dp", "dr"],
                                   ["a) Δpitch: lateral (+ direita), graus",
                                    "b) Δroll: sagital (+ para trás), graus"], [True, True]):
        im = heatmap(ax, matrix(d, key), title, norm=norm, fmt=lambda v: f"{v:.1f}",
                     cmap="RdBu_r", rows=rows)
        fig.colorbar(im, ax=ax, shrink=.85, format=DECIMAL, label="Desvio (°)", extend="both")
    save(fig, "imu_posturas")

    fig, axs = plt.subplots(2, 1, figsize=(8.4, 9.4), layout="constrained")
    im = heatmap(axs[0], matrix(d, "rear"), "a) Assento traseiro / linha de base",
                 norm=plt.Normalize(0, 1.5), fmt=lambda v: f"{v:.2f}")
    fig.colorbar(im, ax=axs[0], shrink=.85, format=DECIMAL, label="Razão traseira", extend="max")
    im = heatmap(axs[1], matrix(d, "lr"), "b) Deslocamento E/D (+ esquerda)",
                 norm=TwoSlopeNorm(vmin=-.5, vcenter=0, vmax=.5),
                 fmt=lambda v: f"{v:.2f}", cmap="RdBu_r", rows=True)
    fig.colorbar(im, ax=axs[1], shrink=.85, format=DECIMAL, label="Variação do índice E/D", extend="both")
    save(fig, "assento_posturas")


def alert_figure(xs: list[F.Inputs]):
    segs = F.evaluate(xs, F.FUSION)
    segs = segs.assign(on=segs.frac * segs.n)
    g = segs.groupby(["tag", "subject"])[["on", "n"]].sum()
    values = (100 * g.on / g.n).unstack("subject").reindex(index=range(1, 10), columns=SUBJECTS)
    fig, ax = plt.subplots(figsize=(9.3, 5.5), layout="constrained")
    im = heatmap(ax, values.to_numpy(), "Fusão com parâmetros finais: tempo com alerta (%)",
                 norm=plt.Normalize(0, 100), fmt=lambda v: f"{v:.0f}")
    fig.colorbar(im, ax=ax, shrink=.85, label="Tempo com alerta (%)")
    save(fig, "alerta_sujeitos")


def ablation_figure(xs: list[F.Inputs]):
    params = [F.CURRENT, replace(F.CURRENT, empty_raw=F.FUSION.empty_raw),
              replace(F.CURRENT, empty_raw=F.FUSION.empty_raw, slump_frac=F.FUSION.slump_frac),
              replace(F.CURRENT, empty_raw=F.FUSION.empty_raw, slump_frac=F.FUSION.slump_frac,
                      recline_frac=F.FUSION.recline_frac), F.FUSION]
    labels = ["Só IMU", "+ presença", "+ escorregamento", "+ reclinação", "+ inclinação lateral"]
    vals = [F.objective(F.evaluate(xs, p)) for p in params]
    fig, ax = plt.subplots(figsize=(9, 3.4), layout="constrained")
    bars = ax.barh(labels, vals, color=[".72", ".65", ".55", ".40", ".25"], edgecolor="black")
    for b, v in zip(bars, vals):
        ax.text(v + .008, b.get_y() + b.get_height() / 2, f"{v:.3f}".replace(".", ","), va="center")
    ax.invert_yaxis()
    ax.set(xlim=(0, 1.03), xlabel="Pontuação balanceada por segmento (0 a 1)")
    ax.xaxis.set_major_formatter(DECIMAL)
    ax.grid(axis="x", alpha=.2)
    ax.set_axisbelow(True)
    save(fig, "ablacao")
    print("ablation:", dict(zip(labels, vals)))


def validation_figures():
    run = C.load(C.DATA / "validation" / "2026-10-22_Lucas-Santos_run1.csv")
    x, r = F.inputs(run), run.rows
    rear, _ = seat_indices(x)
    fig, axs = plt.subplots(4, 1, figsize=(10.5, 6.8), sharex=True, layout="constrained",
                            gridspec_kw={"height_ratios": [2, 1.6, 1, 1]})
    # Keep the recorded magnitude, including motor disturbances; hide the uncalibrated prefix.
    axs[0].plot(r.t, r.dev.where(x.valid), color="black", lw=.85, label="Desvio registrado")
    axs[0].axhline(F.FUSION.alert_deg, color=".4", ls="--", label="Limiar de 10°")
    axs[0].set(ylabel="Desvio (°)", ylim=(0, 45))
    axs[0].legend(loc="upper right", ncol=2, fontsize=8)
    axs[1].plot(r.t, rear, color="black", lw=.85)
    axs[1].axhline(F.FUSION.slump_frac, color=".4", ls="--", label="Escorregamento: 0,3")
    axs[1].axhline(F.FUSION.recline_frac, color=".5", ls=":", label="Reclinação: 0,7")
    axs[1].set(ylabel="Razão traseira", ylim=(-.05, 1.5))
    axs[1].legend(loc="upper right", ncol=2, fontsize=8)
    axs[2].step(r.t, r.alert, where="post", color="black", lw=1)
    axs[2].fill_between(r.t, r.alert, step="post", color=".7")
    axs[2].set(ylabel="Alerta", yticks=[0, 1], ylim=(-.1, 1.15))
    axs[3].step(r.t, r.tag, where="post", color="black", lw=1)
    axs[3].set(ylabel="Etiqueta", yticks=[0, 1, 2, 4, 5, 6, 8], ylim=(-.5, 8.8),
               xlabel="Tempo desde o início do registro (s)", xlim=(0, float(r.t.iloc[-1])))
    for sg in x.segs:
        lo, hi = r.t.iloc[sg.i0], r.t.iloc[sg.i1 - 1]
        for ax in axs:
            ax.axvspan(lo, hi, color=".6", alpha=.12, zorder=-1)
    for ax in axs:
        ax.grid(alpha=.18)
        ax.yaxis.set_major_formatter(DECIMAL)
    save(fig, "validacao_lucas")

    fig, axs = plt.subplots(3, 1, figsize=(10.5, 4.8), sharex=True, layout="constrained")
    lo, hi = r.loc[r.tag == 5, "t"].agg(["min", "max"])
    m = r.t.between(lo - 1, hi + 1)
    axs[0].plot(r.t[m], r.dev[m], color="black", lw=1)
    axs[0].axhline(F.FUSION.alert_deg, color=".4", ls="--", label="Disparo: 10°")
    axs[0].axhline(F.FUSION.clear_deg, color=".4", ls=":", label="Limpeza: 7°")
    axs[0].set(ylabel="Desvio (°)")
    axs[0].legend(loc="upper right", ncol=2, fontsize=8)
    for ax, col, label in zip(axs[1:], ["alert", "bmotor"], ["Alerta da cadeira", "Motor reportado"]):
        ax.step(r.t[m], r[col][m], where="post", color="black", lw=1)
        ax.fill_between(r.t[m], r[col][m], step="post", color=".7")
        ax.set(ylabel=label, yticks=[0, 1], ylim=(-.1, 1.2))
    axs[-1].set(xlabel="Tempo desde o início do registro (s)", xlim=(lo - 1, hi + 1))
    for ax in axs:
        ax.axvspan(lo, hi, color=".6", alpha=.1, zorder=-1)
        ax.grid(alpha=.18)
    save(fig, "motor_lucas")

    for run in C.runs(C.DATA / "validation"):
        x, r = F.inputs(run), run.rows
        print(f"\nvalidation {run.name}: logged alert, settled rows")
        print(F.summary(F.seg_table(x, r.alert.to_numpy() == 1)).to_string())
        a = F.replay_v(x, F.FUSION)
        for tag in sorted(set(r.tag) - {0}):
            full = r.tag.to_numpy() == tag
            settled = full & r.settled.to_numpy() & x.valid
            print(f"tag {tag}: logged full={100*r.alert[full].mean():.3f}%, "
                  f"logged settled={100*r.alert[settled].mean():.3f}%, "
                  f"replay settled={100*a[settled].mean():.3f}%")


def inventory(xs: list[F.Inputs]):
    rows = []
    for x in xs:
        s = F.seg_table(x, F.replay_v(x, F.FUSION))
        rows.append({"run": x.run.name, "duration_s": x.run.rows.t.iloc[-1],
                     "presence_s": (x.f[:, :4].sum(1) >= F.FUSION.empty_raw).sum() / 20,
                     "settled_s": s.n.sum() / 20, "segments": len(s), "samples": len(x.t)})
    tab = pd.DataFrame(rows).set_index("run")
    print("\ncapture inventory (time estimates at 20 Hz except duration_s):")
    print(tab.round(2).to_string())
    print("totals:", tab.sum().to_dict())
    s = F.evaluate(xs, F.FUSION)
    print("\nposture inventory:")
    print(s.groupby("tag").agg(segments=("n", "size"), seconds=("n", lambda n: n.sum() / 20)))
    print("\nempty chair totals:", F.empty_report(xs, F.FUSION).sum().to_dict())
    print("\nposture/subject channel medians:")
    print(channels(xs).groupby(["tag", "subject"])[["dp", "dr", "rear", "lr"]].median().round(2))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    xs = [F.inputs(r) for r in C.runs(pattern="2026-10-21_*_run*.csv")]
    F.C_TAGS.update(xs[0].run.tags)
    channel_figures(channels(xs))
    alert_figure(xs)
    ablation_figure(xs)
    validation_figures()
    inventory(xs)


if __name__ == "__main__":
    main()
