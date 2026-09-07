"""Phase 5 validation -- Steps CW-CZ. ADVERSARIAL checks on the gate's
AUC 0.900 headline: the goal is to try to BREAK it, not confirm it.
OFFLINE, no sim.

CW  -- label-leakage audit: is the model reading TIME-ELAPSED (every
       run ends in collision, so "labelled 1" ~= "near the end") rather
       than DANGER?
CX  -- weights sorted by magnitude, with the Step O2 framing (do the
       RICHER features -- temporal derivatives, gradient energy,
       cone-relative counts -- carry the weight, or the obvious ones?).
CY  -- operating point: ROC curve, deferral rate at the asymmetric-loss
       threshold, implied PERIODIC-baseline k, precision/recall.
CZ  -- leave-one-run-out (mean/spread) AND leave-one-ZONE-out.

Nothing is tuned to preserve 0.900. No disagreement labels.
"""
import numpy as np

from gate_dataset import build_dataset
from gate_train import (NanStandardizer, train_lr, predict_lr, roc_auc,
                        FN_FP_WEIGHT, LABEL_HORIZON_S)


def loro_oof(X, y, run_ids):
    """Leave-one-run-out; returns pooled OOF scores + per-fold AUCs."""
    runs = sorted(set(run_ids))
    oof = np.full(len(y), np.nan)
    per_fold = []
    for held in runs:
        te = run_ids == held
        tr = ~te
        sc = NanStandardizer().fit(X[tr])
        w = train_lr(sc.transform(X[tr]), y[tr])
        oof[te] = predict_lr(w, sc.transform(X[te]))
        per_fold.append((held, roc_auc(y[te], oof[te])))
    return oof, per_fold


def time_since_takeoff(run_ids, tcap):
    """Per-run t_capture minus that run's first t_capture."""
    out = np.zeros(len(tcap))
    for r in sorted(set(run_ids)):
        m = run_ids == r
        out[m] = tcap[m] - tcap[m].min()
    return out


def frame_index(run_ids):
    out = np.zeros(len(run_ids))
    for r in sorted(set(run_ids)):
        m = np.where(run_ids == r)[0]
        out[m] = np.arange(len(m))
    return out


def spearman(a, b):
    """Spearman rho on pairwise-complete (non-NaN) entries."""
    ok = np.isfinite(a) & np.isfinite(b)
    if ok.sum() < 5:
        return np.nan
    ra = np.argsort(np.argsort(a[ok]))
    rb = np.argsort(np.argsort(b[ok]))
    ra = ra - ra.mean(); rb = rb - rb.mean()
    denom = np.sqrt((ra**2).sum() * (rb**2).sum())
    return float((ra * rb).sum() / denom) if denom > 0 else np.nan


def precision_recall_at(y, scores, thr):
    pred = scores >= thr
    tp = int(((pred == 1) & (y == 1)).sum())
    fp = int(((pred == 1) & (y == 0)).sum())
    fn = int(((pred == 0) & (y == 1)).sum())
    prec = tp / (tp + fp) if (tp + fp) else float("nan")
    rec = tp / (tp + fn) if (tp + fn) else float("nan")
    return prec, rec, pred.mean()


def roc_curve(y, scores, n=25):
    """(fpr, tpr, thr) sampled at n thresholds spanning the scores."""
    thrs = np.quantile(scores, np.linspace(0, 1, n))
    pts = []
    P = (y == 1).sum(); N = (y == 0).sum()
    for t in thrs:
        pred = scores >= t
        tpr = ((pred == 1) & (y == 1)).sum() / P if P else np.nan
        fpr = ((pred == 1) & (y == 0)).sum() / N if N else np.nan
        pts.append((fpr, tpr, t))
    return pts


def main():
    X, y, run_ids, zones, names, per_run, excluded = build_dataset()
    tcap = np.array([0.0] * len(y))  # rebuild t_capture per frame for CW check 1
    # rebuild t_capture aligned to X rows (build_dataset doesn't return it)
    import json, glob, os
    from gate_dataset import EVAL_DIR
    # reconstruct in the SAME order build_dataset iterated (sorted usable paths)
    from gate_dataset import discover_usable_runs
    usable, _ = discover_usable_runs()
    tlist = []
    for path in usable:
        for line in open(path):
            line = line.strip()
            if line:
                tlist.append(json.loads(line)["t_capture"])
    tcap = np.array(tlist)
    assert len(tcap) == len(y), f"t_capture len {len(tcap)} != y {len(y)}"

    print(f"dataset: {X.shape[0]} frames, {len(set(run_ids))} runs, {100*y.mean():.2f}% pos\n")

    # ================= Step CW =================
    print("=" * 64)
    print("STEP CW -- LABEL LEAKAGE AUDIT")
    print("=" * 64)

    # CW.1 -- positional-only models
    print("\n[CW.1] AUC from POSITIONAL features alone (leave-one-run-out):")
    for fname, feat in [("time_since_takeoff", time_since_takeoff(run_ids, tcap)),
                        ("frame_index", frame_index(run_ids))]:
        F = feat.reshape(-1, 1)
        oof, _ = loro_oof(F, y, run_ids)
        print(f"  {fname:<20} pooled AUC = {roc_auc(y, oof):.4f}")
    print("  (HIGH here = positional info alone predicts the label = contamination)")

    # CW.2 -- monotonic drift per feature (mean Spearman vs time-since-takeoff, across runs)
    print("\n[CW.2] Monotonic drift: mean per-run Spearman(feature, time-since-takeoff):")
    tst = time_since_takeoff(run_ids, tcap)
    drift = []
    for j, nm in enumerate(names):
        rhos = []
        for r in sorted(set(run_ids)):
            m = run_ids == r
            rho = spearman(X[m, j], tst[m])
            if np.isfinite(rho):
                rhos.append(rho)
        mean_rho = np.mean(rhos) if rhos else np.nan
        std_rho = np.std(rhos) if rhos else np.nan
        drift.append((nm, mean_rho, std_rho, len(rhos)))
    # flag: |mean rho| high AND consistent (low std) = a clock
    drift_sorted = sorted(drift, key=lambda d: -abs(d[1]) if np.isfinite(d[1]) else 0)
    print(f"  {'feature':<28} {'mean_rho':>9} {'std_rho':>8}  drift-flag(|mean|>=0.5 & std<=0.35)")
    drifters = []
    for nm, mr, sr, k in drift_sorted[:20]:
        flag = ""
        if np.isfinite(mr) and abs(mr) >= 0.5 and np.isfinite(sr) and sr <= 0.35:
            flag = "  <== DRIFTS"
            drifters.append(nm)
        print(f"  {nm:<28} {mr:>9.3f} {sr:>8.3f}{flag}")
    print(f"  flagged drifters: {drifters if drifters else 'NONE'}")

    # CW.3 -- retrain excluding drifters
    print("\n[CW.3] Retrain EXCLUDING drifting features:")
    oof_full, _ = loro_oof(X, y, run_ids)
    auc_full = roc_auc(y, oof_full)
    print(f"  full 54-feature pooled AUC = {auc_full:.4f}")
    if drifters:
        keep = [j for j, nm in enumerate(names) if nm not in drifters]
        oof_nd, _ = loro_oof(X[:, keep], y, run_ids)
        auc_nd = roc_auc(y, oof_nd)
        print(f"  excluding {len(drifters)} drifter(s): pooled AUC = {auc_nd:.4f}  (delta {auc_nd-auc_full:+.4f})")
    else:
        print("  no drifters flagged; nothing to exclude")

    # CW.4 -- per-zone AUC (on full-model OOF)
    print("\n[CW.4] Per-zone AUC (full-model out-of-fold predictions):")
    for z in sorted(set(zones)):
        m = zones == z
        print(f"  {z:<8} AUC={roc_auc(y[m], oof_full[m]):.4f}  frames={int(m.sum())} pos={int(y[m].sum())}")

    # ================= Step CX =================
    print("\n" + "=" * 64)
    print("STEP CX -- WEIGHTS (full-data fit)")
    print("=" * 64)
    sc = NanStandardizer().fit(X)
    w = train_lr(sc.transform(X), y)
    weights = w[1:]
    order = np.argsort(-np.abs(weights))
    OBVIOUS = set([f"valid_sector_{i}" for i in range(5)] + ["gyro_valid", "odom_valid"] +
                  [f"count_sector_{i}" for i in range(5)] + ["global_feature_count", "min_tau"])
    print(f"intercept {w[0]:+.3f}. Top 15 by |weight| (O = 'obvious' feature, R = richer/post-O2):")
    for i in order[:15]:
        cat = "O" if names[i] in OBVIOUS else "R"
        print(f"  [{cat}] {weights[i]:+.4f}  {names[i]}")
    # magnitude mass obvious vs richer
    mass_obv = sum(abs(weights[i]) for i, nm in enumerate(names) if nm in OBVIOUS)
    mass_rich = sum(abs(weights[i]) for i, nm in enumerate(names) if nm not in OBVIOUS)
    print(f"  |weight| mass: obvious={mass_obv:.2f}  richer={mass_rich:.2f}  "
          f"(richer share {100*mass_rich/(mass_obv+mass_rich):.0f}%)")

    # ================= Step CY =================
    print("\n" + "=" * 64)
    print("STEP CY -- OPERATING POINT & DEFERRAL RATE")
    print("=" * 64)
    print("ROC curve (fpr, tpr) on full-model OOF, 15 pts:")
    for fpr, tpr, t in roc_curve(y, oof_full, 15):
        print(f"  fpr={fpr:.3f} tpr={tpr:.3f} thr={t:.3f}")
    # asymmetric-loss-implied threshold: cost(FN)=7*cost(FP) -> Bayes threshold
    # p* = cost_FP / (cost_FP + cost_FN) = 1/(1+7) = 0.125
    thr = 1.0 / (1.0 + FN_FP_WEIGHT)
    prec, rec, defer = precision_recall_at(y, oof_full, thr)
    print(f"\nAsymmetric-loss-implied threshold p* = 1/(1+{FN_FP_WEIGHT:.0f}) = {thr:.3f}")
    print(f"  deferral rate (frames scored >= p*) = {100*defer:.1f}%")
    print(f"  precision = {prec:.3f}   recall = {rec:.3f}")
    print(f"  implied PERIODIC-baseline k (defer every k-th frame to match rate) = "
          f"{(1.0/defer):.1f}" if defer > 0 else "  defer rate 0")

    # ================= Step CZ =================
    print("\n" + "=" * 64)
    print("STEP CZ -- CROSS-VALIDATION")
    print("=" * 64)
    _, per_fold = loro_oof(X, y, run_ids)
    aucs = np.array([a for _, a in per_fold if np.isfinite(a)])
    print(f"[CZ.a] leave-one-RUN-out: mean AUC {aucs.mean():.4f} +/- {aucs.std():.4f} "
          f"(pooled {auc_full:.4f}, n={len(aucs)})")
    print("\n[CZ.b] leave-one-ZONE-out (train on other zones, test on held-out zone):")
    print("  (note: no Zone A in the usable cheap runs -- zones present are B/C/D)")
    zone_list = sorted(set(zones))
    for z in zone_list:
        te = zones == z
        tr = ~te
        if te.sum() == 0 or tr.sum() == 0:
            continue
        scz = NanStandardizer().fit(X[tr])
        wz = train_lr(scz.transform(X[tr]), y[tr])
        s = predict_lr(wz, scz.transform(X[te]))
        print(f"  test={z:<8} train={'+'.join(x for x in zone_list if x!=z):<18} "
              f"AUC={roc_auc(y[te], s):.4f}  test_frames={int(te.sum())} pos={int(y[te].sum())}")


if __name__ == "__main__":
    main()
