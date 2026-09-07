"""Steps DP/DQ/DR -- train + fully validate the gate on the 168-feature
recordings dataset, with a matched 102-feature baseline on IDENTICAL
frames (the same vectors minus the 66 contour columns). OFFLINE.

Protocol unchanged from CS-CT / CW-CZ: LR, asymmetric loss (FN 7x FP),
nan-aware per-fold standardization, HOLD OUT ENTIRE RUNS (and, for the
zone test, entire ZONES). Nothing tuned to preserve/improve AUC.
"""
import numpy as np

from gate_train import NanStandardizer, train_lr, predict_lr, roc_auc, FN_FP_WEIGHT

CACHE = "/home/saurabh/ardu_ws/eval_results/gate_recordings_168.npz"

CONTOUR_PREFIXES = ("tau_area_sector_", "contour_region_count_sector_",
                    "contour_total_area_sector_", "contour_growth_sector_",
                    "ttc_source_sector_", "tau_agreement_sector_")
OBVIOUS = set([f"valid_sector_{i}" for i in range(11)] + ["gyro_valid", "odom_valid", "min_tau",
              "global_feature_count"] + [f"count_sector_{i}" for i in range(11)])


def load():
    d = np.load(CACHE, allow_pickle=True)
    return d["X"], d["y"], d["runs"], d["zones"], d["ts"], list(d["names"])


def col_masks(names):
    contour = np.array([any(n.startswith(p) for p in CONTOUR_PREFIXES) for n in names])
    return ~contour, contour  # (base_102_mask, contour_mask)


def loro(X, y, runs):
    oof = np.full(len(y), np.nan)
    per = []
    for r in sorted(set(runs)):
        te = runs == r; tr = ~te
        if y[tr].sum() == 0 or y[tr].sum() == len(y[tr]):
            # a fold whose training set is single-class can't train a ranker; skip its fit
            continue
        sc = NanStandardizer().fit(X[tr])
        w = train_lr(sc.transform(X[tr]), y[tr])
        oof[te] = predict_lr(w, sc.transform(X[te]))
        if len(set(y[te])) == 2:
            per.append((r, roc_auc(y[te], oof[te])))
    ok = ~np.isnan(oof)
    return roc_auc(y[ok], oof[ok]), per, oof


def leave_one_zone(X, y, zones):
    out = {}
    for z in sorted(set(zones)):
        te = zones == z; tr = ~te
        if len(set(y[te])) < 2 or y[tr].sum() == 0:
            out[z] = float("nan"); continue
        sc = NanStandardizer().fit(X[tr])
        w = train_lr(sc.transform(X[tr]), y[tr])
        out[z] = roc_auc(y[te], predict_lr(w, sc.transform(X[te])))
    return out


def time_since_takeoff(runs, ts):
    o = np.zeros(len(ts))
    for r in sorted(set(runs)):
        m = runs == r; o[m] = ts[m] - ts[m].min()
    return o


def spearman_time(col, tst, runs):
    rhos = []
    for r in sorted(set(runs)):
        m = runs == r
        a, b = col[m], tst[m]
        ok = np.isfinite(a) & np.isfinite(b)
        if ok.sum() < 5:
            continue
        ra = np.argsort(np.argsort(a[ok])).astype(float); rb = np.argsort(np.argsort(b[ok])).astype(float)
        ra -= ra.mean(); rb -= rb.mean()
        den = np.sqrt((ra**2).sum()*(rb**2).sum())
        if den > 0:
            rhos.append((ra*rb).sum()/den)
    return np.mean(rhos) if rhos else np.nan, (np.std(rhos) if rhos else np.nan)


def main():
    X, y, runs, zones, ts, names = load()
    base_mask, contour_mask = col_masks(names)
    print(f"dataset: {X.shape[0]} frames, {len(set(runs))} runs, {100*y.mean():.2f}% positive")
    print(f"features: {X.shape[1]} total = {base_mask.sum()} base + {contour_mask.sum()} contour")
    print(f"zones: {sorted(set(zones))}; per-zone positives: "
          + ", ".join(f"{z}:{int(y[zones==z].sum())}/{int((zones==z).sum())}" for z in sorted(set(zones))))

    Xb = X[:, base_mask]  # 102 baseline
    # ===== DP: LORO pooled AUC, 102 vs 168 =====
    print("\n===== DP: leave-one-run-out pooled AUC =====")
    auc102, per102, _ = loro(Xb, y, runs)
    auc168, per168, _ = loro(X, y, runs)
    print(f"  102-feature (base):  pooled AUC = {auc102:.4f}")
    print(f"  168-feature (+contour): pooled AUC = {auc168:.4f}   delta {auc168-auc102:+.4f}")
    a102 = np.array([a for _, a in per102]); a168 = np.array([a for _, a in per168])
    print(f"  per-run mean+-sd: 102 {a102.mean():.3f}+-{a102.std():.3f} | 168 {a168.mean():.3f}+-{a168.std():.3f}")

    # ===== DQ: weights (168 full-data fit) =====
    print("\n===== DQ: weights (168-feature, full-data fit) =====")
    sc = NanStandardizer().fit(X); w = train_lr(sc.transform(X), y); wt = w[1:]
    order = np.argsort(-np.abs(wt))
    print("  top 18 by |weight| (C=contour, O=obvious, R=richer-LK):")
    for i in order[:18]:
        nm = names[i]
        cat = "C" if contour_mask[i] else ("O" if nm in OBVIOUS else "R")
        print(f"    [{cat}] {wt[i]:+.3f}  {nm}")
    mass = np.abs(wt)
    m_obv = mass[[i for i,n in enumerate(names) if n in OBVIOUS]].sum()
    m_cont = mass[contour_mask].sum()
    m_rich = mass.sum() - m_obv - m_cont
    tot = mass.sum()
    print(f"  |weight| mass: obvious {100*m_obv/tot:.0f}% | richer-LK {100*m_rich/tot:.0f}% | contour {100*m_cont/tot:.0f}%")
    print(f"  post-O2 (richer-LK + contour) share: {100*(m_rich+m_cont)/tot:.0f}%")
    # tau_agreement specifically
    ag = [i for i,n in enumerate(names) if n.startswith("tau_agreement_sector_")]
    src = [i for i,n in enumerate(names) if n.startswith("ttc_source_sector_")]
    print(f"  tau_agreement total |weight|: {mass[ag].sum():.3f} (max single {mass[ag].max():.3f})")
    print(f"  ttc_source   total |weight|: {mass[src].sum():.3f} (max single {mass[src].max():.3f})")

    # ===== DR: full re-validation, 102 vs 168 =====
    print("\n===== DR: full re-validation (102 vs 168) =====")
    tst = time_since_takeoff(runs, ts)
    # positional-only
    pos_auc,_,_ = loro(tst.reshape(-1,1), y, runs)
    print(f"  [leakage] positional-only (time-since-takeoff) pooled AUC = {pos_auc:.4f}")
    # drop-drifting-features delta (168)
    drift_idx = []
    for i in range(X.shape[1]):
        mr, srr = spearman_time(X[:,i], tst, runs)
        if np.isfinite(mr) and abs(mr) >= 0.5 and np.isfinite(srr) and srr <= 0.35:
            drift_idx.append(i)
    keep = [i for i in range(X.shape[1]) if i not in drift_idx]
    auc_nd,_,_ = loro(X[:,keep], y, runs)
    print(f"  [leakage] drop {len(drift_idx)} drifting feats: 168 AUC {auc168:.4f} -> {auc_nd:.4f} ({auc_nd-auc168:+.4f})")

    print("\n  comparison table (metric | 102 | 168):")
    # per-zone AUC via LORO oof
    _, _, oof102 = loro(Xb, y, runs); _, _, oof168 = loro(X, y, runs)
    for z in sorted(set(zones)):
        m = (zones==z) & ~np.isnan(oof168)
        if len(set(y[m]))<2:
            print(f"    per-zone AUC {z:<7} | (single-class, n/a)"); continue
        a1=roc_auc(y[m],oof102[m]); a2=roc_auc(y[m],oof168[m])
        print(f"    per-zone AUC {z:<7} | {a1:.3f} | {a2:.3f}")
    print(f"    LORO pooled       | {auc102:.3f} | {auc168:.3f}")
    print(f"    LORO per-run mean | {a102.mean():.3f} | {a168.mean():.3f}")

    # leave-one-ZONE-out -- the one that matters
    print("\n  leave-one-ZONE-out (train other zones, test held zone):")
    lzo102 = leave_one_zone(Xb, y, zones); lzo168 = leave_one_zone(X, y, zones)
    print(f"    {'zone':<8} {'102':>7} {'168':>7} {'delta':>8}")
    for z in sorted(set(zones)):
        a1, a2 = lzo102.get(z, float('nan')), lzo168.get(z, float('nan'))
        dd = a2-a1 if (np.isfinite(a1) and np.isfinite(a2)) else float('nan')
        print(f"    {z:<8} {a1:>7.3f} {a2:>7.3f} {dd:>+8.3f}")

    # operating point (168), asymmetric-loss threshold
    thr = 1.0/(1.0+FN_FP_WEIGHT)
    pred = oof168 >= thr
    ok = ~np.isnan(oof168)
    tp=int(((pred==1)&(y==1)&ok).sum()); fp=int(((pred==1)&(y==0)&ok).sum()); fn=int(((pred==0)&(y==1)&ok).sum())
    defer = pred[ok].mean()
    prec = tp/(tp+fp) if tp+fp else float('nan'); rec = tp/(tp+fn) if tp+fn else float('nan')
    print(f"\n  [operating point, 168, p*={thr:.3f}] deferral={100*defer:.1f}% precision={prec:.3f} "
          f"recall={rec:.3f} implied periodic k={1/defer:.1f}" if defer>0 else "  defer 0")


if __name__ == "__main__":
    main()
