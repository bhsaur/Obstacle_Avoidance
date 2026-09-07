"""Phase 5 / Steps CT-CV -- train and evaluate the learning-to-defer
gate. OFFLINE. Depends only on gate_dataset.py and numpy/scipy (no
sklearn on this machine -- LR and ROC AUC implemented here).

Step CT  -- logistic regression, asymmetric loss (FN weighted
            FN_FP_WEIGHT x FP), leave-ONE-RUN-out CV, pooled ROC AUC on
            held-out runs. This AUC is the go/no-go.
Step CU  -- learned weights sorted by |magnitude|, on a full-data fit.
Step CV  -- hand-tuned heuristic baseline (valid-sector count + min tau)
            AUC on the SAME held-out folds.

CONSTRAINTS honoured (task): one setting, no tuning of the label
horizon or the loss weight to chase AUC; entire RUNS held out, never
random frames (consecutive frames are near-duplicates -- a random split
leaks catastrophically); no disagreement labels.

STANDARDIZATION is nan-aware and fit PER FOLD on the training runs only
(nanmean/nanstd), then NaN->0 after centering so an imputed value sits
at the training mean (neutral contribution); the valid_sector_i /
gyro_valid / odom_valid flags -- which are never NaN -- carry the
"this measurement was missing" signal instead. See README Step CT for
why this is the honest choice for a linear, interpretable model given
CheapStage's features are missing-not-at-random.
"""
import numpy as np
from scipy.optimize import minimize

from gate_dataset import build_dataset, LABEL_HORIZON_S

FN_FP_WEIGHT = 7.0   # false negatives weighted 7x false positives (mid of task's 5-10x). ONE setting.
L2 = 1.0             # ridge strength on standardized features; a plain default, not tuned per-fold.


def roc_auc(y_true, scores):
    """Mann-Whitney U form. Ties in scores get average rank."""
    y_true = np.asarray(y_true)
    scores = np.asarray(scores)
    n_pos = int((y_true == 1).sum())
    n_neg = int((y_true == 0).sum())
    if n_pos == 0 or n_neg == 0:
        return float("nan")
    order = np.argsort(scores, kind="mergesort")
    ranks = np.empty(len(scores), dtype=np.float64)
    s_sorted = scores[order]
    i = 0
    while i < len(s_sorted):
        j = i
        while j + 1 < len(s_sorted) and s_sorted[j + 1] == s_sorted[i]:
            j += 1
        avg_rank = (i + j) / 2.0 + 1.0  # 1-based average rank over the tie block
        ranks[order[i:j + 1]] = avg_rank
        i = j + 1
    sum_ranks_pos = ranks[y_true == 1].sum()
    return (sum_ranks_pos - n_pos * (n_pos + 1) / 2.0) / (n_pos * n_neg)


class NanStandardizer:
    def fit(self, X):
        self.mean_ = np.nanmean(X, axis=0)
        self.std_ = np.nanstd(X, axis=0)
        self.std_[~np.isfinite(self.std_) | (self.std_ == 0)] = 1.0
        self.mean_[~np.isfinite(self.mean_)] = 0.0
        return self

    def transform(self, X):
        z = (X - self.mean_) / self.std_
        return np.nan_to_num(z, nan=0.0, posinf=0.0, neginf=0.0)


def _loss_grad(w, Xb, y, sample_w, l2):
    z = Xb @ w
    # stable log-loss
    p = 1.0 / (1.0 + np.exp(-z))
    eps = 1e-12
    ll = -(sample_w * (y * np.log(p + eps) + (1 - y) * np.log(1 - p + eps))).sum()
    reg = 0.5 * l2 * (w[1:] @ w[1:])   # don't regularize intercept (col 0)
    loss = ll + reg
    g = Xb.T @ (sample_w * (p - y))
    g[1:] += l2 * w[1:]
    return loss, g


def train_lr(X, y, l2=L2, fn_fp_weight=FN_FP_WEIGHT):
    Xb = np.hstack([np.ones((X.shape[0], 1)), X])
    sample_w = np.where(y == 1, fn_fp_weight, 1.0)
    w0 = np.zeros(Xb.shape[1])
    res = minimize(_loss_grad, w0, args=(Xb, y.astype(float), sample_w, l2),
                   jac=True, method="L-BFGS-B", options={"maxiter": 500})
    return res.x  # [intercept, w1..w54]


def predict_lr(w, X):
    Xb = np.hstack([np.ones((X.shape[0], 1)), X])
    return 1.0 / (1.0 + np.exp(-(Xb @ w)))


def leave_one_run_out(X, y, run_ids, names):
    runs = sorted(set(run_ids))
    oof_scores = np.full(len(y), np.nan)
    per_fold = []
    for held in runs:
        te = run_ids == held
        tr = ~te
        scaler = NanStandardizer().fit(X[tr])
        Xtr = scaler.transform(X[tr])
        Xte = scaler.transform(X[te])
        w = train_lr(Xtr, y[tr])
        s = predict_lr(w, Xte)
        oof_scores[te] = s
        auc = roc_auc(y[te], s)
        per_fold.append((held, auc, int(y[te].sum()), int(te.sum())))
    pooled = roc_auc(y, oof_scores)
    return pooled, per_fold, oof_scores


# ---- Step CV: heuristic baseline -------------------------------------
def heuristic_score(X, names):
    """Hand-tuned gate: danger rises as (a) FEWER sectors are valid and
    (b) min tau is SMALLER (closer to contact). Both are exactly the
    signals a human would threshold on. Returns a continuous danger
    score (higher = more dangerous) so ROC AUC is threshold-free and
    directly comparable to the LR's -- we are not picking the human's
    operating point for them, just asking whether these two raw signals
    rank collision-imminent frames above safe ones at all.

    min_tau is NaN exactly when NO sector is valid -- the MOST dangerous
    case -- so NaN min_tau maps to maximum danger, not dropped."""
    idx = {n: i for i, n in enumerate(names)}
    valid_cols = [idx[f"valid_sector_{i}"] for i in range(5)]
    n_valid = X[:, valid_cols].sum(axis=1)          # 0..5, never NaN
    min_tau = X[:, idx["min_tau"]].copy()            # NaN if none valid
    # danger from validity: 5 valid -> 0 danger, 0 valid -> 5 danger
    danger_valid = (5.0 - n_valid)
    # danger from proximity: small tau -> high danger; NaN -> max.
    # Normalise tau by a nominal cap so the two terms are comparable-ish;
    # exact scale is irrelevant to AUC (monotone), the COMBINATION order is.
    tau_cap = 30.0
    prox = np.where(np.isnan(min_tau), 1.0, np.clip(1.0 - min_tau / tau_cap, 0.0, 1.0))
    return danger_valid + 5.0 * prox


def main():
    X, y, run_ids, zones, names, per_run, excluded = build_dataset()
    print(f"dataset: {X.shape[0]} frames, {len(set(run_ids))} runs, "
          f"{100*y.mean():.2f}% positive, horizon={LABEL_HORIZON_S}s, FN:FP weight={FN_FP_WEIGHT}\n")

    # ---- Step CT: LR, leave-one-run-out ----
    print("=== Step CT: Logistic Regression, leave-one-run-out CV ===")
    pooled, per_fold, oof = leave_one_run_out(X, y, run_ids, names)
    print(f"POOLED ROC AUC on held-out runs: {pooled:.4f}   <-- go/no-go")
    fold_aucs = np.array([a for _, a, _, _ in per_fold if np.isfinite(a)])
    print(f"mean per-fold AUC: {fold_aucs.mean():.4f} +/- {fold_aucs.std():.4f} (n={len(fold_aucs)} folds)")
    print("per-fold (held-out run, AUC, n_pos, n_frames):")
    for held, auc, npos, nfr in per_fold:
        print(f"  {held:<14} auc={auc:.4f}  pos={npos:<5} frames={nfr}")

    # ---- Step CV: heuristic baseline on the SAME held-out structure ----
    print("\n=== Step CV: heuristic baseline (valid-count + min tau) ===")
    # heuristic has no parameters to fit, so its 'held-out' AUC is just
    # its AUC per run pooled -- but to compare apples-to-apples with the
    # LR's pooled-OOF number, score every frame and pool.
    h = heuristic_score(X, names)
    h_pooled = roc_auc(y, h)
    print(f"POOLED ROC AUC (all frames): {h_pooled:.4f}")
    h_fold = np.array([roc_auc(y[run_ids == r], h[run_ids == r]) for r in sorted(set(run_ids))])
    h_fold = h_fold[np.isfinite(h_fold)]
    print(f"mean per-run AUC: {h_fold.mean():.4f} +/- {h_fold.std():.4f}")

    print(f"\n=== Verdict ===")
    print(f"LR pooled AUC     = {pooled:.4f}")
    print(f"heuristic pooled  = {h_pooled:.4f}")
    print(f"LR - heuristic    = {pooled - h_pooled:+.4f}")

    # ---- Step CU: interpretability, full-data fit ----
    print("\n=== Step CU: learned weights (full-data fit, standardized features) ===")
    scaler = NanStandardizer().fit(X)
    w = train_lr(scaler.transform(X), y)
    intercept, weights = w[0], w[1:]
    order = np.argsort(-np.abs(weights))
    print(f"intercept: {intercept:+.3f}")
    print("top 20 features by |weight| (standardized -> directly comparable):")
    for i in order[:20]:
        sign = "+" if weights[i] >= 0 else "-"
        print(f"  {weights[i]:+.3f}  {names[i]}")


if __name__ == "__main__":
    main()
