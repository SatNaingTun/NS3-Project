"""
Compute precision, recall, F1, and accuracy from regression predictions.

Important:
    Precision and recall are classification metrics. The thesis models are
    regression models, so this script first converts each continuous target
    into a binary class using a threshold.

Default binary interpretation:
    Positive class = "poor QoS / problematic network condition"

    Throughput: positive if throughput is LOW
    Latency:    positive if latency is HIGH
    Loss:       positive if loss is HIGH

Default thresholds:
    Throughput: 25th percentile of y_true
    Latency:    75th percentile of y_true
    Loss:       75th percentile of y_true

For a thesis defense, always report the threshold used. Precision/recall
values change if the threshold changes.
"""

from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.metrics import (
    accuracy_score,
    confusion_matrix,
    f1_score,
    precision_score,
    recall_score,
)


# ---------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------

cwd = Path.cwd()
PROJECT_ROOT = cwd.parent if cwd.name == "notebooks" else cwd

PRED_DIR = PROJECT_ROOT / "outputs" / "models" / "predictions"
OUT_DIR = PROJECT_ROOT / "outputs" / "classification_metrics"
OUT_DIR.mkdir(parents=True, exist_ok=True)


# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

SPLIT = "test"  # use "valid" or "test"

MODELS = {
    "LR": ["lr"],
    "RF": ["rf"],
    "XGB": ["xgb", "xgboost"],
    "MLP": ["mlp"],
    "LSTM": ["lstm"],
    "GRU": ["gru"],
    "BiLSTM": ["bilstm", "bi_lstm"],
    "Transformer": ["transformer"],
    "CNN-LSTM": ["cnn_lstm", "cnn-lstm", "cnnlstm"],
    "GNN": ["gnn"],
}

TARGETS = ["Throughput", "Latency", "Loss"]

# Positive class means "bad / problematic".
DIRECTION = {
    "Throughput": "low_is_bad",
    "Latency": "high_is_bad",
    "Loss": "high_is_bad",
}

# If a value is None, the script uses DEFAULT_QUANTILES.
# You may replace None with domain-specific thresholds, for example:
# CUSTOM_THRESHOLDS = {"Throughput": 0.30, "Latency": 10.0, "Loss": 5.0}
CUSTOM_THRESHOLDS = {
    "Throughput": None,
    "Latency": None,
    "Loss": None,
}

DEFAULT_QUANTILES = {
    "Throughput": 0.25,
    "Latency": 0.75,
    "Loss": 0.75,
}


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def prediction_path(model_prefix: str, target: str, split: str) -> Path:
    return PRED_DIR / f"{model_prefix}_{target}_{split}_predictions.csv"


def find_prediction_path(model_name: str, target: str, split: str) -> Path | None:
    """
    Find a prediction CSV even if the model prefix has a small naming variation.

    Expected project pattern:
        {model_prefix}_{Target}_{split}_predictions.csv

    Examples:
        lr_Throughput_test_predictions.csv
        xgb_Loss_test_predictions.csv
        cnn_lstm_Latency_test_predictions.csv
    """
    for prefix in MODELS[model_name]:
        path = prediction_path(prefix, target, split)
        if path.exists():
            return path
    return None


def print_prediction_file_coverage(split: str) -> None:
    print(f"\nPrediction file coverage for split='{split}'")
    print("-" * 72)
    for target in TARGETS:
        status = []
        for model_name in MODELS:
            path = find_prediction_path(model_name, target, split)
            status.append(f"{model_name}:{'OK' if path else 'MISSING'}")
        print(f"{target:10s} | " + "  ".join(status))
    print("-" * 72)


def load_prediction_file(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    required = {"y_true", "y_pred"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"{path} is missing columns: {sorted(missing)}")

    df = df[["y_true", "y_pred"]].copy()
    df["y_true"] = pd.to_numeric(df["y_true"], errors="coerce")
    df["y_pred"] = pd.to_numeric(df["y_pred"], errors="coerce")
    df = df.dropna(subset=["y_true", "y_pred"])
    return df


def get_reference_y_true(target: str, split: str) -> np.ndarray:
    """
    Use the first available model file to define the target threshold.
    All models for the same target should share the same y_true test set.
    """
    for model_name in MODELS:
        path = find_prediction_path(model_name, target, split)
        if path is not None:
            return load_prediction_file(path)["y_true"].to_numpy()
    raise FileNotFoundError(f"No prediction files found for {target}-{split}")


def threshold_for_target(target: str, y_true: np.ndarray) -> float:
    custom = CUSTOM_THRESHOLDS.get(target)
    if custom is not None:
        return float(custom)
    return float(np.quantile(y_true, DEFAULT_QUANTILES[target]))


def binarize(values: np.ndarray, target: str, threshold: float) -> np.ndarray:
    if DIRECTION[target] == "low_is_bad":
        return (values <= threshold).astype(int)
    if DIRECTION[target] == "high_is_bad":
        return (values >= threshold).astype(int)
    raise ValueError(f"Unknown direction for target: {target}")


def evaluate_binary_metrics(y_true_cont, y_pred_cont, target: str, threshold: float):
    y_true_bin = binarize(y_true_cont, target, threshold)
    y_pred_bin = binarize(y_pred_cont, target, threshold)
    tn, fp, fn, tp = confusion_matrix(
        y_true_bin,
        y_pred_bin,
        labels=[0, 1],
    ).ravel()

    return {
        "Precision": precision_score(y_true_bin, y_pred_bin, zero_division=0),
        "Recall": recall_score(y_true_bin, y_pred_bin, zero_division=0),
        "F1": f1_score(y_true_bin, y_pred_bin, zero_division=0),
        "Accuracy": accuracy_score(y_true_bin, y_pred_bin),
        "Positive Support": int(y_true_bin.sum()),
        "Predicted Positive": int(y_pred_bin.sum()),
        "TN": int(tn),
        "FP": int(fp),
        "FN": int(fn),
        "TP": int(tp),
        "Total Samples": int(len(y_true_bin)),
        "y_true_min": float(np.min(y_true_cont)),
        "y_true_max": float(np.max(y_true_cont)),
        "y_pred_min": float(np.min(y_pred_cont)),
        "y_pred_max": float(np.max(y_pred_cont)),
    }


# ---------------------------------------------------------------------
# Main evaluation
# ---------------------------------------------------------------------

rows = []

print_prediction_file_coverage(SPLIT)

for target in TARGETS:
    reference_y_true = get_reference_y_true(target, SPLIT)
    threshold = threshold_for_target(target, reference_y_true)

    for model_name in MODELS:
        path = find_prediction_path(model_name, target, SPLIT)
        if path is None:
            aliases = ", ".join(MODELS[model_name])
            print(
                f"[WARN] Missing prediction file for {model_name}-{target}-{SPLIT}. "
                f"Tried prefixes: {aliases}"
            )
            continue

        df = load_prediction_file(path)
        metrics = evaluate_binary_metrics(
            y_true_cont=df["y_true"].to_numpy(),
            y_pred_cont=df["y_pred"].to_numpy(),
            target=target,
            threshold=threshold,
        )

        rows.append({
            "Target": target,
            "Model": model_name,
            "Split": SPLIT,
            "Positive Class": (
                f"{target} <= threshold"
                if DIRECTION[target] == "low_is_bad"
                else f"{target} >= threshold"
            ),
            "Threshold": threshold,
            **metrics,
            "Prediction File": str(path.relative_to(PROJECT_ROOT)),
        })


results = pd.DataFrame(rows)

if results.empty:
    raise RuntimeError(f"No results produced. Check prediction files in {PRED_DIR}")

out_csv = OUT_DIR / f"precision_recall_binary_{SPLIT}.csv"
results.to_csv(out_csv, index=False)

print("\nPrecision / Recall from thresholded regression predictions")
print(f"Positive class definition: poor QoS / problematic condition")
print(f"Saved CSV: {out_csv}")

for target in TARGETS:
    print(f"\n=== {target} ({SPLIT}) ===")
    show_cols = [
        "Model",
        "Threshold",
        "Precision",
        "Recall",
        "F1",
        "Accuracy",
        "Positive Support",
        "Predicted Positive",
        "TP",
        "FP",
        "FN",
        "Total Samples",
    ]
    target_df = results[results["Target"] == target].copy()
    target_df = target_df.sort_values(["F1", "Precision", "Recall"], ascending=False)
    print(target_df[show_cols].to_string(index=False))


# ---------------------------------------------------------------------
# Optional quick plot
# ---------------------------------------------------------------------

try:
    import matplotlib.pyplot as plt

    for target in TARGETS:
        target_df = results[results["Target"] == target].copy()
        target_df = target_df.sort_values("F1", ascending=False)

        x = np.arange(len(target_df))
        width = 0.28

        plt.figure(figsize=(12, 5))
        bars_precision = plt.bar(
            x - width,
            target_df["Precision"],
            width,
            label="Precision",
        )
        bars_recall = plt.bar(
            x,
            target_df["Recall"],
            width,
            label="Recall",
        )
        bars_f1 = plt.bar(
            x + width,
            target_df["F1"],
            width,
            label="F1",
        )

        for bars in [bars_precision, bars_recall, bars_f1]:
            for bar in bars:
                height = bar.get_height()
                if height == 0:
                    plt.text(
                        bar.get_x() + bar.get_width() / 2,
                        0.015,
                        "0",
                        ha="center",
                        va="bottom",
                        fontsize=8,
                        rotation=90,
                    )
        plt.xticks(x, target_df["Model"], rotation=35, ha="right")
        plt.ylim(0, 1.05)
        plt.ylabel("Score")
        plt.title(f"Precision / Recall / F1 from Thresholded Regression - {target}")
        plt.legend()
        plt.tight_layout()

        out_png = OUT_DIR / f"precision_recall_f1_{target.lower()}_{SPLIT}.png"
        plt.savefig(out_png, dpi=300)
        plt.close()
        print(f"Saved plot: {out_png}")

except Exception as exc:
    print(f"[WARN] Plotting skipped: {exc}")
