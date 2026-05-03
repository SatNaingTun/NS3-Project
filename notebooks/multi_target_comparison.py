import os
import warnings
import numpy as np
import pandas as pd
from tqdm import tqdm

METRIC_SUFFIX = {
    "Throughput": "throughput",
    "Latency": "latency",
    "Loss": "loss",
}

def load_tradeoff_metrics(metric="Throughput"):
    assert metric in METRIC_SUFFIX, f"Invalid metric: {metric}"

    metric_key = METRIC_SUFFIX[metric]
    csv_path = f"../outputs/edge_analysis/{metric_key}_test_accuracy_edge_tradeoff.csv"

    df = pd.read_csv(csv_path)

    required = [
        "Model",
        "AccuracyError",
        "Latency_ms",
        "ModelSize_MB",
        "FLOPs",
        "Energy_J",
    ]

    missing = [c for c in required if c not in df.columns]
    if missing:
        raise KeyError(
            f"Missing required columns: {missing}\n"
            f"Available columns: {df.columns.tolist()}"
        )

    df = df.set_index("Model")
    return df

def plot_multi_target_accuracy_efficiency_normalized(
    save_dir="../outputs/edge_analysis/plots/combined/",
    showImage=False
):
    import os
    import numpy as np
    import matplotlib.pyplot as plt

    os.makedirs(save_dir, exist_ok=True)

    def normalize(s):
        return (s - s.min()) / (s.max() - s.min() + 1e-12)

    # Load trade-off CSVs
    df_t = load_tradeoff_metrics("Throughput")
    df_l = load_tradeoff_metrics("Latency")
    df_loss = load_tradeoff_metrics("Loss")

    models = df_t.index.tolist()
    x = np.arange(len(models))
    width = 0.22

    # Raw values
    acc_t = df_t["AccuracyError"]
    acc_l = df_l["AccuracyError"]
    acc_loss = df_loss["AccuracyError"]

    # Use average latency across all targets
    avg_latency = (
        df_t["Latency_ms"] +
        df_l["Latency_ms"] +
        df_loss["Latency_ms"]
    ) / 3

    # Normalize each metric independently
    acc_t_norm = normalize(acc_t)
    acc_l_norm = normalize(acc_l)
    acc_loss_norm = normalize(acc_loss)
    latency_norm = normalize(avg_latency)

    # Balanced trade-off score
    score = (
        acc_t_norm +
        acc_l_norm +
        acc_loss_norm +
        latency_norm
    )

    best_model = score.idxmin()
    best_idx = models.index(best_model)

    fig, ax1 = plt.subplots(figsize=(13, 6))

    b1 = ax1.bar(
        x - width,
        acc_t_norm,
        width,
        label="Throughput Error (normalized)",
        alpha=0.8
    )

    b2 = ax1.bar(
        x,
        acc_l_norm,
        width,
        label="Latency Error (normalized)",
        alpha=0.8
    )

    b3 = ax1.bar(
        x + width,
        acc_loss_norm,
        width,
        label="Loss Error (normalized)",
        alpha=0.8
    )

    # Highlight best model
    for bars in [b1, b2, b3]:
        bars[best_idx].set_edgecolor("black")
        bars[best_idx].set_linewidth(2.5)

    ax1.set_xlabel("Model")
    ax1.set_ylabel("Normalized Prediction Error")
    ax1.set_xticks(x)
    ax1.set_xticklabels(models, rotation=30, ha="right")
    ax1.set_ylim(0, 1.15)
    ax1.grid(True, axis="y", linestyle="--", alpha=0.4)

    ax2 = ax1.twinx()

    ax2.plot(
        x,
        latency_norm,
        color="black",
        marker="o",
        linewidth=2.5,
        label="Average Latency (normalized)"
    )

    ax2.scatter(
        x[best_idx],
        latency_norm.loc[best_model],
        s=140,
        facecolors="none",
        edgecolors="black",
        linewidths=2.5,
        zorder=10
    )

    ax2.set_ylabel("Normalized Average Latency")
    ax2.set_ylim(0, 1.15)

    plt.title(
        f"Multi-Target Accuracy–Efficiency Trade-off\n"
        f"Best Balanced Model: {best_model}",
        pad=12
    )

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()

    fig.legend(
        lines1 + lines2,
        labels1 + labels2,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.12),
        ncol=4,
        frameon=False
    )

    path = os.path.join(
        save_dir,
        "multi_target_accuracy_efficiency_normalized.png"
    )

    plt.tight_layout()
    plt.savefig(path, dpi=300, bbox_inches="tight")

    if showImage:
        plt.show()

    plt.close()

    print(f"Best balanced model: {best_model}")
    print(f"Saved: {path}")

def generate_accuracy_efficiency_table_csv(
    output_path="../outputs/edge_analysis/combined_accuracy_efficiency_table.csv",
    show=True
):
    import os
    import pandas as pd

    def normalize(s):
        return (s - s.min()) / (s.max() - s.min() + 1e-12)

    # -------------------------
    # Load data
    # -------------------------
    df_t = load_tradeoff_metrics("Throughput")
    df_l = load_tradeoff_metrics("Latency")
    df_loss = load_tradeoff_metrics("Loss")

    # -------------------------
    # Combine table
    # -------------------------
    table = pd.DataFrame(index=df_t.index)

    table["Throughput_Error"] = df_t["AccuracyError"]
    table["Latency_Error"] = df_l["AccuracyError"]
    table["Loss_Error"] = df_loss["AccuracyError"]

    # average inference latency (efficiency)
    table["Latency_ms"] = (
        df_t["Latency_ms"] +
        df_l["Latency_ms"] +
        df_loss["Latency_ms"]
    ) / 3

    # -------------------------
    # Normalize
    # -------------------------
    table["Throughput_Norm"] = normalize(table["Throughput_Error"])
    table["Latency_Norm"] = normalize(table["Latency_Error"])
    table["Loss_Norm"] = normalize(table["Loss_Error"])
    table["LatencyEff_Norm"] = normalize(table["Latency_ms"])

    # -------------------------
    # Final Score (equal weight)
    # -------------------------
    table["Score"] = (
        table["Throughput_Norm"] +
        table["Latency_Norm"] +
        table["Loss_Norm"] +
        table["LatencyEff_Norm"]
    )

    # Rank (1 = best)
    table["Rank"] = table["Score"].rank(method="min")

    # Sort best first
    table = table.sort_values("Score")

    # -------------------------
    # Save CSV
    # -------------------------
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    table.to_csv(output_path)

    if show:
        print("\nSaved combined table to:")
        print(output_path)
        print("\nPreview:\n")
        print(table.round(4))

    return table

def save_combined_accuracy_efficiency_table_image(
    csv_path="../outputs/edge_analysis/combined_accuracy_efficiency_table.csv",
    output_path="../outputs/edge_analysis/plots/combined/combined_accuracy_efficiency_table.png",
    top_n=None
):
    import os
    import pandas as pd
    import matplotlib.pyplot as plt

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    df = pd.read_csv(csv_path, index_col=0)

    cols = [
        "Throughput_Error",
        "Latency_Error",
        "Loss_Error",
        "Latency_ms",
        "Score",
        "Rank",
    ]

    table_df = df[cols].copy()

    if top_n is not None:
        table_df = table_df.head(top_n)

    table_df = table_df.round(4)
    table_df = table_df.rename(columns={
        "Throughput_Error": "Thr. Error",
        "Latency_Error": "Lat. Error",
        "Loss_Error": "Loss Error",
        "Latency_ms": "Avg Lat. (ms)",
        "Score": "Score",
        "Rank": "Rank",
    })

    fig_height = max(3, 0.45 * len(table_df) + 1.2)
    fig, ax = plt.subplots(figsize=(12, fig_height))
    ax.axis("off")

    tbl = ax.table(
        cellText=table_df.values,
        colLabels=table_df.columns,
        rowLabels=table_df.index,
        loc="center",
        cellLoc="center",
        rowLoc="center"
    )

    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10)
    tbl.scale(1, 1.4)

    plt.title("Combined Accuracy–Efficiency Ranking", pad=16)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close()

    print(f"Saved table image: {output_path}")

def plot_multi_target_accuracy_energy_normalized(
    save_dir="../outputs/edge_analysis/plots/combined/",
    showImage=False
):
    import os
    import numpy as np
    import matplotlib.pyplot as plt

    os.makedirs(save_dir, exist_ok=True)

    def normalize(s):
        return (s - s.min()) / (s.max() - s.min() + 1e-12)

    df_t = load_tradeoff_metrics("Throughput")
    df_l = load_tradeoff_metrics("Latency")
    df_loss = load_tradeoff_metrics("Loss")

    models = df_t.index.tolist()
    x = np.arange(len(models))
    width = 0.22

    # Accuracy errors
    thr_err = normalize(df_t["AccuracyError"])
    lat_err = normalize(df_l["AccuracyError"])
    loss_err = normalize(df_loss["AccuracyError"])

    # Average energy across targets
    avg_energy = (
        df_t["Energy_J"] +
        df_l["Energy_J"] +
        df_loss["Energy_J"]
    ) / 3

    energy_norm = normalize(avg_energy)

    # Combined score: lower error + lower energy is better
    score = thr_err + lat_err + loss_err + energy_norm
    best_model = score.idxmin()
    best_idx = models.index(best_model)

    fig, ax1 = plt.subplots(figsize=(13, 6))

    b1 = ax1.bar(x - width, thr_err, width, alpha=0.8, label="Throughput Error")
    b2 = ax1.bar(x, lat_err, width, alpha=0.8, label="Latency Error")
    b3 = ax1.bar(x + width, loss_err, width, alpha=0.8, label="Loss Error")

    for bars in [b1, b2, b3]:
        bars[best_idx].set_edgecolor("black")
        bars[best_idx].set_linewidth(2.5)

    ax1.set_xlabel("Model")
    ax1.set_ylabel("Normalized Prediction Error")
    ax1.set_xticks(x)
    ax1.set_xticklabels(models, rotation=30, ha="right")
    ax1.set_ylim(0, 1.15)
    ax1.grid(True, axis="y", linestyle="--", alpha=0.4)

    ax2 = ax1.twinx()
    ax2.plot(
        x,
        energy_norm,
        color="black",
        marker="o",
        linewidth=2.5,
        label="Average Energy"
    )

    ax2.scatter(
        x[best_idx],
        energy_norm.loc[best_model],
        s=140,
        facecolors="none",
        edgecolors="black",
        linewidths=2.5,
        zorder=10
    )

    ax2.set_ylabel("Normalized Average Energy")
    ax2.set_ylim(0, 1.15)

    plt.title(
        f"Multi-Target Accuracy–Energy Trade-off\n"
        f"Best Energy-Aware Model: {best_model}",
        pad=12
    )

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()

    fig.legend(
        lines1 + lines2,
        labels1 + labels2,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.12),
        ncol=4,
        frameon=False
    )

    path = os.path.join(
        save_dir,
        "multi_target_accuracy_energy_normalized.png"
    )

    plt.tight_layout()
    plt.savefig(path, dpi=300, bbox_inches="tight")

    if showImage:
        plt.show()

    plt.close()

    print(f"Best energy-aware model: {best_model}")
    print(f"Saved: {path}")

def plot_multi_target_accuracy_flops_normalized(
    save_dir="../outputs/edge_analysis/plots/combined/",
    showImage=False
):
    import os
    import numpy as np
    import matplotlib.pyplot as plt

    os.makedirs(save_dir, exist_ok=True)

    def normalize(s):
        return (s - s.min()) / (s.max() - s.min() + 1e-12)

    df_t = load_tradeoff_metrics("Throughput")
    df_l = load_tradeoff_metrics("Latency")
    df_loss = load_tradeoff_metrics("Loss")

    models = df_t.index.tolist()
    x = np.arange(len(models))
    width = 0.22

    thr_err = normalize(df_t["AccuracyError"])
    lat_err = normalize(df_l["AccuracyError"])
    loss_err = normalize(df_loss["AccuracyError"])

    avg_flops = (
        df_t["FLOPs"] +
        df_l["FLOPs"] +
        df_loss["FLOPs"]
    ) / 3

    flops_norm = normalize(avg_flops)

    score = thr_err + lat_err + loss_err + flops_norm
    best_model = score.idxmin()
    best_idx = models.index(best_model)

    fig, ax1 = plt.subplots(figsize=(13, 6))

    b1 = ax1.bar(x - width, thr_err, width, alpha=0.8, label="Throughput Error")
    b2 = ax1.bar(x, lat_err, width, alpha=0.8, label="Latency Error")
    b3 = ax1.bar(x + width, loss_err, width, alpha=0.8, label="Loss Error")

    for bars in [b1, b2, b3]:
        bars[best_idx].set_edgecolor("black")
        bars[best_idx].set_linewidth(2.5)

    ax1.set_xlabel("Model")
    ax1.set_ylabel("Normalized Prediction Error")
    ax1.set_xticks(x)
    ax1.set_xticklabels(models, rotation=30, ha="right")
    ax1.set_ylim(0, 1.15)
    ax1.grid(True, axis="y", linestyle="--", alpha=0.4)

    ax2 = ax1.twinx()
    ax2.plot(
        x,
        flops_norm,
        color="black",
        marker="o",
        linewidth=2.5,
        label="Average FLOPs"
    )

    ax2.scatter(
        x[best_idx],
        flops_norm.loc[best_model],
        s=140,
        facecolors="none",
        edgecolors="black",
        linewidths=2.5,
        zorder=10
    )

    ax2.set_ylabel("Normalized Average FLOPs")
    ax2.set_ylim(0, 1.15)

    plt.title(
        f"Multi-Target Accuracy–FLOPs Trade-off\n"
        f"Best Computation-Aware Model: {best_model}",
        pad=12
    )

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()

    fig.legend(
        lines1 + lines2,
        labels1 + labels2,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.12),
        ncol=4,
        frameon=False
    )

    path = os.path.join(
        save_dir,
        "multi_target_accuracy_flops_normalized.png"
    )

    plt.tight_layout()
    plt.savefig(path, dpi=300, bbox_inches="tight")

    if showImage:
        plt.show()

    plt.close()

    print(f"Best computation-aware model: {best_model}")
    print(f"Saved: {path}")

def plot_multi_target_accuracy_modelsize_normalized(
    save_dir="../outputs/edge_analysis/plots/combined/",
    showImage=False
):
    import os
    import numpy as np
    import matplotlib.pyplot as plt

    os.makedirs(save_dir, exist_ok=True)

    def normalize(s):
        return (s - s.min()) / (s.max() - s.min() + 1e-12)

    df_t = load_tradeoff_metrics("Throughput")
    df_l = load_tradeoff_metrics("Latency")
    df_loss = load_tradeoff_metrics("Loss")

    models = df_t.index.tolist()
    x = np.arange(len(models))
    width = 0.22

    thr_err = normalize(df_t["AccuracyError"])
    lat_err = normalize(df_l["AccuracyError"])
    loss_err = normalize(df_loss["AccuracyError"])

    avg_model_size = (
        df_t["ModelSize_MB"] +
        df_l["ModelSize_MB"] +
        df_loss["ModelSize_MB"]
    ) / 3

    size_norm = normalize(avg_model_size)

    score = thr_err + lat_err + loss_err + size_norm
    best_model = score.idxmin()
    best_idx = models.index(best_model)

    fig, ax1 = plt.subplots(figsize=(13, 6))

    b1 = ax1.bar(x - width, thr_err, width, alpha=0.8, label="Throughput Error")
    b2 = ax1.bar(x, lat_err, width, alpha=0.8, label="Latency Error")
    b3 = ax1.bar(x + width, loss_err, width, alpha=0.8, label="Loss Error")

    for bars in [b1, b2, b3]:
        bars[best_idx].set_edgecolor("black")
        bars[best_idx].set_linewidth(2.5)

    ax1.set_xlabel("Model")
    ax1.set_ylabel("Normalized Prediction Error")
    ax1.set_xticks(x)
    ax1.set_xticklabels(models, rotation=30, ha="right")
    ax1.set_ylim(0, 1.15)
    ax1.grid(True, axis="y", linestyle="--", alpha=0.4)

    ax2 = ax1.twinx()
    ax2.plot(
        x,
        size_norm,
        color="black",
        marker="o",
        linewidth=2.5,
        label="Average Model Size"
    )

    ax2.scatter(
        x[best_idx],
        size_norm.loc[best_model],
        s=140,
        facecolors="none",
        edgecolors="black",
        linewidths=2.5,
        zorder=10
    )

    ax2.set_ylabel("Normalized Average Model Size")
    ax2.set_ylim(0, 1.15)

    plt.title(
        f"Multi-Target Accuracy–Model Size Trade-off\n"
        f"Best Memory-Aware Model: {best_model}",
        pad=12
    )

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()

    fig.legend(
        lines1 + lines2,
        labels1 + labels2,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.12),
        ncol=4,
        frameon=False
    )

    path = os.path.join(
        save_dir,
        "multi_target_accuracy_modelsize_normalized.png"
    )

    plt.tight_layout()
    plt.savefig(path, dpi=300, bbox_inches="tight")

    if showImage:
        plt.show()

    plt.close()

    print(f"Best memory-aware model: {best_model}")
    print(f"Saved: {path}")