import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

plt.style.use("seaborn-v0_8")

# ===============================================================
# Load Unified Metrics
# ===============================================================

def load_metrics(csv_path="../outputs/edge_analysis/unified_edge_metrics.csv"):
    df = pd.read_csv(csv_path, index_col=0)

    # Validate required columns
    required = [
        "Model Size (MB)",
        "Latency (ms)",
        "Energy Estimated (J)",
        "FLOPs",
    ]

    missing = [c for c in required if c not in df.columns]
    if missing:
        raise KeyError(f"Missing required columns: {missing}\nAvailable: {df.columns.tolist()}")

    return df



# ===============================================================
# 1. Model Size vs Latency
# ===============================================================

def plot_size_vs_latency(df, save_dir,showImage=False):
    plt.figure(figsize=(10, 6))

    plt.bar(df.index, df["Model Size (MB)"], alpha=0.6, label="Model Size (MB)")
    plt.plot(df.index, df["Latency (ms)"], color="red", marker="o", linewidth=2, label="Latency (ms)")

    plt.title("Model Size vs Inference Latency")
    plt.xlabel("Model")
    plt.ylabel("Value")
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.legend()

    path = os.path.join(save_dir, "size_vs_latency.png")
    plt.savefig(path, dpi=300, bbox_inches="tight")
    if showImage:
        plt.show()
    plt.close()



# ===============================================================
# 2. Energy per Inference
# ===============================================================

def plot_energy(df, save_dir, showImage=False):
    plt.figure(figsize=(10, 6))

    plt.bar(df.index, df["Energy Estimated (J)"], color="orange", alpha=0.7)

    plt.title("Energy per Inference (J)")
    plt.xlabel("Model")
    plt.ylabel("Energy (J)")
    plt.grid(True, linestyle="--", alpha=0.4)

    path = os.path.join(save_dir, "energy_per_inference.png")
    plt.savefig(path, dpi=300, bbox_inches="tight")
    if showImage:
        plt.show()
    plt.close()



# ===============================================================
# 3. FLOPs Comparison
# ===============================================================

def plot_flops(df, save_dir,showImage=False):
    plt.figure(figsize=(10, 6))

    plt.bar(df.index, df["FLOPs"], color="green", alpha=0.7)

    plt.title("Model FLOPs per Inference")
    plt.xlabel("Model")
    plt.ylabel("FLOPs")
    plt.grid(True, linestyle="--", alpha=0.4)

    path = os.path.join(save_dir, "flops_comparison.png")
    plt.savefig(path, dpi=300, bbox_inches="tight")
    if showImage:
        plt.show()
    plt.close()



# ===============================================================
# 4. Radar Chart (Normalized Multi-Metric Comparison)
# ===============================================================

def plot_radar(df, save_dir, showImage=False):

    metrics = ["Model Size (MB)", "Latency (ms)", "Energy Estimated (J)", "FLOPs"]
    labels = df.index.tolist()

    # normalization 0-1
    norm_df = (df[metrics] - df[metrics].min()) / (df[metrics].max() - df[metrics].min())

    angles = np.linspace(0, 2*np.pi, len(metrics), endpoint=False).tolist()
    angles += angles[:1]

    plt.figure(figsize=(8, 8))
    ax = plt.subplot(111, polar=True)

    for model in labels:
        values = norm_df.loc[model].tolist()
        values += values[:1]

        ax.plot(angles, values, linewidth=2, label=model)
        ax.fill(angles, values, alpha=0.1)

    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(metrics)
    plt.title("Normalized Model Comparison (Radar Chart)")
    plt.legend(bbox_to_anchor=(1.2, 1.1))

    path = os.path.join(save_dir, "radar_comparison.png")
    plt.savefig(path, dpi=300, bbox_inches="tight")
    if showImage:
        plt.show()
    plt.close()



# ===============================================================
# 5. Latency–Energy Pareto Plot
# ===============================================================

# def plot_latency_energy(df, save_dir, showImage=False):
#     plt.figure(figsize=(10, 6))

#     x = df["Latency (ms)"]
#     y = df["Energy Estimated (J)"]
#     sizes = df["Model Size (MB)"] * 20

#     plt.scatter(x, y, s=sizes, alpha=0.7)

#     for model in df.index:
#         plt.text(x[model] + 0.5, y[model] + 0.0002, model, fontsize=9)

#     plt.title("Latency vs Energy per Inference (Bubble = Model Size)")
#     plt.xlabel("Latency (ms)")
#     plt.ylabel("Energy (J)")
#     plt.grid(True, linestyle="--", alpha=0.4)

#     path = os.path.join(save_dir, "latency_vs_energy.png")
#     plt.savefig(path, dpi=300, bbox_inches="tight")
#     if showImage:
#         plt.show()
#     plt.close()

def plot_latency_energy(df, save_dir, showImage=False):
    plt.figure(figsize=(10, 6))

    x = df["Latency (ms)"]
    y = df["Energy Estimated (J)"]
    sizes = df["Model Size (MB)"] * 25

    plt.scatter(x, y, s=sizes, alpha=0.75)

    for model in df.index:
        plt.annotate(
            model,
            (x[model], y[model]),
            textcoords="offset points",
            xytext=(5, 5),
            fontsize=9
        )

    # -------------------------------
    # Log scaling (key improvement)
    # -------------------------------
    plt.xscale("log")
    plt.yscale("log")

    plt.title("Latency vs Energy per Inference (Bubble = Model Size)")
    plt.xlabel("Latency (ms, log scale)")
    plt.ylabel("Energy per Inference (J, log scale)")
    plt.grid(True, which="both", linestyle="--", alpha=0.4)

    path = os.path.join(save_dir, "latency_vs_energy_log.png")
    plt.savefig(path, dpi=300, bbox_inches="tight")

    if showImage:
        plt.show()
    plt.close()




# ===============================================================
# MAIN GENERATOR
# ===============================================================

def generate_edge_comparison_plots(showImage=False):
    save_dir = "../outputs/edge_analysis/plots/"
    os.makedirs(save_dir, exist_ok=True)

    df = load_metrics()

    plot_size_vs_latency(df, save_dir, showImage)
    plot_energy(df, save_dir, showImage)
    plot_flops(df, save_dir, showImage)
    plot_radar(df, save_dir, showImage)
    plot_latency_energy(df, save_dir, showImage)

    print("\n✔ All comparison plots generated successfully.")



# ===============================================================
# Run
# ===============================================================
if __name__ == "__main__":
    generate_edge_comparison_plots()
