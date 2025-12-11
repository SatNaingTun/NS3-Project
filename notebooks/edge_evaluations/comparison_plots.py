import os
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

COMPARE_DIR = "../../outputs/comparison/"
PLOT_DIR = "../../outputs/comparison/plots/"

os.makedirs(PLOT_DIR, exist_ok=True)

plt.style.use("seaborn-v0_8-whitegrid")


# ============================================================
# Load comparison table
# ============================================================

def load_comparison_table():
    path = os.path.join(COMPARE_DIR, "model_comparison.csv")
    if not os.path.exists(path):
        raise FileNotFoundError("Run generate_model_comparison() first.")
    return pd.read_csv(path)


# ============================================================
# Plot 1 — RMSE Comparison
# ============================================================

def plot_rmse_comparison(df, showImage=False):
    plt.figure(figsize=(10, 6))

    sns.barplot(
        data=df,
        x="Metric",
        y="RMSE",
        hue="Model",
        palette="Set2"
    )

    plt.title("RMSE Comparison Across Models")
    plt.ylabel("RMSE ↓ (Lower is Better)")
    plt.xlabel("Metric")

    outfile = os.path.join(PLOT_DIR, "comparison_rmse.png")
    plt.tight_layout()
    plt.savefig(outfile, dpi=300)

    if showImage:
        plt.show()
    else:
        plt.close()

    print(f"Saved → {outfile}")


# ============================================================
# Plot 2 — Latency Comparison
# ============================================================

def plot_latency_comparison(df, showImage=False):
    plt.figure(figsize=(10, 6))

    sns.barplot(
        data=df,
        x="Metric",
        y="Latency_ms",
        hue="Model",
        palette="Set3"
    )

    plt.title("Inference Latency Comparison Across Models")
    plt.ylabel("Latency (ms) ↓ (Lower is Better)")
    plt.xlabel("Metric")

    outfile = os.path.join(PLOT_DIR, "comparison_latency.png")
    plt.tight_layout()
    plt.savefig(outfile, dpi=300)

    if showImage:
        plt.show()
    else:
        plt.close()

    print(f"Saved → {outfile}")


# ============================================================
# Plot 3 — Trade-Off Plot (RMSE vs Latency)
# ============================================================

def plot_tradeoff(df, showImage=False):
    plt.figure(figsize=(10, 6))

    for metric in df["Metric"].unique():
        sub = df[df["Metric"] == metric]

        plt.scatter(
            sub["Latency_ms"],
            sub["RMSE"],
            s=180,
            label=f"{metric}",
            alpha=0.7
        )

        # Add labels LR/RF/XGB on points
        for _, row in sub.iterrows():
            plt.text(
                row["Latency_ms"],
                row["RMSE"],
                row["Model"],
                fontsize=10,
                ha="center",
                va="bottom"
            )

    plt.title("Accuracy–Efficiency Trade-Off (Lower Left = Best)")
    plt.xlabel("Latency (ms) ↓")
    plt.ylabel("RMSE ↓")
    plt.grid(True, alpha=0.3)
    plt.legend(title="Metric")

    outfile = os.path.join(PLOT_DIR, "comparison_tradeoff.png")
    plt.tight_layout()
    plt.savefig(outfile, dpi=300)

    if showImage:
        plt.show()
    else:
        plt.close()

    print(f"Saved → {outfile}")


# ============================================================
# Run All Plots
# ============================================================

def generate_all_comparison_plots(showImage=False):
    df = load_comparison_table()

    print("Generating RMSE comparison plot...")
    plot_rmse_comparison(df, showImage=showImage)

    print("Generating latency comparison plot...")
    plot_latency_comparison(df, showImage=showImage)

    print("Generating accuracy–efficiency trade-off plot...")
    plot_tradeoff(df, showImage=showImage)

    print("\nAll comparison plots generated successfully!")
