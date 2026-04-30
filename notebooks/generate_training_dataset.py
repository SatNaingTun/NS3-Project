# %%
import os
import re
import glob
import argparse
import pandas as pd
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def project_path(*parts):
    return PROJECT_ROOT.joinpath(*parts)

# ============================================================
# Helpers
# ============================================================

def extract_seed(filename):
    """Extract seed (integer) from NS-3 CSV file name."""
    m = re.search(r"seed(\d+)", filename)
    return int(m.group(1)) if m else None

def extract_run_datetime(filename):
    """Extract timestamp pattern from file name."""
    m = re.search(r"wifi-random-(.*)-seed", filename)
    return m.group(1) if m else "Unknown"

# ============================================================
# Loaders
# ============================================================

def load_perf_file(path):
    df = pd.read_csv(path)
    df["Perf_Filename"] = os.path.basename(path)
    df["RandSeed"] = extract_seed(path)
    df["RunDateTime"] = extract_run_datetime(path)
    
    # Force safe float columns
    float_cols = [
        "Throughput(Mbps)", "Latency_avg(ms)", "Jitter_avg(ms)",
        "PacketLoss(%)", "Duration_s"
    ]
    for c in float_cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    return df


def load_nodedensity_file(path):
    df = pd.read_csv(path)
    df["ND_Filename"] = os.path.basename(path)
    df["RandSeed"] = extract_seed(path)
    df["RunDateTime"] = extract_run_datetime(path)
    
    # Parse timestamp columns safely
    df["StartDateTime"] = pd.to_datetime(df["StartDateTime"], errors="coerce")
    df["EndDateTime"]   = pd.to_datetime(df["EndDateTime"], errors="coerce")
    
    return df


# ============================================================
# Merging Logic
# ============================================================

def merge_perf_and_nodedensity(df_perf, df_nd):
    """
    NS-3 perf rows = per-flow
    NS-3 nodedensity rows = per-interval (No FlowID)

    We expand perf metrics to interval level by computing:
       avg throughput across flows in simulation
       avg latency, avg jitter
    """

    # performance averages across flows
    perf_avg = df_perf.groupby(["RandSeed", "RunDateTime"]).agg({
        "Throughput(Mbps)": "mean",
        "Latency_avg(ms)":  "mean",
        "Jitter_avg(ms)":   "mean",
        "PacketLoss(%)":    "mean"
    }).reset_index().rename(columns={
        "Throughput(Mbps)": "Perf_Avg_Throughput",
        "Latency_avg(ms)":  "Perf_Avg_Latency",
        "Jitter_avg(ms)":   "Perf_Avg_Jitter",
        "PacketLoss(%)":    "Perf_Avg_Loss"
    })

    # direct join (1 perf summary per simulation)
    df = df_nd.merge(perf_avg, on=["RandSeed","RunDateTime"], how="left")

    return df


# ============================================================
# Master Pipeline
# ============================================================

def build_training_dataset(root_dir, out_csv="final_training_dataset.csv"):
    """
    root_dir: folder containing files:
        *-perf.csv
        *-nodedensity.csv
    """

    root_dir = Path(root_dir).expanduser()
    out_csv = Path(out_csv).expanduser()
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    perf_files = sorted(glob.glob(str(root_dir / "*-perf.csv")))
    nd_files   = sorted(glob.glob(str(root_dir / "*-nodedensity.csv")))

    print(f"Found {len(perf_files)} perf files")
    print(f"Found {len(nd_files)} nodedensity files")

    all_rows = []

    for pf in perf_files:
        seed = extract_seed(pf)
        run  = extract_run_datetime(pf)

        # find matching nodedensity file
        match_nd = None
        for nf in nd_files:
            if f"seed{seed}" in nf and run in nf:
                match_nd = nf
                break

        if match_nd is None:
            print(f"⚠ No matching nodedensity for {pf}")
            continue

        print(f"Merging:\n PERF = {os.path.basename(pf)}\n ND   = {os.path.basename(match_nd)}\n")

        df_perf = load_perf_file(pf)
        df_nd   = load_nodedensity_file(match_nd)

        merged = merge_perf_and_nodedensity(df_perf, df_nd)
        all_rows.append(merged)

    if not all_rows:
        print("❌ No merged data produced.")
        return None

    final_df = pd.concat(all_rows, ignore_index=True)

    # Drop columns that are completely empty (some simulations may not produce BER etc)
    final_df = final_df.dropna(axis=1, how="all")

    # Sort for cleanliness
    final_df = final_df.sort_values(["RandSeed", "StartDateTime"]).reset_index(drop=True)

    final_df.to_csv(out_csv, index=False)
    print(f"\n✅ Final training dataset generated: {out_csv}")
    return final_df



Original_Data_Path = project_path("outputs", "combine_analysis", "final_training_dataset.csv")
Split_Data_Path = project_path("outputs", "datasets")




def split_by_simulation( train_ratio=0.7, val_ratio=0.15, test_ratio=0.15,Original_Data_Path=Original_Data_Path,Split_Data_Path=Split_Data_Path):
    """
    Safely split dataset by simulation run.
    Each run = unique (RandSeed, RunDateTime).
    Guarantees ≥1 simulation in train, validation, and test.
    """
    Original_Data_Path = Path(Original_Data_Path).expanduser()
    Split_Data_Path = Path(Split_Data_Path).expanduser()

    if not Original_Data_Path.exists():
        raise FileNotFoundError(
            f"Training dataset not found: {Original_Data_Path}\n"
            "Run build_training_dataset() with a folder containing "
            "*-perf.csv and *-nodedensity.csv files first."
        )

    df = pd.read_csv(Original_Data_Path)
    os.makedirs(Split_Data_Path, exist_ok=True)

    runs = df.groupby(["RandSeed", "RunDateTime"]).size().reset_index()
    runs = runs.sample(frac=1, random_state=42)  # shuffle

    n = len(runs)

    if n < 3:
        raise ValueError(
            f"❌ Need at least 3 simulation runs for train/valid/test.\n"
            f"   Found only {n} runs.\n"
            f"   → Generate more NS-3 runs."
        )

    # Preliminary sizes
    n_train = max(1, int(train_ratio * n))
    n_val   = max(1, int(val_ratio * n))

    # Ensure test also has ≥1
    if n_train + n_val >= n:
        n_train = max(1, n - 2)
        n_val   = 1

    n_test = n - n_train - n_val

    train_runs = runs.iloc[:n_train]
    val_runs   = runs.iloc[n_train:n_train+n_val]
    test_runs  = runs.iloc[n_train+n_val:]

    def filter_subset(subset):
        return df.merge(
            subset[["RandSeed", "RunDateTime"]],
            on=["RandSeed", "RunDateTime"],
            how="inner"
        ).reset_index(drop=True)

    df_train = filter_subset(train_runs)
    df_val   = filter_subset(val_runs)
    df_test  = filter_subset(test_runs)

    print("\n=== Dataset Split Summary ===")
    print(f"Train: {df_train.shape} | simulations={len(train_runs)}")
    print(f"Valid: {df_val.shape}   | simulations={len(val_runs)}")
    print(f"Test : {df_test.shape}  | simulations={len(test_runs)}")

    df_train.to_csv(os.path.join(Split_Data_Path, "train.csv"), index=False)
    df_val.to_csv(os.path.join(Split_Data_Path, "valid.csv"), index=False)
    df_test.to_csv(os.path.join(Split_Data_Path, "test.csv"), index=False)

    print(f"\nSaved split datasets → {Split_Data_Path}")
    return df_train, df_val, df_test


# def split():
#     print("Loading:", Original_Data_Path)
#     df = pd.read_csv(Original_Data_Path)
#     split_by_simulation(df)



# ============================================================
# Run
# ============================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Build dataset from NS-3 wifi-random perf/nodedensity CSV files."
    )
    parser.add_argument(
        "--source-dir",
        default=str(project_path("outputs", "csv", "wifi-random")),
        help=(
            "Folder containing *-perf.csv and *-nodedensity.csv files. "
            "Quote the path if it contains spaces."
        ),
    )
    parser.add_argument(
        "--out-csv",
        default=str(Original_Data_Path),
        help="Merged dataset CSV output path.",
    )
    parser.add_argument(
        "--split-dir",
        default=str(Split_Data_Path),
        help="Folder for train.csv, valid.csv, and test.csv.",
    )
    # parser.add_argument(
    #     "--pred-dir",
    #     default=str(PRED_DIR),
    #     help="Folder for LR prediction CSVs.",
    # )
    args = parser.parse_args()

    Original_Data_Path = Path(args.out_csv).expanduser()
    Split_Data_Path = Path(args.split_dir).expanduser()
    # PRED_DIR = Path(args.pred_dir).expanduser()
    # os.makedirs(PRED_DIR, exist_ok=True)

    final_df = build_training_dataset(
        root_dir=Path(args.source_dir).expanduser(),
        out_csv=Original_Data_Path,
    )

    if final_df is None:
        raise SystemExit(
            "Cannot continue: no merged dataset was generated. "
            "Check --source-dir contains matching *-perf.csv and "
            "*-nodedensity.csv files."
        )

    split_by_simulation(Original_Data_Path=Original_Data_Path, Split_Data_Path=Split_Data_Path)
    
