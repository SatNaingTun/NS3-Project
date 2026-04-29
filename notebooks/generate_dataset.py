# %%
import os
import re
import glob
import pandas as pd

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

    perf_files = sorted(glob.glob(os.path.join(root_dir, "*-perf.csv")))
    nd_files   = sorted(glob.glob(os.path.join(root_dir, "*-nodedensity.csv")))

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
        return

    final_df = pd.concat(all_rows, ignore_index=True)

    # Drop columns that are completely empty (some simulations may not produce BER etc)
    final_df = final_df.dropna(axis=1, how="all")

    # Sort for cleanliness
    final_df = final_df.sort_values(["RandSeed", "StartDateTime"]).reset_index(drop=True)

    final_df.to_csv(out_csv, index=False)
    print(f"\n✅ Final training dataset generated: {out_csv}")
    return final_df



Original_Data_Path = "../outputs/combine_analysis/final_training_dataset.csv"
Split_Data_Path = "../outputs/datasets/"




def split_by_simulation( train_ratio=0.7, val_ratio=0.15, test_ratio=0.15,Original_Data_Path=Original_Data_Path,Split_Data_Path=Split_Data_Path):
    """
    Safely split dataset by simulation run.
    Each run = unique (RandSeed, RunDateTime).
    Guarantees ≥1 simulation in train, validation, and test.
    """
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

import os
import numpy as np
import pandas as pd
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import joblib

MODEL_DIR = "../outputs/models/"
PRED_DIR  = "../outputs/models/predictions/"
Split_Data_Path  = "../outputs/datasets/"

os.makedirs(MODEL_DIR, exist_ok=True)
os.makedirs(PRED_DIR, exist_ok=True)

TARGETS = {
    "Throughput": "Perf_Avg_Throughput",
    "Latency":    "Perf_Avg_Latency",
    "Loss":       "Perf_Avg_Loss"
}

EXCLUDE_COLS = [
    "StartDateTime", "EndDateTime",
    "Perf_Filename", "ND_Filename",
    "RunDateTime"
]


def load_split_data(Split_Data_Path=Split_Data_Path):
    df_train = pd.read_csv(os.path.join(Split_Data_Path, "train.csv"))
    df_valid = pd.read_csv(os.path.join(Split_Data_Path, "valid.csv"))
    df_test  = pd.read_csv(os.path.join(Split_Data_Path, "test.csv"))
    return df_train, df_valid, df_test


def extract_xy(df):
    df = df.drop(columns=[c for c in EXCLUDE_COLS if c in df.columns], errors="ignore")
    df_num = df.select_dtypes(include=[np.number])

    # Features = all numeric except targets
    feature_cols = [c for c in df_num.columns if c not in TARGETS.values()]
    X = df_num[feature_cols].fillna(0)

    # Targets
    y = {name: df_num[col].fillna(0) for name, col in TARGETS.items()}

    return X, y, feature_cols


def scale_data(X_train, X_valid, X_test):
    scaler = StandardScaler()
    X_train_s = scaler.fit_transform(X_train)
    X_valid_s = scaler.transform(X_valid)
    X_test_s  = scaler.transform(X_test)

    joblib.dump(scaler, os.path.join(MODEL_DIR, "lr_scaler.pkl"))
    print("Scaler saved → outputs/models/lr_scaler.pkl")

    return X_train_s, X_valid_s, X_test_s

def train_single_lr_model(X_train, X_valid, X_test,
                          y_train, y_valid, y_test,
                          model_name):

    print("\n=================================================")
    print(f" TRAINING LR MODEL FOR: {model_name}")
    print("=================================================")

    model = LinearRegression()
    model.fit(X_train, y_train)

    # Save model
    model_path = os.path.join(MODEL_DIR, f"lr_model_{model_name.lower()}.pkl")
    joblib.dump(model, model_path)
    print(f"Model saved → {model_path}")

    # Predictions
    y_valid_pred = model.predict(X_valid)
    y_test_pred  = model.predict(X_test)

    # Validation metrics
    val_rmse = np.sqrt(mean_squared_error(y_valid, y_valid_pred))
    val_mae  = mean_absolute_error(y_valid, y_valid_pred)
    val_r2   = r2_score(y_valid, y_valid_pred)

    # Test metrics
    test_rmse = np.sqrt(mean_squared_error(y_test, y_test_pred))
    test_mae  = mean_absolute_error(y_test, y_test_pred)
    test_r2   = r2_score(y_test, y_test_pred)

    print("Validation Results:")
    print(f"  RMSE: {val_rmse:.4f},  MAE: {val_mae:.4f},  R²: {val_r2:.4f}")

    print("Test Results:")
    print(f"  RMSE: {test_rmse:.4f}, MAE: {test_mae:.4f}, R²: {test_r2:.4f}")

    # --------------------------
    # Save VALIDATION predictions
    # --------------------------
    valid_path = os.path.join(PRED_DIR, f"lr_{model_name}_valid_predictions.csv")
    pd.DataFrame({
        "y_true": y_valid,
        "y_pred": y_valid_pred
    }).to_csv(valid_path, index=False)
    print(f"Validation predictions saved → {valid_path}")

    # --------------------------
    # Save TEST predictions
    # --------------------------
    test_path = os.path.join(PRED_DIR, f"lr_{model_name}_test_predictions.csv")
    pd.DataFrame({
        "y_true": y_test,
        "y_pred": y_test_pred
    }).to_csv(test_path, index=False)
    print(f"Test predictions saved → {test_path}")

    return {
        "Model": model_name,
        "Val_RMSE": val_rmse,
        "Val_MAE": val_mae,
        "Val_R2": val_r2,
        "Test_RMSE": test_rmse,
        "Test_MAE": test_mae,
        "Test_R2": test_r2
    }



def generate_predictions(Split_Data_Path=Split_Data_Path):
    # Load datasets
    df_train, df_valid, df_test = load_split_data(Split_Data_Path)

    # Extract features and targets
    X_train, y_train_dict, feature_cols = extract_xy(df_train)
    X_valid, y_valid_dict, _ = extract_xy(df_valid)
    X_test,  y_test_dict, _  = extract_xy(df_test)

    # Scale
    X_train_s, X_valid_s, X_test_s = scale_data(X_train, X_valid, X_test)

    # Train LR models
    results = []
    for model_name in TARGETS.keys():
        r = train_single_lr_model(
            X_train_s, X_valid_s, X_test_s,
            y_train_dict[model_name],
            y_valid_dict[model_name],
            y_test_dict[model_name],
            model_name
        )
        results.append(r)

    print("\n\n==================== FINAL SUMMARY ====================")
    for r in results:
        print(r)


# ============================================================
# Run
# ============================================================

if __name__ == "__main__":
    build_training_dataset(
        root_dir="../outputs/csv/wifi-random",
        # out_csv="../outputs/combine_analysis/final_training_dataset.csv"
        out_csv=Original_Data_Path
    )
    split_by_simulation(Original_Data_Path=Original_Data_Path, Split_Data_Path=Split_Data_Path)
    generate_predictions(Split_Data_Path=Split_Data_Path)
