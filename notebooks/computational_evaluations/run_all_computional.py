import os
import sys
import numpy as np
import pandas as pd
try:
    from tqdm import tqdm
except ImportError:
    def tqdm(iterable, **kwargs):
        return iterable

# Import evaluation modules
from lr_computational_evaluation import evaluate as eval_lr
from rf_computational_evaluation import evaluate as eval_rf
from xgb_computational_evaluation import evaluate as eval_xgb, load_model as load_xgb_model
from torch_computational_evaluation import evaluate as eval_torch

import joblib
import torch

EDGE_EVAL_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "edge_evaluations"
)
EDGE_EVAL_DIR = os.path.abspath(EDGE_EVAL_DIR)
if EDGE_EVAL_DIR not in sys.path:
    sys.path.append(EDGE_EVAL_DIR)

from mlp_edge_evaluation import MLP
from lstm_edge_evaluation import LSTMModel
from gru_edge_evaluation import GRUModel
from bilstm_edge_evaluation import BiLSTMModel
from transformer_edge_evaluation import TransformerRegressor
from cnn_lstm_edge_evaluation import CNNLSTM
from gnn_edge_evaluation import evaluate_gnn_edge


# ============================================================
# 1. Config
# ============================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(BASE_DIR, "..", ".."))

MODEL_DIR = os.path.join(PROJECT_ROOT, "outputs", "models")
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "outputs", "computational_analysis")
PLOTS_DIR = os.path.join(OUTPUT_DIR, "plots")

os.makedirs(OUTPUT_DIR, exist_ok=True)
os.makedirs(PLOTS_DIR, exist_ok=True)

INPUT_DIM = 73
SEQ_LEN = 10
REPEATS = 1000


# ============================================================
# 2. Load Models
# ============================================================

def load_models(metric_key):
    print("[INFO] Loading models...")

    models = {}

    models["LR"] = joblib.load(os.path.join(MODEL_DIR, f"lr_model_{metric_key}.pkl"))
    models["RF"] = joblib.load(os.path.join(MODEL_DIR, f"rf_model_{metric_key}.pkl"))
    models["XGBoost"] = load_xgb_model(os.path.join(MODEL_DIR, f"xgb_model_{metric_key}.json"))

    models["GRU"], _ = load_torch_model_from_state(
        "GRU",
        os.path.join(MODEL_DIR, f"gru_model_{metric_key}.pt")
    )
    models["LSTM"], _ = load_torch_model_from_state(
        "LSTM",
        os.path.join(MODEL_DIR, f"lstm_model_{metric_key}.pt")
    )
    models["CNN-LSTM"], _ = load_torch_model_from_state(
        "CNN-LSTM",
        os.path.join(MODEL_DIR, f"cnn_lstm_model_{metric_key}.pt")
    )
    models["Transformer"], _ = load_torch_model_from_state(
        "Transformer",
        os.path.join(MODEL_DIR, f"transformer_model_{metric_key}.pt")
    )

    print("[INFO] Models loaded.")
    return models


# ============================================================
# 3. Generate Inputs
# ============================================================

def generate_inputs():
    print("[INFO] Generating input samples...")

    X_static = np.random.randn(1, INPUT_DIM).astype(np.float32)
    X_seq = torch.randn(1, SEQ_LEN, INPUT_DIM)

    return X_static, X_seq


def load_torch_model_from_state(model_name, model_path):
    state = torch.load(model_path, map_location="cpu")

    if model_name == "MLP":
        input_dim = state["net.0.weight"].shape[1]
        model = MLP(input_dim)
    elif model_name == "LSTM":
        input_dim = state["lstm.weight_ih_l0"].shape[1]
        model = LSTMModel(input_dim)
    elif model_name == "GRU":
        input_dim = state["gru.weight_ih_l0"].shape[1]
        model = GRUModel(input_dim)
    elif model_name == "BiLSTM":
        input_dim = state["lstm.weight_ih_l0"].shape[1]
        model = BiLSTMModel(input_dim)
    elif model_name == "Transformer":
        input_dim = state["embed.weight"].shape[1]
        model = TransformerRegressor(input_dim)
    elif model_name == "CNN-LSTM":
        input_dim = state["cnn.0.weight"].shape[1]
        model = CNNLSTM(input_dim=input_dim)
    else:
        raise ValueError(f"Unsupported torch model: {model_name}")

    model.load_state_dict(state)
    model.eval()
    return model, input_dim


def evaluate_torch_state(model_name, model_path, repeats=1000):
    model, input_dim = load_torch_model_from_state(model_name, model_path)

    if model_name == "MLP":
        x_sample = torch.randn(1, input_dim)
    else:
        x_sample = torch.randn(1, SEQ_LEN, input_dim)

    return eval_torch(
        model=model,
        model_name=model_name,
        x_sample=x_sample,
        repeats=repeats
    )


def evaluate_gnn_state(model_path):
    try:
        raw = evaluate_gnn_edge(
            model_path=model_path,
            input_dim=3,
            hidden_dim=32,
            num_nodes=10
        )
        return {
            "Model": "GNN",
            "Model Size (MB)": raw["Model Size (MB)"],
            "Computational Latency (ms/sample)": raw["Latency (ms)"],
            "Energy (J/sample)": raw["Estimated Energy (J)"],
            "Parameters": raw["Parameters"]
        }
    except ModuleNotFoundError as e:
        print(f"[WARN] GNN skipped: {e}")
        return {
            "Model": "GNN",
            "Model Size (MB)": np.nan,
            "Computational Latency (ms/sample)": np.nan,
            "Energy (J/sample)": np.nan,
            "Parameters": np.nan
        }


# ============================================================
# 4. Run Evaluations (with tqdm)
# ============================================================

def run_evaluations(metric_key):
    print(f"[INFO] Running computational evaluation for: {metric_key}")

    X_static = np.random.randn(1, INPUT_DIM).astype(np.float32)

    

    tasks = [
        # -----------------------------
        # Linear Regression
        # -----------------------------
        ("LR", lambda: eval_lr(
            model_path=f"{MODEL_DIR}/lr_model_{metric_key}.pkl",
            X_sample=X_static,
            repeats=REPEATS
        )),

        # -----------------------------
        # Random Forest
        # -----------------------------
        ("RF", lambda: eval_rf(
            model_path=f"{MODEL_DIR}/rf_model_{metric_key}.pkl",
            X_sample=X_static,
            repeats=REPEATS
        )),

        # -----------------------------
        # XGBoost
        # -----------------------------
        ("XGBoost", lambda: eval_xgb(
            model_path=f"{MODEL_DIR}/xgb_model_{metric_key}.json",
            X_sample=X_static,
            repeats=REPEATS
        )),

        # -----------------------------
        # Neural models
        # -----------------------------
        ("MLP", lambda: evaluate_torch_state(
            "MLP",
            f"{MODEL_DIR}/mlp_model_{metric_key}.pt",
            repeats=REPEATS
        )),

        ("LSTM", lambda: evaluate_torch_state(
            "LSTM",
            f"{MODEL_DIR}/lstm_model_{metric_key}.pt",
            repeats=REPEATS
        )),

        ("GRU", lambda: evaluate_torch_state(
            "GRU",
            f"{MODEL_DIR}/gru_model_{metric_key}.pt",
            repeats=REPEATS
        )),

        ("BiLSTM", lambda: evaluate_torch_state(
            "BiLSTM",
            f"{MODEL_DIR}/bilstm_model_{metric_key}.pt",
            repeats=REPEATS
        )),

        ("Transformer", lambda: evaluate_torch_state(
            "Transformer",
            f"{MODEL_DIR}/transformer_model_{metric_key}.pt",
            repeats=REPEATS
        )),

        ("CNN-LSTM", lambda: evaluate_torch_state(
            "CNN-LSTM",
            f"{MODEL_DIR}/cnn_lstm_model_{metric_key}.pt",
            repeats=REPEATS
        )),

        # -----------------------------
        # GNN (special input)
        # -----------------------------
        ("GNN", lambda: evaluate_gnn_state(
            f"{MODEL_DIR}/gnn_model_{metric_key}.pt"
        )),
    ]

    results = []

    for name, task in tqdm(tasks, desc=f"{metric_key} models"):
        try:
            results.append(task())
        except Exception as e:
            print(f"[ERROR] {name}: {e}")

    return results


# ============================================================
# 5. Save CSV
# ============================================================

def save_results(results):
    df = pd.DataFrame(results)
    df = df.drop(columns=["Model Path"], errors="ignore")

    output_csv = os.path.join(OUTPUT_DIR, "computational_metrics.csv")
    df.to_csv(output_csv, index=False)

    print(f"[INFO] Saved: {output_csv}")
    return df


# ============================================================
# 6. Merge with Accuracy
# ============================================================

def merge_with_accuracy(df):
    try:
        acc_path = "../outputs/edge_analysis/rmse_test_accuracy_edge_tradeoff.csv"
        acc_df = pd.read_csv(acc_path)

        merged = acc_df.merge(df, on="Model", how="inner")

        out_path = os.path.join(OUTPUT_DIR, "rmse_computational_tradeoff.csv")
        merged.to_csv(out_path, index=False)

        print(f"[INFO] Merged saved: {out_path}")
        return merged

    except Exception as e:
        print("[WARN] Merge skipped:", e)
        return None


# ============================================================
# 7. Plot
# ============================================================

def plot_bar_line(df):
    import matplotlib.pyplot as plt
    import numpy as np

    models = df["Model"]
    rmse = df["RMSE"]
    latency = df["Computational Latency (ms/sample)"]

    x = np.arange(len(models))

    fig, ax1 = plt.subplots(figsize=(12, 6))

    ax1.bar(x, rmse)
    ax1.set_ylabel("RMSE")

    ax2 = ax1.twinx()
    ax2.plot(x, latency, marker="o")
    ax2.set_ylabel("Latency (ms/sample)")

    ax1.set_xticks(x)
    ax1.set_xticklabels(models, rotation=30)

    plt.title("Accuracy vs Computational Latency")

    save_path = os.path.join(PLOTS_DIR, "rmse_vs_computational_latency.png")
    plt.savefig(save_path, dpi=300, bbox_inches="tight")

    print(f"[INFO] Plot saved: {save_path}")
    plt.show()


# ============================================================
# 8. MAIN
# ============================================================

def main():
    metric_key = "throughput"   # change as needed
    results = run_evaluations(metric_key)

    df = pd.DataFrame(results)
    df = df.drop(columns=["Model Path"], errors="ignore")

    output_csv = f"{OUTPUT_DIR}/{metric_key}_computational_metrics.csv"
    df.to_csv(output_csv, index=False)

    print(f"[INFO] Saved: {output_csv}")


if __name__ == "__main__":
    main()
