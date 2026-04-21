import os
import time
import joblib
import numpy as np
from sklearn.ensemble import RandomForestRegressor

import warnings
warnings.filterwarnings("ignore", message="X does not have valid feature names")


# ============================================================
# 1. Load RF Model
# ============================================================

def load_rf_model(model_path):
    """
    Load RandomForest model from disk.
    """
    return joblib.load(model_path)


# ============================================================
# 2. RF Structural Complexity
# ============================================================

def rf_structure_stats(rf: RandomForestRegressor):
    """
    Extracts structural properties of RF.
    """
    n_trees = len(rf.estimators_)
    nodes_per_tree = []
    depth_per_tree = []

    for tree in rf.estimators_:
        t = tree.tree_
        nodes_per_tree.append(t.node_count)
        depth_per_tree.append(t.max_depth)

    return {
        "Trees": n_trees,
        "Average Nodes per Tree": float(np.mean(nodes_per_tree)),
        "Total Nodes": int(np.sum(nodes_per_tree)),
        "Average Tree Depth": float(np.mean(depth_per_tree)),
        "Max Tree Depth": int(np.max(depth_per_tree)),
    }


# ============================================================
# 3. Model Size
# ============================================================

def rf_model_size_bytes(model_path):
    """
    Returns model size on disk.
    """
    size = os.path.getsize(model_path)
    return size, size / (1024 * 1024)


# ============================================================
# 4. Computational Latency (Batch-Based)
# ============================================================

def measure_rf_computational_latency(
    rf_model,
    X_sample,
    repeats=1000,
    warmup=3
):
    """
    Measures computational latency using batch inference.

    This removes Python loop overhead and estimates
    per-sample computational cost.

    Parameters
    ----------
    X_sample : shape (1, n_features)
    repeats : batch size for timing
    """

    # Force single-thread inference to avoid Windows joblib/multiprocessing
    # permission issues in restricted environments.
    rf_model.n_jobs = 1

    # Create batch
    X_batch = np.repeat(X_sample, repeats, axis=0)

    # Warmup
    for _ in range(warmup):
        _ = rf_model.predict(X_batch)

    # Measure
    t0 = time.perf_counter()
    _ = rf_model.predict(X_batch)
    t1 = time.perf_counter()

    latency_per_sample = (t1 - t0) / repeats
    return latency_per_sample


# ============================================================
# 5. Energy Estimate
# ============================================================

def estimate_energy(latency_s, assumed_power_W=2.0):
    """
    E = P × t
    """
    return latency_s * assumed_power_W


# ============================================================
# 6. Full Computational Evaluation
# ============================================================

def evaluate(
    model_path,
    X_sample,
    repeats=1000,
    assumed_power_W=2.0
):
    """
    Full computational efficiency evaluation for RF.

    Returns:
        dict of metrics
    """

    rf_model = load_rf_model(model_path)

    # Structure
    structure = rf_structure_stats(rf_model)

    # Model size
    size_bytes, size_mb = rf_model_size_bytes(model_path)

    # Computational latency
    latency = measure_rf_computational_latency(
        rf_model,
        X_sample,
        repeats=repeats
    )

    # Energy
    energy = estimate_energy(latency, assumed_power_W)

    # Peak activation (minimal for RF)
    peak_activation_bytes = X_sample.nbytes

    return {
        "Model": "RF",
        "Model Path": model_path,

        # Complexity
        "Trees": structure["Trees"],
        "Total Nodes": structure["Total Nodes"],
        "Average Nodes per Tree": structure["Average Nodes per Tree"],
        "Average Tree Depth": structure["Average Tree Depth"],
        "Max Tree Depth": structure["Max Tree Depth"],

        # Memory
        "Model Size (Bytes)": size_bytes,
        "Model Size (MB)": size_mb,

        # Computational latency
        "Computational Latency (s/sample)": latency,
        "Computational Latency (ms/sample)": latency * 1000,

        # Energy
        "Energy Estimated (J/sample)": energy,

        # Misc
        "Peak Activation Memory (Bytes)": peak_activation_bytes,
        "Batch Size (Repeats)": repeats
    }
