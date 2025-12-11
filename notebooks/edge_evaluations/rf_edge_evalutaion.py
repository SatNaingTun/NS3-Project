import os
import time
import joblib
import numpy as np
from sklearn.ensemble import RandomForestRegressor

import warnings
warnings.filterwarnings("ignore", message="X does not have valid feature names")

# =====================================================================
# 1. Load RF Model From Path
# =====================================================================

def load_rf_model(model_path):
    """
    Loads a RandomForestRegressor from disk.
    Returns the RF model object.
    """
    model = joblib.load(model_path)
    return model


# =====================================================================
# 2. Count Parameters, Nodes, Tree Depth
# =====================================================================

def rf_structure_stats(rf: RandomForestRegressor):
    """
    Extracts RF structural complexity:
    - number of trees
    - nodes per tree
    - average depth
    - total nodes
    """
    n_trees = len(rf.estimators_)
    nodes_per_tree = []
    depth_per_tree = []

    for tree in rf.estimators_:
        t = tree.tree_
        nodes_per_tree.append(t.node_count)
        depth_per_tree.append(t.max_depth)

    return {
        "n_trees": n_trees,
        "avg_nodes_per_tree": float(np.mean(nodes_per_tree)),
        "total_nodes": int(np.sum(nodes_per_tree)),
        "avg_tree_depth": float(np.mean(depth_per_tree)),
        "max_tree_depth": int(np.max(depth_per_tree)),
    }


# =====================================================================
# 3. Model Size in Bytes
# =====================================================================

def rf_model_size_bytes(model_path):
    """
    Model size on disk = true storage cost.
    """
    size = os.path.getsize(model_path)
    return size, size / (1024 * 1024)  # bytes, MB


# =====================================================================
# 4. Inference Latency Measurement
# =====================================================================

def measure_rf_latency(rf_model, X_sample, runs=200, warmup=20):
    """
    Measures RF inference latency using real CPU time.

    X_sample: shape (1, n_features)
    """

    # Warmup
    for _ in range(warmup):
        _ = rf_model.predict(X_sample)

    t0 = time.perf_counter()
    for _ in range(runs):
        _ = rf_model.predict(X_sample)
    t1 = time.perf_counter()

    return (t1 - t0) / runs


# =====================================================================
# 5. Energy Estimate
# =====================================================================

def estimate_energy(latency_s, assumed_power_W=2.0):
    """
    E = Power × Latency
    (Use estimated power if real measurement unavailable)
    """
    return latency_s * assumed_power_W


# =====================================================================
# 6. Full RF Edge Evaluation
# =====================================================================

def evaluate_rf_edge(model_path, X_sample, assumed_power_W=2.0):
    """
    Full RF efficiency evaluation.
    
    Inputs:
      model_path → path to saved RF model
      X_sample → array of shape (1, n_features)
      assumed_power_W → estimated CPU power draw
    
    Returns:
        dict of metrics
    """

    rf_model = load_rf_model(model_path)

    # Structure stats
    structure = rf_structure_stats(rf_model)

    # Model size
    size_bytes, size_mb = rf_model_size_bytes(model_path)

    # Latency (measured)
    latency = measure_rf_latency(rf_model, X_sample)

    # Energy
    energy = estimate_energy(latency, assumed_power_W)

    # Peak activations (RF = negligible, no tensors)
    peak_activation_bytes = X_sample.nbytes

    return {
        "Model Path": model_path,
        "Trees": structure["n_trees"],
        "Total Nodes": structure["total_nodes"],
        "Average Nodes per Tree": structure["avg_nodes_per_tree"],
        "Average Tree Depth": structure["avg_tree_depth"],
        "Max Tree Depth": structure["max_tree_depth"],
        "Model Size (Bytes)": size_bytes,
        "Model Size (MB)": size_mb,
        "Latency (s)": latency,
        "Latency (ms)": latency * 1000,
        "Energy Estimated (J)": energy,
        "Peak Activation Memory (Bytes)": peak_activation_bytes
    }
