import os
import time
import numpy as np
import xgboost as xgb


# =====================================================================
# 1. Load XGBoost Model from File
# =====================================================================

def load_xgb_model(model_path):
    """
    Loads an XGBoost model (old or new versions).
    """
    bst = xgb.Booster()
    bst.load_model(model_path)
    return bst


# =====================================================================
# 2. Extract Tree Structure Stats (depth, nodes, #trees)
# =====================================================================

def xgb_structure_stats(bst):
    """
    Extract tree complexity:
    - number of trees
    - total nodes
    - average nodes per tree
    - average tree depth
    - max tree depth
    """
    dump = bst.get_dump(with_stats=True)

    n_trees = len(dump)
    total_nodes = 0
    depths = []

    for tree_text in dump:
        lines = tree_text.strip().split("\n")

        node_count = len(lines)
        total_nodes += node_count

        # Estimate depth by counting indentation or node ids
        depth = max(line.count('\t') for line in lines)
        depths.append(depth)

    avg_depth = float(np.mean(depths))
    max_depth = max(depths)
    avg_nodes = total_nodes / n_trees

    return {
        "n_trees": n_trees,
        "total_nodes": total_nodes,
        "avg_nodes_per_tree": avg_nodes,
        "avg_tree_depth": avg_depth,
        "max_tree_depth": max_depth
    }


# =====================================================================
# 3. Model Size (bytes & MB)
# =====================================================================

def xgb_model_size(model_path):
    size = os.path.getsize(model_path)
    return size, size / (1024 * 1024)  # bytes, MB


# =====================================================================
# 4. Inference Latency (Measured)
# =====================================================================

def measure_xgb_latency(bst, X_sample, runs=200, warmup=20):
    """
    X_sample must be numpy array (1, n_features)
    """

    dtest = xgb.DMatrix(X_sample)

    # Warmup
    for _ in range(warmup):
        _ = bst.predict(dtest)

    # Measured runs
    t0 = time.perf_counter()
    for _ in range(runs):
        _ = bst.predict(dtest)
    t1 = time.perf_counter()

    return (t1 - t0) / runs


# =====================================================================
# 5. Estimated Energy
# =====================================================================

def estimate_energy(latency, power_W=2.0):
    """
    E = P × t
    """
    return latency * power_W


# =====================================================================
# 6. Main Edge Evaluation Function
# =====================================================================

def evaluate_xgb_edge(model_path, X_sample, assumed_power_W=2.0):
    bst = load_xgb_model(model_path)

    # Structure metrics
    struct = xgb_structure_stats(bst)

    # Model size
    size_bytes, size_mb = xgb_model_size(model_path)

    # Latency
    latency = measure_xgb_latency(bst, X_sample)

    # Energy
    energy = estimate_energy(latency, assumed_power_W)

    # Peak activation memory
    peak_activation_bytes = X_sample.to_numpy().nbytes

    return {
        "Model Path": model_path,
        "Trees": struct["n_trees"],
        "Total Nodes": struct["total_nodes"],
        "Average Nodes per Tree": struct["avg_nodes_per_tree"],
        "Average Tree Depth": struct["avg_tree_depth"],
        "Max Tree Depth": struct["max_tree_depth"],
        "Model Size (Bytes)": size_bytes,
        "Model Size (MB)": size_mb,
        "Latency (s)": latency,
        "Latency (ms)": latency * 1000,
        "Energy Estimated (J)": energy,
        "Peak Activation Memory (Bytes)": peak_activation_bytes
    }

