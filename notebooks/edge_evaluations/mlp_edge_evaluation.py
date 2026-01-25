import os
import time
import numpy as np
import torch
import torch.nn as nn


# ============================================================
# 1. Load MLP Model from Path
# ============================================================

class MLP(nn.Module):
    def __init__(self, input_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )

    def forward(self, x):
        return self.net(x)


def load_mlp_model(model_path, input_dim):
    """
    Load a saved MLP model (.pt) and return the PyTorch model object.
    """
    model = MLP(input_dim)
    state = torch.load(model_path, map_location="cpu")
    model.load_state_dict(state)
    model.eval()
    return model


# ============================================================
# 2. Count Parameters & Model Size
# ============================================================

def count_parameters(model):
    return sum(p.numel() for p in model.parameters())


def model_size_bytes(model):
    """
    Size = number_of_parameters * 4 bytes (FP32)
    """
    return count_parameters(model) * 4


# ============================================================
# 3. FLOPs Estimation (exact for fully-connected MLP)
# ============================================================

def compute_mlp_flops(model, input_dim):
    """
    Count FLOPs for:
      - Linear layers: 2 * In * Out  (multiply + add)
      - Activations: negligible
    """

    flops = 0
    prev_dim = input_dim

    for layer in model.net:
        if isinstance(layer, nn.Linear):
            out_dim = layer.out_features
            flops += 2 * prev_dim * out_dim
            prev_dim = out_dim

    return flops


# ============================================================
# 4. Latency Measurement
# ============================================================

def measure_latency(model, example_input, runs=200, warmup=20):
    """
    Measures average inference latency in seconds.
    """
    device = "cpu"
    model.to(device)

    example_input = example_input.to(device)

    # Warmup
    with torch.no_grad():
        for _ in range(warmup):
            _ = model(example_input)

        t0 = time.perf_counter()
        for _ in range(runs):
            _ = model(example_input)
        t1 = time.perf_counter()

    return (t1 - t0) / runs


# ============================================================
# 5. Energy Estimate
# ============================================================

def estimate_energy(latency_s, power_W=2.0):
    return latency_s * power_W


# ============================================================
# 6. Peak Activation Memory
# ============================================================

def estimate_peak_activation_memory(model, example_input):
    with torch.no_grad():
        out = model(example_input)
    return out.numel() * 4    # FP32


# ============================================================
# 7. Full MLP Edge Evaluation Pipeline
# ============================================================

# def evaluate_mlp_edge(model_path, input_dim, assumed_power_W=2.0):
#     """
#     Main evaluation function.
#     """

#     # ------------------------------
#     # Load model
#     # ------------------------------
#     model = load_mlp_model(model_path, input_dim)

#     # Dummy input for evaluation
#     example_input = torch.randn(1, input_dim)

#     # ------------------------------
#     # Compute metrics
#     # ------------------------------
#     params = count_parameters(model)
#     size_b = model_size_bytes(model)
#     size_mb = size_b / (1024 * 1024)

#     flops = compute_mlp_flops(model, input_dim)
#     latency = measure_latency(model, example_input)
#     energy = estimate_energy(latency, assumed_power_W)
#     peak_mem = estimate_peak_activation_memory(model, example_input)

#     return {
#         "Model Path": model_path,
#         "Parameters": params,
#         "Model Size (Bytes)": size_b,
#         "Model Size (MB)": size_mb,
#         "FLOPs": flops,
#         "Latency (s)": latency,
#         "Latency (ms)": latency * 1000,
#         "Energy Estimated (J)": energy,
#         "Peak Activation Memory (Bytes)": peak_mem
#     }


def evaluate_mlp_edge(model_path, assumed_power_W=2.0):
    """
    Edge evaluation for MLP.
    input_dim is inferred from checkpoint.
    """

    # ---- Load checkpoint ----
    state = torch.load(model_path, map_location="cpu")

    # ---- Infer input_dim ----
    input_dim = state["net.0.weight"].shape[1]

    # ---- Build model ----
    model = MLP(input_dim)
    model.load_state_dict(state)
    model.eval()

    example_input = torch.randn(1, input_dim)

    # ---- Metrics ----
    params = count_parameters(model)
    size_b = model_size_bytes(model)
    size_mb = size_b / (1024 * 1024)

    flops = compute_mlp_flops(model, input_dim)
    latency = measure_latency(model, example_input)
    energy = estimate_energy(latency, assumed_power_W)
    peak_mem = estimate_peak_activation_memory(model, example_input)

    return {
        "Input Dim": input_dim,
        "Parameters": params,
        "Model Size (Bytes)": size_b,
        "Model Size (MB)": size_mb,
        "FLOPs": flops,
        "Latency (s)": latency,
        "Latency (ms)": latency * 1000,
        "Energy Estimated (J)": energy,
        "Peak Activation Memory (Bytes)": peak_mem,
        "Model Path": model_path,
    }

