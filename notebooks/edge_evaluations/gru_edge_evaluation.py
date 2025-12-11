import os
import time
import torch
import torch.nn as nn
import numpy as np


# ============================================================
# 1. GRU Model (must match your training architecture)
# ============================================================

class GRUModel(nn.Module):
    def __init__(self, input_dim, hidden_dim=64, num_layers=2):
        super().__init__()

        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers

        self.gru = nn.GRU(
            input_size=input_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True
        )

        self.fc = nn.Linear(hidden_dim, 1)

    def forward(self, x):
        out, _ = self.gru(x)
        last = out[:, -1, :]
        return self.fc(last)


# ============================================================
# 2. Load GRU model weights
# ============================================================

def load_gru_model(model_path, input_dim):
    model = GRUModel(input_dim)
    state = torch.load(model_path, map_location="cpu")
    model.load_state_dict(state)
    model.eval()
    return model


# ============================================================
# 3. Parameter count + Model size
# ============================================================

def count_parameters(model):
    return sum(p.numel() for p in model.parameters())


def model_size_bytes(model):
    return count_parameters(model) * 4     # FP32 → 4 bytes / param


# ============================================================
# 4. FLOPs calculation for GRU
# ============================================================

def compute_gru_flops(model, sequence_len=1):
    """
    GRU FLOPs:

    GRU cell equations:
        r = σ(Wr*x + Ur*h + br)
        z = σ(Wz*x + Uz*h + bz)
        h~ = tanh(Wh*x + Uh*(r ⊙ h) + bh)
        h_new = (1-z)⊙h + z⊙h~

    FLOPs per GRU cell per timestep:
        FLOPs = 3 * [H*(I + H + 1)]  (3 gates)

    For L layers & T timesteps:
        FLOPs_total = L * T * FLOPs_cell

    FC layer:
        FLOPs_fc = 2 * H
    """

    I = model.input_dim
    H = model.hidden_dim
    L = model.num_layers

    # GRU single layer FLOPs per timestep
    flops_cell = 3 * H * (I + H + 1)

    # All layers × timesteps
    flops_gru = L * sequence_len * flops_cell

    # Final FC layer
    flops_fc = 2 * H

    return flops_gru + flops_fc


# ============================================================
# 5. Latency measurement
# ============================================================

def measure_latency(model, example_input, runs=200, warmup=20):

    device = "cpu"
    model.to(device)
    example_input = example_input.to(device)

    # Warmup (avoid first-run overhead)
    with torch.no_grad():
        for _ in range(warmup):
            _ = model(example_input)

    # Timed loop
    t0 = time.perf_counter()
    with torch.no_grad():
        for _ in range(runs):
            _ = model(example_input)
    t1 = time.perf_counter()

    return (t1 - t0) / runs


# ============================================================
# 6. Estimated energy
# ============================================================

def estimate_energy(latency_s, power_W=2.0):
    return latency_s * power_W


# ============================================================
# 7. Peak activation memory
# ============================================================

def estimate_peak_activation_memory(model, example_input):
    with torch.no_grad():
        out = model(example_input)
    return out.numel() * 4     # FP32


# ============================================================
# 8. Full GRU edge evaluation
# ============================================================

def evaluate_gru_edge(model_path, input_dim, sequence_len=1, assumed_power_W=2.0):

    # Load model
    model = load_gru_model(model_path, input_dim)

    # Dummy input (1 batch, T timesteps, input_dim features)
    example_input = torch.randn(1, sequence_len, input_dim)

    # Compute metrics
    params = count_parameters(model)
    size_b = model_size_bytes(model)
    size_mb = size_b / (1024 * 1024)

    flops = compute_gru_flops(model, sequence_len)
    latency_s = measure_latency(model, example_input)
    latency_ms = latency_s * 1000

    energy = estimate_energy(latency_s, assumed_power_W)
    peak_mem = estimate_peak_activation_memory(model, example_input)

    return {
        "Model Path": model_path,
        "Parameters": params,
        "Model Size (Bytes)": size_b,
        "Model Size (MB)": size_mb,
        "FLOPs": flops,
        "Latency (s)": latency_s,
        "Latency (ms)": latency_ms,
        "Energy Estimated (J)": energy,
        "Peak Activation Memory (Bytes)": peak_mem
    }
