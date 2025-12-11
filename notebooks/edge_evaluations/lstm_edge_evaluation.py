import os
import time
import numpy as np
import torch
import torch.nn as nn


# ============================================================
# 1. Define LSTM model (must match training architecture)
# ============================================================

class LSTMModel(nn.Module):
    def __init__(self, input_dim, hidden_dim=64, num_layers=2):
        super().__init__()
        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers

        self.lstm = nn.LSTM(
            input_size=input_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True
        )

        self.fc = nn.Linear(hidden_dim, 1)

    def forward(self, x):
        output, _ = self.lstm(x)
        return self.fc(output[:, -1, :])  # last timestep
        

# ============================================================
# 2. Load model from .pt file
# ============================================================

def load_lstm_model(model_path, input_dim):
    model = LSTMModel(input_dim)
    state = torch.load(model_path, map_location="cpu")
    model.load_state_dict(state)
    model.eval()
    return model


# ============================================================
# 3. Count parameters & model size
# ============================================================

def count_parameters(model):
    return sum(p.numel() for p in model.parameters())


def model_size_bytes(model):
    """
    FP32 weights → 4 bytes per parameter
    """
    return count_parameters(model) * 4


# ============================================================
# 4. FLOPs estimation for LSTM
# ============================================================

def compute_lstm_flops(model, sequence_len=1):
    """
    FLOPs for stacked LSTM + final FC layer.

    LSTM FLOPs per cell:
      FLOPs = 4 * H * (H + I + 1)
    (4 gates: input, forget, output, candidate)

    For L layers and L_seq timesteps:
      Total = layers * timesteps * FLOPs_cell

    FC FLOPs = 2 * H * 1
    """

    I = model.input_dim
    H = model.hidden_dim
    L = model.num_layers

    # FLOPs for one LSTM cell
    flops_cell = 4 * H * (H + I + 1)

    # Total for all layers * timesteps
    flops_lstm = L * sequence_len * flops_cell

    # Final FC
    flops_fc = 2 * H * 1

    return flops_lstm + flops_fc


# ============================================================
# 5. Measure inference latency
# ============================================================

def measure_latency(model, example_input, runs=200, warmup=20):
    device = "cpu"
    model.to(device)
    example_input = example_input.to(device)

    # Warmup
    with torch.no_grad():
        for _ in range(warmup):
            _ = model(example_input)

    # Timed runs
    t0 = time.perf_counter()
    with torch.no_grad():
        for _ in range(runs):
            _ = model(example_input)
    t1 = time.perf_counter()

    return (t1 - t0) / runs


# ============================================================
# 6. Energy estimation
# ============================================================

def estimate_energy(latency_s, power_W=2.0):
    """
    E = P × t
    """
    return latency_s * power_W


# ============================================================
# 7. Peak activation memory
# ============================================================

def estimate_peak_activation_memory(model, example_input):
    with torch.no_grad():
        output = model(example_input)
    return output.numel() * 4  # FP32


# ============================================================
# 8. Full evaluation function
# ============================================================

def evaluate_lstm_edge(model_path, input_dim, sequence_len=1, assumed_power_W=2.0):
    
    # ---------------- Load Model ----------------
    model = load_lstm_model(model_path, input_dim)

    # Input tensor for evaluation
    example_input = torch.randn(1, sequence_len, input_dim)

    # ---------------- Compute Metrics ----------------
    params = count_parameters(model)
    size_b = model_size_bytes(model)
    size_mb = size_b / (1024 * 1024)

    flops = compute_lstm_flops(model, sequence_len)
    latency_s = measure_latency(model, example_input)
    latency_ms = latency_s * 1000

    energy = estimate_energy(latency_s, assumed_power_W)
    peak_mem = estimate_peak_activation_memory(model, example_input)

    # ---------------- Package Output ----------------
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

