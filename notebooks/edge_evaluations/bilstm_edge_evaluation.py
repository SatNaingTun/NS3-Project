import os
import time
import torch
import torch.nn as nn
import numpy as np


# ============================================================
# 1. BiLSTM Model (must match your training architecture)
# ============================================================

class BiLSTMModel(nn.Module):
    def __init__(self, input_dim, hidden_dim=64, num_layers=2):
        super().__init__()

        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers

        self.lstm = nn.LSTM(
            input_size=input_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True,
            bidirectional=True
        )

        # For BiLSTM → output size = 2*hidden_dim
        self.fc = nn.Linear(2 * hidden_dim, 1)

    def forward(self, x):
        out, _ = self.lstm(x)
        last = out[:, -1, :]   # final timestep
        return self.fc(last)


# ============================================================
# 2. Load BiLSTM model weights
# ============================================================

# def load_bilstm_model(model_path, input_dim):
#     model = BiLSTMModel(input_dim)
#     state = torch.load(model_path, map_location="cpu")
#     model.load_state_dict(state)
#     model.eval()
#     return model


# ============================================================
# 3. Count Parameters + Model Size
# ============================================================

def count_parameters(model):
    return sum(p.numel() for p in model.parameters())


def model_size_bytes(model):
    return count_parameters(model) * 4   # FP32


# ============================================================
# 4. FLOPs Calculation for BiLSTM
# ============================================================

def compute_bilstm_flops(model, sequence_len=1):
    """
    FLOPs for BiLSTM:

    For each LSTM direction:
      FLOPs_cell = 4 * H * (H + I + 1)

    For bidirectional:
      FLOPs_bidir = 2 * FLOPs_cell

    For L layers:
      FLOPs_lstm_total = L * sequence_len * FLOPs_bidir

    FC Layer:
      FLOPs_fc = 2 * (2*H) * 1
    """
    I = model.input_dim
    H = model.hidden_dim
    L = model.num_layers

    # Single-LSTM direction FLOPs
    flops_cell = 4 * H * (H + I + 1)

    # BiLSTM doubles this
    flops_bidir = 2 * flops_cell

    # Total LSTM FLOPs
    flops_lstm = L * sequence_len * flops_bidir

    # Final FC layer FLOPs
    flops_fc = 2 * (2 * H)

    return flops_lstm + flops_fc


# ============================================================
# 5. Latency Measurement
# ============================================================

def measure_latency(model, example_input, runs=200, warmup=20):

    device = "cpu"
    model.to(device)
    example_input = example_input.to(device)

    # Warmup
    with torch.no_grad():
        for _ in range(warmup):
            _ = model(example_input)

    # Timed inference runs
    t0 = time.perf_counter()
    with torch.no_grad():
        for _ in range(runs):
            _ = model(example_input)
    t1 = time.perf_counter()

    return (t1 - t0) / runs


# ============================================================
# 6. Simple Energy Estimate
# ============================================================

def estimate_energy(latency_s, power_W=2.0):
    """
    Approximate energy = P * t
    """
    return latency_s * power_W


# ============================================================
# 7. Peak Activation Memory
# ============================================================

def estimate_peak_activation_memory(model, example_input):
    with torch.no_grad():
        out = model(example_input)
    return out.numel() * 4   # FP32 byte size


# ============================================================
# 8. Full BiLSTM Edge Evaluation Pipeline
# ============================================================

# def evaluate_bilstm_edge(model_path, input_dim, sequence_len=1, assumed_power_W=2.0):

#     # Load model
#     model = load_bilstm_model(model_path, input_dim)

#     # Dummy input for FLOPs, latency, activations
#     example_input = torch.randn(1, sequence_len, input_dim)

#     # ---- Compute Metrics ----
#     params = count_parameters(model)
#     size_b = model_size_bytes(model)
#     size_mb = size_b / (1024 * 1024)

#     flops = compute_bilstm_flops(model, sequence_len)
#     latency_s = measure_latency(model, example_input)
#     latency_ms = latency_s * 1000

#     energy = estimate_energy(latency_s, assumed_power_W)
#     peak_mem = estimate_peak_activation_memory(model, example_input)

#     # ---- Return Results ----
#     return {
#         "Model Path": model_path,
#         "Parameters": params,
#         "Model Size (Bytes)": size_b,
#         "Model Size (MB)": size_mb,
#         "FLOPs": flops,
#         "Latency (s)": latency_s,
#         "Latency (ms)": latency_ms,
#         "Energy Estimated (J)": energy,
#         "Peak Activation Memory (Bytes)": peak_mem
#     }
def load_bilstm_model(model_path):
    state = torch.load(model_path, map_location="cpu")

    # Infer input_dim from checkpoint
    input_dim = state["lstm.weight_ih_l0"].shape[1]

    model = BiLSTMModel(input_dim)
    model.load_state_dict(state)
    model.eval()

    return model, input_dim


def evaluate_bilstm_edge(model_path, sequence_len=1, assumed_power_W=2.0):

    model, input_dim = load_bilstm_model(model_path)

    example_input = torch.randn(1, sequence_len, input_dim)

    params = count_parameters(model)
    size_b = model_size_bytes(model)
    size_mb = size_b / (1024 * 1024)

    flops = compute_bilstm_flops(model, sequence_len)
    latency_s = measure_latency(model, example_input)
    energy = estimate_energy(latency_s, assumed_power_W)
    peak_mem = estimate_peak_activation_memory(model, example_input)

    return {
        "Input Dim": input_dim,
        "Parameters": params,
        "Model Size (Bytes)": size_b,
        "Model Size (MB)": size_mb,
        "FLOPs": flops,
        "Latency (s)": latency_s,
        "Latency (ms)": latency_s * 1000,
        "Energy Estimated (J)": energy,
        "Peak Activation Memory (Bytes)": peak_mem,
        "Model Path": model_path,
    }
