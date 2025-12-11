import os
import time
import torch
import torch.nn as nn
import numpy as np


# ================================================================
# Load Transformer Architecture (must match training)
# ================================================================

class TransformerRegressor(nn.Module):
    def __init__(self, input_dim, embed_dim=128, num_heads=4, num_layers=2, dropout=0.1):
        super().__init__()

        self.input_dim = input_dim
        self.embed_dim = embed_dim
        self.num_heads = num_heads
        self.num_layers = num_layers

        self.embed = nn.Linear(input_dim, embed_dim)

        encoder_layer = nn.TransformerEncoderLayer(
            d_model=embed_dim,
            nhead=num_heads,
            dim_feedforward=embed_dim * 4,
            dropout=dropout,
            batch_first=True
        )
        self.encoder = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)

        self.fc_out = nn.Linear(embed_dim, 1)

    def forward(self, x):
        x = self.embed(x)          # (batch, seq, embed_dim)
        x = self.encoder(x)        # (batch, seq, embed_dim)
        x = x[:, -1, :]            # last token for regression
        return self.fc_out(x)


# ================================================================
# Load saved transformer model
# ================================================================

def load_transformer_model(model_path, input_dim):
    model = TransformerRegressor(input_dim)
    state = torch.load(model_path, map_location="cpu")
    model.load_state_dict(state)
    model.eval()
    return model


# ================================================================
# Parameter count + Model size
# ================================================================

def count_parameters(model):
    return sum(p.numel() for p in model.parameters())


def model_size_bytes(model):
    return count_parameters(model) * 4    # FP32 = 4 bytes per param


# ================================================================
# FLOPs for Transformer Encoder (accurate analytical formula)
# ================================================================
"""
Transformer FLOPs (single encoder layer):

1. Multi-Head Self-Attention:
   - Q,K,V projection: 3 × (d_model × d_model)
   - Attention scores: seq_len² × d_model
   - Output projection: d_model × d_model

2. Feedforward Network (FFN):
   - First linear: d_model × (4*d_model)
   - Second linear: (4*d_model) × d_model

Total FLOPs per layer:
   FLOPs = attention_FLOPs + FFN_FLOPs
"""

def compute_transformer_flops(model, sequence_len=1):

    d_model = model.embed_dim
    num_layers = model.num_layers
    heads = model.num_heads

    # Self-attention FLOPs
    flops_attention = (
        3 * (d_model * d_model) +
        (sequence_len * sequence_len * d_model) + 
        (d_model * d_model)
    )

    # FFN FLOPs
    d_ff = d_model * 4
    flops_ffn = (d_model * d_ff) + (d_ff * d_model)

    flops_per_layer = flops_attention + flops_ffn

    total_flops = num_layers * flops_per_layer + (2 * d_model)  # FC head

    return total_flops


# ================================================================
# Latency measurement
# ================================================================

def measure_latency(model, example_input, runs=200, warmup=20):
    device = "cpu"
    model.to(device)
    example_input = example_input.to(device)

    with torch.no_grad():
        for _ in range(warmup):
            _ = model(example_input)

    t0 = time.perf_counter()
    with torch.no_grad():
        for _ in range(runs):
            _ = model(example_input)
    t1 = time.perf_counter()

    return (t1 - t0) / runs


# ================================================================
# Energy estimate (power × latency)
# ================================================================

def estimate_energy(latency_s, power_W=2.0):
    return latency_s * power_W


# ================================================================
# Peak activation memory (output tensor size)
# ================================================================

def estimate_peak_activation_memory(model, example_input):
    with torch.no_grad():
        out = model(example_input)
    return out.numel() * 4  # FP32 bytes


# ================================================================
# Full Transformer Edge Evaluation
# ================================================================

def evaluate_transformer_edge(model_path, input_dim, sequence_len=1, assumed_power_W=2.0):

    model = load_transformer_model(model_path, input_dim)

    example_input = torch.randn(1, sequence_len, input_dim)

    params = count_parameters(model)
    size_b = model_size_bytes(model)
    size_mb = size_b / (1024 * 1024)

    flops = compute_transformer_flops(model, sequence_len)
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
        "Estimated Energy (J)": energy,
        "Peak Activation Memory (Bytes)": peak_mem
    }
