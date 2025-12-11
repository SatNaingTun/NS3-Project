import torch
import time
import psutil
import os

try:
    from thop import profile
    THOP_AVAILABLE = True
except ImportError:
    THOP_AVAILABLE = False
    print("⚠ THOP not installed → FLOPs will use manual estimation.")


# ======================================================================
# 1. MODEL PARAMETER COUNT & SIZE (bytes / MB)
# ======================================================================
import torch
import torch.nn as nn
import joblib

def load_lr_model(model_path, scaler_path, input_dim):
    """
    Load sklearn LR model and convert to PyTorch Linear.
    Returns: sklearn_model, scaler, pytorch_model
    """

    # Load sklearn model and scaler
    sk_model = joblib.load(model_path)
    scaler = joblib.load(scaler_path)

    # Build PyTorch model (1 output)
    pt_model = nn.Linear(input_dim, 1)

    # Assign weights correctly (ensure 2D shape)
    w = torch.tensor(sk_model.coef_, dtype=torch.float32).reshape(1, -1)
    b = torch.tensor(sk_model.intercept_, dtype=torch.float32)

    pt_model.weight.data = w
    pt_model.bias.data = b

    return sk_model, scaler, pt_model


def count_parameters(model):
    """Returns total number of trainable parameters."""
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


def model_size_bytes(model, bytes_per_param=4):
    """
    Estimate model size in bytes.
    float32 = 4 bytes; int8 = 1 byte.
    """
    n_params = count_parameters(model)
    return n_params * bytes_per_param


def model_size_megabytes(model, bytes_per_param=4):
    return model_size_bytes(model, bytes_per_param) / (1024 ** 2)


# ======================================================================
# 2. FLOPs ESTIMATION
# ======================================================================

def compute_flops(model, example_input):
    """
    Returns FLOPs using THOP if available.
    Otherwise returns None.
    """
    if THOP_AVAILABLE:
        flops, params = profile(model, inputs=(example_input,), verbose=False)
        return flops
    else:
        print("⚠ FLOPs unavailable (THOP not installed). Returning None.")
        return None

def compute_flops_lr(input_dim):
    return 2 * input_dim   # multiply + add per feature


# ======================================================================
# 3. INFERENCE LATENCY (CPU/Edge Device)
# ======================================================================

def measure_latency(model, example_input, runs=100, warmup=10, device="cpu"):
    model.eval()
    model.to(device)

    # Ensure correct shape
    if example_input.ndim == 1:
        example_input = example_input.unsqueeze(0)

    example_input = example_input.to(device)

    with torch.no_grad():

        # Warmup
        for _ in range(warmup):
            _ = model(example_input)

        # Timed runs
        import time
        t0 = time.perf_counter()
        for _ in range(runs):
            _ = model(example_input)
        t1 = time.perf_counter()

    return (t1 - t0) / runs



# ======================================================================
# 4. POWER & ENERGY MEASUREMENT (measurement-based)
# ======================================================================

def measure_power_cpu():
    """
    Returns CPU power consumption in Watts using psutil.
    NOTE: This is an approximation; best is external USB meter.
    """
    try:
        return psutil.sensors_battery().power_plugged  # often 0 on desktops
    except:
        return None


def estimate_energy(latency, power_watts):
    """
    Energy = Power * Latency
    """
    if power_watts is None:
        return None
    return latency * power_watts


# ======================================================================
# 5. ENERGY VIA FLOPS ESTIMATION (model-based)
# ======================================================================

def estimate_energy_from_flops(flops, E_per_FLOP=5e-10):
    """
    E_est = FLOPs * E_per_FLOP
    Default: 5e-10 J/FLOP (example from literature)
    """
    if flops is None:
        return None
    return flops * E_per_FLOP


# ======================================================================
# 6. PEAK ACTIVATION MEMORY ESTIMATION
# ======================================================================

def estimate_peak_activation_memory(example_output, bytes_per_activation=4):
    """
    Estimate peak activation memory from example model output.
    """
    return example_output.numel() * bytes_per_activation


# ======================================================================
# 7. FULL EDGE DEVICE METRICS (wrapper function)
# ======================================================================

def evaluate_edge_metrics(model, input_dim, device="cpu"):
    dummy_input = torch.randn(1, input_dim)

    # FLOPs
    flops = compute_flops_lr(input_dim)

    # Latency
    latency = measure_latency(model, dummy_input, device=device)

    # Parameters
    params = sum(p.numel() for p in model.parameters())

    # Model size (float32 = 4 bytes)
    size_bytes = params * 4
    size_mb = size_bytes / (1024 * 1024)

    # Peak activation = output tensor size
    with torch.no_grad():
        out = model(dummy_input)
    peak_act = out.numel() * 4

    # Energy estimation (no power measurement)
    E_est = flops * 5e-10

    return {
        "Parameters": params,
        "Model Size (Bytes)": size_bytes,
        "Model Size (MB)": size_mb,
        "FLOPs": flops,
        "Latency (s)": latency,
        "Latency (ms)": latency * 1000,
        "Energy Estimated (J)": E_est,
        "Peak Activation Memory (Bytes)": peak_act
    }

def evaluate_model(pt_model, input_dim, device="cpu"):
    return evaluate_edge_metrics(pt_model, input_dim, device=device)

def evaluate_lr_edge(model_path, input_dim,scaler_path="../../outputs/models/lr_scaler.pkl", device="cpu"):
    """
    Full LR edge evaluation pipeline.
    """

    # Load model
    sk_model, scaler, pt_model = load_lr_model(model_path, scaler_path, input_dim)

    # Evaluate metrics
    results = evaluate_edge_metrics(pt_model, input_dim, device=device)

    # Add model path info
    results["Model Path"] = model_path

    return results


