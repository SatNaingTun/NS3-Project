import time
import torch


def count_params(model):
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


def model_size_bytes(model):
    total = 0
    for p in model.parameters():
        total += p.numel() * p.element_size()
    return total, total / (1024 * 1024)


def measure_latency(model, x_sample, repeats=1000, warmup=3, device="cpu"):
    model.eval()
    model.to(device)

    x_sample = x_sample.to(device)
    x_batch = x_sample.repeat(repeats, *([1] * (x_sample.dim() - 1)))

    with torch.no_grad():
        for _ in range(warmup):
            model(x_batch)

        t0 = time.perf_counter()
        model(x_batch)
        t1 = time.perf_counter()

    return (t1 - t0) / repeats


def evaluate(model, model_name, x_sample, repeats=1000, power_W=2.0):
    latency = measure_latency(model, x_sample, repeats)

    size_bytes, size_mb = model_size_bytes(model)

    return {
        "Model": model_name,
        "Model Size (MB)": size_mb,
        "Computational Latency (ms/sample)": latency * 1000,
        "Energy (J/sample)": latency * power_W,
        "Parameters": count_params(model)
    }