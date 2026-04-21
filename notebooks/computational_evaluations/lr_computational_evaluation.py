import os
import time
import joblib
import numpy as np


def load_model(path):
    return joblib.load(path)


def model_size_bytes(path):
    size = os.path.getsize(path)
    return size, size / (1024 * 1024)


def measure_latency(model, X_sample, repeats=1000, warmup=3):
    X_batch = np.repeat(X_sample, repeats, axis=0)

    for _ in range(warmup):
        model.predict(X_batch)

    t0 = time.perf_counter()
    model.predict(X_batch)
    t1 = time.perf_counter()

    return (t1 - t0) / repeats


def evaluate(model_path, X_sample, repeats=1000, power_W=2.0):
    model = load_model(model_path)

    size_bytes, size_mb = model_size_bytes(model_path)
    latency = measure_latency(model, X_sample, repeats)

    return {
        "Model": "LR",
        "Model Size (MB)": size_mb,
        "Computational Latency (ms/sample)": latency * 1000,
        "Energy (J/sample)": latency * power_W,
        "Num Parameters": X_sample.shape[1] + 1
    }