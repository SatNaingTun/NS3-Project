import os
import time
import joblib
import numpy as np
from xgboost import XGBRegressor


def load_model(path):
    ext = os.path.splitext(path)[1].lower()

    if ext == ".json":
        model = XGBRegressor()
        model.load_model(path)
        return model

    return joblib.load(path)


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

    size_bytes = os.path.getsize(model_path)
    size_mb = size_bytes / (1024 * 1024)

    latency = measure_latency(model, X_sample, repeats)

    return {
        "Model": "XGBoost",
        "Model Size (MB)": size_mb,
        "Computational Latency (ms/sample)": latency * 1000,
        "Energy (J/sample)": latency * power_W,
        "Trees": getattr(model, "n_estimators", None)
    }
