import torch
import time
import numpy as np
# from cnn_lstm_model import CNNLSTM

import torch
import torch.nn as nn

class CNNLSTM(nn.Module):
    def __init__(self, input_dim, cnn_channels=64, lstm_hidden=64):
        super().__init__()

        # CNN over temporal axis (NO pooling)
        self.cnn = nn.Sequential(
            nn.Conv1d(
                in_channels=input_dim,
                out_channels=cnn_channels,
                kernel_size=3,
                padding=1
            ),
            nn.ReLU()
        )

        self.lstm = nn.LSTM(
            input_size=cnn_channels,
            hidden_size=lstm_hidden,
            batch_first=True
        )

        self.fc = nn.Linear(lstm_hidden, 1)

    def forward(self, x):
        # x: (batch, seq_len, features)
        x = x.permute(0, 2, 1)      # (batch, features, seq_len)
        x = self.cnn(x)             # (batch, cnn_channels, seq_len)
        x = x.permute(0, 2, 1)      # (batch, seq_len, cnn_channels)
        out, _ = self.lstm(x)
        return self.fc(out[:, -1, :]).squeeze(1)

def evaluate_cnn_lstm_edge(model_path, input_dim, sequence_len=10, assumed_power_W=2.0):

    model = CNNLSTM(input_dim=input_dim)
    state = torch.load(model_path, map_location="cpu")
    model.load_state_dict(state)
    model.eval()

    params = sum(p.numel() for p in model.parameters())
    size_mb = params * 4 / (1024**2)
    flops = params * 2  # approx

    dummy = torch.randn(1, sequence_len, input_dim)

    warm = 20
    runs = 200

    for _ in range(warm):
        model(dummy)

    t0 = time.perf_counter()
    for _ in range(runs):
        model(dummy)
    t1 = time.perf_counter()

    latency_ms = ((t1 - t0) / runs) * 1000
    energy_j = (latency_ms / 1000) * assumed_power_W

    activation_bytes = sequence_len * input_dim * 4

    return {
        "Parameters": params,
        "Model Size (MB)": size_mb,
        "FLOPs": flops,
        "Latency (ms)": latency_ms,
        "Energy Estimated (J)": energy_j,
        "Peak Activation Memory (Bytes)": activation_bytes,
        "Model Path": model_path,
    }
