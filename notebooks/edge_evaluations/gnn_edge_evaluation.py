def evaluate_gnn_edge(
    model_path,
    input_dim=3,          # MUST match training
    hidden_dim=32,        # MUST match training
    num_nodes=10,
    assumed_power_W=5.0
):
    import time
    import torch
    from torch import nn
    from torch_geometric.nn import GCNConv

    DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

    # --------------------------------------------------
    # GNN architecture (MATCH TRAINING EXACTLY)
    # --------------------------------------------------
    class GNNModel(nn.Module):
        def __init__(self, input_dim, hidden_dim):
            super().__init__()
            self.conv1 = GCNConv(input_dim, hidden_dim)
            self.conv2 = GCNConv(hidden_dim, hidden_dim)
            self.fc = nn.Linear(hidden_dim, 1)

        def forward(self, x, edge_index):
            x = self.conv1(x, edge_index)
            x = torch.relu(x)
            x = self.conv2(x, edge_index)
            x = torch.relu(x)
            x = self.fc(x)
            return x.mean(dim=0, keepdim=True)

    # --------------------------------------------------
    # Load model weights (CORRECT)
    # --------------------------------------------------
    model = GNNModel(input_dim, hidden_dim)
    state_dict = torch.load(model_path, map_location=DEVICE)
    model.load_state_dict(state_dict)
    model.to(DEVICE)
    model.eval()

    # --------------------------------------------------
    # Synthetic graph input (node-level)
    # --------------------------------------------------
    x = torch.randn(num_nodes, input_dim, device=DEVICE)

    edge_index = torch.combinations(
        torch.arange(num_nodes), r=2
    ).t()
    edge_index = torch.cat([edge_index, edge_index.flip(0)], dim=1)
    edge_index = edge_index.to(DEVICE)

    # --------------------------------------------------
    # Latency measurement
    # --------------------------------------------------
    with torch.no_grad():
        for _ in range(20):  # warmup
            _ = model(x, edge_index)

        runs = 200
        t0 = time.perf_counter()
        for _ in range(runs):
            _ = model(x, edge_index)
        t1 = time.perf_counter()

    latency = (t1 - t0) / runs

    # --------------------------------------------------
    # Metrics
    # --------------------------------------------------
    params = sum(p.numel() for p in model.parameters())
    size_bytes = sum(p.numel() * p.element_size() for p in model.parameters())
    size_mb = size_bytes / (1024 ** 2)
    energy = latency * assumed_power_W
    peak_activation = x.numel() * x.element_size()

    return {
        "Parameters": params,
        "Model Size (Bytes)": size_bytes,
        "Model Size (MB)": size_mb,
        "Latency (s)": latency,
        "Latency (ms)": latency * 1e3,
        "Estimated Energy (J)": energy,
        "Peak Activation Memory (Bytes)": peak_activation,
        "Num Nodes": num_nodes,
        "Node Feature Dim": input_dim,
        "Hidden Dim": hidden_dim,
        "Model Path": model_path,
    }
