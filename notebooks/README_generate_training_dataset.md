# Generate Training Dataset Workflow

This README explains the workflow for creating training data from NS-3
`wifi-random` outputs, splitting it into train/validation/test CSVs, and then
using those files for model training and model comparison.

`generate_training_dataset.py` is **dataset-only**. It does not train LR models
and does not write model `.pkl` files. The older `generate_dataset.py` is
different because it also contains Linear Regression training code.

## 1. Generate NS-3 CSV Files

First run the NS-3 `wifi-random` simulation to create the raw CSV files.

Example using the packaged launcher:

```bash
wifi-random --outputDir="/media/sat-naing-tun/Data/test NS3/outputs/csv/wifi-random"
```

Example using the ns-3 scratch program directly:

```bash
./ns3 run "scratch/wifi-random --outputDir=/media/sat-naing-tun/Data/test\ NS3/outputs/csv/wifi-random --seed=10"
```

Run multiple seeds so the dataset can be split by simulation run. At least
3 simulation runs are required for train/validation/test splitting.

Expected files per run:

```text
wifi-random-...-seed10-perf.csv
wifi-random-...-seed10-nodedensity.csv
wifi-random-...-seed10-rssi.csv
wifi-random-...-seed10-modulation.csv
```

Only these two files are required by `generate_training_dataset.py`:

```text
*-perf.csv
*-nodedensity.csv
```

These files may be present but are not used by this script:

```text
*-rssi.csv
*-modulation.csv
```

## 2. Combine And Split Training Dataset

Run `generate_training_dataset.py` after the NS-3 CSV files are created.

Example using an external drive output folder:

```bash
/home/sat-naing-tun/Documents/NS3-Project/.venv/bin/python \
  /home/sat-naing-tun/Documents/NS3-Project/notebooks/generate_training_dataset.py \
  --source-dir "/media/sat-naing-tun/Data/test NS3/outputs/csv/wifi-random" \
  --out-csv "/media/sat-naing-tun/Data/test NS3/outputs/combine_analysis/final_training_dataset.csv" \
  --split-dir "/media/sat-naing-tun/Data/test NS3/outputs/datasets"
```

Example using the default repo-local output paths:

```bash
/home/sat-naing-tun/Documents/NS3-Project/.venv/bin/python \
  /home/sat-naing-tun/Documents/NS3-Project/notebooks/generate_training_dataset.py \
  --source-dir "/media/sat-naing-tun/Data/test NS3/outputs/csv/wifi-random"
```

Default outputs:

```text
outputs/combine_analysis/final_training_dataset.csv
outputs/datasets/train.csv
outputs/datasets/valid.csv
outputs/datasets/test.csv
```

If your path contains spaces, such as `test NS3`, quote the full path.

## 3. How The Script Works

For each matching simulation run:

1. Load `*-perf.csv`.
2. Load the matching `*-nodedensity.csv`.
3. Match files using `RandSeed` and run timestamp from the filename.
4. Average performance metrics from `perf.csv`.
5. Merge those metrics into interval rows from `nodedensity.csv`.
6. Save the merged dataset to `final_training_dataset.csv`.
7. Split by unique `(RandSeed, RunDateTime)`.

Default split ratio:

```text
train = 70%
valid = 15%
test  = 15%
```

The split is done by simulation run, not by individual row, so rows from the
same simulation do not leak across train/validation/test.

## 4. Train Models

After the split files are created, train each model using the training
notebooks/scripts, for example:

```text
notebooks/training_LR_model_perSTA.ipynb
notebooks/training_RF_perSTA.ipynb
notebooks/training_XGBoost_perSTA.ipynb
notebooks/training_MLP_model.ipynb
notebooks/training_LSTM_model.ipynb
notebooks/training_GRU_model.ipynb
notebooks/training_BILSTM_model.ipynb
notebooks/training_CNN_LSTM_model.ipynb
notebooks/training_Transformer_model.ipynb
notebooks/training_GNN_model.ipynb
```

Those training notebooks use the split CSVs as model input and write trained
models and prediction CSVs under `outputs/models/`.

## 5. Evaluate And Compare Models

After all models are trained, run:

```text
notebooks/models_comparison.ipynb
```

This notebook compares prediction results and edge/computational metrics across
models and targets.

## Troubleshooting

### `Found 0 perf files`

The `--source-dir` folder does not contain files matching:

```text
*-perf.csv
```

Check that you passed the folder containing the CSV files, not the parent
folder.

### `Found 0 nodedensity files`

The `--source-dir` folder does not contain files matching:

```text
*-nodedensity.csv
```

Run the NS-3 simulation again and confirm it writes nodedensity CSVs.

### `No matching nodedensity`

The script found a `*-perf.csv` file but could not find a matching
`*-nodedensity.csv` with the same seed and run timestamp.

Check filename pairs such as:

```text
wifi-random-29-Apr-2026_16-35-seed10-perf.csv
wifi-random-29-Apr-2026_16-35-seed10-nodedensity.csv
```

### `Need at least 3 simulation runs`

The train/validation/test split needs at least 3 unique simulation runs.

Generate more NS-3 runs with different seeds before running
`generate_training_dataset.py` again.

### `unrecognized arguments: NS3/...`

A path with spaces was not quoted. Use:

```bash
--source-dir "/media/sat-naing-tun/Data/test NS3/outputs/csv/wifi-random"
```

not:

```bash
--source-dir /media/sat-naing-tun/Data/test NS3/outputs/csv/wifi-random
```
