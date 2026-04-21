# wifi-random Debian Package Guide

This guide shows how to package, install, run, and uninstall the sample
`wifi-random` ns-3 script from this repository.

## Prerequisites

Run these commands from the project root:

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build
```

The Debian packaging script also expects these tools to be available:

- `dpkg-deb`
- `realpath`
- `python3`
- `cp`

## 1. Build The Debian Package

Package `scratch/wifi-random.cc`:

```bash
./scripts/build_ns3_source_deb.sh scratch/wifi-random.cc
```

Expected package output:

```bash
dist/deb/wifi-random_1.0.0_amd64.deb
```

## 2. Install The Package

Use `apt install` so Debian or Ubuntu can resolve package dependencies:

```bash
sudo apt install ./dist/deb/wifi-random_1.0.0_amd64.deb
```

## 3. Run wifi-random

After installation, run the packaged launcher with:

```bash
wifi-random
```

The launcher compiles the installed source on first run and caches the
executable under:

```bash
~/.cache/wifi-random/build/
```

To pass arguments to the program, append them after the command. Example:

```bash
wifi-random --outputDir=outputs/csv/wifi-random
```

## 4. Uninstall The Installed Package

Remove the installed Debian package:

```bash
sudo apt remove wifi-random
```

If you also want to remove cached build files created by the launcher:

```bash
rm -rf ~/.cache/wifi-random
```

## 5. Remove Local Packaging Artifacts

Remove the generated `.deb` file and package build folders for `wifi-random`:

```bash
./scripts/clean_ns3_source_deb.sh scratch/wifi-random.cc
```

To remove all generated `.deb` files and package build folders for every
packaged script in this repository:

```bash
./scripts/clean_all_ns3_source_deb.sh
```
