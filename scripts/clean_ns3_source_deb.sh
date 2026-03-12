#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
    cat <<'EOF'
Usage:
  scripts/clean_ns3_source_deb.sh <path-to-ns3-script.cc>

Removes Debian packaging artifacts created for the selected ns-3 script from:
  dist/.pkgbuild/<script-name>/
  dist/<script-name>/
EOF
}

die() {
    printf '[clean-ns3-deb] Error: %s\n' "$*" >&2
    exit 1
}

sanitize_package_name() {
    printf '%s' "$1" | tr '[:upper:]' '[:lower:]' | sed -E 's/[^a-z0-9.+-]+/-/g; s/^-+//; s/-+$//'
}

main() {
    if (($# != 1)); then
        usage
        exit 1
    fi

    local input_file="$1"
    [[ -f "$input_file" ]] || die "Input file not found: $input_file"

    local base_name package_name build_root out_dir
    base_name="$(basename "${input_file%.*}")"
    package_name="$(sanitize_package_name "$base_name")"
    [[ -n "$package_name" ]] || die "Failed to derive package name from $input_file"

    build_root="$ROOT_DIR/dist/.pkgbuild/$package_name"
    out_dir="$ROOT_DIR/dist/$package_name"

    rm -rf "$build_root" "$out_dir"

    printf '[clean-ns3-deb] Removed %s and %s\n' "$build_root" "$out_dir"
}

main "$@"
