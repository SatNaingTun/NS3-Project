#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
    cat <<'EOF'
Usage:
  scripts/clean_ns3_source_deb.sh <path-to-ns3-script.cc>

Removes Debian packaging artifacts created for the selected ns-3 script from:
  dist/.pkgbuild/<script-name>/
  dist/deb/<script-name>_*.deb
EOF
}

die() {
    printf '[clean-ns3-deb] Error: %s\n' "$*" >&2
    exit 1
}

clear_dir_contents() {
    local dir_path="$1"
    if [[ -d "$dir_path" ]]; then
        find "$dir_path" -mindepth 1 -delete
    fi
}

sanitize_package_name() {
    printf '%s' "$1" | tr '[:upper:]' '[:lower:]' | sed -E 's/[^a-z0-9.+-]+/-/g; s/^-+//; s/-+$//'
}

normalize_input_path() {
    local input_path="$1"

    if [[ -f "$input_path" ]]; then
        realpath "$input_path"
        return 0
    fi

    local prefix="/usr/share/"
    local marker="/src/"
    if [[ "$input_path" == *"$prefix"*"$marker"* ]]; then
        local after_prefix rel_path candidate
        after_prefix="${input_path#*"$prefix"}"
        rel_path="${after_prefix#*"$marker"}"
        candidate="$ROOT_DIR/$rel_path"

        if [[ -f "$candidate" ]]; then
            realpath "$candidate"
            return 0
        fi
    fi

    return 1
}

main() {
    if (($# != 1)); then
        usage
        exit 1
    fi

    local input_file="$1"
    input_file="$(normalize_input_path "$input_file")" || die "Input file not found: $input_file"

    local base_name package_name build_root out_dir legacy_out_dir temp_build_root deb_pattern
    base_name="$(basename "${input_file%.*}")"
    package_name="$(sanitize_package_name "$base_name")"
    [[ -n "$package_name" ]] || die "Failed to derive package name from $input_file"

    build_root="$ROOT_DIR/dist/.pkgbuild/$package_name"
    out_dir="$ROOT_DIR/dist/deb"
    legacy_out_dir="$ROOT_DIR/dist/$package_name"
    temp_build_root="/tmp/ns3-source-deb/$package_name"
    deb_pattern="$out_dir/${package_name}_"*".deb"

    rm -rf "$build_root" "$temp_build_root"
    rm -f $deb_pattern
    clear_dir_contents "$legacy_out_dir"

    printf '[clean-ns3-deb] Removed artifacts from %s, removed matching packages from %s, cleared legacy output %s, and removed %s\n' "$build_root" "$out_dir" "$legacy_out_dir" "$temp_build_root"
}

main "$@"
