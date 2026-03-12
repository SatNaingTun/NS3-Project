#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

pkgbuild_root="$ROOT_DIR/dist/.pkgbuild"
dist_root="$ROOT_DIR/dist"
deb_root="$dist_root/deb"
temp_root="/tmp/ns3-source-deb"

rm -rf "$pkgbuild_root" "$temp_root"

if [[ -d "$deb_root" ]]; then
    find "$deb_root" -mindepth 1 -delete
fi

if [[ -d "$dist_root" ]]; then
    while IFS= read -r package_dir; do
        find "$package_dir" -mindepth 1 -delete
    done < <(find "$dist_root" -mindepth 1 -maxdepth 1 -type d ! -name '.pkgbuild' ! -name 'deb')
fi

printf '[clean-ns3-deb] Removed all package build folders under %s, cleared shared package outputs under %s, and cleared legacy package directories under %s\n' "$pkgbuild_root" "$deb_root" "$dist_root"
