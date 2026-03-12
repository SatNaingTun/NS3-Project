#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
    cat <<'EOF'
Usage:
  scripts/build_ns3_source_deb.sh <path-to-ns3-script.cc> [extra-files...]

Builds a Debian package that contains the selected ns-3 source file plus the
local source/header files it depends on. The generated package installs a
launcher with the same name as the script and declares apt-installable build
dependencies such as libns3-dev.

Install the resulting package with:
  sudo apt install ./dist/<script-name>/<script-name>_1.0.0_amd64.deb
EOF
}

log() {
    printf '[build-ns3-deb] %s\n' "$*"
}

die() {
    printf '[build-ns3-deb] Error: %s\n' "$*" >&2
    exit 1
}

require_cmd() {
    command -v "$1" >/dev/null 2>&1 || die "Missing required command: $1"
}

sanitize_package_name() {
    printf '%s' "$1" | tr '[:upper:]' '[:lower:]' | sed -E 's/[^a-z0-9.+-]+/-/g; s/^-+//; s/-+$//'
}

relative_to_root() {
    local target="$1"
    python3 - "$ROOT_DIR" "$target" <<'PY'
import os
import sys

root = os.path.realpath(sys.argv[1])
target = os.path.realpath(sys.argv[2])

if os.path.commonpath([root, target]) != root:
    raise SystemExit(1)

print(os.path.relpath(target, root))
PY
}

resolve_local_include() {
    local including_file="$1"
    local include_name="$2"
    local base_dir
    base_dir="$(dirname "$including_file")"

    if [[ -f "$base_dir/$include_name" ]]; then
        realpath "$base_dir/$include_name"
        return 0
    fi

    if [[ -f "$ROOT_DIR/$include_name" ]]; then
        realpath "$ROOT_DIR/$include_name"
        return 0
    fi

    return 1
}

collect_local_dependencies() {
    local main_file="$1"
    shift || true

    declare -A seen=()
    local queue=()
    local file

    queue+=("$(realpath "$main_file")")
    for file in "$@"; do
        queue+=("$(realpath "$file")")
    done

    while ((${#queue[@]} > 0)); do
        file="${queue[0]}"
        queue=("${queue[@]:1}")

        [[ -f "$file" ]] || die "Dependency file does not exist: $file"
        [[ -n "${seen[$file]:-}" ]] && continue

        seen["$file"]=1

        local line include_name resolved stem sibling
        while IFS= read -r line; do
            include_name="${line#*\"}"
            include_name="${include_name%%\"*}"

            if resolved="$(resolve_local_include "$file" "$include_name")"; then
                queue+=("$resolved")

                stem="${resolved%.*}"
                for sibling in "$stem.cc" "$stem.cpp" "$stem.cxx" "$stem.c"; do
                    if [[ -f "$sibling" ]]; then
                        queue+=("$(realpath "$sibling")")
                    fi
                done
            fi
        done < <(grep -E '^[[:space:]]*#include[[:space:]]+"[^"]+"' "$file" || true)
    done

    printf '%s\n' "${!seen[@]}" | sort
}

main() {
    require_cmd dpkg-deb
    require_cmd realpath
    require_cmd python3

    if (($# < 1)); then
        usage
        exit 1
    fi

    local input_file="$1"
    shift || true

    [[ -f "$input_file" ]] || die "Input file not found: $input_file"

    local input_abs input_rel base_name package_name version arch
    input_abs="$(realpath "$input_file")"
    input_rel="$(relative_to_root "$input_abs")" || die "Input file must be inside the workspace"
    base_name="$(basename "${input_abs%.*}")"
    package_name="$(sanitize_package_name "$base_name")"
    [[ -n "$package_name" ]] || die "Failed to derive package name from $input_file"

    version="1.0.0"
    arch="$(dpkg --print-architecture)"

    local out_dir build_root pkg_root package_dir installed_src_root control_file
    out_dir="$ROOT_DIR/dist/$package_name"
    build_root="$ROOT_DIR/dist/.pkgbuild/$package_name"
    pkg_root="$build_root/${package_name}_${version}_${arch}"
    package_dir="$pkg_root/DEBIAN"
    installed_src_root="$pkg_root/usr/share/$package_name/src"

    rm -rf "$build_root" "$out_dir"
    mkdir -p "$package_dir" "$installed_src_root" "$pkg_root/usr/bin" "$pkg_root/usr/share/doc/$package_name"

    log "Collecting local source dependencies for $input_rel"

    local dependencies
    mapfile -t dependencies < <(collect_local_dependencies "$input_abs" "$@")

    local dep_file dep_rel dep_dest dep_dir
    for dep_file in "${dependencies[@]}"; do
        dep_rel="$(relative_to_root "$dep_file")" || die "Dependency is outside workspace: $dep_file"
        dep_dest="$installed_src_root/$dep_rel"
        dep_dir="$(dirname "$dep_dest")"
        mkdir -p "$dep_dir"
        cp "$dep_file" "$dep_dest"
    done

    cat > "$pkg_root/usr/bin/$package_name" <<EOF
#!/usr/bin/env bash
set -euo pipefail

APP_NAME="$package_name"
SRC_ROOT="/usr/share/$package_name/src"
MAIN_SOURCE="\$SRC_ROOT/$input_rel"
CACHE_ROOT="\${XDG_CACHE_HOME:-\$HOME/.cache}/\$APP_NAME"
BUILD_DIR="\$CACHE_ROOT/build"
BIN_PATH="\$BUILD_DIR/\$APP_NAME"

mkdir -p "\$BUILD_DIR"

mapfile -t SOURCES < <(find "\$SRC_ROOT" -type f \\( -name '*.cc' -o -name '*.cpp' -o -name '*.cxx' -o -name '*.c' \\) | sort)

if [[ \${#SOURCES[@]} -eq 0 ]]; then
    echo "No source files installed for \$APP_NAME" >&2
    exit 1
fi

rebuild=0
if [[ ! -x "\$BIN_PATH" ]]; then
    rebuild=1
else
    while IFS= read -r source_file; do
        if [[ "\$source_file" -nt "\$BIN_PATH" ]]; then
            rebuild=1
            break
        fi
    done < <(printf '%s\n' "\${SOURCES[@]}")
fi

if (( rebuild )); then
    echo "[\$APP_NAME] Building cached executable at \$BIN_PATH"
    mapfile -t NS3_MODULES < <(pkg-config --list-all | awk '/^ns3/ {print \$1}' | sort -u)
    if [[ \${#NS3_MODULES[@]} -eq 0 ]]; then
        echo "No ns-3 pkg-config modules found. Install libns3-dev and pkg-config." >&2
        exit 1
    fi

    g++ -std=c++17 -O2 -I"\$SRC_ROOT" "\${SOURCES[@]}" \\
        \$(pkg-config --cflags --libs "\${NS3_MODULES[@]}") \\
        -o "\$BIN_PATH"
fi

if [[ -x "\$BIN_PATH" ]]; then
    echo "[\$APP_NAME] Using cached executable: \$BIN_PATH"
fi

exec "\$BIN_PATH" "\$@"
EOF
    chmod 0755 "$pkg_root/usr/bin/$package_name"

    cat > "$pkg_root/usr/share/doc/$package_name/README.Debian" <<EOF
$package_name ships the selected ns-3 script source and its local helper files.

Run it with:
  $package_name

The first run compiles the installed sources against the system ns-3
development package and caches the executable under:
  \$XDG_CACHE_HOME/$package_name/build/

Install this package with apt so dependencies are pulled automatically:
  sudo apt install ./${package_name}_${version}_${arch}.deb
EOF

    control_file="$package_dir/control"
    cat > "$control_file" <<EOF
Package: $package_name
Version: $version
Section: science
Priority: optional
Architecture: $arch
Maintainer: NS3-Project Packager <noreply@example.com>
Depends: bash, g++, libns3-dev, pkg-config, libgsl-dev, libsqlite3-dev, libxml2-dev
Description: Packaged ns-3 source launcher for $base_name
 This package contains the ns-3 source file $input_rel and any local project
 files it includes. The installed launcher compiles only those files, not the
 entire upstream ns-3 tree, and relies on apt-installable ns-3 development
 libraries from the distribution.
EOF

    chmod 0755 "$package_dir"

    mkdir -p "$out_dir"
    local deb_path="$out_dir/${package_name}_${version}_${arch}.deb"
    log "Building Debian package $deb_path"
    dpkg-deb --build "$pkg_root" "$deb_path" >/dev/null

    log "Done"
    printf 'Package: %s\n' "$deb_path"
    printf 'Install with: sudo apt install ./%s\n' "${deb_path#$ROOT_DIR/}"
}

main "$@"
