#!/bin/bash
# Download the GeographicLib datasets MAVROS uses into $GEOGRAPHICLIB_DATA (default: ./.geographiclib).
# Same datasets as MAVROS's install_geographiclib_datasets.sh, but no root needed:
#   geoids/egm96-5     REQUIRED: MAVROS's UAS node shuts down without it
#   gravity/egm96      optional
#   magnetic/emm2015   optional
# Run via `pixi run geoids` (needs internet once). Safe to re-run; existing data is kept.

set -eo pipefail

DATA="${GEOGRAPHICLIB_DATA:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/.geographiclib}"
BASE="https://sourceforge.net/projects/geographiclib/files"
mkdir -p "$DATA"

fetch() {   # fetch <kind> <name> <check-file> <required:yes|no>
    local kind="$1" name="$2" check="$3" required="$4"
    if [[ -f "$DATA/$check" ]]; then
        echo "[geoids] $kind/$name already installed"
        return 0
    fi
    echo "[geoids] Downloading $kind/$name ..."
    local tmp
    tmp="$(mktemp)"
    if curl -fL --retry 3 --connect-timeout 20 -o "$tmp" "$BASE/${kind}-distrib/${name}.tar.bz2/download" \
        && tar -xjf "$tmp" -C "$DATA"; then
        rm -f "$tmp"
        if [[ -f "$DATA/$check" ]]; then
            echo "[geoids] $kind/$name installed"
            return 0
        fi
    fi
    rm -f "$tmp"
    if [[ "$required" == yes ]]; then
        echo "[geoids] ERROR: could not install $kind/$name (MAVROS will not start without it)." >&2
        echo "[geoids] Check internet access, or copy a 'geoids/egm96-5.pgm' into $DATA/geoids/ by hand." >&2
        exit 1
    fi
    echo "[geoids] WARNING: optional $kind/$name not installed"
}

fetch geoids   egm96-5 geoids/egm96-5.pgm      yes
fetch gravity  egm96   gravity/egm96.egm       no
fetch magnetic emm2015 magnetic/emm2015.wmm    no
echo "[geoids] Data in $DATA"
