#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"

# Create venv on first run
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
fi

# On Debian/Ubuntu, venv can be created without pip if python3-venv is
# incomplete. Bootstrap pip with ensurepip if it's missing.
if [ ! -f "$VENV_DIR/bin/pip" ]; then
    echo "pip not found in venv, bootstrapping..."
    "$VENV_DIR/bin/python" -m ensurepip --upgrade 2>/dev/null || {
        echo ""
        echo "ERROR: could not bootstrap pip into the venv."
        echo "On Debian/Ubuntu, install the missing packages first:"
        echo ""
        echo "  sudo apt install python3-venv python3-pip"
        echo ""
        echo "Then delete .venv/ and re-run ./run.sh:"
        echo ""
        echo "  rm -rf .venv && ./run.sh"
        exit 1
    }
fi

# Install/update dependencies
"$VENV_DIR/bin/pip" install --quiet -r "$SCRIPT_DIR/requirements.txt"

# Run
"$VENV_DIR/bin/python" "$SCRIPT_DIR/main.py" "$@"
