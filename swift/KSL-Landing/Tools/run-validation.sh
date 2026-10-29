#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p Validation
swift --version > Validation/toolchain.txt
swift build -c release
swift test -c release 2>&1 | tee Validation/swift-tests.log
python3 Tools/verify_mock_krpc.py
python3 Tools/validate.py
