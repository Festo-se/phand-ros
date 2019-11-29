#!/bin/bash
cd "$(dirname "$0")/../debug_ui/"

python -m SimpleHTTPServer 7954
