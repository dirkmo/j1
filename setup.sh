#! /bin/bash

python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt

echo
echo To setup the Python venv in VS Code, press Ctrl+Shift+p and select "Python: Select Environment".
