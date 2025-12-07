#!/bin/bash

cd /app

if [ -z "${PACKAGE}" ]; then
    echo "PACKAGE is not set or blank, skipping installation."
else
    COMMIT=${COMMIT:-0000000}
    echo "Installing ${PACKAGE}@${COMMIT}..."
    SETUPTOOLS_SCM_PRETEND_VERSION="0.0.post0.dev38+${COMMIT}.dirty" /venv/bin/python -m pip install --no-cache-dir -e /app
fi

echo "Executing CMD"
exec "$@"