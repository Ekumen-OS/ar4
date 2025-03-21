#!/bin/bash

. .venv/bin/activate

urdf2mjcf \
    --copy-meshes \
    --metadata-file meta/metadata.json \
    --output mjcf/mj_ar4.mjcf \
    urdf/mj_ar4.urdf
