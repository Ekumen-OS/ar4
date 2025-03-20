# Description

This package contains the simulation of the AR4 robot in Mujoco.

## Instructions to import the model

The Mujoco model of the AR4 robot can be imported from the URDF description in the `ar4_description` package. The process requires running the two scripts in the `scripts/` folder in sequence.

1. First run `stage1_create_urdf_from_xacro.sh` from **within** the development container. This script will generate the URDF file from the xacro description.
1. Then run `stage2_create_mjcf_from_urdf.sh` from **outside** the development container (this script needs to create a new containerized environment running Ubuntu 24.04). This script will generate the Mujoco model from the URDF file.

At this point you'll see the `urdf/` and `mjcf/` folders in the `ar4_mujoco_sim` package as updated. Commit those changes.
