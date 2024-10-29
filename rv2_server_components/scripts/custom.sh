# This is an optional script that will be executed under `create-service` command in the `rv2_startup` script.
# The script will be executed before the `runfile.sh` is created, before the service file is created.

#!/bin/bash

# echo "[custom]"

# The argument $1 should be the system.yaml file path.
# The argument $2 should be the repo path, the path could be under ROS2 workspace source directory or the ROS2 global share directory.
# echo ${@}

# ref: https://stackoverflow.com/a/47791935
# yaml file_path key
# Ex: local val=$(yaml /path/to/file.yaml "['custom']['key']")
# yaml ()
# {
#     python3 -c "import yaml;print(yaml.safe_load(open('$1'))$2)" 2>/dev/null
# }
