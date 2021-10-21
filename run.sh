#!/bin/bash

set -euo pipefail

case "${1:-}" in
  ("server")
    meshcat-server
    ;;
  ("armcontrol")
    cd bazel-bin/apps/armcontrol/
    ./armcontrol
    ;;
  ("visualizer")
    cd bazel-bin/apps/visualizer/
    ./visualizer
    ;;
  ("obstaclefreeregion")
    cd bazel-bin/apps/obstaclefreeregion/
    ./obstaclefreeregion
    ;;
  ("planner")
    cd bazel-bin/apps/planner/
    ./planner
    ;;
  (*)
    echo "Usage: $0 <package>" 1>&2
    echo "where <package> is one of the following:" 1>&2
    echo "  server" 1>&2
    echo "  armcontrol" 1>&2
    echo "  visualizer" 1>&2
    echo "  obstaclefreeregion" 1>&2
    echo "  planner" 1>&2
    exit 1
    ;;
esac
