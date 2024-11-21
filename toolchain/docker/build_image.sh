#!/bin/bash

docker build \
  --platform linux/amd64 \
  -t misc:0.0.0 \
  "$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"
