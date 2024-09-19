#!/bin/bash

docker build \
  --platform linux/amd64 \
  -t dumpling:cuttle \
  "$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"
