#!/bin/bash
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
#

set -euxo pipefail


OS=$(uname -s | tr '[:upper:]' '[:lower:]')

# NOTE: this is written under the assumption that it will be built in canon
if [[ ${OS} == "linux" ]]; then
    sudo apt -y update && sudo apt -y upgrade && sudo apt-get install -y \
        python3 \
        python3-venv \
        python3-pip \
        cmake \
        cmake-data \
        autoconf \
        automake \
        build-essential \
        ca-certificates \
        curl \
        doxygen \
        g++ \
        git \
        gdb \
        gnupg \
        gpg \
        less \
        libssl-dev \
        libudev-dev \
        ninja-build \
        pkg-config \
        software-properties-common \
        wget
fi

# Check python3 availability
if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 not found in PATH. Aborting." >&2
  exit 1
fi

# Check venv module
if ! python3 -m venv --help >/dev/null 2>&1; then
  echo "python3 venv module not available. Try: sudo apt-get install --reinstall python3-venv python3-full python3-pip" >&2
  exit 1
fi


if [ ! -f "./venv/bin/activate" ]; then
  echo 'creating and sourcing virtual env'
  if ! python3 -m venv venv; then
    echo "Failed to create venv. Trying with full path to python3."
    if ! /usr/bin/python3 -m venv venv; then
      echo "Failed to create venv with /usr/bin/python3. Aborting." >&2
      exit 1
    fi
  fi
  source ./venv/bin/activate
else
  echo 'sourcing virtual env'
  source ./venv/bin/activate
fi


# Set up conan
if [ ! -f "./venv/bin/conan" ]; then
  echo 'installing conan'
  . ./venv/bin/activate
  pip install --upgrade pip
  pip install conan
fi


conan profile detect || echo "Conan is already installed"
conan remote add viamconan https://viam.jfrog.io/artifactory/api/conan/viamconan --index 0 || echo "Viam conan remote already exists"

