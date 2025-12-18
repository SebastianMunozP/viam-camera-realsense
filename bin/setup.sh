#!/bin/bash
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
#
set -euxo pipefail

OS=$(uname -s | tr '[:upper:]' '[:lower:]')

# NOTE: this is written under the assumption that it will be built in canon

if  [[ ${OS} == "linux" ]]; then
    sudo apt -y update && sudo apt -y upgrade && sudo apt-get install -y python3-venv cmake
fi

if [ ! -f "./venv/bin/activate" ]; then
  echo 'creating and sourcing virtual env'
  python3 -m venv venv && source ./venv/bin/activate
else
  echo 'sourcing virtual env'
  source ./venv/bin/activate
fi

# Set up conan
if [ ! -f "./venv/bin/conan" ]; then
  echo 'installing conan'
  python3 -m pip install conan
fi

conan profile detect || echo "Conan is already installed"

conan remote add viamconan https://viam.jfrog.io/artifactory/api/conan/viamconan --force || echo "Viam conan remote already exists"