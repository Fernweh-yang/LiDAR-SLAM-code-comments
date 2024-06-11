#!/usr/bin/bash

set -e

urlencode() {
  python -c 'import urllib.parse, sys; print(urllib.parse.quote(sys.argv[1], sys.argv[2]))' \
    "$1" "$urlencode_safe"
}

pip3 install wheel
password=$(urlencode "$PASSWORD")
cd /tmp
echo "http://${USERNAME}:${password}@10.10.88.24/algorithm_toolchain/algorithm/hxbasic.git"
git clone http://${USERNAME}:${password}@10.10.88.24/algorithm_toolchain/algorithm/hxbasic.git
cd hxbasic
make install-core-deps
rm -rf /tmp/hxbasic
