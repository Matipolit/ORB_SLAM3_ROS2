#!/bin/bash
CORE_DIR=$1
if [ -z "$CORE_DIR" ]; then echo "Podaj sciezke do silnika"; exit 1; fi

echo "Patching ORB_SLAM3 Core at $CORE_DIR"
sed -i 's/std=c++11/std=c++14/g' ${CORE_DIR}/CMakeLists.txt
find ${CORE_DIR}/src ${CORE_DIR}/include -type f \( -name "*.cc" -o -name "*.h" \) -exec \
    sed -i '1i #include <unistd.h>\n#include <stdint.h>\n#include <stdexcept>\n#include <stdio.h>\n#include <stdlib.h>' {} +

