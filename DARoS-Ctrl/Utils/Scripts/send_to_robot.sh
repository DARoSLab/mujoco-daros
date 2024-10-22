#!/bin/bash
set -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
echo ${DIR}

cd ${DIR}/../../robot-build/
rm -rf daros-lab
mkdir daros-lab
mkdir daros-lab/build
#cp common/test-common daros-lab/build
cp $1 daros-lab/build
find . -name \*.so* -exec cp {} ./daros-lab/build \;
cp ${DIR}/run_* ./daros-lab/build
cp ${DIR}/setup_network_mc.py ./daros-lab/build
cp ${DIR}/run_mc_debug.sh ./daros-lab/build
cp ${DIR}/run_daros.sh ./daros-lab/build
cp ${DIR}/run_guide-dog.sh ./daros-lab/build
cp -r ../independent/speechRecognition ./daros-lab/build
cp -r ../config daros-lab

mkdir daros-lab/systems

# copy over librealsense library/ pytorch library
#cp /usr/local/lib/librealsense* ./daros-lab/build
#cp ${DIR}/../../third-parties/libtorch/libtorch_linux/lib/*.so* ./daros-lab/build
cp /usr/local/lib/liblcm.* ./daros-lab/build
cp /usr/local/lib/libipopt.* ./daros-lab/build
cp /usr/local/lib/libcoinhsl.* ./daros-lab/build
cp /usr/lib/x86_64-linux-gnu/liblapack.* ./daros-lab/build
cp /usr/lib/x86_64-linux-gnu/libblas.* ./daros-lab/build
cp /usr/lib/x86_64-linux-gnu/libgfortran.* ./daros-lab/build

DATE=$(date +"%Y%m%d%H%M")
scp -r daros-lab user@10.0.0.40:~/


