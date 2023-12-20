#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/bin/bash
RELEASE_DIR="release-candidate"
# RELEASE_TAR="${RELEASE_DIR}.tgz"
RELEASE_TAR="mace.zip"
DIR=$(dirname ${BASH_SOURCE})
cd ${DIR}
rm ${RELEASE_TAR} || true
rm -rf ${RELEASE_DIR} || true
mkdir -p ${RELEASE_DIR}

#Make sure CMakeLists.txt isn't corrupted (this can cause the build to fail)
echo "/opt/ros/melodic/share/catkin/cmake/toplevel.cmake" > code/stomp/ros_sim/src/CMakeLists.txt

#Build the docker container
echo "Building docker container."
docker build . --tag mace 
echo "Saving docker container."
docker save mace | gzip > ${RELEASE_DIR}/mace-docker.tar.gz
echo "Docker container saved."
cp docker-compose.yaml ${RELEASE_DIR}/

#Copy windows install/startup scripts to top level dir
cp mace.bat ${RELEASE_DIR}/
cp install_mace.bat ${RELEASE_DIR}/

#Copy STARTUP-GUIDE.html and supporting media
cp STARTUP-GUIDE.html ${RELEASE_DIR}/
cp -r docs/media ${RELEASE_DIR}/docs/media

#Copy elements from the docs folder
mkdir -p ${RELEASE_DIR}/docs/manuals
cp -r docs/manuals/*.pdf ${RELEASE_DIR}/docs/manuals
cp -r docs/examples ${RELEASE_DIR}/docs

#Copy elements from the scripts folder
mkdir -p ${RELEASE_DIR}/scripts
cp -r scripts/*.sh ${RELEASE_DIR}/scripts/
cp -r scripts/*.bat ${RELEASE_DIR}/scripts/
cp -r scripts/debugging ${RELEASE_DIR}/scripts/
cp -r scripts/install ${RELEASE_DIR}/scripts/

#Copy elements from the python folder
mkdir -p ${RELEASE_DIR}/src
cp -r src ${RELEASE_DIR}/src

#Build STOMP
echo "Building STOMP release candidate."
pushd code/stomp > /dev/null
./gradlew clean assembleRelease > /dev/null
popd > /dev/null
mkdir -p ${RELEASE_DIR}/code
cp -r code/stomp/ReleaseCandidate ${RELEASE_DIR}/code/stomp

#Tar and GZIP
echo "Compressing release."
# tar zcf ${RELEASE_TAR} ${RELEASE_DIR}
zip -r -q ${RELEASE_TAR} ${RELEASE_DIR}
echo 'Finished.'
