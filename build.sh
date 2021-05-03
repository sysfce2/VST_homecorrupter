#!/usr/bin/env bash
clear
echo "Flushing build caches and output folders"
rm -rf build
echo "Creating build folders"
mkdir build
cd build

# get path to VST3 SDK
if [ -n "${VST3_SDK_ROOT}" ]; then
  DVST3_SDK_ROOT="-DVST3_SDK_ROOT=${VST3_SDK_ROOT}"
fi

# get optional argument for plugin type (vst3, vst2 or au)
while getopts t: flag
do
    case "${flag}" in
        t) TYPE=${OPTARG};;
    esac
done

# build for requested plugin type
case "$TYPE" in
    au) echo "Building Audio Unit plugin...";
    cmake "-G" "Xcode" "-DJAMBA_ENABLE_AUDIO_UNIT=ON" ${DVST3_SDK_ROOT} ..;
    sh jamba.sh -r install-au;
    cd ..;;

    vst2) echo "Building VST2 plugin...";
    cmake "-DBUILD_VST2=ON" "-DCMAKE_OSX_ARCHITECTURES=x86_64" ${DVST3_SDK_ROOT} ..;
    make;;

    *) echo "Building VST3 plugin...";
    cmake "-DCMAKE_OSX_ARCHITECTURES=x86_64" ${DVST3_SDK_ROOT} ..;
    make;;
esac
