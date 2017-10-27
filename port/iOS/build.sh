#!/bin/bash

#
# Written and maintained by the.arul@gmail.com (2014)
#

BUILD_DIR="./lib/iOS"

IOS_SDK_TARGET=9.3
#(iPhoneOS iPhoneSimulator) -- determined from arch
IOS_SDK_DEVICE=

XCODE_ROOT_DIR=/Applications/Xcode.app/Contents
TOOLCHAIN=$XCODE_ROOT_DIR//Developer/Toolchains/XcodeDefault.xctoolchain

BUILD_ARCHS_DEVICE="armv7 armv7s arm64"
BUILD_ARCHS_SIMULATOR="i386 x86_64"
BUILD_ARCHS_ALL=(armv7 armv7s arm64 i386 x86_64)

CPP_DEV_TARGET_LIST=(miphoneos-version-min mios-simulator-version-min)
CPP_DEV_TARGET=
CPP_STD_LIB_LIST=(libc++ libstdc++)
CPP_STD_LIB=
CPP_STD_LIST=(c++11 c++14)
CPP_STD=

function join { local IFS="$1"; shift; echo "$*"; }

build_arch()
{
    IOS_SDK_DEVICE=iPhoneOS
    CPP_DEV_TARGET=${CPP_DEV_TARGET_LIST[0]}

    if [[ "$BUILD_ARCHS_SIMULATOR" =~ "$1" ]]
    then
        echo '[!] Target SDK set to SIMULATOR.'
        IOS_SDK_DEVICE=iPhoneSimulator
        CPP_DEV_TARGET=${CPP_DEV_TARGET_LIST[1]}
    else
        echo '[!] Target SDK set to DEVICE.'
    fi

    unset DEVROOT SDKROOT CFLAGS LDFLAGS CPPFLAGS CXXFLAGS

    export DEVROOT=$XCODE_ROOT_DIR/Developer/Platforms/$IOS_SDK_DEVICE.platform/Developer
    export SDKROOT=$DEVROOT/SDKs/$IOS_SDK_DEVICE.sdk
    export CFLAGS="-arch $1 -pipe -no-cpp-precomp -stdlib=$CPP_STD_LIB -isysroot $SDKROOT -$CPP_DEV_TARGET=$IOS_SDK_TARGET -I$SDKROOT/usr/include/"
    export LDFLAGS="-L$SDKROOT/usr/lib/"
    export CPPFLAGS=$CFLAGS
    export CXXFLAGS="$CFLAGS -std=$CPP_STD"

    rm CMakeCache.txt

    cmake  -G 'Unix Makefiles' -DCMAKE_TOOLCHAIN_FILE=./port/iOS/IPHONEOS_$(echo $1 | tr '[:lower:]' '[:upper:]')_TOOLCHAIN.cmake -DENABLE_BOOST_WORKAROUND=ON -DBUILD_SHARED_LIBS=OFF -DASSIMP_BUILD_AMF_IMPORTER=OFF -DASSIMP_BUILD_3DS_IMPORTER=OFF -DASSIMP_BUILD_AC_IMPORTER=OFF -DASSIMP_BUILD_ASE_IMPORTER=OFF -DASSIMP_BUILD_ASSBIN_IMPORTER=OFF -DASSIMP_BUILD_ASSXML_IMPORTER=OFF -DASSIMP_BUILD_B3D_IMPORTER=OFF -DASSIMP_BUILD_BVH_IMPORTER=OFF -DASSIMP_BUILD_COLLADA_IMPORTER=OFF -DASSIMP_BUILD_DXF_IMPORTER=OFF -DASSIMP_BUILD_CSM_IMPORTER=OFF -DASSIMP_BUILD_HMP_IMPORTER=OFF -DASSIMP_BUILD_IRR_IMPORTER=OFF -DASSIMP_BUILD_LWO_IMPORTER=OFF -DASSIMP_BUILD_IRRMESH_IMPORTER=OFF -DASSIMP_BUILD_LWS_IMPORTER=OFF -DASSIMP_BUILD_MD2_IMPORTER=OFF -DASSIMP_BUILD_MD3_IMPORTER=OFF -DASSIMP_BUILD_MD5_IMPORTER=OFF -DASSIMP_BUILD_MDC_IMPORTER=OFF -DASSIMP_BUILD_MDL_IMPORTER=OFF -DASSIMP_BUILD_MMD_IMPORTER=OFF -DASSIMP_BUILD_NFF_IMPORTER=OFF -DASSIMP_BUILD_NDO_IMPORTER=OFF -DASSIMP_BUILD_OFF_IMPORTER=OFF -DASSIMP_BUILD_OGRE_IMPORTER=OFF -DASSIMP_BUILD_OPENGEX_IMPORTER=OFF -DASSIMP_BUILD_PLY_IMPORTER=OFF -DASSIMP_BUILD_MS3D_IMPORTER=OFF -DASSIMP_BUILD_COB_IMPORTER=OFF -DASSIMP_BUILD_BLEND_IMPORTER=OFF -DASSIMP_BUILD_IFC_IMPORTER=OFF -DASSIMP_BUILD_XGL_IMPORTER=OFF -DASSIMP_BUILD_FBX_IMPORTER=OFF -DASSIMP_BUILD_Q3D_IMPORTER=OFF -DASSIMP_BUILD_Q3BSP_IMPORTER=OFF -DASSIMP_BUILD_RAW_IMPORTER=OFF -DASSIMP_BUILD_SIB_IMPORTER=OFF -DASSIMP_BUILD_SMD_IMPORTER=OFF -DASSIMP_BUILD_STL_IMPORTER=OFF -DASSIMP_BUILD_TERRAGEN_IMPORTER=OFF -DASSIMP_BUILD_3D_IMPORTER=OFF -DASSIMP_BUILD_3MF_IMPORTER=OFF -DASSIMP_BUILD_X_IMPORTER=OFF -DASSIMP_BUILD_X3D_IMPORTER=OFF

    echo "[!] Building $1 library"

    $XCODE_ROOT_DIR/Developer/usr/bin/make clean
    $XCODE_ROOT_DIR/Developer/usr/bin/make assimp -j 8 -l

    echo "[!] Moving built library into: $BUILD_DIR/$1/"

    mv ./lib/libassimp.a $BUILD_DIR/$1/
}

echo "[!] $0 - assimp iOS build script"

CPP_STD_LIB=${CPP_STD_LIB_LIST[0]}
CPP_STD=${CPP_STD_LIST[0]}
DEPLOY_ARCHS=${BUILD_ARCHS_ALL[*]}
DEPLOY_FAT=1

for i in "$@"; do
    case $i in
    -s=*|--std=*)
        CPP_STD=`echo $i | sed 's/[-a-zA-Z0-9]*=//'`
        echo "[!] Selecting c++ standard: $CPP_STD"
    ;;
    -l=*|--stdlib=*)
        CPP_STD_LIB=`echo $i | sed 's/[-a-zA-Z0-9]*=//'`
        echo "[!] Selecting c++ std lib: $CPP_STD_LIB"
    ;;
    -a=*|--archs=*)
        DEPLOY_ARCHS=`echo $i | sed 's/[-a-zA-Z0-9]*=//'`
        echo "[!] Selecting architectures: $DEPLOY_ARCHS"
    ;;
    -n|--no-fat)
        DEPLOY_FAT=0
        echo "[!] Fat binary will not be created."
    ;;
    -h|--help)
        echo " - don't build fat library (--no-fat)."
        echo " - supported architectures (--archs):  $(echo $(join , ${BUILD_ARCHS_ALL[*]}) | sed 's/,/, /g')"
        echo " - supported C++ STD libs (--stdlib): $(echo $(join , ${CPP_STD_LIB_LIST[*]}) | sed 's/,/, /g')"
        echo " - supported C++ standards (--std): $(echo $(join , ${CPP_STD_LIST[*]}) | sed 's/,/, /g')"
        exit
    ;;
    *)
    ;;
    esac
done

cd ../../
rm -rf $BUILD_DIR

for ARCH_TARGET in $DEPLOY_ARCHS; do
    mkdir -p $BUILD_DIR/$ARCH_TARGET
    build_arch $ARCH_TARGET
    #rm ./lib/libassimp.a
done

if [[ "$DEPLOY_FAT" -eq 1 ]]; then
    echo '[+] Creating fat binary ...'
    for ARCH_TARGET in $DEPLOY_ARCHS; do
        LIPO_ARGS="$LIPO_ARGS-arch $ARCH_TARGET $BUILD_DIR/$ARCH_TARGET/libassimp.a "
    done
    LIPO_ARGS="$LIPO_ARGS-create -output $BUILD_DIR/libassimp-fat.a"
    lipo $LIPO_ARGS
    echo "[!] Done! The fat binary can be found at $BUILD_DIR"
fi


