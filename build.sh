TARGETS="armeabi-v7a x86  arm64-v8a x86_64"

# TARGETS="x86"
mkdir -p "build"

for TARGET in ${TARGETS}
do    
    # create one build dir per target architecture
    mkdir -p build/${TARGET}
    cd build/${TARGET}

    cmake -DANDROID_NATIVE_API_LEVEL=21 -DCMAKE_TOOLCHAIN_FILE=$NDK/build/cmake/android.toolchain.cmake -DANDROID_NDK=$NDK -DCMAKE_BUILD_TYPE=Debug -DANDROID_ABI=${TARGET} ../../

    make -j32

    cd -
done