SET (CMAKE_CROSSCOMPILING   TRUE)
SET (CMAKE_SYSTEM_NAME      "Darwin")
SET (CMAKE_SYSTEM_PROCESSOR "x86_64")

SET (SDKVER     "9.3")

SET (DEVROOT    "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain")
SET (SDKROOT    "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneSimulator.platform/Developer/SDKs/iPhoneSimulator${SDKVER}.sdk")
SET (CMAKE_C_COMPILER         "${DEVROOT}/usr/bin/clang")
SET (CMAKE_CXX_COMPILER        "${DEVROOT}/usr/bin/clang++")

SET (CMAKE_FIND_ROOT_PATH               "${SDKROOT}" "${DEVROOT}")
SET (CMAKE_FIND_ROOT_PATH_MODE_PROGRAM  NEVER)
SET (CMAKE_FIND_ROOT_PATH_MODE_LIBRARY  ONLY)
SET (CMAKE_FIND_ROOT_PATH_MODE_INCLUDE  ONLY)