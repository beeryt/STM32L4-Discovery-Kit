# set the system name and processor to specify cross-compiling
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_PREFIX arm-none-eabi-)

# specify where to find compiler tools
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size)

# when cmake tries to compile a simple test program the arm linker will fail
# with "undefined reference to _exit". Instead we specify that cmake should
# should avoid using the linker in this step by compiling a static library.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
