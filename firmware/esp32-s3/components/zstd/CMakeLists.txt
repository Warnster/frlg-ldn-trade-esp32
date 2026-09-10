file(GLOB ZSTD_SRCS
     "${CMAKE_CURRENT_LIST_DIR}/lib/common/*.c"
     "${CMAKE_CURRENT_LIST_DIR}/lib/compress/*.c"
     "${CMAKE_CURRENT_LIST_DIR}/lib/decompress/*.c")

idf_component_register(SRCS ${ZSTD_SRCS}
                       INCLUDE_DIRS "lib")

# xtensa build: use the portable C Huffman decoder (no x86 asm) and namespace xxhash.
target_compile_definitions(${COMPONENT_LIB} PRIVATE ZSTD_DISABLE_ASM=1 XXH_NAMESPACE=ZSTD_)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-maybe-uninitialized -Wno-error=maybe-uninitialized)
