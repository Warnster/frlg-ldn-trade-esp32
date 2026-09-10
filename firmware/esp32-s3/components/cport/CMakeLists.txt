# cport: the FRLG<->LDN trade brain (rfu/gbaframe/block/reliable/pia_zstd/crypto/ccmp/
# ldn_advert/ni/wap), shared with the host build. Portable C; AES/SHA via mbedtls, zstd via
# the vendored Xtensa component. Sources referenced in-place from gba-switch-bridge/cport.
set(CPORT /home/lewis/projects/gba-switch-adapter/gba-switch-bridge/cport)
idf_component_register(
    SRCS
        "${CPORT}/rfu.c" "${CPORT}/gbaframe.c" "${CPORT}/block.c" "${CPORT}/reliable.c"
        "${CPORT}/pia_zstd.c" "${CPORT}/crypto.c" "${CPORT}/ccmp.c" "${CPORT}/ldn_advert.c"
        "${CPORT}/ni.c" "${CPORT}/wap.c" "${CPORT}/wonder_card.c" "${CPORT}/charmap.c"
        "${CPORT}/pia_connect.c"
        "${CPORT}/sim.c"
        "${CPORT}/ldn_auth.c"
        "${CPORT}/aes_backend_psa.c"
    INCLUDE_DIRS "${CPORT}"
    REQUIRES mbedtls zstd
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)
