# gba_link — Gen 3 link-CABLE cart side (docs/25). Hardware-independent layers build today;
# the physical layer (gba_link_*.c) and trade sections land as they're ported.
idf_component_register(
    SRCS
        "gba_packet.c"
    INCLUDE_DIRS "."
)
