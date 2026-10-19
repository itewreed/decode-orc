set(STAGE_PLUGIN_SOURCES
    plugin.cpp
    videodat_sink_stage.cpp
    videodat_sink_deps.cpp
    videodat_pipeline.cpp
    videodat_stream_decoder.cpp
    videodat_line_slicer.cpp
    videodat_packet.cpp
    videodat_file_assembler.cpp
    videodat_services.cpp
)

orc_add_stage_plugin(
    orc-stage-plugin-videodat-sink
    OUTPUT_NAME orc-stage-plugin-videodat-sink
    PLUGIN_VERSION "${ORC_VERSION}"
    SOURCES ${STAGE_PLUGIN_SOURCES}
)
