idf_component_register(SRCS "ui_html.c" "ui_server.c" "main.c" "detector.cpp" "control.c" "blob_rect.cpp" "point.cpp" "wifi.c" "model.cpp"
                    INCLUDE_DIRS ".")

add_custom_command(OUTPUT "ui_html.h" "ui_html.c"
    COMMAND python ${COMPONENT_DIR}/encode_ui_html.py ${COMPONENT_DIR}
    DEPENDS "${COMPONENT_DIR}/index.html" "${COMPONENT_DIR}/encode_ui_html.py"
    VERBATIM)

add_custom_target(html DEPENDS "ui_html.h" "ui_html.c")
add_dependencies("${COMPONENT_LIB}" html)

set_property(DIRECTORY "${COMPONENT_DIR}" APPEND PROPERTY
    ADDITIONAL_CLEAN_FILES "ui_html.h" "ui_html.c")
