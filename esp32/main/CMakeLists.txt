idf_component_register(SRCS "ui_server.c" "main.c" "detector.cpp" "control.c" "blob_rect.cpp" "point.cpp" "wifi.c" "model.cpp"
                    INCLUDE_DIRS "."
                    EMBED_FILES index.html ifttt_tls_root.pem)