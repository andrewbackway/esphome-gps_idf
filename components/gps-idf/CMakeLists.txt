idf_component_register(
    SRCS "gps-idf.cpp" "TinyGPS++.cpp"
    INCLUDE_DIRS "."
    REQUIRES log
)
