idf_component_register(
    SRCS "src/ES_PH_SOIL.cpp"
    INCLUDE_DIRS "include"
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)