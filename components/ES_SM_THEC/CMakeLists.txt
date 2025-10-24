idf_component_register(
    SRCS "src/ES_SM_THEC.cpp"
    INCLUDE_DIRS "include"
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)