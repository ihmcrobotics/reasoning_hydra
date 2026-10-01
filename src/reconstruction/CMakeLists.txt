target_sources(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/marching_cubes.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/mesh_integrator.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/mesh_integrator_config.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/projection_interpolators.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/projective_integrator.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/projective_integrator_config.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/reconstruction_module.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/reconstruction_output.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/semantic_integrator.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/tsdf_interpolators.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/volumetric_map.cpp
)

# Optional geometry staging library; backend modes remain gated until semantic
# conversion and archival are implemented. CPU-only builds need no CUDA SDK.
option(HYDRA_ENABLE_NVBLOX_STAGE "Build internal nvblox geometry staging" OFF)
if(HYDRA_ENABLE_NVBLOX_STAGE)
  find_package(CUDAToolkit REQUIRED)
  find_package(glog REQUIRED)
  find_package(gflags REQUIRED)
  find_package(nvblox CONFIG REQUIRED)
  target_compile_definitions(${PROJECT_NAME} PRIVATE HYDRA_ENABLE_NVBLOX_STAGE=1)
  target_sources(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/nvblox_reconstruction_stage.cpp)
  target_link_libraries(${PROJECT_NAME} PRIVATE nvblox::nvblox_lib CUDA::cudart)
endif()

if(HYDRA_ENABLE_NVBLOX_STAGE AND HYDRA_ENABLE_TESTS)
  add_executable(hydra_nvblox_contract_check
    ${PROJECT_SOURCE_DIR}/tests/reconstruction/nvblox_contract_check.cpp)
  target_link_libraries(hydra_nvblox_contract_check PRIVATE ${PROJECT_NAME})
  add_test(NAME hydra_nvblox_contract_check COMMAND hydra_nvblox_contract_check)
  set_tests_properties(hydra_nvblox_contract_check PROPERTIES LABELS "gpu;nvblox")
endif()
