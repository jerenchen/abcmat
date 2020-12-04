
# example data file
if(NOT EXISTS octopus_example.tgz)
set(ABC_URL https://storage.googleapis.com/google-code-archive-downloads/v2/code.google.com/alembic/Alembic_Octopus_Example.tgz)
file(
  DOWNLOAD ${ABC_URL} octopus_example.tgz
  TIMEOUT 60
)
endif()
file(ARCHIVE_EXTRACT INPUT octopus_example.tgz DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
add_definitions(-DOCTOPUS_FILE=\"${CMAKE_CURRENT_BINARY_DIR}/Alembic_Octopus_Example/alembic_octopus.abc\")

# example project/source
add_subdirectory(octread)