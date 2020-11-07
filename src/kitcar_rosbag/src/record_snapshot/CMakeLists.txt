
add_executable(record_snapshot
  src/record_snapshot/record_snapshot.cpp
)

add_dependencies(record_snapshot
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(record_snapshot
  ${catkin_LIBRARIES}
)
