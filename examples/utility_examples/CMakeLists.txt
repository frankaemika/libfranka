set(EXAMPLES
  logging_example
)

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_include_directories(${example} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${example} Franka::Franka examples_common fmt::fmt)
  
  install(TARGETS ${example}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endforeach()
