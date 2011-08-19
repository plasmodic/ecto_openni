macro(pytest pyfile)
  string(REPLACE ";" " " args "${ARGN}")
  add_test(opencv_${pyfile}
    ${CMAKE_CURRENT_SOURCE_DIR}/run_test.sh ${CMAKE_BINARY_DIR}/python_path.sh ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
    )
endmacro()

pytest(test_import)
