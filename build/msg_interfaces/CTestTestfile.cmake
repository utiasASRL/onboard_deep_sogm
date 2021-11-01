# CMake generated Testfile for 
# Source directory: /home/administrator/eloquent_ws/src/msg_interfaces
# Build directory: /home/administrator/eloquent_ws/build/msg_interfaces
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/eloquent/share/ament_cmake_test/cmake/run_test.py" "/home/administrator/eloquent_ws/build/msg_interfaces/test_results/msg_interfaces/lint_cmake.xunit.xml" "--package-name" "msg_interfaces" "--output-file" "/home/administrator/eloquent_ws/build/msg_interfaces/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/eloquent/bin/ament_lint_cmake" "--xunit-file" "/home/administrator/eloquent_ws/build/msg_interfaces/test_results/msg_interfaces/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/administrator/eloquent_ws/src/msg_interfaces")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/eloquent/share/ament_cmake_test/cmake/run_test.py" "/home/administrator/eloquent_ws/build/msg_interfaces/test_results/msg_interfaces/xmllint.xunit.xml" "--package-name" "msg_interfaces" "--output-file" "/home/administrator/eloquent_ws/build/msg_interfaces/ament_xmllint/xmllint.txt" "--command" "/opt/ros/eloquent/bin/ament_xmllint" "--xunit-file" "/home/administrator/eloquent_ws/build/msg_interfaces/test_results/msg_interfaces/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/administrator/eloquent_ws/src/msg_interfaces")
subdirs("msg_interfaces__py")
