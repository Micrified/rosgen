#include "ros_env.h"

/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/


bool generate_cmake (const char *project_name, const char **executables, 
	const char **dependencies)
{
	FILE *file = NULL;

	// Create the file
	if ((file = fopen("CMakeLists.txt", "w")) == NULL) {
		fprintf(stderr, "Unable to create writable file/stream \
			CMakeLists.txt\n");
		return false;
	}

	// Write the header information and project name
	fprintf(file, "cmake_minimum_required(VERSION 3.5)\n");
	fprintf(file, "project(%s)\n\n", project_name);

	// Write all dependencies
	fprintf(file, "# Find dependencies\n");
	for (const char **dep = dependencies; *dep != NULL; ++dep) {
		fprintf(file, "find_package(%s REQUIRED)\n", *dep);
	}
	fprintf(file, "\n");

	// Write all executables
	fprintf(file, "# Add executables and list their dependencies\n");
	for (const char **exec = executables; *exec != NULL; ++exec) {
		fprintf(file, "add_executable(%s src/%s.cpp)\n", *exec, *exec);
	}
	fprintf(file, "\n");

	// Add dependencies to executables
	for (const char **exec = executables; *exec != NULL; ++exec) {
		fprintf(file, "ament_target_dependencies(%s", *exec);

		for (const char **dep = dependencies; *dep != NULL; ++dep) {
			fprintf(file, " %s", *dep);
		}
		fprintf(file, ")\n\n");
	}

	// Add install commands
	fprintf(file, "install(TARGETS\n");
	for (const char **exec = executables; *exec != NULL; ++exec) {
		fprintf(file, "  %s\n", *exec);
	}
	fprintf(file, "  DESTINATION lib/${PROJECT_NAME})\n");

	// Add ament package line
	fprintf(file, "ament_package()\n");

	// Close the file
	fclose(file);

	return true;
}

cmake_minimum_required(VERSION 3.5)
project(automatic)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)

add_executable(executor_1 src/executor_1.cpp)
add_executable(executor_2 src/executor_2.cpp)
ament_target_dependencies(executor_1 rclcpp tutorial_interfaces)
ament_target_dependencies(executor_2 rclcpp tutorial_interfaces)

install(TARGETS
  executor_1
  executor_2
  DESTINATION lib/${PROJECT_NAME})


ament_package()
