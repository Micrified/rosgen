#include "ros_env.h"

/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/

bool generate_cmake (ros_package_t *package_p)
{
	FILE *file = NULL;
	const char *project_name;
	ros_executor_t **executors = NULL;
	ros_value_t **dependencies = NULL;

	// Parameter check
	if (package_p == NULL) {
		fprintf(stderr, "%s:%d: Null parameter!\n", __FILE__, __LINE__);
		return false;
	} else {
		project_name = package_p->name->data.data_string;
		executors = package_p->executors;
		dependencies = package_p->dependencies;
	}



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
	for (ros_value_t **d = dependencies; *d != NULL; ++d) {
		fprintf(file, "find_package(%s REQUIRED)\n", (*d)->data.data_string);
	}
	fprintf(file, "\n");

	// Write all executors
	fprintf(file, "# Add executables and list their dependencies\n");
	for (ros_executor_t **e = executors; *e != NULL; ++e) {
		fprintf(file, "add_executable(executor_%ld src/executor_%ld.cpp)\n", 
			(*e)->id->data.data_long, (*e)->id->data.data_long);
	}
	fprintf(file, "\n");

	// Add dependencies to executors
	for (ros_executor_t **e = executors; *e != NULL; ++e) {
		fprintf(file, "ament_target_dependencies(executor_%ld", 
			(*e)->id->data.data_long);

		for (ros_value_t **d = dependencies; *d != NULL; ++d) {
			fprintf(file, " %s", (*d)->data.data_string);
		}
		fprintf(file, ")\n\n");
	}

	// Add install commands
	fprintf(file, "install(TARGETS\n");
	for (ros_executor_t **e = executors; *e != NULL; ++e) {
		fprintf(file, "  executor_%ld\n", (*e)->id->data.data_long);
	}
	fprintf(file, "  DESTINATION lib/${PROJECT_NAME})\n");

	// Add ament package line
	fprintf(file, "ament_package()\n");

	// Close the file
	fclose(file);

	return true;
}
