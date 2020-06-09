#include "ros_env.h"

/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/

bool generate_callback_header_file (ros_package_t *package_p)
{
	FILE *header_file = NULL;
	const char *msg_type = NULL;
	const char *msg_include = NULL;
	const char *filename = ROS_CALLBACK_FILENAME;

	// Parameter check
	if (package_p == NULL) {
		fprintf(stderr, "%s:%d: Null parameter!\n", __FILE__, __LINE__);
		return false;
	} else {
		msg_type = package_p->msg_type->data.data_string;
		msg_include = package_p->msg_include->data.data_string;
	}

	// Create the header file
	if ((header_file = fopen(ROS_CALLBACK_FILENAME ".hpp", "w")) == NULL) {
		fprintf(stderr, "Unable to create writable file/stream "\
			ROS_CALLBACK_FILENAME ".hpp\n");
		return false;
	}

	// Write the header 
	fprintf(header_file, "#if !defined(");
	for (const char *p = filename; *p != '\0'; ++p) {
		fputc(toupper(*p), header_file);
	}
	fprintf(header_file, "_H)\n");
	fprintf(header_file, "#define ");
	for (const char *p = filename; *p != '\0'; ++p) {
		fputc(toupper(*p), header_file);
	}
	fprintf(header_file, "_H\n");

	// Put required include statments
	fprintf(header_file, "#include \"%s\"\n", "rclcpp/rclcpp.hpp");

	// Put message include statement
	fprintf(header_file, "#include \"%s\"\n", msg_include);

	// Put function declaration
	fprintf(header_file, "%s *%s (%s%s %s, %s, %s, %s, %s, %s, %s);\n",
		msg_type,
		ROS_CALLBACK_SIGNATURE,
		msg_type, "::SharedPtr", "msg_recv",
		"int64_t executor_id",
		"const char *node_name",
		"const char *callback_name",
		"int64_t callback_priority",
		"int64_t callback_wcet_ns",
		"bool is_timer_triggered");

	// Put end of file marker
	fprintf(header_file, "#endif\n");

	// Close the header file
	fclose(header_file);

	return true;
}

bool generate_callback_source_file (ros_package_t *package_p)
{
	FILE *source_file = NULL;
	const char *msg_type = NULL;
	const char *filename = ROS_CALLBACK_FILENAME;

	// Parameter check
	if (package_p == NULL) {
		fprintf(stderr, "%s:%d: Null parameter!\n", __FILE__, __LINE__);
		return false;
	} else {
		msg_type = package_p->msg_type->data.data_string;
	}

	// Create the source file
	if ((source_file = fopen(ROS_CALLBACK_FILENAME ".cpp", "w")) == NULL) {
		fprintf(stderr, "Unable to create writable file/stream "\
			ROS_CALLBACK_FILENAME ".cpp\n");
		return false;
	}

	// Print the include directive
	fprintf(source_file, "#include \"%s/%s.hpp\"\n\n",
		package_p->name->data.data_string, ROS_CALLBACK_FILENAME);

	// Print the function definition
	fprintf(source_file, "%s *%s (%s%s\n\t%s,\n\t%s,\n\t%s,"\
		"\n\t%s,\n\t%s,\n\t%s,\n\t%s)\n",
		msg_type,
		ROS_CALLBACK_SIGNATURE,
		msg_type, "::SharedPtr", "msg_recv",
		"int64_t executor_id",
		"const char *node_name",
		"const char *callback_name",
		"int64_t callback_priority",
		"int64_t callback_wcet_ns",
		"bool is_timer_triggered");
	fprintf(source_file, "{\n\t// TODO\n\treturn NULL;\n}\n\n");

	// Close the source file
	fclose(source_file);

	return true;
}

bool generate_package_xml (ros_package_t *package_p)
{
	FILE *file = NULL;
	const char *project_name;
	ros_value_t **dependencies = NULL;

	// Parameter check
	if (package_p == NULL) {
		fprintf(stderr, "%s:%d: Null parameter!\n", __FILE__, __LINE__);
		return false;
	} else {
		project_name = package_p->name->data.data_string;
		dependencies = package_p->dependencies;
	}

	// Create the file
	if ((file = fopen("package.xml", "w")) == NULL) {
		fprintf(stderr, "Unable to create writable file/stream \
			package.xml\n");
		return false;
	}

	// Print standard metadata tags
	fprintf(file, "<?xml version=\"1.0\"?>\n");
	fprintf(file, "<?xml-model href=\"%s\" schematypens=\"%s\"?>\n",
		"http://download.ros.org/schema/package_format3.xsd",
		"http://www.w3.org/2001/XMLSchema");

	// Print package information
	fprintf(file, "<package format=\"%d\">\n", 3);
	fprintf(file, "<name>%s</name>\n", project_name);
	fprintf(file, "<version>%d.%d.%d</version>\n", 0, 0, 0);
	fprintf(file, "<description>TODO</description>\n");
	fprintf(file, "<maintainer email=\"john.doe@mail.com\">TODO"\
		"</maintainer>\n");
	fprintf(file, "<license>TODO</license>\n");
	fprintf(file, "<buildtool_depend>%s</buildtool_depend>\n",
		"ament_cmake");

	// Put all dependencies here
	for (ros_value_t **p = package_p->dependencies; *p != NULL; ++p) {
		fprintf(file, "<depend>%s</depend>\n", (*p)->data.data_string);
	}

	fprintf(file, "<test_depend>%s</test_depend>\n", "ament_lint_auto");
	fprintf(file, "<test_depend>%s</test_depend>\n", "ament_lint_common");

	fprintf(file, "<export>\n<build_type>%s</build_type>\n</export>\n",
		"ament_cmake");

	fprintf(file, "</package>\n");

	// Close the file
	fclose(file);

	return true;
}

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

	// Set include directories
	fprintf(file, "# Set include directories\n");
	fprintf(file, "include_directories(include)\n");

	// Write all dependencies
	fprintf(file, "# Find dependencies\n");
	for (ros_value_t **d = dependencies; *d != NULL; ++d) {
		fprintf(file, "find_package(%s REQUIRED)\n", (*d)->data.data_string);
	}
	fprintf(file, "\n");

	// Write all executors
	fprintf(file, "# Add executables and list their dependencies\n");
	for (ros_executor_t **e = executors; *e != NULL; ++e) {
		fprintf(file, "add_executable(executor_%ld src/executor_%ld.cpp "\
			"src/%s.cpp)\n", 
			(*e)->id->data.data_long, (*e)->id->data.data_long,
			ROS_CALLBACK_FILENAME);
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
