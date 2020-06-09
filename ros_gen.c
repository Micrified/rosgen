#include "ros_gen.h"

/*
 *******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************
*/

// Table: C++ libraries and headers to include
static const char *g_include_cpp_statements[] = {
	"<chrono>",
	"<memory>",
	"\"rclcpp/rclcpp.hpp\"",
};

// Table: C-style libraries and headers to include
static const char *g_include_c_statements[] = {
	"<time.h>"
};

// Table: C++ style using macros to use
static const char *g_using_statements[] = {
	"std::placeholders::_1"
};


/*
 *******************************************************************************
 *                        External Function Definitions                        *
 *******************************************************************************
*/

bool generate_callback (FILE *file, ros_callback_t *callback_p,
	ros_package_t *package_p)
{
	// The message type pointer
	const char *msg_type = package_p->msg_type->data.data_string;

	// Verify pointer
	if (file == NULL || callback_p == NULL) {
		return false;
	}

	// Extract callback fields
	const char *name = callback_p->name->data.data_string;
	int64_t wcet = callback_p->wcet->data.data_long;
	int64_t prio = callback_p->prio->data.data_long;
	int64_t timer_period = (callback_p->timer_period == NULL ? 
		-1 : (callback_p->timer_period->data.data_long));

	// Write method definition (only include arguments if it is a timer)
	if (timer_period != -1) {
		fprintf(file, "\t\tvoid %s () {\n", name);
	} else {
		fprintf(file, "\t\tvoid %s (const %s::SharedPtr msg_recv) const {\n",
			name, msg_type);
		fprintf(file, "\t\t\t(void)msg_recv;\n");
	}

	// Apply the WCET delay
	int64_t seconds = wcet / (int64_t)1E9, nanoseconds = wcet % (int64_t)1E9;
	fprintf(file, "\t\t\tstruct timespec delay = {%ld, %ld};\n",
		seconds, nanoseconds);
	fprintf(file, "\t\t\tnanosleep(&delay, NULL);\n\n");

	// Call the custom function
	fprintf(file, 
		"\t\t\tauto msg_send = %s(%s, %s, %s, \"%s\", %ld, %ld, %s);\n",
		ROS_CALLBACK_SIGNATURE,
		(timer_period == -1 ? "msg_recv" : "NULL"), 
		"this->data_executor_id",
		"this->data_name", name, prio, wcet, 
		(timer_period == -1 ? "false" : "true"));

	// Return now if not publishing since no need to make message
	if (callback_p->topics_publish == NULL || 
		callback_p->topics_publish[0] == NULL) {
		goto end;
	}

	// Publish to publishers
	for (ros_value_t **t = callback_p->topics_publish; *t != NULL; ++t) {
		const char *topic = (*t)->data.data_string;
		fprintf(file, "\t\t\tif (msg_send != NULL) {\n");
		fprintf(file, "\t\t\t\tpub_%s_%s->publish(*msg_send);\n",
			name, topic);
		fprintf(file, "\t\t\t\tdelete msg_send;\n");
		fprintf(file, "\t\t\t}\n");
	}
end:
	// End method
	fprintf(file, "\t\t}\n");

	return true;
}

bool generate_node (FILE *file, ros_node_t *node_p,
	ros_package_t *package_p, int64_t executor_id)
{
	ros_callback_t **c = NULL, **callbacks = node_p->callbacks;
	ros_value_t **s = NULL, **p = NULL;
	const char *name = node_p->name->data.data_string;

	// The message type pointer
	const char *msg_type = package_p->msg_type->data.data_string;

	// Check arguments 
	if (file == NULL || node_p == NULL) {
		return false;
	}

	// Construct class declaration for node
	fprintf(file, "class %s : public rclcpp::Node\n", name);
	fprintf(file, "{\n");
	fprintf(file, "private:\n");

	// Generate default fields (name + executor ID)
	fprintf(file, "\t\tchar const *data_name = \"%s\";\n", name);
	fprintf(file, "\t\tint64_t data_executor_id = %ld;\n", 
		executor_id);

	// For each callback generate the subscription variable
	for (c = callbacks; *c != NULL; ++c) {
		ros_value_t **subscriptions = (*c)->topics_subscribed;
		for (s = subscriptions; *s != NULL; ++s) {
			fprintf(file, 
				"\t\trclcpp::Subscription<%s>::SharedPtr sub_%s_%s;\n",
				msg_type,
				(*c)->name->data.data_string, 
				(*s)->data.data_string);
		}
	}

	// For each callback generate the publisher variable
	for (c = callbacks; *c != NULL; ++c) {
		ros_value_t **publish = (*c)->topics_publish;
		for (p = publish; *p != NULL; ++p) {
			fprintf(file, "\t\trclcpp::Publisher<%s>::SharedPtr pub_%s_%s;\n",
				msg_type, 
				(*c)->name->data.data_string, 
				(*p)->data.data_string);
		}
	}

	// For each callback that is a timer generate the timer variable
	for (c = callbacks; *c != NULL; ++c) {

		// Skip elements without a timer field
		if ((*c)->timer_period == NULL) {
			continue;
		}

		// Generate the timer variable
		fprintf(file, "\t\trclcpp::TimerBase::SharedPtr timer_%s;\n",
			(*c)->name->data.data_string);
	}

	// Create all the callbacks
	for (ros_callback_t **c = callbacks; *c != NULL; ++c) {
		if (generate_callback(file, *c, package_p) == false) {
			fprintf(stderr, "Unable to generate callback!\n");
			return false;
		}
		fprintf(file, "\n");
	}

	// Declare public section
	fprintf(file, "public:\n");

	// Create the constructor
	fprintf(file, "\t%s(): Node(\"%s\") {\n",
		name, name);

	// Initialize all subscriptions
	for (c = callbacks; *c != NULL; ++c) {
		ros_value_t **subscriptions = (*c)->topics_subscribed;
		for (s = subscriptions; *s != NULL; ++s) {
			fprintf(file, "\t\tsub_%s_%s = this->create_subscription<%s>(\"%s\", 10,\n\
				std::bind(&%s::%s, this, _1));\n",
				(*c)->name->data.data_string, 
				(*s)->data.data_string,
				msg_type,
				(*s)->data.data_string,
				name,
				(*c)->name->data.data_string); 
		}
	}

	// Initialize all publishers
	for (c = callbacks; *c != NULL; ++c) {
		ros_value_t **publish = (*c)->topics_publish;
		for (p = publish; *p != NULL; ++p) {
			fprintf(file, "\t\tpub_%s_%s = this->create_publisher<%s>(\"%s\", 10);\n",
				(*c)->name->data.data_string, 
				(*p)->data.data_string,
				msg_type,
				(*p)->data.data_string); 
		}
	}

	// Initialize all timers
	for (ros_callback_t **c = callbacks; *c != NULL; ++c) {

		// Skip callbacks without timers
		if ((*c)->timer_period == NULL) {
			continue;
		}

		// Get timer period
		int64_t period_nsec = (*c)->timer_period->data.data_long;

		// If it has a timer, initialize it
		fprintf(file, "\t\ttimer_%s = this->create_wall_timer(" \
			"std::chrono::nanoseconds(%ld), std::bind(&%s::%s, this));\n",
			(*c)->name->data.data_string,
			period_nsec,
			name, 
			(*c)->name->data.data_string);
	}

	// End of constructor + separator
	fprintf(file, "\t}\n\n");

	// End of class
	fprintf(file, "};\n");

	return true;
}


bool generate_executor (ros_executor_t *executor_p, 
	ros_package_t *package_p)
{
	FILE *file = NULL;
	char file_name[MAX_FILE_NAME_LENGTH];
	size_t lines = 0;
	ros_node_t *node = NULL, **nodes = NULL;
	int64_t executor_id = -1;

	// Extract pointer to include statement for msg_type
	const char *msg_include = package_p->msg_include->data.data_string; 

	// Check parameters
	if (executor_p == NULL) {
		fprintf(stderr, "Cannot create executor from NULL pointer!\n");
		return false;
	}

	// Extract executor ID
	executor_id = executor_p->id->data.data_long;

	// Create the file name
	if (snprintf(file_name, MAX_FILE_NAME_LENGTH, "executor_%ld.cpp", 
		executor_id) >= MAX_FILE_NAME_LENGTH) {
		fprintf(stderr, 
			"Unable to create executor filename! Exceeds buffer capacity!\n");
		return false;
	}

	// Create new file
	if ((file = fopen(file_name, "w")) == NULL) {
		fprintf(stderr, "Unable to create writable file/stream \"%s\"!\n", 
			file_name);
		return false;
	}

	// Put standard include statements for C++
	lines = sizeof(g_include_cpp_statements) / sizeof(char *);
	for (off_t i = 0; i < lines; ++i) {
		fprintf(file, "#include %s\n", g_include_cpp_statements[i]);
	}

	// Put down the include statement for the message (assumes C++)
	fprintf(file, "#include \"%s\"\n", msg_include);

	// Put down the include statement for the callback file
	fprintf(file, "#include \"%s/%s.hpp\"\n", 
		package_p->name->data.data_string, ROS_CALLBACK_FILENAME);

	// Put include statements for C
	fprintf(file, "\nextern \"C\" {\n");
	lines = sizeof(g_include_c_statements) / sizeof(char *);
	for (off_t i = 0; i < lines; ++i) {
		fprintf(file, "\t#include %s\n", g_include_c_statements[i]);
	}
	fprintf(file, "}\n");

	// Put using statements
	lines = sizeof(g_using_statements) / sizeof(char *);
	for (off_t i = 0; i < lines; ++i) {
		fprintf(file, "using %s;\n", g_using_statements[i]);
	}

	// Print the nodes
	for (ros_node_t **n = executor_p->nodes; *n != NULL; ++n) {
		fprintf(file, "\n");
		if (generate_node(file, *n, package_p, executor_id) == false) {
			fprintf(stderr, "Unable to generate node \"%s\" - aborting!\n",
				(*n)->name->data.data_string);
			break;
		}
	}

	// Print the main statement and opening brace
	fprintf(file, "\n\nint main (int argc, char *argv[])\n");
	fprintf(file, "{\n");

	// Print initialization statements (only first node is used)
	fprintf(file, "\trclcpp::init(argc, argv);\n");

	// Initialize all nodes
	for (nodes = executor_p->nodes; *nodes != NULL; ++nodes) {
		node = *nodes;

		// Create local variable
		fprintf(file, "\tauto node_handle_%s = std::make_shared<%s>();\n",
			node->name->data.data_string, node->name->data.data_string);
	}

	// Spacing
	fprintf(file, "\n");

	// Declare the executor
	fprintf(file, "\trclcpp::executors::SingleThreadedExecutor executor_%ld;\n",
		executor_p->id->data.data_long);

	// Add nodes to the executor
	for (nodes = executor_p->nodes; *nodes != NULL; ++nodes) {
		node = *nodes;

		// Create the method invocation
		fprintf(file, "\texecutor_%ld.add_node(node_handle_%s);\n", 
			executor_p->id->data.data_long, node->name->data.data_string);
	}

	// Spin the executor
	fprintf(file, "\texecutor_%ld.spin();\n", executor_p->id->data.data_long);

	// Print shutdown API call
	fprintf(file, "\trclcpp::shutdown();\n");

	// Print closing brace
	fprintf(file, "}\n");

	// Close the file
	fclose(file); 

	return true;
}

bool generate_package (ros_package_t *package_p)
{

	// Check parameters
	if (package_p == NULL) {
		fprintf(stderr, "Cannot create package from NULL pointer!\n");
		return false;
	}

	// Generate all executor files
	for (ros_executor_t **p = package_p->executors; *p != NULL; ++p) {
		if (generate_executor(*p, package_p) == false) {
			fprintf(stderr, "Unable to generate an executor!\n");
			return false;
		}
	}

	// Generate the makefile
	if (generate_cmake(package_p) == false) {
		fprintf(stderr, "Unable to generate the CMakeLists.txt file!\n");
	}

	// Generate the source and header file
	if (generate_callback_header_file(package_p) == false) {
		fprintf(stderr, "Unable to generate callback header file!\n");
	}

	// Generate the source and header file
	if (generate_callback_source_file(package_p) == false) {
		fprintf(stderr, "Unable to generate callback source file!\n");
	}

	// Generate the package description 
	if (generate_package_xml(package_p) == false) {
		fprintf(stderr, "Unable to generate package.xml file!\n");
	}

	return true;
}