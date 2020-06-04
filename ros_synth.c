#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>

#include "ros_parse.h"
#include "ros_semantics.h"
#include "ros_gen.h"


/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


// Format String: Terminal usage
#define FMT_USAGE     "%s [<filename>]"

// Format String: Error template
#define FMT_ERR       "Err (%s:%d): "


/*
 *******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************
*/


// Table: Mapping of exit codes to strings
static const char const *g_error_str_tab[0xFF] = 
{
	[0] = "Ok",
	[1] = "Invalid program parameters",
	[2] = "Unable to open supplied file",
	[3] = "Parse error",
	[4] = "Configuration conflict",
	[5] = "Unable to close supplied file"
};


/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/


// Main program
int main (int argc, char *argv[])
{
	bool is_file = false;
	int err;
	FILE *file = NULL;
	parse_err_t parse_err;
	xml_element_t *element_p;

	// Arg check
	if (argc > 2) {
		fprintf(stderr, FMT_USAGE "\n", argv[0]);
		err = 1;
		goto end;
	} else {
		is_file = (argc == 2);
	}

	// Input configuration
	if (is_file) {
		if ((file = fopen(argv[1], "r")) == NULL) {
			fprintf(stderr, FMT_ERR "%s\n", __FILE__, __LINE__, strerror(errno));
			err = 2;
			goto end;
		}
	} else {
		file = stdin;
	}

	// Parse a single element from the file stream
	while ((parse_err = parse_xml_element(file, &element_p)) == PARSE_OK) {
		show_xml_element(element_p);
		ros_callback_t *callback_p = NULL;
		ros_node_t *node_p = NULL;
		ros_executor_t *executor_p = NULL;

		// Process the element (expecting only a callback for now)
		if ((executor_p = parse_ros_executor(element_p)) == NULL)
		{
			fprintf(stderr, "Err: Unable to parse the executor!\n");
		} else {
			show_executor(executor_p);
			if (!generate_executor(executor_p)) {
				fprintf(stderr, "Err: Unable to generate executor!\n");
			}
			free_executor(executor_p);
		}
		free_xml_element(element_p);
		element_p = NULL;
	}

	if (parse_err != PARSE_ERROR_BAD_START_TAG) {
		fprintf(stderr, FMT_ERR "%s: %s\n", __FILE__, __LINE__, parse_err_str(parse_err),
			get_err_context());
		err = 3;
		goto end;
	} else {
		fprintf(stdout, "At the end of the file!\n");
	}

	// Close the file stream
	if ((err = fclose(file)) != 0) {
		fprintf(stderr, FMT_ERR "%s\n", __FILE__, __LINE__, strerror(errno));
		err = 5;
		goto end;
	}

end:
	if (err != 0) {
		fprintf(stderr, "Exit Code %d: %s\n", err, g_error_str_tab[err]);
	}
	return err;
}