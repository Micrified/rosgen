#if !defined(ROS_GEN_H)
#define ROS_GEN_H


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>

#include "ros_semantics.h"
#include "ros_env.h"

/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


// Maximum permitted file name length
#define MAX_FILE_NAME_LENGTH    64


/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/


/*\
 * @brief Writes a callback method to the given file
 * @note Assumes level three indentation for content
 * @param file The file stream to write to
 * @param callback_p The pointer to the callback to construct
 * @param package_p Pointer to package with metadata
 * @return true if could be generated; else false
\*/
bool generate_callback (FILE *file, ros_callback_t *callback_p,
	ros_package_t *package_p);

/*\
 * @brief Writes a node class to the given file
 * @param file The file to write to
 * @param node_p The pointer to the node to construct
 * @param package_p Pointer to package with metadata
 * @param executor_id Id of the parent executor
 * @return true if could be generated; else false
\*/
bool generate_node (FILE *file, ros_node_t *node_p,
	ros_package_t *package_p, int64_t executor_id);

/*\
 * @brief Creates a file and writes an executor to it
 * @note Files are named using the executor parameters
 * @param path Directory in which generated files will be put
 * @param executor_p Pointer to executor to create file for
 * @param package_p Pointer to package with metadata
 * @return true if could be generated; else false
\*/
bool generate_executor (const char *path, 
	ros_executor_t *executor_p, 
	ros_package_t *package_p);


/*\
 * @brief Creates a ROS2 package
 * @note Executors are converted to source files
 * @param package_p Pointer to package to create project for
 * @return true if could be generated; else false
\*/
bool generate_package (ros_package_t *package_p);


#endif