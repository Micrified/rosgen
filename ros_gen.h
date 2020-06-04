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

/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


#define MAX_FILE_NAME_LENGTH    64

#define ROS_MSG_TYPE            "tutorial_interfaces::msg::Num"

/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/


/*\
 * @brief Writes a callback method to the given file
 * @note Assumes level three indentation for conetns
 * @param file The file stream to write to
 * @param callback_p The pointer to the callback to construct
 * @return true if could be generated; else false
\*/
bool generate_callback (FILE *file, ros_callback_t *callback_p);

/*\
 * @brief Writes a node class to the given file
 * @param file The file to write to
 * @param node_p The pointer to the node to construct
 * @return true if could be generated; else false
\*/
bool generate_node (FILE *file, ros_node_t *node_p);

/*\
 * @brief Creates a file and writes an executor to it
 * @note Files are named using the executor parameters
 * @param executor_p Pointer to executor to create file for
 * @return true if could be generated; else false
\*/
bool generate_executor (ros_executor_t *executor_p);


#endif