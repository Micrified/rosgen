#if !defined(ROS_SEMANTICS_H)
#define ROS_SEMANTICS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdarg.h>
#include <inttypes.h>

#include "ros_parse.h"


/*
 *******************************************************************************
 *                              Type Definitions                               *
 *******************************************************************************
*/


// Enumeration: Possible element types
typedef enum {
	TYPE_BOOL,
	TYPE_LONG,
	TYPE_STRING,

	TYPE_MAX
} ros_element_type_t;

// Structure: key-value intepretation
typedef struct {
	ros_element_type_t type;           // Data-type: found
	union {
		bool data_bool;                // Data-type: bool
		int64_t data_long;             // Data-type: long
		char *data_string;             // Data-type: string
	} data;
} ros_value_t;

// Structure: Describes a ROS callback
typedef struct {
	ros_value_t *name;                 // Callback identifier 
	ros_value_t *wcet;                 // Worst case execution time (ns)
	ros_value_t *prio;                 // Callback priority
	ros_value_t *timer_period;         // Period of the timer
	ros_value_t **topics_subscribed;   // Topics triggering callback
	ros_value_t **topics_publish;      // Topics published on after WCET
} ros_callback_t;

// Structure: Describes a ROS node
typedef struct {
	ros_value_t *name;                 // Name of the node
	ros_callback_t **callbacks;        // Null-terminated list of callbacks
} ros_node_t;

// Structure: Describes a ROS executor
typedef struct {
	ros_value_t *id;                   // Executor identifier
	ros_node_t **nodes;                // Null-terminated list of nodes
} ros_executor_t;

// Structure: Describing a ROS package configuration
typedef struct {
	ros_value_t *name;                 // Name of the ROS project/package
	ros_value_t *msg_type;             // Message type to use
	ros_value_t *msg_include;          // Include statement to use
	ros_value_t **dependencies;        // Dependencies for the CMakeLists
	ros_executor_t **executors;        // Null-terminated list of executors
} ros_package_t;


/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/


/*\
 * @brief Free's the memory associated with a given callback
 * @param callback_p Pointer to the callback to free
 * @return None
\*/
void free_callback (ros_callback_t *callback_p);

/*\
 * @brief Prints a callback to STDOUT
 * @note DEBUG function
 * @param callback_p Pointer to the callback
 * @return None
\*/
void show_callback (ros_callback_t *callback_p);

/*\
 * @brief Free's the memory associated with a given node
 * @param node_p Pointer to the node to free
 * @return None
\*/
void free_node (ros_node_t *node_p);

/*\
 * @brief Prints a node to STDOUT
 * @note DEBUG function
 * @param node_p Pointer to the callback
 * @return None
\*/
void show_node (ros_node_t *node_p);

/*\
 * @brief Free's the memory associated with a given executor
 * @param callback_p Pointer to the executor to free
 * @return None
\*/
void free_executor (ros_executor_t *executor_p);

/*\
 * @brief Prints an executor to STDOUT
 * @note DEBUG function
 * @param executor_p Pointer to the executor
 * @return None
\*/
void show_executor (ros_executor_t *executor_p);

/*\
 * @brief Free's the memory associated with a package
 * @param package_p Pointer to the package to free
 * @return None
\*/
void free_package (ros_package_t *package_p);

/*\
 * @brief Prints a package to STDOUT
 * @note DEBUG function
 * @param package_p Pointer to the package
 * @return None
\*/
void show_package (ros_package_t *package_p);

/*\
 * @brief Parses a callback from the given xml element. Performs semantic 
 *        checking
 * @param element The xml element to parse
 * @return NULL on error; else configured callback
\*/
ros_callback_t *parse_ros_callback (xml_element_t *element);

/*\
 * @brief Parses a node from the given xml element. Performs semantic
 *        checking
 * @param element The xml element to parse
 * @return NULL on error; else configured node
\*/
ros_node_t *parse_ros_node (xml_element_t *element);

/*\
 * @brief Parses an executor from the given xml element. Performs semantic
 *        checking
 * @param element The xml element to parse
 * @return NULL on error; else configured executor
\*/
ros_executor_t *parse_ros_executor (xml_element_t *element);

/*\
 * @brief Parses a package configuration from the given xml element.
 * @param element The xml element to parse
 * @return NULL on error; else configured package
\*/
ros_package_t *parse_ros_package (xml_element_t *element);

#endif