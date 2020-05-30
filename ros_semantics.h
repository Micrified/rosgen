#if !defined(ROS_SEMANTICS_H)
#define ROS_SEMANTICS_H

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
	char *name;                        // Name of the node
	ros_callback_t **callbacks;        // Null-terminated list of callbacks
} ros_node_t;

// Structure: Describes a ROS executor
typedef struct {
	int64_t id;                        // Executor identifier
	ros_node_t **nodes;                // Null-terminated list of nodes
} ros_executor_t;



/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/

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


#endif