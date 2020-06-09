#if !defined(ROS_ENV_H)
#define ROS_ENV_H

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


// File name for the callback
#define ROS_CALLBACK_FILENAME   "on_callback"

// The name of the callback 
#define ROS_CALLBACK_SIGNATURE  "on_callback"

/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/


/*\
 * @brief Generates the header callback file compiled with the project
 * @param package_p Pointer to package containing project information
 * @return true if files successfully created; else false
\*/
bool generate_callback_header_file (ros_package_t *package_p);

/*\
 * @brief Generates the source callback file compiled with the project
 * @param package_p Pointer to package containing project information
 * @return true if files successfully created; else false
\*/
bool generate_callback_source_file (ros_package_t *package_p);

/*\
 * @brief Generates a package.xml file for the given project
 * @param package_p Pointer to package containing project information
 * @return true if file successfully created; else false
\*/
bool generate_package_xml (ros_package_t *package_p);


/*\
 * @brief Generates a CMakeLists.txt file for the given project, executables,
 *        and dependencies
 * @param package_p Pointer to package containing project information
 * @return true if the file was successfully created; else false
\*/
bool generate_cmake (ros_package_t *package_p);


#endif