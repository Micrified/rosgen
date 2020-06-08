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
 *                            Function Declarations                            *
 *******************************************************************************
*/


/*\
 * @brief Generates a CMakeLists.txt file for the given project, executables,
 *        and dependencies
 * @param package_p The package to generate
 * @return true if the file was successfully created; else false
\*/
bool generate_cmake (ros_package_t *package_p);


#endif