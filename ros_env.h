#if !defined(ROS_ENV_H)
#define ROS_ENV_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>


/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/


/*\
 * @brief Generates a CMakeLists.txt file for the given project, executables,
 *        and dependencies
 * @param project_name Name of the project (should be same used to generate 
 *                     the ROS package). List should be NULL terminated
 * @param executables Name of all the executables that will be created
 *                    (i.e. the executors that will become executables)
 *                    List should be NULL terminated
 * @param dependencies The packages that this project depends on
 * @return true if the file was successfully created; else false
\*/
bool generate_cmake (const char *project_name, const char **executables, 
	const char **dependencies);


#endif