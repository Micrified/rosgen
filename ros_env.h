#if !defined(ROS_ENV_H)
#define ROS_ENV_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

#include "ros_semantics.h"


/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


// Maximum length of a file path
#define MAX_FILE_PATH_SIZE      512

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
 * @brief Creates a concatentation of strings as defined in the given format
 * @note This function is NON-REENTRANT as it relies on a single static buffer
 * @note The string created is limited to MAX_FILE_PATH_SIZE - 1 bytes
 * @param fmt The formatting string (only accepts %s token)
 * @param ... variadic arguments
\*/
const char *makeStaticPath (const char *fmt, ...);

/*\
 * @brief Generates the header callback file compiled with the project
 * @param path Path to directory in which the file should be generated
 * @param package_p Pointer to package containing project information
 * @return true if files successfully created; else false
\*/
bool generate_callback_header_file (const char *path, ros_package_t *package_p);

/*\
 * @brief Generates the source callback file compiled with the project
 * @param path Path to the directory in which the file should be generated
 * @param package_p Pointer to package containing project information
 * @return true if files successfully created; else false
\*/
bool generate_callback_source_file (const char *path, ros_package_t *package_p);

/*\
 * @brief Generates a package.xml file for the given project
 * @param path Path to directory in which the file should be generated
 * @param package_p Pointer to package containing project information
 * @return true if file successfully created; else false
\*/
bool generate_package_xml (const char *path, ros_package_t *package_p);

/*\
 * @brief Generates a CMakeLists.txt file for the given project, executables,
 *        and dependencies
 * @param path Path to directory in which the file should be generated
 * @param package_p Pointer to package containing project information
 * @return true if the file was successfully created; else false
\*/
bool generate_cmake (const char *path, ros_package_t *package_p);

/*\
 * @brief Generates the necessary directories for a ROS2 package
 * @param package_p Pointer to the package containing project information
 * @return true if the directories were successfully created; else false
\*/
bool generate_directories (ros_package_t *package_p);


#endif