#if !defined(ROS_PARSE_H)
#define ROS_PARSE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdarg.h>

/*
 *******************************************************************************
 *                              Type Definitions                               *
 *******************************************************************************
*/

// Enumeration: Parse errors
typedef enum {
	PARSE_OK,
	PARSE_ERROR_BAD_PARAM,
	PARSE_ERROR_INVALID_FORMAT,
	PARSE_ERROR_BAD_TAG,
	PARSE_ERROR_FILE_STREAM,
	PARSE_ERROR_TAG_MISMATCH,
	PARSE_ERROR_NO_MEMORY,

	PARSE_ERROR_MAX
} parse_err_t;

// Enumeration: Element types
typedef enum {
	XML_STRING,
	XML_COLLECTION,
} xml_type_t;

// Structure: XML element
typedef struct xml_element_t {
	char *tag;
	xml_type_t type;
	union {
		struct xml_element_t **collection;
		char           *string;
	} data;
} xml_element_t;


/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/

/*\
 * @brief Parses a string containing numbers, letters, underscores, and dashes
 * @note  Strips leading whitespace
 * @note  Owner responsible for freeing string memory
 * @param file The file stream to read from
 * @param string_p Pointer to string at which string will be assigned
 * @return PARSE_OK on success; else error
\*/
parse_err_t parse_xml_string (FILE *file, char **string_p);

/*\
 * @brief Parses an XML opening or closing tag; saves tag in given pointer
 * @note  Owner responsible for freeing string memory
 * @param file The file stream to read from
 * @param open Set to true if expecting an opening tag; else closing one
 * @param tag_p Pointer to the string-pointer at which tag will be saved
 * @return PARSE_OK on success; else error
\*/
parse_err_t parse_xml_tag (FILE *file, bool open, char **tag_p);

/*\
 * @brief Parses an XML element, which can be a string or collection 
 *        of nested elements. 
 * @note  The pointer to the parsed element is saved at the given 
 *        pointer, and must be freed by the user using free_xml_element
 * @param file The file stream to read from
 * @param element_p Pointer to the element-pointer at which element
 *                  will be assigned
 * @return PARSE_OK on succcess; else error
\*/
parse_err_t parse_xml_element (FILE *file, xml_element_t **element_p);

/*\
 * @brief Prints an XML element.
 * @note  Supports only basic one level indentation
 * @param element The element to be displayed on stdout
 * @return PARSE_OK on success; else error
\*/
parse_err_t show_xml_element (xml_element_t *element);

/*\
 * @brief Frees memory allocated for an XML element.
 *        This includes the element itself
 * @note  Element pointer no longer valid after use
 * @param element Element to be freed
 * @return PARSE_OK on success; else error
\*/
parse_err_t free_xml_element (xml_element_t *element);

/*\
 * @brief Convert a parse error value to a string
 * @param err The error to index
 * @return Pointer to string describing error
\*/
const char *parse_err_str (parse_err_t err);

/*\
 * @brief Returns contextual description of an error, if any
 * @note String is internally limited
 * @return Pointer to string describing context
\*/
const char *get_err_context ();


#endif