#include "ros_parse.h"

/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


// Maximum length of error description string in bytes
#define MAX_ERROR_DESCRIPTION_SIZE        64


/*
 *******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************
*/

// Global line number
int g_lineno = 1;

// String: Global error buffer
char g_error_description[MAX_ERROR_DESCRIPTION_SIZE];

// Table: Mapping of parse codes to strings
const char const *g_parse_str_tab[PARSE_ERROR_MAX] = 
{
	[PARSE_OK] = "Ok",
	[PARSE_ERROR_BAD_PARAM] = "Invalid parameters",
	[PARSE_ERROR_INVALID_FORMAT] = "Invalid XML format",
	[PARSE_ERROR_BAD_START_TAG] = "Invalid start tag format",
	[PARSE_ERROR_UNESCAPED_STRING] = "Unescaped string",
	[PARSE_ERROR_BAD_END_TAG] = "Invalid end tag format",
	[PARSE_ERROR_FILE_STREAM] = "File stream manipulation error",
	[PARSE_ERROR_TAG_MISMATCH] = "Mismatching open/close tags",
	[PARSE_ERROR_NO_MEMORY] = "Unable to allocate memory"
};


/*
 *******************************************************************************
 *                        Internal Function Definitions                        *
 *******************************************************************************
*/


int _fgetc (FILE *stream)
{
	int c = fgetc(stream);
	if (c == '\n') {
		g_lineno++;
	}
	return c;
}

static void set_err (const char *fmt, ...)
{
	static bool error_set = false;

	// If an error was already set; don't allow it to be overwritten
	if (error_set == true) {
		return;
	} else {
		error_set = true;
	}

	// Init variadic list for variadic arguments
   	va_list ap;
    va_start(ap, fmt);
    snprintf(g_error_description, MAX_ERROR_DESCRIPTION_SIZE, 
    	"line %d. ", g_lineno);
    off_t offset = strlen(g_error_description);
    vsnprintf(g_error_description + offset, MAX_ERROR_DESCRIPTION_SIZE - offset, 
    	fmt, ap);
    va_end(ap);
}


static void free_xml_param (xml_element_parameter_t *param)
{
	if (param == NULL) {
		return;
	}
	free(param->key);
	free(param->value);
	free(param);
}


/*
 *******************************************************************************
 *                            Function Definitions                             *
 *******************************************************************************
*/


parse_err_t parse_xml_string (FILE *file, char **string_p)
{
	int c;
	parse_err_t err = PARSE_OK;
	size_t size = 8; off_t offset = 0;
	char *string = NULL;

	// Drop leading whitespace
	while (isspace((c = _fgetc(file))));

	// Accept alphanumerics and underscores and dashes
	while (isalnum(c) || c == '_' || c == '-' || c == ':') {
		if (string == NULL) {
			string = malloc(size * sizeof(char));
		}
		if (offset >= (size - 1)) {
			size *= 2;
			string = realloc(string, size);
		}
		string[offset++] = c;
		c = _fgetc(file);
	}

	// If nothing was in the tag - return error
	if (offset == 0) {
		err = PARSE_ERROR_INVALID_FORMAT;
		goto unget;
	}

	// Null-terminate the string
	string[offset] = '\0';

	// Assign pointer (if not then free immediately)
	if (string_p == NULL) {
		free(string);
	} else {
		*string_p = string;
	}

unget:
	// Put back last character
	if (ungetc(c, file) != c) {
		return PARSE_ERROR_FILE_STREAM;
	}

	return err;
}

parse_err_t parse_xml_value (FILE *file, char **string_p)
{
	int c;
	off_t offset = 0;
	char *string = NULL;
	size_t size = 8;

	if ((c = _fgetc(file)) != '"') {
		// Put back last character
		if (ungetc(c, file) != c) {
			return PARSE_ERROR_FILE_STREAM;
		}
		return PARSE_ERROR_INVALID_FORMAT;
	}

	while ((c = _fgetc(file)) != '"' && c != EOF) {
		if (string == NULL) {
			string = malloc(size * sizeof(char));
		}
		if (offset >= (size - 1)) {
			size *= 2;
			string = realloc(string, size * sizeof(char));
		}
		string[offset++] = c;
	}

	string[offset] = '\0';

	if (c == EOF) {
		free(string);
		set_err("String not closed");
		return PARSE_ERROR_UNESCAPED_STRING;
	}

	if (string_p != NULL) {
		*string_p = string;
	} else {
		free(string);
	}

	return PARSE_OK;
}

parse_err_t parse_xml_param (FILE *file, xml_element_parameter_t **param_p)
{
	char *key = NULL, *value = NULL;
	parse_err_t err;
	int c;

	// Drop leading whitespace
	while (isspace((c = _fgetc(file))));

	// If the end bracket was reached, put it back and return invalid format
	if (c == '>') {
		// Put back last character
		if (ungetc(c, file) != c) {
			return PARSE_ERROR_FILE_STREAM;
		}
		return PARSE_ERROR_INVALID_FORMAT;
	} else {

		// Put it back anyways so it can be parsed into a string
		if (ungetc(c, file) != c) {
			return PARSE_ERROR_FILE_STREAM;
		}
	}

	// Otherwise try to accept the key (string)
	if ((err = parse_xml_string(file, &key)) != PARSE_OK) {
		free(key);
		return err;
	}

	// Expect an equals sign
	if ((c = _fgetc(file)) != '=') {
		set_err("Missing assignment operator for %s", key);
		free(key);
		return PARSE_ERROR_BAD_PARAM;
	}

	// Expect a value
	if ((err = parse_xml_value(file, &value)) != PARSE_OK) {
		free(key);
		free(value);
		return PARSE_ERROR_BAD_PARAM;
	}

	// Return early if no assignment needed
	if (param_p == NULL) {
		free(key);
		free(value);
		return PARSE_OK;
	}

	// Allocate and assign parameter
	xml_element_parameter_t *param = malloc(sizeof(xml_element_parameter_t));
	param->key = key;
	param->value = value;
	*param_p = param;

	return PARSE_OK;
}

parse_err_t parse_xml_tag (FILE *file, bool open, char **tag_p, 
	xml_element_parameter_t **param_p)
{
	int c;
	parse_err_t err = PARSE_OK;
	char *tag = NULL;
	off_t offset = 0;
	xml_element_parameter_t *param = NULL;

	// Drop leading whitespace
	while (isspace((c = _fgetc(file))));

	// Expect open tag bracket
	if (c != '<') {
		set_err("Expected '<', but got %c", c);
		// Put back last character
		if (ungetc(c, file) != c) {
			return PARSE_ERROR_FILE_STREAM;
		}
		return PARSE_ERROR_BAD_START_TAG;
	}

	// Grab the next character
	if ((c = _fgetc(file)) == '/') {
		if (open == true) {
			if (fseek(file, -2L, SEEK_CUR) == -1) {
				return PARSE_ERROR_FILE_STREAM;
			}
			return PARSE_ERROR_BAD_START_TAG;			
		}
	} else {
		if (ungetc(c, file) != c) {
			return PARSE_ERROR_FILE_STREAM;
		}
		if (open == false) {
			return PARSE_ERROR_BAD_END_TAG;
		}
	}

	// Expect tag identifier
	if ((err = parse_xml_string(file, &tag)) != PARSE_OK) {
		set_err("Tag has no label");
		free(tag);
		return err;
	}

	// Expect a parameter (only if an opening tag)
	if (open == true && (err = parse_xml_param(file, &param)) != PARSE_OK) {
		if (err != PARSE_ERROR_INVALID_FORMAT) {
			set_err("Invalid parameter in %s", tag);
			free(tag);
			free_xml_param(param);
			return err;
		}
	}

	// Drop trailing whitespace
	while (isspace((c = _fgetc(file))));

	// Expect closing tag bracket
	if (c != '>') {
		free(tag);
		free_xml_param(param);
		set_err("Tag missing closing bracket");
		return PARSE_ERROR_UNESCAPED_STRING;
	}

	// If a pointer was suppied, save string; else free
	if (tag_p != NULL) {
		*tag_p = tag;
	} else {
		free(tag);
		tag = NULL;
	}

	// Assign param
	if (open == true) {
		if (param_p != NULL && param != NULL) {
			*param_p = param;
		}
		if (param_p == NULL && param != NULL) {
			free_xml_param(param);
		}
	}

	return PARSE_OK;
}

parse_err_t parse_xml_element (FILE *file, xml_element_t **element_p)
{
	xml_type_t type;
	parse_err_t err = PARSE_OK;
	char *tag_open = NULL, *string = NULL, *tag_close = NULL;
	xml_element_t **collection = NULL;
	xml_element_t *next_elem_p = NULL;
	xml_element_t *element = NULL;
	xml_element_parameter_t *param = NULL;
	off_t i = 0;

	// Accept opening tag
	if ((err = parse_xml_tag(file, true, &tag_open, &param)) != PARSE_OK) {
		goto discard;
	}

	// Try: accept string element
	if (parse_xml_string(file, &string) == PARSE_OK) {
		type = XML_STRING;
		goto closing_tag;
	} else {
		free(string);
	}



	// Otherwise: Initialize a collection
	type = XML_COLLECTION;
	size_t collection_size = 2;
	collection = malloc(collection_size * sizeof(xml_element_t *));

	// Accept elements
	while (1) {

		// Buffer check
		if (i >= (collection_size - 1)) {
			collection_size *= 2;
			collection = realloc(collection, 
				collection_size * sizeof(xml_element_t *));
		}

		// Parse next element 
		if ((err = parse_xml_element(file, &next_elem_p)) != PARSE_OK) {
			break;
		} else {
			collection[i] = next_elem_p;
		}

		// Increment
		i++;
	}

	// If the error wasn't related to a bad tag - report it
	if (err != PARSE_ERROR_BAD_START_TAG) {
		goto discard;
	}

	collection[i] = NULL;

closing_tag:

	// Accept closing tag
	if ((err = parse_xml_tag(file, false, &tag_close, NULL)) != PARSE_OK) {
		set_err("No closing tag for %s", tag_open);
		goto discard;
	}

	// Verify tags match
	if (strcmp(tag_open, tag_close) != 0) {
		set_err("Tag mismatch (<%s> -> <%s>)",
			tag_open, tag_close);
		err = PARSE_ERROR_TAG_MISMATCH;
		goto discard;
	}

	// Discard element if it won't be saved
	if (element_p == NULL) {
		goto discard;
	}

	// Otherwise allocate and assign element
	if ((element = malloc(sizeof(xml_element_t))) == NULL) {
		err = PARSE_ERROR_NO_MEMORY;
		set_err("No memory!\n");
		goto discard;
	} 

	// Assign the fields
	element->tag = tag_open;
	element->param = param;
	element->type = type;
	if (type == XML_STRING) {
		element->data.string = string;
	} else {
		element->data.collection = collection;
	}

	// Assign pointer
	*element_p = element;

	// Discard the closing tag, since we only keep the opening one
	free(tag_close);

	return PARSE_OK;

discard:
	free(tag_open); 
	free(string); 
	free(tag_close);
	if (collection != NULL) {
		for (xml_element_t **p = collection; *p != NULL; ++p) {
			free_xml_element(*p);
		}
		free(collection);
	}
	free(element);
	free_xml_param(param);

	return err;
}

parse_err_t show_xml_element (xml_element_t *element)
{
	off_t i = 0;

	// Parameter check
	if (element == NULL) {
		return PARSE_ERROR_BAD_PARAM;
	}

	// Print opening tag
	printf("<%s", element->tag);
	if (element->param != NULL) {
		printf(" %s=\"%s\"", element->param->key, element->param->value);
	}
	printf(">\n");

	// Print contents
	switch (element->type) {
		case XML_STRING: {
			printf("\t%s\n", element->data.string);
		}
		break;
		case XML_COLLECTION: {
			for (i = 0; element->data.collection[i] != NULL; ++i)
			{
				show_xml_element(element->data.collection[i]);
			}
		}
		break;
		default: {
			return PARSE_ERROR_INVALID_FORMAT;
		}
	}

	// Print closing tag
	printf("</%s>\n", element->tag);

	return PARSE_OK;
}

parse_err_t free_xml_element (xml_element_t *element)
{
	xml_element_t **p = NULL;

	// Verify parameter
	if (element == NULL) {
		return PARSE_ERROR_BAD_PARAM;
	}

	// Free the tag
	free(element->tag);

	// Free parameter
	free_xml_param(element->param);

	// Free the contents
	switch (element->type) {
		case XML_STRING: {
			free(element->data.string);
		}
		break;
		case XML_COLLECTION: {
			for (p = element->data.collection; *p != NULL; ++p) {
				free_xml_element(*p);
			}
			free(element->data.collection);
		}
		break;
		default: {
			return PARSE_ERROR_INVALID_FORMAT;
		}
	}

	// Free element itself
	free(element);

	return PARSE_OK;
}

const char *parse_err_str (parse_err_t err) 
{
	if (err > PARSE_ERROR_MAX) {
		return "Invalid error code";
	}
	return g_parse_str_tab[err];
}

const char *get_err_context ()
{
	return g_error_description;
}
