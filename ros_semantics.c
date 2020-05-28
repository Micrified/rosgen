#include "ros_semantics.c"


/*
 *******************************************************************************
 *                              Global Variables                               *
 *******************************************************************************
*/


// Map: Element-type to string
const char *g_ros_element_type_str_tab[TYPE_MAX] = {
	[TYPE_BOOL]   = "bool",
	[TYPE_LONG]   = "long",
	[TYPE_STRING] = "string"
};


/*
 *******************************************************************************
 *                        Internal Function Definitions                        *
 *******************************************************************************
*/

/*\
 * @brief Frees memory associated with a ros value type
 * @param ros_value_p Pointer to the ros value structure
 * @return none
\*/
static void free_ros_value (ros_value_t *ros_value_p)
{
	if (ros_value_p->type == TYPE_STRING) {
		free(ros_value_p->data->string);
	}
	free(ros_value_p);
}


/*\
 * @brief Parses signed long value from given string
 * @note Assumes valid string pointer
 * @param string The signed string-encoded integer value
 * @param data_long_p Pointer to long at which value is saved
 * @return true if successfully parsed; else false
\*/
static bool acceptLong (const char *string, int64_t *data_long_p)
{
	int sign = 1;
	int64_t data_long = 0;
	off_t offset = 0;
	size_t length = strlen(string);

	// Accept an optional negative sign
	if (string[offset] == '-') {
		sign = -1;
		offset++;
	}

	// Must accept at least one digit
	if (offset >= length || !isdigit(string[offset])) {
		return false;
	} else {
		data_long = (int64_t)(string[offset] - '0');
		offset++;
	}

	// May accept zero or more digits following
	while (offset < length) {
		if (!isdigit(string[offset])) {
			return false;
		} else {
			data_long = data_long * 10 + (string[offset] - '0');
		}
		offset++;
	}

	// Save value if pointer valid
	if (*data_long_p != NULL) {
		*data_long_p = sign * data_long;
	}

	return true;
}

/*\
 * @brief Parses a boolean vaue from the given string
 * @note Assumes valid string pointer
 * @param string The string containing the boolean value
 * @param data_bool_p Pointer to the bool at which value is saved
 * @return true if successfully parsed; else false
\*/
static bool acceptBool (const char *string, bool *data_bool_p)
{
	bool data_bool = false;

	// Attempt to string match true
	if (strcmp("true", string) == 0) {
		data_bool = true;
		goto save;
	}

	// Attempt to string match false
	if (strcmp("false", string) == 0) {
		goto save;
	}

	// No match - error
	return false;

save:
	if (data_bool_p != NULL) {
		*data_bool_p = data_bool;
	}

	return true;
}

/*\
 * @brief Copies a string (basically)
 * @note Assumes valid string pointer
 * @note User is responsible for freeing the string
 * @param string The string to copy
 * @param string_p Pointer at which the string copy will be saved
 * @return true if successfully copied; else false
\*/
static bool acceptString (const char *string, char **string_p)
{
	size_t len = strlen(string);

	// Allocate copy
	void *string_copy = malloc((len + 1) * sizeof(char));

	// Check copy outcome
	if (string_copy == NULL) {
		return false;
	}

	// Copy 
	memcpy(string, string_copy, len * sizeof(char));

	// Affix null char
	string_copy[len] = '\0';

	return true;
}

/*\
 * @brief Returns a non-null element if it could be found within the collection
 * @param key The key to search for
 * @param collection A null terminated array of elements to search
 * @return Non-null element if found; else null
\*/
static xml_element_t *exists_element_with_key (char *key, xml_element_t **collection)
{
	xml_element_t *value = NULL;

	// Check parameters
	if (key == NULL || collection == NULL) {
		return NULL;
	}

	// Search for element with given key
	for (xml_element_t **p = collection; *p != NULL; ++p) {
		if (strcmp((*p)->tag, key) == 0) {
			value = (*p);
			break;
		}
	}

	return value;
}

/*\
 * @brief Attempts to locate and decode the given key-value from the collection element
 * @note  User is responsible for freeing allocated memory; including string copy in data
 * @param element    The element to search
 * @param key        The key to match on
 * @param type       The type to attempt to decode the value on
 * @param value_p    Pointer to the value where the result will be assigned
 * @param required   Whether this key is necessary; if so, an error is displayed on stderr
 * @return True if element found and value assigned; else false
\*/
bool element_has_key_value (xml_element_t *element, const char *key, ros_element_type_t type,
	ros_value_t **value_p, bool required)
{
	ros_value_t temp = {0};
	ros_value_t *value;
	xml_element_t *element = NULL;
	bool data_bool;
	int64_t data_long;
	char *data_string = NULL;

	// If the element isn't a collection, it is immediately invalid
	if (element->type != XML_COLLECTION) {
		fprintf(stderr, "Element \"%s\" is of wrong format. Expecting nested elements!\n",
			element->tag);
		return false;
	}

	// Check if the element exists
	if ((element = exists_element_with_key(key, element->data->collection)) == NULL) {
		if (required) {
			fprintf(stderr, "Element \"%s\" doesn't contain required element \"%s\"!\n",
				element->tag, key);			
		}
		return false;
	}

	// If the element exists, attempt to parse the expected type
	switch (type) {
		case TYPE_BOOL: {
			if (!acceptBool(element->data->string, &data_bool)) {
				goto bad_type;
			}
			temp.data.data_bool = data_bool;
		}
		break;
		case TYPE_LONG: {
			if (!acceptLong(element->data->string, &data_long)) {
				goto bad_type;
			}
			temp.data.data_long = data_long;
		}
		break;
		case TYPE_STRING: {
			if (!acceptString(element->data->string, &data_string)) {
				goto bad_type;
			}
			temp.data.data_string = string;
		}
		break;
	}

	// Assuming proper type found: allocate; configure; return
	if (value_p != NULL && (value = malloc(sizeof(ros_value_t))) != NULL) {
		value->type = type;
		value->data = temp.data;
		*value_p = value;
	} else {

	// Otherwise free string if allocated to avoid losing reference
		free(data_string);
	}

	// Accept match
	return true;

bad_type:

	// Attempt to free the string (should do nothing if null)
	free(data_string);

	// Inform cause of mismatch
	fprintf(stderr, "Element \"%s\" unable to parse \"%s\" as %s!\n",
		element->tag, element->data->string, g_ros_element_type_str_tab[type]);

	// Reject match
	return false;
}


/*
 *******************************************************************************
 *                        External Function Definitions                        *
 *******************************************************************************
*/


ros_callback_t *parse_ros_callback (xml_element_t *element)
{
	ros_callback_t callback = {0};
	ros_value_t *value = NULL;
	xml_element_t *set_element = NULL;

	// Check element name
	if (strcmp(element->tag, "callback") != 0) {
		fprintf(stderr, "Expected tag \"callback\", but got %s\n", 
			element->tag);
		return NULL;
	}

	// Make sure that it's a collection
	if (element->type != XML_COLLECTION) {
		fprintf(stderr, 
			"Expected \"callback\" to contain a collection, but got string\n");
		return NULL;
	}

	// Match name field
	if (!element_has_key_value(element->data->collection, "name", TYPE_STRING, 
		&value, true)) {
		return NULL;
	} else {
		callback.name = value;
	}

	// Match wcet field
	if (!element_has_key_value(element->data->collection, "wcet", TYPE_LONG, 
		&value, true)) {
		return NULL;
	} else {
		callback.wcet = value;
	}

	// Match prio field
	if (!element_has_key_value(element->data->collection, "prio", TYPE_LONG, 
		&value, true)) {
		return NULL;
	} else {
		callback.prio = value;
	}

	// Match timer field
	if (!element_has_key_value(element->data->collection, "timer", TYPE_LONG, 
		&value, false)) {
		return NULL;
	} else {
		callback.timer_period = value;
	}

	// Fetch subscribe topics
	if ((set_element = exists_element_with_key("topics_subscribed", 
		element->data->collection)) != NULL) {
		ros_value_t **topics_subscribed = NULL;
		xml_element_t **p;
		size_t size = 0;
		off_t offset = 0;

		// Verify it's a collection
		if (set_element->type != XML_COLLECTION) {
			fprintf(stderr, 
				"Expected topics_subscribed in \"callback\" to be a set!\n");
			goto discard;
		}

		// Compute size
		for (p = set_element->data->collection; (*p) != NULL; ++p, ++size)
			;

		// Allocate and perform check
		if ((topics_subscribed = malloc(size * sizeof(ros_value_t *))) 
			== NULL) {
			fprintf(stderr,
			"Unable to allocate enough memory for \"callback\" topic set!\n");
			goto discard;
		}

		// Convert all elements, expecting a string
		for (p = set_element->data->collection; (*p) != NULL; ++p) {

			// todo: perform copy

			topics_subscribed[offset] = 
		}

	}

	// Fetch publish topics

	// Discard
discard:
	free_ros_value(callback.name);
	free_ros_value(callback.wcet);
	free_ros_value(callback.prio);
	free_ros_value(callback.timer_period);
	if (callback.topics_subscribed != NULL) {
		for (ros_value_t **p = callback.topics_subscribed; (*p) != NULL; p++) {
			free_ros_value(*p);
		}
	}
	if (callback.topics_publish != NULL) {
		for (ros_value_t **p = callback.topics_publish; (*p) != NULL; p++) {
			free_ros_value(*p);
		}
	}

	return NULL;
}

ros_node_t *parse_ros_node (xml_element_t *element)
{

}

ros_executor_t *parse_ros_executor (xml_element_t *element)
{

}