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

ros_value_t **parse_strings_from_xml_collection (xml_element_t **collection)
{
	ros_value_t **v, **value_set = NULL;
	xml_element_t **p, *element = NULL;
	size_t size = 0;
	off_t offset = 0;

	// Compute the size of the collection
	for (p = collection; *p != NULL; ++p) {
		size++;
	}

	// Allocate the corresponding set of strings
	if ((value_set = malloc((size + 1) * sizeof(ros_value_t *))) == NULL) {
		fprintf(stderr, 
			"%s:%d: Unable to allocate enough memory for collection!\n",
			__FILE__, __LINE__);
		goto discard;
	}

	// Make last element NULL
	value_set[size] = NULL;

	// Attempt to convert all elements, expecting a string type
	for (off_t i = 0, p = collection; *p != NULL; ++p, ++i) {
		ros_value_t *value = NULL;
		char *string = NULL;

		// Check name; assume topic
		if (strcmp(p->tag, "topic") != 0) {
			fprintf(stderr, "Expected element \"%s\" but \
				found \"%s\" instead!\n", "topic", p->tag);
			goto discard;
		}

		// Check type; expect string
		if (p->type != XML_STRING) {
			fprintf(stderr, "Exists element \"%s\" not of type string!\n");
			goto discard;
		}

		// Allocate a new value
		if ((value = malloc(sizeof(ros_value_t))) == NULL) {
			fprintf(stderr, "%s:%d: Unable to allocate enough memory for \
				value!\n", __FILE__, __LINE__);
			goto discard;
		}

		// Allocate a copy of the string 
		if (!acceptString(p->data.string, &string)) {
			fprintf(stderr, "%s:%d: Unable to allocate string copy!\n",
				__FILE__, __LINE__);
			goto discard;
		}

		// Configure the value
		value->type = TYPE_STRING;
		value->data->string = string;

		// Assign to array
		value_set[i] = value;
	}


	return value_set;

discard:

	// Destroy set if created
	if (value_set != NULL) {
		for (v = value_set; *v != NULL; ++v) {
			free_ros_value(*v);
		}
		free(value_set);
	}

	return NULL;
}

/*
 *******************************************************************************
 *                        External Function Definitions                        *
 *******************************************************************************
*/

void free_callback (ros_callback_t *callback_p)
{

	// Ignore NULL pointer
	if (callback_p == NULL) {
		return;
	}

	// Free fields
	free_ros_value(name);
	free_ros_value(wcet);
	free_ros_value(prio);
	free_ros_value(timer_period);

	// Free subscribed topic list
	if (topics_subscribed != NULL) {
		for (ros_value_t **p = topics_subscribed; *p != NULL; ++p) {
			free_ros_value(*p);
		}
		free_ros_value(topics_subscribed);
	}

	// Free publish topic list
	if (topics_publish != NULL) {
		for (ros_value_t **p = topics_publish; *p != NULL; ++p) {
			free_ros_value(*p);
		}
		free_ros_value(topics_publish);
	}

	// Finally free the callback itself
	free(callback_p);
}

void free_node (ros_node_t *node_p)
{

	// Ignore NULL pointer
	if (node_p == NULL) {
		return;
	}

	// Free name
	free(node->name);

	// Free individual callbacks, then the callback array
	if (node->callbacks != NULL) {
		for (ros_callback_t **p = node->callbacks; *p != NULL; ++p) {
			free_callback(*p);
		}
		free(node->callbacks);
	}

	// Finally free the node itself
	free(node_p);
}

void free_executor (ros_executor_t *executor_p)
{

	// Ignore NULL pointer
	if (executor_p == NULL) {
		return;
	}

	// Free node elements
	if (executor_p->nodes != NULL) {
		for (ros_node_t **n = executor_p->nodes; *n != NULL; ++n) {
			free_node(*n);
		}
		free(executor_p->nodes);
	}

	// Finally free the executor itself
	free(executor_p);
}

ros_callback_t *parse_ros_callback (xml_element_t *element)
{
	ros_callback_t callback_p = NULL;
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

	// Match optional subscribed topics
	if ((set_element = exists_element_with_key("topics_subscribed", 
		element->data->collection)) != NULL) {
		ros_value_t **value_set = NULL;

		if ((value_set = parse_strings_from_xml_collection(
			set_element->data.collection)) == NULL) {
			fprintf(stderr, "Anomaly in \"topics_subscribed\" collection!\n");
			goto discard;
		} else {
			callback.topics_subscribed = value_set;
		}
	}

	// Match optional publish topics
	if ((set_element = exists_element_with_key("topics_publish", 
		element->data->collection)) != NULL) {
		ros_value_t **value_set = NULL;

		if ((value_set = parse_strings_from_xml_collection(
			set_element->data.collection)) == NULL) {
			fprintf(stderr, "Anomaly in \"topics_publish\" collection!\n");
			goto discard;
		} else {
			callback.topics_publish = value_set;
		}
	}

	// Allocate a callback element 
	if ((callback_p = malloc(sizeof(ros_callback_t))) == NULL) {
		fprintf(stderr, "%s:%d: Unable to allocate callback!\n", __FILE__, 
			__LINE__);
		goto discard;
	}

	// Copy contents in
	memcpy(callback_p, &callback, sizeof(ros_callback_t));

	// Return pointer
	return callback_p;

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
	ros_node_t *node_p = NULL;
	xml_element_t **e = NULL;
	ros_callback_t **callbacks = NULL;
	size_t size = 0;
	off_t offset = 0;

	// Expect element is called node
	if (strcmp(element->tag, "node") != 0) {
		fprintf(stderr, "Expected tag \"node\", but got %s\n", 
			element->tag);
		return NULL;
	}

	// Expect element has type collection
	if (element->type != XML_COLLECTION) {
		fprintf(stderr, 
			"Expected \"node\" to contain a collection, but got string\n");
		return NULL;	
	}

	// Expect has a parameter with the name field
	if (element->param == NULL || strcmp(element->param->key, "name") == 0) {
		fprintf(stderr, "Expected parameter \"name\"! Either missing \
			or mistyped!\n");
		return NULL;
	}

	// Compute number of callbacks
	for (e = element->data->collection; (*e) != NULL; ++e) {
		size++;
	}

	// Allocate memory for callbacks
	if ((callbacks = malloc((size + 1) * sizeof(ros_callback_t *))) == NULL) {
		fprintf(stderr, "%s:%d Unable to allocate memory for callbacks!\n",
			__FILE__, __LINE__);
		goto discard;
	}

	// Mark end of array with NULL
	callbacks[size] = NULL;

	// Parse elements into callbacks
	for (e = element->data->collection; (*e) != NULL; ++e, ++offset) {
		ros_callback_t *callback;

		// Try parsing as callback
		if ((callback = parse_ros_callback(*e)) == NULL) {
			fprintf(stderr, "Unable to parse callback in node!\n");
			goto discard;
		}

		// Assign pointer
		callbacks[offset] = callback;
	}

	// Allocate and configure a node
	if ((node_p = malloc(sizeof(ros_callback_t))) == NULL) {
		fprintf(stderr, "%s:%d: Unable to allocate memory for node!\n",
			__FILE__, __LINE__);
		goto discard;
	}

	// Copy the name of the node (parameter)
	if (!acceptString(element->param->value, &(node_p->name))) {
		fprintf(stderr, "%s:%d: Unable to allocate copy of parameter value!\n",
			__FILE__, __LINE__);
		goto discard;
	}

	// Configure and return pointer
	node_p->callbacks = callbacks;

	return node_p;

discard:

	if (callbacks != NULL) {
		for (ros_callback_t **c = callbacks; *c != NULL; ++c) {
			free_callback(*c);
		}
		free(callbacks);
	}

	if (node_p != NULL) {
		free(node_p->name);
		free(node_p);
	}

	return NULL;
}

ros_executor_t *parse_ros_executor (xml_element_t *element)
{
	ros_executor_t *executor_p = NULL;
	xml_element_t **e = NULL;
	ros_node_t **nodes = NULL;
	size_t size = 0;
	off_t offset = 0;

	// Expect element is called executor
	if (strcmp(element->tag, "executor") != 0) {
		fprintf(stderr, "Expected tag \"node\", but got %s\n", 
			element->tag);
		return NULL;
	}

	// Expect element has type collection
	if (element->type != XML_COLLECTION) {
		fprintf(stderr, 
			"Expected \"node\" to contain a collection, but got string\n");
		return NULL;	
	}

	// Expect it has a parameter called id
	if (element->param == NULL || strcmp(element->param->key, "id") == 0) {
		fprintf(stderr, "Expected parameter \"id\"! Either missing \
			or mistyped!\n");
		return NULL;
	}

	// Compute number of nodes
	for (e = element->data->collection; (*e) != NULL; ++e) {
		size++;
	}

	// Allocate memory for nodes
	if ((nodes = malloc((size + 1) * sizeof(ros_node_t *))) == NULL) {
		fprintf(stderr, "%s:%d Unable to allocate memory for nodes!\n",
			__FILE__, __LINE__);
		goto discard;
	}

	// Mark end of array with NULL
	nodes[size] = NULL;

	// Parse elements into callbacks
	for (e = element->data->collection; (*e) != NULL; ++e, ++offset) {
		ros_node_t *node;

		// Try parsing as node
		if ((node = parse_ros_node(*e)) == NULL) {
			fprintf(stderr, "Unable to parse node in executor!\n");
			goto discard;
		}

		// Assign pointer
		nodes[offset] = node;
	}

	// Allocate and configure a executor
	if ((executor_p = malloc(sizeof(ros_executor_t))) == NULL) {
		fprintf(stderr, "%s:%d: Unable to allocate memory for executor!\n",
			__FILE__, __LINE__);
		goto discard;
	}

	// Copy the identifier of the executor (parameter)
	if (!acceptLong(element->param->value, &(executor_p->id))) {
		fprintf(stderr, "Unable to parse integer ID from \"%s\" for \
			executor parameter!\n", element->param->value);
		goto discard;
	}

	// Configure and return pointer
	executor_p->nodes = nodes;

discard:

	if (nodes != NULL) {
		for (ros_node_t **n = nodes; *n != NULL; ++n) {
			free_node(*n);
		}
		free(nodes);
	}

	if (executor_p != NULL) {
		free(executor_p);
	}

	return NULL;
}