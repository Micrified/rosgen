#include "ros_semantics.c"


const char *g_callback_keys[] = {
	"name",
	"wcet",
	"prio"
};

ros_value_t *fetch_field (char *key, ros_element_type_t type, 
	xml_element_t **collection)
{
	xml_element_t *value = NULL;

	// Search for key
	for (xml_element_t **p = collection; *p != NULL; ++p) {
		if (strcmp((*p)->tag, key) == 0) {
			value = (*p);
			break;
		}
	}

	// If not found
	if (value == NULL) {
		fprintf(stderr, "No value for key \"%s\"\n", key);
		return NULL;
	}

	// Attempt to convert field to desired type
	switch (type) {
		case TYPE_SHORT: {

		}
		break;
		case TYPE_LONG: {

		}
		break;
		case TYPE_STRING: {

		}
		break;
		default: {

		}
		break;
	}

	return // TODO
}

ros_callback_t *parse_ros_callback (xml_element_t *element)
{
	const char *keys =
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

}

ros_node_t *parse_ros_node (xml_element_t *element)
{

}

ros_executor_t *parse_ros_executor (xml_element_t *element)
{

}