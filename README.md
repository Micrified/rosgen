# ROSGEN

A ROS2 program generator. Program input is accepted from either:

1. stdin
2. file (specified in the program arguments)

## Input Format

Input files are basic XML. They may contain `callback` elements with the following fields

* `id`: [Required] Identifier for the callback
* `wcet`: [Required] An artificial delay representing the worst-case execution time of the callback
* `node`: [Required] The node (identifier) that this callback will be placed in
* `exec`: [Required] The identifier (integer) of the executor that callback should be placed in
* `prio`: [Required] A priority value (integer)
* `timer`: [Optional] If present, the callback is invoked by the given integer delay (ns). May not be used with subscribers
* `topics_subscribed`: What topics trigger this callback
* `topics_publish`: What topics this callback publishes to

The semantics of the input file have limited checking. Therefore it is largely on the author to ensure that the generated
program makes sense. This is a work-in-progress


