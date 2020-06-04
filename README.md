# ROSGEN

A ROS2 program generator. Program input is accepted from either:

1. stdin
2. file (specified in the program arguments)

## Compiling

Compile this as a normal C++ program, as shown below:

```
gcc -o rosgen ros_synth.c ros_gen.c ros_parse.c ros_semantics.c
```

## Input Format

Input files should be formatted in XML. The file should contain the following elements in a hierarchy:

1. Executor (with an integer identifier parameter)
2. Nodes (within an executor, with a name parameter)
3. Callbacks (within nodes, these contain extensive callback details)

A callback is where most of the configuration is done. You have the following available fields:

* `<name> STRING </name>`: The name of the callback (this becomes the method identifier)
* `<wcet> INT </wcet>`: The worst-case execution time (implemented as an artificial delay)
* `<prio> INT </prio>`: A priority value (used as message argument when publishing)
* `<timer> INT </timer>`: An optional field, which if present, makes it the callback time triggered with a timer (period in nanoseconds)
* `<topics_publish> ... </topics_publish>`: The topics that the callback will publish on (nested as `<topic> STRING </topic>`)
* `<topics_subscribed> ... </topics_subscribed>`: The topics that the callback will be invoked when the executor receives a message (nested as `<topic> STRING </topic>`)

See the `test_input.xml` file of the repository for an example of how to create a configuration!

---

## Catches

**NOTE**: If you implement a callback as a timer, it should not be a callback to a subscription!

**NOTE**: All executors generated are single-threaded executors. 

## Output

This program outputs one executable per-executor. That executor runs all the nested nodes, and callbacks within the nodes. You should configure a new project in ROS2, and then use this to generate the executables. 

For the time being, you still need to make the Makefile and the package.xml file yourself. Dependencies are also a bit tricky to do. Please check the Wiki for an early guide on how to make these!


