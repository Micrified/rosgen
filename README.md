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

1. Package: (name, dependencies, message-type, executors)
1. Executor: (identifier, nodes)
2. Nodes: (name, callbacks)
3. Callbacks: (name, worst-case execution time, priority, timer period, topics, subscriptions)


These are mapped to executable as follows: 

1. Packages are mapped to a set of executable files
2. Executors are mapped to an executable file (single-threaded ROS2 executor)
3. Nodes are mapped to C++ classes and registered within their host executor
4. Callbacks are mapped to nodes as methods

See the `test_input.xml` file of the repository for an example of how to create a configuration!

---

## Customization

It's tiresome to generate a project, then have to tediously edit each file to add your logging. Since all the action happens in callbacks, a generalized callback function is compiled that gets called by each callback. This method contains all the information required to identify the callback, and can be useful for implementing custom logging or other functionality: 

```
<msg_type> *on_callback (<msg_type>::SharedPtr msg_recv, int64_t executor_id, const char *node_name, const char *callback_name, int64_t callback_priority, int64_t callback_wcet_ns, bool is_timer_triggered);
```

Simply add code to `on_callback.cpp` and this will be run by all callbacks. You may also optionally allocate an instance of your message type with (`new`) and return it to be sent. It will be automatically recycled if non-null.

## Catches

**NOTE**: If you implement a callback as a timer, it should not be a callback to a subscription!

**NOTE**: All executors generated are single-threaded executors. 

## Output

This program only outputs executable files along with the `CMakeLists.txt` and `package.xml` file. You should use the `pkg create --build-type ament_cmake <package_name>` command from the ROS2 CLI to create your project. Then: 

1. Put all your executables in `src/<package_name>/src/`
2. Put all your headers in `src/<package_name>/include/<package_name>/`
3. Put the `CMakeLists` and `package.xml` in `src/<package_name>/`

Note that these files are somewhat crudely generated and may need tweaking if additional packages and libraries are involved. This tool is meant for rapid program generation and not really as a flexible tool for building real ROS2 projects.
