#include "callback_duration.hpp"
#include "memory_usage.hpp"

/* Mandatory */
BT_PLUGIN_MODULE();

/* Define the `trace_analysis` plugin */
BT_PLUGIN(trace_analysis);

/* Author Information */
BT_PLUGIN_DESCRIPTION("ROS2 trace processing sinks");
BT_PLUGIN_AUTHOR("William Kaiser");
BT_PLUGIN_LICENSE("CC BY-NC 4.0.");

/* Define the `memory_usage` sink component class */
BT_PLUGIN_SINK_COMPONENT_CLASS(memory_usage, memory_usage_consume);
BT_PLUGIN_SINK_COMPONENT_CLASS_DESCRIPTION(memory_usage,
    "Computes the heap memory usage of ROS 2 nodes");

/* Set some of the `memory_usage` sink component class's optional methods */
BT_PLUGIN_SINK_COMPONENT_CLASS_INITIALIZE_METHOD(memory_usage,
    memory_usage_initialize);
BT_PLUGIN_SINK_COMPONENT_CLASS_FINALIZE_METHOD(memory_usage, memory_usage_finalize);
BT_PLUGIN_SINK_COMPONENT_CLASS_GRAPH_IS_CONFIGURED_METHOD(memory_usage,
    memory_usage_graph_is_configured);

/* Define the `callback_duration` sink component class */
BT_PLUGIN_SINK_COMPONENT_CLASS(callback_duration, callback_durations_consume);
BT_PLUGIN_SINK_COMPONENT_CLASS_DESCRIPTION(memory_usage,
    "Computes the callback durations of ROS 2 nodes");

/* Set some of the `output` sink component class's optional methods */
BT_PLUGIN_SINK_COMPONENT_CLASS_INITIALIZE_METHOD(callback_duration,
    callback_duration_initialize);
BT_PLUGIN_SINK_COMPONENT_CLASS_FINALIZE_METHOD(callback_duration, callback_duration_finalize);
BT_PLUGIN_SINK_COMPONENT_CLASS_GRAPH_IS_CONFIGURED_METHOD(callback_duration,
    callback_duration_graph_is_configured);