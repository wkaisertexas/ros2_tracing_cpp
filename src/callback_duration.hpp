/// Callback duration calculator
///
/// Uses three lttng tracepoints
/// - ros2:rclcpp_callback_register to associate a symbol with a callback address
/// - ros2:callback_start to write to an unordered map that the callback at that memory address was called
/// - ros2:callback_end to log the ending (and therefore the duration of) the callback

#pragma once

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cinttypes>
#include <cstring>
#include <sys/types.h>
#include <vector>
#include <unordered_map>
#include <utility>
#include <string>
#include <iostream>
#include <fstream>
#include <babeltrace2/babeltrace.h>

/// Structure to hold callback metadata
struct CallbackInfo {
    /// The full path to the symbol registered
    std::string symbol;
    /// Name of the registering node
    std::string procname;
};

/// Sink component's private data
struct CallbackDuration {
    /// Message iterator used by every babeltrace program
    bt_message_iterator *message_iterator;

    /// Index of the events trigger
    uint64_t events_index;

    /// Maps a callback address to some metadata about the callback
    std::unordered_map<uint64_t, CallbackInfo> callback_map;

    /// Unordered map which maps a callback address to the nearest starting callback
    ///
    /// The value of the unordered map is the time in nanoseconds since the UNIX epoch
    std::unordered_map<uint64_t, uint64_t> callback_last_called;

    /// Unordered map which maps a callback address to a list of callbacks and their duration
    std::unordered_map<uint64_t, std::vector<std::pair<uint64_t, uint64_t>>> callback_durations;
};

/// Creates a final consumer of trace information which collects the callback durations
static
bt_component_class_initialize_method_status callback_duration_initialize(
        bt_self_component_sink *self_component_sink,
        bt_self_component_sink_configuration * /*configuration*/,
        const bt_value * /*params*/, void * /*initialize_method_data*/);

/// Called when the sink has processed all the traces
static
void callback_duration_finalize(bt_self_component_sink *self_component_sink);

/// Called when the trace processing graph containing the sink component is configured.
static
bt_component_class_sink_graph_is_configured_method_status
callback_duration_graph_is_configured(bt_self_component_sink *self_component_sink);

/// Uses the registration of callbacks to capture the symbol
/// 
/// Then uses a standard map to get the time each callback was called and log the callback being created
static
void callback_duration_calculator(struct CallbackDuration *callback_duration, const bt_message *message);

/// Consumes a batch of messages to calculate the ros2 callback durations
bt_component_class_sink_consume_method_status callback_durations_consume(
        bt_self_component_sink *self_component_sink);