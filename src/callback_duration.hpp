/// Callback duration calculator
///
/// Uses three lttng tracepoints
/// - ros2:rclcpp_callback_register to associate a symbol with a callback address
/// - ros2:callback_start to write to an unordered map that the callback at that memory address was called
/// - ros2:callback_end to log the ending (and therefore the duration of) the callback

#pragma once

#include <sys/types.h>

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

extern "C" {
#include <babeltrace2/babeltrace.h>
}

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
static bt_component_class_initialize_method_status callback_duration_initialize(
    bt_self_component_sink *self_component_sink, bt_self_component_sink_configuration * /*configuration*/,
    const bt_value * /*params*/, void * /*initialize_method_data*/) {
  auto *callback_duration = new CallbackDuration();

  /* Initialize the first event message's index */
  callback_duration->events_index = 1;

  const size_t DEFAULT_SIZE = 10000;
  callback_duration->callback_last_called.reserve(DEFAULT_SIZE);
  callback_duration->callback_last_called.max_load_factor(0.3);

  /* Set the component's user data to our private data structure */
  bt_self_component_set_data(bt_self_component_sink_as_self_component(self_component_sink), callback_duration);

  /* Add an input port named `in` to the sink component */
  bt_self_component_sink_add_input_port(self_component_sink, "in", NULL, NULL);

  return BT_COMPONENT_CLASS_INITIALIZE_METHOD_STATUS_OK;
}

/// Called when the sink has processed all the traces
static void callback_duration_finalize(bt_self_component_sink *self_component_sink) {
  /* Retrieve our private data from the component's user data */
  auto *callback_duration = (struct CallbackDuration *)bt_self_component_get_data(
      bt_self_component_sink_as_self_component(self_component_sink));

  std::ofstream metadataFile("callback_duration_metadata.csv");
  metadataFile << "symbol,procname,address,count,path,avg_duration" << std::endl;

  /* Iterate over the callback durations to create CSV files */
  for (const auto &entry : callback_duration->callback_durations) {
    uint64_t callback_address = entry.first;
    const auto &durations = entry.second;

    // Get the CallbackInfo for this callback address
    auto &callback_info = callback_duration->callback_map[callback_address];
    auto &callback_name = callback_info.symbol;
    auto &procname = callback_info.procname;

    std::string s = callback_name;

    /* Sanitize the string for CSV seralization */
    size_t pos = 0;
    while ((pos = s.find('"', pos)) != std::string::npos) {
      s.replace(pos, 1, "\"\"");
      pos += 2;  // Move past the escaped double quotes
    }
    s = "\"" + s + "\"";

    metadataFile << s << ",";
    metadataFile << procname << ",";
    metadataFile << callback_address << ",";
    metadataFile << durations.size() << ",";

    // Sanitize callback_name for filename
    std::string sanitized_callback_name = callback_name;
    // Replace any invalid filename characters
    for (auto &ch : sanitized_callback_name) {
      if (!isalnum(ch) && ch != '-' && ch != '_') {
        ch = '_';
      }
    }

    /* creating the filename */
    char filename[4096];
    snprintf(filename, sizeof(filename), "callback_%" PRIu64 "_%s.csv", callback_address, procname.c_str());
    metadataFile << filename << ",";

    /* opens the file for writing */
    FILE *file = fopen(filename, "w");
    if (file == nullptr) {
      fprintf(stderr, "Failed to open file %s for writing\n", filename);
      continue;
    }

    // Write metadata: callback name on top
    fprintf(file, "%s\n", callback_name.c_str());
    fprintf(file, "%s\n", procname.c_str());

    /* writes the column data */
    fprintf(file, "time,duration\n");

    /* writes the file data */
    double total = 0.0;
    for (const auto &pair : durations) {
      uint64_t time = pair.first;
      uint64_t duration = pair.second;
      total += duration / 1000000000.0 / durations.size();

      fprintf(file, "%" PRIu64 ",%" PRIu64 "\n", time, duration);
    }

    metadataFile << total << std::endl;

    fclose(file);
  }

  std::cout << "Wrote output to metadata.csv..." << std::endl;

  /* Free the allocated structure */
  delete callback_duration;
}

/// Called when the trace processing graph containing the sink component is configured.
static bt_component_class_sink_graph_is_configured_method_status callback_duration_graph_is_configured(
    bt_self_component_sink *self_component_sink) {
  /* Retrieve our private data from the component's user data */
  auto *callback_duration = (struct CallbackDuration *)bt_self_component_get_data(
      bt_self_component_sink_as_self_component(self_component_sink));

  /* Borrow our unique port */
  bt_self_component_port_input *in_port = bt_self_component_sink_borrow_input_port_by_index(self_component_sink, 0);

  /* Create the upstream message iterator */
  bt_message_iterator_create_from_sink_component(self_component_sink, in_port, &callback_duration->message_iterator);

  return BT_COMPONENT_CLASS_SINK_GRAPH_IS_CONFIGURED_METHOD_STATUS_OK;
}

/// Uses the registration of callbacks to capture the symbol
///
/// Then uses a standard map to get the time each callback was called and log the callback being created
static void callback_duration_calculator(struct CallbackDuration *callback_duration, const bt_message *message) {
  /* Discard if it's not an event message */
  if (bt_message_get_type(message) != BT_MESSAGE_TYPE_EVENT) {
    return;
  }

  /* Increment the current event message's index */
  callback_duration->events_index++;

  /* Borrow the event message's event and its class */
  const bt_event *event = bt_message_event_borrow_event_const(message);
  const bt_event_class *event_class = bt_event_borrow_class_const(event);
  const char *event_name = bt_event_class_get_name(event_class);

  /* Check for `ros2:callbackRegistered` event */
  if (strcmp(event_name, "ros2:rclcpp_callback_register") == 0) {
    /* Simulate retrieving the callback address (for demonstration) */

    //  we borrow the payload and use the bt_field_structure_borrow_member_field_by_name_const
    const bt_field *payload = bt_event_borrow_payload_field_const(event);
    const bt_field *event_common_context = bt_event_borrow_common_context_field_const(event);
    const bt_field *callback_address_field = bt_field_structure_borrow_member_field_by_name_const(payload, "callback");
    uint64_t callback_address_value = bt_field_integer_unsigned_get_value(callback_address_field);
    const bt_field *callback_symbol_field = bt_field_structure_borrow_member_field_by_name_const(payload, "symbol");
    const char *symbol_value = callback_symbol_field != nullptr ? bt_field_string_get_value(callback_symbol_field) : "";
    const bt_field *procname_field =
        bt_field_structure_borrow_member_field_by_name_const(event_common_context, "procname");
    const char *procname_value = procname_field != nullptr ? bt_field_string_get_value(procname_field) : "";

    /* Create CallbackInfo and add it to the callback map */
    CallbackInfo callback_info;
    callback_info.symbol = symbol_value;
    callback_info.procname = procname_value;

    callback_duration->callback_map[callback_address_value] = callback_info;

    /* Log the callback registration */
    printf("callback registered(#%" PRIu64 "): %s \n", callback_duration->events_index, procname_value);
  } else if (strcmp(event_name, "ros2:callback_start") == 0) {
    /* log that a callback has been started and the time in nanoseconds that that happens */
    const bt_field *payload = bt_event_borrow_payload_field_const(event);
    const bt_field *callback_address_field = bt_field_structure_borrow_member_field_by_name_const(payload, "callback");
    if (callback_address_field == nullptr) {
      return;
    }
    uint64_t callback_address_value = bt_field_integer_unsigned_get_value(callback_address_field);

    const bt_clock_snapshot *clock_snapshot = bt_message_event_borrow_default_clock_snapshot_const(message);

    int64_t ns_from_origin;
    bt_clock_snapshot_get_ns_from_origin(clock_snapshot, &ns_from_origin);

    callback_duration->callback_last_called[callback_address_value] = ns_from_origin;
  } else if (strcmp(event_name, "ros2:callback_end") == 0) {
    /* log that a callback has been ended */
    const bt_field *payload = bt_event_borrow_payload_field_const(event);
    const bt_field *callback_address_field = bt_field_structure_borrow_member_field_by_name_const(payload, "callback");
    uint64_t callback_address_value = bt_field_integer_unsigned_get_value(callback_address_field);

    const bt_clock_snapshot *clock_snapshot = bt_message_event_borrow_default_clock_snapshot_const(message);

    int64_t ns_from_origin;
    bt_clock_snapshot_get_ns_from_origin(clock_snapshot, &ns_from_origin);

    // logging the duration
    int64_t last_called = callback_duration->callback_last_called[callback_address_value];
    callback_duration->callback_durations[callback_address_value].emplace_back(last_called,
                                                                               ns_from_origin - last_called);

    if (last_called > ns_from_origin) {
      std::cerr << "Message ordering error" << std::endl;
    }
  }
}

/// Consumes a batch of messages to calculate the ros2 callback durations
bt_component_class_sink_consume_method_status callback_durations_consume(bt_self_component_sink *self_component_sink) {
  bt_component_class_sink_consume_method_status status = BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_OK;

  /* Retrieve our private data from the component's user data */
  auto *callback_duration = (struct CallbackDuration *)bt_self_component_get_data(
      bt_self_component_sink_as_self_component(self_component_sink));

  /* Consume a batch of messages from the upstream message iterator */
  bt_message_array_const messages;
  uint64_t message_count;
  bt_message_iterator_next_status next_status =
      bt_message_iterator_next(callback_duration->message_iterator, &messages, &message_count);

  if (next_status == BT_MESSAGE_ITERATOR_NEXT_STATUS_END) {
    /* End of iteration: put the message iterator's reference */
    bt_message_iterator_put_ref(callback_duration->message_iterator);
    return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_END;
  } else if (next_status == BT_MESSAGE_ITERATOR_NEXT_STATUS_AGAIN) {
    return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_AGAIN;
  } else if (next_status == BT_MESSAGE_ITERATOR_NEXT_STATUS_MEMORY_ERROR) {
    return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_MEMORY_ERROR;
  } else if (next_status == BT_MESSAGE_ITERATOR_NEXT_STATUS_ERROR) {
    return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_ERROR;
  }

  /* For each consumed message */
  for (uint64_t i = 0; i < message_count; i++) {
    /* Current message */
    const bt_message *message = messages[i];

    /* Print line for current message if it's an event message */
    callback_duration_calculator(callback_duration, message);

    /* Put this message's reference */
    bt_message_put_ref(message);
  }

  return status;
}
