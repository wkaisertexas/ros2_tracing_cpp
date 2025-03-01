/// Memory usage calculator
///
/// Logs allocation and deallocations and the life of each. Does not really care about ros2 nodes because I wanted to
/// avoid building ros2 from scratch
///
/// Uses six lttng tracepoints
/// - lttng_ust_libc:malloc
/// - lttng_ust_libc:calloc
/// - lttng_ust_libc:realloc
/// - lttng_ust_libc:memalign
/// - lttng_ust_libc:posix_memalign
/// - lttng_ust_libc:free

#pragma once

#include <sys/types.h>

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

extern "C" {
#include <babeltrace2/babeltrace.h>
}

/// Change the address size if you are running on a non-64 bit architecture
using Address = uint64_t;

/// Types of vpid and vtid
using VID = int64_t;

/// Print frequency to tell how many events are processed
constexpr const size_t PRINT_FREQ = 1000000;
constexpr const size_t DEFAULT_SIZE = 10000;

/// The types of memory allocation (each have different overhead)
enum AllocationType {
  MALLOC,
  CALLOC,
  REALLOC,
  MEMALIGN,
  POSIX_MEMALIGN,
};

/// Keeps track of metadata about allocations
///
/// Required because different allocation types have differing overhead
struct AllocationInfo {
  /// The time the memory was allocated
  int64_t time;

  /// The virtual pid of the process allocating memory
  VID vpid;

  /// The virtual thread id of process allocating memory
  VID vtid;

  /// The length the allocation lasted (start time minus end_time)
  int64_t duration;

  // The size of the memory allocation
  size_t size;

  // The size of the previous allocation (if realloc was used)
  size_t prev_size;

  /// The type of allocation
  AllocationType type;
};

/// Aggregator of summary statistics per ros2 node
struct NodeInfo {
  /// vpid of the node
  VID vpid;

  /// Current heap-allocated memory usage
  int64_t curr_memory;

  /// Maximum heap-allocated memory usage
  int64_t max_memory;

  /// Name of the process
  std::string procname;

  /// Total number of allocations
  size_t count;

  /// Total size of allocations
  size_t total_size;

  /// Total duration of allocations
  size_t total_duration;

  /// Allocation summary output file
  std::ofstream output_file;
};

/// Makes a filename from a NodeInfo
inline std::string node_info_make_filename(const NodeInfo &node) { return "mem_" + node.procname + ".csv"; }

/// Computes the size difference from an allocation
inline uint64_t allocation_info_diff_size(const AllocationInfo &alloc_info) {
  if (alloc_info.type == AllocationType::REALLOC) {
    return alloc_info.size - alloc_info.prev_size;
  }

  return alloc_info.size;
}

/// Memory usage sink's private data
struct MemoryUsage {
  /// Message iterator used by every babeltrace program
  bt_message_iterator *message_iterator;

  /// The maximum timestamp an allocation occurred, used to calculate the lifecycle of objects which were not freed
  int64_t max_timestamp;

  /// Events count
  uint64_t events_index;

  /// Previous message time
  int64_t prev_msg;

  /// Current memory usage measured in megabytes
  uint64_t memory_usage;

  /// VPID to node_info which is called once
  std::unordered_map<VID, NodeInfo> vpid_to_node;

  /// Memory address to vpid used to keep track of allocation
  std::unordered_map<Address, size_t> addr_to_alllocations_idx;

  /// Final allocation information logged by pid
  /// NOTE: now that I am thinking about it having such a huge vector may not be the best idea, will look at later
  std::vector<AllocationInfo> allocations;
};

/// Initializes the memory usage sink
bt_component_class_initialize_method_status memory_usage_initialize(
    bt_self_component_sink *self_component_sink, bt_self_component_sink_configuration * /*configuration*/,
    const bt_value * /*params*/, void * /*initialize_method_data*/);

/// Finalize is called when the message stream ends and this leads to the data being written to a file
void memory_usage_finalize(bt_self_component_sink *self_component_sink);

/// Called when the trace processing graph containing the sink component
/// is configured.
bt_component_class_sink_graph_is_configured_method_status memory_usage_graph_is_configured(
    bt_self_component_sink *self_component_sink);

/// Computes memory usage statistics given an event message
void memory_usage_calculator(struct MemoryUsage *memory_usage, const bt_message *message);

/// Consumes a "batch" of messages all at once and calculates their memory usage
bt_component_class_sink_consume_method_status memory_usage_consume(bt_self_component_sink *self_component_sink);