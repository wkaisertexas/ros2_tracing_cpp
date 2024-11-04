/// Memory usage calculator
///
/// Logs allocation and deallocations and the life of each. Does not really care about ros2 nodes because I wanted to avoid building ros2 from scratch 
///
/// Uses six lttng tracepoints
/// - lttng_ust_libc:malloc
/// - lttng_ust_libc:calloc
/// - lttng_ust_libc:realloc
/// - lttng_ust_libc:memalign
/// - lttng_ust_libc:posix_memalign
/// - lttng_ust_libc:free 

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cinttypes>
#include <cstring>
#include <sys/types.h>
#include <vector>
#include <unordered_map>
#include <utility>
#include <queue>
#include <string>
#include <iostream>
#include <fstream>
#include <babeltrace2/babeltrace.h>

/// Change the address size if you are running on a non-64 bit architecture
using Address = uint64_t;

/// Types of vpid and vtid
using VID = int64_t;

/// Print frequency to tell how many events are processed
constexpr const size_t PRINT_FREQ = 1000000;

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
std::string node_info_make_filename(const NodeInfo& node) {
    return "mem_" + node.procname + ".csv";
}

/// Computes the size difference from an allocation
inline uint64_t allocation_info_diff_size(const AllocationInfo& alloc_info){
    if(alloc_info.type == AllocationType::REALLOC){
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
static
bt_component_class_initialize_method_status memory_usage_initialize(
        bt_self_component_sink *self_component_sink,
        bt_self_component_sink_configuration * /*configuration*/,
        const bt_value * /*params*/, void * /*initialize_method_data*/)
{
    auto *memory_usage = new MemoryUsage();

    /* Initialize the first event message's index */
    const size_t DEFAULT_SIZE = 10000;
    memory_usage->events_index = 1;
    memory_usage->addr_to_alllocations_idx.reserve(DEFAULT_SIZE);
    memory_usage->addr_to_alllocations_idx.max_load_factor(0.3);
    memory_usage->allocations.reserve(DEFAULT_SIZE);

    // TODO: think about passing data to my configuration with params
    // see: https://chatgpt.com/c/6726ae8a-5e00-8008-b4d7-5fbd480f8964

    /* Set the component's user data to our private data structure */
    bt_self_component_set_data(
        bt_self_component_sink_as_self_component(self_component_sink),
        memory_usage);

    /* Add an input port named `in` to the sink component */
    bt_self_component_sink_add_input_port(self_component_sink, "in", nullptr, nullptr);

    return BT_COMPONENT_CLASS_INITIALIZE_METHOD_STATUS_OK;
}

/// Finalize is called when the message stream ends and this leads to the data being written to a file
static
void memory_usage_finalize(bt_self_component_sink *self_component_sink)
{
    /* Retrieve our private data from the component's user data */
    auto *memory_usage = (struct MemoryUsage*) bt_self_component_get_data(
        bt_self_component_sink_as_self_component(self_component_sink));

    /* Iterate through writing allocations */
    for(size_t idx = 0; idx < memory_usage->allocations.size(); idx++){
        auto &alloc_info = memory_usage->allocations[idx]; 
        auto &node_info = memory_usage->vpid_to_node[alloc_info.vpid];
        auto &output_file = node_info.output_file;

        if(!output_file.is_open()){
            output_file.open(node_info_make_filename(node_info));

            output_file << "type,time,duration,prev_size,size,vtid" << std::endl;
        }

        /* writing allocation type */
        switch(alloc_info.type) {
            case AllocationType::MALLOC:
                output_file << "malloc,";
                break;
            case AllocationType::CALLOC:
                output_file << "calloc,";
                break;
            case AllocationType::REALLOC:
                output_file << "realloc,";
                break;
            case AllocationType::MEMALIGN:
                output_file << "memalign,";
                break;
            case AllocationType::POSIX_MEMALIGN:
                output_file << "posix_memalign,";
                break;
            default:
                std::cerr << "Allocation type not known" << std::endl;
        }

        /* writing the time */
        output_file << alloc_info.time << ",";

        /* writing the duration */
        output_file << alloc_info.duration << ",";

        /* writing the previous size */
        if(alloc_info.type == AllocationType::REALLOC){
            output_file << alloc_info.prev_size;
        }

        output_file << ",";

        /* writing the allocation size */
        output_file << alloc_info.size << ",";

        /* writing the thread id */
        output_file << alloc_info.vtid;

        /* writing a new line (noteable not std::endl) so the buffer is not flushed each time for performance reasons */
        output_file << "\n";
    }

    /* Going through to close each file */
    for(auto &vpid_and_node_info: memory_usage->vpid_to_node){
        auto& output_file = vpid_and_node_info.second.output_file;
        if(output_file.is_open()){
            output_file.close();
        }
    }
    
    /* Creating a metadata page */
    std::ofstream metadata("memory_usage_metadata.csv");
    metadata << "path,vpid,procname,avg_alloc_lifecycle,avg_alloc_size,allocation_count,max_memory" << std::endl;

    for(auto &vpid_and_node_info: memory_usage->vpid_to_node){
        auto &node_info = vpid_and_node_info.second;

        /* filename */
        metadata << node_info_make_filename(node_info) << ",";

        /* virtual pid */
        metadata << node_info.vpid << ",";

        /* procname */
        metadata << node_info.procname << ",";

        /* summary statistics */
        uint64_t count = node_info.count;

        uint64_t avg_alloc_size = node_info.total_size / count;
        uint64_t avg_alloc_lifecycle = node_info.total_duration / count;

        metadata << avg_alloc_lifecycle << ",";
        metadata << avg_alloc_size << ",";
        metadata << count << ",";

        /* maximum process memory */
        metadata << node_info.max_memory;

        metadata << std::endl;
    }

    delete memory_usage;
}

/// Called when the trace processing graph containing the sink component
/// is configured.
static
bt_component_class_sink_graph_is_configured_method_status
memory_usage_graph_is_configured(bt_self_component_sink *self_component_sink)
{
    /* Retrieve our private data from the component's user data */
    auto *memory_usage = (struct MemoryUsage*) bt_self_component_get_data(
        bt_self_component_sink_as_self_component(self_component_sink));

    /* Borrow our unique port */
    bt_self_component_port_input *in_port =
        bt_self_component_sink_borrow_input_port_by_index(
            self_component_sink, 0);

    /* Create the upstream message iterator */
    bt_message_iterator_create_from_sink_component(self_component_sink,
        in_port, &memory_usage->message_iterator);

    return BT_COMPONENT_CLASS_SINK_GRAPH_IS_CONFIGURED_METHOD_STATUS_OK;
}

/// Computes memory usage statistics given an event message
static
void memory_usage_calculator(struct MemoryUsage *memory_usage, const bt_message *message)
{
    /* Discard if it's not an event message */
    if (bt_message_get_type(message) != BT_MESSAGE_TYPE_EVENT) {
        return;
    }

    /* Increment the current event message's index */
    memory_usage->events_index++;
    if(memory_usage->events_index % PRINT_FREQ == 0){
        std::cerr << "Processed " << memory_usage->events_index / 1000000 << " million events" << std::endl; 
    }

    /* Borrow the event message's event and its class */
    const bt_event *event = bt_message_event_borrow_event_const(message);
    const bt_event_class *event_class = bt_event_borrow_class_const(event);
    const char *event_name = bt_event_class_get_name(event_class);

    /* Getting the timestamp of the event */
    const bt_clock_snapshot *clock_snapshot = bt_message_event_borrow_default_clock_snapshot_const(message);
    int64_t timestamp_ns;
    if(clock_snapshot != nullptr) {
        bt_clock_snapshot_get_ns_from_origin(clock_snapshot, &timestamp_ns);
    } else {
        std::cerr << "Failed to get the clock for a message" << std::endl;
        return;
    }

    memory_usage->max_timestamp = std::max(memory_usage->max_timestamp, timestamp_ns);

    /* Looking at memory allocations */
    AllocationType alloc_type;
    bool is_allocation = true;
    if (strcmp(event_name, "lttng_ust_libc:malloc") == 0) {
        alloc_type = AllocationType::MALLOC;
    } else if(strcmp(event_name, "lttng_ust_libc:calloc") == 0) {
        alloc_type = AllocationType::CALLOC;
    } else if(strcmp(event_name, "lttng_ust_libc:realloc") == 0) {
        alloc_type = AllocationType::REALLOC;
    } else if(strcmp(event_name, "lttng_ust_libc:memalign") == 0){
        alloc_type = AllocationType::MEMALIGN;
    } else if(strcmp(event_name, "lttng_ust_libc:posix_memalign") == 0){
        alloc_type = AllocationType::POSIX_MEMALIGN;
    } else {
        is_allocation = false;
    }

    if(is_allocation) {
        const bt_field *payload = bt_event_borrow_payload_field_const(event);
        const bt_field *event_common_context = bt_event_borrow_common_context_field_const(event);

        /* Fields extraction */
        const bt_field *pointer_address_field = bt_field_structure_borrow_member_field_by_name_const(payload, "ptr");
        const Address pointer_address_value = bt_field_integer_unsigned_get_value(pointer_address_field);
        const bt_field *vtid_field = bt_field_structure_borrow_member_field_by_name_const(event_common_context, "vtid");
        const VID vtid = bt_field_integer_signed_get_value(vtid_field);
        const bt_field *vpid_field = bt_field_structure_borrow_member_field_by_name_const(event_common_context, "vpid");
        const VID vpid = bt_field_integer_signed_get_value(vpid_field);
        const bt_field *size_field = bt_field_structure_borrow_member_field_by_name_const(payload, "size");
        const Address allocation_size = bt_field_integer_unsigned_get_value(size_field);

        if(memory_usage->vpid_to_node.find(vpid) == memory_usage->vpid_to_node.end()){
            auto& node_info = memory_usage->vpid_to_node[vpid];

            const bt_field *procname_field = bt_field_structure_borrow_member_field_by_name_const(event_common_context, "procname");
            const char *procname_value = procname_field != nullptr ? bt_field_string_get_value(procname_field) : "";

            node_info.procname = std::string(procname_value);
            node_info.vpid = vpid;
        }

        /* Checks whether a previous value was contained in the array in cases where a realloc is present */
        bool should_log_reallocation = alloc_type == AllocationType::REALLOC && memory_usage->addr_to_alllocations_idx.find(pointer_address_value) != memory_usage->addr_to_alllocations_idx.end();

        /* Special reallocation case */
        if(should_log_reallocation){
            const auto realloc_idx = memory_usage->addr_to_alllocations_idx[pointer_address_value];
            auto &alloc_info = memory_usage->allocations[realloc_idx];

            alloc_info.duration = timestamp_ns - alloc_info.time;
        }

        /* Creating the allocation information object */
        AllocationInfo alloc_info;

        alloc_info.vtid = vtid;
        alloc_info.vpid = vpid;
        alloc_info.time = timestamp_ns;
        alloc_info.duration = 0; // 0 means it was never freed
        alloc_info.size = allocation_size;
        alloc_info.prev_size = 0;
        alloc_info.type = alloc_type;

        /* Updating summary statistics */
        auto& node_info = memory_usage->vpid_to_node[vpid];
        node_info.count++;
        
        const uint64_t diff_size = allocation_info_diff_size(alloc_info); 
        node_info.total_size += diff_size;
        node_info.curr_memory += diff_size;
        node_info.max_memory = std::max(node_info.max_memory, node_info.curr_memory);

        memory_usage->addr_to_alllocations_idx[pointer_address_value] = memory_usage->allocations.size();
        memory_usage->allocations.push_back(alloc_info);

        return;
    }

    /* Handling memory free events */
    if(strcmp(event_name, "lttng_ust_libc:free") == 0) {
        const bt_field *payload = bt_event_borrow_payload_field_const(event);
        const bt_field *event_common_context = bt_event_borrow_common_context_field_const(event);

        /* Fields extraction */
        const bt_field *vpid_field = bt_field_structure_borrow_member_field_by_name_const(event_common_context, "vpid");
        const VID vpid = bt_field_integer_signed_get_value(vpid_field);
        const bt_field *pointer_address_field = bt_field_structure_borrow_member_field_by_name_const(payload, "ptr");
        const Address pointer_address_value = bt_field_integer_unsigned_get_value(pointer_address_field);

        /* Get the past allocation information and set the duration */
        auto allocation_idx = memory_usage->addr_to_alllocations_idx[pointer_address_value];
        if(allocation_idx >= memory_usage->allocations.size()){
            return;
        }

        auto &allocation_info = memory_usage->allocations[allocation_idx];
        auto &node_info = memory_usage->vpid_to_node[vpid];
        node_info.curr_memory -= allocation_info.size;

        allocation_info.duration = timestamp_ns - allocation_info.time;
        node_info.total_duration += allocation_info.duration;
        memory_usage->addr_to_alllocations_idx.erase(pointer_address_value);
    }
}

/// Consumes a "batch" of messages all at once and calculates their memory usage
bt_component_class_sink_consume_method_status memory_usage_consume(
        bt_self_component_sink *self_component_sink)
{
    bt_component_class_sink_consume_method_status status =
        BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_OK;

    /* Retrieve our private data from the component's user data */
    auto *memory_usage = (struct MemoryUsage *) bt_self_component_get_data(
        bt_self_component_sink_as_self_component(self_component_sink));

    /* Consume a batch of messages from the upstream message iterator */
    bt_message_array_const messages;
    Address message_count;
    bt_message_iterator_next_status next_status =
        bt_message_iterator_next(memory_usage->message_iterator, &messages,
            &message_count);

    if (next_status == BT_MESSAGE_ITERATOR_NEXT_STATUS_END) {
        /* Decrements the reference counter of the message (effectivelly a free) */
        bt_message_iterator_put_ref(memory_usage->message_iterator);
        return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_END;
    }
    
    if (next_status == BT_MESSAGE_ITERATOR_NEXT_STATUS_AGAIN) {
        return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_AGAIN;
    }
    
    if (next_status == BT_MESSAGE_ITERATOR_NEXT_STATUS_MEMORY_ERROR) {
        return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_MEMORY_ERROR;
    } 

    if (next_status == BT_MESSAGE_ITERATOR_NEXT_STATUS_ERROR) {
        return BT_COMPONENT_CLASS_SINK_CONSUME_METHOD_STATUS_ERROR;
    }

    /* For each consumed message */
    for (Address i = 0; i < message_count; i++) {
        /* Current message */
        const bt_message *message = messages[i];

        /* Print line for current message if it's an event message */
        memory_usage_calculator(memory_usage, message);

        /* Decrements the reference counter of the message (effectivelly a free) */
        bt_message_put_ref(message);
    }

    return status;
}

/* Mandatory */
BT_PLUGIN_MODULE();

/* Define the `memory_usage` plugin */
BT_PLUGIN(memory_usage);

/* Define the `output` sink component class */
BT_PLUGIN_SINK_COMPONENT_CLASS(output, memory_usage_consume);

/* Set some of the `output` sink component class's optional methods */
BT_PLUGIN_SINK_COMPONENT_CLASS_INITIALIZE_METHOD(output,
    memory_usage_initialize);
BT_PLUGIN_SINK_COMPONENT_CLASS_FINALIZE_METHOD(output, memory_usage_finalize);
BT_PLUGIN_SINK_COMPONENT_CLASS_GRAPH_IS_CONFIGURED_METHOD(output,
    memory_usage_graph_is_configured);
