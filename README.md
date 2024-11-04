# ROS2 Tracing C++

ROS2 Tracing C++ is a custom plugin for `babeltrace2` to speed up trace-processing for ROS2 nodes.

Trace analysis is incredibly powerful. However, processing traces with ROS2 tracing took quite a bit longer than program execution times themselves. As a result, I wrote a custom C++ plugin which uses the same mechanism of analyzing tracing, but is incredibly performance optimized.

## Alternatives to ROS2 Tracing C++

There exist other trace analysis solutions for ROS2

- [ROS2 tracing analysis](https://github.com/ros-tracing/tracetools_analysis/tree/humble) is a good choice if you do not need high-performance trace analysis.
- [LTTNG Analyses](https://github.com/lttng/lttng-analyses) have good examples of trace-analysis scripts written in Python if you are looking to learn more about what you can get from trace-analysis.

## Supported Analysis Sinks

The supported features are inspired by the [ROS2 tracing analysis](https://github.com/ros-tracing/tracetools_analysis/tree/humble) package.

| Plugin | Required Tracepoints | Description |
| :----- | :------------------- | :---------- |
| `sink.trace_analysis.callback_duration` | `ros2:rclcpp_callback_register`, `ros2:callback_start`, `ros2:callback_end` | Collects the duration of each callback triggered |
| `sink.trace_analysis.memory_usage` | `lttng_ust_libc:malloc`, `lttng_ust_libc:calloc`, `lttng_ust_libc:realloc`, `lttng_ust_libc:memalign`, `lttng_ust_libc:posix_memalign`, `lttng_ust_libc:free` | Gets information about the lifecycle of objects allocated by nodes |

## Installation

To use ROS2 Tracing C++, four steps are required:

1. installing the Linux Trace Toolkit: next generation(LTTNG)
2. installing ros2 tracetools and building your code with tracing enabled
3. building `babeltrace2` from source
4. building the plugin `libtrace_analysis.so`

### Installing LTTNG

Install LTTNG from [the stable Linux PPA package](lttng.org/docs/v2.13/#doc-ubuntu-ppa):

```bash
sudo apt-add-repository ppa:lttng/stable-2.13
sudo apt-get update
sudo apt install lttng-tools lttng-modules-dkms liblttng-ust-dev
```

### Installing ROS2 TraceTools Launch

Installing tracetools requires `apt` installing the `tracetools-launch` package and then sourcing the tracetools package **before** building any of the other packages which should be instrumented.

```bash
sudo apt install ros-humble-tracetools-launch
(cd external && git clone https://github.com/ros2/ros2_tracing && cd ros2_tracing && git checkout humble)
colcon build --packages-up-to tracetools
source install/setup.sh
```

After this, you can build your packages with `colcon` like normal. 

> [!WARNING]
> However, you must `source install/setup.sh` before building any of the other packages you want tracepoints enabled in. This is very atypical for a ROS2 project, but is a valid workaround to building with tracepoints enabled while not requiring a from-source ROS2 build.

### Building Babeltrace2 from Source

To build a plugin, building from source is required ([see guide](https://babeltrace.org/docs/v2.0/libbabeltrace2/guide-build-bt2-dev.html)). This assumes you have a `.gitignored` directory named `external` for non-source packages.

```bash
# downloading and extracting babeltrace2
mkdir -p external
cd external
curl https://www.efficios.com/files/babeltrace/babeltrace2-2.0.6.tar.bz2 -o babeltrace.tar.bz2
tar -xvf babeltrace.tar.bz2
cd babeltrace2-*/

# building an installing from source
BABELTRACE_DEV_MODE=1 BABELTRACE_MINIMAL_LOG_LEVEL=TRACE ./configure --disable-debug-info
make -j$(nproc)
sudo make install
cd ../..
```

After that, `babeltrace2` will be built from source and installed.

### Building ROS2 Tracing C++

To build the ROS2 Tracing C++ plugin, run:

```bash
git clone https://github.com/wkaisertexas/ros2_tracing_cpp
cd ros2_tracing_cpp

mkdir build
cd build
cmake ..
make -j$(nproc)
```

At this point, the plugin will be built in `build/plugins/libtrace_analysis.so` where you can reference the different sinks to process your traces.

## Collecting Traces with ROS2 TraceTools Launch

With `tracetools-launch` installed, you can add the following to your launch file to collect traces.

```python
from launch import LaunchDescription

from tracetools_launch.action import Trace

CALLBACK_TRACEPOINTS = set(["ros2:rclcpp_callback_register", "ros2:callback_start", "ros2:callback_end"])
"""Tracepoints required for callback duration analysis"""
MEMORY_TRACEPOINTS = set(["lttng_ust_libc:malloc", "lttng_ust_libc:calloc", "lttng_ust_libc:realloc", "lttng_ust_libc:memalign", "lttng_ust_libc:posix_memalign", "lttng_ust_libc:free"])
"""Tracepoints required for memory usage analysis"""

def generate_launch_description() -> LaunchDescription:
    tracepoints_to_collect = CALLBACK_TRACEPOINTS | MEMORY_TRACEPOINTS
    trace_session = Trace(
        session_name="callback_duration_and_memory_usage",
        events_ust=",".join(tracepoints_to_collect),
        base_path="~/.ros/tracing", # default trace location
    )

    # the rest of your launch file

    return LaunchDescription([
        trace_session,
        # your nodes and parameteres
    ])
```

This launch command will create `~/.ros/tracing/callback_duration_and_memory_usage` which contains several nested folders. Running `babeltrace2 ~/.ros/tracing/callback_duration_and_memory_usage | less` will print out traces.

## Running the plugins

To run the plugin, use babeltrace with the specified plugin path and your tracing session output path.

```bash
babeltrace2 --plugin-path . /home/autera/.ros/tracing/ros2_tracing_session/ --component=sink.callback_duration.output
```

## Using the Pre-Built Jupyter Notebooks for Trace Analysis

For each sink, there is a pre-built Jupyter notebook which makes a standard set of plots per node. These analyses are designed to be extensible and a good starting point for any future work.

| Plugin | Jupyter Notebook |
| :----- | :--------------- |
| `sink.trace_analysis.callback_duration` | [process_callback_duration.ipynb](./scripts/process_callback_duration.ipynb) |
| `sink.trace_analysis.memory_usage` | [process_callback_duration.ipynb](./scripts/process_memory_usage.ipynb) |

## Processing Outputs

In this section, notes and helpful tips to process outputs generated by each plugin is included.

### Callback Duration

For callback duration, you get a `metadata.csv` file which contains links and metadata to a bunch of `callback_*.csv` files. There is a script called `scripts/process_callback_duration.ipynb` which makes callback duration plots using the recording information

| Column | Description |
| :----- | :---------- |
| `symbol` | The raw callback symbol monitored |
| `procname` | Process name of collected callback | 
| `address`  | The address of the selected callback, used as a pseudo-identifier |
| `count` | the number of times the callback was called |
| `path` | the relative path of the file containing the callback |
| `avg_duration` | the average duration of the callback (useful for crude filtering) |

The file contained in the path variable is a CSV file. The first two rows are the `symbol` and the `procname`. Then the file has the following columns:

| Column | Description |
| :----- | :---------- |
| `time`   | time in nanoseconds since the unix epoch that the callback occurred |
| `duration` | the duration of the callback in nanoseconds |

> [!TIP]
> If you are processing this in Pandas, you can use `pd.read_csv(path, skiprows=2)` to ignore the header

### Memory Usage

For the memory usage calculator, the memory usage will produce a metadata file with the following information:

| Columns | Description |
| :------ | :---------- |
| `path`    | the relative path to the `.csv` file containing the allocation information |
| `vpid`    | the virtual pid of the ros2 node |
| `procname` | process name making the allocation |
| `avg_alloc_lifecycle` | the average lifecycle of allocated objects |
| `avg_alloc_size` | the average allocation size |
| `allocation_count` | the total number of allocations |
| `max_process_memory` | the maximum amount of process memory |

The file contained in the `path` variable is a CSV file containing the following information:

| Columns | Description |
| :------ | :---------- |
| `type`  | The type of the allocation either `malloc`, `calloc`, `realloc`, `memalign` or `posix_memalign` |
| `time`  | The time in nanoseconds since the unix epoch the object was allocated |
| `duration` | How long the memory stuck around. A nan value means that the object was not freed |
| `prev_size` | The previous allocation size in bytes (only for realloc) |
| `size` | The size of the allocation in bytes |
| `vtid` | The virtual thread id which called the allocator | 

## Trace Analysis is Taking a Long Time

Looking at a profile of this plugin, iterating through the traces with babeltrace2 takes **over 85% of total time**. Therefore, collecting the minimal set of events required to use each plugin is the single-greatest tactic you can use to reduce program execution time.

![Perf Trace of `memory_usage`](./scripts/imgs/perf_data.png)

> [!IMPORTANT]
> If you want to speed up processing time, consider using a live session created with `llttng create my-session --live` and `babeltrace2 --plugin-path . --input-format=lttng-live net://localhost/host/localhost/my-session --component=sink.callback_duration.output`

## Resources

1. [Babeltrace 2 Sink Example](https://babeltrace.org/docs/v2.0/libbabeltrace2/example-simple-sink-cmp-cls.html)
2. [ROS2 Trace Analysis](https://github.com/ros-tracing/tracetools_analysis/tree/humble)
3. [LTTNG Documentation](https://lttng.org/docs/v2.13/)
4. [ROS2 Tracing Whitepaper](https://arxiv.org/abs/2201.00393)
5. [ROS2 Tracing Github](https://github.com/ros2/ros2_tracing/tree/humble)