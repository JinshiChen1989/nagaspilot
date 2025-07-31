# NagasPilot Debugging Tools Collection - Comprehensive Manual

## System Type Classification: **Development/Analysis/Runtime Tools**

### Overview
This manual covers the comprehensive debugging tools collection in NagasPilot, designed for development, analysis, and runtime diagnostics. These tools are essential for development, troubleshooting, performance optimization, and system analysis.

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [CAN Analysis Tools](#can-analysis-tools)
3. [Performance Analysis Tools](#performance-analysis-tools)
4. [Vehicle Integration Tools](#vehicle-integration-tools)
5. [Data Analysis Tools](#data-analysis-tools)
6. [Touch and UI Debugging Tools](#touch-and-ui-debugging-tools)
7. [System Monitoring Tools](#system-monitoring-tools)
8. [Advanced Analysis Tools](#advanced-analysis-tools)
9. [Integration with Other Systems](#integration-with-other-systems)
10. [Performance Optimization](#performance-optimization)
11. [Troubleshooting Guide](#troubleshooting-guide)
12. [Complete Tool Reference](#complete-tool-reference)

---

## System Architecture

### Core Components
- **Debug Directory**: `/selfdrive/debug/` - Primary debugging tools
- **Tools Directory**: `/tools/` - Advanced analysis and development tools
- **Panda Scripts**: `/panda/scripts/` - Hardware debugging utilities
- **Car-Specific Tools**: `/selfdrive/debug/car/` - Vehicle-specific debugging

### Tool Categories
1. **Runtime Debugging**: Live system analysis and monitoring
2. **Development Tools**: Code development and testing utilities
3. **Analysis Tools**: Post-processing and data analysis
4. **Hardware Tools**: Low-level hardware debugging
5. **Integration Tools**: Cross-system debugging and validation

---

## CAN Analysis Tools

### 1. CAN Printer (`can_printer.py`)

**Purpose**: Real-time CAN message monitoring and analysis

**Usage**:
```bash
# Monitor CAN bus 0 with all messages
cd /data/openpilot
python selfdrive/debug/can_printer.py

# Monitor specific CAN bus
python selfdrive/debug/can_printer.py --bus 1

# Filter messages by maximum address
python selfdrive/debug/can_printer.py --max_msg 2048

# Decode messages as ASCII when possible
python selfdrive/debug/can_printer.py --ascii

# Monitor remote system
python selfdrive/debug/can_printer.py --addr 192.168.1.100
```

**Features**:
- Real-time message frequency analysis
- Hexadecimal and ASCII display modes
- Message count and timing statistics
- Automatic screen clearing and refresh
- Support for remote monitoring

**Output Format**:
```
Time: 123.45
ADDR(decimal)(count)(freq) hexdata                ascii_decode
0123(291)(000012)(25Hz)   DEADBEEF12345678    "....."
0456(1161)(000045)(50Hz)  0102030405060708    "........"
```

### 2. CAN Table (`can_table.py`)

**Purpose**: Tabular display of CAN messages with statistical analysis

**Usage**:
```bash
# Basic CAN table display
python selfdrive/debug/can_table.py

# Show only specific addresses
python selfdrive/debug/can_table.py --filter 0x123,0x456

# Export to CSV format
python selfdrive/debug/can_table.py --csv output.csv
```

### 3. CAN Print Changes (`can_print_changes.py`)

**Purpose**: Monitor and highlight changes in CAN message content

**Usage**:
```bash
# Monitor all message changes
python selfdrive/debug/can_print_changes.py

# Monitor specific addresses only
python selfdrive/debug/can_print_changes.py --addr 0x123,0x456

# Set change detection threshold
python selfdrive/debug/can_print_changes.py --threshold 5
```

**Features**:
- Change detection and highlighting
- Configurable sensitivity thresholds
- Historical change tracking
- Color-coded output for easy identification

### 4. CAN Performance Check (`check_can_parser_performance.py`)

**Purpose**: Analyze CAN parser performance and bottlenecks

**Usage**:
```bash
# Run comprehensive performance test
python selfdrive/debug/check_can_parser_performance.py

# Test specific DBC files
python selfdrive/debug/check_can_parser_performance.py --dbc toyota_prius_2017.dbc

# Benchmark parsing speed
python selfdrive/debug/check_can_parser_performance.py --benchmark
```

---

## Performance Analysis Tools

### 1. CPU Usage Statistics (`cpu_usage_stat.py`)

**Purpose**: Comprehensive CPU usage monitoring for all system processes

**Usage**:
```bash
# Monitor all managed processes
python selfdrive/debug/cpu_usage_stat.py

# Monitor specific processes
python selfdrive/debug/cpu_usage_stat.py pandad,ubloxd,controlsd

# Show detailed CPU time breakdown
python selfdrive/debug/cpu_usage_stat.py --detailed_times

# List all running processes
python selfdrive/debug/cpu_usage_stat.py --list_all
```

**Features**:
- 5-second sampling intervals with 200ms resolution
- Min/max/average CPU usage tracking
- Long-term accumulative statistics
- Detailed time breakdown (user/system/children)
- Process ranking by CPU usage

**Output Format**:
```
Add monitored proc: ./pandad
Add monitored proc: python locationd/ubloxd.py
pandad: avg: 1.96%, min: 1.50%, max: 2.40%, acc: 1.96%
ubloxd.py: avg: 0.39%, min: 0.20%, max: 0.65%, acc: 0.39%
avg sum: 15.40% over 150 samples 30.0 seconds
```

### 2. Timing Analysis (`check_timings.py`)

**Purpose**: Real-time timing analysis of message streams

**Usage**:
```bash
# Monitor specific services
python selfdrive/debug/check_timings.py controlsState carState

# Monitor all camera services
python selfdrive/debug/check_timings.py roadCameraState wideRoadCameraState driverCameraState

# Monitor CAN timing
python selfdrive/debug/check_timings.py can
```

**Features**:
- Real-time timing statistics (mean, std, max, min)
- Rolling 100-message window analysis
- Summary statistics on exit
- Configurable service monitoring

**Output Format**:
```
controlsState            20.05    0.45   22.30   19.60
carState                 10.02    0.23   11.10    9.80
roadCameraState          50.01    2.15   55.40   47.20
```

### 3. Live CPU and Temperature Monitor (`live_cpu_and_temp.py`)

**Purpose**: Real-time system health monitoring

**Usage**:
```bash
# Monitor system health
python selfdrive/debug/live_cpu_and_temp.py

# Log to file
python selfdrive/debug/live_cpu_and_temp.py --log health.log

# Set monitoring interval
python selfdrive/debug/live_cpu_and_temp.py --interval 1.0
```

### 4. Frequency Checker (`check_freq.py`)

**Purpose**: Verify message frequencies against expected rates

**Usage**:
```bash
# Check specific service frequency
python selfdrive/debug/check_freq.py controlsState

# Check multiple services
python selfdrive/debug/check_freq.py controlsState carState roadCameraState
```

---

## Vehicle Integration Tools

### 1. Vehicle Fingerprint Tool (`get_fingerprint.py`)

**Purpose**: Generate vehicle CAN fingerprints for new car support

**Prerequisites**:
- Vehicle in stock mode (no aftermarket modifications active)
- Panda connected and pandad running
- Engine off, ignition on

**Usage**:
```bash
# Start pandad first
cd /data/openpilot
python selfdrive/pandad/pandad.py

# In another terminal, run fingerprinting
python selfdrive/debug/get_fingerprint.py

# Let run for at least 30 seconds to capture all messages
```

**Output**:
```
number of messages 45:
fingerprint 0x18F: 8, 0x191: 8, 0x1A0: 8, 0x1A2: 8, ...
```

**Best Practices**:
- Run for 30+ seconds to capture low-frequency messages
- Ensure all vehicle systems are active (lights, A/C, etc.)
- Document any error codes or warning lights
- Test with different vehicle states (park, drive, etc.)

### 2. Firmware Versions (`fw_versions.py`)

**Purpose**: Query and display vehicle ECU firmware versions

**Usage**:
```bash
# Query all ECU firmware versions
cd /data/openpilot/selfdrive/debug/car
python fw_versions.py

# Query specific ECUs
python fw_versions.py --ecu eps,abs,engine

# Save results to file
python fw_versions.py --output fw_report.json
```

### 3. VIN Reader (`vin.py`)

**Purpose**: Extract Vehicle Identification Number from CAN bus

**Usage**:
```bash
# Read VIN from vehicle
cd /data/openpilot/selfdrive/debug/car
python vin.py

# Decode VIN information
python vin.py --decode

# Verify VIN checksum
python vin.py --verify
```

### 4. Diagnostic Trouble Codes (`read_dtc_status.py`)

**Purpose**: Read and clear diagnostic trouble codes

**Usage**:
```bash
# Read all DTCs
python selfdrive/debug/read_dtc_status.py

# Read DTCs from specific ECU
python selfdrive/debug/read_dtc_status.py --ecu 0x7E0

# Clear DTCs (use with caution)
python selfdrive/debug/read_dtc_status.py --clear
```

### 5. ECU Disabling (`car/disable_ecu.py`)

**Purpose**: Safely disable specific ECUs for testing

**Usage**:
```bash
# List available ECUs
cd /data/openpilot/selfdrive/debug/car
python disable_ecu.py --list

# Disable radar ECU
python disable_ecu.py --disable radar

# Re-enable ECU
python disable_ecu.py --enable radar
```

---

## Data Analysis Tools

### 1. Data Dump Tool (`dump.py`)

**Purpose**: Comprehensive data logging and analysis

**Usage**:
```bash
# Dump all available services
python selfdrive/debug/dump.py

# Dump specific services
python selfdrive/debug/dump.py controlsState carState

# Output as JSON
python selfdrive/debug/dump.py --json controlsState

# Raw binary dump
python selfdrive/debug/dump.py --raw can

# Monitor specific values
python selfdrive/debug/dump.py --values "controlsState.steerAngleDeg,carState.vEgo" controlsState carState

# Pipe output for processing
python selfdrive/debug/dump.py --pipe controlsState | your_analysis_tool
```

**Output Formats**:
- **Standard**: Human-readable protobuf format
- **JSON**: Machine-readable JSON format
- **Raw**: Binary hexdump format
- **Pipe**: Raw binary for external processing

### 2. Log Size Analysis (`qlog_size.py`)

**Purpose**: Analyze log file sizes and storage usage

**Usage**:
```bash
# Analyze current logs
python selfdrive/debug/qlog_size.py

# Analyze specific log directory
python selfdrive/debug/qlog_size.py --path /data/media/0/realdata/

# Show detailed breakdown
python selfdrive/debug/qlog_size.py --detailed

# Export size analysis
python selfdrive/debug/qlog_size.py --export size_report.csv
```

### 3. Event Counter (`count_events.py`)

**Purpose**: Count and analyze message frequencies

**Usage**:
```bash
# Count all events
python selfdrive/debug/count_events.py

# Count events in specific logs
python selfdrive/debug/count_events.py --log /path/to/rlog

# Filter by message type
python selfdrive/debug/count_events.py --filter controlsState,carState
```

### 4. Route Process Runner (`run_process_on_route.py`)

**Purpose**: Run analysis processes on historical route data

**Usage**:
```bash
# Run controlsd on historical route
python selfdrive/debug/run_process_on_route.py --route "route_id" --process controlsd

# Run with custom parameters
python selfdrive/debug/run_process_on_route.py --route "route_id" --process plannerd --params "param1=value1"
```

---

## Touch and UI Debugging Tools

### 1. Touch Replay Visualizer (`touch_replay.py`)

**Purpose**: Visualize and analyze touch interactions

**Usage**:
```bash
# Analyze touch data from route
python selfdrive/debug/touch_replay.py --route /path/to/rlog

# Custom screen dimensions
python selfdrive/debug/touch_replay.py --width 2160 --height 1080 --route /path/to/rlog

# Analyze specific touch patterns
python selfdrive/debug/touch_replay.py --route /path/to/rlog --pattern tap,swipe
```

**Features**:
- Heat map visualization of touch points
- Touch frequency analysis
- Multi-touch gesture tracking
- Custom screen resolution support

### 2. UI View Tool (`uiview.py`)

**Purpose**: Debug UI state and interactions

**Usage**:
```bash
# Monitor UI state
python selfdrive/debug/uiview.py

# Log UI events
python selfdrive/debug/uiview.py --log ui_debug.log

# Filter specific UI elements
python selfdrive/debug/uiview.py --filter alerts,navigation
```

---

## System Monitoring Tools

### 1. Lag Checker (`check_lag.py`)

**Purpose**: Detect and analyze system lag and delays

**Usage**:
```bash
# Check overall system lag
python selfdrive/debug/check_lag.py

# Monitor specific services for lag
python selfdrive/debug/check_lag.py --services controlsState,carState

# Set lag threshold
python selfdrive/debug/check_lag.py --threshold 50
```

### 2. Alert Cycler (`cycle_alerts.py`)

**Purpose**: Test and validate alert system functionality

**Usage**:
```bash
# Cycle through all alerts
python selfdrive/debug/cycle_alerts.py

# Test specific alert categories
python selfdrive/debug/cycle_alerts.py --category safety,warning

# Test alert timing
python selfdrive/debug/cycle_alerts.py --timing
```

### 3. Message Filter (`filter_log_message.py`)

**Purpose**: Filter and analyze specific log messages

**Usage**:
```bash
# Filter by message type
python selfdrive/debug/filter_log_message.py --type error

# Filter by service
python selfdrive/debug/filter_log_message.py --service controlsd

# Filter by time range
python selfdrive/debug/filter_log_message.py --start-time 1234567890 --end-time 1234567900
```

---

## Advanced Analysis Tools

### 1. Firmware Fingerprinting (`debug_fw_fingerprinting_offline.py`)

**Purpose**: Advanced firmware analysis and fingerprinting

**Usage**:
```bash
# Analyze firmware fingerprints
python selfdrive/debug/debug_fw_fingerprinting_offline.py --route "route_id"

# Compare fingerprints
python selfdrive/debug/debug_fw_fingerprinting_offline.py --compare route1 route2

# Generate fingerprint database
python selfdrive/debug/debug_fw_fingerprinting_offline.py --generate-db
```

### 2. Max Lateral Acceleration (`max_lat_accel.py`)

**Purpose**: Analyze maximum lateral acceleration capabilities

**Usage**:
```bash
# Analyze route for max lateral acceleration
python selfdrive/debug/max_lat_accel.py --route "route_id"

# Set speed thresholds
python selfdrive/debug/max_lat_accel.py --route "route_id" --min-speed 20 --max-speed 80

# Export results
python selfdrive/debug/max_lat_accel.py --route "route_id" --export results.csv
```

### 3. Torque Response Analysis (`measure_torque_time_to_max.py`)

**Purpose**: Measure torque response timing and characteristics

**Usage**:
```bash
# Measure torque response
python selfdrive/debug/measure_torque_time_to_max.py --route "route_id"

# Analyze specific maneuvers
python selfdrive/debug/measure_torque_time_to_max.py --route "route_id" --maneuver lane_change

# Generate response curves
python selfdrive/debug/measure_torque_time_to_max.py --route "route_id" --plot-curves
```

### 4. Fingerprint Analysis (`fingerprint_from_route.py`)

**Purpose**: Extract fingerprints from route data

**Usage**:
```bash
# Extract fingerprint from route
python selfdrive/debug/fingerprint_from_route.py --route "route_id"

# Format fingerprint output
python selfdrive/debug/format_fingerprints.py --input extracted_fingerprint.json
```

---

## Integration with Other Systems

### WiFi Setup for Remote Debugging

**Setup WiFi Access**:
```bash
# Configure WiFi hotspot
cd /data/openpilot
./scripts/setup_wifi_hotspot.sh

# Connect to external WiFi
./scripts/wifi_connect.sh SSID PASSWORD

# Enable SSH access
./scripts/setup_ssh_keys.py
```

**Remote Debugging Session**:
```bash
# SSH into device
ssh root@192.168.43.1

# Port forwarding for services
ssh -L 8082:localhost:8082 root@192.168.43.1

# Remote CAN monitoring
python selfdrive/debug/can_printer.py --addr 192.168.43.1
```

### Integration with Cabana

**Launch Cabana for CAN Analysis**:
```bash
# Start Cabana with live data
cd /data/openpilot/tools/cabana
./cabana

# Load specific route
./cabana --route "route_id"

# Load DBC files
./cabana --dbc toyota_prius_2017.dbc
```

### Integration with PlotJuggler

**Launch PlotJuggler**:
```bash
# Install PlotJuggler
cd /data/openpilot/tools/plotjuggler
./install.sh

# Start with debugging layout
python juggle.py --layout controls_mismatch_debug.xml

# Load route data
python juggle.py --route "route_id" --layout CAN-bus-debug.xml
```

---

## Performance Optimization

### CPU Optimization

**Identify CPU Bottlenecks**:
```bash
# Monitor CPU usage patterns
python selfdrive/debug/cpu_usage_stat.py --detailed_times

# Profile specific processes
python -m cProfile -o profile.stats your_process.py
```

**Memory Optimization**:
```bash
# Monitor memory usage
python selfdrive/debug/memory_usage.py

# Check for memory leaks
valgrind --tool=memcheck python your_process.py
```

### Timing Optimization

**Optimize Message Timing**:
```bash
# Analyze timing patterns
python selfdrive/debug/check_timings.py controlsState carState

# Identify timing outliers
python selfdrive/debug/timing_analysis.py --outliers
```

### CAN Bus Optimization

**Optimize CAN Performance**:
```bash
# Check CAN parser performance
python selfdrive/debug/check_can_parser_performance.py

# Optimize DBC file usage
python selfdrive/debug/optimize_dbc.py
```

---

## Troubleshooting Guide

### Common Issues and Solutions

#### 1. High CPU Usage

**Symptoms**: System lag, high temperature, slow response

**Diagnosis**:
```bash
# Monitor CPU usage
python selfdrive/debug/cpu_usage_stat.py

# Check process priorities
top -p $(pgrep -d',' python)
```

**Solutions**:
- Reduce process priority: `nice -n 10 python process.py`
- Optimize algorithms in high-usage processes
- Enable CPU frequency scaling

#### 2. CAN Message Issues

**Symptoms**: Missing messages, incorrect frequencies, parsing errors

**Diagnosis**:
```bash
# Check CAN bus activity
python selfdrive/debug/can_printer.py

# Verify message frequencies
python selfdrive/debug/check_freq.py can
```

**Solutions**:
- Check physical CAN connections
- Verify DBC file accuracy
- Update message parsing logic

#### 3. Timing Issues

**Symptoms**: Jerky control, delayed responses, timing warnings

**Diagnosis**:
```bash
# Monitor timing patterns
python selfdrive/debug/check_timings.py controlsState

# Check for system lag
python selfdrive/debug/check_lag.py
```

**Solutions**:
- Optimize process scheduling
- Reduce computational load
- Check hardware performance

#### 4. Memory Issues

**Symptoms**: System crashes, out-of-memory errors, slow performance

**Diagnosis**:
```bash
# Monitor memory usage
free -h
ps aux --sort=-%mem

# Check for memory leaks
python selfdrive/debug/memory_monitor.py
```

**Solutions**:
- Optimize memory usage in processes
- Increase swap space
- Fix memory leaks

#### 5. UI/Touch Issues

**Symptoms**: Unresponsive touch, UI lag, display problems

**Diagnosis**:
```bash
# Analyze touch patterns
python selfdrive/debug/touch_replay.py --route current

# Monitor UI state
python selfdrive/debug/uiview.py
```

**Solutions**:
- Calibrate touch screen
- Update UI rendering logic
- Check display hardware

---

## Complete Tool Reference

### Core Debugging Tools

| Tool | Location | Purpose | Key Features |
|------|----------|---------|--------------|
| `can_printer.py` | `selfdrive/debug/` | Real-time CAN monitoring | Live display, frequency analysis, ASCII decode |
| `can_table.py` | `selfdrive/debug/` | Tabular CAN analysis | Statistical analysis, filtering, CSV export |
| `dump.py` | `selfdrive/debug/` | Data logging and export | Multiple formats, value filtering, piping |
| `cpu_usage_stat.py` | `selfdrive/debug/` | CPU performance monitoring | Process tracking, long-term stats, detailed breakdown |
| `check_timings.py` | `selfdrive/debug/` | Timing analysis | Real-time stats, summary reporting, service monitoring |

### Vehicle Integration Tools

| Tool | Location | Purpose | Key Features |
|------|----------|---------|--------------|
| `get_fingerprint.py` | `selfdrive/debug/` | Vehicle fingerprinting | CAN message capture, frequency analysis |
| `fw_versions.py` | `selfdrive/debug/car/` | Firmware version query | ECU identification, version tracking |
| `vin.py` | `selfdrive/debug/car/` | VIN extraction | Vehicle identification, decoding, verification |
| `read_dtc_status.py` | `selfdrive/debug/` | Diagnostic codes | DTC reading, clearing, ECU communication |

### Analysis and Monitoring Tools

| Tool | Location | Purpose | Key Features |
|------|----------|---------|--------------|
| `touch_replay.py` | `selfdrive/debug/` | Touch analysis | Visualization, gesture tracking, heat maps |
| `live_cpu_and_temp.py` | `selfdrive/debug/` | System health monitoring | Temperature tracking, CPU monitoring, logging |
| `check_lag.py` | `selfdrive/debug/` | Lag detection | Delay measurement, threshold alerts |
| `qlog_size.py` | `selfdrive/debug/` | Log analysis | Size tracking, storage optimization |

### Advanced Tools

| Tool | Location | Purpose | Key Features |
|------|----------|---------|--------------|
| `max_lat_accel.py` | `selfdrive/debug/` | Performance analysis | Acceleration limits, route analysis |
| `measure_torque_time_to_max.py` | `selfdrive/debug/` | Torque response | Response timing, curve generation |
| `debug_fw_fingerprinting_offline.py` | `selfdrive/debug/` | Firmware analysis | Fingerprint comparison, database generation |

### Hardware Tools

| Tool | Location | Purpose | Key Features |
|------|----------|---------|--------------|
| `debug_console.py` | `panda/scripts/` | Panda debugging | Hardware communication, low-level debugging |
| `som_debug.sh` | `panda/scripts/` | SOM debugging | System-on-module diagnostics |

### Configuration Files

| File | Purpose |
|------|---------|
| `services.py` | Service definitions and socket configurations |
| `process_config.py` | Process management and priority settings |
| DBC files | CAN message definitions for various vehicles |

---

## Best Practices

### Development Workflow

1. **Start with Basic Tools**: Use `can_printer.py` and `dump.py` for initial analysis
2. **Use Specific Tools**: Move to specialized tools based on findings
3. **Document Results**: Save outputs and analysis for future reference
4. **Test Systematically**: Use structured testing approaches
5. **Validate Changes**: Always verify changes with appropriate tools

### Performance Monitoring

1. **Regular Health Checks**: Monitor CPU, memory, and timing regularly
2. **Baseline Establishment**: Create performance baselines for comparison
3. **Trend Analysis**: Track performance over time
4. **Proactive Monitoring**: Set up automated monitoring for critical metrics

### Data Analysis

1. **Comprehensive Logging**: Use multiple tools for complete data capture
2. **Structured Analysis**: Follow systematic analysis procedures
3. **Cross-Validation**: Verify findings with multiple tools
4. **Documentation**: Maintain detailed analysis records

---

## Conclusion

The NagasPilot Debugging Tools Collection provides a comprehensive suite of utilities for development, analysis, and troubleshooting. These tools are essential for:

- **Development**: Creating and testing new features
- **Integration**: Adding support for new vehicles
- **Optimization**: Improving system performance
- **Troubleshooting**: Diagnosing and fixing issues
- **Analysis**: Understanding system behavior and performance

Regular use of these tools ensures system reliability, performance, and maintainability. The combination of real-time monitoring, historical analysis, and specialized diagnostic tools provides complete visibility into system operation.

For optimal results, use these tools as part of a structured development and maintenance workflow, combining multiple tools for comprehensive analysis and validation.