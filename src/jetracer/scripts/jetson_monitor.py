#!/usr/bin/env python3
"""
Publishes Jetson Orin Nano hardware metrics as a flat Float64MultiArray
on /<agent_name>/jetson/metrics at a configurable rate.

Indices (METRIC_KEYS order):
  0  cpu0_pct        – per-core utilization 0-100
  1  cpu1_pct
  2  cpu2_pct
  3  cpu3_pct
  4  cpu4_pct
  5  cpu5_pct
  6  gpu_pct         – GPU busy 0-100
  7  ram_used_mb
  8  ram_total_mb
  9  cpu_freq_mhz    – cpu0 current frequency
  10 gpu_freq_mhz
  11 gpu_freq_max_mhz
  12 vdd_in_mw       – total board power (mW)
  13 vdd_cpu_gpu_cv_mw
  14 vdd_soc_mw
  15 cpu_temp_c
  16 gpu_temp_c
  17 tj_temp_c       – junction temp (throttle indicator)
"""

import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

METRIC_KEYS = [
    'cpu0_pct', 'cpu1_pct', 'cpu2_pct', 'cpu3_pct', 'cpu4_pct', 'cpu5_pct',
    'gpu_pct',
    'ram_used_mb', 'ram_total_mb',
    'cpu_freq_mhz', 'gpu_freq_mhz', 'gpu_freq_max_mhz',
    'vdd_in_mw', 'vdd_cpu_gpu_cv_mw', 'vdd_soc_mw',
    'cpu_temp_c', 'gpu_temp_c', 'tj_temp_c',
]

N_CPUS = 6
GPU_LOAD_PATH   = '/sys/devices/platform/bus@0/17000000.gpu/load'
GPU_FREQ_PATH   = '/sys/class/devfreq/17000000.gpu/cur_freq'
GPU_MAXFREQ_PATH= '/sys/class/devfreq/17000000.gpu/max_freq'
HWMON_BASE      = '/sys/bus/i2c/devices/1-0040/hwmon/hwmon1'
THERMAL_BASE    = '/sys/class/thermal'


def _read(path, default=0.0):
    try:
        with open(path) as f:
            return float(f.read().strip())
    except Exception:
        return default


def _cpu_jiffies():
    """Return list of (user+system+nice, total) per CPU from /proc/stat."""
    result = []
    with open('/proc/stat') as f:
        for line in f:
            if not line.startswith('cpu') or line.startswith('cpu '):
                continue
            parts = line.split()
            # user nice system idle iowait irq softirq steal guest guest_nice
            vals = [int(x) for x in parts[1:]]
            idle = vals[3] + vals[4]          # idle + iowait
            total = sum(vals)
            result.append((total - idle, total))
            if len(result) == N_CPUS:
                break
    return result


def _ram_mb():
    total, avail = 0.0, 0.0
    with open('/proc/meminfo') as f:
        for line in f:
            if line.startswith('MemTotal:'):
                total = int(line.split()[1]) / 1024.0
            elif line.startswith('MemAvailable:'):
                avail = int(line.split()[1]) / 1024.0
    return total - avail, total


def _thermal(zone_type):
    for z in os.listdir(THERMAL_BASE):
        if not z.startswith('thermal_zone'):
            continue
        t = f'{THERMAL_BASE}/{z}/type'
        v = f'{THERMAL_BASE}/{z}/temp'
        try:
            with open(t) as f:
                if f.read().strip() == zone_type:
                    return _read(v) / 1000.0   # millidegrees → C
        except Exception:
            pass
    return 0.0


def _power_mw(rail_idx):
    """rail_idx: 1=VDD_IN, 2=VDD_CPU_GPU_CV, 3=VDD_SOC"""
    v = _read(f'{HWMON_BASE}/in{rail_idx}_input')    # mV
    i = _read(f'{HWMON_BASE}/curr{rail_idx}_input')  # mA
    return v * i / 1000.0                            # mW


class JetsonMonitor(Node):
    def __init__(self):
        super().__init__('jetson_monitor')
        self.declare_parameter('agent_name', 'agent_0')
        self.declare_parameter('rate_hz', 2.0)

        agent = self.get_parameter('agent_name').get_parameter_value().string_value
        rate  = self.get_parameter('rate_hz').get_parameter_value().double_value

        topic = f'/{agent}/jetson/metrics'
        self._pub = self.create_publisher(Float64MultiArray, topic, 10)
        self._prev_jiffies = _cpu_jiffies()
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f'Publishing Jetson metrics on {topic} at {rate} Hz')

    def _publish(self):
        vals = [0.0] * len(METRIC_KEYS)

        # CPU utilization (delta jiffies between samples)
        cur = _cpu_jiffies()
        for i in range(min(N_CPUS, len(cur))):
            d_busy  = cur[i][0] - self._prev_jiffies[i][0]
            d_total = cur[i][1] - self._prev_jiffies[i][1]
            vals[i] = 100.0 * d_busy / d_total if d_total > 0 else 0.0
        self._prev_jiffies = cur

        # GPU (0-1000 → 0-100)
        vals[6] = _read(GPU_LOAD_PATH) / 10.0

        # RAM
        used, total = _ram_mb()
        vals[7] = used
        vals[8] = total

        # Frequencies
        vals[9]  = _read(f'/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq') / 1e3  # kHz→MHz
        vals[10] = _read(GPU_FREQ_PATH) / 1e6      # Hz→MHz
        vals[11] = _read(GPU_MAXFREQ_PATH) / 1e6

        # Power
        vals[12] = _power_mw(1)
        vals[13] = _power_mw(2)
        vals[14] = _power_mw(3)

        # Temperatures
        vals[15] = _thermal('cpu-thermal')
        vals[16] = _thermal('gpu-thermal')
        vals[17] = _thermal('tj-thermal')

        msg = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.label  = ','.join(METRIC_KEYS)
        dim.size   = len(vals)
        dim.stride = len(vals)
        msg.layout.dim.append(dim)
        msg.data = vals
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JetsonMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
