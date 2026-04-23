#!/usr/bin/env python3

import subprocess
import rclpy
from rclpy.node import Node

try:
    import Adafruit_SSD1306
    import PIL.Image
    import PIL.ImageFont
    import PIL.ImageDraw
    DISPLAY_AVAILABLE = True
except ImportError:
    DISPLAY_AVAILABLE = False


def _get_ip(interface):
    try:
        with open(f'/sys/class/net/{interface}/operstate', 'r') as f:
            if f.read().strip() != 'up':
                return None
    except Exception:
        return None
    try:
        cmd = (
            "ifconfig %s | grep -Eo 'inet (addr:)?([0-9]*\\.){3}[0-9]*'"
            " | grep -Eo '([0-9]*\\.){3}[0-9]*' | grep -v '127.0.0.1'" % interface
        )
        return subprocess.check_output(cmd, shell=True).decode('ascii').strip()
    except Exception:
        return None


def _cpu_usage():
    try:
        raw = subprocess.check_output(
            "top -bn1 | grep load | awk '{printf \"%.2f\", $(NF-2)}'",
            shell=True).decode()
        return int(round(float(raw) * 100.0))
    except Exception:
        return 0


def _memory_usage():
    try:
        raw = subprocess.check_output(
            "free -m | awk 'NR==2{printf \"%.2f\", $3*100/$2}'",
            shell=True).decode()
        return int(round(float(raw)))
    except Exception:
        return 0


def _disk_usage():
    try:
        raw = subprocess.check_output(
            "df -h | awk '$NF==\"/\"{printf \"%s\", $5}'",
            shell=True).decode().strip('%')
        return int(raw)
    except Exception:
        return 0


class DisplayNode(Node):

    def __init__(self):
        super().__init__('display_node')

        self.declare_parameter('agent_name', 'agent_0')
        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('i2c_bus', 1)

        self._agent_name = self.get_parameter('agent_name').value
        update_rate     = self.get_parameter('update_rate').value
        i2c_bus         = self.get_parameter('i2c_bus').value

        self._disp  = None
        self._image = None
        self._draw  = None
        self._font  = None

        if DISPLAY_AVAILABLE:
            try:
                self._disp = Adafruit_SSD1306.SSD1306_128_32(
                    rst=None, i2c_bus=i2c_bus, gpio=1)
                self._disp.begin()
                self._disp.clear()
                self._disp.display()
                self._font  = PIL.ImageFont.load_default()
                self._image = PIL.Image.new('1', (self._disp.width, self._disp.height))
                self._draw  = PIL.ImageDraw.Draw(self._image)
                self.get_logger().info('OLED display initialised (128x32 SSD1306)')
            except Exception as exc:
                self.get_logger().warn(f'Display init failed: {exc}')
                self._disp = None
        else:
            self.get_logger().warn(
                'Adafruit_SSD1306 not installed — display node running without hardware')

        self.create_timer(1.0 / update_rate, self._update)

    # ------------------------------------------------------------------
    def _update(self):
        if self._disp is None:
            return

        try:
            W, H = self._image.width, self._image.height
            self._draw.rectangle((0, 0, W, H), outline=0, fill=0)

            # Row 1: IP address (eth0 preferred, wlan0 fallback)
            ip  = _get_ip('eth0') or _get_ip('wlan0')
            self._draw.text((4, -2),
                            'IP: ' + (ip if ip else 'not available'),
                            font=self._font, fill=255)

            # Row 2: agent name
            self._draw.text((4, 6),
                            f'Agent: {self._agent_name}',
                            font=self._font, fill=255)

            # Row 3: stat headers
            for i, hdr in enumerate(['CPU', 'RAM', 'DSK']):
                self._draw.text((i * 40 + 4, 14), hdr, font=self._font, fill=255)

            # Row 4: stat values
            for i, val in enumerate([
                '%02d%%' % _cpu_usage(),
                '%02d%%' % _memory_usage(),
                '%02d%%' % _disk_usage(),
            ]):
                self._draw.text((i * 40 + 4, 22), val, font=self._font, fill=255)

            self._disp.image(self._image)
            self._disp.display()

        except Exception as exc:
            self.get_logger().warn(f'Display update error: {exc}')

    # ------------------------------------------------------------------
    def destroy_node(self):
        if self._disp is not None:
            try:
                self._draw.rectangle(
                    (0, 0, self._image.width, self._image.height), outline=0, fill=0)
                self._disp.image(self._image)
                self._disp.display()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
