"""RViz 2D text overlay of the pallet state (rviz_2d_overlay_plugins).

In:  /pallet/status (PalletStatus, transient local), /packing/status (PackingState)
Out: /pallet/overlay (rviz_2d_overlay_msgs/OverlayText): state, SKU, boxes placed / pattern
     count, layer, fill %, reason.
"""

import rclpy
from palletizer_interfaces.msg import PackingState, PalletStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
COLORS = {'EMPTY': (0.6, 0.8, 1.0), 'IN_PROGRESS': (0.4, 1.0, 0.5), 'FULL': (1.0, 0.8, 0.2),
          'EXCHANGING': (1.0, 0.5, 0.2), 'READY': (0.6, 0.8, 1.0)}


class StatusOverlay(Node):

    def __init__(self):
        super().__init__('status_overlay')
        self.status = None
        self.packing = None
        self.pub = self.create_publisher(OverlayText, '/pallet/overlay', LATCHED)
        self.create_subscription(PalletStatus, '/pallet/status', self.on_status, LATCHED)
        self.create_subscription(PackingState, '/packing/status', self.on_packing, LATCHED)
        self.create_timer(1.0, self.publish)

    def on_status(self, m):
        self.status = m
        self.publish()

    def on_packing(self, m):
        self.packing = m

    def publish(self):
        s = self.status
        t = OverlayText()
        t.action = OverlayText.ADD
        t.width, t.height = 420, 190
        t.horizontal_alignment, t.vertical_alignment = OverlayText.LEFT, OverlayText.TOP
        t.horizontal_distance, t.vertical_distance = 10, 10
        t.text_size = 12.0
        t.line_width = 2
        t.font = 'DejaVu Sans Mono'
        t.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.55)
        if s is None:
            t.text = 'pallet: waiting for the orchestrator'
            t.fg_color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=1.0)
            self.pub.publish(t)
            return
        c = COLORS.get(s.state_name, (1.0, 1.0, 1.0))
        t.fg_color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=1.0)
        sku = (f'{s.sku_length * 1000:.0f} x {s.sku_width * 1000:.0f} x '
               f'{s.sku_height * 1000:.0f} mm' if s.sku_length > 0 else '-')
        pallet = (f'{s.pallet_length * 1000:.0f} x {s.pallet_width * 1000:.0f} mm'
                  if s.pallet_length > 0 else '-')
        boxes = f'{s.boxes_placed} / {s.boxes_in_pattern}' if s.boxes_in_pattern else \
            f'{s.boxes_placed}'
        layer = f'{s.layer} / {s.layers}' if s.layers else '-'
        lines = [f'PALLET {s.pallet_id}   {s.state_name}   ({s.mode})',
                 f'pallet  {pallet} (measured)',
                 f'SKU     {sku}',
                 f'boxes   {boxes}',
                 f'layer   {layer}',
                 f'fill    {s.fill_percent:.1f} %',
                 f'events  {s.shift_events} shift/collapse']
        if s.reason:
            lines.append(f'reason  {s.reason}')
        t.text = '\n'.join(lines)
        self.pub.publish(t)


def main():
    rclpy.init()
    rclpy.spin(StatusOverlay())
    return 0


if __name__ == '__main__':
    main()
