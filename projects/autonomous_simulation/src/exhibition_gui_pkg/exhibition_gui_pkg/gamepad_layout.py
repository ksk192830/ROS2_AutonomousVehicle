"""Normalize Linux joystick axes to the exhibit's established input contract.

Game axes: LX, LY, RT, RX, RY, LT, hat X, hat Y. Both released
triggers are +1; fully pressed is -1. F310/XInput and Bluetooth Xbox
use different axis orders and trigger polarities.
"""
import array
import fcntl


class GamepadLayout:
    def __init__(self, axis_codes, button_codes):
        self.axis_codes = list(axis_codes)
        self.button_codes = list(button_codes)
        self.bluetooth_triggers = 9 in axis_codes and 10 in axis_codes
        rt, lt = (9, 10) if self.bluetooth_triggers else (5, 2)
        rx, ry = (2, 5) if self.bluetooth_triggers else (3, 4)
        self.axis_indices = [self.axis_codes.index(code) if code in self.axis_codes else None
                             for code in (0, 1, rt, rx, ry, lt, 16, 17)]
        self.required = {self.axis_indices[i] for i in (0, 2, 5)}
        if None in self.required:
            raise ValueError('핸들과 양쪽 트리거가 있는 게임패드가 필요합니다')
        # Linux xpad reports the physical X/Y as BTN_NORTH/BTN_WEST.
        self.button_indices = [self.button_codes.index(code) if code in self.button_codes else None
                               for code in (304,305,307,308,310,311,314,315,316,317,318)]

    @classmethod
    def read(cls, device):
        axis_count = array.array('B', [0]); button_count = array.array('B', [0])
        fcntl.ioctl(device, 0x80016a11, axis_count)
        fcntl.ioctl(device, 0x80016a12, button_count)
        axes = array.array('B', [0]*64); buttons = array.array('H', [0]*200)
        fcntl.ioctl(device, 0x80406a32, axes); fcntl.ioctl(device, 0x81906a34, buttons)
        return cls(axes[:axis_count[0]], buttons[:button_count[0]])

    def normalize(self, axes, buttons):
        values = [axes[i] if i is not None and i < len(axes) else 0.0 for i in self.axis_indices]
        if not self.bluetooth_triggers:
            values[2] *= -1
            values[5] *= -1
        keys = [buttons[i] if i is not None and i < len(buttons) else 0 for i in self.button_indices]
        return values, keys
