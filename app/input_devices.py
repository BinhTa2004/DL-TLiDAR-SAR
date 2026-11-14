"""Windows keyboard and XInput handling; unchanged button/dead-zone mappings."""
import ctypes
from ctypes import wintypes
user32 = ctypes.windll.user32


def is_key_down(vk_code):
    return bool(user32.GetAsyncKeyState(vk_code) & 0x8000)

VK_TAB    = 0x09
VK_SHIFT  = 0x10
VK_CTRL   = 0x11
VK_SPACE  = 0x20
VK_LEFT   = 0x25
VK_UP     = 0x26
VK_RIGHT  = 0x27
VK_DOWN   = 0x28
VK_PGUP   = 0x21
VK_PGDN   = 0x22
VK_0      = 0x30
VK_1      = 0x31
VK_2      = 0x32
VK_3      = 0x33
VK_NUMPAD1= 0x61
VK_NUMPAD2= 0x62
VK_NUMPAD3= 0x63
VK_W      = 0x57
VK_A      = 0x41
VK_S      = 0x53
VK_D      = 0x44
VK_Q      = 0x51
VK_E      = 0x45
VK_Y      = 0x59
VK_Z      = 0x5A  # Toggle gimbal stabilization [Z]
VK_B      = 0x42
VK_X      = 0x58
VK_C      = 0x43
VK_V      = 0x56
VK_R      = 0x52
VK_H      = 0x48
VK_T      = 0x54
VK_L      = 0x4C
VK_G      = 0x47
VK_I      = 0x49  # Invert Cam Pitch toggle
VK_J         = 0x4A  # Toggle LiDAR scan [J] / D-Pad Right
VK_U         = 0x55  # Toggle Thermal FLIR [U] / D-Pad Left
VK_K      = 0x4B  # Toggle Record Search Data [K]
VK_M      = 0x4D  # Toggle AI YOLO Detection [M]
VK_O      = 0x4F  # Toggle Bounding Box (Khung do nhan dien) [O]
VK_N      = 0x4E  # Toggle Radar Heading-Up / North-Up [N]
VK_F9     = 0x78  # Toggle Record Search Data [F9]
VK_F1     = 0x70
VK_F2     = 0x71  # Mo / Chuyen toi Cua so Dung Dia Hinh 3D Open3D
VK_F8     = 0x77  # Clear marked victims [F8], independent of window focus
VK_F10    = 0x79
VK_F11    = 0x7A
VK_LBRACKET = 0xDB
VK_RBRACKET = 0xDD
VK_OEM_PLUS  = 0xBB  # Phím '+' hoặc '='
VK_OEM_MINUS = 0xBD  # Phím '-' hoặc '_'
VK_ADD       = 0x6B  # Phím '+' trên Numpad
VK_SUBTRACT  = 0x6D  # Phím '-' trên Numpad

class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [
        ("wButtons", wintypes.WORD),
        ("bLeftTrigger", wintypes.BYTE),
        ("bRightTrigger", wintypes.BYTE),
        ("sThumbLX", wintypes.SHORT),
        ("sThumbLY", wintypes.SHORT),
        ("sThumbRX", wintypes.SHORT),
        ("sThumbRY", wintypes.SHORT),
    ]

class XINPUT_STATE(ctypes.Structure):
    _fields_ = [
        ("dwPacketNumber", wintypes.DWORD),
        ("Gamepad", XINPUT_GAMEPAD),
    ]

class XInputController:
    """Đọc tay cầm Xbox qua Windows XInput C-API (độ trễ 0ms, không phụ thuộc focus)."""
    def __init__(self):
        self.xinput = None
        self.connected = False
        for dll in ("xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"):
            try:
                self.xinput = ctypes.windll.LoadLibrary(dll)
                break
            except Exception:
                pass

    def poll(self):
        if not self.xinput:
            return None
        state = XINPUT_STATE()
        res = self.xinput.XInputGetState(0, ctypes.byref(state))
        if res == 0:
            self.connected = True
            gp = state.Gamepad
            deadzone_l = 7849
            deadzone_r = 8689
            lx = gp.sThumbLX / 32767.0
            ly = gp.sThumbLY / 32767.0
            rx = gp.sThumbRX / 32767.0
            ry = gp.sThumbRY / 32767.0
            if abs(gp.sThumbLX) < deadzone_l: lx = 0.0
            if abs(gp.sThumbLY) < deadzone_l: ly = 0.0
            if abs(gp.sThumbRX) < deadzone_r: rx = 0.0
            if abs(gp.sThumbRY) < deadzone_r: ry = 0.0
            lt = (gp.bLeftTrigger - 30) / 225.0 if gp.bLeftTrigger > 30 else 0.0
            rt = (gp.bRightTrigger - 30) / 225.0 if gp.bRightTrigger > 30 else 0.0
            btns = gp.wButtons
            return {
                "lx": 0.0 if abs(lx) < 0.12 else lx,
                "ly": 0.0 if abs(ly) < 0.12 else ly,
                "rx": 0.0 if abs(rx) < 0.12 else rx,
                "ry": 0.0 if abs(ry) < 0.12 else ry,
                "lt": lt,
                "rt": rt,
                "btn_a": bool(btns & 0x1000),
                "btn_b": bool(btns & 0x2000),
                "btn_x": bool(btns & 0x4000),
                "btn_y": bool(btns & 0x8000),
                "btn_lb": bool(btns & 0x0100),
                "btn_rb": bool(btns & 0x0200),
                "btn_back": bool(btns & 0x0020),
                "btn_start": bool(btns & 0x0010),
                "btn_l3": bool(btns & 0x0040),
                "btn_r3": bool(btns & 0x0080),
                "dpad_up": bool(btns & 0x0001),
                "dpad_down": bool(btns & 0x0002),
                "dpad_left": bool(btns & 0x0004),
                "dpad_right": bool(btns & 0x0008),
            }
        self.connected = False
        return None
