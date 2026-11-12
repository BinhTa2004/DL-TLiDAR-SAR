"""GCS layout and Windows display sizing, isolated from sensor algorithms."""
import ctypes
import os
user32 = ctypes.windll.user32

try:
    try: user32.SetProcessDPIAware()
    except Exception: pass
    SCREEN_W = user32.GetSystemMetrics(0)
    SCREEN_H = user32.GetSystemMetrics(1)
except Exception:
    SCREEN_W, SCREEN_H = 1920, 1080

CANVAS_W, CANVAS_H = 1600, 900
HEADER_H = 36
FOOTER_H = 28
DECK_Y = HEADER_H
DECK_H = CANVAS_H - HEADER_H - FOOTER_H          # 836
PANE_W = int(CANVAS_W * 0.40)                    # 640
PANE_H = DECK_H // 2                             # 418
SIDE_X = PANE_W * 2                              # 1280
SIDE_W = CANVAS_W - SIDE_X                       # 320
PANE_HDR = 30

_fit = min((SCREEN_W - 40) / CANVAS_W, (SCREEN_H - 100) / CANVAS_H, 1.0)
DEFAULT_GCS_W = min(1280, max(960, int(CANVAS_W * _fit)))
DEFAULT_GCS_H = int(DEFAULT_GCS_W * 9 / 16)

os.environ["SDL_VIDEO_CENTERED"] = "1"
