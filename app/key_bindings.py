"""Live-map shortcuts use GLFW key codes, separate from global UAV controls."""

# GLFW reports letters as uppercase key codes, even without Shift.
MAP_KEYS = {
    "save_map": ord("P"),
    "toggle_markers": 292,       # F3
    "reset_view": 293,           # F4
    "change_colormap": 294,      # F5
    "clear_map": 295,            # F6 (press twice)
    "increase_point_size": 296,  # F7
    "decrease_point_size": 301,  # F12
    "toggle_follow": ord("F"),
}

# These are handled by the GCS. Suppress Open3D's built-in shortcuts so a
# flight key cannot also change, save or close the map. F8 clears markers.
UAV_GLFW_KEYS = (
    frozenset(map(ord, "WASDQEVYTKUIOGLMJRHCBXNZ123[]=+-_"))
    | frozenset(map(ord, "wasdqevytkuioglmjrhcbxnz"))
    | {
        8, 32, 127,                    # ASCII Backspace, Space, Delete
        256, 258, 259, 261,            # Escape, Tab, GLFW Backspace, GLFW Delete
        262, 263, 264, 265, 266, 267,  # Arrows, Page Up/Down
        290, 291, 297, 298, 299, 300,  # F1, F2, F8, F9, F10, F11
        321, 322, 323, 333, 334,       # Numpad 1/2/3, -, +
        340, 341, 342, 344, 345, 346,  # Left/right Shift, Ctrl, Alt
    }
)


def register_map_keys(vis, callbacks, ignore, exit_viewer):
    """Reject conflicting bindings instead of silently replacing callbacks."""
    keys = tuple(MAP_KEYS.values())
    if len(keys) != len(set(keys)) or set(keys) & UAV_GLFW_KEYS:
        raise ValueError("Map shortcuts conflict with another map or UAV control")
    for name, key in MAP_KEYS.items():
        vis.register_key_callback(key, callbacks[name])
        if ord("A") <= key <= ord("Z"):
            vis.register_key_callback(ord(chr(key).lower()), callbacks[name])
    for key in UAV_GLFW_KEYS - {256}:
        vis.register_key_callback(key, ignore)
    vis.register_key_callback(256, exit_viewer)

