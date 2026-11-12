"""System-wide constants, paths, and sensor/algorithm thresholds."""
from .config import (
    ROOT, DATA, SIMULATOR, local_path, configure_runtime,
    HUMAN_ID, ANIMAL_ID, ANIMAL_IR_VALUE, ANIMAL_DISPLAY_VALUE,
    HUMAN_TOKENS, EXCLUDED_TOKENS, ANIMAL_PATTERN, target_kind,
    DetectorConfig, FusionConfig, RuntimeConfig, MappingConfig,
    DETECTOR, FUSION, RUNTIME, MAPPING,
)

__all__ = [
    "ROOT", "DATA", "SIMULATOR", "local_path", "configure_runtime",
    "HUMAN_ID", "ANIMAL_ID", "ANIMAL_IR_VALUE", "ANIMAL_DISPLAY_VALUE",
    "HUMAN_TOKENS", "EXCLUDED_TOKENS", "ANIMAL_PATTERN", "target_kind",
    "DetectorConfig", "FusionConfig", "RuntimeConfig", "MappingConfig",
    "DETECTOR", "FUSION", "RUNTIME", "MAPPING",
]
