"""Portable camera/LiDAR launch defaults for the third SKKU contest."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import yaml


def load_hardware_settings():
    """Return the packaged device settings. No developer home paths required."""
    share_dir = Path(get_package_share_directory("camera_pkg"))
    with (share_dir / "config" / "hardware.yaml").open(encoding="utf-8") as stream:
        settings = yaml.safe_load(stream) or {}
    settings["model_path"] = str(share_dir / "model" / settings.get("model_filename", "final.pt"))
    settings["image_dir"] = str(Path(settings.get("image_dir", "~/ros2_av_captures/3rd_skku")).expanduser())
    return settings
