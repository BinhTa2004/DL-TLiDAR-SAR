# -*- coding: utf-8 -*-
"""Assign synthetic IR intensities and verify actual actors in Medieval Village."""
import argparse
from datetime import datetime, timezone
import json
import math
import os
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from configs.config import DATA, ANIMAL_ID, ANIMAL_IR_VALUE, HUMAN_ID, target_kind
from app.runtime_support import connect_airsim, get_logger

log = get_logger("thermal_setup")

# Order matters: specific scenery overrides the generic static-mesh baseline.
SCENERY_GROUPS = (
    (44, ("sm_", "staticmesh", "house", "roof", "wood", "wall", "fence", "beam",
          "plank", "door", "mill", "building", "barn", "tower", "cart", "barrel", "prop")),
    (38, ("landscape", "cliff", "rock", "stone", "boulder", "ground", "terrain",
          "soil", "dirt", "path", "road", "cobble", "gravel")),
    (24, ("foliage", "spruce", "tree", "pine", "birch", "oak", "fir", "bush", "shrub",
          "plant", "branch", "leaf", "vegetation", "canopy")),
    (26, ("trunk",)),
    (34, ("grass", "mannequin", "nathan", "charactermesh", "fpcharacter",
          "thirdperson", "firstperson", "pawn")),
    (12, ("water", "river", "lake", "pond", "stream")),
    (40, ("flag", "cloth", "banner", "tarp", "rope", "scarecrow", "poseasset")),
)


def write_json_atomic(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(prefix=path.stem + "_", suffix=".json", dir=path.parent)
    try:
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        for attempt in range(10):
            try:
                os.replace(temporary, path)
                break
            except PermissionError:
                if attempt == 9:
                    Path(path).write_text(Path(temporary).read_text(encoding="utf-8"), encoding="utf-8")
                else:
                    time.sleep(0.02)
    finally:
        if os.path.exists(temporary):
            os.remove(temporary)


def calibrate_thermal_camera(client, scene_objects=None):
    """Set IDs, check setter results, then read back IDs using engine mesh names."""
    try:
        objects = sorted(set(scene_objects if scene_objects is not None
                             else client.simListSceneObjects(".*")))
        if not objects:
            raise RuntimeError("Ban do chua co doi tuong; thu lai sau khi tai xong")
        if not client.simSetSegmentationObjectID(".*", 34, True):
            raise RuntimeError("AirSim khong gan duoc IR baseline")
        # One RPC per scenery group instead of repeated overlapping regex calls.
        for value, tokens in SCENERY_GROUPS:
            client.simSetSegmentationObjectID(".*(" + "|".join(tokens) + ").*", value, True)

        humans, animals = [], []
        setter_results = {}
        for name in objects:
            kind = target_kind(name)
            if kind == "human":
                humans.append(name)
                setter_results[name] = bool(client.simSetSegmentationObjectID(name, HUMAN_ID, False))
            elif kind == "animal":
                animals.append(name)
                setter_results[name] = bool(client.simSetSegmentationObjectID(name, ANIMAL_ID, False))

        verified = {}
        unverified = []
        for name, expected in [(n, HUMAN_ID) for n in humans] + [(n, ANIMAL_ID) for n in animals]:
            actual = int(client.simGetSegmentationObjectID(name))
            ok = setter_results.get(name, False) and (
                actual == expected or (actual == -1 and setter_results.get(name, False))
            )
            verified[name] = {"expected": expected, "readback": actual,
                              "set_ok": setter_results.get(name, False), "verified": ok}
            if not ok:
                unverified.append(name)

        audit = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "human_id": HUMAN_ID, "animal_id": ANIMAL_ID, "animal_ir_byte": ANIMAL_IR_VALUE,
            "humans": humans, "animals": animals, "targets": verified,
            "unverified": unverified,
        }
        write_json_atomic(DATA / "thermal_audit.json", audit)
        if not humans:
            log.warning("[NHIET] Khong tim thay actor nguoi trong ban do.")
            return False
        if unverified:
            log.warning("[NHIET] %d doi tuong chua xac minh duoc ID; xem outputs/thermal_audit.json", len(unverified))
        log.info("[NHIET] Nguoi: %d | Dong vat: %d | ID da xac minh: %d/%d",
                 len(humans), len(animals), len(verified) - len(unverified), len(verified))
        return len(unverified) == 0
    except Exception as exc:
        log.error("[LOI] Khong gan duoc vat lieu nhiet: %s", exc)
        return False


def deploy_rescue_targets(client, scene_objects=None):
    """Record valid finite poses for actual human actors without teleporting them."""
    objects = sorted(set(scene_objects if scene_objects is not None
                         else client.simListSceneObjects(".*")))
    targets = []
    for name in objects:
        if target_kind(name) != "human":
            continue
        try:
            p = client.simGetObjectPose(name).position
            x, y, z = float(p.x_val), float(p.y_val), float(p.z_val)
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            targets.append({"id": len(targets) + 1, "name": name,
                            "x_ned": x, "y_ned": y, "z_ned": z,
                            "x_enu": y, "y_enu": x, "z_enu": -z})
        except Exception as exc:
            log.debug("Khong doc duoc pose %s: %s", name, exc)
    write_json_atomic(DATA / "ground_truth_targets.json", targets)
    if not targets:
        log.warning("[MUC TIEU] Chua lay duoc toa do nguoi hop le.")
    log.debug("[MUC TIEU] Da xuat %d toa do nguoi", len(targets))
    return targets


def setup_scene(client):
    objects = client.simListSceneObjects(".*")
    ok = calibrate_thermal_camera(client, objects)
    targets = deploy_rescue_targets(client, objects)
    return ok and bool(targets)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ip", nargs="?", default=os.getenv("AIRSIM_IP", "127.0.0.1"))
    args = parser.parse_args()
    client = None
    for _ in range(30):
        try:
            client = connect_airsim(args.ip, timeout=3)
            break
        except Exception as exc:
            log.debug("Cho AirSim: %s", exc)
            time.sleep(2)
    if client is None:
        log.error("[LOI] Khong ket noi duoc AirSim sau 30 lan thu.")
        return 1
    try:
        return 0 if setup_scene(client) else 1
    finally:
        client.client.close()


if __name__ == "__main__":
    sys.exit(main())
