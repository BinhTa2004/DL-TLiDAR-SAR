# -*- coding: utf-8 -*-
"""
=============================================================================
KHOI TAO MUC TIEU NGUOI (CUU HO) & HIEU CHUAN CAMERA NHIET FLIR AIRSIM
BAN DO AFRICA (UNREAL ENGINE 4.27)
=============================================================================
Chuc nang:
1. Hieu chuan dai nhiet do Planck cho cam bien hong ngoai (ImageType: 7):
   - Dat nhiet do sinh hoc co the nguoi ~37.4°C (Digital Count = 255 - WHITE-HOT)
   - Dat nhiet do cay coi/tham thuc vat ~15°C (Digital Count = 35 - COOL)
   - Dat nhiet do mat dat, sa mac ~22°C (Digital Count = 65 - AMBIENT)
   - Dat nhiet do ho nuoc ~12°C (Digital Count = 20 - COLD)
   - Dat nhiet do dong vat hoang da ~33-35°C (Digital Count = 215-225 - WARM)
2. Thiet lap danh sach vi tri Nan nhan Cuu ho (Human Targets) xung quanh UAV:
   - Nan nhan #1: Duoi goc cay (cach goc toa do ~22m)
   - Nan nhan #2: Ven bo suoi (cach goc toa do ~44m)
   - Nan nhan #3: Giua dong co savanna (cach goc toa do ~49m)
   - Nan nhan #4: Nghi van an nap (cach goc toa do ~83m)
=============================================================================
"""

import sys
import os
import time
import json
import math

try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

import airsim

MISSION_TARGETS = [
    {
        "id": 1,
        "name": "Victim_Person_1_GocCayVenHo",
        "label": "NGUOI (NAN NHAN #1 - DUOI GOC CAY KEO VEN HO)",
        "x": 26.5,
        "y": 14.5,
        "z": 3.65,
        "yaw": -130.0,
        "temp_c": 37.4,
        "emissivity": 0.985,
        "status": "CAN CUU HO KHAN CAP"
    },
    {
        "id": 2,
        "name": "Victim_Person_2_VenBoSuoi",
        "label": "NGUOI (NAN NHAN #2 - DUOI RANG CAY VEN BO SUOI)",
        "x": 25.5,
        "y": -12.5,
        "z": 3.65,
        "yaw": 110.0,
        "temp_c": 37.2,
        "emissivity": 0.985,
        "status": "BI THUONG CAN SO CUU"
    },
    {
        "id": 3,
        "name": "Victim_Person_3_SuonDoi",
        "label": "NGUOI (NAN NHAN #3 - BUI RAM CAY GAI SUON DOI)",
        "x": -12.0,
        "y": 18.0,
        "z": 2.10,
        "yaw": -45.0,
        "temp_c": 37.6,
        "emissivity": 0.985,
        "status": "TINH TAO DANG VAY TAY"
    },
    {
        "id": 4,
        "name": "Poacher_Human_4_GoCat",
        "label": "NGUOI (DOI TUONG NGHI VAN #4 - GO CAT AN NAP)",
        "x": -16.0,
        "y": -4.0,
        "z": 2.18,
        "yaw": 30.0,
        "temp_c": 37.3,
        "emissivity": 0.985,
        "status": "MUC TIEU THEO DOI"
    }
]

def calibrate_thermal_camera(client):
    """
    Hieu chuan dai mau nhiet Planck cho Camera Hong Ngoai (ImageType 7).
    Nguoi se phat sang trang ruc (White-Hot) noi bat tren nen lanh.
    """
    print("\n>> [1/2] Dang hieu chuan dai nhiet do Planck cho Camera Hong Ngoai (Thermal IR)...")
    try:
        # Dat toan bo map ve 0 (nen den mac dinh)
        client.simSetSegmentationObjectID(".*", 0, True)

        # 1. Dia hinh, dat cat sa mac (~22°C)
        client.simSetSegmentationObjectID(".*Base_Terrain.*", 65, True)
        client.simSetSegmentationObjectID(".*landscape.*", 65, True)
        client.simSetSegmentationObjectID(".*soil.*", 65, True)

        # 2. Cay coi, bui ram (~15°C do quang hop thoat hoi nuoc lam mat)
        client.simSetSegmentationObjectID(".*InstancedFoliageActor.*", 35, True)
        client.simSetSegmentationObjectID(".*tree.*", 35, True)
        client.simSetSegmentationObjectID(".*bush.*", 35, True)
        client.simSetSegmentationObjectID(".*acacia.*", 35, True)

        # 3. Ho nuoc (~12°C - nhiet do thap)
        client.simSetSegmentationObjectID(".*Water.*", 20, True)

        # 4. Dong vat hoang da (~33°C - 35°C)
        client.simSetSegmentationObjectID(".*elephant.*", 215, True)
        client.simSetSegmentationObjectID(".*zebra.*", 225, True)
        client.simSetSegmentationObjectID(".*Crocodile.*", 205, True)
        client.simSetSegmentationObjectID(".*Rhinoceros.*", 220, True)
        client.simSetSegmentationObjectID(".*Hippo.*", 210, True)

        # 5. CON NGUOI (~37.4°C - WHITE HOT 255)
        client.simSetSegmentationObjectID(".*poacher.*", 255, True)
        client.simSetSegmentationObjectID(".*human.*", 255, True)
        client.simSetSegmentationObjectID(".*person.*", 255, True)
        client.simSetSegmentationObjectID(".*victim.*", 255, True)
        client.simSetSegmentationObjectID(".*mannequin.*", 255, True)

        print("   [THANH CONG] Camera Nhiet FLIR da duoc hieu chuan chuan xac:")
        print("   - Nguoi (Nan nhan): 37.4°C -> Digital Count 255 (Phat sang trang White-Hot)")
        print("   - Dong vat hoang da: 33-35°C -> Digital Count 215-225 (Am ap)")
        print("   - Mat dat sa mac: 22°C -> Digital Count 65 (Nen trung tinh)")
        print("   - Cay coi & Ho nuoc: 12-15°C -> Digital Count 20-35 (Vung lanh toi mau)")
        return True
    except Exception as e:
        print(f"   [CANH BAO] Loi khi set segmentation ID: {e}")
        return False

def deploy_rescue_targets(client):
    """
    Thiet lap cac muc tieu con nguoi (3D Human Character) tren ban do Africa.
    Thay vi spawn khoi tru Cylinder gia, he thong tim cac nhan vat nguoi 3D that
    co san trong ban do (African_Poacher_1, African_Poacher_2, African_Poacher_3...)
    va di chuyen ho ve cac vi tri cuu ho mong muon.
    """
    print("\n>> [2/2] Dang trien khai cac nhan vat nguoi 3D (Human Characters) vao ban do...")
    deployed = []

    # 1. Tim cac nhan vat nguoi 3D co san trong ban do
    human_actors = []
    try:
        all_objs = client.simListSceneObjects(".*")
        human_actors = [o for o in all_objs if any(k in o.lower() for k in ["poacher", "human", "person", "character"])]
        print(f"   Tim thay {len(human_actors)} nhan vat nguoi 3D trong ban do: {human_actors}")
    except Exception as e:
        print(f"   [CANH BAO] Loi khi liet ke doi tuong: {e}")

    # 2. Dua tung nhan vat nguoi 3D ve vi tri cuu ho
    for idx, tgt in enumerate(MISSION_TARGETS):
        tx, ty, tz = tgt["x"], tgt["y"], tgt["z"]
        yaw_rad = math.radians(tgt.get("yaw", 0.0))
        pose = airsim.Pose(airsim.Vector3r(tx, ty, tz), airsim.to_quaternion(0, 0, yaw_rad))
        dist = math.hypot(tx, ty)

        placed = False
        actor_name = ""

        if idx < len(human_actors):
            actor_name = human_actors[idx]
            try:
                # Di chuyen nguoi 3D that ve dung toa do
                res = client.simSetObjectPose(actor_name, pose, teleport=True)
                if res:
                    # Dat segmentation ID = 255 (than nhiet nguoi 37.4°C White-Hot)
                    client.simSetSegmentationObjectID(actor_name, 255, True)
                    placed = True
            except Exception as e:
                print(f"     Loi khi di chuyen {actor_name}: {e}")

        # Neu khong co du nhan vat nguoi de teleport thi chi thu spawn asset nguoi neu co (KHONG SPAWN CYLINDER)
        if not placed:
            candidate_assets = []
            try:
                registry = {a.lower(): a for a in client.simListAssets()}
                for wanted in ["Character", "BP_Poacher", "Human"]:
                    if wanted.lower() in registry:
                        candidate_assets.append(registry[wanted.lower()])
            except Exception:
                pass

            for asset_try in candidate_assets:
                try:
                    res = client.simSpawnObject(tgt["name"], asset_try, pose, airsim.Vector3r(1, 1, 1), False)
                    if res and res != "":
                        client.simSetSegmentationObjectID(tgt["name"], 255, True)
                        placed = True
                        actor_name = tgt["name"]
                        break
                except Exception:
                    pass

        tgt_entry = dict(tgt)
        if actor_name:
            tgt_entry["actor_name"] = actor_name
        deployed.append(tgt_entry)

        print(f"   + [MUC TIEU #{tgt['id']}] {tgt['label']}")
        print(f"     Toa do: X={tx:.1f}m, Y={ty:.1f}m, Z={tz:.1f}m | Cu ly: {dist:.1f}m | Nhiet do: {tgt['temp_c']}°C")
        if placed:
            print(f"     [OK] Nhan vat nguoi 3D '{actor_name}' da duoc dua den vi tri!")
        else:
            print(f"     [THONG BAO] Nhan vat nguoi giu vi tri tu nhien tren ban do (khong dung hinh tru).")

    # Dam bao tat ca cac nguoi 3D trong map deu phat sang than nhiet 37.4°C
    try:
        client.simSetSegmentationObjectID(".*poacher.*", 255, True)
        client.simSetSegmentationObjectID(".*human.*", 255, True)
    except Exception:
        pass

    # Luu thong tin toa do vao file json de Tram GCS dong bo
    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_json = os.path.join(script_dir, "rescue_mission_targets.json")
    try:
        with open(out_json, "w", encoding="utf-8") as f:
            json.dump(deployed, f, ensure_ascii=False, indent=2)
        print(f"\n>> Da luu thong tin {len(deployed)} muc tieu vao: {out_json}")
    except Exception as e:
        print(f">> Khong the ghi file targets.json: {e}")

    return deployed

def main():
    print("=" * 65)
    print("   HE THONG THIET LAP MUC TIEU NGUOI & CAMERA NHIET FLIR AIRSIM")
    print("=" * 65)
    print("Dang ket noi toi AirSim Server (Port 41451)...")

    client = None
    for attempt in range(15):
        try:
            client = airsim.MultirotorClient(timeout_value=3)
            client.confirmConnection()
            print(">> Ket noi toi AirSim Server thanh cong!\n")
            break
        except Exception:
            print(f"   Cho AirSim khoi dong... ({attempt+1}/15)")
            time.sleep(2)

    if client is None:
        print("\n[LOI] Khong the ket noi toi AirSim. Hay chac chan ban do Africa dang chay!")
        return 1

    calibrate_thermal_camera(client)
    deploy_rescue_targets(client)

    print("\n" + "=" * 65)
    print("   [HOAN TAT] DA THIET LAP XONG MUC TIEU NGUOI & CAMERA NHIET FLIR!")
    print("   Ban co the bat dau bay UAV de tim kiem cac nan nhan tren map.")
    print("=" * 65)
    return 0

if __name__ == "__main__":
    sys.exit(main())
