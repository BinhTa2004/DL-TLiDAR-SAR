"""Stage 3a: Thermal Processing Branch — Radiometric Synthesis and YOLO26m Detection (Report Section 3.2.3)."""
import time
import cv2
import numpy as np
from configs.config import DETECTOR, FUSION, HUMAN_ID, ANIMAL_IR_VALUE, ANIMAL_DISPLAY_VALUE

SEARCH_CLASS_IDS = DETECTOR.class_ids

_CLAHE_DDE = cv2.createCLAHE(clipLimit=2.2, tileGridSize=(8, 8))
_CLAHE_HUMAN = cv2.createCLAHE(clipLimit=2.4, tileGridSize=(4, 4))


def synthesize_radiometric_flir(im_ir, im_rgb):
    """Report Section 3.5.1: deterministic FLIR radiometric thermal synthesis."""
    if im_ir is None:
        return None
    th_seg = im_ir[:, :, 0] if im_ir.ndim == 3 else im_ir
    h, w = th_seg.shape[:2]
    if h == 0 or w == 0:
        return None

    if im_rgb is not None and getattr(im_rgb, "size", 0) > 0 and im_rgb.ndim >= 2:
        if im_rgb.shape[0] != h or im_rgb.shape[1] != w:
            rgb = cv2.resize(im_rgb, (w, h), interpolation=cv2.INTER_LINEAR)
        else:
            rgb = im_rgb

        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)

        # 1. Bo can bang thich nghi Dynamic Detail Enhancement (DDE / CLAHE Plateau Equalization)
        # Giu nguyen ven chi tiet nui non hung vi, tuyet trang, vach da va tan la (Khong cat den gia tao)
        gray_dde = _CLAHE_DDE.apply(gray)
        th_scene = cv2.convertScaleAbs(gray_dde, alpha=0.46, beta=32)

        # 2. Nang luong nhiet thuc vat (Foliage Warmth Boost) theo quang pho la cay tu nhien
        rb_half = cv2.addWeighted(rgb[:, :, 0], 0.5, rgb[:, :, 2], 0.5, 0)
        veg = cv2.subtract(rgb[:, :, 1], rb_half)
        veg_boost = cv2.convertScaleAbs(veg, alpha=0.32, beta=0)
        th_ambient = cv2.add(th_scene, veg_boost)
    else:
        th_ambient = cv2.convertScaleAbs(th_seg, alpha=2.0, beta=28)
        gray = None

    # 3. Nguon nhiet co the nguoi White-Hot voi chi tiet giai phau ro ret (dau, than, quan ao, chan tay)
    # IR bytes are gamma encoded. 243 is buffalo, 255 is human.
    animal_mask = (th_seg >= ANIMAL_IR_VALUE - 1) & (th_seg <= ANIMAL_IR_VALUE + 1)
    human_mask = (th_seg >= HUMAN_ID - 1).astype(np.uint8) * 255
    if human_mask.any():
        coords = cv2.findNonZero(human_mask)
        if coords is not None and gray is not None:
            bx, by, bw, bh = cv2.boundingRect(coords)
            x1 = max(0, bx - 2)
            y1 = max(0, by - 2)
            x2 = min(w, bx + bw + 2)
            y2 = min(h, by + bh + 2)

            sub_mask = human_mask[y1:y2, x1:x2]
            sub_rgb = rgb[y1:y2, x1:x2]
            sub_gray = gray[y1:y2, x1:x2]
            sub_amb = th_ambient[y1:y2, x1:x2]

            # Tang cuong chi tiet nep gap quan ao va cau truc giai phau cuc bo bang CLAHE
            sub_clahe = _CLAHE_HUMAN.apply(sub_gray)
            heat = cv2.convertScaleAbs(sub_clahe, alpha=0.10, beta=235).astype(np.float32)

            # Dan truyen nhiet loi co the (Torso Core Heat): nguc/lung am hon cac chi
            dt = cv2.distanceTransform(sub_mask, cv2.DIST_L2, 3)
            max_d = dt.max()
            if max_d > 0:
                heat += (dt / max_d) * 12.0

            # Nhan dien vung da ho (Mat, co, ban tay, canh tay) de dat dinh White-Hot ~37.0C - 38.5C
            r_ch = sub_rgb[:, :, 0]
            g_ch = sub_rgb[:, :, 1]
            sub_skin = (r_ch > 60) & (g_ch > 40) & (r_ch > g_ch) & ((r_ch.astype(np.int16) - g_ch.astype(np.int16)) >= 6) & (sub_mask > 0)
            if sub_skin.any():
                heat[sub_skin] += 15.0

            sub_heat = np.clip(heat, 0, 255).astype(np.uint8)

            # Vien mem 1px chong rang cua quang hoc (khop 1:1, khong tran vien ra dat da)
            sub_alpha = cv2.GaussianBlur(human_mask[y1:y2, x1:x2], (3, 3), 0.6).astype(np.float32) / 255.0
            th_combined = th_ambient.copy()
            th_combined[y1:y2, x1:x2] = (sub_alpha * sub_heat + (1.0 - sub_alpha) * sub_amb).astype(np.uint8)
        else:
            alpha = cv2.GaussianBlur(human_mask, (3, 3), 0.6).astype(np.float32) / 255.0
            body_heat = np.full_like(th_ambient, 252)
            th_combined = (alpha * body_heat + (1.0 - alpha) * th_ambient).astype(np.uint8)
    else:
        th_combined = th_ambient

    # 4. Tan xa quang hoc thau kinh Germanium (Optical PSF / MTF)
    th_psf = cv2.GaussianBlur(th_combined, (3, 3), 0.65)

    # The hottest pixels must follow the IR segmentation mask exactly.
    # Keep the soft optical edge outside the mask, without frame-varying noise.
    th_psf[human_mask == 0] = np.minimum(th_psf[human_mask == 0], 244)
    th_psf[human_mask != 0] = 255
    # Preserve the original environment and human rendering.
    # Animals remain visibly hot without entering the human fallback threshold.
    th_psf[animal_mask] = ANIMAL_DISPLAY_VALUE
    return th_psf


def temperature_from_intensity(value):
    """Report Section 3.5.1: map 8-bit radiometric intensity to temperature in Celsius."""
    return FUSION.intensity_base_c + (float(value) / FUSION.intensity_max) * FUSION.intensity_span_c


def detect_thermal_targets(th_gray, infrared_frame=None, *, model, enabled, device, imgsz, confidence, person_threshold, previous_fps):
    """Report Section 3.2.3: run YOLO26m thermal inference and produce 2D ROIs D = {(b_i, s_i)}.

    Returns (detections, smoothed_fps), independent of GUI or simulator modules.
    """
    yolo_fps = previous_fps
    if th_gray is None or getattr(th_gray, "size", 0) == 0:
        return [], yolo_fps
    h_im, w_im = th_gray.shape[:2]
    dets = []

    # 1. ƯU TIÊN SỬ DỤNG MÔ HÌNH AI YOLO26M
    if enabled and model is not None:
        try:
            t0 = time.perf_counter()
            if th_gray.ndim == 2:
                img_in = cv2.cvtColor(th_gray, cv2.COLOR_GRAY2RGB)
            else:
                img_in = th_gray

            results = model(img_in, device=device, imgsz=imgsz,
                                      conf=confidence, classes=list(SEARCH_CLASS_IDS),
                                      verbose=False)[0]
            boxes = results.boxes
            if boxes is not None and len(boxes) > 0:
                for b in boxes:
                    class_id = int(b.cls[0])
                    if class_id not in SEARCH_CLASS_IDS:
                        continue
                    x0, y0, x1, y1 = [int(v) for v in b.xyxy[0].tolist()]
                    conf = float(b.conf[0])
                    x0 = max(0, min(w_im - 1, x0))
                    y0 = max(0, min(h_im - 1, y0))
                    x1 = max(0, min(w_im - 1, x1))
                    y1 = max(0, min(h_im - 1, y1))
                    bw, bh = max(1, x1 - x0), max(1, y1 - y0)
                    cx = (x0 + x1) / 2.0
                    cy = (y0 + y1) / 2.0
                    area = bw * bh
                    crop = th_gray[y0:y1, x0:x1]
                    peak = float(crop.max()) if crop.size > 0 else float(th_gray[int(cy), int(cx)])
                    if infrared_frame is not None and class_id == 0 and peak < person_threshold:
                        # In this calibrated synthetic camera human heat is >=245.
                        # A YOLO person prediction on a cooler animal silhouette
                        # must not become a rescue waypoint.
                        continue
                    temp = temperature_from_intensity(peak)
                    dets.append({
                        "bbox": (x0, y0, x1, y1),
                        "cx": cx,
                        "cy": cy,
                        "area": area,
                        "temp": temp,
                        "conf": conf,
                        "is_yolo": True,
                        "class_id": class_id,
                    })
            lat = max(1e-4, time.perf_counter() - t0)
            yolo_fps = yolo_fps * 0.8 + (1.0 / lat) * 0.2
            return sorted(dets, key=lambda d: -d["conf"])[:DETECTOR.max_regions], yolo_fps
        except Exception:
            dets = []

    # 2. Connected-component thermal candidate extraction when YOLO model is disabled or unavailable.
    mask = (th_gray >= person_threshold).astype(np.uint8)
    if infrared_frame is not None and infrared_frame.shape[:2] == th_gray.shape[:2]:
        raw = infrared_frame[:, :, 0] if infrared_frame.ndim == 3 else infrared_frame
        mask &= (raw >= HUMAN_ID - 1).astype(np.uint8)
    if float(mask.mean()) > DETECTOR.fallback_max_image_fraction:
        return sorted(dets, key=lambda d: -d["conf"])[:DETECTOR.max_regions], yolo_fps
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    for i in range(1, n):
        x, y, bw, bh, area = [int(v) for v in stats[i]]
        if area < DETECTOR.fallback_min_area or area > DETECTOR.fallback_max_area:
            continue
        pad = DETECTOR.fallback_padding
        peak = float(th_gray[y:y + bh, x:x + bw].max())
        dets.append({
            "bbox": (max(0, x - pad), max(0, y - pad), min(w_im - 1, x + bw + pad), min(h_im - 1, y + bh + pad)),
            "cx": float(cents[i][0]), "cy": float(cents[i][1]),
            "area": area, "temp": temperature_from_intensity(peak),
            "conf": DETECTOR.fallback_confidence,
            "is_yolo": False,
            "class_id": None,
        })
    dets.sort(key=lambda d: (not d["is_yolo"], -d["conf"], -d["area"]))
    return dets[:DETECTOR.max_regions], yolo_fps
