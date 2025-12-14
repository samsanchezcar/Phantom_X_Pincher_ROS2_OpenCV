import cv2
import numpy as np
import math

# Physical radius of the white disk (in cm)
DISK_RADIUS_CM = 7.25


def calcular_caracteristicas_geometricas(cnt):
    """
    Calcula múltiples características geométricas invariantes a rotación.
    Estas son más robustas que solo contar vértices.
    """
    area = cv2.contourArea(cnt)
    perimetro = cv2.arcLength(cnt, True)

    if area < 100 or perimetro < 10:
        return None

    circularidad = (4 * math.pi * area) / (perimetro * perimetro)

    hull = cv2.convexHull(cnt)
    area_hull = cv2.contourArea(hull)
    solidez = area / area_hull if area_hull > 0 else 0

    x, y, w, h = cv2.boundingRect(cnt)
    area_rect = w * h
    extension = area / area_rect if area_rect > 0 else 0
    aspect_ratio = w / h if h > 0 else 0

    if len(cnt) >= 5:
        ellipse = cv2.fitEllipse(cnt)
        (cx_e, cy_e), (eje_menor, eje_mayor), angle = ellipse
        elipse_ratio = eje_menor / eje_mayor if eje_mayor > 0 else 0
    else:
        elipse_ratio = aspect_ratio

    vertices_list = []
    for eps_factor in [0.01, 0.02, 0.03, 0.04]:
        approx = cv2.approxPolyDP(cnt, eps_factor * perimetro, True)
        vertices_list.append(len(approx))
    n_vertices = int(np.median(vertices_list))

    hull_indices = cv2.convexHull(cnt, returnPoints=False)
    if len(hull_indices) > 3:
        defects = cv2.convexityDefects(cnt, hull_indices)
        n_defectos = len(defects) if defects is not None else 0
    else:
        n_defectos = 0

    momentos = cv2.moments(cnt)
    hu_moments = cv2.HuMoments(momentos).flatten()
    hu_log = -np.sign(hu_moments) * np.log10(np.abs(hu_moments) + 1e-10)

    return {
        "area": area,
        "perimetro": perimetro,
        "circularidad": circularidad,
        "solidez": solidez,
        "extension": extension,
        "aspect_ratio": aspect_ratio,
        "elipse_ratio": elipse_ratio,
        "n_vertices": n_vertices,
        "vertices_list": vertices_list,
        "n_defectos": n_defectos,
        "hu_moments": hu_log,
        "contorno": cnt,
    }


def clasificar_forma(features, area_cm2=None):
    """
    Clasifica la forma en: circle, square, rectangle o pentagon.
    """
    if features is None:
        return "unknown", 0.0

    circ = features["circularidad"]
    solid = features["solidez"]
    aspect = features["aspect_ratio"]
    elipse = features["elipse_ratio"]
    n_vert = features["n_vertices"]
    vert_list = features["vertices_list"]

    vertices_4 = sum(1 for v in vert_list if v == 4)
    vertices_5 = sum(1 for v in vert_list if v == 5)
    vertices_5_6 = sum(1 for v in vert_list if v in [5, 6])
    vertices_6_plus = sum(1 for v in vert_list if v >= 6)

    scores = {}

    # circle
    score_circulo = 0
    if circ > 0.92:
        score_circulo += 50
    elif circ > 0.88:
        score_circulo += 30
    elif circ > 0.82:
        score_circulo += 10

    if solid > 0.95:
        score_circulo += 15
    if 0.85 < elipse < 1.15:
        score_circulo += 10
    if 0.85 < aspect < 1.15:
        score_circulo += 10
    if vertices_6_plus >= 3:
        score_circulo += 15

    if area_cm2 is not None:
        if 4.0 < area_cm2 < 6.0:
            score_circulo += 20
        elif 3.5 < area_cm2 < 6.5:
            score_circulo += 10

    if vertices_5 >= 2:
        score_circulo -= 20
    if vertices_4 >= 2:
        score_circulo -= 25
    scores["circle"] = score_circulo

    # square
    score_cuadrado = 0
    if vertices_4 >= 3:
        score_cuadrado += 45
    elif vertices_4 >= 2:
        score_cuadrado += 35
    elif n_vert == 4:
        score_cuadrado += 25

    if 0.85 < aspect < 1.15:
        score_cuadrado += 25
    elif 0.75 < aspect < 1.25:
        score_cuadrado += 15

    if solid > 0.9:
        score_cuadrado += 10
    if 0.72 < circ < 0.82:
        score_cuadrado += 15

    if area_cm2 is not None:
        if 5.0 < area_cm2 < 8.0:
            score_cuadrado += 15
        elif 4.0 < area_cm2 < 9.0:
            score_cuadrado += 5

    if circ > 0.88:
        score_cuadrado -= 40
    scores["square"] = score_cuadrado

    # rectangle
    score_rectangulo = 0
    if vertices_4 >= 3:
        score_rectangulo += 40
    elif vertices_4 >= 2:
        score_rectangulo += 30
    elif n_vert == 4:
        score_rectangulo += 20

    if aspect < 0.6 or aspect > 1.7:
        score_rectangulo += 45
    elif aspect < 0.7 or aspect > 1.45:
        score_rectangulo += 30

    if solid > 0.9:
        score_rectangulo += 10

    if area_cm2 is not None:
        if area_cm2 > 10.0:
            score_rectangulo += 25
        elif area_cm2 > 8.0:
            score_rectangulo += 15

    if circ > 0.85:
        score_rectangulo -= 35
    if 0.85 < aspect < 1.15:
        score_rectangulo -= 30
    scores["rectangle"] = score_rectangulo

    # pentagon
    score_pentagono = 0
    if vertices_5 >= 3:
        score_pentagono += 50
    elif vertices_5 >= 2:
        score_pentagono += 40
    elif vertices_5_6 >= 3:
        score_pentagono += 35
    elif n_vert == 5:
        score_pentagono += 30
    elif n_vert == 6:
        score_pentagono += 15

    if 0.82 < circ < 0.90:
        score_pentagono += 25
    elif 0.78 < circ < 0.92:
        score_pentagono += 10

    if solid > 0.93:
        score_pentagono += 10
    if 0.80 < aspect < 1.25:
        score_pentagono += 5

    if area_cm2 is not None:
        if 2.5 < area_cm2 < 5.0:
            score_pentagono += 30
        elif area_cm2 < 5.5:
            score_pentagono += 15
        if area_cm2 > 6.0:
            score_pentagono -= 25

    if vertices_4 >= 3:
        score_pentagono -= 35
    if vertices_6_plus >= 3:
        score_pentagono -= 15
    scores["pentagon"] = score_pentagono

    mejor_forma = max(scores, key=scores.get)
    mejor_score = scores[mejor_forma]
    if mejor_score < 35:
        return "unknown", mejor_score / 100.0

    confianza = min(mejor_score / 100.0, 1.0)
    return mejor_forma, confianza


def detectar_figuras_y_polares(frame, debug=False):
    """
    Detecta el disco blanco y TODAS las piezas naranjas dentro.
    Devuelve dict con shapes: [{shape_name, confidence, r_cm, theta_deg, ...}, ...]
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = gray.shape

    # 1) disco blanco (ROI centrada)
    cx_img, cy_img = w // 2, h // 2
    roi_w, roi_h = int(w * 0.6), int(h * 0.6)
    x0 = max(cx_img - roi_w // 2, 0)
    y0 = max(cy_img - roi_h // 2, 0)
    x1 = min(cx_img + roi_w // 2, w)
    y1 = min(cy_img + roi_h // 2, h)

    roi = gray[y0:y1, x0:x1]
    roi_blur = cv2.GaussianBlur(roi, (5, 5), 0)
    _, roi_th = cv2.threshold(roi_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    contours, _ = cv2.findContours(roi_th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    areas = [cv2.contourArea(c) for c in contours]
    idx_max = int(np.argmax(areas))
    cnt_disk = contours[idx_max]
    if areas[idx_max] < 1000:
        return None

    (xc_roi, yc_roi), R_pix = cv2.minEnclosingCircle(cnt_disk)
    cx_disk = x0 + xc_roi
    cy_disk = y0 + yc_roi

    # 2) máscara disco + máscara naranja
    mask_disk = np.zeros_like(gray, dtype=np.uint8)
    cv2.circle(mask_disk, (int(cx_disk), int(cy_disk)), int(R_pix * 0.9), 255, -1)

    hsv_inside = cv2.bitwise_and(hsv, hsv, mask=mask_disk)

    lower_orange = np.array([3, 70, 70], dtype=np.uint8)
    upper_orange = np.array([28, 255, 255], dtype=np.uint8)
    mask_orange = cv2.inRange(hsv_inside, lower_orange, upper_orange)

    kernel_small = np.ones((3, 3), np.uint8)
    kernel_med = np.ones((5, 5), np.uint8)

    mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_OPEN, kernel_small, iterations=1)
    mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, kernel_med, iterations=2)
    mask_orange = cv2.GaussianBlur(mask_orange, (3, 3), 0)
    _, mask_orange = cv2.threshold(mask_orange, 127, 255, cv2.THRESH_BINARY)

    contours_obj, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result = {
        "disk_center": (cx_disk, cy_disk),
        "disk_radius": R_pix,
        "shapes": [],
        "mask_debug": mask_orange if debug else None,
    }

    if not contours_obj:
        return result

    area_min = 300
    scale = DISK_RADIUS_CM / R_pix  # cm/pixel

    for cnt in contours_obj:
        area = cv2.contourArea(cnt)
        if area < area_min:
            continue

        features = calcular_caracteristicas_geometricas(cnt)
        if features is None:
            continue

        area_cm2 = features["area"] * (scale ** 2)
        shape_name, confidence = clasificar_forma(features, area_cm2)

        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue

        cx_obj = M["m10"] / M["m00"]
        cy_obj = M["m01"] / M["m00"]

        dx = cx_obj - cx_disk
        dy = cy_obj - cy_disk

        r_pix = math.hypot(dx, dy)
        r_cm = r_pix * scale

        theta_rad = math.atan2(-dy, dx)
        theta_deg = (math.degrees(theta_rad) + 360.0) % 360.0

        features["area_cm2"] = area_cm2

        result["shapes"].append(
            {
                "shape_name": shape_name,
                "confidence": confidence,
                "r_cm": r_cm,
                "theta_deg": theta_deg,
                "obj_center": (cx_obj, cy_obj),
                "features": features,
            }
        )

    result["shapes"].sort(key=lambda s: s["shape_name"])
    return result


# =========================
# NUEVO: utilidades pedidas
# =========================

def shape_name_to_code(shape_name: str) -> str:
    """square->s, rectangle->r, circle->c, pentagon->p, unknown->u"""
    mapping = {
        "square": "s",
        "rectangle": "r",
        "circle": "c",
        "pentagon": "p",
    }
    return mapping.get(shape_name, "u")


def polar_to_cartesian_m(r_cm: float, theta_deg: float, offset_x_m: float = 0.0, offset_y_m: float = 0.0):
    """
    Convierte (r_cm, theta_deg) a (x_m, y_m) y aplica offset.
    theta_deg está en el sistema del disco (0° hacia +x, CCW).
    """
    r_m = r_cm / 100.0
    th = math.radians(theta_deg)
    x = r_m * math.cos(th) + offset_x_m
    y = r_m * math.sin(th) + offset_y_m
    return x, y


def pick_best_detection(info: dict):
    """
    Escoge 1 objeto (el de mayor confidence que NO sea unknown).
    Si todos son unknown, devuelve el de mayor confidence igual.
    """
    if info is None or not info.get("shapes"):
        return None

    shapes = info["shapes"]
    candidates = [s for s in shapes if s["shape_name"] != "unknown"]
    if not candidates:
        candidates = shapes

    best = max(candidates, key=lambda s: s.get("confidence", 0.0))
    return best
