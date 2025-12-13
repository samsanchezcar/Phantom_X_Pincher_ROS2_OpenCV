import cv2
import numpy as np
import math

# Physical radius of the white disk (in cm)
DISK_RADIUS_CM = 7.25
WINDOW_NAME = "Shape detection on disk - C270"


def calcular_caracteristicas_geometricas(cnt):
    """
    Calcula múltiples características geométricas invariantes a rotación.
    Estas son más robustas que solo contar vértices.
    """
    area = cv2.contourArea(cnt)
    perimetro = cv2.arcLength(cnt, True)

    if area < 100 or perimetro < 10:
        return None

    # 1. Circularidad (4π * área / perímetro²) - 1.0 para círculo perfecto
    circularidad = (4 * math.pi * area) / (perimetro * perimetro)

    # 2. Convex hull y solidez
    hull = cv2.convexHull(cnt)
    area_hull = cv2.contourArea(hull)
    solidez = area / area_hull if area_hull > 0 else 0

    # 3. Bounding box y extensión
    x, y, w, h = cv2.boundingRect(cnt)
    area_rect = w * h
    extension = area / area_rect if area_rect > 0 else 0
    aspect_ratio = w / h if h > 0 else 0

    # 4. Ellipse ajustada (si hay suficientes puntos)
    if len(cnt) >= 5:
        ellipse = cv2.fitEllipse(cnt)
        (cx_e, cy_e), (eje_menor, eje_mayor), angle = ellipse
        elipse_ratio = eje_menor / eje_mayor if eje_mayor > 0 else 0
    else:
        elipse_ratio = aspect_ratio

    # 5. Número de vértices con múltiples valores de epsilon
    vertices_list = []
    for eps_factor in [0.01, 0.02, 0.03, 0.04]:
        approx = cv2.approxPolyDP(cnt, eps_factor * perimetro, True)
        vertices_list.append(len(approx))

    # Usar la mediana para más estabilidad
    n_vertices = int(np.median(vertices_list))

    # 6. Defectos de convexidad (útil para formas complejas)
    hull_indices = cv2.convexHull(cnt, returnPoints=False)
    if len(hull_indices) > 3:
        defects = cv2.convexityDefects(cnt, hull_indices)
        n_defectos = len(defects) if defects is not None else 0
    else:
        n_defectos = 0

    # 7. Momentos de Hu (invariantes a rotación, escala, traslación)
    momentos = cv2.moments(cnt)
    hu_moments = cv2.HuMoments(momentos).flatten()
    # Log transform para mejor comparación
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
    Usa múltiples características geométricas + área real para robustez.
    """
    if features is None:
        return "unknown", 0.0

    circ = features["circularidad"]
    solid = features["solidez"]
    ext = features["extension"]
    aspect = features["aspect_ratio"]
    elipse = features["elipse_ratio"]
    n_vert = features["n_vertices"]
    vert_list = features["vertices_list"]

    # Contar vértices consistentes
    vertices_4 = sum(1 for v in vert_list if v == 4)
    vertices_5 = sum(1 for v in vert_list if v == 5)
    vertices_5_6 = sum(1 for v in vert_list if v in [5, 6])
    vertices_6_plus = sum(1 for v in vert_list if v >= 6)

    scores = {}

    # === CÍRCULO ===
    score_circulo = 0

    # Circularidad MUY alta para ser círculo
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

    # Muchos vértices (el círculo se aproxima con muchos)
    if vertices_6_plus >= 3:
        score_circulo += 15

    # Área del círculo: ~4.9 cm²
    if area_cm2 is not None:
        if 4.0 < area_cm2 < 6.0:
            score_circulo += 20
        elif 3.5 < area_cm2 < 6.5:
            score_circulo += 10

    # Penalizar si tiene pocos vértices consistentes
    if vertices_5 >= 2:
        score_circulo -= 20
    if vertices_4 >= 2:
        score_circulo -= 25

    scores["circle"] = score_circulo

    # === CUADRADO ===
    score_cuadrado = 0

    if vertices_4 >= 3:
        score_cuadrado += 45
    elif vertices_4 >= 2:
        score_cuadrado += 35
    elif n_vert == 4:
        score_cuadrado += 25

    # Aspect ratio cercano a 1
    if 0.85 < aspect < 1.15:
        score_cuadrado += 25
    elif 0.75 < aspect < 1.25:
        score_cuadrado += 15

    if solid > 0.9:
        score_cuadrado += 10

    # Circularidad del cuadrado: ~0.78
    if 0.72 < circ < 0.82:
        score_cuadrado += 15

    # Área del cuadrado: ~6.25 cm²
    if area_cm2 is not None:
        if 5.0 < area_cm2 < 8.0:
            score_cuadrado += 15
        elif 4.0 < area_cm2 < 9.0:
            score_cuadrado += 5

    # Penalizar si es muy circular
    if circ > 0.88:
        score_cuadrado -= 40

    scores["square"] = score_cuadrado

    # === RECTÁNGULO ===
    score_rectangulo = 0

    if vertices_4 >= 3:
        score_rectangulo += 40
    elif vertices_4 >= 2:
        score_rectangulo += 30
    elif n_vert == 4:
        score_rectangulo += 20

    # Aspect ratio alejado de 1 (es alargado)
    if aspect < 0.6 or aspect > 1.7:
        score_rectangulo += 45
    elif aspect < 0.7 or aspect > 1.45:
        score_rectangulo += 30

    if solid > 0.9:
        score_rectangulo += 10

    # Área del rectángulo: ~12.5 cm² (la más grande)
    if area_cm2 is not None:
        if area_cm2 > 10.0:
            score_rectangulo += 25
        elif area_cm2 > 8.0:
            score_rectangulo += 15

    # Penalizar si es muy circular o aspect ~1
    if circ > 0.85:
        score_rectangulo -= 35
    if 0.85 < aspect < 1.15:
        score_rectangulo -= 30

    scores["rectangle"] = score_rectangulo

    # === PENTÁGONO ===
    score_pentagono = 0

    # Vértices: el pentágono debe mostrar ~5 vértices
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

    # Circularidad del pentágono regular: ~0.865
    if 0.82 < circ < 0.90:
        score_pentagono += 25
    elif 0.78 < circ < 0.92:
        score_pentagono += 10

    # Solidez alta (convexo)
    if solid > 0.93:
        score_pentagono += 10

    # Aspect ratio moderado
    if 0.80 < aspect < 1.25:
        score_pentagono += 5

    # Pentágono es la figura más pequeña (~3.9 cm²)
    if area_cm2 is not None:
        if 2.5 < area_cm2 < 5.0:
            score_pentagono += 30
        elif area_cm2 < 5.5:
            score_pentagono += 15
        if area_cm2 > 6.0:
            score_pentagono -= 25

    # Penalizar si tiene 4 vértices claros o muchos vértices (círculo)
    if vertices_4 >= 3:
        score_pentagono -= 35
    if vertices_6_plus >= 3:
        score_pentagono -= 15

    scores["pentagon"] = score_pentagono

    # Mejor forma
    mejor_forma = max(scores, key=scores.get)
    mejor_score = scores[mejor_forma]

    if mejor_score < 35:
        return "unknown", mejor_score / 100.0

    confianza = min(mejor_score / 100.0, 1.0)
    return mejor_forma, confianza


def detectar_figuras_y_polares(frame, debug=False):
    """
    Detecta el disco blanco y TODAS las piezas naranjas dentro.
    Devuelve:
      {
        'disk_center': (cx, cy),
        'disk_radius': R_pix,
        'shapes': [
            {
              'shape_name': str,
              'confidence': float,
              'r_cm': float,
              'theta_deg': float,
              'obj_center': (cx, cy),
              'features': dict
            },
            ...
        ],
        'mask_debug': mask_orange (opcional)
      }
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = gray.shape

    # --- 1) Encontrar el disco blanco ---
    cx_img, cy_img = w // 2, h // 2
    roi_w, roi_h = int(w * 0.6), int(h * 0.6)
    x0 = max(cx_img - roi_w // 2, 0)
    y0 = max(cy_img - roi_h // 2, 0)
    x1 = min(cx_img + roi_w // 2, w)
    y1 = min(cy_img + roi_h // 2, h)

    roi = gray[y0:y1, x0:x1]
    roi_blur = cv2.GaussianBlur(roi, (5, 5), 0)
    _, roi_th = cv2.threshold(
        roi_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    contours, _ = cv2.findContours(
        roi_th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
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

    # --- 2) Máscara del disco y máscara naranja ---
    mask_disk = np.zeros_like(gray, dtype=np.uint8)
    cv2.circle(
        mask_disk,
        (int(cx_disk), int(cy_disk)),
        int(R_pix * 0.9),
        255,
        -1,
    )

    hsv_inside = cv2.bitwise_and(hsv, hsv, mask=mask_disk)

    # Rango naranja (ajustable)
    lower_orange = np.array([3, 70, 70], dtype=np.uint8)
    upper_orange = np.array([28, 255, 255], dtype=np.uint8)

    mask_orange = cv2.inRange(hsv_inside, lower_orange, upper_orange)

    kernel_small = np.ones((3, 3), np.uint8)
    kernel_med = np.ones((5, 5), np.uint8)

    mask_orange = cv2.morphologyEx(
        mask_orange, cv2.MORPH_OPEN, kernel_small, iterations=1
    )
    mask_orange = cv2.morphologyEx(
        mask_orange, cv2.MORPH_CLOSE, kernel_med, iterations=2
    )
    mask_orange = cv2.GaussianBlur(mask_orange, (3, 3), 0)
    _, mask_orange = cv2.threshold(mask_orange, 127, 255, cv2.THRESH_BINARY)

    contours_obj, _ = cv2.findContours(
        mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

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

    # Ordenar por nombre para salida estable
    result["shapes"].sort(key=lambda s: s["shape_name"])
    return result


def main():
    cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print("❌ Could not open camera.")
        return

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    print("✅ Camera opened. Press 'q' to quit, 'd' for debug mode.")

    debug_mode = False
    last_summary = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Could not read frame.")
            break

        info = detectar_figuras_y_polares(frame, debug=debug_mode)

        if info is not None:
            cx_disk, cy_disk = info["disk_center"]
            R_pix = info["disk_radius"]
            shapes = info["shapes"]

            # Disco
            cv2.circle(
                frame, (int(cx_disk), int(cy_disk)), int(R_pix), (0, 255, 0), 2
            )
            cv2.circle(
                frame, (int(cx_disk), int(cy_disk)), 3, (0, 0, 255), -1
            )

            summary_parts = []

            for s in shapes:
                shape_name = s["shape_name"]
                conf = s["confidence"]
                r_cm = s["r_cm"]
                theta_deg = s["theta_deg"]
                cx_obj, cy_obj = s["obj_center"]
                features = s["features"]

                # Contorno
                if features and "contorno" in features:
                    cv2.drawContours(frame, [features["contorno"]], -1, (255, 0, 255), 2)

                # Línea centro disco -> centro figura
                cv2.line(
                    frame,
                    (int(cx_disk), int(cy_disk)),
                    (int(cx_obj), int(cy_obj)),
                    (255, 0, 0),
                    2,
                )
                cv2.circle(
                    frame, (int(cx_obj), int(cy_obj)), 5, (0, 255, 255), -1
                )

                # Texto cerca de la figura
                text = f"{shape_name} ({conf:.0%}), r={r_cm:.2f}cm, Angle={theta_deg:.1f} deg"
                cv2.putText(
                    frame,
                    text,
                    (int(cx_obj) + 5, int(cy_obj) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

                summary_parts.append(text)

            # Debug overlay (solo máscara y 1–2 líneas genéricas)
            if debug_mode and info["mask_debug"] is not None:
                mask_small = cv2.resize(info["mask_debug"], (160, 120))
                mask_color = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)
                frame[10:130, 470:630] = mask_color

            if summary_parts:
                summary = " | ".join(summary_parts)
                if summary != last_summary:
                    print(summary)
                    last_summary = summary

        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("d"):
            debug_mode = not debug_mode
            print(f"Debug mode: {'ON' if debug_mode else 'OFF'}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
