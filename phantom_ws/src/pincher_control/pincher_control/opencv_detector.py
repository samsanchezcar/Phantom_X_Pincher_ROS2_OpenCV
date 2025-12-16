# ============================================================
#  DETECTOR DE FIGURAS GEOMÉTRICAS CON OPENCV
# ============================================================
#
#  Este módulo detecta figuras geométricas (círculo, cuadrado,
#  rectángulo, pentágono) de color naranja colocadas sobre un
#  disco blanco. Calcula las coordenadas polares de cada figura
#  respecto al centro del disco.
#
#  Flujo general:
#  1. Detectar el disco blanco en la imagen
#  2. Crear máscara para objetos naranjas dentro del disco
#  3. Encontrar contornos de las figuras
#  4. Calcular características geométricas de cada contorno
#  5. Clasificar la forma basándose en las características
#  6. Calcular posición polar (r, theta) respecto al centro del disco
#
# ============================================================

import cv2          # OpenCV - biblioteca de visión por computadora
import numpy as np  # NumPy - operaciones numéricas y matrices
import math         # Funciones matemáticas (atan2, hypot, etc.)

# ============================================================
#  CONSTANTE DE CALIBRACIÓN
# ============================================================

# Radio físico real del disco blanco en centímetros
# Se usa para convertir de píxeles a unidades reales (cm)
# IMPORTANTE: Medir el disco real y ajustar este valor
DISK_RADIUS_CM = 7.25


def calcular_caracteristicas_geometricas(cnt):
    """
    Calcula múltiples características geométricas de un contorno.
    Estas características son invariantes a rotación y se usan
    para clasificar qué tipo de figura es.
    
    Args:
        cnt: Contorno de OpenCV (array de puntos)
    
    Returns:
        dict: Diccionario con todas las características calculadas,
              o None si el contorno es demasiado pequeño
    
    Características calculadas:
    - area: Área del contorno en píxeles²
    - perimetro: Perímetro del contorno en píxeles
    - circularidad: Qué tan circular es (1.0 = círculo perfecto)
    - solidez: Área del contorno / Área del convex hull
    - extension: Área del contorno / Área del bounding rect
    - aspect_ratio: Ancho / Alto del bounding rect
    - elipse_ratio: Eje menor / Eje mayor de la elipse ajustada
    - n_vertices: Número mediano de vértices (aproximación poligonal)
    - n_defectos: Número de defectos de convexidad
    - hu_moments: Momentos de Hu (invariantes a escala y rotación)
    """
    
    # Calcular área del contorno
    area = cv2.contourArea(cnt)
    
    # Calcular perímetro (longitud del contorno cerrado)
    perimetro = cv2.arcLength(cnt, True)  # True = contorno cerrado

    # Filtrar contornos muy pequeños (ruido)
    if area < 100 or perimetro < 10:
        return None

    # ============================================================
    #  CIRCULARIDAD (Isoperimetric quotient)
    # ============================================================
    # Fórmula: 4π × Area / Perímetro²
    # Un círculo perfecto tiene circularidad = 1.0
    # Cuadrado ≈ 0.785, Triángulo ≈ 0.604
    circularidad = (4 * math.pi * area) / (perimetro * perimetro)

    # ============================================================
    #  SOLIDEZ (Solidity)
    # ============================================================
    # Relación entre el área del contorno y su envolvente convexa (convex hull)
    # Figuras convexas (sin concavidades) tienen solidez cercana a 1.0
    # Figuras con "entrantes" tienen solidez menor
    hull = cv2.convexHull(cnt)
    area_hull = cv2.contourArea(hull)
    solidez = area / area_hull if area_hull > 0 else 0

    # ============================================================
    #  EXTENSIÓN (Extent)
    # ============================================================
    # Relación entre el área del contorno y su rectángulo delimitador
    # Cuadrado rotado 45° tiene menor extensión que uno alineado
    x, y, w, h = cv2.boundingRect(cnt)  # Rectángulo delimitador alineado a ejes
    area_rect = w * h
    extension = area / area_rect if area_rect > 0 else 0
    
    # ============================================================
    #  ASPECT RATIO (Relación de aspecto)
    # ============================================================
    # Ancho / Alto del rectángulo delimitador
    # Cuadrado ≈ 1.0, Rectángulo horizontal > 1.0, vertical < 1.0
    aspect_ratio = w / h if h > 0 else 0

    # ============================================================
    #  RELACIÓN DE ELIPSE
    # ============================================================
    # Ajusta una elipse al contorno y calcula eje_menor / eje_mayor
    # Círculo ≈ 1.0, formas alargadas < 1.0
    if len(cnt) >= 5:  # fitEllipse requiere mínimo 5 puntos
        ellipse = cv2.fitEllipse(cnt)
        # ellipse = ((centro_x, centro_y), (eje_menor, eje_mayor), ángulo)
        (cx_e, cy_e), (eje_menor, eje_mayor), angle = ellipse
        elipse_ratio = eje_menor / eje_mayor if eje_mayor > 0 else 0
    else:
        # Si no hay suficientes puntos, usar aspect_ratio como aproximación
        elipse_ratio = aspect_ratio

    # ============================================================
    #  CONTEO DE VÉRTICES (Aproximación poligonal)
    # ============================================================
    # Aproxima el contorno a un polígono con menos puntos
    # El número de vértices indica el tipo de figura:
    # - 3 vértices = triángulo
    # - 4 vértices = cuadrado/rectángulo
    # - 5 vértices = pentágono
    # - 6+ vértices = círculo (aproximado)
    #
    # Probamos varios factores de epsilon para robustez
    # epsilon = factor × perímetro (mayor epsilon = menos vértices)
    vertices_list = []
    for eps_factor in [0.01, 0.02, 0.03, 0.04]:
        approx = cv2.approxPolyDP(cnt, eps_factor * perimetro, True)
        vertices_list.append(len(approx))
    
    # Usar la mediana para mayor robustez ante ruido
    n_vertices = int(np.median(vertices_list))

    # ============================================================
    #  DEFECTOS DE CONVEXIDAD
    # ============================================================
    # Los defectos son las "entrantes" entre el contorno y su hull convexo
    # Útil para detectar formas cóncavas o con protuberancias
    hull_indices = cv2.convexHull(cnt, returnPoints=False)  # Índices, no puntos
    if len(hull_indices) > 3:
        defects = cv2.convexityDefects(cnt, hull_indices)
        n_defectos = len(defects) if defects is not None else 0
    else:
        n_defectos = 0

    # ============================================================
    #  MOMENTOS DE HU
    # ============================================================
    # Los momentos de Hu son 7 valores invariantes a:
    # - Traslación
    # - Escala
    # - Rotación
    # Son útiles para comparar formas independientemente de su posición/tamaño
    momentos = cv2.moments(cnt)
    hu_moments = cv2.HuMoments(momentos).flatten()  # Array de 7 valores
    
    # Transformación logarítmica para mejor manejo numérico
    # (los momentos pueden variar en muchos órdenes de magnitud)
    hu_log = -np.sign(hu_moments) * np.log10(np.abs(hu_moments) + 1e-10)

    # Retornar todas las características en un diccionario
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
    Clasifica la forma geométrica basándose en sus características.
    Usa un sistema de puntuación (scoring) donde cada característica
    suma o resta puntos a cada posible clasificación.
    
    Args:
        features: Diccionario de características (de calcular_caracteristicas_geometricas)
        area_cm2: Área en cm² (opcional, ayuda a la clasificación)
    
    Returns:
        tuple: (nombre_forma, confianza)
            - nombre_forma: "circle", "square", "rectangle", "pentagon" o "unknown"
            - confianza: valor entre 0.0 y 1.0
    
    El sistema de scoring funciona así:
    - Cada forma tiene un score inicial de 0
    - Se suman puntos cuando una característica coincide con lo esperado
    - Se restan puntos cuando una característica contradice la forma
    - La forma con mayor score gana
    - Si el score máximo es menor a 35, se clasifica como "unknown"
    """
    
    if features is None:
        return "unknown", 0.0

    # Extraer características del diccionario
    circ = features["circularidad"]       # Qué tan circular (1.0 = círculo)
    solid = features["solidez"]           # Qué tan "lleno" es el contorno
    aspect = features["aspect_ratio"]     # Ancho/Alto del bounding rect
    elipse = features["elipse_ratio"]     # Eje menor/mayor de elipse ajustada
    n_vert = features["n_vertices"]       # Número de vértices (mediana)
    vert_list = features["vertices_list"] # Lista de vértices con diferentes epsilon

    # Contar cuántas veces aparece cada número de vértices
    # Esto da robustez: si 3 de 4 pruebas dan 4 vértices, es más probable que sea un cuadrilátero
    vertices_4 = sum(1 for v in vert_list if v == 4)      # Conteo de 4 vértices
    vertices_5 = sum(1 for v in vert_list if v == 5)      # Conteo de 5 vértices
    vertices_5_6 = sum(1 for v in vert_list if v in [5, 6])  # 5 o 6 vértices
    vertices_6_plus = sum(1 for v in vert_list if v >= 6)    # 6 o más vértices

    # Diccionario para almacenar scores de cada forma
    scores = {}

    # ============================================================
    #  SCORING PARA CÍRCULO
    # ============================================================
    # Características típicas de un círculo:
    # - Alta circularidad (> 0.85)
    # - Alta solidez (> 0.95)
    # - Aspect ratio cercano a 1
    # - Muchos vértices en la aproximación poligonal (6+)
    
    score_circulo = 0
    
    # Circularidad alta = muy probablemente círculo
    if circ > 0.92:
        score_circulo += 50   # Muy circular
    elif circ > 0.88:
        score_circulo += 30   # Bastante circular
    elif circ > 0.82:
        score_circulo += 10   # Algo circular

    # Solidez alta (sin concavidades)
    if solid > 0.95:
        score_circulo += 15
    
    # Relación de elipse cercana a 1 (igual en ambos ejes)
    if 0.85 < elipse < 1.15:
        score_circulo += 10
    
    # Aspect ratio cercano a 1
    if 0.85 < aspect < 1.15:
        score_circulo += 10
    
    # Muchos vértices = contorno suave = círculo
    if vertices_6_plus >= 3:
        score_circulo += 15

    # Bonus por área típica de círculo (basado en calibración empírica)
    if area_cm2 is not None:
        if 4.0 < area_cm2 < 6.0:
            score_circulo += 20
        elif 3.5 < area_cm2 < 6.5:
            score_circulo += 10

    # Penalizaciones: si tiene pocos vértices, probablemente NO es círculo
    if vertices_5 >= 2:
        score_circulo -= 20   # Parece pentágono
    if vertices_4 >= 2:
        score_circulo -= 25   # Parece cuadrilátero
    
    scores["circle"] = score_circulo

    # ============================================================
    #  SCORING PARA CUADRADO
    # ============================================================
    # Características típicas de un cuadrado:
    # - Exactamente 4 vértices
    # - Aspect ratio muy cercano a 1 (lados iguales)
    # - Circularidad media (~0.785 para cuadrado perfecto)
    
    score_cuadrado = 0
    
    # 4 vértices es clave para cuadriláteros
    if vertices_4 >= 3:
        score_cuadrado += 45
    elif vertices_4 >= 2:
        score_cuadrado += 35
    elif n_vert == 4:
        score_cuadrado += 25

    # Aspect ratio cercano a 1 distingue cuadrado de rectángulo
    if 0.85 < aspect < 1.15:
        score_cuadrado += 25   # Muy cuadrado
    elif 0.75 < aspect < 1.25:
        score_cuadrado += 15   # Algo cuadrado

    # Alta solidez (figura convexa sin huecos)
    if solid > 0.9:
        score_cuadrado += 10
    
    # Circularidad típica de cuadrado (π/4 ≈ 0.785)
    if 0.72 < circ < 0.82:
        score_cuadrado += 15

    # Bonus por área típica
    if area_cm2 is not None:
        if 5.0 < area_cm2 < 8.0:
            score_cuadrado += 15
        elif 4.0 < area_cm2 < 9.0:
            score_cuadrado += 5

    # Penalización: circularidad muy alta = probablemente círculo
    if circ > 0.88:
        score_cuadrado -= 40
    
    scores["square"] = score_cuadrado

    # ============================================================
    #  SCORING PARA RECTÁNGULO
    # ============================================================
    # Características típicas de un rectángulo:
    # - Exactamente 4 vértices
    # - Aspect ratio alejado de 1 (lados desiguales)
    # - Área típicamente mayor que cuadrado
    
    score_rectangulo = 0
    
    # 4 vértices
    if vertices_4 >= 3:
        score_rectangulo += 40
    elif vertices_4 >= 2:
        score_rectangulo += 30
    elif n_vert == 4:
        score_rectangulo += 20

    # Aspect ratio alejado de 1 es CLAVE para rectángulo vs cuadrado
    if aspect < 0.6 or aspect > 1.7:
        score_rectangulo += 45   # Muy alargado
    elif aspect < 0.7 or aspect > 1.45:
        score_rectangulo += 30   # Bastante alargado

    # Alta solidez
    if solid > 0.9:
        score_rectangulo += 10

    # Área grande (rectángulos suelen ser más grandes)
    if area_cm2 is not None:
        if area_cm2 > 10.0:
            score_rectangulo += 25
        elif area_cm2 > 8.0:
            score_rectangulo += 15

    # Penalizaciones
    if circ > 0.85:
        score_rectangulo -= 35   # Muy circular = no rectángulo
    if 0.85 < aspect < 1.15:
        score_rectangulo -= 30   # Aspect ratio cuadrado = no rectángulo
    
    scores["rectangle"] = score_rectangulo

    # ============================================================
    #  SCORING PARA PENTÁGONO
    # ============================================================
    # Características típicas de un pentágono:
    # - Exactamente 5 vértices
    # - Circularidad media-alta (~0.865 para pentágono regular)
    # - Área típicamente pequeña
    
    score_pentagono = 0
    
    # 5 vértices es clave
    if vertices_5 >= 3:
        score_pentagono += 50
    elif vertices_5 >= 2:
        score_pentagono += 40
    elif vertices_5_6 >= 3:
        score_pentagono += 35
    elif n_vert == 5:
        score_pentagono += 30
    elif n_vert == 6:
        score_pentagono += 15   # 6 vértices podría ser pentágono mal detectado

    # Circularidad típica de pentágono
    if 0.82 < circ < 0.90:
        score_pentagono += 25
    elif 0.78 < circ < 0.92:
        score_pentagono += 10

    # Alta solidez
    if solid > 0.93:
        score_pentagono += 10
    
    # Aspect ratio relativamente cuadrado
    if 0.80 < aspect < 1.25:
        score_pentagono += 5

    # Área típica de pentágono (generalmente pequeño)
    if area_cm2 is not None:
        if 2.5 < area_cm2 < 5.0:
            score_pentagono += 30
        elif area_cm2 < 5.5:
            score_pentagono += 15
        if area_cm2 > 6.0:
            score_pentagono -= 25   # Muy grande para pentágono

    # Penalizaciones
    if vertices_4 >= 3:
        score_pentagono -= 35   # Parece cuadrilátero
    if vertices_6_plus >= 3:
        score_pentagono -= 15   # Parece círculo
    
    scores["pentagon"] = score_pentagono

    # ============================================================
    #  SELECCIÓN FINAL
    # ============================================================
    
    # Encontrar la forma con mayor score
    mejor_forma = max(scores, key=scores.get)
    mejor_score = scores[mejor_forma]
    
    # Si el score es muy bajo, la clasificación no es confiable
    if mejor_score < 35:
        return "unknown", mejor_score / 100.0

    # Calcular confianza normalizada (0.0 a 1.0)
    confianza = min(mejor_score / 100.0, 1.0)
    
    return mejor_forma, confianza


def detectar_figuras_y_polares(frame, debug=False):
    """
    Función principal de detección.
    Detecta el disco blanco y todas las piezas naranjas dentro de él.
    Calcula las coordenadas polares de cada pieza respecto al centro del disco.
    
    Args:
        frame: Imagen BGR de OpenCV (numpy array)
        debug: Si True, incluye máscara de debug en el resultado
    
    Returns:
        dict: Diccionario con toda la información de detección:
            - disk_center: (cx, cy) centro del disco en píxeles
            - disk_radius: Radio del disco en píxeles
            - disk_cnt: Contorno del disco (para dibujar)
            - disk_center_px: Igual que disk_center (para compatibilidad)
            - shapes: Lista de figuras detectadas, cada una con:
                - shape_name: Nombre de la forma
                - confidence: Confianza de la clasificación
                - r_cm: Distancia al centro del disco en cm
                - theta_deg: Ángulo en grados (0° = derecha, CCW)
                - obj_center: Centro del objeto en píxeles
                - center_px: Igual que obj_center
                - cnt: Contorno del objeto
                - features: Características geométricas
            - mask_debug: Máscara binaria (solo si debug=True)
        
        Retorna None si no se detecta el disco blanco.
    """
    
    # ============================================================
    #  PREPROCESAMIENTO
    # ============================================================
    
    # Convertir a escala de grises (para detectar disco blanco)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Convertir a HSV (para detectar color naranja)
    # HSV = Hue (tono), Saturation (saturación), Value (brillo)
    # Es mejor que RGB para segmentar colores
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Dimensiones de la imagen
    h, w = gray.shape

    # ============================================================
    #  PASO 1: DETECTAR DISCO BLANCO
    # ============================================================
    
    # Definir ROI (Region of Interest) en el centro de la imagen
    # Asumimos que el disco está aproximadamente centrado
    # Esto reduce falsos positivos en los bordes
    cx_img, cy_img = w // 2, h // 2  # Centro de la imagen
    roi_w, roi_h = int(w * 0.6), int(h * 0.6)  # ROI = 60% del tamaño
    
    # Calcular coordenadas del ROI (con clipping a los bordes)
    x0 = max(cx_img - roi_w // 2, 0)
    y0 = max(cy_img - roi_h // 2, 0)
    x1 = min(cx_img + roi_w // 2, w)
    y1 = min(cy_img + roi_h // 2, h)

    # Extraer la región de interés
    roi = gray[y0:y1, x0:x1]
    
    # Suavizar para reducir ruido
    roi_blur = cv2.GaussianBlur(roi, (5, 5), 0)
    
    # Binarizar usando Otsu (encuentra el umbral óptimo automáticamente)
    # El disco blanco debería quedar en blanco (255)
    _, roi_th = cv2.threshold(roi_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Encontrar contornos en la imagen binarizada
    # RETR_EXTERNAL = solo contornos externos (no anidados)
    contours, _ = cv2.findContours(roi_th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Si no hay contornos, no se detectó nada
    if not contours:
        return None

    # Encontrar el contorno más grande (debería ser el disco)
    areas = [cv2.contourArea(c) for c in contours]
    idx_max = int(np.argmax(areas))
    cnt_disk = contours[idx_max]
    
    # Verificar que el área es suficientemente grande
    if areas[idx_max] < 1000:
        return None

    # Encontrar el círculo mínimo que encierra el contorno del disco
    (xc_roi, yc_roi), R_pix = cv2.minEnclosingCircle(cnt_disk)
    
    # Convertir coordenadas del ROI a coordenadas de imagen completa
    cx_disk = x0 + xc_roi  # Centro X del disco
    cy_disk = y0 + yc_roi  # Centro Y del disco

    # Crear copia del contorno del disco en coordenadas de imagen completa
    # (el contorno original está en coordenadas del ROI)
    cnt_disk_full = cnt_disk.copy()
    cnt_disk_full[:, 0, 0] += x0  # Sumar offset X
    cnt_disk_full[:, 0, 1] += y0  # Sumar offset Y

    # ============================================================
    #  PASO 2: CREAR MÁSCARA PARA OBJETOS NARANJAS
    # ============================================================
    
    # Crear máscara circular del disco (ligeramente más pequeña para evitar bordes)
    mask_disk = np.zeros_like(gray, dtype=np.uint8)
    cv2.circle(mask_disk, (int(cx_disk), int(cy_disk)), int(R_pix * 0.9), 255, -1)

    # Aplicar máscara a la imagen HSV (solo analizar dentro del disco)
    hsv_inside = cv2.bitwise_and(hsv, hsv, mask=mask_disk)

    # Definir rango de color naranja en HSV
    # H: 3-28 (tonos naranjas/rojos)
    # S: 70-255 (saturación media-alta)
    # V: 70-255 (brillo medio-alto)
    lower_orange = np.array([3, 70, 70], dtype=np.uint8)
    upper_orange = np.array([28, 255, 255], dtype=np.uint8)
    
    # Crear máscara binaria de píxeles naranjas
    mask_orange = cv2.inRange(hsv_inside, lower_orange, upper_orange)

    # ============================================================
    #  PASO 3: LIMPIAR MÁSCARA (OPERACIONES MORFOLÓGICAS)
    # ============================================================
    
    # Kernels para operaciones morfológicas
    kernel_small = np.ones((3, 3), np.uint8)
    kernel_med = np.ones((5, 5), np.uint8)

    # OPEN: Erosión seguida de dilatación
    # Elimina pequeños puntos blancos (ruido)
    mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_OPEN, kernel_small, iterations=1)
    
    # CLOSE: Dilatación seguida de erosión
    # Rellena pequeños huecos negros dentro de las figuras
    mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, kernel_med, iterations=2)
    
    # Suavizar bordes
    mask_orange = cv2.GaussianBlur(mask_orange, (3, 3), 0)
    
    # Re-binarizar después del blur
    _, mask_orange = cv2.threshold(mask_orange, 127, 255, cv2.THRESH_BINARY)

    # ============================================================
    #  PASO 4: ENCONTRAR CONTORNOS DE OBJETOS NARANJAS
    # ============================================================
    
    contours_obj, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Preparar diccionario de resultado
    result = {
        "disk_center": (cx_disk, cy_disk),    # Centro del disco
        "disk_radius": R_pix,                  # Radio del disco en píxeles
        "disk_cnt": cnt_disk_full,             # Contorno del disco (para dibujar)
        "disk_center_px": (cx_disk, cy_disk),  # Igual que disk_center (compatibilidad)
        "shapes": [],                          # Lista de figuras detectadas
        "mask_debug": mask_orange if debug else None,  # Máscara para debug
    }

    # Si no hay objetos naranjas, retornar solo info del disco
    if not contours_obj:
        return result

    # ============================================================
    #  PASO 5: ANALIZAR CADA OBJETO NARANJA
    # ============================================================
    
    # Área mínima para considerar un objeto (filtra ruido)
    area_min = 300
    
    # Factor de escala: convierte píxeles a centímetros
    # Basado en el radio conocido del disco
    scale = DISK_RADIUS_CM / R_pix  # cm/pixel

    for cnt in contours_obj:
        area = cv2.contourArea(cnt)
        
        # Filtrar objetos muy pequeños
        if area < area_min:
            continue

        # Calcular características geométricas
        features = calcular_caracteristicas_geometricas(cnt)
        if features is None:
            continue

        # Calcular área en cm² (para ayudar a la clasificación)
        area_cm2 = features["area"] * (scale ** 2)
        
        # Clasificar la forma
        shape_name, confidence = clasificar_forma(features, area_cm2)

        # ============================================================
        #  CALCULAR CENTRO DEL OBJETO (usando momentos)
        # ============================================================
        # Los momentos son propiedades estadísticas del contorno
        # m00 = área, m10/m00 = centro X, m01/m00 = centro Y
        M = cv2.moments(cnt)
        if M["m00"] == 0:  # Evitar división por cero
            continue

        # Centro del objeto en píxeles
        cx_obj = M["m10"] / M["m00"]
        cy_obj = M["m01"] / M["m00"]

        # ============================================================
        #  CALCULAR COORDENADAS POLARES
        # ============================================================
        
        # Distancia del objeto al centro del disco (en píxeles)
        dx = cx_obj - cx_disk  # Diferencia en X
        dy = cy_obj - cy_disk  # Diferencia en Y

        # Distancia radial (teorema de Pitágoras)
        r_pix = math.hypot(dx, dy)  # sqrt(dx² + dy²)
        
        # Convertir a centímetros
        r_cm = r_pix * scale

        # Calcular ángulo theta
        # atan2(-dy, dx) porque el eje Y de imagen está invertido
        # (Y crece hacia abajo en coordenadas de imagen)
        theta_rad = math.atan2(-dy, dx)
        
        # Convertir a grados y normalizar a [0, 360)
        theta_deg = (math.degrees(theta_rad) + 360.0) % 360.0

        # Guardar área en cm² en las características
        features["area_cm2"] = area_cm2

        # Agregar objeto a la lista de resultados
        result["shapes"].append(
            {
            "shape_name": shape_name,          # Nombre: "circle", "square", etc.
            "confidence": confidence,          # Confianza de clasificación
            "r_cm": r_cm,                      # Distancia al centro en cm
            "theta_deg": theta_deg,            # Ángulo en grados
            "obj_center": (cx_obj, cy_obj),    # Centro en píxeles
            "center_px": (cx_obj, cy_obj),     # Igual (compatibilidad)
            "cnt": cnt,                        # Contorno (para dibujar)
            "features": features,              # Todas las características
            }
        )

    # Ordenar las figuras por nombre (para consistencia)
    result["shapes"].sort(key=lambda s: s["shape_name"])
    
    return result


# ============================================================
#  FUNCIONES UTILITARIAS
# ============================================================

def shape_name_to_code(shape_name: str) -> str:
    """
    Convierte el nombre completo de una forma a un código de una letra.
    
    Args:
        shape_name: Nombre de la forma ("square", "rectangle", "circle", "pentagon")
    
    Returns:
        str: Código de una letra:
            - "s" = square (cuadrado)
            - "r" = rectangle (rectángulo)
            - "c" = circle (círculo)
            - "p" = pentagon (pentágono)
            - "u" = unknown (desconocido)
    
    Ejemplo:
        >>> shape_name_to_code("square")
        's'
        >>> shape_name_to_code("circle")
        'c'
    """
    mapping = {
        "square": "s",
        "rectangle": "r",
        "circle": "c",
        "pentagon": "p",
    }
    return mapping.get(shape_name, "u")  # "u" si no está en el diccionario


def polar_to_cartesian_m(r_cm: float, theta_deg: float, offset_x_m: float = 0.0, offset_y_m: float = 0.0):
    """
    Convierte coordenadas polares (r, theta) a cartesianas (x, y).
    Incluye conversión de cm a metros y aplicación de offsets.
    
    Sistema de coordenadas:
    - theta = 0° apunta hacia +X (derecha)
    - theta crece en sentido antihorario (CCW)
    - 90° = +Y (arriba), 180° = -X (izquierda), 270° = -Y (abajo)
    
    Args:
        r_cm: Distancia radial en centímetros
        theta_deg: Ángulo en grados
        offset_x_m: Offset a sumar en X (metros)
        offset_y_m: Offset a sumar en Y (metros)
    
    Returns:
        tuple: (x_m, y_m) coordenadas en metros
    
    Ejemplo:
        >>> polar_to_cartesian_m(10.0, 0.0)    # 10cm a 0°
        (0.1, 0.0)  # 0.1m en +X
        >>> polar_to_cartesian_m(10.0, 90.0)   # 10cm a 90°
        (0.0, 0.1)  # 0.1m en +Y
    """
    # Convertir radio de cm a metros
    r_m = r_cm / 100.0
    
    # Convertir ángulo de grados a radianes
    th = math.radians(theta_deg)
    
    # Fórmulas de conversión polar -> cartesiano
    x = r_m * math.cos(th) + offset_x_m
    y = r_m * math.sin(th) + offset_y_m
    
    return x, y


def pick_best_detection(info: dict):
    """
    Selecciona la mejor detección de entre todas las figuras encontradas.
    
    Criterio de selección:
    1. Preferir figuras que NO sean "unknown"
    2. Entre las válidas, elegir la de mayor confianza
    3. Si todas son "unknown", elegir la de mayor confianza igual
    
    Args:
        info: Diccionario retornado por detectar_figuras_y_polares()
    
    Returns:
        dict: La mejor detección (un elemento de info["shapes"]),
              o None si no hay detecciones
    
    Ejemplo:
        >>> info = detectar_figuras_y_polares(frame)
        >>> best = pick_best_detection(info)
        >>> if best:
        ...     print(f"Detectado: {best['shape_name']} con confianza {best['confidence']}")
    """
    # Verificar que hay información válida
    if info is None or not info.get("shapes"):
        return None

    shapes = info["shapes"]
    
    # Filtrar solo las figuras conocidas (no "unknown")
    candidates = [s for s in shapes if s["shape_name"] != "unknown"]
    
    # Si no hay candidatos conocidos, usar todas las detecciones
    if not candidates:
        candidates = shapes

    # Seleccionar la de mayor confianza
    best = max(candidates, key=lambda s: s.get("confidence", 0.0))
    
    return best