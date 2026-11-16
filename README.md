<div align="right">
  <a href="./README.md"><img src="https://img.shields.io/badge/Idioma-Español-red?style=for-the-badge" alt="Español"></a>
  <a href="./README_EN.md"><img src="https://img.shields.io/badge/Language-English-blue?style=for-the-badge" alt="English"></a>
</div>

# 🏎️ Controlador Reactivo Follow the Gap (FTG) — Circuito de Sepang (F1Tenth ROS 2)

![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat-square&logo=ros)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04_LTS-E95420?style=flat-square&logo=ubuntu)
![Python 3.10](https://img.shields.io/badge/Python-3.10-3776AB?style=flat-square&logo=python)
![Status](https://img.shields.io/badge/Prueba_10_Vueltas-100%25_Sin_Colisiones-success?style=flat-square)

Este repositorio presenta el diseño, implementación y validación de un **controlador autónomo puramente reactivo** basado en el algoritmo **Follow the Gap (FTG)** mejorado con **Extensor de Disparidades (Disparity Extender)** y **Frenado Físico Anticipado**, desarrollado en **ROS 2 Humble** para el simulador `f1tenth_gym_ros` sobre el circuito internacional de **Sepang**.

El sistema completa de forma autónoma las **10 vueltas reglamentarias sin colisiones**, alcanzando tiempos de vuelta consistentes de **~42.9s – 43.2s** y deteniéndose automáticamente al cruzar la meta en la décima vuelta.

---

## 🎥 Demostración en Video y Evidencia de Ejecución

🔗 **Enlace directo al video en YouTube:** [https://youtu.be/Th56oVOA0Pc](https://youtu.be/Th56oVOA0Pc)

<div align="center">
  <a href="https://youtu.be/Th56oVOA0Pc" target="_blank">
    <img src="https://img.youtube.com/vi/Th56oVOA0Pc/maxresdefault.jpg" alt="Ver Video de Demostración en YouTube" width="750" style="border-radius: 10px;">
  </a>
  <br>
  <a href="https://youtu.be/Th56oVOA0Pc" target="_blank">
    <img src="https://img.shields.io/badge/▶️_Reproducir_Video_en_YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white" alt="Ver en YouTube">
  </a>
</div>

### 🎬 Video Integrado Directamente en el Repositorio
*(Para reproducir sin salir de GitHub: al editar este README, arrastra y suelta aquí abajo tu archivo de video `.mp4`)*

<!-- ARRASTRA Y SUELTA TU ARCHIVO DE VIDEO .MP4 AQUÍ DEBAJO -->


---

## 🧠 1. Descripción del Enfoque Utilizado (Follow the Gap Mejorado)

El algoritmo clásico **Follow the Gap (FTG)** navega identificando el espacio libre más amplio en las lecturas del sensor LiDAR 2D (`/scan`). Sin embargo, debido a que el circuito de **Sepang** combina dos rectas largas de alta velocidad con horquillas cerradas de $180^\circ$ y curvas de radio variable, se implementaron **cinco mejoras clave de ingeniería**:

1. **Recorte de Campo de Visión Antirretorno ($\pm 60^\circ$):** Se filtra el escaneo LiDAR exclusivamente al cono frontal de $120^\circ$ totales. Esto evita que el vehículo detecte el espacio libre detrás de sí mismo al girar en horquillas anchas, eliminando por completo los giros en U.
2. **Extensor de Disparidades (*Disparity Extender*):** Detecta saltos bruscos entre lecturas consecutivas del LiDAR ($\vert{}\Delta r\vert{} > 0.50\text{ m}$), los cuales corresponden a vértices o esquinas interiores de las curvas. El algoritmo proyecta el ancho físico del chasis con margen de seguridad (`0.52 m`) sobre el lado más lejano de la esquina para evitar que las ruedas interiores muerdan el muro.
3. **Burbuja de Seguridad Dinámica (*Safety Bubble*):** Identifica el punto más cercano al vehículo ($r_{\min}$) y anula a `0.0 m` un radio angular proporcional ($\theta = \arctan(0.47 / r_{\min})$), obligando al vehículo a rodear cualquier obstáculo inminente.
4. **Selección de Gap con Continuidad y Centro de Meseta (*Plateau Centering*):** En lugar de apuntar a un borde cuando múltiples rayos alcanzan el horizonte máximo de decisión (`11.0 m`), el controlador calcula el centro geométrico de la región más profunda (ponderando **60% centro del hueco** y **40% zona más profunda**) y aplica un escudo lateral antiroce cuando las paredes se acercan a menos de `0.70 m`.
5. **Control Dual de Velocidad y Frenado Físico Anticipado:**
   * En las rectas despejadas permite acelerar hasta **15.5 m/s**.
   * Antes de entrar a cada curva, evalúa un cono láser frontal ($\pm 2.5^\circ$) y aplica una ley física de desaceleración proporcional a la raíz cuadrada de la distancia disponible:
     $$v_{\text{frenado}} = 2.6 + 3.28\sqrt{\max(0,\; d_{\text{efectiva}} - 1.5)}$$
   * Finalmente, selecciona el mínimo entre la velocidad de frenado frontal y la velocidad máxima permitida por el ángulo de dirección (`speed = min(speed_by_braking, speed_by_angle)`).

---

## 📂 2. Estructura del Proyecto y del Código

```text
F1Tenth-Repository/
├── README.md                  # Documentación y tutorial en Español (Portada)
├── README_EN.md               # Documentación y tutorial en Inglés
└── src/
    ├── f1tenth_gym_ros/       # Simulador puente ROS 2 + RViz2
    │   ├── config/
    │   │   └── sim.yaml       # Configuración de la ruta del mapa Sepang
    │   └── maps/
    │       ├── Sepang.png     # Imagen ocupacional del circuito Sepang
    │       └── Sepang.yaml    # Metadatos de escala y origen del mapa
    └── controllers/           # Paquete del controlador autónomo
        ├── package.xml
        ├── setup.py           # Punto de entrada: gap_node = controllers.gap_node:main
        └── controllers/
            └── gap_node.py    # Nodo principal ReactiveFollowGap
