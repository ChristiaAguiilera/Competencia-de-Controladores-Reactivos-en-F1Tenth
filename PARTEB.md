<div align="right">
  <a href="https://github.com/ChristiaAguiilera/Competencia-de-Controladores-Reactivos-en-F1Tenth/blob/main/README.md"><img src="https://img.shields.io/badge/🏠_Inicio-Portada-444444?style=for-the-badge" alt="Inicio"/></a>
  <a href="https://github.com/ChristiaAguiilera/Competencia-de-Controladores-Reactivos-en-F1Tenth/blob/main/documentaci_n_en_espa_ol_readme_md.md"><img src="https://img.shields.io/badge/🇪🇸_Parte_A-Pista_Limpia-D92626?style=for-the-badge" alt="Parte A"/></a>
  <a href="https://github.com/ChristiaAguiilera/Competencia-de-Controladores-Reactivos-en-F1Tenth/blob/main/english_documentation_readme_en_md.md"><img src="https://img.shields.io/badge/🇺🇸_Language-English-0052CC?style=for-the-badge" alt="English"/></a>
  <a href="https://youtu.be/wi7857e5MLQ"><img src="https://img.shields.io/badge/▶️_Video_Parte_B-YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white" alt="YouTube Parte B"/></a>
</div>

# 🏆 Parte B: Competencia con Obstáculos Estáticos y Dinámicos (Multi-Agente)

En esta segunda fase del proyecto se evalúa la robustez, capacidad de evasión y estabilidad del controlador reactivo **Follow the Gap (FTG)** bajo condiciones de competencia en el **Circuito Internacional de Sepang**, superando dos escenarios exigentes de **10 vueltas consecutivas sin colisión**:

1. **Prueba B1 — Evasión de 5 Obstáculos Estáticos (`Sepang_obs_map`):** Navegación autónoma esquivando bloques físicos distribuidos en rectas y salidas de curva.
2. **Prueba B2 — Adelantamiento a Obstáculo Dinámico Multi-Agente (`num_agent: 2`):** Carrera simultánea con un segundo vehículo autónomo (`opp_racecar`) posicionado **5.0 metros** por delante en la grilla de salida, ejecutando maniobras de rebase limpio durante las 10 vueltas reglamentarias.

---

## 🎥 Video Demostrativo — Parte B (Obstáculos Estáticos y Rebase Multi-Agente)

Haz clic en la vista previa inferior o en el enlace directo para reproducir la demostración de la Parte B en YouTube:

* 🔗 **Enlace directo al video:** [https://youtu.be/wi7857e5MLQ](https://youtu.be/wi7857e5MLQ)

<div align="center">
  <a href="https://youtu.be/wi7857e5MLQ" target="_blank">
    <img src="https://img.youtube.com/vi/wi7857e5MLQ/maxresdefault.jpg" alt="Demostración F1Tenth Parte B - Obstáculos y Multi-Agente" width="85%" style="border-radius: 10px; border: 2px solid #333;" />
  </a>
  <br>
  <sub><i>👆 Haz clic sobre la imagen para reproducir el video de la Parte B en YouTube (https://youtu.be/wi7857e5MLQ)</i></sub>
</div>

---

## 🛠️ 1. Generación del Mapa con 5 Obstáculos Estáticos (`GIMP`)

Para simular barreras físicas detectables por el sensor LiDAR 2D (`/scan`), se editó el mapa de ocupación en escala de grises de Sepang mediante **GIMP**:

* **Duplicación segura del mapa:** Se crearon `Sepang_obs_map.png` y `Sepang_obs_map.yaml` a partir de `Sepang_map` para conservar intacta la pista base.
* **Edición a nivel de píxel:** Utilizando la herramienta **Lápiz (`Pencil Tool`)** con dureza al `100%` y color negro puro (`#000000`, que el servidor de mapas de ROS 2 interpreta como celda ocupada/muro sólido), se dibujaron **5 obstáculos** estratégicamente ubicados en rectas y zonas de transición, garantizando un ancho libre mínimo superior al 65% del carril para permitir la maniobra de evasión.

---

## 🧠 2. Mejoras Algorítmicas Implementadas para la Parte B

Para evitar colisiones o salidas de pista cuando un obstáculo estático coincide con el adelantamiento al segundo vehículo, se reforzó la arquitectura de `gap_node.py` y se desarrolló el nodo oponente `opp_gap_node.py`:

| Módulo / Parámetro | Ajuste Implementado | Propósito en Competencia |
| :--- | :---: | :--- |
| **Campo de Visión (FOV)** | `±55°` (`110°` totales) | Evita saltos bruscos hacia los muros laterales al rodear un vehículo en plena recta. |
| **Extensor de Disparidades** | `car_width_margin = 0.58 m` | Proyecta un margen de seguridad alrededor de los 5 bloques negros y del chasis del `opp_racecar`. |
| **Freno Preventivo de Rebase** | `effective_dist = min(nose_dist, target_dist)` | Desacelera proporcionalmente si el robot oponente y un obstáculo bloquean temporalmente el frente. |
| **Escudo Lateral Antiroce** | Umbral `< 0.78 m` (`k = 0.14`) | Corrige la trayectoria si el auto se aproxima demasiado al borde exterior durante un adelantamiento. |
| **Controlador Oponente (`opp_gap_node`)** | Tope `12.0 m/s` (~82% del ritmo principal) | Mantiene un flujo rápido en curvas sin estorbar, permitiendo rebases limpios en las rectas largas. |

---

## ⚙️ 3. Configuración del Simulador (`sim.yaml`)

Para habilitar ambos vehículos con una separación inicial de **5.0 metros** sobre la recta principal en `~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml`:

```yaml
    map_path: '/home/christian/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Sepang_obs_map'
    map_img_ext: '.png'
    num_agent: 2
    sx: 0.0
    sy: 0.0
    stheta: 0.0
    sx1: 5.0
    sy1: 0.0
    stheta1: 0.0
