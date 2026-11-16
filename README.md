<div align="center">

  # 🏎️ Competencia de Controladores Reactivos en F1Tenth
  ### **Algoritmo Follow the Gap Optimizado — Circuito Internacional de Sepang (ROS 2 Humble)**

  <p>
    <b>Autor:</b> Christian Raphael Aguilera López | <b>Ingeniería en Mecatrónica (FIMCP - ESPOL)</b>
  </p>

  <!-- BOTONES DE DOCUMENTACIÓN EN ESPAÑOL E INGLÉS -->
  <a href="https://github.com/ChristiaAguiilera/Competencia-de-Controladores-Reactivos-en-F1Tenth/blob/main/documentaci_n_en_espa_ol_readme_md.md">
    <img src="https://img.shields.io/badge/🇪🇸_Leer_Documentación-Español-D92626?style=for-the-badge" alt="Documentación en Español"/>
  </a>
    
  <a href="https://github.com/ChristiaAguiilera/Competencia-de-Controladores-Reactivos-en-F1Tenth/blob/main/english_documentation_readme_en_md.md">
    <img src="https://img.shields.io/badge/🇺🇸_Read_Documentation-English-0052CC?style=for-the-badge" alt="English Documentation"/>
  </a>
    
  <a href="https://youtu.be/6gFwV3bl328">
    <img src="https://img.shields.io/badge/▶️_Ver_Simulación-YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white" alt="YouTube Video"/>
  </a>

</div>

---

## 📖 Descripción General del Proyecto / Project Overview

🇪🇸 **Español:** Este repositorio presenta el diseño, implementación y ajuste fino de un controlador autónomo reactivo basado en el método **Follow the Gap (FTG)** sobre **ROS 2 Humble** y el simulador **F1Tenth Gym**. El sistema procesa lecturas LiDAR 2D (`/scan`) y odometría (`/ego_racecar/odom`) en tiempo real para navegar de forma 100% autónoma a través de las rectas de alta velocidad y horquillas cerradas del **Circuito Internacional de Sepang**, completando las **10 vueltas reglamentarias sin colisiones** con un tiempo récord de **~43 segundos por vuelta**.

🇺🇸 **English:** This repository presents the design, implementation, and fine-tuning of a reactive autonomous controller based on the **Follow the Gap (FTG)** method running on **ROS 2 Humble** and the **F1Tenth Gym** simulator. The system processes 2D LiDAR scans (`/scan`) and odometry (`/ego_racecar/odom`) in real time to autonomously navigate the high-speed straights and tight hairpins of the **Sepang International Circuit**, completing all **10 required laps collision-free** with a best lap time of **~43 seconds**.

---

## 🎥 Video Demostrativo (10/10 Vueltas en Sepang)

Haz clic en la vista previa inferior o en el enlace directo para reproducir la prueba completa en YouTube:

* 🔗 **Enlace directo al video:** [https://youtu.be/6gFwV3bl328](https://youtu.be/6gFwV3bl328)

<div align="center">
  <a href="https://youtu.be/6gFwV3bl328" target="_blank">
    <img src="https://img.youtube.com/vi/6gFwV3bl328/maxresdefault.jpg" alt="Demostración F1Tenth Sepang - 10 Vueltas" width="85%" style="border-radius: 10px; border: 2px solid #333;" />
  </a>
  <br>
  <sub><i>👆 Haz clic sobre la imagen para reproducir el video de la simulación en YouTube (https://youtu.be/6gFwV3bl328)</i></sub>
</div>

---

## 📊 Resumen de Rendimiento Logrado / Performance Summary

| Métrica / Metric | Resultado Obtenido / Result | Estado / Status |
| :--- | :---: | :---: |
| **Pista de Prueba / Track** | Circuito Internacional de Sepang | ✅ Configurado |
| **Vueltas Completadas / Completed Laps** | **10 / 10 Vueltas** (Parada automática en meta) | ✅ 100% Completado |
| **Mejor Tiempo de Vuelta / Best Lap Time** | **~43.24 s** | 🏆 Óptimo |
| **Velocidad Punta en Recta / Top Speed** | **15.50 m/s** | 🚀 Alta Velocidad |
| **Registro de Colisiones / Collisions** | **0 Colisiones** | 🛡️ 100% Limpio |

---

## 📚 Manuales y Tutoriales Completos / Full Documentation

Selecciona tu idioma preferido para acceder al **tutorial paso a paso de instalación en Ubuntu**, la **arquitectura del algoritmo (`gap_node.py`)**, las **ecuaciones de frenado dinámico** y las **instrucciones de ejecución**:

* 🇪🇸 **[Ir al Tutorial y Documentación Completa en Español](./documentaci_n_en_espa_ol_readme_md.md)**
  *(Enlace directo: `https://github.com/ChristiaAguiilera/Competencia-de-Controladores-Reactivos-en-F1Tenth/blob/main/documentaci_n_en_espa_ol_readme_md.md`)*

* 🇺🇸 **[Go to the Full Tutorial and Documentation in English](./english_documentation_readme_en_md.md)**
  *(Direct link: `https://github.com/ChristiaAguiilera/Competencia-de-Controladores-Reactivos-en-F1Tenth/blob/main/english_documentation_readme_en_md.md`)*

---

## 🛠️ Tecnologías Utilizadas / Tech Stack

* **Sistema Operativo:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **Middleware Robótico:** ROS 2 Humble Hawksbill (`rclpy`)
* **Simulador y Visualización:** F1Tenth Gym ROS Bridge + RViz2
* **Algoritmo de Navegación:** Reactive Follow the Gap + Disparity Extender + Dynamic Lookahead Braking (`Python 3` / `NumPy`)
