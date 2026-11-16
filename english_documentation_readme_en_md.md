<div align="right">
  <a href="https://github.com/ChristiaAguiilera/Competencia-de-Controladores-Reactivos-en-F1Tenth/blob/main/README.md"><img src="https://img.shields.io/badge/🏠_Inicio-Portada-444444?style=for-the-badge" alt="Inicio"/></a>
  <a href="https://github.com/ChristiaAguiilera/Competencia-de-Controladores-Reactivos-en-F1Tenth/blob/main/documentaci_n_en_espa_ol_readme_md.md"><img src="https://img.shields.io/badge/🇪🇸_Idioma-Español-D92626?style=for-the-badge" alt="Español"/></a>
  <a href="https://github.com/ChristiaAguiilera/Competencia-de-Controladores-Reactivos-en-F1Tenth/blob/main/english_documentation_readme_en_md.md"><img src="https://img.shields.io/badge/🇺🇸_Language-English-0052CC?style=for-the-badge" alt="English"/></a>
</div>



# 🏎️ F1TENTH Autonomous Racing: "Follow the Gap" Reactive Controller (Sepang Circuit)

🌐 **Language / Idioma:** [**Español**](README.md) | [**English**](README_EN.md)

---

## 📋 1. Project Overview

This repository contains the design, implementation, and optimization of a purely reactive autonomous racing controller based on the **Follow the Gap (FTG)** algorithm enhanced with a **Disparity Extender** and **Physics-Based Lookahead Braking**. It is built on **ROS 2 (Humble / Jazzy)** for the official **F1TENTH Gym** simulator.

The controller was benchmarked on the challenging **Sepang International Circuit**, which features two long parallel straights connected by tight $180^\circ$ hairpins and variable-radius compound corners. The system autonomously completes **10 consecutive collision-free laps**, automatically stopping the vehicle upon completion and reporting real-time telemetry metrics (~42.7 seconds per lap).

---

## 🧠 2. Reactive Approach Description (Optimized Follow the Gap)

The classical *Follow the Gap* method navigates by identifying the widest free space in front of the vehicle using 2D LiDAR scans (`/scan`). However, at high speeds ($> 12\text{ m/s}$), the baseline algorithm suffers from braking inertia, corner-cutting collisions, and U-turn disorientation inside wide hairpins.

To overcome these limitations, a **7-stage** enhanced reactive pipeline was implemented:

1. **Pre-processing & Strict Frontal Field of View ($\pm 60^\circ$ FOV):**
   * A 5-sample moving average convolution filter smooths raw LiDAR readings.
   * The Field of View is strictly bounded to $\pm 60^\circ$ in front of the car. This geometrically prevents the LiDAR from seeing backward inside $180^\circ$ hairpins, eliminating accidental U-turns.
2. **Disparity Extender:**
   * Detects sharp range jumps between adjacent LiDAR beams ($|\Delta r| > 0.50\text{ m}$), which indicate inner track corners or walls.
   * Masks an angular span corresponding to the car width plus a safety margin ($w = 0.52\text{ m}$):
     $$\Delta \theta = \arctan\left(\frac{w}{d_{\min}}\right)$$
   * This forces the vehicle to take a wider entry line around tight apexes, preventing inner-wall clipping.
3. **Dynamic Safety Bubble:**
   * Finds the closest obstacle point ($d_{\min}$) and zeroes out all beams within a dynamic angular radius to create a safe exclusion zone.
4. **Deep Gap Selection with Temporal Continuity:**
   * Filters contiguous free-space segments whose depth exceeds $55\%$ of the current visible horizon ($11.0\text{ m}$) and selects the optimal gap while penalizing abrupt jumps away from the previous steering angle.
5. **Weighted Target & Plateau Centering:**
   * Instead of a naive `argmax` (which biases the target toward one edge when multiple beams saturate at $11.0\text{ m}$), the controller computes the mean index of the deepest plateau region and blends it with the gap center ($60\%$ center / $40\%$ deepest plateau).
6. **Lateral Anti-Rubbing Shield:**
   * Monitors $28^\circ$ lateral sectors on both sides. If the vehicle approaches within $0.70\text{ m}$ of a side wall, a smooth proportional steering correction pushes the car back toward the centerline.
7. **Dual Velocity Control (Physics-Based Braking + Steering Angle Limit):**
   * A narrow $\pm 2.5^\circ$ frontal laser cone (up to $30\text{ m}$ range) measures the true unobstructed distance ahead ($d_{\text{eff}}$).
   * The maximum allowable braking speed follows kinematic deceleration laws:
     $$v_{\text{brake}} = \min\left(15.5,\; 2.6 + 3.28\sqrt{\max(0,\; d_{\text{eff}} - 1.5)}\right)$$
   * This enables top speeds of **$15.5\text{ m/s}$** on Sepang's long straights while progressively braking into tight hairpins ($2.85\text{ m/s}$).

---

## 📂 3. Code and Repository Structure

```text
F1Tenth-Repository/
├── README.md                                 # Spanish tutorial and documentation
├── README_EN.md                              # English tutorial and documentation
└── src/
    ├── f1tenth_gym_ros/                      # ROS 2 simulator bridge and RViz2 package
    │   ├── config/
    │   │   └── sim.yaml                      # Map configuration (set to Sepang)
    │   └── maps/
    │       ├── Sepang.png                    # Sepang track occupancy grid image
    │       └── Sepang.yaml                   # Map scale and origin metadata
    └── controllers/                          # Autonomous controllers package
        ├── package.xml                       # ROS 2 package dependencies
        ├── setup.py                          # Entry point registration (gap_node)
        └── controllers/
            └── gap_node.py                   # Main Reactive Follow the Gap source code
```

### Core Components of `gap_node.py`

* **`__init__(self)`**: Initializes the `gap_node` ROS 2 node, sets up the `/drive` publisher (`AckermannDriveStamped`), and subscribes to `/scan` (`LaserScan`) and `/ego_racecar/odom` (`Odometry`).
* **`odom_callback(self, msg)`**: Accumulates real traveled distance ($\sum \sqrt{\Delta x^2 + \Delta y^2}$). Ensures a lap is only counted when the car returns within $1.8\text{ m}$ of the starting pose after covering at least $55.0\text{ m}$ along the circuit. Upon completing Lap 10, it commands $0.0\text{ m/s}$ speed, prints the final performance summary, and cleanly exits the node.
* **`scan_callback(self, msg)`**: Executes the 7-stage *Follow the Gap* pipeline at ~40 Hz, publishing the computed `steering_angle` and longitudinal `speed`.

---

## 🛠️ 4. Installation & Setup Tutorial on Another Ubuntu PC

Follow these steps to download, build, and run this project from scratch on any computer running **Ubuntu 22.04 (ROS 2 Humble)** or **Ubuntu 24.04 (ROS 2 Jazzy)**.

### Step 4.1: Install system dependencies and `f1tenth_gym`

Open a terminal (`Ctrl + Alt + T`) and run:

```bash
sudo apt update
sudo apt install -y git python3-pip python3-colcon-common-extensions

# Clone and install the base F1TENTH Gym physics environment
cd $HOME
git clone https://github.com/f1tenth/f1tenth_gym
cd f1tenth_gym
pip3 install -e .
pip3 install transforms3d numpy
```

### Step 4.2: Clone this repository and configure the map path

Replace `<YOUR_GITHUB_USERNAME>` and `<YOUR_REPO_NAME>` with your repository URL:

```bash
cd $HOME
git clone https://github.com/<YOUR_GITHUB_USERNAME>/<YOUR_REPO_NAME>.git F1Tenth-Repository
cd ~/F1Tenth-Repository
```

Automatically update the map path inside `sim.yaml` to match the current computer's username (`$USER`):

```bash
sed -i "s|/home/.*/F1Tenth-Repository|$HOME/F1Tenth-Repository|g" ~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
```

### Step 4.3: Build the ROS 2 Workspace

```bash
cd ~/F1Tenth-Repository
# On Ubuntu 22.04 use humble | On Ubuntu 24.04 use jazzy
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

---

## 🚀 5. Running the 10-Lap Benchmark

Open **two terminals** to launch the simulation and the autonomous controller.

### Terminal 1: Launch the Simulator and RViz2
```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Terminal 2: Run the Reactive Controller (`gap_node`)
Open a second terminal and run:
```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run controllers gap_node
```

---

## 📊 6. Sample Terminal Output & Benchmark Results

During the run, **Terminal 2** logs telemetry for each completed lap and automatically stops after Lap 10:

```text
[INFO] [gap_node]: 🏁 Nodo FTG Iniciado - Sepang (42.8s Estable 10/10 Vueltas)
[INFO] [gap_node]: ==== [VUELTA 1/10 COMPLETADA] ====
[INFO] [gap_node]: ⏱️ Tiempo de vuelta: 43.52s | 🚀 Vel. Promedio: 6.78 m/s
[INFO] [gap_node]:  ✅ Estado: Sin colisión
...
[INFO] [gap_node]: ==== [VUELTA 10/10 COMPLETADA] ====
[INFO] [gap_node]: ⏱️ Tiempo de vuelta: 42.88s | 🚀 Vel. Promedio: 6.88 m/s
[INFO] [gap_node]:  ✅ Estado: Sin colisión
[INFO] [gap_node]: ==============================================================
[INFO] [gap_node]: 🎉 PRUEBA COMPLETADA: 10/10 Vueltas Finalizadas sin colisión.
[INFO] [gap_node]: ------------------- RESUMEN DE LOGROS ------------------------
[INFO] [gap_node]: 🥇 Mejor Vuelta Registrada : Vuelta #4 (42.74s)
[INFO] [gap_node]: ⏱️ Tiempo Promedio / Vuelta: 43.10s (Tiempo Total: 431.02s)
[INFO] [gap_node]: 🚀 Velocidad Promedio Total: 6.84 m/s
[INFO] [gap_node]: ✅ Desempeño de Seguridad  : 0 Colisiones (100% Limpio)
[INFO] [gap_node]: ==============================================================
```
