# 🏎️ F1TENTH Autonomous Racing: Controlador Reactivo "Follow the Gap" (Circuito de Sepang)

🌐 **Idioma / Language:** [**Español**](README.md) | [**English**](README_EN.md)

---

## 📋 1. Descripción General del Proyecto

Este repositorio contiene el diseño, implementación y optimización de un controlador autónomo puramente reactivo basado en el algoritmo **Follow the Gap (FTG)** con **Extensor de Disparidades** y **Control Físico de Frenado Anticipado**, desarrollado sobre **ROS 2 (Humble / Jazzy)** para el simulador oficial **F1TENTH Gym**.

El controlador fue puesto a prueba en el exigente **Circuito Internacional de Sepang**, caracterizado por dos largas rectas paralelas conectadas mediante horquillas cerradas de $180^\circ$ y curvas compuestas de radio variable. El sistema completa de manera autónoma **10 vueltas consecutivas sin colisiones**, deteniéndose automáticamente al finalizar la prueba y reportando las métricas de rendimiento en tiempo real (~42.7 segundos por vuelta).

---

## 🧠 2. Descripción del Enfoque Utilizado (Follow the Gap Optimizado)

El método clásico *Follow the Gap* navega identificando el espacio libre más amplio frente al vehículo a partir del sensor LiDAR 2D (`/scan`). Sin embargo, a altas velocidades ($> 12\text{ m/s}$), el algoritmo básico sufre de inercia en las frenadas, cortes contra las esquinas interiores y desorientación en horquillas anchas.

Para superar estas limitaciones, se implementó una arquitectura reactiva mejorada de **7 etapas**:

1. **Filtrado y Recorte de Campo de Visión (FOV de $\pm 60^\circ$):**
   * Se aplica un filtro de media móvil (ventana de $5$ muestras) para eliminar ruido del LiDAR.
   * El campo de visión se restringe estrictamente al cono frontal de $\pm 60^\circ$. Esto garantiza geométricamente que el vehículo siempre evalúe trayectorias hacia adelante y jamás intente regresar en "U" en las horquillas de $180^\circ$.
2. **Extensor de Disparidades (*Disparity Extender*):**
   * Detecta saltos bruscos entre lecturas consecutivas del LiDAR ($|\Delta r| > 0.50\text{ m}$), los cuales corresponden a vértices o esquinas internas de los muros.
   * Enmascara un arco angular equivalente al ancho físico del vehículo más un margen de seguridad ($w = 0.52\text{ m}$):
     $$\Delta \theta = \arctan\left(\frac{w}{d_{\min}}\right)$$
   * Esto obliga al auto a abrir su trazada antes de doblar, evitando impactos laterales contra los vértices.
3. **Burbuja de Seguridad Dinámica:**
   * Localiza el punto más cercano del entorno ($d_{\min}$) y anula a $0.0\text{ m}$ todos los rayos dentro de un radio angular proporcional para crear una zona de exclusión alrededor del obstáculo más próximo.
4. **Selección de Gap Profundo con Continuidad Temporal:**
   * Filtra únicamente los espacios cuya profundidad supere el $55\%$ del alcance máximo visible ($\text{horizonte} = 11.0\text{ m}$) y selecciona el hueco óptimo penalizando cambios violentos respecto al ángulo previo del volante.
5. **Objetivo Ponderado y Centro de Meseta (*Plateau Centering*):**
   * En lugar de usar `argmax` simple (que sesga el vehículo hacia un borde cuando múltiples rayos saturan a $11.0\text{ m}$), calcula el centro geométrico de la región más profunda y lo combina con el centro del hueco ($60\%$ centro / $40\%$ fondo).
6. **Escudo Lateral Antiroce:**
   * Monitorea sectores laterales de $28^\circ$ a izquierda y derecha. Si el chasis se aproxima a menos de $0.70\text{ m}$ de un muro, inyecta una corrección proporcional suave para mantener el auto centrado en el carril.
7. **Control Dual de Velocidad (Frenado Físico Anticipado + Límite por Ángulo):**
   * Un cono láser frontal de $\pm 2.5^\circ$ (hasta $30\text{ m}$ de alcance) mide la distancia libre real frente a la trompa ($d_{\text{eff}}$).
   * La velocidad máxima permitida por distancia de frenado sigue la ley cinemática de desaceleración:
     $$v_{\text{freno}} = \min\left(15.5,\; 2.6 + 3.28\sqrt{\max(0,\; d_{\text{eff}} - 1.5)}\right)$$
   * Esto permite alcanzar **$15.5\text{ m/s}$** en las rectas largas de Sepang y clavar los frenos progresivamente antes de entrar a las curvas cerradas ($2.85\text{ m/s}$).

---

## 📂 3. Estructura del Código y del Repositorio

```text
F1Tenth-Repository/
├── README.md                                 # Tutorial y documentación en Español
├── README_EN.md                              # Tutorial y documentación en Inglés
└── src/
    ├── f1tenth_gym_ros/                      # Paquete del simulador puente ROS 2 y RViz2
    │   ├── config/
    │   │   └── sim.yaml                      # Configuración del mapa (apuntando a Sepang)
    │   └── maps/
    │       ├── Sepang.png                    # Mapa de ocupación de la pista de Sepang
    │       └── Sepang.yaml                   # Metadatos de escala y origen del mapa
    └── controllers/                          # Paquete de controladores autónomos
        ├── package.xml                       # Dependencias del paquete ROS 2
        ├── setup.py                          # Registro del punto de entrada (gap_node)
        └── controllers/
            └── gap_node.py                   # Código fuente principal del controlador FTG
```

### Partes Esenciales de `gap_node.py`

* **`__init__(self)`**: Inicializa el nodo `gap_node`, crea el publicador hacia `/drive` (`AckermannDriveStamped`) y las suscripciones a `/scan` (`LaserScan`) y `/ego_racecar/odom` (`Odometry`).
* **`odom_callback(self, msg)`**: Calcula la distancia real recorrida paso a paso ($\sum \sqrt{\Delta x^2 + \Delta y^2}$). Valida que una vuelta solo sea contabilizada cuando el vehículo regresa a menos de $1.8\text{ m}$ de la línea de meta habiendo recorrido al menos $55.0\text{ m}$ de pista. Al cumplir la vuelta 10, envía velocidad $0.0\text{ m/s}$, imprime el resumen estadístico y cierra el nodo limpiamente.
* **`scan_callback(self, msg)`**: Ejecuta en tiempo real (~40 Hz) las 7 etapas del algoritmo *Follow the Gap*, publicando el ángulo de dirección (`steering_angle`) y la velocidad longitudinal (`speed`).

---

## 🛠️ 4. Tutorial de Instalación y Ejecución en Otra Computadora (Ubuntu)

Sigue estos pasos para descargar, compilar y ejecutar este proyecto en cualquier computadora con **Ubuntu 22.04 (ROS 2 Humble)** o **Ubuntu 24.04 (ROS 2 Jazzy)**.

### Paso 4.1: Instalar dependencias del sistema y el motor `f1tenth_gym`

Abre una terminal (`Ctrl + Alt + T`) y ejecuta:

```bash
sudo apt update
sudo apt install -y git python3-pip python3-colcon-common-extensions

# Clonar e instalar el entorno físico base de F1TENTH Gym
cd $HOME
git clone https://github.com/f1tenth/f1tenth_gym
cd f1tenth_gym
pip3 install -e .
pip3 install transforms3d numpy
```

### Paso 4.2: Clonar este repositorio y ajustar la ruta del mapa

Reemplaza `<TU_USUARIO_GITHUB>` y `<NOMBRE_DE_TU_REPO>` con la URL de tu repositorio:

```bash
cd $HOME
git clone https://github.com/<TU_USUARIO_GITHUB>/<NOMBRE_DE_TU_REPO>.git F1Tenth-Repository
cd ~/F1Tenth-Repository
```

Actualiza automáticamente la ruta del archivo `sim.yaml` para que coincida con el nombre de usuario de la computadora actual (`$USER`):

```bash
sed -i "s|/home/.*/F1Tenth-Repository|$HOME/F1Tenth-Repository|g" ~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
```

### Paso 4.3: Compilar el Workspace con `colcon`

```bash
cd ~/F1Tenth-Repository
# En Ubuntu 22.04 usa humble | En Ubuntu 24.04 usa jazzy
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

---

## 🚀 5. Instrucciones de Ejecución de la Prueba (10 Vueltas)

Para iniciar la carrera necesitas abrir **dos terminales**.

### Terminal 1: Lanzar el Simulador y RViz2
```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Terminal 2: Ejecutar el Controlador Reactivo (`gap_node`)
Abre una segunda terminal y ejecuta:
```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run controllers gap_node
```

---

## 📊 6. Ejemplo de Salida en Terminal y Resultados Obtenidos

Durante la simulación, la **Terminal 2** reporta cada vuelta completada y detiene automáticamente la prueba en la vuelta 10:

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