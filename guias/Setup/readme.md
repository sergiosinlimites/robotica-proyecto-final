# 🛠️ Setup para el Kit de Phantom X Pincher

Este repositorio requiere el uso de [Git LFS (Large File Storage)](https://git-lfs.com/) para gestionar archivos pesados como `.stp`, `.stl`, `.ipt`, entre otros, que superan los 100 MB y no pueden ser subidos directamente a GitHub.

---

## Índice

1. Configurar Git LFS para este repositorio  
   1.1 Instalación de Git LFS (Large File Storage)  
   1.2 Clasificación de tipos de archivos  
   1.3 Confirmar seguimiento con `.gitattributes`  
2. Instalar ROS 2 Jazzy en Ubuntu 24.04  
3. Hacer que ROS 2 siempre esté listo para usarse en terminal  
4. Instalación y configuración adicionales (“Configure the Development Environment in Ubuntu 24.04”)  
   4.1 Actualizar el sistema  
   4.2 Instalar un terminal mejorado (Terminator)  
   4.3 Paquetes ROS 2 necesarios usados en el curso  
   4.4 Paquetes de Python necesarios (Alexa e interconexión)  
   4.5 Paquetes adicionales para comunicación serie en C++  
5. Referencia rápida de “¿Qué hace cada línea?”

---

## 1. Configurar Git LFS para este repositorio

### 1.1 ✅ Instalación Git LFS (Large File Storage)

1. **Instalar Git LFS**  
   Abre la terminal y ejecuta:

   ```bash
   sudo apt update
   sudo apt install git-lfs
   ```

2. **Inicializar Git LFS** (solo una vez por repositorio):

   ```bash
   git lfs install
   ```

3. **Track de los distintos tipos de archivos**  
   Ejecuta el siguiente comando en la raíz del repositorio:

   ```bash
   git lfs track "*.ipt" "*.iam" "*.idw" "*.ipn" "*.stl" "*.step" "*.stp" "*.igs" "*.iges" "*.dwg" "*.dxf" "*.3ds" "*.obj" "*.fbx" "*.mp4" "*.mov" "*.avi" "*.mkv" "*.zip" "*.rar" "*.7z"
   ```

---

### 1.2 📂 Clasificación de tipos de archivos

| Categoría      | Extensiones                                                                      |
|----------------|----------------------------------------------------------------------------------|
| **Inventor**   | `.ipt`, `.iam`, `.idw`, `.ipn`                                                   |
| **CAD/3D**     | `.stl`, `.step`, `.stp`, `.igs`, `.iges`, `.dwg`, `.dxf`, `.3ds`, `.obj`, `.fbx` |
| **Video**      | `.mp4`, `.mov`, `.avi`, `.mkv`                                                   |
| **Compresión** | `.zip`, `.rar`, `.7z`                                                            |

---

### 1.3 🧩 Confirmar seguimiento con `.gitattributes`

1. Asegúrate de estar en la raíz del repositorio.
2. Agrega el archivo `.gitattributes` al repositorio:

   ```bash
   git add .gitattributes
   git commit -m "Tracking CAD, video, and archive files with Git LFS"
   ```

---

> ⚠️ **Importante:** Asegúrate de hacer `git add` a los archivos rastreados con LFS **después** de configurar el `git lfs track`, para que el seguimiento sea efectivo.

---

## 2. Instalar ROS 2 Jazzy en Ubuntu 24.04

Para instalar **ROS 2 Jazzy** en **Ubuntu 24.04**, visita el siguiente enlace y sigue los pasos indicados:

👉 [Guía oficial de instalación de ROS 2 Jazzy en Ubuntu](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

> ⚠️ **Importante:** Se debe usar la opción **"Desktop Install"** y **NO** la opción **"ros-base (Bare Bones)"**.

---

## 3. Hacer que ROS 2 siempre esté listo para usarse en terminal

1. Abrir el terminal y usar el comando `cd` para ir a home:
   ```bash
   cd
   ```
2. En home usar el siguiente comando para abrir el archivo `.bashrc` y poder modificarlo:
   ```bash
   sudo gedit .bashrc
   ```
3. Al final del archivo `.bashrc` añadir el `source` con la ruta de ROS 2 instalado:
   ```bash
   # Inicializar siempre ROS 2
   source /opt/ros/jazzy/setup.bash
   ```
4. Guardar el archivo y cerrarlo.

---

## 4. 🧰 Instalación y configuración adicionales  (“Configure the Development Environment in Ubuntu 24.04”)

> Estas instrucciones complementan las ya presentes en este repositorio. No reemplazan ni modifican nada de lo anterior; solo añaden los pasos de instalación que se usan en las clases.

### 4.1 Actualizar el sistema

```bash
sudo apt-get update && sudo apt-get upgrade
```
- **¿Qué hace?** Actualiza el índice de paquetes de Ubuntu y aplica las actualizaciones disponibles para mantener el sistema al día.

---

### 4.2 Instalar un terminal mejorado (Terminator)

```bash
sudo apt-get install terminator
```
- **¿Qué hace?** Instala **Terminator**, un emulador de terminal que permite dividir paneles, guardar disposiciones y trabajar con varias consolas en una sola ventana.

---

### 4.3 Paquetes ROS 2 necesarios usados en el curso

Ejecuta cada línea (puedes pegarlas una por una o todas juntas).

```bash
sudo apt-get install ros-jazzy-ros2-control
sudo apt-get install ros-jazzy-ros2-controllers
sudo apt-get install ros-jazzy-xacro
sudo apt-get install ros-jazzy-ros-gz-*
sudo apt-get install ros-jazzy-*-ros2-control
sudo apt-get install ros-jazzy-joint-state-publisher-gui
sudo apt-get install ros-jazzy-tf-transformations
sudo apt-get install ros-jazzy-moveit*
```

- **ros-jazzy-ros2-control**: Framework para administrar hardware y controladores en ROS 2 (back-end de `ros2_control`).
- **ros-jazzy-ros2-controllers**: Conjunto de controladores listos (p. ej., `joint_state_broadcaster`, `joint_trajectory_controller`) que usarás en simulación y/o hardware real.
- **ros-jazzy-xacro**: Procesador de macros Xacro para generar URDF a partir de archivos `.xacro`.
- **ros-jazzy-ros-gz-***: Paquetes de integración entre ROS 2 y Gazebo (puentes de tópicos, utilidades, etc.). El comodín `*` instala los subpaquetes relevantes.
- **ros-jazzy-*-ros2-control**: Instala paquetes relacionados con `ros2_control` para diferentes integraciones (plugins/adaptadores). El comodín `*` permite que APT resuelva todos los que apliquen.
- **ros-jazzy-joint-state-publisher-gui**: Interfaz gráfica para publicar estados de articulaciones y probar cinemática sin hardware.
- **ros-jazzy-tf-transformations**: Utilidades para transformaciones espaciales (TF) en Python.
- **ros-jazzy-moveit***: Instala MoveIt y sus componentes (planificación de movimiento, RViz plugins, etc.).

> 💡 **Instalación en un solo comando (opcional):**
> ```bash
> sudo apt-get install -y >   ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro >   ros-jazzy-ros-gz-* ros-jazzy-*-ros2-control >   ros-jazzy-joint-state-publisher-gui ros-jazzy-tf-transformations >   ros-jazzy-moveit*
> ```

---

### 4.4 Paquetes de Python necesarios (Alexa e interconexión)

```bash
sudo apt-get install python3-pip
pip install transforms3d
pip install flask
pip install pyserial
pip install flask-ask-sdk
pip install ask-sdk
```

- **python3-pip**: Gestor de paquetes para Python.
- **transforms3d**: Transformaciones 3D (rotaciones, cuaterniones, matrices) útiles para robótica.
- **flask**: Micro-framework web para crear servicios y dashboards ligeros.
- **pyserial**: Comunicación serie en Python; útil para hablar con microcontroladores (p. ej., Arduino).
- **flask-ask-sdk / ask-sdk**: SDKs para integrar Alexa Skills con aplicaciones Python/Flask.

> ⚠️ Si tu `pip` apunta a Python 2 (poco común en 22.04 o 24.04), usa `pip3` en su lugar.

---

### 4.5 Paquetes adicionales para comunicación serie en C++

```bash
sudo apt-get install libserial-dev
```

- **¿Qué hace?** Instala los headers y librerías de **libserial** para aplicaciones C++ que se comuniquen por puerto serie (p. ej., ROS 2 ↔ Arduino).

---

## 5. 📌 Referencia de “¿Qué hace cada línea?” (resumen rápido)

- `sudo apt-get update && sudo apt-get upgrade`: sincroniza índices e instala actualizaciones.
- `sudo apt-get install terminator`: emulador de terminal con paneles múltiples.
- `sudo apt-get install ros-jazzy-ros2-control`: instala el back-end de `ros2_control`.
- `sudo apt-get install ros-jazzy-ros2-controllers`: controladores genéricos (broadcasters/controladores de articulaciones).
- `sudo apt-get install ros-jazzy-xacro`: utilidades Xacro para generar URDF.
- `sudo apt-get install ros-jazzy-ros-gz-*`: puente e integración ROS 2 ↔ Gazebo.
- `sudo apt-get install ros-jazzy-*-ros2-control`: complementos vinculados a `ros2_control`.
- `sudo apt-get install ros-jazzy-joint-state-publisher-gui`: GUI para publicar estados articulares.
- `sudo apt-get install ros-jazzy-tf-transformations`: helpers de transformaciones TF en Python.
- `sudo apt-get install ros-jazzy-moveit*`: instala MoveIt y sus componentes.
- `sudo apt-get install python3-pip`: gestor de paquetes Python 3.
- `pip install ...`: instala paquetes Python listados (transforms3d, flask, pyserial, Alexa SDKs).
- `sudo apt-get install libserial-dev`: librería C++ para comunicación serie.