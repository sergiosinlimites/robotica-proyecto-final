# 🛠️ Setup para el Curso

Este repositorio requiere el uso de [Git LFS (Large File Storage)](https://git-lfs.com/) para gestionar archivos pesados como `.stp`, `.stl`, `.ipt`, entre otros, que superan los 100 MB y no pueden ser subidos directamente a GitHub.

---

### ✅ Instalación Git LFS (Larfe file Storage)

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
   git lfs track "*.ipt" "*.iam" "*.idw" "*.ipn" "*.stl" "*.step" "*.stp" "*.igs" "*.iges" "*.dwg" "*.dxf" "*.3ds" "*.obj" "*.fbx" "*.mp4" "*.mov" "*.avi" "*.mkv" "*.zip" "*.rar" "*.7z" "*.dae"
   ```

---

### 📂 Clasificación de tipos de archivos

| Categoría      | Extensiones                                                                               |
|----------------|-------------------------------------------------------------------------------------------|
| **Inventor**   | `.ipt`, `.iam`, `.idw`, `.ipn`                                                            |
| **CAD/3D**     | `.stl`, `.step`, `.stp`, `.igs`, `.iges`, `.dwg`, `.dxf`, `.3ds`, `.obj`, `.fbx` , `.dae` |
| **Video**      | `.mp4`, `.mov`, `.avi`, `.mkv`                                                            |
| **Compresión** | `.zip`, `.rar`, `.7z`                                                                     |

---

### 🧩 Confirmar seguimiento con `.gitattributes`

1. Asegúrate de estar en la raíz del repositorio.
2. Agrega el archivo `.gitattributes` al repositorio:

   ```bash
   git add .gitattributes
   git commit -m "Tracking CAD, video, and archive files with Git LFS"
   ```

---

> ⚠️ **Importante:** Asegúrate de hacer `git add` a los archivos rastreados con LFS **después** de configurar el `git lfs track`, para que el seguimiento sea efectivo.

---

## Instalar ROS2 Humble en Ubuntu 22.04

Para instalar **ROS2 Humble** en **Ubuntu 22.04**, visita el siguiente enlace y sigue los pasos indicados:

👉 [Guía oficial de instalación de ROS2 Humble en Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

> ⚠️ **Importante:** Se debe usar la opción **"Desktop Install"** y **NO** la opción **"ros-base (Bare Bones)"**.

## Hacer que ROS2 siempre este listo para usarse en terminal

1. Abrir el terminal y usar el comando cd para ir a home
```
cd
```
2. En home usar el siguiente comando para abrir el archivo .bashrc y poder modificarlo
```
sudo gedit .bashrc
```
3. Al final del archivo .bashrc añadir el source con la ruta de ros2 instalado
```
#Inicializar siempre ros2 
source /opt/ros/humble/setup.bash
```
4. Guardar el archivo y cerrarlo

---

## 🧰 Instalación y configuración adicionales  (“Configure the Development Environment in Ubuntu 22.04”)

> Estas instrucciones complementan las ya presentes en este repositorio. No reemplazan ni modifican nada de lo anterior; solo añaden los pasos de instalación que se usan en las clases.

### 1) Actualizar el sistema
```bash
sudo apt-get update && sudo apt-get upgrade
```
- **¿Qué hace?** Actualiza el índice de paquetes de Ubuntu y aplica las actualizaciones disponibles para mantener el sistema al día.

### 2) Instalar un terminal mejorado (Terminator)
```bash
sudo apt-get install terminator
```
- **¿Qué hace?** Instala **Terminator**, un emulador de terminal que permite dividir paneles, guardar disposiciones y trabajar con varias consolas en una sola ventana.

### 3) Paquetes ROS 2 necesarios usados en el curso
Ejecuta cada línea (puedes pegarlas una por una o todas juntas).

```bash
sudo apt-get install ros-humble-ros2-control
sudo apt-get install ros-humble-ros2-controllers
sudo apt-get install ros-humble-xacro
sudo apt-get install ros-humble-ros-gz-*
sudo apt-get install ros-humble-*-ros2-control
sudo apt-get install ros-humble-joint-state-publisher-gui
sudo apt-get install ros-humble-tf-transformations
sudo apt-get install ros-humble-moveit*
```
- **ros-humble-ros2-control**: Framework para administrar hardware y controladores en ROS 2 (back-end de `ros2_control`).
- **ros-humble-ros2-controllers**: Conjunto de controladores listos (p. ej., `joint_state_broadcaster`, `joint_trajectory_controller`) que usarás en simulación y/o hardware real.
- **ros-humble-xacro**: Procesador de macros Xacro para generar URDF a partir de archivos `.xacro`.
- **ros-humble-ros-gz-***: Paquetes de integración entre ROS 2 y Gazebo (puentes de tópicos, utilidades, etc.). El comodín `*` instala los subpaquetes relevantes.
- **ros-humble-*-ros2-control**: Instala paquetes relacionados con `ros2_control` para diferentes integraciones (plugins/adaptadores). El comodín `*` permite que APT resuelva todos los que apliquen.
- **ros-humble-joint-state-publisher-gui**: Interfaz gráfica para publicar estados de articulaciones y probar cinemática sin hardware.
- **ros-humble-tf-transformations**: Utilidades para transformaciones espaciales (TF) en Python.
- **ros-humble-moveit***: Instala MoveIt y sus componentes (planificación de movimiento, RViz plugins, etc.).

> 💡 **Instalación en un solo comando (opcional):**
> ```bash
> sudo apt-get install -y \
>   ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro \
>   ros-humble-ros-gz-* ros-humble-*-ros2-control \
>   ros-humble-joint-state-publisher-gui ros-humble-tf-transformations \
>   ros-humble-moveit*
> ```

### 4) Paquetes de Python necesarios (Alexa e interconexión)
```bash
sudo apt-get install python3-pip
pip install transforms3d
pip install flask
pip install pyserial
pip install flask-ask-sdk
pip install ask-sdk
sudo apt install python3-lark
pip install lark
```
- **python3-pip**: Gestor de paquetes para Python.
- **transforms3d**: Transformaciones 3D (rotaciones, cuaterniones, matrices) útiles para robótica.
- **flask**: Micro‑framework web para crear servicios y dashboards ligeros.
- **pyserial**: Comunicación serie en Python; útil para hablar con microcontroladores (p. ej., Arduino).
- **flask-ask-sdk / ask-sdk**: SDKs para integrar Alexa Skills con aplicaciones Python/Flask.

> ⚠️ Si tu `pip` apunta a Python 2 (poco común en 22.04), usa `pip3` en su lugar.

### 5) Paquetes adicionales para comunicación serie en C++
```bash
sudo apt-get install libserial-dev
```
- **¿Qué hace?** Instala los headers y librerías de **libserial** para aplicaciones C++ que se comuniquen por puerto serie (p. ej., ROS 2 ↔ Arduino).

---

### 📌 Referencia de “¿Qué hace cada línea?” (resumen rápido)
- `sudo apt-get update && sudo apt-get upgrade`: sincroniza índices e instala actualizaciones.
- `sudo apt-get install terminator`: emulador de terminal con paneles múltiples.
- `sudo apt-get install ros-humble-ros2-control`: instala el back‑end de `ros2_control`.
- `sudo apt-get install ros-humble-ros2-controllers`: controladores genéricos (broadcasters/controladores de articulaciones).
- `sudo apt-get install ros-humble-xacro`: utilidades Xacro para generar URDF.
- `sudo apt-get install ros-humble-ros-gz-*`: puente e integración ROS 2 ↔ Gazebo.
- `sudo apt-get install ros-humble-*-ros2-control`: complementos vinculados a `ros2_control`.
- `sudo apt-get install ros-humble-joint-state-publisher-gui`: GUI para publicar estados articulares.
- `sudo apt-get install ros-humble-tf-transformations`: helpers de transformaciones TF en Python.
- `sudo apt-get install ros-humble-moveit*`: instala MoveIt y sus componentes.
- `sudo apt-get install python3-pip`: gestor de paquetes Python 3.
- `pip install ...`: instala paquetes Python listados (transforms3d, flask, pyserial, Alexa SDKs).
- `sudo apt-get install libserial-dev`: librería C++ para comunicación serie.

