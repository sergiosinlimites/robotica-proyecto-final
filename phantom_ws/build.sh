#!/bin/bash
# Script para compilar forzando Python 3.10 (Sistema) para evitar conflictos con PyEnv

echo "--- Iniciando compilación segura (Python 3.10) ---"

# 1. Asegurar que usamos /usr/bin/python3 (el del sistema) y no el de pyenv
export PATH=/usr/bin:$PATH

# 2. Cargar entorno de ROS 2 Humble
source /opt/ros/humble/setup.bash

# 3. Compilar forzando el ejecutable de Python correcto
# --symlink-install: Para no tener que recompilar si solo cambias Python (en algunos casos)
# -DPython3_EXECUTABLE: Fuerza CMake a usar el Python del sistema
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

echo "--- Compilación terminada ---"
echo "Para usar los cambios, ejecuta: source install/setup.bash"
