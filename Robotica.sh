#!/bin/bash

# 1. Detectar versión de ROS instalada
if [ -d "/opt/ros" ]; then
    ROS_VERSION=$(ls /opt/ros | head -n 1)
    ROS_SETUP="/opt/ros/$ROS_VERSION/setup.bash"
    echo "Versión de ROS detectada: $ROS_VERSION"
else
    echo "No se encontró instalación de ROS en /opt/ros"
    exit 1
fi

# 2. Activar entorno ROS
if [ -f "$ROS_SETUP" ]; then
    echo "Activando entorno ROS..."
    source "$ROS_SETUP"
else
    echo "No se encontró el archivo: $ROS_SETUP"
    exit 1
fi

# 3. Instalar dependencias Python si existe requirements.txt
REQS_FILE="src/mi_paquete/requirements.txt"
if [ -f "$REQS_FILE" ]; then
    echo "Instalando dependencias Python desde $REQS_FILE..."
    pip install -r "$REQS_FILE" --user
else
    echo "No se encontró $REQS_FILE. Se omite instalación de paquetes."
fi

# 4. Compilar con colcon
echo "Compilando workspace ROS 2 con colcon..."
colcon build --symlink-install

# 5. Source del entorno local generado por colcon
if [ -f "install/setup.bash" ]; then
    echo "Activando entorno local del workspace..."
    source install/setup.bash
else
    echo "No se encontró install/setup.bash. Revisa si colcon build tuvo errores."
fi
