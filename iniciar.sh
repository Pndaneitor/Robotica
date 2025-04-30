#!/bin/bash
env_folder="robotica"

if [ ! -d "$env_folder" ]; then
  echo "Creando entorno virtual..."
  python3 -m venv $env_folder
fi

echo "Activando entorno virtual..."
source "$env_folder/bin/activate"
echo "Instalando dependencias..."
py -m pip install -r requirements.txt
echo "Entorno virtual activado y dependencias instaladas."
