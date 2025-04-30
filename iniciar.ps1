# Archivo: iniciar.ps1
$envFolder = "venv" # Nombre del entorno virtual

if (-Not (Test-Path $envFolder)) {
    Write-Host "Creando entorno virtual..."
    python -m venv $envFolder
}

Write-Host "Activando entorno virtual..."
& "$envFolder\Scripts\Activate.ps1"
Write-Host "Instalando dependencias..."
py -m pip install -r requirements.txt
Write-Host "Entorno virtual activado y dependencias instaladas."