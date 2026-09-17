$ErrorActionPreference = "Stop"

$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$Compose = Join-Path $Here "docker-compose.wslg.yml"
$EnvFile = Join-Path $Here ".env"

Push-Location $Here

try {
    docker compose `
        --env-file $EnvFile `
        -f $Compose `
        down

    Write-Host ""
    Write-Host "WSLg workcell stopped."
}
finally {
    Pop-Location
}