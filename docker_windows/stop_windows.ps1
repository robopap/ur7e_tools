$ErrorActionPreference = "Stop"
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$Compose = Join-Path $Here "docker-compose.yml"
$EnvFile = Join-Path $Here ".env"

Push-Location $Here
try {
    if (Test-Path $EnvFile) {
        docker compose --env-file $EnvFile -f $Compose down
    } else {
        docker compose -f $Compose down
    }
}
finally {
    Pop-Location
}
