$ErrorActionPreference = "Stop"

$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$Compose = Join-Path $Here "docker-compose.wslg.yml"
$FallbackCompose = Join-Path $Here "docker-compose.yml"
$EnvFile = Join-Path $Here ".env"
$EnvExample = Join-Path $Here ".env.example"

if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    throw "Docker CLI not found. Start/install Docker Desktop first."
}

docker info | Out-Null

if (-not (Test-Path $EnvFile)) {
    Copy-Item $EnvExample $EnvFile
    Write-Host "Created $EnvFile from .env.example"
}

# Prefer the Windows NIC on the robot subnet.
# Robot1 = 10.0.0.1
# Robot2 = 10.0.0.2
# Windows host should normally be 10.0.0.8.
$RobotNet = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Where-Object {
        $_.IPAddress -like "10.0.0.*" -and
        $_.IPAddress -ne "10.0.0.1" -and
        $_.IPAddress -ne "10.0.0.2" -and
        $_.IPAddress -ne "10.0.0.255" -and
        $_.PrefixOrigin -ne "WellKnown"
    } |
    Sort-Object InterfaceMetric |
    Select-Object -First 1

if ($RobotNet) {
    $env:UR_REVERSE_IP = $RobotNet.IPAddress
    Write-Host "UR reverse IP: $($env:UR_REVERSE_IP) [$($RobotNet.InterfaceAlias)]"
} else {
    $Configured = (
        Get-Content $EnvFile |
        Where-Object { $_ -match '^UR_REVERSE_IP=' } |
        Select-Object -First 1
    )

    if ($Configured) {
        $env:UR_REVERSE_IP = ($Configured -split '=', 2)[1].Trim()
    }

    Write-Warning "No Windows 10.0.0.X NIC was auto-detected. UR_REVERSE_IP=$($env:UR_REVERSE_IP)"
}

Push-Location $Here

try {
    # The noVNC and WSLg modes use the same UR/NANSENSE host ports,
    # so they cannot run at the same time.
    docker compose --env-file $EnvFile -f $FallbackCompose down 2>$null

    # Always remove any previous WSLg container, including a cleanly
    # exited instance left behind after closing the Workcell window.
    $ExistingContainer = docker ps -aq `
        --filter "name=^ur7e_tools_windows_wslg$"

    if ($ExistingContainer) {
        docker rm -f ur7e_tools_windows_wslg | Out-Null
    }

    # Start a fresh WSLg instance.
    docker compose --env-file $EnvFile -f $Compose up --build -d

    Start-Sleep -Seconds 2

    $Running = docker inspect `
        -f '{{.State.Running}}' `
        ur7e_tools_windows_wslg 2>$null

    if ($Running -ne "true") {
        docker compose --env-file $EnvFile -f $Compose logs --tail 100
        throw "WSLg container did not remain running."
    }

    Write-Host ""
    Write-Host "WSLg workcell started."
    Write-Host "Container: ur7e_tools_windows_wslg"
    Write-Host "UR reverse IP: $($env:UR_REVERSE_IP)"
    Write-Host "No browser/noVNC is used."
    Write-Host ""
    Write-Host "Logs:"
    Write-Host "docker compose --env-file `"$EnvFile`" -f `"$Compose`" logs -f"
}
finally {
    Pop-Location
}