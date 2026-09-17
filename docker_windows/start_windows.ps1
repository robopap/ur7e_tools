$ErrorActionPreference = "Stop"

$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$Compose = Join-Path $Here "docker-compose.yml"
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

# Prefer the Windows NIC on the current robot subnet.  Do not use the robot IPs.
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
    $Configured = (Get-Content $EnvFile | Where-Object { $_ -match '^UR_REVERSE_IP=' } | Select-Object -First 1)
    if ($Configured) {
        $env:UR_REVERSE_IP = ($Configured -split '=', 2)[1].Trim()
    }
    Write-Warning "No Windows 10.0.0.X NIC was auto-detected. UR_REVERSE_IP=$($env:UR_REVERSE_IP)"
}

Push-Location $Here
try {
    docker compose --env-file $EnvFile -f $Compose up --build -d

    $Ready = $false
    foreach ($i in 1..120) {
        $Result = Test-NetConnection -ComputerName 127.0.0.1 -Port 6080 -WarningAction SilentlyContinue
        if ($Result.TcpTestSucceeded) {
            $Ready = $true
            break
        }
        Start-Sleep -Seconds 1
    }

    if (-not $Ready) {
        docker compose --env-file $EnvFile -f $Compose logs --tail 100
        throw "Container started, but the noVNC UI port did not become reachable."
    }

    Start-Process "http://localhost:6080/vnc.html?autoconnect=true&resize=scale"
    Write-Host "Robot Workcell Control: http://localhost:6080/vnc.html?autoconnect=true&resize=scale"
    Write-Host "Logs: docker compose -f `"$Compose`" logs -f"
}
finally {
    Pop-Location
}
