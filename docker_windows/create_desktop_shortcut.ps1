$ErrorActionPreference = "Stop"

$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Split-Path -Parent $Here
$StartScript = Join-Path $Here "start_windows_wslg.ps1"
$Icon = Join-Path $Here "robot_workcell.ico"
$Desktop = [Environment]::GetFolderPath("Desktop")

if (-not (Test-Path $StartScript)) {
    throw "Launcher not found: $StartScript"
}

if (-not (Test-Path $Icon)) {
    throw "Icon not found: $Icon"
}

$Shell = New-Object -ComObject WScript.Shell
$ShortcutPath = Join-Path $Desktop "Robot Workcell.lnk"
$Shortcut = $Shell.CreateShortcut($ShortcutPath)

$Shortcut.TargetPath = "$env:SystemRoot\System32\WindowsPowerShell\v1.0\powershell.exe"
$Shortcut.Arguments = "-NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$StartScript`""
$Shortcut.WorkingDirectory = $RepoRoot
$Shortcut.IconLocation = "$Icon,0"
$Shortcut.Description = "Start UR7e Robot Workcell"
$Shortcut.Save()

Write-Host "Created desktop shortcut:"
Write-Host $ShortcutPath
