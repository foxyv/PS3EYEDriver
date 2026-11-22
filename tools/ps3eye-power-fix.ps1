<#
.SYNOPSIS
  Stops Windows from power-managing PS3 Eye cameras (USB selective suspend),
  which can leave the camera in a bad state after sleep.
.DESCRIPTION
  Sets EnhancedPowerManagementEnabled=0, AllowIdleIrpInD3=0 and
  DeviceSelectiveSuspended=0 on the Device Parameters key of every PS3 Eye
  (VID_1415 PID_2000) Windows has seen, on any USB port. Run as Administrator.
  Use -Undo to remove the values again, -WhatIf to preview.
  SPDX-License-Identifier: ISC   Copyright (c) 2026 foxyv
#>
[CmdletBinding(SupportsShouldProcess)]
param([switch]$Undo)

$principal = [Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Write-Error 'Run this script from an Administrator PowerShell.'; exit 1
}

$names = 'EnhancedPowerManagementEnabled', 'AllowIdleIrpInD3', 'DeviceSelectiveSuspended'
$root = 'HKLM:\SYSTEM\CurrentControlSet\Enum\USB\VID_1415&PID_2000'
if (-not (Test-Path -LiteralPath $root)) { Write-Host 'No PS3 Eye has been connected to this PC.'; exit 0 }

foreach ($dev in Get-ChildItem -LiteralPath $root) {
    $key = Join-Path $dev.PSPath 'Device Parameters'
    if ($Undo) {
        if (-not (Test-Path -LiteralPath $key)) { continue }
        foreach ($n in $names) {
            if ($PSCmdlet.ShouldProcess("$($dev.PSChildName)", "remove $n")) {
                Remove-ItemProperty -LiteralPath $key -Name $n -ErrorAction SilentlyContinue
            }
        }
        Write-Host "Restored defaults: $($dev.PSChildName)"
    } else {
        if (-not (Test-Path -LiteralPath $key)) { New-Item -Path $key -Force | Out-Null }
        foreach ($n in $names) {
            if ($PSCmdlet.ShouldProcess("$($dev.PSChildName)", "set $n = 0")) {
                Set-ItemProperty -LiteralPath $key -Name $n -Value 0 -Type DWord
            }
        }
        Write-Host "Power management off: $($dev.PSChildName)"
    }
}
Write-Host 'Done. Unplug and replug the camera (or restart) for it to take effect.'
