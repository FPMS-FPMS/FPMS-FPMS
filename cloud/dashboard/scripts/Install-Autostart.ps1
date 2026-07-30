<#
.SYNOPSIS
  Register (or remove) the "FPMS HQ" logon task that keeps the dashboard and its
  public tunnel alive.

.DESCRIPTION
  The public URL only exists while this laptop is running the app + cloudflared.
  Every outage so far has come from one of them quietly exiting and nobody
  noticing until a phone showed "server cannot be found".

  This registers a Scheduled Task that runs Publish-Public-Persistent.bat at
  logon. That script already contains restart-on-crash loops for both processes,
  so this only has to handle "start it at logon and keep it running".

  Runs in the interactive user session (not SYSTEM) so the WebView2 window and
  the user-scope FPMS_PASSWORD are both available. No elevation required.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\Install-Autostart.ps1
  powershell -ExecutionPolicy Bypass -File .\Install-Autostart.ps1 -Remove
#>
[CmdletBinding()]
param(
    [switch]$Remove,
    [string]$TaskName = 'FPMS HQ'
)

$ErrorActionPreference = 'Stop'

$dashboard = Split-Path -Parent $PSScriptRoot
$bat = Join-Path $dashboard 'Publish-Public-Persistent.bat'

if ($Remove) {
    if (Get-ScheduledTask -TaskName $TaskName -ErrorAction Ignore) {
        Unregister-ScheduledTask -TaskName $TaskName -Confirm:$false
        Write-Host "[ok] Removed scheduled task '$TaskName'." -ForegroundColor Green
    } else {
        Write-Host "[--] No task named '$TaskName' to remove." -ForegroundColor Yellow
    }
    return
}

if (-not (Test-Path $bat)) {
    throw "Cannot find $bat - run this from the dashboard's scripts\ folder."
}

# Publishing without a password is refused by the backend, and the task has no
# console to prompt on, so fail loudly here instead of at 3am on a logon.
$pw = [Environment]::GetEnvironmentVariable('FPMS_PASSWORD', 'User')
if ([string]::IsNullOrWhiteSpace($pw)) {
    throw ("FPMS_PASSWORD is not set at User scope. Set it first:`n" +
           "  [Environment]::SetEnvironmentVariable('FPMS_PASSWORD','<strong-password>','User')")
}

# FPMS_UNATTENDED tells the .bat to log and exit rather than prompt for input.
$action = New-ScheduledTaskAction -Execute 'cmd.exe' `
    -Argument ('/c set FPMS_UNATTENDED=1 && "{0}"' -f $bat) `
    -WorkingDirectory $dashboard

# Two triggers. Logon covers the normal case; the repeating one is a safety net
# for the months-long run - if the app, the tunnel and the restart loop all die
# together, this brings the stack back within half an hour instead of leaving it
# down until someone notices the public link is dead.
# MultipleInstances=IgnoreNew means the repeat is a no-op while it's healthy.
$triggers = @(
    # Survives an unattended reboot - a 3am Windows Update restart must not take
    # the public link down until somebody happens to log in.
    (New-ScheduledTaskTrigger -AtStartup),
    (New-ScheduledTaskTrigger -AtLogOn -User $env:USERNAME),
    (New-ScheduledTaskTrigger -Once -At (Get-Date).AddMinutes(5) `
        -RepetitionInterval (New-TimeSpan -Minutes 30) `
        -RepetitionDuration (New-TimeSpan -Days 3650))
)

# S4U: runs as this user with no stored password AND with nobody logged in.
# Deliberately not SYSTEM - settings_store resolves ~/.fpms via Path.home(), so
# SYSTEM would look in C:\Windows\System32\config\systemprofile and silently
# lose the gateway URL and secret. The service half is headless, so it doesn't
# need an interactive desktop; your own window still opens on demand and just
# attaches to this backend.
$principal = New-ScheduledTaskPrincipal -UserId "$env:USERDOMAIN\$env:USERNAME" `
    -LogonType S4U -RunLevel Limited

$settings = New-ScheduledTaskSettingsSet `
    -AllowStartIfOnBatteries `
    -DontStopIfGoingOnBatteries `
    -StartWhenAvailable `
    -MultipleInstances IgnoreNew `
    -ExecutionTimeLimit ([TimeSpan]::Zero) `
    -RestartCount 3 `
    -RestartInterval (New-TimeSpan -Minutes 1)

if (Get-ScheduledTask -TaskName $TaskName -ErrorAction Ignore) {
    Unregister-ScheduledTask -TaskName $TaskName -Confirm:$false
}

$desc = 'Starts the FPMS dashboard and its public Cloudflare tunnel, restarting either if it crashes.'

# Preferred: S4U + at-startup, so an unattended reboot brings the public link
# back with nobody logged in. That combination needs elevation. If we don't have
# it, fall back to a logon task rather than leaving NO task registered at all -
# degraded autostart beats none.
$mode = 'startup (S4U)'
try {
    Register-ScheduledTask -TaskName $TaskName -Action $action -Trigger $triggers `
        -Principal $principal -Settings $settings -Description $desc -ErrorAction Stop | Out-Null
} catch {
    Write-Warning "Could not register the at-startup task ($($_.Exception.Message.Trim()))."
    Write-Warning "Falling back to a logon-only task. Re-run this elevated for reboot-proof autostart."
    $mode = 'logon only (not elevated)'
    $fallbackTriggers = @(
        (New-ScheduledTaskTrigger -AtLogOn -User $env:USERNAME),
        (New-ScheduledTaskTrigger -Once -At (Get-Date).AddMinutes(5) `
            -RepetitionInterval (New-TimeSpan -Minutes 30) `
            -RepetitionDuration (New-TimeSpan -Days 3650))
    )
    $fallbackPrincipal = New-ScheduledTaskPrincipal -UserId $env:USERNAME `
        -LogonType Interactive -RunLevel Limited
    Register-ScheduledTask -TaskName $TaskName -Action $action -Trigger $fallbackTriggers `
        -Principal $fallbackPrincipal -Settings $settings -Description $desc | Out-Null
}

Write-Host "[ok] Registered '$TaskName' - mode: $mode" -ForegroundColor Green
Write-Host "     Start now : Start-ScheduledTask -TaskName '$TaskName'"
Write-Host "     Remove    : .\Install-Autostart.ps1 -Remove"
