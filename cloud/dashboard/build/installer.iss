; FPMS Dashboard - Windows installer (Inno Setup 6+)
;
; Compile:  ISCC.exe build\installer.iss
; Output:   build\installer-dist\FPMS-Dashboard-Setup.exe
;
; NOTE FOR WHOEVER READS THIS NEXT: Inno Setup (ISCC.exe) is NOT installed on
; the build machine this was last edited on, so this script has been verified by
; inspection only - it has never been compiled. The no-Inno path is
; scripts\Install-App.ps1, which does the same job and HAS been run. If you
; install Inno Setup 6, `scripts\Build-Windows.ps1` picks ISCC up automatically.
;
; What this installer does when the user runs Setup.exe:
;   * Installs the whole one-folder build to %LOCALAPPDATA%\Programs\FPMS Dashboard
;     (no admin rights needed)
;   * Registers the app in Add / Remove Programs, with a real uninstaller
;   * Creates a Start Menu shortcut, and a Desktop shortcut (checkbox)
;   * Sets an AppUserModelID so pinning to the taskbar works

#define AppName        "FPMS Dashboard"
#define AppVersion     "1.0.0"
#define AppPublisher   "FPMS Robotics"
#define AppExeName     "FPMS-Dashboard.exe"
#define AppId          "com.fpms.robotics.dashboard"
#define SourceRoot     SourcePath + "..\"

; ---------------------------------------------------------------------------
; ONE-FOLDER, NOT ONE-FILE.
;
; build\fpms.spec switched to a COLLECT (one-folder) build, because one-file
; unpacked ~57 MB to %TEMP% on every launch and the app appeared to hang. The
; output shape changed with it:
;
;     BEFORE:  dist\FPMS-Dashboard.exe                 <- a single file
;     AFTER:   dist\FPMS-Dashboard\FPMS-Dashboard.exe  <- exe + _internal\
;              dist\FPMS-Dashboard\_internal\...          (~500 files)
;
; This script used to package the single file. Pointed at the new tree it would
; still "succeed" - dist\FPMS-Dashboard.exe is STILL THERE, left over from the
; last one-file build on 2026-08-01 - and produce a setup that installs a
; months-stale exe with no _internal\ beside it. That exe cannot start: the
; one-folder bootloader looks for _internal\ next to itself and finds nothing.
;
; So DistSubDir is deliberately part of the path and the #error below refuses to
; compile if the one-folder tree is missing, rather than silently packaging
; whatever single file happens to be lying in dist\.
;
; Override the source folder with:  ISCC /DDistDir=dist-new build\installer.iss
; ---------------------------------------------------------------------------
#ifndef DistDir
  #define DistDir "dist"
#endif
#define AppDir SourceRoot + DistDir + "\FPMS-Dashboard"

#if !FileExists(AppDir + "\" + AppExeName)
  #error One-folder build not found. Expected <DistDir>\FPMS-Dashboard\FPMS-Dashboard.exe next to an _internal\ folder. Run scripts\Build-Windows.ps1 first, or pass /DDistDir=<folder>.
#endif
#if !DirExists(AppDir + "\_internal")
  #error Found the exe but no _internal\ beside it. That is a one-FILE build; installing it produces an app that cannot start. Rebuild with the current build\fpms.spec.
#endif

[Setup]
AppId={{9F1CBA07-2C6B-4A1E-B14A-3F2B9C4E5D01}
AppName={#AppName}
AppVersion={#AppVersion}
AppPublisher={#AppPublisher}
AppPublisherURL=https://github.com/FPMS-FPMS
AppSupportURL=https://github.com/FPMS-FPMS/issues
VersionInfoVersion={#AppVersion}
VersionInfoCompany={#AppPublisher}
VersionInfoDescription=FPMS Robotics Operations Console

; Per-user install, no admin needed.
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
; With PrivilegesRequired=lowest, {autopf} resolves to
; {localappdata}\Programs - the same place scripts\Install-App.ps1 installs to,
; on purpose, so the two paths cannot produce two rival copies.
DefaultDirName={autopf}\FPMS Dashboard
DefaultGroupName=FPMS
DisableProgramGroupPage=yes
DisableDirPage=yes
LicenseFile=
OutputDir={#SourceRoot}build\installer-dist
OutputBaseFilename=FPMS-Dashboard-Setup
SetupIconFile={#SourceRoot}build\fpms.ico
UninstallDisplayIcon={app}\fpms.ico
UninstallDisplayName={#AppName}
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
WizardResizable=yes
ShowLanguageDialog=no
; The app holds its own files open while running, and _internal\ is ~500 of
; them. Ask Windows Restart Manager to close it rather than failing mid-copy.
; This CANNOT close an instance started by the 'FPMS HQ' scheduled task under a
; different logon - see CurStepChanged below, which detects that and says so.
CloseApplications=yes
CloseApplicationsFilter=*.exe,*.dll,*.pyd
RestartApplications=no
; An update must land on top of the previous install, not beside it.
UsePreviousAppDir=yes

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "Create a &desktop shortcut"; GroupDescription: "Additional shortcuts:"

[InstallDelete]
; Wipe the previous _internal\ before laying down the new one. PyInstaller
; renames and drops modules between builds, and an orphaned .pyd left behind
; from an older build gets imported in preference to nothing at all - which
; fails at runtime, inside a windowed app, with no console to say why.
Type: filesandordirs; Name: "{app}\_internal"
; A leftover one-file exe sitting beside a one-folder install is the same
; ambiguity in reverse. There is only ever one exe here.
Type: files; Name: "{app}\FPMS-Dashboard.old.exe"

[Files]
; The whole one-folder tree. recursesubdirs+createallsubdirs are both required:
; without them _internal\ subfolders (PIL\, webview\, frontend\dist\assets\...)
; are skipped and the app starts to a blank window.
Source: "{#AppDir}\{#AppExeName}"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#AppDir}\_internal\*"; DestDir: "{app}\_internal"; \
  Flags: ignoreversion recursesubdirs createallsubdirs
; Shipped as a real file as well as being embedded in the exe. Shortcut icons
; read from a .ico refresh reliably; icons read out of an exe get cached by
; Explorer per-path and an updated exe at the same path often keeps showing the
; old glyph until the icon cache is cleared.
Source: "{#SourceRoot}build\fpms.ico"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceRoot}bin\cloudflared.exe"; DestDir: "{app}\bin"; \
  Flags: ignoreversion skipifsourcedoesntexist

[Icons]
; {group} is Start Menu\Programs\FPMS (DefaultGroupName above), which is the
; same place scripts\Install-App.ps1 puts its Start Menu shortcut. Using
; {autoprograms} here instead put it at the top level, so installing with one
; tool after the other left two shortcuts with the same name in two places.
Name: "{group}\{#AppName}"; Filename: "{app}\{#AppExeName}"; \
  WorkingDir: "{app}"; IconFilename: "{app}\fpms.ico"; \
  Comment: "FPMS Robotics Operations Console"; \
  AppUserModelID: "{#AppId}"
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\{#AppExeName}"; \
  WorkingDir: "{app}"; IconFilename: "{app}\fpms.ico"; \
  Comment: "FPMS Robotics Operations Console"; \
  AppUserModelID: "{#AppId}"; \
  Tasks: desktopicon
Name: "{group}\Uninstall {#AppName}"; Filename: "{uninstallexe}"

[Run]
Filename: "{app}\{#AppExeName}"; WorkingDir: "{app}"; \
  Description: "&Launch FPMS Dashboard"; \
  Flags: nowait postinstall skipifsilent

[UninstallDelete]
; Only the app's own scratch state. The operator's data under
; {localappdata}\FPMS (settings, downloads) is deliberately left alone.
Type: filesandordirs; Name: "{localappdata}\FPMS\browser-profile"
Type: files; Name: "{localappdata}\FPMS\launch.log"
Type: filesandordirs; Name: "{app}\_internal"
Type: dirifempty; Name: "{app}"

[Registry]
; Register as an Application so Windows knows this is a real installed app
; (helps with taskbar pinning + default-app associations later if desired)
Root: HKCU; Subkey: "Software\Classes\Applications\{#AppExeName}"; \
  ValueType: string; ValueName: "FriendlyAppName"; ValueData: "{#AppName}"; \
  Flags: uninsdeletekey
Root: HKCU; Subkey: "Software\Classes\Applications\{#AppExeName}\shell\open\command"; \
  ValueType: string; ValueName: ""; ValueData: """{app}\{#AppExeName}"""; \
  Flags: uninsdeletekey

[Code]
const
  PortToCheck = 8000;

{ ------------------------------------------------------------------------
  Two failure modes this installer has to name out loud, because both look
  like success and neither is discoverable from the UI afterwards.
  ------------------------------------------------------------------------ }

function RunHidden(const Cmd: String; var Output: Integer): Boolean;
begin
  Result := Exec(ExpandConstant('{cmd}'), '/C ' + Cmd, '', SW_HIDE,
                 ewWaitUntilTerminated, Output);
end;

{ An instance launched by the 'FPMS HQ' scheduled task runs under an S4U logon.
  It is not in this desktop's session, Restart Manager cannot close it, and
  Stop-Process from an unelevated shell gets Access Denied. Detect it by asking
  the task scheduler, not by looking at processes we may not be allowed to see. }
function FpmsHqTaskRunning(): Boolean;
var
  Code: Integer;
begin
  Result := False;
  { schtasks /query returns 0 only if the task exists; findstr narrows it to a
    Running state. Both non-zero => not running / not registered. }
  if RunHidden('schtasks /Query /TN "FPMS HQ" /FO LIST 2>nul | findstr /I /C:"Running" >nul', Code) then
    Result := (Code = 0);
end;

procedure CurStepChanged(CurStep: TSetupStep);
var
  Code: Integer;
begin
  if CurStep = ssInstall then
  begin
    if FpmsHqTaskRunning() then
    begin
      { Not fatal by itself - the copy may still succeed if that instance runs
        from a different folder - but it WILL keep port 8000, so the app you
        launch afterwards attaches to it and shows the old UI. Say so now. }
      MsgBox('The scheduled task "FPMS HQ" is currently RUNNING.' + #13#10#13#10 +
             'It holds an older FPMS backend open, and this installer cannot ' +
             'stop it - it runs under a different logon, so even Task Manager ' +
             'refuses without elevation.' + #13#10#13#10 +
             'Setup will continue, but before you launch the new app, stop it ' +
             'from an ADMINISTRATOR PowerShell:' + #13#10#13#10 +
             '    Stop-ScheduledTask -TaskName "FPMS HQ"' + #13#10#13#10 +
             'If you skip this, the new app will find port ' +
             IntToStr(PortToCheck) + ' already answering and ATTACH TO THE OLD ' +
             'BACKEND instead of starting its own. The window opens, everything ' +
             'looks fine, and you are looking at the PREVIOUS build.',
             mbError, MB_OK);
    end;
  end;

  if CurStep = ssPostInstall then
  begin
    { Anything listening on the port is enough: the launcher's own check
      (backend/desktop.py _server_alive) treats ANY HTTP answer below 500 as
      "a backend is already up, attach to it". }
    if RunHidden('netstat -ano -p TCP | findstr /R /C:":' + IntToStr(PortToCheck) +
                 ' .*LISTENING" >nul', Code) then
    begin
      if Code = 0 then
        MsgBox('Port ' + IntToStr(PortToCheck) + ' is ALREADY IN USE.' + #13#10#13#10 +
               'FPMS Dashboard will not start its own backend while something ' +
               'else is answering there - it attaches to the existing one. ' +
               'The new UI you just installed will NOT be the one you see.' + #13#10#13#10 +
               'Find the owner:' + #13#10 +
               '    Get-NetTCPConnection -LocalPort ' + IntToStr(PortToCheck) +
               ' -State Listen' + #13#10#13#10 +
               'Then stop it, or set FPMS_BIND_PORT to a free port before ' +
               'launching.', mbError, MB_OK);
    end;
  end;
end;
