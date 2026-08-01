; FPMS Dashboard - Windows installer (Inno Setup 6+)
;
; Compile:  ISCC.exe build\installer.iss
; Output:   build\installer-dist\FPMS-Dashboard-Setup.exe
;
; What this installer does when the user runs Setup.exe:
;   * Installs FPMS-Dashboard.exe to  %LOCALAPPDATA%\Programs\FPMS Dashboard
;     (no admin rights needed)
;   * Registers the app in Add / Remove Programs
;   * Creates Start Menu shortcut
;   * Optional Desktop shortcut (checkbox)
;   * Optional launch after install (checkbox)
;   * Sets an AppUserModelID so pinning to the taskbar works
;     (Windows treats it as a real installed application)

#define AppName        "FPMS Dashboard"
#define AppVersion     "1.0.0"
#define AppPublisher   "FPMS Robotics"
#define AppExeName     "FPMS-Dashboard.exe"
#define AppId          "com.fpms.robotics.dashboard"
#define SourceRoot     SourcePath + "..\"

; Which folder to take FPMS-Dashboard.exe from, relative to SourceRoot.
; Override with:  ISCC /DExeDir=dist-new  installer.iss
; That matters because the background service holds dist\FPMS-Dashboard.exe
; open — under the S4U logon task it can't even be killed without elevation —
; so a fresh build has to be packaged straight from its own output folder.
#ifndef ExeDir
  #define ExeDir "dist"
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

; Per-user install, no admin needed
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
DefaultDirName={autopf}\FPMS Dashboard
DefaultGroupName=FPMS
DisableProgramGroupPage=yes
DisableDirPage=yes
LicenseFile=
OutputDir={#SourceRoot}build\installer-dist
OutputBaseFilename=FPMS-Dashboard-Setup
SetupIconFile={#SourceRoot}build\fpms.ico
UninstallDisplayIcon={app}\{#AppExeName}
UninstallDisplayName={#AppName}
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
WizardResizable=yes
ShowLanguageDialog=no
CloseApplications=yes
RestartApplications=no

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon";  Description: "Create a &desktop shortcut"; GroupDescription: "Additional shortcuts:"

[Files]
Source: "{#SourceRoot}{#ExeDir}\{#AppExeName}"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceRoot}bin\cloudflared.exe"; DestDir: "{app}\bin"; Flags: ignoreversion skipifsourcedoesntexist

[Icons]
Name: "{autoprograms}\{#AppName}";  Filename: "{app}\{#AppExeName}"; \
  IconFilename: "{app}\{#AppExeName}"; \
  AppUserModelID: "{#AppId}"
Name: "{autodesktop}\{#AppName}";   Filename: "{app}\{#AppExeName}"; \
  IconFilename: "{app}\{#AppExeName}"; \
  AppUserModelID: "{#AppId}"; \
  Tasks: desktopicon

[Run]
Filename: "{app}\{#AppExeName}"; \
  Description: "&Launch FPMS Dashboard"; \
  Flags: nowait postinstall skipifsilent

[UninstallDelete]
Type: filesandordirs; Name: "{localappdata}\FPMS\browser-profile"
Type: filesandordirs; Name: "{localappdata}\FPMS\launch.log"

[Registry]
; Register as an Application so Windows knows this is a real installed app
; (helps with taskbar pinning + default-app associations later if desired)
Root: HKCU; Subkey: "Software\Classes\Applications\{#AppExeName}"; \
  ValueType: string; ValueName: "FriendlyAppName"; ValueData: "{#AppName}"; \
  Flags: uninsdeletekey
Root: HKCU; Subkey: "Software\Classes\Applications\{#AppExeName}\shell\open\command"; \
  ValueType: string; ValueName: ""; ValueData: """{app}\{#AppExeName}"""; \
  Flags: uninsdeletekey
