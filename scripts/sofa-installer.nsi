;--------------------------------
;Include Modern UI

  !include "MUI2.nsh"

;--------------------------------

; The name of the installer
Name "SurfLab SOFA framework"

; The file to write
OutFile "surflab-sofa-dist.exe"

; The default installation directory
InstallDir $LOCALAPPDATA\SurfLab\SOFA

; Registry key to check for directory (so if you install again, it will 
; overwrite the old one automatically)
InstallDirRegKey HKCU "Software\SurfLab\SOFA" "Install_Dir"

; Request application privileges for Windows Vista
RequestExecutionLevel user

;--------------------------------

!define MUI_FINISHPAGE_RUN "$INSTDIR\bin\runSofa.exe"

; Pages

  !insertmacro MUI_PAGE_LICENSE "Licence.txt"
  !insertmacro MUI_PAGE_COMPONENTS
  !insertmacro MUI_PAGE_DIRECTORY
  !insertmacro MUI_PAGE_INSTFILES
  !insertmacro MUI_PAGE_FINISH

  !insertmacro MUI_UNPAGE_CONFIRM
  !insertmacro MUI_UNPAGE_INSTFILES

;--------------------------------
;Languages
 
  !insertmacro MUI_LANGUAGE "English"

;--------------------------------
; The stuff to install
Section "Application binaries and examples"

  SectionIn RO
  
  ; Set output path to the installation directory.
  SetOutPath $INSTDIR
  
  ; Put file there
  File /r /x *.ilk /x *.pdb "bin" 
  File /r /x *.comp "examples" "share"
  
  CreateDirectory "$INSTDIR\etc"
  FileOpen $0 "$INSTDIR\etc\sofa.ini" w
  FileWrite $0 "SHARE_DIR=$INSTDIR\share"
  FileWriteByte $0 "13"
  FileWriteByte $0 "10"
  FileWrite $0 "EXAMPLES_DIR=$INSTDIR\examples"
  FileWriteByte $0 "13"
  FileWriteByte $0 "10"
  FileClose $0
  
  ; Write the installation path into the registry
  WriteRegStr HKCU SOFTWARE\SurfLab\SOFA "Install_Dir" "$INSTDIR"
  
  ; Write the uninstall keys for Windows
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\SurfLab_SOFA" "DisplayName" "SurfLab SOFA"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\SurfLab_SOFA" "Publisher" "SurfLab, CISE, University of Florida"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\SurfLab_SOFA" "UrlInfoAbout" "https://bitbucket.org/surflab/sofa"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\SurfLab_SOFA" "DisplayVersion" "15.09"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\SurfLab_SOFA" "UninstallString" '"$INSTDIR\surflab-sofa-uninstall.exe"'
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\SurfLab_SOFA" "DisplayIcon" "$INSTDIR\bin\runSOFA.exe"
  WriteRegDWORD HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\SurfLab_SOFA" "NoModify" 1
  WriteRegDWORD HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\SurfLab_SOFA" "NoRepair" 1
  WriteUninstaller "surflab-sofa-uninstall.exe"
  
SectionEnd

; Optional section (can be disabled by the user)
Section "Register filetypes"
  WriteRegStr HKCU "Software\Classes\.scn" "" "SurfLabSOFA.Scene"
  WriteRegStr HKCU "Software\Classes\.scn" "Content Type" "text/xml"
  WriteRegStr HKCU "Software\Classes\.scn" "PerceivedType" "text"
  WriteRegStr HKCU "Software\Classes\SurfLabSOFA.Scene" "" "SOFA simulation scene"
  WriteRegStr HKCU "Software\Classes\SurfLabSOFA.Scene\DefaultIcon" "" "$\"$INSTDIR\bin\runSOFA.exe$\",0"
  WriteRegStr HKCU "Software\Classes\SurfLabSOFA.Scene\shell\open\command" "" "$\"$INSTDIR\bin\runSOFA.exe$\" --msaa 8 $\"%1$\""
  WriteRegStr HKCU "Software\Classes\SurfLabSOFA.Scene\shell\edit" "" "Edit in Modeler"
  WriteRegStr HKCU "Software\Classes\SurfLabSOFA.Scene\shell\edit\command" "" "$\"$INSTDIR\bin\Modeler.exe$\" $\"%1$\""

  WriteRegStr HKCU "Software\Classes\.salua" "" "SurfLabSOFA.LuaScene"
  WriteRegStr HKCU "Software\Classes\.salua" "Content Type" "text/lua"
  WriteRegStr HKCU "Software\Classes\.salua" "PerceivedType" "text"
  WriteRegStr HKCU "Software\Classes\SurfLabSOFA.LuaScene" "" "SOFA simulation scene in Lua format"
  WriteRegStr HKCU "Software\Classes\SurfLabSOFA.LuaScene\DefaultIcon" "" "$\"$INSTDIR\bin\runSOFA.exe$\",0"
  WriteRegStr HKCU "Software\Classes\SurfLabSOFA.LuaScene\shell\open\command" "" "$\"$INSTDIR\bin\runSOFA.exe$\" --msaa 8 $\"%1$\""

SectionEnd

Section "Start Menu Shortcuts"

  CreateDirectory "$SMPROGRAMS\SurfLab SOFA"
  CreateShortCut "$SMPROGRAMS\SurfLab SOFA\Uninstall.lnk" "$INSTDIR\surflab-sofa-uninstall.exe" "" "$INSTDIR\surflab-sofa-uninstall.exe" 0
  CreateShortCut "$SMPROGRAMS\SurfLab SOFA\runSOFA.lnk" "$INSTDIR\bin\runSOFA.exe" "" "$INSTDIR\bin\runSOFA.exe" 0
  CreateShortCut "$SMPROGRAMS\SurfLab SOFA\Modeler.lnk" "$INSTDIR\bin\Modeler.exe" "" "$INSTDIR\bin\Modeler.exe" 0
  
SectionEnd

Section "Desktop Shortcuts"

  CreateShortCut "$DESKTOP\runSOFA.lnk" "$INSTDIR\bin\runSOFA.exe" "" "$INSTDIR\bin\runSOFA.exe" 0
  CreateShortCut "$DESKTOP\Modeler.lnk" "$INSTDIR\bin\Modeler.exe" "" "$INSTDIR\bin\Modeler.exe" 0
  
SectionEnd


;--------------------------------

; Uninstaller

Section "Uninstall"
  
  ; Remove registry keys
  DeleteRegKey HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\SurfLab_SOFA"
  DeleteRegKey HKCU SOFTWARE\SurfLab_SOFA

  DeleteRegKey HKCU "Software\Classes\SurfLabSOFA.Scene"
  DeleteRegKey HKCU "Software\Classes\SurfLabSOFA.LuaScene"
  
  ; Remove files and uninstaller
  RMDir /r $INSTDIR\bin
  RMDir /r $INSTDIR\examples
  RMDir /r $INSTDIR\share 
  RMDir /r $INSTDIR\etc 
  ; RMDir /r $INSTDIR\config
  Delete $INSTDIR\surflab-sofa-uninstall.exe

  ; Remove shortcuts, if any
  Delete "$SMPROGRAMS\SurfLab SOFA\*.*"
  Delete "$DESKTOP\runSOFA.lnk"
  Delete "$DESKTOP\Modeler.lnk"

  ; Remove directories used
  RMDir "$SMPROGRAMS\Surflab SOFA"
  RMDir "$INSTDIR"

SectionEnd
