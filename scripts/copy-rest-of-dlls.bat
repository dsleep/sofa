echo "Usage: copy-rest-of-dlls <boost library dir> <qt bin dir>"

set DEST=.
rem set BOOST_LIBRARY_DIR=C:\opt\boost_1_60_0\lib32-msvc-12.0
rem set QTBINDIR=C:\opt\Qt5.5.1\5.5\msvc2013\bin 
set BOOST_LIBRARY_DIR=%1
set QTBINDIR=%2

copy  %BOOST_LIBRARY_DIR%\boost_chrono-vc120-mt-1_60.dll %DEST%
copy  %BOOST_LIBRARY_DIR%\boost_date_time-vc120-mt-1_60.dll %DEST%
copy  %BOOST_LIBRARY_DIR%\boost_thread-vc120-mt-1_60.dll %DEST%
copy  %BOOST_LIBRARY_DIR%\boost_system-vc120-mt-1_60.dll %DEST%

copy  %QTBINDIR%\Qt5Gui.dll %DEST%
copy  %QTBINDIR%\Qt5Core.dll %DEST%
copy  %QTBINDIR%\Qt5Widgets.dll %DEST%
copy  %QTBINDIR%\Qt5OpenGL.dll %DEST%
copy  %QTBINDIR%\libEGL.dll %DEST%
copy  %QTBINDIR%\libGLESv2.dll %DSET%
copy  %QTBINDIR%\d3dcompiler_??.dll %DEST%
mkdir %DEST%\platforms
copy  %QTBINDIR%\..\plugins\platforms\qwindows.dll %DEST%\platforms

copy %windir%\syswow64\msvc?120.dll %DEST%

copy c:\OpenHaptics\Academic\3.2\lib\Win32\ReleaseAcademicEdition\*.dll %DEST%

