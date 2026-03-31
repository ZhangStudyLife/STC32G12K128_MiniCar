@echo off
setlocal

set "MDK_DIR=%~dp0Project\mdk"
if not exist "%MDK_DIR%\" exit /b 1

pushd "%MDK_DIR%"
rd "out_file" /s /q 2>nul
del "seekfree.uvg*.*" /q 2>nul
del "seekfree.uvopt" /q 2>nul
popd

exit /b 0
