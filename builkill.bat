@echo off
setlocal

set "target=build"

for /d /r %%i in (*) do (
    if exist "%%i\%target%" (
        echo Deleting folder: "%%i\%target%"
        rd /s /q "%%i\%target%"
    )
)

endlocal