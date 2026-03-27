@echo off

del /s *.ewt
del /s *.dep
del /s *.wsdt
del /s *.scvd
del /s *.uvgui.*
del /s *.uvguix.*

if exist Samples\LibSamples\settings (

    rmdir /q/s Samples\LibSamples\settings
)

for /d %%a in (Samples\LibSamples\*) do (

    for /d %%b in (%%a\*) do (

        for /d %%c in (%%b\*) do (

            for /d %%d in (%%c\*) do (

                echo %%d

                rmdir /q/s %%d
            )
        )
    )
)

pause