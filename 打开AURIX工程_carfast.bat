@echo off
set ADS=C:\Infineon\AURIX-Studio-1.9.12\AURIX-studio.exe
set WS=C:\Users\23898\Desktop\carfast\.ads-workspace
if not exist "%WS%" mkdir "%WS%"
start "AURIX Development Studio" "%ADS%" -data "%WS%" -showLocation
