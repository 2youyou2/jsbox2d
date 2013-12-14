@echo off
builder.exe

setlocal
set PATH=%PATH%;../backend_win/builder/bin/Release/

echo Running YUI Compressor...
java.exe -jar yuicompressor-2.4.8pre.jar --nomunge -o ../jsbox2d.min.js ../jsbox2d.js
endlocal