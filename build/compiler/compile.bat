@echo off
setlocal
set PATH=%PATH%;../backend_win/builder/bin/Release/

echo Building minified version...
builder.exe

echo Running YUI Compressor...
java.exe -jar yuicompressor-2.4.8pre.jar --nomunge -o ../jsbox2d.min.js ../jsbox2d.js

echo Building debug version...
builder.exe pArgs="DEBUG"
endlocal