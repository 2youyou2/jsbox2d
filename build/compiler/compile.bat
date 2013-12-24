@echo off
setlocal
set PATH=%PATH%;../backend_win/builder/bin/Release/

echo Building minified version...
builder.exe

echo Running Closure Compiler...
java.exe -jar closurecompiler.jar --js ../jsbox2d.js --js_output_file ../jsbox2d.min.js

echo Building debug version...
builder.exe pArgs="DEBUG"
endlocal