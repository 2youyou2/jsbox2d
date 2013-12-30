@echo off
setlocal
set PATH=%PATH%;../backend_win/builder/bin/Release/

REM echo Building minified version...
REM builder.exe pArgs="LIQUIDFUN"
REM java.exe -jar closurecompiler.jar --js ../jsbox2d.js --js_output_file ../jsbox2d.min.js

echo Building Box2D Debug...
builder.exe pArgs="DEBUG" input="build.inc" output="jsbox2d.js"

echo Building LiquidFun Debug...
builder.exe pArgs="DEBUG;LIQUIDFUN" input="build.inc" output="jsliquidfun.js"

echo Building Box2D Release...
builder.exe input="build.inc" output="jsbox2d.min.js"
java.exe -jar closurecompiler.jar --js ../jsbox2d.min.js --js_output_file ../jsbox2d.min.js

echo Building LiquidFun Release...
builder.exe pArgs="LIQUIDFUN" input="build.inc" output="jsliquidfun.min.js"
java.exe -jar closurecompiler.jar --js ../jsliquidfun.min.js --js_output_file ../jsliquidfun.min.js

endlocal