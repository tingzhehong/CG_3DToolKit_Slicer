# OpenCV
INCLUDEPATH += $$PWD\..\..\lib\opencv\build\include
INCLUDEPATH += $$PWD\..\..\lib\opencv\build\include\opencv
INCLUDEPATH += $$PWD\..\..\lib\opencv\build\include\opencv2

# OpenCV release
CONFIG(debug,debug|release){
LIBS += $$PWD\..\..\lib\opencv\build\x64\vc14\lib\opencv_world341d.lib
}

CONFIG(release,debug|release){
LIBS += $$PWD\..\..\lib\opencv\build\x64\vc14\lib\opencv_world341.lib
}
