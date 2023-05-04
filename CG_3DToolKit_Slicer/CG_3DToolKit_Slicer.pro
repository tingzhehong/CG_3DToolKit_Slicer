QT       += core gui charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

QMAKE_CXXFLAGS -= -Zc:strictStrings

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    src/alg/CGCSV.cpp \
    src/alg/CGImage.cpp \
    src/alg/CGPointCloud.cpp \
    src/com/CGOCVHeader.cpp \
    src/com/CGPCLHeader.cpp \
    src/node/GroupItem.cpp \
    src/node/NodeItem.cpp \
    src/node/NodeView.cpp \
    src/node/PortItem.cpp \
    src/node/RopeItem.cpp \
    src/tool/CGGraphicsCircleItem.cpp \
    src/tool/CGGraphicsLineItem.cpp \
    src/tool/CGGraphicsLineItemHorizontal.cpp \
    src/tool/CGGraphicsLineItemVertical.cpp \
    src/tool/CGGraphicsRectItem.cpp \
    src/tool/CGImage2DGraphicsItemAdapter.cpp \
    src/ui/CGAboutDialog.cpp \
    src/ui/CGDepthImageDialog.cpp \
    src/ui/CGDisOrderDialog.cpp \
    src/ui/CGProfileForm2D.cpp \
    src/ui/CGProfileForm3D.cpp \
    src/ui/CGPropertiesForm1.cpp \
    src/ui/CGPropertiesForm2.cpp \
    src/ui/CGUsersLoginDialog.cpp \
    src/ui/CGWaitingDialog.cpp \
    src/util/CGBaseTreeWidget.cpp \
    src/util/CGBaseWidget.cpp \
    src/alg/CGImageFormatConvert.cpp \
    src/util/CGSubWindowWidget.cpp \
    src/util/CGViewRegulator.cpp \
    src/view/CG2DImageView.cpp \
    src/view/CG3DImageView.cpp \
    src/view/CGChartView.cpp \
    src/view/CGConsoleView.cpp \
    src/view/CGDataTreeView.cpp \
    src/view/CGGraphicsView.cpp \
    src/view/CGNodeView.cpp \
    src/view/CGProfileView.cpp \
    src/view/CGProjectTreeView.cpp \
    src/view/CGPropertiesView.cpp \
    src/vtk/CGAbstractWidgetObserver.cpp \
    src/vtk/CGAngleWidgetObserver.cpp \
    src/vtk/CGBoxWidgetObserver.cpp \
    src/vtk/CGDistanceWidgetObserver.cpp \
    src/vtk/CGPointPickObserver.cpp \
    src/vtk/CGVTKWidget.cpp \
    src/vtk/CGVTKutils.cpp \
    src/vtk/CGActorExporter.cpp \
    src/vtk/utils/Point3f.cpp \
    src/vtk/utils/Vector4f.cpp \
    src/vtk/utils/Utils.cpp

HEADERS += \
    mainwindow.h \
    src/alg/CGCSV.h \
    src/alg/CGImage.h \
    src/alg/CGPointCloud.h \
    src/com/CGOCVHeader.h \
    src/com/CGPCLHeader.h \
    src/com/CGVTKHeader.h \
    src/com/CGCommon.h \
    src/node/GroupItem.h \
    src/node/NodeItem.h \
    src/node/NodeView.h \
    src/node/PortItem.h \
    src/node/RopeItem.h \
    src/tool/CGGraphicsCircleItem.h \
    src/tool/CGGraphicsLineItem.h \
    src/tool/CGGraphicsLineItemHorizontal.h \
    src/tool/CGGraphicsLineItemVertical.h \
    src/tool/CGGraphicsRectItem.h \
    src/tool/CGImage2DGraphicsItemAdapter.h \
    src/ui/CGAboutDialog.h \
    src/ui/CGDepthImageDialog.h \
    src/ui/CGDisOrderDialog.h \
    src/ui/CGProfileForm2D.h \
    src/ui/CGProfileForm3D.h \
    src/ui/CGPropertiesForm1.h \
    src/ui/CGPropertiesForm2.h \
    src/ui/CGUsersLoginDialog.h \
    src/ui/CGWaitingDialog.h \
    src/util/CGBaseTreeWidget.h \
    src/util/CGBaseWidget.h \
    src/alg/CGImageFormatConvert.h \
    src/util/CGSubWindowWidget.h \
    src/util/CGViewRegulator.h \
    src/view/CG2DImageView.h \
    src/view/CG3DImageView.h \
    src/view/CGChartView.h \
    src/view/CGConsoleView.h \
    src/view/CGDataTreeView.h \
    src/view/CGGraphicsView.h \
    src/view/CGNodeView.h \
    src/view/CGProfileView.h \
    src/view/CGProjectTreeView.h \
    src/view/CGPropertiesView.h \
    src/vtk/CGAbstractWidgetObserver.h \
    src/vtk/CGAngleWidgetObserver.h \
    src/vtk/CGBoxWidgetObserver.h \
    src/vtk/CGDistanceWidgetObserver.h \
    src/vtk/CGPointPickObserver.h \
    src/vtk/CGPointPickerInteractorStyle.h \
    src/vtk/CGVTKUtils.h \
    src/vtk/CGVTKWidget.h \
    src/vtk/CGActorExporter.h \
    src/vtk/CGRenderersLayoutAlg.h \
    src/vtk/utils/Point3f.h \
    src/vtk/utils/Vector4f.h \
    src/vtk/utils/Utils.h

FORMS += \
    mainwindow.ui

INCLUDEPATH += \
    ./src/alg \
    ./src/com \
    ./src/dev \
    ./src/node \
    ./src/tool \
    ./src/ui \
    ./src/util \
    ./src/view \
    ./src/vtk \
    ./src/vtk/utils

RESOURCES += \
    icon.qrc \
    qss.qrc

# OpenCV
include($$PWD\src\ocv\opencv.pri)

# PCL
include($$PWD\src\pcl\pcl.pri)

# ICONS
RC_ICONS = $$PWD\slicer.ico

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
