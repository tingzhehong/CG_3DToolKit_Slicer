QT       += core gui charts script

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
    src/alg/CGImageFormatConvert.cpp \
    src/alg/CGModeling.cpp \
    src/alg/CGPointCloud.cpp \
    src/block/NodeBlock.cpp \
    src/block/NodeBlockFactory.cpp \
    src/block/NodeBlockManager.cpp \
    src/block/NodeBlockWidget.cpp \
    src/block/functions2d/Functions2DLocalNodeBlock.cpp \
    src/block/functions2d/Functions2DSourceNodeBlock.cpp \
    src/block/functions2d/Functions2DTerminalNodeBlock.cpp \
    src/block/functions3d/Functions3DLocalNodeBlock.cpp \
    src/block/functions3d/Functions3DSourceNodeBlock.cpp \
    src/block/functions3d/Functions3DTerminalNodeBlock.cpp \
    src/block/logics/LogicsCirculate.cpp \
    src/block/logics/LogicsCondition.cpp \
    src/block/logics/LogicsGroup.cpp \
    src/block/logics/LogicsProfile.cpp \
    src/block/logics/LogicsScriptCpp.cpp \
    src/block/maths/MathAddNodeBlock.cpp \
    src/block/maths/MathDivNodeBlock.cpp \
    src/block/maths/MathMulNodeBlock.cpp \
    src/block/maths/MathSubNodeBlock.cpp \
    src/block/maths/NumberInputNodeBlock.cpp \
    src/block/maths/NumberOutputNodeBlock.cpp \
    src/block/maths/ValueTerminalNodeBlock.cpp \
    src/com/CGGlobalVariable.cpp \
    src/com/CGOCVHeader.cpp \
    src/com/CGPCLHeader.cpp \
    src/com/CGVTKHeader.cpp \
    src/node/GroupItem.cpp \
    src/node/NodeItem.cpp \
    src/node/NodeView.cpp \
    src/node/PortItem.cpp \
    src/node/RopeItem.cpp \
    src/plugin/AlgorithmInterface.cpp \
    src/plugin/AlgorithmNodeBlock.cpp \
    src/plugin/PluginManager.cpp \
    src/script/CGScriptFunction.cpp \
    src/tool/CGGraphicsCircleItem.cpp \
    src/tool/CGGraphicsLineItem.cpp \
    src/tool/CGGraphicsLineItemHorizontal.cpp \
    src/tool/CGGraphicsLineItemVertical.cpp \
    src/tool/CGGraphicsRectItem.cpp \
    src/tool/CGImage2DGraphicsItemAdapter.cpp \
    src/tool/CGImage3DGraphicsItemAdapter.cpp \
    src/tool/CGImage3DSectionItemHorizontal.cpp \
    src/tool/CGImage3DSectionItemVertical.cpp \
    src/tool/CGImage3DSectionLineItem.cpp \
    src/tool/CGShapeBaseItem.cpp \
    src/tool/CGShapeCircleItem.cpp \
    src/tool/CGShapeConcentricCircleItem.cpp \
    src/tool/CGShapeControlItem.cpp \
    src/tool/CGShapeLineItem.cpp \
    src/tool/CGShapePolygonItem.cpp \
    src/tool/CGShapeRectItem.cpp \
    src/tool/CGShapeRotateRectangleItem.cpp \
    src/ui/CGAboutDialog.cpp \
    src/ui/CGAlgorithmArgumentsDialog.cpp \
    src/ui/CGDepthImageDialog.cpp \
    src/ui/CGDisOrderDialog.cpp \
    src/ui/CGLocalDataFileDialog.cpp \
    src/ui/CGProfileForm2D.cpp \
    src/ui/CGProfileForm3D.cpp \
    src/ui/CGPropertiesForm1.cpp \
    src/ui/CGPropertiesForm2.cpp \
    src/ui/CGScriptCppEditor.cpp \
    src/ui/CGUsersLoginDialog.cpp \
    src/ui/CGValueIndicator.cpp \
    src/ui/CGWaitingDialog.cpp \
    src/util/CGBaseTreeWidget.cpp \
    src/util/CGBaseWidget.cpp \
    src/util/CGPropertiesRegulator.cpp \
    src/util/CGSubWindowWidget.cpp \
    src/util/CGViewRegulator.cpp \
    src/view/CG2DImageView.cpp \
    src/view/CG3DImageView.cpp \
    src/view/CGChartView.cpp \
    src/view/CGConsoleView.cpp \
    src/view/CGDataTreeView.cpp \
    src/view/CGFullScreenView.cpp \
    src/view/CGGraphicsScene.cpp \
    src/view/CGGraphicsView.cpp \
    src/view/CGNodeView.cpp \
    src/view/CGProfileView.cpp \
    src/view/CGProjectTreeView.cpp \
    src/view/CGPropertiesView.cpp \
    src/view/CGWebView.cpp \
    src/vtk/CGAbstractWidgetObserver.cpp \
    src/vtk/CGAngleWidgetObserver.cpp \
    src/vtk/CGAreaPickerInteractorStyle.cpp \
    src/vtk/CGBoxWidgetObserver.cpp \
    src/vtk/CGDistanceWidgetObserver.cpp \
    src/vtk/CGMouseEventInteractorStyle.cpp \
    src/vtk/CGMoveActorInteractorStyle.cpp \
    src/vtk/CGPlaneWidgetObserver.cpp \
    src/vtk/CGPointPickObserver.cpp \
    src/vtk/CGPointPickerInteractorStyle.cpp \
    src/vtk/CGSphereWidgetObserver.cpp \
    src/vtk/CGVTKWidget.cpp \
    src/vtk/CGVTKUtils.cpp \
    src/vtk/CGActorExporter.cpp \
    src/vtk/utils/Point3f.cpp \
    src/vtk/utils/Vector4f.cpp \
    src/vtk/utils/Utils.cpp

HEADERS += \
    mainwindow.h \
    src/alg/CGCSV.h \
    src/alg/CGImage.h \
    src/alg/CGImageFormatConvert.h \
    src/alg/CGModeling.h \
    src/alg/CGPointCloud.h \
    src/block/NodeBlock.h \
    src/block/NodeBlockFactory.h \
    src/block/NodeBlockManager.h \
    src/block/NodeBlockWidget.h \
    src/block/functions2d/Functions2DLocalNodeBlock.h \
    src/block/functions2d/Functions2DSourceNodeBlock.h \
    src/block/functions2d/Functions2DTerminalNodeBlock.h \
    src/block/functions3d/Functions3DLocalNodeBlock.h \
    src/block/functions3d/Functions3DSourceNodeBlock.h \
    src/block/functions3d/Functions3DTerminalNodeBlock.h \
    src/block/logics/LogicsCirculate.h \
    src/block/logics/LogicsCondition.h \
    src/block/logics/LogicsGroup.h \
    src/block/logics/LogicsProfile.h \
    src/block/logics/LogicsScriptCpp.h \
    src/block/maths/MathAddNodeBlock.h \
    src/block/maths/MathDivNodeBlock.h \
    src/block/maths/MathMulNodeBlock.h \
    src/block/maths/MathSubNodeBlock.h \
    src/block/maths/NumberInputNodeBlock.h \
    src/block/maths/NumberOutputNodeBlock.h \
    src/block/maths/ValueTerminalNodeBlock.h \
    src/com/CGGlobalVariable.h \
    src/com/CGMetaType.h \
    src/com/CGOCVHeader.h \
    src/com/CGPCLHeader.h \
    src/com/CGVTKHeader.h \
    src/com/CGCommon.h \
    src/node/GroupItem.h \
    src/node/NodeItem.h \
    src/node/NodeView.h \
    src/node/PortItem.h \
    src/node/RopeItem.h \
    src/plugin/AlgorithmInterface.h \
    src/plugin/AlgorithmNodeBlock.h \
    src/plugin/PluginManager.h \
    src/script/CGScriptFunction.h \
    src/tool/CGGraphicsCircleItem.h \
    src/tool/CGGraphicsLineItem.h \
    src/tool/CGGraphicsLineItemHorizontal.h \
    src/tool/CGGraphicsLineItemVertical.h \
    src/tool/CGGraphicsRectItem.h \
    src/tool/CGImage2DGraphicsItemAdapter.h \
    src/tool/CGImage3DGraphicsItemAdapter.h \
    src/tool/CGImage3DSectionItemHorizontal.h \
    src/tool/CGImage3DSectionItemVertical.h \
    src/tool/CGImage3DSectionLineItem.h \
    src/tool/CGShapeBaseItem.h \
    src/tool/CGShapeCircleItem.h \
    src/tool/CGShapeConcentricCircleItem.h \
    src/tool/CGShapeControlItem.h \
    src/tool/CGShapeLineItem.h \
    src/tool/CGShapePolygonItem.h \
    src/tool/CGShapeRectItem.h \
    src/tool/CGShapeRotateRectangleItem.h \
    src/ui/CGAboutDialog.h \
    src/ui/CGAlgorithmArgumentsDialog.h \
    src/ui/CGDepthImageDialog.h \
    src/ui/CGDisOrderDialog.h \
    src/ui/CGLocalDataFileDialog.h \
    src/ui/CGProfileForm2D.h \
    src/ui/CGProfileForm3D.h \
    src/ui/CGPropertiesForm1.h \
    src/ui/CGPropertiesForm2.h \
    src/ui/CGScriptCppEditor.h \
    src/ui/CGUsersLoginDialog.h \
    src/ui/CGValueIndicator.h \
    src/ui/CGWaitingDialog.h \
    src/util/CGBaseTreeWidget.h \
    src/util/CGBaseWidget.h \
    src/util/CGPropertiesRegulator.h \
    src/util/CGSubWindowWidget.h \
    src/util/CGViewRegulator.h \
    src/view/CG2DImageView.h \
    src/view/CG3DImageView.h \
    src/view/CGChartView.h \
    src/view/CGConsoleView.h \
    src/view/CGDataTreeView.h \
    src/view/CGFullScreenView.h \
    src/view/CGGraphicsScene.h \
    src/view/CGGraphicsView.h \
    src/view/CGNodeView.h \
    src/view/CGProfileView.h \
    src/view/CGProjectTreeView.h \
    src/view/CGPropertiesView.h \
    src/view/CGWebView.h \
    src/vtk/CGAbstractWidgetObserver.h \
    src/vtk/CGAngleWidgetObserver.h \
    src/vtk/CGAreaPickerInteractorStyle.h \
    src/vtk/CGBoxWidgetObserver.h \
    src/vtk/CGDistanceWidgetObserver.h \
    src/vtk/CGMouseEventInteractorStyle.h \
    src/vtk/CGMoveActorInteractorStyle.h \
    src/vtk/CGPlaneWidgetObserver.h \
    src/vtk/CGPointPickObserver.h \
    src/vtk/CGPointPickerInteractorStyle.h \
    src/vtk/CGSphereWidgetObserver.h \
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
    ./src/block \
    ./src/block/maths \
    ./src/block/logics \
    ./src/block/functions2d \
    ./src/block/functions3d \
    ./src/com \
    ./src/dev \
    ./src/node \
    ./src/plugin \
    ./src/script \
    ./src/tool \
    ./src/ui \
    ./src/util \
    ./src/view \
    ./src/vtk \
    ./src/vtk/utils

RESOURCES += \
    icon.qrc \
    qss.qrc

LIBS += -luser32

# OCCT
include($$PWD\src\occ\occt.pri)

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
