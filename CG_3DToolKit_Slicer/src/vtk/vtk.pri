CONFIG(release, debug|release) {
    BUILD_TYPE = release
}
else {
    BUILD_TYPE = debug
}

VTKPATH = "D:/VTK/VTK 8.0"
VTKVER = "8.0"

INCLUDEPATH += $$quote($${VTKPATH}/include/vtk-$${VTKVER})

LIBS += $$quote($${VTKPATH}/lib/vtkalglib-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkChartsCore-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkCommonColor-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkCommonComputationalGeometry-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkCommonCore-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkCommonDataModel-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkCommonExecutionModel-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkCommonMath-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkCommonMisc-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkCommonSystem-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkCommonTransforms-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkDICOMParser-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkDomainsChemistry-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkexoIIc-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkexpat-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersAMR-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersCore-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersExtraction-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersFlowPaths-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersGeneral-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersGeneric-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersGeometry-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersHybrid-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersHyperTree-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersImaging-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersModeling-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersParallel-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersParallelImaging-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersPoints-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersProgrammable-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersSelection-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersSMP-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersSources-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersStatistics-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersTexture-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersTopology-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkFiltersVerdict-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkfreetype-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkGeovisCore-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkgl2ps-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkGUISupportQt-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkGUISupportQtOpenGL-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkGUISupportQtSQL-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkhdf5-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkhdf5_hl-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkImagingColor-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkImagingCore-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkImagingFourier-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkImagingGeneral-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkImagingHybrid-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkImagingMath-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkImagingMorphological-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkImagingSources-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkImagingStatistics-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkImagingStencil-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkInfovisCore-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkInfovisLayout-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkInteractionImage-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkInteractionStyle-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkInteractionWidgets-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOAMR-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOCore-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOEnSight-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOExodus-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOExport-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOExportOpenGL-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOGeometry-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOImage-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOImport-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOInfovis-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOLegacy-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOLSDyna-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOMINC-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOMovie-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIONetCDF-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOParallel-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOParallelXML-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOPLY-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOSQL-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOTecplotTable-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOVideo-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOXML-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkIOXMLParser-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkjpeg-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkjsoncpp-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtklibharu-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtklibxml2-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtklz4-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkmetaio-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkNetCDF-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtknetcdf_c++.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkoggtheora-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkParallelCore-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkpng-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkproj4-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkRenderingAnnotation-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkRenderingContext2D-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkRenderingContextOpenGL-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkRenderingCore-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkRenderingFreeType-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkRenderingGL2PS-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkRenderingImage-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkRenderingLabel-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkRenderingLIC-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkRenderingLOD-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkRenderingOpenGL-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkRenderingQt-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkRenderingVolume-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkRenderingVolumeOpenGL-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtksqlite-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtksys-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtktiff-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkverdict-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkViewsContext2D-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkViewsCore-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkViewsInfovis-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkViewsQt-$${VTKVER}.lib)
LIBS += $$quote($${VTKPATH}/lib/vtkzlib-$${VTKVER}.lib)
LIBS += opengl32.lib
