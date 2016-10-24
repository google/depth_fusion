cmake_minimum_required(VERSION 3.6)

project(artoolkit)

set(SRCDIR "src/")
set(AR_INCLUDES)
list (APPEND AR_INCLUDES
  "include"
)


set (AR_SRC)
list (APPEND AR_SRC
include/AR/ar.h
include/AR/param.h
include/AR/matrix.h
include/AR/config.h
${SRCDIR}arDetectMarker.c
${SRCDIR}arDetectMarker2.c
${SRCDIR}arGetCode.c
${SRCDIR}arGetMarkerInfo.c
${SRCDIR}arGetTransMat.c
${SRCDIR}arGetTransMat2.c
${SRCDIR}arGetTransMat3.c
${SRCDIR}arGetTransMatCont.c
${SRCDIR}arLabeling.c
${SRCDIR}arUtil.c
${SRCDIR}Makefile.in
${SRCDIR}mAlloc.c
${SRCDIR}mAllocDup.c
${SRCDIR}mAllocInv.c
${SRCDIR}mAllocMul.c
${SRCDIR}mAllocTrans.c
${SRCDIR}mAllocUnit.c
${SRCDIR}mDet.c
${SRCDIR}mDisp.c
${SRCDIR}mDup.c
${SRCDIR}mFree.c
${SRCDIR}mInv.c
${SRCDIR}mMul.c
${SRCDIR}mPCA.c
${SRCDIR}mSelfInv.c
${SRCDIR}mTrans.c
${SRCDIR}mUnit.c
${SRCDIR}paramChangeSize.c
${SRCDIR}paramDecomp.c
${SRCDIR}paramDisp.c
${SRCDIR}paramDistortion.c
${SRCDIR}paramFile.c
${SRCDIR}paramGet.c
${SRCDIR}vAlloc.c
${SRCDIR}vDisp.c
${SRCDIR}vFree.c
${SRCDIR}vHouse.c
${SRCDIR}vInnerP.c
${SRCDIR}vTridiag.c
${SRCDIR}arMultiActivate.c
${SRCDIR}arMultiGetTransMat.c
${SRCDIR}arMultiReadConfigFile.c
)

add_library(artoolkit STATIC ${AR_SRC})
target_include_directories(artoolkit PUBLIC ${AR_INCLUDES})

