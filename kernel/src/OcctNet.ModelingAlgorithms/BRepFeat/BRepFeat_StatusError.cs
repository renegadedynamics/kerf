// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_StatusError.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Describes the error of a feature construction.</summary>
public enum BRepFeat_StatusError
{
    BRepFeat_OK,
    BRepFeat_BadDirect,
    BRepFeat_BadIntersect,
    BRepFeat_EmptyBaryCurve,
    BRepFeat_EmptyCutResult,
    BRepFeat_FalseSide,
    BRepFeat_IncDirection,
    BRepFeat_IncSlidFace,
    BRepFeat_IncParameter,
    BRepFeat_IncTypes,
    BRepFeat_IntervalOverlap,
    BRepFeat_InvFirstShape,
    BRepFeat_InvOption,
    BRepFeat_InvShape,
    BRepFeat_LocOpeNotDone,
    BRepFeat_LocOpeInvNotDone,
    BRepFeat_NoExtFace,
    BRepFeat_NoFaceProf,
    BRepFeat_NoGluer,
    BRepFeat_NoIntersectF,
    BRepFeat_NoIntersectU,
    BRepFeat_NoParts,
    BRepFeat_NoProjPt,
    BRepFeat_NotInitialized,
    BRepFeat_NotYetImplemented,
    BRepFeat_NullRealTool,
    BRepFeat_NullToolF,
    BRepFeat_NullToolU,
}
