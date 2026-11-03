// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_StatusError.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Features;

/// <summary>Describes the error of a feature construction.</summary>
public enum StatusError
{
    OK,
    BadDirect,
    BadIntersect,
    EmptyBaryCurve,
    EmptyCutResult,
    FalseSide,
    IncDirection,
    IncSlidFace,
    IncParameter,
    IncTypes,
    IntervalOverlap,
    InvFirstShape,
    InvOption,
    InvShape,
    LocOpeNotDone,
    LocOpeInvNotDone,
    NoExtFace,
    NoFaceProf,
    NoGluer,
    NoIntersectF,
    NoIntersectU,
    NoParts,
    NoProjPt,
    NotInitialized,
    NotYetImplemented,
    NullRealTool,
    NullToolF,
    NullToolU,
}
