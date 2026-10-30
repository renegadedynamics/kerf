// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_IType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The kind of a bisecting curve: line, circle, ellipse, parabola, hyperbola or point.</summary>
public enum GccInt_IType
{
    GccInt_Lin,
    GccInt_Cir,
    GccInt_Ell,
    GccInt_Par,
    GccInt_Hpr,
    GccInt_Pnt,
}
