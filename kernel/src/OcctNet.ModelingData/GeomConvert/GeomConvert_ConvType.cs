// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_ConvType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>How GeomConvert_CurveToAnaCurve (and GeomConvert_SurfToAnaSurf) choose the analytical result.</summary>
public enum GeomConvert_ConvType
{
    /// <summary>Only the target type is tried.</summary>
    GeomConvert_Target,

    /// <summary>The simplest type within the tolerance (line, then circle, then ellipse).</summary>
    GeomConvert_Simplest,

    /// <summary>The type with the smallest deviation.</summary>
    GeomConvert_MinGap,
}
