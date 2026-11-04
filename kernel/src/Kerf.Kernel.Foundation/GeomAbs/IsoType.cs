// Port of: src/FoundationClasses/TKMath/GeomAbs/GeomAbs_IsoType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Geometry;

/// <summary>Whether a curve is a U isoparametric, a V isoparametric, or neither.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.GeomAbs_IsoType;</c>.</remarks>
internal enum IsoType
{
    IsoU,
    IsoV,
    NoneIso,
}
