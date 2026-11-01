// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyMask.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The bit masks of the triangles' and edges' flags of the polygonal hidden-line removal.</summary>
/// <remarks>
/// OCCT's unscoped enum is only ever combined with the integer flags (<c>Flags &amp; HLRAlgo_PolyMask_FMskBack</c>), so
/// the port makes its members integer constants; code that writes <c>using static Occt.HLRAlgo_PolyMask;</c> reads as
/// the C++ does (decision 10105).
/// </remarks>
public static class HLRAlgo_PolyMask
{
    public const int HLRAlgo_PolyMask_EMskOutLin1 = 1;
    public const int HLRAlgo_PolyMask_EMskOutLin2 = 2;
    public const int HLRAlgo_PolyMask_EMskOutLin3 = 4;
    public const int HLRAlgo_PolyMask_EMskGrALin1 = 8;
    public const int HLRAlgo_PolyMask_EMskGrALin2 = 16;
    public const int HLRAlgo_PolyMask_EMskGrALin3 = 32;
    public const int HLRAlgo_PolyMask_FMskBack = 64;
    public const int HLRAlgo_PolyMask_FMskSide = 128;
    public const int HLRAlgo_PolyMask_FMskHiding = 256;
    public const int HLRAlgo_PolyMask_FMskFlat = 512;
    public const int HLRAlgo_PolyMask_FMskOnOutL = 1024;
    public const int HLRAlgo_PolyMask_FMskOrBack = 2048;
    public const int HLRAlgo_PolyMask_FMskFrBack = 4096;
}
