// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// TEMPORARY API stubs (unit 6B) for the forks; removed when IntTools_Tools and BOPTools_AlgoTools are ported.
public static partial class IntTools_Tools
{
    /// <summary>Stub.</summary>
    public static bool IsDirsCoinside(in gp_Dir D1, in gp_Dir D2) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static bool IsDirsCoinside(in gp_Dir D1, in gp_Dir D2, double aTol) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static bool ComputeTolerance(Geom_Curve theCurve3D, Geom2d_Curve theCurve2D, Geom_Surface theSurf, double theFirst, double theLast, out double theMaxDist, out double theMaxPar) => throw new NotImplementedException();
    /// <summary>Stub.</summary>
    public static bool ComputeTolerance(Geom_Curve theCurve3D, Geom2d_Curve theCurve2D, Geom_Surface theSurf, double theFirst, double theLast, out double theMaxDist, out double theMaxPar, double theTolRange, bool theToRunParallel = false) => throw new NotImplementedException();
}

