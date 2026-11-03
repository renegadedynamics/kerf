// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PathPointTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PathPointTool.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>Static access to an <see cref="IntSurf_PathPoint"/>, the tool IntWalk_IWalking reads its start points with.</summary>
/// <remarks>
/// OCCT's static class is a <c>readonly struct</c> with static members (decision 338), so that it can be a type
/// argument if IntWalk's generic needs one.
/// </remarks>
public readonly struct IntSurf_PathPointTool
{
    /// <summary>The 3D point.</summary>
    public static Point3D Value3d(in IntSurf_PathPoint PStart) => PStart.Value();

    /// <summary>The first pair of parameters.</summary>
    public static void Value2d(in IntSurf_PathPoint PStart, out double U, out double V) => PStart.Value2d(out U, out V);

    public static bool IsPassingPnt(in IntSurf_PathPoint PStart) => PStart.IsPassingPnt();

    public static bool IsTangent(in IntSurf_PathPoint PStart) => PStart.IsTangent();

    public static Vector3D Direction3d(in IntSurf_PathPoint PStart) => PStart.Direction3d();

    public static Direction2D Direction2d(in IntSurf_PathPoint PStart) => PStart.Direction2d();

    public static int Multiplicity(in IntSurf_PathPoint PStart) => PStart.Multiplicity();

    public static void Parameters(in IntSurf_PathPoint PStart, int Mult, out double U, out double V) =>
        PStart.Parameters(Mult, out U, out V);
}
