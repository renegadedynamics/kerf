// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PathPointTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PathPointTool.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>Static access to an <see cref="PathPoint"/>, the tool IntWalk_IWalking reads its start points with.</summary>
/// <remarks>
/// OCCT's static class is a <c>readonly struct</c> with static members (decision 338), so that it can be a type
/// argument if IntWalk's generic needs one.
/// </remarks>
public readonly struct PathPointTool
{
    /// <summary>The 3D point.</summary>
    public static Point3D Value3d(in PathPoint pStart) => pStart.Value();

    /// <summary>The first pair of parameters.</summary>
    public static void Value2d(in PathPoint pStart, out double u, out double v) => pStart.Value2d(out u, out v);

    public static bool IsPassingPnt(in PathPoint pStart) => pStart.IsPassingPnt;

    public static bool IsTangent(in PathPoint pStart) => pStart.IsTangent;

    public static Vector3D Direction3d(in PathPoint pStart) => pStart.Direction3d();

    public static Direction2D Direction2d(in PathPoint pStart) => pStart.Direction2d();

    public static int Multiplicity(in PathPoint pStart) => pStart.Multiplicity();

    public static void Parameters(in PathPoint pStart, int mult, out double u, out double v) =>
        pStart.Parameters(mult, out u, out v);
}
