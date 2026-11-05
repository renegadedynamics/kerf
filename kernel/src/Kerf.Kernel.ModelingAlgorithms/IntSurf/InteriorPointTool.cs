// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_InteriorPointTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_InteriorPointTool.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>Static access to an <see cref="InteriorPoint"/>, the tool IntWalk_IWalking reads its interior points with.</summary>
/// <remarks>
/// OCCT's static class is a <c>readonly struct</c> with static members (decision 338). As in OCCT,
/// <see cref="Direction2d"/> converts the point's 2D vector to a direction (it throws on a null vector).
/// </remarks>
internal readonly struct InteriorPointTool
{
    /// <summary>The 3D point.</summary>
    public static Point3D Value3d(in InteriorPoint pStart) => pStart.Value();

    /// <summary>The parameters on the parametric surface.</summary>
    public static void Value2d(in InteriorPoint pStart, out double u, out double v) =>
        pStart.Parameters(out u, out v);

    /// <summary>The tangent in 3D.</summary>
    public static Vector3D Direction3d(in InteriorPoint pStart) => pStart.Direction();

    /// <summary>The tangent in the parametric space, as a direction.</summary>
    public static Direction2D Direction2d(in InteriorPoint pStart) => new Direction2D(pStart.Direction2d());
}
