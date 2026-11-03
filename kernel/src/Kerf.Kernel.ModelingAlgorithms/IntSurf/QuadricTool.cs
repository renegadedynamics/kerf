// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_QuadricTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_QuadricTool.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_QuadricTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>The services of an <see cref="SurfaceQuadric"/> that the intersection algorithms (IntWalk) call.</summary>
public static class QuadricTool
{
    /// <summary>The value of the implicit function of <paramref name="quad"/> at the point (X, Y, Z).</summary>
    public static double Value(in SurfaceQuadric quad, double x, double y, double z) => quad.Distance(new Point3D(x, y, z));

    /// <summary>The gradient of the implicit function of <paramref name="quad"/> at the point (X, Y, Z).</summary>
    public static void Gradient(in SurfaceQuadric quad, double x, double y, double z, out Vector3D v) =>
        v = quad.Gradient(new Point3D(x, y, z));

    /// <summary>The value and the gradient of the implicit function at the point (X, Y, Z).</summary>
    /// <remarks>As in OCCT, an empty quadric leaves <paramref name="val"/> and <paramref name="grad"/> unchanged.</remarks>
    public static void ValueAndGradient(in SurfaceQuadric quad, double x, double y, double z, ref double val,
                                        ref Vector3D grad) =>
        quad.ValAndGrad(new Point3D(x, y, z), ref val, ref grad);

    /// <summary>The tolerance of the zero of the implicit function.</summary>
    public static double Tolerance(in SurfaceQuadric q)
    {
        switch (q.TypeQuadric())
        {
            case SurfaceKind.Sphere:
                return 2.0e-6 * q.Sphere().Radius;
            case SurfaceKind.Cylinder:
                return 2.0e-6 * q.Cylinder().Radius;
            default:
                break;
        }
        return (1e-6);
    }
}
