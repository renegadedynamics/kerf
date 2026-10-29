// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_QuadricTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_QuadricTool.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_QuadricTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>The services of an <see cref="IntSurf_Quadric"/> that the intersection algorithms (IntWalk) call.</summary>
public static class IntSurf_QuadricTool
{
    /// <summary>The value of the implicit function of <paramref name="Quad"/> at the point (X, Y, Z).</summary>
    public static double Value(in IntSurf_Quadric Quad, double X, double Y, double Z) => Quad.Distance(new gp_Pnt(X, Y, Z));

    /// <summary>The gradient of the implicit function of <paramref name="Quad"/> at the point (X, Y, Z).</summary>
    public static void Gradient(in IntSurf_Quadric Quad, double X, double Y, double Z, out gp_Vec V) =>
        V = Quad.Gradient(new gp_Pnt(X, Y, Z));

    /// <summary>The value and the gradient of the implicit function at the point (X, Y, Z).</summary>
    /// <remarks>As in OCCT, an empty quadric leaves <paramref name="Val"/> and <paramref name="Grad"/> unchanged.</remarks>
    public static void ValueAndGradient(in IntSurf_Quadric Quad, double X, double Y, double Z, ref double Val,
                                        ref gp_Vec Grad) =>
        Quad.ValAndGrad(new gp_Pnt(X, Y, Z), ref Val, ref Grad);

    /// <summary>The tolerance of the zero of the implicit function.</summary>
    public static double Tolerance(in IntSurf_Quadric Q)
    {
        switch (Q.TypeQuadric())
        {
            case GeomAbs_Sphere:
                return 2.0e-6 * Q.Sphere().Radius();
            case GeomAbs_Cylinder:
                return 2.0e-6 * Q.Cylinder().Radius();
            default:
                break;
        }
        return (1e-6);
    }
}
