// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_BlendTool.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_BlendTool.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_BlendTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Occt;

/// <summary>The walkers' tools on the arcs of restriction: projection, intersection with a segment, vertices.</summary>
public static class BRepBlend_BlendTool
{
    /// <summary>
    /// Projects the point <paramref name="P"/> on the arc <paramref name="C"/>: <paramref name="Paramproj"/> is the
    /// parameter of the nearest point (the ends included) and <paramref name="Dist"/> its distance. Always true.
    /// </summary>
    public static bool Project(in Point2D P, SurfaceAdaptorBase? S, Curve2DAdaptorBase C, out double Paramproj,
                               out double Dist)
    {
        Paramproj = BRepBlend_HCurve2dTool.FirstParameter(C);
        Point2D P2d;
        BRepBlend_HCurve2dTool.D0(C, Paramproj, out P2d);
        Dist = P2d.Distance(P);

        double t = BRepBlend_HCurve2dTool.LastParameter(C);
        BRepBlend_HCurve2dTool.D0(C, t, out P2d);
        if (P2d.Distance(P) < Dist)
        {
            Paramproj = t;
            Dist = P2d.Distance(P);
        }

        const double epsX = 1.0e-8;
        const int Nbu = 20;
        const double Tol = 1.0e-5;
        EpcOfExtPC2d extrema = new(P, C, Nbu, epsX, Tol);
        if (!extrema.IsDone)
        {
            return true;
        }

        int Nbext = extrema.ExtCount;
        double aDist2 = Dist * Dist;
        for (int i = 1; i <= Nbext; i++)
        {
            if (extrema.SquareDistance(i) < aDist2)
            {
                aDist2 = extrema.SquareDistance(i);
                Paramproj = extrema.Point(i).Parameter();
            }
        }
        Dist = Math.Sqrt(aDist2);

        return true;
    }

    /// <summary>
    /// Intersects the segment [<paramref name="P1"/>, <paramref name="P2"/>] (extended by 1% at each end) with the arc
    /// <paramref name="C"/>: the first intersection's parameter on the arc and its distance to <paramref name="P1"/>.
    /// </summary>
    /// <remarks><paramref name="Param"/> and <paramref name="Dist"/> are left unchanged on failure, as OCCT's.</remarks>
    public static bool Inters(in Point2D P1, in Point2D P2, SurfaceAdaptorBase? S, Curve2DAdaptorBase C,
                              ref double Param, ref double Dist)
    {
        const double Tol = 1.0e-8;
        Vector2D v = new(P1, P2);
        double mag = v.Magnitude();
        if (mag < Tol)
        {
            return false;
        }

        Direction2D d = new(v);
        LineCurve2D bid = new(P1, d);
        CurveAdaptor2D seg = new(bid, -0.01 * mag, 1.01 * mag);

        Geom2dInt_GInter inter = new(seg, C, Tol, Tol);
        if (!inter.IsDone())
        {
            return false;
        }

        int Nbint = inter.NbPoints();
        if (Nbint == 0)
        {
            return false;
        }

        IntRes2d_IntersectionPoint ip = inter.Point(1);
        Param = ip.ParamOnSecond();
        Dist = P1.Distance(ip.Value());
        return true;
    }

    /// <summary>The parameter of the vertex <paramref name="V"/> on the arc <paramref name="C"/>.</summary>
    public static double Parameter(HVertex V, Curve2DAdaptorBase C)
    {
        return V.Parameter(C);
    }

    /// <summary>The parametric tolerance on the arc <paramref name="A"/> within which a point meets the vertex.</summary>
    public static double Tolerance(HVertex V, Curve2DAdaptorBase A)
    {
        return V.Resolution(A);
    }

    public static bool SingularOnUMin(SurfaceAdaptorBase S) => false;

    public static bool SingularOnUMax(SurfaceAdaptorBase S) => false;

    public static bool SingularOnVMin(SurfaceAdaptorBase S) => false;

    public static bool SingularOnVMax(SurfaceAdaptorBase S) => false;

    public static int NbSamplesU(SurfaceAdaptorBase S, double u1, double u2) => 10;

    public static int NbSamplesV(SurfaceAdaptorBase S, double v1, double v2) => 10;

    /// <summary>The parametric limits of the arc <paramref name="A"/>.</summary>
    public static void Bounds(Curve2DAdaptorBase A, out double Ufirst, out double Ulast)
    {
        Ufirst = BRepBlend_HCurve2dTool.FirstParameter(A);
        Ulast = BRepBlend_HCurve2dTool.LastParameter(A);
    }

    public static Curve2DAdaptorBase CurveOnSurf(Curve2DAdaptorBase C, SurfaceAdaptorBase S)
    {
        return C;
    }
}
