// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_BlendTool.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_BlendTool.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_BlendTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>The walkers' tools on the arcs of restriction: projection, intersection with a segment, vertices.</summary>
internal static class BlendTool
{
    /// <summary>
    /// Projects the point <paramref name="p"/> on the arc <paramref name="c"/>: <paramref name="paramproj"/> is the
    /// parameter of the nearest point (the ends included) and <paramref name="dist"/> its distance. Always true.
    /// </summary>
    public static bool Project(in Point2D p, SurfaceAdaptorBase? s, Curve2DAdaptorBase c, out double paramproj,
                               out double dist)
    {
        paramproj = HCurve2dTool.FirstParameter(c);
        Point2D p2d;
        HCurve2dTool.D0(c, paramproj, out p2d);
        dist = p2d.DistanceTo(p);

        double t = HCurve2dTool.LastParameter(c);
        HCurve2dTool.D0(c, t, out p2d);
        if (p2d.DistanceTo(p) < dist)
        {
            paramproj = t;
            dist = p2d.DistanceTo(p);
        }

        const double epsX = 1.0e-8;
        const int nbu = 20;
        const double tol = 1.0e-5;
        EpcOfExtPC2d extrema = new(p, c, nbu, epsX, tol);
        if (!extrema.IsDone)
        {
            return true;
        }

        int nbext = extrema.ExtCount;
        double dist2 = dist * dist;
        for (int i = 1; i <= nbext; i++)
        {
            if (extrema.SquareDistance(i) < dist2)
            {
                dist2 = extrema.SquareDistance(i);
                paramproj = extrema.Point(i).Parameter();
            }
        }
        dist = Math.Sqrt(dist2);

        return true;
    }

    /// <summary>
    /// Intersects the segment [<paramref name="p1"/>, <paramref name="p2"/>] (extended by 1% at each end) with the arc
    /// <paramref name="c"/>: the first intersection's parameter on the arc and its distance to <paramref name="p1"/>.
    /// </summary>
    /// <remarks><paramref name="param"/> and <paramref name="dist"/> are left unchanged on failure, as OCCT's.</remarks>
    public static bool Inters(in Point2D p1, in Point2D p2, SurfaceAdaptorBase? s, Curve2DAdaptorBase c,
                              ref double param, ref double dist)
    {
        const double tol = 1.0e-8;
        Vector2D v = new(p1, p2);
        double mag = v.Magnitude;
        if (mag < tol)
        {
            return false;
        }

        Direction2D d = new(v);
        LineCurve2D bid = new(p1, d);
        CurveAdaptor2D seg = new(bid, -0.01 * mag, 1.01 * mag);

        GInter inter = new(seg, c, tol, tol);
        if (!inter.IsDone)
        {
            return false;
        }

        int nbint = inter.PointCount;
        if (nbint == 0)
        {
            return false;
        }

        IntersectionPoint2D ip = inter.Point(1);
        param = ip.ParamOnSecond();
        dist = p1.DistanceTo(ip.Value());
        return true;
    }

    /// <summary>The parameter of the vertex <paramref name="v"/> on the arc <paramref name="c"/>.</summary>
    public static double Parameter(HVertex v, Curve2DAdaptorBase c)
    {
        return v.Parameter(c);
    }

    /// <summary>The parametric tolerance on the arc <paramref name="a"/> within which a point meets the vertex.</summary>
    public static double Tolerance(HVertex v, Curve2DAdaptorBase a)
    {
        return v.Resolution(a);
    }

    public static bool SingularOnUMin(SurfaceAdaptorBase s) => false;

    public static bool SingularOnUMax(SurfaceAdaptorBase s) => false;

    public static bool SingularOnVMin(SurfaceAdaptorBase s) => false;

    public static bool SingularOnVMax(SurfaceAdaptorBase s) => false;

    public static int NbSamplesU(SurfaceAdaptorBase s, double u1, double u2) => 10;

    public static int NbSamplesV(SurfaceAdaptorBase s, double v1, double v2) => 10;

    /// <summary>The parametric limits of the arc <paramref name="a"/>.</summary>
    public static void Bounds(Curve2DAdaptorBase a, out double ufirst, out double ulast)
    {
        ufirst = HCurve2dTool.FirstParameter(a);
        ulast = HCurve2dTool.LastParameter(a);
    }

    public static Curve2DAdaptorBase CurveOnSurf(Curve2DAdaptorBase c, SurfaceAdaptorBase s)
    {
        return c;
    }
}
