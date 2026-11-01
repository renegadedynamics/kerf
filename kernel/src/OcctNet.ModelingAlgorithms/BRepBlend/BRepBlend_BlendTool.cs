// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_BlendTool.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_BlendTool.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_BlendTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The walkers' tools on the arcs of restriction: projection, intersection with a segment, vertices.</summary>
public static class BRepBlend_BlendTool
{
    /// <summary>
    /// Projects the point <paramref name="P"/> on the arc <paramref name="C"/>: <paramref name="Paramproj"/> is the
    /// parameter of the nearest point (the ends included) and <paramref name="Dist"/> its distance. Always true.
    /// </summary>
    public static bool Project(in gp_Pnt2d P, Adaptor3d_Surface? S, Adaptor2d_Curve2d C, out double Paramproj,
                               out double Dist)
    {
        Paramproj = BRepBlend_HCurve2dTool.FirstParameter(C);
        gp_Pnt2d P2d;
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
        Extrema_EPCOfExtPC2d extrema = new(P, C, Nbu, epsX, Tol);
        if (!extrema.IsDone())
        {
            return true;
        }

        int Nbext = extrema.NbExt();
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
    public static bool Inters(in gp_Pnt2d P1, in gp_Pnt2d P2, Adaptor3d_Surface? S, Adaptor2d_Curve2d C,
                              ref double Param, ref double Dist)
    {
        const double Tol = 1.0e-8;
        gp_Vec2d v = new(P1, P2);
        double mag = v.Magnitude();
        if (mag < Tol)
        {
            return false;
        }

        gp_Dir2d d = new(v);
        Geom2d_Line bid = new(P1, d);
        Geom2dAdaptor_Curve seg = new(bid, -0.01 * mag, 1.01 * mag);

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
    public static double Parameter(Adaptor3d_HVertex V, Adaptor2d_Curve2d C)
    {
        return V.Parameter(C);
    }

    /// <summary>The parametric tolerance on the arc <paramref name="A"/> within which a point meets the vertex.</summary>
    public static double Tolerance(Adaptor3d_HVertex V, Adaptor2d_Curve2d A)
    {
        return V.Resolution(A);
    }

    public static bool SingularOnUMin(Adaptor3d_Surface S) => false;

    public static bool SingularOnUMax(Adaptor3d_Surface S) => false;

    public static bool SingularOnVMin(Adaptor3d_Surface S) => false;

    public static bool SingularOnVMax(Adaptor3d_Surface S) => false;

    public static int NbSamplesU(Adaptor3d_Surface S, double u1, double u2) => 10;

    public static int NbSamplesV(Adaptor3d_Surface S, double v1, double v2) => 10;

    /// <summary>The parametric limits of the arc <paramref name="A"/>.</summary>
    public static void Bounds(Adaptor2d_Curve2d A, out double Ufirst, out double Ulast)
    {
        Ufirst = BRepBlend_HCurve2dTool.FirstParameter(A);
        Ulast = BRepBlend_HCurve2dTool.LastParameter(A);
    }

    public static Adaptor2d_Curve2d CurveOnSurf(Adaptor2d_Curve2d C, Adaptor3d_Surface S)
    {
        return C;
    }
}
