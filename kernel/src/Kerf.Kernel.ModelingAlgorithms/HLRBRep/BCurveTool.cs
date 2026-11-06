// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BCurveTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BCurveTool.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BCurveTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.HiddenLines;

/// <summary>The static tool over the 3D curve of an edge (BRepAdaptor_Curve) that HLRBRep_Curve reads through.</summary>
internal static class BCurveTool
{
    public static double FirstParameter(EdgeCurveAdaptor c) => (c.FirstParameter());

    public static double LastParameter(EdgeCurveAdaptor c) => (c.LastParameter());

    public static Continuity Continuity(EdgeCurveAdaptor c) => (c.Continuity());

    public static int NbIntervals(EdgeCurveAdaptor c, Continuity sh) => (c.NbIntervals(sh));

    public static void Intervals(EdgeCurveAdaptor c, NCollection_Array1<double> tab, Continuity sh)
    {
        c.Intervals(tab, sh);
    }

    public static bool IsClosed(EdgeCurveAdaptor c) => (c.IsClosed);

    public static bool IsPeriodic(EdgeCurveAdaptor c) => (c.IsPeriodic);

    public static double Period(EdgeCurveAdaptor c) => (c.Period());

    public static Point3D Value(EdgeCurveAdaptor c, double u) => (c.Value(u));

    public static void D0(EdgeCurveAdaptor c, double u, out Point3D p)
    {
        c.D0(u, out p);
    }

    public static void D1(EdgeCurveAdaptor c, double u, out Point3D p, out Vector3D t)
    {
        c.D1(u, out p, out t);
    }

    public static void D2(EdgeCurveAdaptor c, double u, out Point3D p, out Vector3D t, out Vector3D n)
    {
        c.D2(u, out p, out t, out n);
    }

    public static void D3(EdgeCurveAdaptor c, double u, out Point3D p, out Vector3D v1, out Vector3D v2, out Vector3D v3)
    {
        c.D3(u, out p, out v1, out v2, out v3);
    }

    public static Vector3D DN(EdgeCurveAdaptor c, double u, int n) => (c.DN(u, n));

    public static double Resolution(EdgeCurveAdaptor c, double r3d) => (c.Resolution(r3d));

    public static CurveKind GetType(EdgeCurveAdaptor c) => (c.GetType());

    public static Line3D Line(EdgeCurveAdaptor c) => (c.Line());

    public static Circle3D Circle(EdgeCurveAdaptor c) => (c.Circle());

    public static Ellipse3D Ellipse(EdgeCurveAdaptor c) => (c.Ellipse());

    public static Parabola3D Parabola(EdgeCurveAdaptor c) => (c.Parabola());

    public static Hyperbola3D Hyperbola(EdgeCurveAdaptor c) => (c.Hyperbola());

    public static int Degree(EdgeCurveAdaptor c) => (c.Degree());

    public static bool IsRational(EdgeCurveAdaptor c) => (c.IsRational);

    public static int NbPoles(EdgeCurveAdaptor c) => (c.PoleCount);

    public static int NbKnots(EdgeCurveAdaptor c) => (c.KnotCount);

    /// <summary>The number of samples of the curve on [U0, U1], between 2 and 50.</summary>
    public static int NbSamples(EdgeCurveAdaptor c, double u0, double u1)
    {
        CurveKind typC = c.GetType();
        const double nbsOther = 10.0; // OCCT: a function-scope static never written
        double nbs = nbsOther;

        if (typC == global::Kerf.Geometry.CurveKind.Line)
        {
            nbs = 2;
        }
        else if (typC == BezierCurve)
        {
            nbs = 3 + c.PoleCount;
        }
        else if (typC == BSplineCurve)
        {
            nbs = c.KnotCount;
            nbs *= c.Degree();
            nbs *= c.LastParameter() - c.FirstParameter();
            nbs /= u1 - u0;
            if (nbs < 2.0)
            {
                nbs = 2;
            }
        }
        if (nbs > 50)
        {
            nbs = 50;
        }
        return ((int)nbs);
    }

    /// <summary>Copies the poles of a Bezier or B-spline curve into T (by T's indices).</summary>
    public static void Poles(EdgeCurveAdaptor c, NCollection_Array1<Point3D> t)
    {
        if (c.GetType() == BezierCurve)
        {
            BezierCurve3D bez = c.Bezier();
            NCollection_Array1<Point3D> srcPoles = bez.InternalPoles();
            for (int i = t.Lower(); i <= t.Upper(); i++)
            {
                t[i] = srcPoles[i];
            }
        }
        else if (c.GetType() == BSplineCurve)
        {
            BSplineCurve3D bSpl = c.BSpline();
            NCollection_Array1<Point3D> srcPoles = bSpl.InternalPoles();
            for (int i = t.Lower(); i <= t.Upper(); i++)
            {
                t[i] = srcPoles[i];
            }
        }
    }

    /// <summary>Copies the poles and weights of a Bezier or B-spline curve into T and W (by their indices).</summary>
    public static void PolesAndWeights(EdgeCurveAdaptor c, NCollection_Array1<Point3D> t, NCollection_Array1<double> w)
    {
        if (c.GetType() == BezierCurve)
        {
            BezierCurve3D hb = c.Bezier();
            NCollection_Array1<Point3D> srcPoles = hb.InternalPoles();
            NCollection_Array1<double> srcWeights = hb.InternalWeightsArray();
            for (int i = t.Lower(); i <= t.Upper(); i++)
            {
                t[i] = srcPoles[i];
            }
            for (int i = w.Lower(); i <= w.Upper(); i++)
            {
                w[i] = srcWeights[i];
            }
        }
        else if (c.GetType() == BSplineCurve)
        {
            BSplineCurve3D hb = c.BSpline();
            NCollection_Array1<Point3D> srcPoles = hb.InternalPoles();
            NCollection_Array1<double> srcWeights = hb.InternalWeightsArray();
            for (int i = t.Lower(); i <= t.Upper(); i++)
            {
                t[i] = srcPoles[i];
            }
            for (int i = w.Lower(); i <= w.Upper(); i++)
            {
                w[i] = srcWeights[i];
            }
        }
    }

    public static BezierCurve3D Bezier(EdgeCurveAdaptor c) => (c.Bezier());

    public static BSplineCurve3D BSpline(EdgeCurveAdaptor c) => (c.BSpline());
}
