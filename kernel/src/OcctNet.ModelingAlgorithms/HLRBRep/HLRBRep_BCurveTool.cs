// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BCurveTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BCurveTool.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_BCurveTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>The static tool over the 3D curve of an edge (BRepAdaptor_Curve) that HLRBRep_Curve reads through.</summary>
public static class HLRBRep_BCurveTool
{
    public static double FirstParameter(BRepAdaptor_Curve C) => (C.FirstParameter());

    public static double LastParameter(BRepAdaptor_Curve C) => (C.LastParameter());

    public static Continuity Continuity(BRepAdaptor_Curve C) => (C.Continuity());

    public static int NbIntervals(BRepAdaptor_Curve C, Continuity Sh) => (C.NbIntervals(Sh));

    public static void Intervals(BRepAdaptor_Curve C, NCollection_Array1<double> Tab, Continuity Sh)
    {
        C.Intervals(Tab, Sh);
    }

    public static bool IsClosed(BRepAdaptor_Curve C) => (C.IsClosed());

    public static bool IsPeriodic(BRepAdaptor_Curve C) => (C.IsPeriodic());

    public static double Period(BRepAdaptor_Curve C) => (C.Period());

    public static Point3D Value(BRepAdaptor_Curve C, double U) => (C.Value(U));

    public static void D0(BRepAdaptor_Curve C, double U, out Point3D P)
    {
        C.D0(U, out P);
    }

    public static void D1(BRepAdaptor_Curve C, double U, out Point3D P, out Vector3D T)
    {
        C.D1(U, out P, out T);
    }

    public static void D2(BRepAdaptor_Curve C, double U, out Point3D P, out Vector3D T, out Vector3D N)
    {
        C.D2(U, out P, out T, out N);
    }

    public static void D3(BRepAdaptor_Curve C, double U, out Point3D P, out Vector3D V1, out Vector3D V2, out Vector3D V3)
    {
        C.D3(U, out P, out V1, out V2, out V3);
    }

    public static Vector3D DN(BRepAdaptor_Curve C, double U, int N) => (C.DN(U, N));

    public static double Resolution(BRepAdaptor_Curve C, double R3d) => (C.Resolution(R3d));

    public static CurveKind GetType(BRepAdaptor_Curve C) => (C.GetType());

    public static Line3D Line(BRepAdaptor_Curve C) => (C.Line());

    public static Circle3D Circle(BRepAdaptor_Curve C) => (C.Circle());

    public static Ellipse3D Ellipse(BRepAdaptor_Curve C) => (C.Ellipse());

    public static Parabola3D Parabola(BRepAdaptor_Curve C) => (C.Parabola());

    public static Hyperbola3D Hyperbola(BRepAdaptor_Curve C) => (C.Hyperbola());

    public static int Degree(BRepAdaptor_Curve C) => (C.Degree());

    public static bool IsRational(BRepAdaptor_Curve C) => (C.IsRational());

    public static int NbPoles(BRepAdaptor_Curve C) => (C.NbPoles());

    public static int NbKnots(BRepAdaptor_Curve C) => (C.NbKnots());

    /// <summary>The number of samples of the curve on [U0, U1], between 2 and 50.</summary>
    public static int NbSamples(BRepAdaptor_Curve C, double U0, double U1)
    {
        CurveKind typC = C.GetType();
        const double nbsOther = 10.0; // OCCT: a function-scope static never written
        double nbs = nbsOther;

        if (typC == global::Kerf.Geometry.CurveKind.Line)
        {
            nbs = 2;
        }
        else if (typC == BezierCurve)
        {
            nbs = 3 + C.NbPoles();
        }
        else if (typC == BSplineCurve)
        {
            nbs = C.NbKnots();
            nbs *= C.Degree();
            nbs *= C.LastParameter() - C.FirstParameter();
            nbs /= U1 - U0;
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
    public static void Poles(BRepAdaptor_Curve C, NCollection_Array1<Point3D> T)
    {
        if (C.GetType() == BezierCurve)
        {
            Geom_BezierCurve aBez = C.Bezier();
            NCollection_Array1<Point3D> aSrcPoles = aBez.InternalPoles();
            for (int i = T.Lower(); i <= T.Upper(); i++)
            {
                T[i] = aSrcPoles[i];
            }
        }
        else if (C.GetType() == BSplineCurve)
        {
            Geom_BSplineCurve aBSpl = C.BSpline();
            NCollection_Array1<Point3D> aSrcPoles = aBSpl.InternalPoles();
            for (int i = T.Lower(); i <= T.Upper(); i++)
            {
                T[i] = aSrcPoles[i];
            }
        }
    }

    /// <summary>Copies the poles and weights of a Bezier or B-spline curve into T and W (by their indices).</summary>
    public static void PolesAndWeights(BRepAdaptor_Curve C, NCollection_Array1<Point3D> T, NCollection_Array1<double> W)
    {
        if (C.GetType() == BezierCurve)
        {
            Geom_BezierCurve HB = C.Bezier();
            NCollection_Array1<Point3D> aSrcPoles = HB.InternalPoles();
            NCollection_Array1<double> aSrcWeights = HB.InternalWeightsArray();
            for (int i = T.Lower(); i <= T.Upper(); i++)
            {
                T[i] = aSrcPoles[i];
            }
            for (int i = W.Lower(); i <= W.Upper(); i++)
            {
                W[i] = aSrcWeights[i];
            }
        }
        else if (C.GetType() == BSplineCurve)
        {
            Geom_BSplineCurve HB = C.BSpline();
            NCollection_Array1<Point3D> aSrcPoles = HB.InternalPoles();
            NCollection_Array1<double> aSrcWeights = HB.InternalWeightsArray();
            for (int i = T.Lower(); i <= T.Upper(); i++)
            {
                T[i] = aSrcPoles[i];
            }
            for (int i = W.Lower(); i <= W.Upper(); i++)
            {
                W[i] = aSrcWeights[i];
            }
        }
    }

    public static Geom_BezierCurve Bezier(BRepAdaptor_Curve C) => (C.Bezier());

    public static Geom_BSplineCurve BSpline(BRepAdaptor_Curve C) => (C.BSpline());
}
