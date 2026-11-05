// Port of: src/Draw/TKTopTest/GeomliteTest/GeomliteTest_API2dCommands.cxx (2dproj, 2dextrema, 2dintersect, 2dintanalytical, intconcon)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// GeomliteTest_API2dCommands.cxx's analysis and intersection commands on 2D curves (unit 7W): <c>2dproj</c>
/// (Geom2dAPI_ProjectPointOnCurve), <c>2dextrema</c> (Geom2dAPI_ExtremaCurveCurve), <c>2dintersect</c>
/// (Geom2dAPI_InterCurveCurve), <c>2dintanalytical</c> and <c>intconcon</c> (IntAna2d_AnaIntersection).
/// </summary>
/// <remarks>
/// The markers and segment curves OCCT draws in the 2D view (Draw_Marker2D, DrawTrSurf_Curve2d) are left out. The
/// file's approximations (<c>2dapprox</c>, <c>2dinterpole</c>) and <c>2ddeviation</c> aren't ported.
/// </remarks>
internal static class GeomliteTestAPI2dCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "2dproj", Proj);
        DrawCxx.Add(theInterpreter, "2dextrema", Extrema);
        DrawCxx.Add(theInterpreter, "2dintersect", Intersect);
        DrawCxx.Add(theInterpreter, "2dintanalytical", IntersectAna);
        DrawCxx.Add(theInterpreter, "intconcon", IntConCon);
    }

    // proj curve x y
    private static int Proj(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n != 4)
        {
            di.Append("Syntax error: wrong number of arguments");
            return 1;
        }

        Curve2D? GC = DrawTrSurf.GetCurve2d(theDI, a[1]);
        if (GC is null)
        {
            di.Append("Syntax error: '").Append(a[1]).Append("' is NULL");
            return 1;
        }

        var P = new Point2D(theDI.Atof(a[2]), theDI.Atof(a[3]));
        var proj = new ProjectPointOnCurve2D(P, GC, GC.FirstParameter, GC.LastParameter);
        for (int i = 1; i <= proj.NbPoints(); i++)
        {
            Point2D aP1 = proj.Point(i);
            double aDist = P.DistanceTo(aP1);
            string aName = $"ext_{i}";
            if (aDist > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                var L = new LineCurve2D(P, new Direction2D(aP1.XY() - P.XY()));
                var CT = new TrimmedCurve2D(L, 0.0, aDist);
                DrawTrSurf.Set(theDI, aName, CT);
            }
            else
            {
                DrawTrSurf.Set(theDI, aName, aP1);
            }

            di.Append(aName).Append(" ");
        }

        return 0;
    }

    // extrema curve curve
    private static int Extrema(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n != 3)
        {
            di.Append("Syntax error: wrong number of arguments");
            return 1;
        }

        Curve2D? GC1 = DrawTrSurf.GetCurve2d(theDI, a[1]);
        Curve2D? GC2 = DrawTrSurf.GetCurve2d(theDI, a[2]);
        if (GC1 is null)
        {
            di.Append("Syntax error: '").Append(a[1]).Append("' is NULL");
            return 1;
        }
        if (GC2 is null)
        {
            di.Append("Syntax error: '").Append(a[2]).Append("' is NULL");
            return 1;
        }

        double U1f = GC1.FirstParameter;
        double U1l = GC1.LastParameter;
        double U2f = GC2.FirstParameter;
        double U2l = GC2.LastParameter;

        var Ex = new ExtremaCurveCurve2D(GC1, GC2, U1f, U1l, U2f, U2l);
        bool isInfinitySolutions = Ex.Extrema().IsParallel;
        int aNExtr = Ex.NbExtrema();

        if (aNExtr == 0 || isInfinitySolutions)
        {
            // Infinity solutions flag may be set with 0 number of
            // solutions in analytic extrema Curve/Curve.
            if (isInfinitySolutions)
            {
                di.Append("Infinite number of extremas, distance = ").Append(Ex.LowerDistance()).Append("\n");
            }
            else
            {
                di.Append("No solutions!\n");
            }
        }

        for (int i = 1; i <= aNExtr; i++)
        {
            Ex.Points(i, out Point2D P1, out Point2D P2);
            di.Append("dist ").Append(i).Append(": ").Append(Ex.Distance(i)).Append("  ");
            string aName = $"ext_{i}";
            if (Ex.Distance(i) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                DrawTrSurf.Set(theDI, aName, P1);
            }
            else
            {
                var L = new LineCurve2D(P1, new Direction2D(new Vector2D(P1, P2)));
                var CT = new TrimmedCurve2D(L, 0.0, P1.DistanceTo(P2));
                DrawTrSurf.Set(theDI, aName, CT);
            }
            di.Append(aName).Append("\n");
        }

        return 0;
    }

    // 2dintersect curve1 [curve2] [-tol tol] [-state]
    private static int Intersect(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        Curve2D? C1 = null, C2 = null;
        double Tol = 0.001;
        bool bPrintState = false;

        // Retrieve other parameters if any
        for (int i = 1; i < n; ++i)
        {
            if (a[i] == "-tol")
            {
                // OCCT: a[++i] past the end, Draw::Atof reads the null pointer; the port reads an empty word (0)
                ++i;
                Tol = theDI.Atof(i < n ? a[i] : "");
            }
            else if (a[i] == "-state")
            {
                bPrintState = true;
            }
            else if (C1 is null)
            {
                C1 = DrawTrSurf.GetCurve2d(theDI, a[i]);
                if (C1 is null)
                {
                    di.Append("Syntax error: curve '").Append(a[i]).Append("' is null");
                    return 1;
                }
            }
            else if (C2 is null)
            {
                C2 = DrawTrSurf.GetCurve2d(theDI, a[i]);
                if (C2 is null)
                {
                    di.Append("Syntax error: curve '").Append(a[i]).Append("' is null");
                    return 1;
                }
            }
            else
            {
                di.Append("Syntax error at '").Append(a[i]).Append("'");
                return 1;
            }
        }
        if (C1 is null)
        {
            di.Append("Syntax error: wrong number of arguments");
            return 1;
        }

        var Intersector = new CurveCurveIntersection2D();
        if (C2 is not null)
        {
            // Curves intersection
            Intersector.Init(C1, C2, Tol);
        }
        else
        {
            // Self-intersection of the curve
            Intersector.Init(C1, Tol);
        }

        GInter anIntTool = Intersector.Intersector();
        if (!anIntTool.IsDone)
        {
            di.Append("Intersection failed\n");
            return 0;
        }

        if (anIntTool.IsEmpty)
        {
            return 0;
        }

        int aNbPoints = Intersector.NbPoints();
        for (int i = 1; i <= aNbPoints; i++)
        {
            // API simplified result
            Point2D P = Intersector.Point(i);
            di.Append("Intersection point ").Append(i).Append(" : ").Append(P.X).Append(" ").Append(P.Y).Append("\n");
            // Intersection extended results from intersection tool
            IntersectionPoint2D aPInt = anIntTool.Point(i);
            di.Append("parameter on the fist: ").Append(aPInt.ParamOnFirst());
            di.Append(" parameter on the second: ").Append(aPInt.ParamOnSecond()).Append("\n");
            if (bPrintState)
            {
                di.Append("Intersection type: ").Append(aPInt.TransitionOfFirst().IsTangent ? "TOUCH" : "INTERSECTION").Append("\n");
            }
        }

        int aNbSegments = Intersector.NbSegments();
        for (int i = 1; i <= aNbSegments; i++)
        {
            di.Append("Segment #").Append(i).Append(" found.\n");
            // the segments' curves are only drawn
            Intersector.Segment(i, out _, out _);
        }

        return 0;
    }

    // 2dintanalytical circle1 circle2
    private static int IntersectAna(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n != 3)
        {
            di.Append("Syntax error: wrong number of arguments");
            return 1;
        }

        Curve2D? C1 = DrawTrSurf.GetCurve2d(theDI, a[1]);
        Curve2D? C2 = DrawTrSurf.GetCurve2d(theDI, a[2]);
        var aCir1 = C1 as CircleCurve2D;
        var aCir2 = C2 as CircleCurve2D;
        if (aCir1 is null || aCir2 is null)
        {
            di.Append("Syntax error: '").Append(a[aCir1 is null ? 1 : 2]).Append("' is not a circle");
            return 1;
        }

        var Intersector = new AnalyticIntersection(aCir1.Circle, aCir2.Circle);
        for (int i = 1; i <= Intersector.PointCount; i++)
        {
            Point2D P = Intersector.Point(i).Value();
            di.Append("Intersection point ").Append(i).Append(" : ").Append(P.X).Append(" ").Append(P.Y).Append("\n");
            di.Append("parameter on the fist: ").Append(Intersector.Point(i).ParamOnFirst());
            di.Append(" parameter on the second: ").Append(Intersector.Point(i).ParamOnSecond()).Append("\n");
        }
        return 0;
    }

    // intconcon curve1 curve2
    private static int IntConCon(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n != 3)
        {
            di.Append("Syntax error: wrong number of arguments");
            return 1;
        }

        Curve2D? C1 = DrawTrSurf.GetCurve2d(theDI, a[1]);
        if (C1 is null)
        {
            di.Append("Syntax error: '").Append(a[1]).Append("' is Null");
            return 1;
        }

        Curve2D? C2 = DrawTrSurf.GetCurve2d(theDI, a[2]);
        if (C2 is null)
        {
            di.Append("Syntax error: '").Append(a[2]).Append("' is Null");
            return 1;
        }

        var AC1 = new CurveAdaptor2D(C1);
        var AC2 = new CurveAdaptor2D(C2);
        CurveKind T1 = AC1.GetType(), T2 = AC2.GetType();
        Conic pCon;
        switch (T2)
        {
            case CurveKind.Line:
                pCon = new Conic(AC2.Line());
                break;
            case CurveKind.Circle:
                pCon = new Conic(AC2.Circle());
                break;
            case CurveKind.Ellipse:
                pCon = new Conic(AC2.Ellipse());
                break;
            case CurveKind.Hyperbola:
                pCon = new Conic(AC2.Hyperbola());
                break;
            case CurveKind.Parabola:
                pCon = new Conic(AC2.Parabola());
                break;
            default:
                di.Append("Syntax error: '").Append(a[2]).Append("' is not conic");
                return 1;
        }

        var Intersector = new AnalyticIntersection();
        switch (T1)
        {
            case CurveKind.Line:
                Intersector.Perform(AC1.Line(), pCon);
                break;
            case CurveKind.Circle:
                Intersector.Perform(AC1.Circle(), pCon);
                break;
            case CurveKind.Ellipse:
                Intersector.Perform(AC1.Ellipse(), pCon);
                break;
            case CurveKind.Hyperbola:
                Intersector.Perform(AC1.Hyperbola(), pCon);
                break;
            case CurveKind.Parabola:
                Intersector.Perform(AC1.Parabola(), pCon);
                break;
            default:
                di.Append("Syntax error: '").Append(a[1]).Append("' is not conic");
                return 1;
        }

        for (int i = 1; i <= Intersector.PointCount; i++)
        {
            Point2D P = Intersector.Point(i).Value();
            di.Append("Intersection point ").Append(i).Append(" : ").Append(P.X).Append(" ").Append(P.Y).Append("\n");
            di.Append("parameter on the fist: ").Append(Intersector.Point(i).ParamOnFirst());
            if (!Intersector.Point(i).SecondIsImplicit())
            {
                di.Append(" parameter on the second: ").Append(Intersector.Point(i).ParamOnSecond()).Append("\n");
            }
            else
            {
                di.Append("\n");
            }
        }
        return 0;
    }
}
