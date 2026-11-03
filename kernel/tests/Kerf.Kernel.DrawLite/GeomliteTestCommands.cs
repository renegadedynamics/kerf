// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// The curve and surface commands of GeomliteTest_CurveCommands.cxx and GeomliteTest_SurfaceCommands.cxx that
/// construct, trim, offset and evaluate geometry: the analytic curves and surfaces, the Bezier and B-spline ones
/// from poles, the swept surfaces, offsets and trims, isos, values, bounds and parameters; and (unit 7W)
/// <c>incdeg</c>, <c>insertpole</c>, <c>cmovepoint</c>, <c>convert</c>, <c>segsur</c> and GeomliteTest_ModificationCommands.cxx's
/// <c>setweight</c>.
/// </summary>
/// <remarks>
/// OCCT's <c>reverse</c>, <c>ureverse</c>, <c>vreverse</c>, <c>segment</c>, <c>setorigin</c>, <c>incdeg</c>,
/// <c>insertpole</c>, <c>cmovepoint</c>, <c>segsur</c> and <c>setweight</c> change the bound geometry in place. Geometry is frozen when
/// bound (DrawTrSurf.Set), so they bind the changed copy that the port's public API returns (<c>Reversed</c>,
/// <c>UReversed</c>, <c>Segmented</c>, <c>WithOrigin</c>, <c>Increased</c>, <c>DegreeIncreased</c>,
/// <c>PoleInsertedAfter</c>, <c>PointMoved</c>, <c>WithWeight</c>) under the same name:
/// a shape made from the geometry before keeps the old geometry, where in OCCT it would see the change.
/// Draw passes a null pointer for an argument past the end of the words (Tcl's argv[argc]), which Draw::Atof would
/// dereference; the port reads it as an empty word (0).
/// </remarks>
internal static class GeomliteTestCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        foreach (string aName in new[] { "line", "circle", "ellipse", "parabola", "hyperbola" })
        {
            DrawCxx.Add(theInterpreter, aName, AnaCurve);
        }

        foreach (string aName in new[] { "beziercurve", "bsplinecurve", "pbsplinecurve" })
        {
            DrawCxx.Add(theInterpreter, aName, PoleCurve);
        }

        foreach (string aName in new[] { "2dbeziercurve", "2dbsplinecurve", "2dpbsplinecurve" })
        {
            DrawCxx.Add(theInterpreter, aName, PoleCurve2d);
        }

        DrawCxx.Add(theInterpreter, "reverse", Reverse);
        DrawCxx.Add(theInterpreter, "segment", Segment);
        DrawCxx.Add(theInterpreter, "setorigin", SetOrigin);
        DrawCxx.Add(theInterpreter, "cvalue", Value);
        DrawCxx.Add(theInterpreter, "2dcvalue", Value2d);
        DrawCxx.Add(theInterpreter, "coord", Coord);

        foreach (string aName in new[] { "plane", "cone", "cylinder", "sphere", "torus" })
        {
            DrawCxx.Add(theInterpreter, aName, AnaSurface);
        }

        foreach (string aName in new[] { "beziersurf", "bsplinesurf", "upbsplinesurf", "vpbsplinesurf", "uvpbsplinesurf" })
        {
            DrawCxx.Add(theInterpreter, aName, PoleSurface);
        }

        DrawCxx.Add(theInterpreter, "extsurf", AlgoSurface);
        DrawCxx.Add(theInterpreter, "revsurf", AlgoSurface);
        DrawCxx.Add(theInterpreter, "offset", Offseting);
        DrawCxx.Add(theInterpreter, "trim", Trimming);
        DrawCxx.Add(theInterpreter, "trimu", Trimming);
        DrawCxx.Add(theInterpreter, "trimv", Trimming);
        DrawCxx.Add(theInterpreter, "ureverse", SReverse);
        DrawCxx.Add(theInterpreter, "vreverse", SReverse);
        DrawCxx.Add(theInterpreter, "uiso", Iso);
        DrawCxx.Add(theInterpreter, "viso", Iso);
        DrawCxx.Add(theInterpreter, "svalue", SValue);
        DrawCxx.Add(theInterpreter, "sderivative", Derivative);
        DrawCxx.Add(theInterpreter, "parameters", Parameters);
        DrawCxx.Add(theInterpreter, "bounds", Bounds);
        DrawCxx.Add(theInterpreter, "length", Length);
        DrawCxx.Add(theInterpreter, "incdeg", IncreaseDegree);
        DrawCxx.Add(theInterpreter, "insertpole", InsertPole);
        DrawCxx.Add(theInterpreter, "convert", Converting);
        DrawCxx.Add(theInterpreter, "segsur", SegSur);
        DrawCxx.Add(theInterpreter, "setweight", SetWeight);
        DrawCxx.Add(theInterpreter, "cmovepoint", CMovePoint);
    }

    // a[i], or an empty word past the end (OCCT reads Tcl's null argv[argc] there)
    private static string Arg(IReadOnlyList<string> a, int i) => i < a.Count ? a[i] : "";

    // -------------------------------------------------------------------------------------------------------------
    // GeomliteTest_CurveCommands.cxx

    private static int AnaCurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 5)
        {
            return 1;
        }

        double F(int i) => theDI.Atof(a[i]);
        Point2D P2(int i) => new(F(i), F(i + 1));
        Point3D P3(int i) => new(F(i), F(i + 1), F(i + 2));
        Direction2D D2(int i) => new(F(i), F(i + 1));
        Direction3D D3(int i) => new(F(i), F(i + 1), F(i + 2));
        Frame2D Ax22dX() => new(P2(2), new Direction2D(Direction2D.D.X));
        Frame3D Ax2Z() => new(P3(2), new Direction3D(Direction3D.D.Z));

        Geometry3D? result = null;
        Curve2D? result2d = null;
        switch (a[0])
        {
            case "line":
                if (n == 6)
                {
                    result2d = new LineCurve2D(P2(2), D2(4));
                }
                else if (n == 8)
                {
                    result = new LineCurve3D(P3(2), D3(5));
                }
                else
                {
                    return 1;
                }

                break;
            case "circle":
                switch (n)
                {
                    case 5:
                        result2d = new CircleCurve2D(Ax22dX(), F(4));
                        break;
                    case 6:
                        result = new CircleCurve3D(Ax2Z(), F(5));
                        break;
                    case 7:
                        result2d = new CircleCurve2D(new Frame2D(P2(2), D2(4)), F(6));
                        break;
                    case 9:
                        result = new CircleCurve3D(new Frame3D(P3(2), D3(5)), F(8));
                        break;
                    case 12:
                        result = new CircleCurve3D(new Frame3D(P3(2), D3(5), D3(8)), F(11));
                        break;
                    default:
                        return 1;
                }

                break;
            case "parabola":
                switch (n)
                {
                    case 5:
                        result2d = new ParabolaCurve2D(Ax22dX(), F(4));
                        break;
                    case 6:
                        result = new ParabolaCurve3D(Ax2Z(), F(5));
                        break;
                    case 7:
                        result2d = new ParabolaCurve2D(new Frame2D(P2(2), D2(4)), F(6));
                        break;
                    case 9:
                        result = new ParabolaCurve3D(new Frame3D(P3(2), D3(5)), F(8));
                        break;
                    case 12:
                        result = new ParabolaCurve3D(new Frame3D(P3(2), D3(5), D3(8)), F(11));
                        break;
                    default:
                        return 1;
                }

                break;
            case "ellipse":
                switch (n)
                {
                    case 6:
                        result2d = new EllipseCurve2D(Ax22dX(), F(4), F(5));
                        break;
                    case 7:
                        result = new EllipseCurve3D(Ax2Z(), F(5), F(6));
                        break;
                    case 8:
                        result2d = new EllipseCurve2D(new Frame2D(P2(2), D2(4)), F(6), F(7));
                        break;
                    case 10:
                        result = new EllipseCurve3D(new Frame3D(P3(2), D3(5)), F(8), F(9));
                        break;
                    case 13:
                        result = new EllipseCurve3D(new Frame3D(P3(2), D3(5), D3(8)), F(11), F(12));
                        break;
                    default:
                        return 1;
                }

                break;
            case "hyperbola":
                switch (n)
                {
                    case 6:
                        result2d = new HyperbolaCurve2D(Ax22dX(), F(4), F(5));
                        break;
                    case 7:
                        result = new HyperbolaCurve3D(Ax2Z(), F(5), F(6));
                        break;
                    case 8:
                        result2d = new HyperbolaCurve2D(new Frame2D(P2(2), D2(4)), F(6), F(7));
                        break;
                    case 10:
                        result = new HyperbolaCurve3D(new Frame3D(P3(2), D3(5)), F(8), F(9));
                        break;
                    case 13:
                        result = new HyperbolaCurve3D(new Frame3D(P3(2), D3(5), D3(8)), F(11), F(12));
                        break;
                    default:
                        return 1;
                }

                break;
        }

        if (result is not null)
        {
            DrawTrSurf.Set(theDI, a[1], result);
        }
        else if (result2d is not null)
        {
            DrawTrSurf.Set(theDI, a[1], result2d);
        }
        else
        {
            return 1;
        }

        return 0;
    }

    private static int PoleCurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        int k, i;
        if (n < 3)
        {
            return 1;
        }

        if (a[0] == "beziercurve")
        {
            int np = theDI.Atoi(a[2]);
            if (np == 0)
            {
                return 1;
            }

            i = (n - 3) / np;
            if (i < 3 || i > 4)
            {
                return 1;
            }

            bool hasw = i == 4;
            var poles = new NCollection_Array1<Point3D>(1, np);
            var weights = new NCollection_Array1<double>(1, np);
            k = 3;
            for (i = 1; i <= np; i++)
            {
                poles[i] = new Point3D(theDI.Atof(a[k]), theDI.Atof(a[k + 1]), theDI.Atof(a[k + 2]));
                k += 3;
                if (hasw)
                {
                    weights[i] = theDI.Atof(a[k]);
                    k++;
                }
            }

            BezierCurve3D result = hasw ? new BezierCurve3D(poles, weights) : new BezierCurve3D(poles);
            DrawTrSurf.Set(theDI, a[1], result);
        }
        else if ((a[0].StartsWith('p') ? a[0][1..] : a[0]) == "bsplinecurve")
        {
            int deg = theDI.Atoi(a[2]);
            int nbk = theDI.Atoi(Arg(a, 3));
            var knots = new NCollection_Array1<double>(1, nbk);
            var mults = new NCollection_Array1<int>(1, nbk);
            k = 4;
            int Sigma = 0;
            for (i = 1; i <= nbk; i++)
            {
                knots[i] = theDI.Atof(Arg(a, k));
                k++;
                mults[i] = theDI.Atoi(Arg(a, k));
                Sigma += mults[i];
                k++;
            }

            bool periodic = a[0].StartsWith('p');
            int np = periodic ? Sigma - mults[nbk] : Sigma - deg - 1;
            var poles = new NCollection_Array1<Point3D>(1, np);
            var weights = new NCollection_Array1<double>(1, np);
            for (i = 1; i <= np; i++)
            {
                poles[i] = new Point3D(theDI.Atof(Arg(a, k)), theDI.Atof(Arg(a, k + 1)), theDI.Atof(Arg(a, k + 2)));
                k += 3;
                weights[i] = theDI.Atof(Arg(a, k));
                k++;
            }

            var result = new BSplineCurve3D(poles, weights, knots, mults, deg, periodic);
            DrawTrSurf.Set(theDI, a[1], result);
        }

        return 0;
    }

    private static int PoleCurve2d(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        int k, i;
        if (n < 3)
        {
            return 1;
        }

        if (a[0] == "2dbeziercurve")
        {
            int np = theDI.Atoi(a[2]);
            if (np == 0)
            {
                return 1;
            }

            // OCCT divides n - 2 here (n - 3 for 3D): the name and the count are two words before the poles
            i = (n - 2) / np;
            if (i < 2 || i > 3)
            {
                return 1;
            }

            bool hasw = i == 3;
            var poles = new NCollection_Array1<Point2D>(1, np);
            var weights = new NCollection_Array1<double>(1, np);
            k = 3;
            for (i = 1; i <= np; i++)
            {
                poles[i] = new Point2D(theDI.Atof(Arg(a, k)), theDI.Atof(Arg(a, k + 1)));
                k += 2;
                if (hasw)
                {
                    weights[i] = theDI.Atof(Arg(a, k));
                    k++;
                }
            }

            BezierCurve2D result = hasw ? new BezierCurve2D(poles, weights) : new BezierCurve2D(poles);
            DrawTrSurf.Set(theDI, a[1], result);
        }
        else if ((a[0].Length > 2 && a[0][2] == 'p' ? a[0][3..] : a[0][2..]) == "bsplinecurve")
        {
            int deg = theDI.Atoi(a[2]);
            int nbk = theDI.Atoi(Arg(a, 3));
            var knots = new NCollection_Array1<double>(1, nbk);
            var mults = new NCollection_Array1<int>(1, nbk);
            k = 4;
            int Sigma = 0;
            for (i = 1; i <= nbk; i++)
            {
                knots[i] = theDI.Atof(Arg(a, k));
                k++;
                mults[i] = theDI.Atoi(Arg(a, k));
                Sigma += mults[i];
                k++;
            }

            bool periodic = a[0][2] == 'p';
            int np = periodic ? Sigma - mults[nbk] : Sigma - deg - 1;
            var poles = new NCollection_Array1<Point2D>(1, np);
            var weights = new NCollection_Array1<double>(1, np);
            for (i = 1; i <= np; i++)
            {
                poles[i] = new Point2D(theDI.Atof(Arg(a, k)), theDI.Atof(Arg(a, k + 1)));
                k += 2;
                weights[i] = theDI.Atof(Arg(a, k));
                k++;
            }

            var result = new BSplineCurve2D(poles, weights, knots, mults, deg, periodic);
            DrawTrSurf.Set(theDI, a[1], result);
        }

        return 0;
    }

    /// <summary>reverse: OCCT reverses the bound curve in place; the port binds the reversed copy.</summary>
    private static int Reverse(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            return 1;
        }

        for (int i = 1; i < n; i++)
        {
            if (DrawTrSurf.GetCurve(theDI, a[i]) is { } GC)
            {
                DrawTrSurf.Set(theDI, a[i], GC.Reversed());
            }

            if (DrawTrSurf.GetCurve2d(theDI, a[i]) is { } GC2d)
            {
                DrawTrSurf.Set(theDI, a[i], GC2d.Reversed());
            }
        }

        return 0;
    }

    // cmovepoint name u dx dy [dz index1 index2]
    private static int CMovePoint(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 5)
        {
            return 1;
        }

        double dx = theDI.Atof(a[3]);
        double dy = theDI.Atof(a[4]);
        double dz = 0;
        if (n >= 6 && n != 7)
        {
            dz = theDI.Atof(a[5]);
        }
        double u = theDI.Atof(a[2]);
        int index1 = 0;
        int index2 = 0;
        if (n == 7)
        {
            index1 = theDI.Atoi(a[5]);
            index2 = theDI.Atoi(a[6]);
        }
        else if (n == 8)
        {
            index1 = theDI.Atoi(a[6]);
            index2 = theDI.Atoi(a[7]);
        }

        BSplineCurve3D? G2 = DrawTrSurf.GetBSplineCurve(theDI, a[1]);
        if (G2 is not null)
        {
            if (index1 == 0)
            {
                index1 = 2;
                index2 = G2.PoleCount - 1;
            }
            G2.D0(u, out Point3D p);
            p.SetCoord(p.X() + dx, p.Y() + dy, p.Z() + dz);
            DrawTrSurf.Set(theDI, a[1], G2.PointMoved(u, p, index1, index2, out _, out _));
            return 0;
        }

        BSplineCurve2D? G4 = DrawTrSurf.GetBSplineCurve2d(theDI, a[1]);
        if (G4 is not null)
        {
            if (index1 == 0)
            {
                index1 = 2;
                index2 = G4.PoleCount - 1;
            }
            G4.D0(u, out Point2D p);
            p.SetCoord(p.X() + dx, p.Y() + dy);
            DrawTrSurf.Set(theDI, a[1], G4.PointMoved(u, p, index1, index2, out _, out _));
            return 0;
        }
        return 0;
    }

    // incdeg name degree
    private static int IncreaseDegree(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        int Deg = theDI.Atoi(a[2]);

        BezierCurve3D? GBz = DrawTrSurf.GetBezierCurve(theDI, a[1]);
        BSplineCurve3D? GBs = DrawTrSurf.GetBSplineCurve(theDI, a[1]);
        BezierCurve2D? GBz2d = DrawTrSurf.GetBezierCurve2d(theDI, a[1]);
        BSplineCurve2D? GBs2d = DrawTrSurf.GetBSplineCurve2d(theDI, a[1]);

        if (GBz is not null)
        {
            DrawTrSurf.Set(theDI, a[1], GBz.Increased(Deg));
        }
        else if (GBs is not null)
        {
            DrawTrSurf.Set(theDI, a[1], GBs.DegreeIncreased(Deg));
        }
        else if (GBz2d is not null)
        {
            DrawTrSurf.Set(theDI, a[1], GBz2d.Increased(Deg));
        }
        else if (GBs2d is not null)
        {
            DrawTrSurf.Set(theDI, a[1], GBs2d.DegreeIncreased(Deg));
        }
        else
        {
            return 1;
        }

        return 0;
    }

    // insertpole name index x y [z] [weight]
    private static int InsertPole(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 6)
        {
            return 1;
        }

        int Index = theDI.Atoi(a[2]);

        BezierCurve3D? GBZ = DrawTrSurf.GetBezierCurve(theDI, a[1]);
        BezierCurve2D? GBZ2d = DrawTrSurf.GetBezierCurve2d(theDI, a[1]);
        if (GBZ is not null)
        {
            var P = new Point3D(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));
            if (n == 7)
            {
                DrawTrSurf.Set(theDI, a[1], GBZ.PoleInsertedAfter(Index, P, theDI.Atof(a[6])));
            }
            else
            {
                DrawTrSurf.Set(theDI, a[1], GBZ.PoleInsertedAfter(Index, P));
            }
        }
        else if (GBZ2d is not null)
        {
            // OCCT: n >= 6 here, so the 2D form always takes a[5] as the weight (occt-issues.md, unit 7W)
            var P = new Point2D(theDI.Atof(a[3]), theDI.Atof(a[4]));
            if (n == 6)
            {
                DrawTrSurf.Set(theDI, a[1], GBZ2d.PoleInsertedAfter(Index, P, theDI.Atof(a[5])));
            }
            else
            {
                DrawTrSurf.Set(theDI, a[1], GBZ2d.PoleInsertedAfter(Index, P));
            }
        }
        else
        {
            di.Append("insertpole needs a bezier curve");
            return 1;
        }

        return 0;
    }

    // convert result c2d/c3d/surf [qa,c1,s1,s2,s3,s4,po]
    private static int Converting(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        ParameterisationType Parameterisation = a[n - 1] switch
        {
            "qa" => ParameterisationType.QuasiAngular,
            "c1" => ParameterisationType.RationalC1,
            "s1" => ParameterisationType.TgtThetaOver2_1,
            "s2" => ParameterisationType.TgtThetaOver2_2,
            "s3" => ParameterisationType.TgtThetaOver2_3,
            "s4" => ParameterisationType.TgtThetaOver2_4,
            "po" => ParameterisationType.Polynomial,
            _ => ParameterisationType.TgtThetaOver2,
        };

        Curve3D? GC = DrawTrSurf.GetCurve(theDI, a[2]);
        if (GC is null)
        {
            Surface? GS = DrawTrSurf.GetSurface(theDI, a[2]);
            if (GS is null)
            {
                Curve2D? G2d = DrawTrSurf.GetCurve2d(theDI, a[2]);
                if (G2d is null)
                {
                    return 1;
                }
                else
                {
                    G2d = global::Kerf.Engine.Geometry.Conversion.Geom2dConvert2D.CurveToBSplineCurve(G2d, Parameterisation);
                    DrawTrSurf.Set(theDI, a[1], G2d);
                }
            }
            else
            {
                GS = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(GS);
                DrawTrSurf.Set(theDI, a[1], GS);
            }
        }
        else
        {
            GC = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(GC, Parameterisation);
            DrawTrSurf.Set(theDI, a[1], GC);
        }

        return 0;
    }

    // segsur name Ufirst Ulast Vfirst Vlast [Utol [Vtol]]
    private static int SegSur(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 6 || n > 8)
        {
            return 1;
        }

        BezierSurface? GBz = DrawTrSurf.GetBezierSurface(theDI, a[1]);
        BSplineSurface? GBs;
        if (GBz is null)
        {
            GBs = DrawTrSurf.GetBSplineSurface(theDI, a[1]);
            if (GBs is null)
            {
                return 1;
            }

            double aUTolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
            double aVTolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
            if (n >= 7)
            {
                aUTolerance = aVTolerance = theDI.Atof(a[6]);
            }
            if (n == 8)
            {
                aVTolerance = theDI.Atof(a[7]);
            }

            DrawTrSurf.Set(theDI, a[1], GBs.Segmented(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]),
                                                      aUTolerance, aVTolerance));
        }
        else
        {
            DrawTrSurf.Set(theDI, a[1], GBz.Segmented(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5])));
        }

        return 0;
    }

    // setweight curve/surf index1 [index2] weight (GeomliteTest_ModificationCommands.cxx)
    private static int SetWeight(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4 || n > 5)
        {
            // Message::SendFail (decision 405)
            theDI.Output.WriteLine("Syntax error: Wrong parameters");
            return 1;
        }

        int anIndex1 = theDI.Atoi(a[2]);
        int anIndex2 = n == 5 ? theDI.Atoi(a[3]) : 0;
        double aWeight = theDI.Atof(a[n - 1]);

        if (DrawTrSurf.GetBSplineCurve(theDI, a[1]) is { } aBSplCurve)
        {
            DrawTrSurf.Set(theDI, a[1], aBSplCurve.WithWeight(anIndex1, aWeight));
            return 0;
        }

        if (DrawTrSurf.GetBezierCurve(theDI, a[1]) is { } aBezCurve)
        {
            DrawTrSurf.Set(theDI, a[1], aBezCurve.WithWeight(anIndex1, aWeight));
            return 0;
        }

        if (DrawTrSurf.GetBSplineCurve2d(theDI, a[1]) is { } aBSplCurve2d)
        {
            DrawTrSurf.Set(theDI, a[1], aBSplCurve2d.WithWeight(anIndex1, aWeight));
            return 0;
        }

        if (DrawTrSurf.GetBezierCurve2d(theDI, a[1]) is { } aBezCurve2d)
        {
            DrawTrSurf.Set(theDI, a[1], aBezCurve2d.WithWeight(anIndex1, aWeight));
            return 0;
        }

        BSplineSurface? aBSplSurf = DrawTrSurf.GetBSplineSurface(theDI, a[1]);
        BezierSurface? aBezSurf = DrawTrSurf.GetBezierSurface(theDI, a[1]);
        if (n != 5 && (aBSplSurf is not null || aBezSurf is not null))
        {
            theDI.Output.WriteLine("Syntax error: Incorrect parameters");
            return 1;
        }

        if (aBSplSurf is not null)
        {
            DrawTrSurf.Set(theDI, a[1], aBSplSurf.WithWeight(anIndex1, anIndex2, aWeight));
            return 0;
        }

        if (aBezSurf is not null)
        {
            DrawTrSurf.Set(theDI, a[1], aBezSurf.WithWeight(anIndex1, anIndex2, aWeight));
            return 0;
        }

        theDI.Output.WriteLine($"{a[1]} is not a B-spline nor a Bezier curve/surface");
        return 1;
    }

    private static int Segment(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4 || n > 5)
        {
            return 1;
        }

        BezierCurve3D? GBz = DrawTrSurf.GetBezierCurve(theDI, a[1]);
        BSplineCurve3D? GBs = DrawTrSurf.GetBSplineCurve(theDI, a[1]);
        BezierCurve2D? GBz2d = DrawTrSurf.GetBezierCurve2d(theDI, a[1]);
        BSplineCurve2D? GBs2d = DrawTrSurf.GetBSplineCurve2d(theDI, a[1]);
        double f = theDI.Atof(a[2]), l = theDI.Atof(a[3]);
        double aTolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (n == 5)
        {
            aTolerance = theDI.Atof(a[4]);
        }

        if (GBz is not null)
        {
            DrawTrSurf.Set(theDI, a[1], GBz.Segmented(f, l));
        }
        else if (GBs is not null)
        {
            DrawTrSurf.Set(theDI, a[1], GBs.Segmented(f, l, aTolerance));
        }
        else if (GBz2d is not null)
        {
            DrawTrSurf.Set(theDI, a[1], GBz2d.Segmented(f, l));
        }
        else if (GBs2d is not null)
        {
            DrawTrSurf.Set(theDI, a[1], GBs2d.Segmented(f, l, aTolerance));
        }
        else
        {
            return 1;
        }

        return 0;
    }

    private static int SetOrigin(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            return 1;
        }

        BSplineCurve3D? GBs = DrawTrSurf.GetBSplineCurve(theDI, a[1]);
        BSplineCurve2D? GBs2d = DrawTrSurf.GetBSplineCurve2d(theDI, a[1]);
        if (GBs is not null)
        {
            DrawTrSurf.Set(theDI, a[1], GBs.WithOrigin(theDI.Atoi(a[2])));
        }

        // OCCT: the "else" pairs with the 2D test, so a 3D curve gets its origin and the command still fails
        if (GBs2d is not null)
        {
            DrawTrSurf.Set(theDI, a[1], GBs2d.WithOrigin(theDI.Atoi(a[2])));
        }
        else
        {
            return 1;
        }

        return 0;
    }

    private static int Value(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }

        Curve3D? GC = DrawTrSurf.GetCurve(theDI, a[1]);
        if (GC is null)
        {
            return 1;
        }

        double U = theDI.Atof(a[2]);
        bool DrawPoint = n % 3 == 1;
        if (DrawPoint)
        {
            n--;
        }

        Point3D P;
        if (n > 6)
        {
            if (n < 9)
            {
                return 1;
            }

            Vector3D D1;
            if (n > 9)
            {
                if (n < 12)
                {
                    return 1;
                }

                GC.D2(U, out P, out D1, out Vector3D D2);
                theDI.SetNumber(a[9], D2.X());
                theDI.SetNumber(a[10], D2.Y());
                theDI.SetNumber(a[11], D2.Z());
            }
            else
            {
                GC.D1(U, out P, out D1);
            }

            theDI.SetNumber(a[6], D1.X());
            theDI.SetNumber(a[7], D1.Y());
            theDI.SetNumber(a[8], D1.Z());
        }
        else
        {
            GC.D0(U, out P);
        }

        if (n > 3)
        {
            theDI.SetNumber(a[3], P.X());
            theDI.SetNumber(a[4], P.Y());
            theDI.SetNumber(a[5], P.Z());
        }

        if (DrawPoint)
        {
            DrawTrSurf.Set(theDI, a[n], P);
        }

        return 0;
    }

    private static int Value2d(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }

        Curve2D? GC = DrawTrSurf.GetCurve2d(theDI, a[1]);
        if (GC is null)
        {
            return 1;
        }

        double U = theDI.Atof(a[2]);
        bool DrawPoint = n % 2 == 0;
        if (DrawPoint)
        {
            n--;
        }

        Point2D P;
        if (n > 5)
        {
            if (n < 7)
            {
                return 1;
            }

            Vector2D D1;
            if (n > 7)
            {
                if (n < 9)
                {
                    return 1;
                }

                GC.D2(U, out P, out D1, out Vector2D D2);
                theDI.SetNumber(a[7], D2.X());
                theDI.SetNumber(a[8], D2.Y());
            }
            else
            {
                GC.D1(U, out P, out D1);
            }

            theDI.SetNumber(a[5], D1.X());
            theDI.SetNumber(a[6], D1.Y());
        }
        else
        {
            GC.D0(U, out P);
        }

        if (n > 3)
        {
            theDI.SetNumber(a[3], P.X());
            theDI.SetNumber(a[4], P.Y());
        }

        if (DrawPoint)
        {
            DrawTrSurf.Set(theDI, a[n], P);
        }

        return 0;
    }

    private static int Coord(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n == 4)
        {
            var P = new Point2D();
            if (!DrawTrSurf.GetPoint2d(theDI, a[1], ref P))
            {
                return 1;
            }

            theDI.SetNumber(a[2], P.X());
            theDI.SetNumber(a[3], P.Y());
        }
        else if (n == 5)
        {
            var P = new Point3D();
            if (!DrawTrSurf.GetPoint(theDI, a[1], ref P))
            {
                return 1;
            }

            theDI.SetNumber(a[2], P.X());
            theDI.SetNumber(a[3], P.Y());
            theDI.SetNumber(a[4], P.Z());
        }
        else
        {
            return 1;
        }

        return 0;
    }

    // -------------------------------------------------------------------------------------------------------------
    // GeomliteTest_SurfaceCommands.cxx

    private static int AnaSurface(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            return 1;
        }

        double F(int j) => theDI.Atof(a[j]);
        CoordinateSystem loc;
        int i;
        if (n < 5)
        {
            loc = new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
            i = 2;
        }
        else if (n < 8)
        {
            loc = new CoordinateSystem(new Point3D(F(2), F(3), F(4)), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
            i = 5;
        }
        else if (n < 11)
        {
            loc = new CoordinateSystem(new Point3D(F(2), F(3), F(4)), new Direction3D(F(5), F(6), F(7)));
            i = 8;
        }
        else if (n < 14)
        {
            loc = new CoordinateSystem(new Point3D(F(2), F(3), F(4)), new Direction3D(F(5), F(6), F(7)), new Direction3D(F(8), F(9), F(10)));
            i = 11;
        }
        else
        {
            return 1;
        }

        Geometry3D? result = null;
        string aName = a[0].ToLowerInvariant();
        if (aName == "plane")
        {
            result = new PlaneSurface(loc);
        }
        else
        {
            if (i >= n)
            {
                return 1;
            }

            double par1 = F(i);
            if (aName == "cylinder")
            {
                result = new CylindricalSurface(loc, par1);
            }
            else if (aName == "sphere")
            {
                result = new SphericalSurface(loc, par1);
            }
            else
            {
                if (i + 1 >= n)
                {
                    return 1;
                }

                double par2 = F(i + 1);
                if (aName == "cone")
                {
                    par1 *= Math.PI / 180.0;
                    result = new ConicalSurface(loc, par1, par2);
                }
                else if (aName == "torus")
                {
                    result = new ToroidalSurface(loc, par1, par2);
                }
            }
        }

        DrawTrSurf.Set(theDI, a[1], result);
        return 0;
    }

    private static int PoleSurface(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        int k, j, i;
        if (n < 4)
        {
            return 1;
        }

        if (a[0].Equals("beziersurf", StringComparison.OrdinalIgnoreCase))
        {
            int nup = theDI.Atoi(a[2]);
            int nvp = theDI.Atoi(a[3]);
            if (nup * nvp == 0)
            {
                return 1;
            }

            i = (n - 4) / (nup * nvp);
            if (i < 3 || i > 4)
            {
                return 1;
            }

            bool hasw = i == 4;
            var poles = new NCollection_Array2<Point3D>(1, nup, 1, nvp);
            var weights = new NCollection_Array2<double>(1, nup, 1, nvp);
            k = 4;
            for (j = 1; j <= nvp; j++)
            {
                for (i = 1; i <= nup; i++)
                {
                    poles[i, j] = new Point3D(theDI.Atof(Arg(a, k)), theDI.Atof(Arg(a, k + 1)), theDI.Atof(Arg(a, k + 2)));
                    k += 3;
                    if (hasw)
                    {
                        weights[i, j] = theDI.Atof(Arg(a, k));
                        k++;
                    }
                }
            }

            BezierSurface result = hasw ? new BezierSurface(poles, weights) : new BezierSurface(poles);
            DrawTrSurf.Set(theDI, a[1], result);
        }
        else
        {
            int udeg = theDI.Atoi(a[2]);
            int nbuk = theDI.Atoi(Arg(a, 3));
            bool uper = a[0][0] == 'u' || (a[0].Length > 1 && a[0][1] == 'u');
            bool vper = a[0][0] == 'v' || (a[0].Length > 1 && a[0][1] == 'v');
            var uk = new NCollection_Array1<double>(1, nbuk);
            var umult = new NCollection_Array1<int>(1, nbuk);
            k = 4;
            int SigmaU = 0;
            for (i = 1; i <= nbuk; i++)
            {
                uk[i] = theDI.Atof(Arg(a, k));
                k++;
                umult[i] = theDI.Atoi(Arg(a, k));
                SigmaU += umult[i];
                k++;
            }

            int vdeg = theDI.Atoi(Arg(a, k));
            k++;
            int nbvk = theDI.Atoi(Arg(a, k));
            k++;
            var vk = new NCollection_Array1<double>(1, nbvk);
            var vmult = new NCollection_Array1<int>(1, nbvk);
            int SigmaV = 0;
            for (i = 1; i <= nbvk; i++)
            {
                vk[i] = theDI.Atof(Arg(a, k));
                k++;
                vmult[i] = theDI.Atoi(Arg(a, k));
                SigmaV += vmult[i];
                k++;
            }

            int nup = uper ? SigmaU - umult[nbuk] : SigmaU - udeg - 1;
            int nvp = vper ? SigmaV - vmult[nbvk] : SigmaV - vdeg - 1;
            var poles = new NCollection_Array2<Point3D>(1, nup, 1, nvp);
            var weights = new NCollection_Array2<double>(1, nup, 1, nvp);
            for (j = 1; j <= nvp; j++)
            {
                for (i = 1; i <= nup; i++)
                {
                    poles[i, j] = new Point3D(theDI.Atof(Arg(a, k)), theDI.Atof(Arg(a, k + 1)), theDI.Atof(Arg(a, k + 2)));
                    k += 3;
                    weights[i, j] = theDI.Atof(Arg(a, k));
                    k++;
                }
            }

            var result = new BSplineSurface(poles, weights, uk, vk, umult, vmult, udeg, vdeg, uper, vper);
            DrawTrSurf.Set(theDI, a[1], result);
        }

        return 0;
    }

    private static int AlgoSurface(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 5)
        {
            return 1;
        }

        Curve3D? GC = DrawTrSurf.GetCurve(theDI, a[2]);
        if (GC is null)
        {
            return 1;
        }

        if (a[0].Equals("extsurf", StringComparison.OrdinalIgnoreCase))
        {
            var D = new Direction3D(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(Arg(a, 5)));
            DrawTrSurf.Set(theDI, a[1], new ExtrusionSurface(GC, D));
        }
        else if (a[0].Equals("revsurf", StringComparison.OrdinalIgnoreCase))
        {
            if (n < 8)
            {
                return 1;
            }

            var P = new Point3D(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));
            var D = new Direction3D(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(Arg(a, 8)));
            DrawTrSurf.Set(theDI, a[1], new RevolutionSurface(GC, new Axis3D(P, D)));
        }

        return 0;
    }

    private static int Trimming(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        Curve3D? GC = DrawTrSurf.GetCurve(theDI, a[2]);
        Curve2D? GC2d = DrawTrSurf.GetCurve2d(theDI, a[2]);
        Surface? GS = DrawTrSurf.GetSurface(theDI, a[2]);
        if (n == 3)
        {
            if (GC is not null)
            {
                if (GC is TrimmedCurve3D T)
                {
                    GC = T.BasisCurve();
                }

                DrawTrSurf.Set(theDI, a[1], GC);
            }
            else if (GC2d is not null)
            {
                if (GC2d is TrimmedCurve2D T)
                {
                    GC2d = T.BasisCurve();
                }

                DrawTrSurf.Set(theDI, a[1], GC2d);
            }
            else if (GS is not null)
            {
                if (GS is TrimmedSurface T)
                {
                    GS = T.BasisSurface();
                }

                DrawTrSurf.Set(theDI, a[1], GS);
            }

            return 0;
        }

        if (n < 5)
        {
            return 1;
        }

        double u1 = theDI.Atof(a[3]);
        double u2 = theDI.Atof(a[4]);
        bool USense = true, VSense = true;
        Geometry3D? result = null;
        Curve2D? result2d = null;
        static bool Sense(string theWord) => theWord.Length == 0 || theWord[0] != '0';

        if (a[0].Equals("trim", StringComparison.OrdinalIgnoreCase))
        {
            if (GS is not null)
            {
                if (n < 7)
                {
                    return 1;
                }

                double v1 = theDI.Atof(a[5]);
                double v2 = theDI.Atof(a[6]);
                if (n > 7)
                {
                    USense = Sense(a[7]);
                    // OCCT reads argv[8] even when there are 8 words (a null pointer there)
                    VSense = Sense(Arg(a, 8));
                }

                result = new TrimmedSurface(GS, u1, u2, v1, v2, USense, VSense);
            }
            else if (GC is not null)
            {
                if (n > 5)
                {
                    USense = Sense(a[5]);
                }

                result = new TrimmedCurve3D(GC, u1, u2, USense);
            }
            else if (GC2d is not null)
            {
                if (n > 5)
                {
                    USense = Sense(a[5]);
                }

                result2d = new TrimmedCurve2D(GC2d, u1, u2, USense);
            }
            else
            {
                return 1;
            }
        }
        else
        {
            if (GS is null)
            {
                return 1;
            }

            bool Utrim = a[0].Equals("trimu", StringComparison.OrdinalIgnoreCase);
            if (n > 5)
            {
                USense = Sense(a[5]);
            }

            result = new TrimmedSurface(GS, u1, u2, Utrim, USense);
        }

        if (result is not null)
        {
            DrawTrSurf.Set(theDI, a[1], result);
        }
        else
        {
            DrawTrSurf.Set(theDI, a[1], result2d);
        }

        return 0;
    }

    private static int Offseting(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }

        // test the Geom2d curve
        if (DrawTrSurf.GetCurve2d(theDI, a[2]) is { } C2d)
        {
            DrawTrSurf.Set(theDI, a[1], new OffsetCurve2D(C2d, theDI.Atof(a[3])));
            return 0;
        }

        Curve3D? GC = DrawTrSurf.GetCurve(theDI, a[2]);
        Surface? GS = null;
        if (GC is null)
        {
            GS = DrawTrSurf.GetSurface(theDI, a[2]);
            if (GS is null)
            {
                return 1;
            }
        }

        double dist = theDI.Atof(a[3]);
        Geometry3D result;
        if (GS is not null)
        {
            result = new OffsetSurface(GS, dist);
        }
        else
        {
            if (n < 7)
            {
                return 1;
            }

            var D = new Direction3D(theDI.Atof(a[4]), theDI.Atof(a[5]), theDI.Atof(a[6]));
            result = new OffsetCurve3D(GC!, dist, D);
        }

        DrawTrSurf.Set(theDI, a[1], result);
        return 0;
    }

    /// <summary>ureverse, vreverse: OCCT reverses the bound surface in place; the port binds the reversed copy.</summary>
    private static int SReverse(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            return 1;
        }

        for (int i = 1; i < n; i++)
        {
            if (DrawTrSurf.GetSurface(theDI, a[i]) is { } GS)
            {
                DrawTrSurf.Set(theDI, a[i], a[0][0] == 'u' ? GS.UReversed() : GS.VReversed());
            }
        }

        return 0;
    }

    private static int Iso(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 4)
        {
            return 1;
        }

        double par = theDI.Atof(a[3]);
        if (DrawTrSurf.GetSurface(theDI, a[2]) is { } GS)
        {
            DrawTrSurf.Set(theDI, a[1], a[0][0] == 'u' ? GS.UIso(par) : GS.VIso(par));
        }

        return 0;
    }

    private static int SValue(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 5)
        {
            return 1;
        }

        Surface? GS = DrawTrSurf.GetSurface(theDI, a[1]);
        if (GS is null)
        {
            return 1;
        }

        double U = theDI.Atof(a[2]);
        double V = theDI.Atof(a[3]);
        bool DrawPoint = n % 3 == 2;
        if (DrawPoint)
        {
            n--;
        }

        Point3D P;
        if (n >= 13)
        {
            Vector3D DU, DV;
            if (n >= 22)
            {
                GS.D2(U, V, out P, out DU, out DV, out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV);
                theDI.SetNumber(a[13], D2U.X());
                theDI.SetNumber(a[14], D2U.Y());
                theDI.SetNumber(a[15], D2U.Z());
                theDI.SetNumber(a[16], D2V.X());
                theDI.SetNumber(a[17], D2V.Y());
                theDI.SetNumber(a[18], D2V.Z());
                theDI.SetNumber(a[19], D2UV.X());
                theDI.SetNumber(a[20], D2UV.Y());
                theDI.SetNumber(a[21], D2UV.Z());
            }
            else
            {
                GS.D1(U, V, out P, out DU, out DV);
            }

            theDI.SetNumber(a[7], DU.X());
            theDI.SetNumber(a[8], DU.Y());
            theDI.SetNumber(a[9], DU.Z());
            theDI.SetNumber(a[10], DV.X());
            theDI.SetNumber(a[11], DV.Y());
            theDI.SetNumber(a[12], DV.Z());
        }
        else
        {
            GS.D0(U, V, out P);
        }

        if (n > 6)
        {
            theDI.SetNumber(a[4], P.X());
            theDI.SetNumber(a[5], P.Y());
            theDI.SetNumber(a[6], P.Z());
        }

        if (DrawPoint)
        {
            DrawTrSurf.Set(theDI, a[n], P);
        }

        return 0;
    }

    private static int Derivative(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgv)
    {
        if (theArgv.Count != 9)
        {
            return 1;
        }

        Surface? aSurf = DrawTrSurf.GetSurface(theDI, theArgv[1]);
        if (aSurf is null)
        {
            return 1;
        }

        double aU = theDI.Atof(theArgv[2]);
        double aV = theDI.Atof(theArgv[3]);
        int aNu = theDI.Atoi(theArgv[4]);
        int aNv = theDI.Atoi(theArgv[5]);
        Vector3D aDeriv = aSurf.DN(aU, aV, aNu, aNv);
        theDI.SetNumber(theArgv[6], aDeriv.X());
        theDI.SetNumber(theArgv[7], aDeriv.Y());
        theDI.SetNumber(theArgv[8], aDeriv.Z());
        return 0;
    }

    private static int Parameters(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n == 8)
        {
            Surface? S = DrawTrSurf.GetSurface(theDI, a[1]);
            if (S is null)
            {
                di.Append("Unknown surface\n");
                return 1;
            }

            var P = new Point3D(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]));
            double Tol = theDI.Atof(a[5]);
            bool res = GeometryLibraryTool.Parameters(S, P, Tol, out double U, out double V);
            theDI.SetNumber(a[6], U);
            theDI.SetNumber(a[7], V);
            if (!res)
            {
                di.Append("Wrong point\n");
                return 1;
            }
        }
        else if (n == 7)
        {
            Curve3D? C = DrawTrSurf.GetCurve(theDI, a[1]);
            if (C is null)
            {
                di.Append("Unknown curve\n");
                return 1;
            }

            var P = new Point3D(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]));
            double Tol = theDI.Atof(a[5]);
            bool res = GeometryLibraryTool.Parameter(C, P, Tol, out double U);
            theDI.SetNumber(a[6], U);
            if (!res)
            {
                di.Append("Wrong point\n");
                return 1;
            }
        }
        else if (n == 6)
        {
            Curve2D? C = DrawTrSurf.GetCurve2d(theDI, a[1]);
            if (C is null)
            {
                di.Append("Unknown curve 2d\n");
                return 1;
            }

            var P = new Point2D(theDI.Atof(a[2]), theDI.Atof(a[3]));
            double Tol = theDI.Atof(a[4]);
            bool res = GeometryLibraryTool.Parameter(C, P, Tol, out double U);
            theDI.SetNumber(a[5], U);
            if (!res)
            {
                di.Append("Wrong point\n");
                return 1;
            }
        }
        else
        {
            di.Append("Invalid parameters!\n");
            di.Append("Usage:\n");
            di.Append("parameters Surf X Y Z Tol U V\n");
            di.Append("parameters Curv X Y Z Tol U\n");
            di.Append("parameters Curv2d X Y Tol U\n");
            return 1;
        }

        return 0;
    }

    private static int Bounds(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n == 4)
        {
            double U1, U2;
            Curve3D? C3d = DrawTrSurf.GetCurve(theDI, a[1]);
            if (C3d is null)
            {
                Curve2D? C2d = DrawTrSurf.GetCurve2d(theDI, a[1]);
                if (C2d is null)
                {
                    return 1;
                }

                U1 = C2d.FirstParameter();
                U2 = C2d.LastParameter();
            }
            else
            {
                U1 = C3d.FirstParameter();
                U2 = C3d.LastParameter();
            }

            theDI.SetNumber(a[2], U1);
            theDI.SetNumber(a[3], U2);
        }
        else if (n == 6)
        {
            Surface? S = DrawTrSurf.GetSurface(theDI, a[1]);
            if (S is null)
            {
                return 1;
            }

            S.Bounds(out double U1, out double U2, out double V1, out double V2);
            theDI.SetNumber(a[2], U1);
            theDI.SetNumber(a[3], U2);
            theDI.SetNumber(a[4], V1);
            theDI.SetNumber(a[5], V2);
        }

        return 0;
    }

    private static int Length(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            return 1;
        }

        Curve3D? GC = DrawTrSurf.GetCurve(theDI, a[1]);
        Curve2D? GC2d = DrawTrSurf.GetCurve2d(theDI, a[1]);
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion(), L;
        if (n == 3)
        {
            Tol = theDI.Atof(a[2]);
        }

        if (GC is not null)
        {
            L = AbscissaPoint.Length(new CurveAdaptor(GC), Tol);
        }
        else if (GC2d is not null)
        {
            L = AbscissaPoint.Length(new CurveAdaptor2D(GC2d), Tol);
        }
        else
        {
            // OCCT: no space before "is"
            di.Append(a[1]).Append("is not a curve\n");
            return 1;
        }

        di.Append("The length ").Append(a[1]).Append(" is ").Append(L).Append("\n");
        return 0;
    }
}