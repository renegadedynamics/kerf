// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// The curve and surface commands of GeomliteTest_CurveCommands.cxx and GeomliteTest_SurfaceCommands.cxx that
/// construct, trim, offset and evaluate geometry: the analytic curves and surfaces, the Bezier and B-spline ones
/// from poles, the swept surfaces, offsets and trims, isos, values, bounds and parameters.
/// </summary>
/// <remarks>
/// OCCT's <c>reverse</c>, <c>ureverse</c>, <c>vreverse</c>, <c>segment</c> and <c>setorigin</c> change the bound
/// geometry in place. Geometry is frozen when bound (DrawTrSurf.Set), so they bind the changed copy that the port's
/// public API returns (<c>Reversed</c>, <c>UReversed</c>, <c>Segmented</c>, <c>WithOrigin</c>) under the same name:
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
        gp_Pnt2d P2(int i) => new(F(i), F(i + 1));
        gp_Pnt P3(int i) => new(F(i), F(i + 1), F(i + 2));
        gp_Dir2d D2(int i) => new(F(i), F(i + 1));
        gp_Dir D3(int i) => new(F(i), F(i + 1), F(i + 2));
        gp_Ax22d Ax22dX() => new(P2(2), new gp_Dir2d(gp_Dir2d.D.X));
        gp_Ax2 Ax2Z() => new(P3(2), new gp_Dir(gp_Dir.D.Z));

        Geom_Geometry? result = null;
        Geom2d_Curve? result2d = null;
        switch (a[0])
        {
            case "line":
                if (n == 6)
                {
                    result2d = new Geom2d_Line(P2(2), D2(4));
                }
                else if (n == 8)
                {
                    result = new Geom_Line(P3(2), D3(5));
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
                        result2d = new Geom2d_Circle(Ax22dX(), F(4));
                        break;
                    case 6:
                        result = new Geom_Circle(Ax2Z(), F(5));
                        break;
                    case 7:
                        result2d = new Geom2d_Circle(new gp_Ax22d(P2(2), D2(4)), F(6));
                        break;
                    case 9:
                        result = new Geom_Circle(new gp_Ax2(P3(2), D3(5)), F(8));
                        break;
                    case 12:
                        result = new Geom_Circle(new gp_Ax2(P3(2), D3(5), D3(8)), F(11));
                        break;
                    default:
                        return 1;
                }

                break;
            case "parabola":
                switch (n)
                {
                    case 5:
                        result2d = new Geom2d_Parabola(Ax22dX(), F(4));
                        break;
                    case 6:
                        result = new Geom_Parabola(Ax2Z(), F(5));
                        break;
                    case 7:
                        result2d = new Geom2d_Parabola(new gp_Ax22d(P2(2), D2(4)), F(6));
                        break;
                    case 9:
                        result = new Geom_Parabola(new gp_Ax2(P3(2), D3(5)), F(8));
                        break;
                    case 12:
                        result = new Geom_Parabola(new gp_Ax2(P3(2), D3(5), D3(8)), F(11));
                        break;
                    default:
                        return 1;
                }

                break;
            case "ellipse":
                switch (n)
                {
                    case 6:
                        result2d = new Geom2d_Ellipse(Ax22dX(), F(4), F(5));
                        break;
                    case 7:
                        result = new Geom_Ellipse(Ax2Z(), F(5), F(6));
                        break;
                    case 8:
                        result2d = new Geom2d_Ellipse(new gp_Ax22d(P2(2), D2(4)), F(6), F(7));
                        break;
                    case 10:
                        result = new Geom_Ellipse(new gp_Ax2(P3(2), D3(5)), F(8), F(9));
                        break;
                    case 13:
                        result = new Geom_Ellipse(new gp_Ax2(P3(2), D3(5), D3(8)), F(11), F(12));
                        break;
                    default:
                        return 1;
                }

                break;
            case "hyperbola":
                switch (n)
                {
                    case 6:
                        result2d = new Geom2d_Hyperbola(Ax22dX(), F(4), F(5));
                        break;
                    case 7:
                        result = new Geom_Hyperbola(Ax2Z(), F(5), F(6));
                        break;
                    case 8:
                        result2d = new Geom2d_Hyperbola(new gp_Ax22d(P2(2), D2(4)), F(6), F(7));
                        break;
                    case 10:
                        result = new Geom_Hyperbola(new gp_Ax2(P3(2), D3(5)), F(8), F(9));
                        break;
                    case 13:
                        result = new Geom_Hyperbola(new gp_Ax2(P3(2), D3(5), D3(8)), F(11), F(12));
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
            var poles = new NCollection_Array1<gp_Pnt>(1, np);
            var weights = new NCollection_Array1<double>(1, np);
            k = 3;
            for (i = 1; i <= np; i++)
            {
                poles[i] = new gp_Pnt(theDI.Atof(a[k]), theDI.Atof(a[k + 1]), theDI.Atof(a[k + 2]));
                k += 3;
                if (hasw)
                {
                    weights[i] = theDI.Atof(a[k]);
                    k++;
                }
            }

            Geom_BezierCurve result = hasw ? new Geom_BezierCurve(poles, weights) : new Geom_BezierCurve(poles);
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
            var poles = new NCollection_Array1<gp_Pnt>(1, np);
            var weights = new NCollection_Array1<double>(1, np);
            for (i = 1; i <= np; i++)
            {
                poles[i] = new gp_Pnt(theDI.Atof(Arg(a, k)), theDI.Atof(Arg(a, k + 1)), theDI.Atof(Arg(a, k + 2)));
                k += 3;
                weights[i] = theDI.Atof(Arg(a, k));
                k++;
            }

            var result = new Geom_BSplineCurve(poles, weights, knots, mults, deg, periodic);
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
            var poles = new NCollection_Array1<gp_Pnt2d>(1, np);
            var weights = new NCollection_Array1<double>(1, np);
            k = 3;
            for (i = 1; i <= np; i++)
            {
                poles[i] = new gp_Pnt2d(theDI.Atof(Arg(a, k)), theDI.Atof(Arg(a, k + 1)));
                k += 2;
                if (hasw)
                {
                    weights[i] = theDI.Atof(Arg(a, k));
                    k++;
                }
            }

            Geom2d_BezierCurve result = hasw ? new Geom2d_BezierCurve(poles, weights) : new Geom2d_BezierCurve(poles);
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
            var poles = new NCollection_Array1<gp_Pnt2d>(1, np);
            var weights = new NCollection_Array1<double>(1, np);
            for (i = 1; i <= np; i++)
            {
                poles[i] = new gp_Pnt2d(theDI.Atof(Arg(a, k)), theDI.Atof(Arg(a, k + 1)));
                k += 2;
                weights[i] = theDI.Atof(Arg(a, k));
                k++;
            }

            var result = new Geom2d_BSplineCurve(poles, weights, knots, mults, deg, periodic);
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

    private static int Segment(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4 || n > 5)
        {
            return 1;
        }

        Geom_BezierCurve? GBz = DrawTrSurf.GetBezierCurve(theDI, a[1]);
        Geom_BSplineCurve? GBs = DrawTrSurf.GetBSplineCurve(theDI, a[1]);
        Geom2d_BezierCurve? GBz2d = DrawTrSurf.GetBezierCurve2d(theDI, a[1]);
        Geom2d_BSplineCurve? GBs2d = DrawTrSurf.GetBSplineCurve2d(theDI, a[1]);
        double f = theDI.Atof(a[2]), l = theDI.Atof(a[3]);
        double aTolerance = Precision.PConfusion();
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

        Geom_BSplineCurve? GBs = DrawTrSurf.GetBSplineCurve(theDI, a[1]);
        Geom2d_BSplineCurve? GBs2d = DrawTrSurf.GetBSplineCurve2d(theDI, a[1]);
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

        Geom_Curve? GC = DrawTrSurf.GetCurve(theDI, a[1]);
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

        gp_Pnt P;
        if (n > 6)
        {
            if (n < 9)
            {
                return 1;
            }

            gp_Vec D1;
            if (n > 9)
            {
                if (n < 12)
                {
                    return 1;
                }

                GC.D2(U, out P, out D1, out gp_Vec D2);
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

        Geom2d_Curve? GC = DrawTrSurf.GetCurve2d(theDI, a[1]);
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

        gp_Pnt2d P;
        if (n > 5)
        {
            if (n < 7)
            {
                return 1;
            }

            gp_Vec2d D1;
            if (n > 7)
            {
                if (n < 9)
                {
                    return 1;
                }

                GC.D2(U, out P, out D1, out gp_Vec2d D2);
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
            var P = new gp_Pnt2d();
            if (!DrawTrSurf.GetPoint2d(theDI, a[1], ref P))
            {
                return 1;
            }

            theDI.SetNumber(a[2], P.X());
            theDI.SetNumber(a[3], P.Y());
        }
        else if (n == 5)
        {
            var P = new gp_Pnt();
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
        gp_Ax3 loc;
        int i;
        if (n < 5)
        {
            loc = new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X));
            i = 2;
        }
        else if (n < 8)
        {
            loc = new gp_Ax3(new gp_Pnt(F(2), F(3), F(4)), new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X));
            i = 5;
        }
        else if (n < 11)
        {
            loc = new gp_Ax3(new gp_Pnt(F(2), F(3), F(4)), new gp_Dir(F(5), F(6), F(7)));
            i = 8;
        }
        else if (n < 14)
        {
            loc = new gp_Ax3(new gp_Pnt(F(2), F(3), F(4)), new gp_Dir(F(5), F(6), F(7)), new gp_Dir(F(8), F(9), F(10)));
            i = 11;
        }
        else
        {
            return 1;
        }

        Geom_Geometry? result = null;
        string aName = a[0].ToLowerInvariant();
        if (aName == "plane")
        {
            result = new Geom_Plane(loc);
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
                result = new Geom_CylindricalSurface(loc, par1);
            }
            else if (aName == "sphere")
            {
                result = new Geom_SphericalSurface(loc, par1);
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
                    result = new Geom_ConicalSurface(loc, par1, par2);
                }
                else if (aName == "torus")
                {
                    result = new Geom_ToroidalSurface(loc, par1, par2);
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
            var poles = new NCollection_Array2<gp_Pnt>(1, nup, 1, nvp);
            var weights = new NCollection_Array2<double>(1, nup, 1, nvp);
            k = 4;
            for (j = 1; j <= nvp; j++)
            {
                for (i = 1; i <= nup; i++)
                {
                    poles[i, j] = new gp_Pnt(theDI.Atof(Arg(a, k)), theDI.Atof(Arg(a, k + 1)), theDI.Atof(Arg(a, k + 2)));
                    k += 3;
                    if (hasw)
                    {
                        weights[i, j] = theDI.Atof(Arg(a, k));
                        k++;
                    }
                }
            }

            Geom_BezierSurface result = hasw ? new Geom_BezierSurface(poles, weights) : new Geom_BezierSurface(poles);
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
            var poles = new NCollection_Array2<gp_Pnt>(1, nup, 1, nvp);
            var weights = new NCollection_Array2<double>(1, nup, 1, nvp);
            for (j = 1; j <= nvp; j++)
            {
                for (i = 1; i <= nup; i++)
                {
                    poles[i, j] = new gp_Pnt(theDI.Atof(Arg(a, k)), theDI.Atof(Arg(a, k + 1)), theDI.Atof(Arg(a, k + 2)));
                    k += 3;
                    weights[i, j] = theDI.Atof(Arg(a, k));
                    k++;
                }
            }

            var result = new Geom_BSplineSurface(poles, weights, uk, vk, umult, vmult, udeg, vdeg, uper, vper);
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

        Geom_Curve? GC = DrawTrSurf.GetCurve(theDI, a[2]);
        if (GC is null)
        {
            return 1;
        }

        if (a[0].Equals("extsurf", StringComparison.OrdinalIgnoreCase))
        {
            var D = new gp_Dir(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(Arg(a, 5)));
            DrawTrSurf.Set(theDI, a[1], new Geom_SurfaceOfLinearExtrusion(GC, D));
        }
        else if (a[0].Equals("revsurf", StringComparison.OrdinalIgnoreCase))
        {
            if (n < 8)
            {
                return 1;
            }

            var P = new gp_Pnt(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));
            var D = new gp_Dir(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(Arg(a, 8)));
            DrawTrSurf.Set(theDI, a[1], new Geom_SurfaceOfRevolution(GC, new gp_Ax1(P, D)));
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

        Geom_Curve? GC = DrawTrSurf.GetCurve(theDI, a[2]);
        Geom2d_Curve? GC2d = DrawTrSurf.GetCurve2d(theDI, a[2]);
        Geom_Surface? GS = DrawTrSurf.GetSurface(theDI, a[2]);
        if (n == 3)
        {
            if (GC is not null)
            {
                if (GC is Geom_TrimmedCurve T)
                {
                    GC = T.BasisCurve();
                }

                DrawTrSurf.Set(theDI, a[1], GC);
            }
            else if (GC2d is not null)
            {
                if (GC2d is Geom2d_TrimmedCurve T)
                {
                    GC2d = T.BasisCurve();
                }

                DrawTrSurf.Set(theDI, a[1], GC2d);
            }
            else if (GS is not null)
            {
                if (GS is Geom_RectangularTrimmedSurface T)
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
        Geom_Geometry? result = null;
        Geom2d_Curve? result2d = null;
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

                result = new Geom_RectangularTrimmedSurface(GS, u1, u2, v1, v2, USense, VSense);
            }
            else if (GC is not null)
            {
                if (n > 5)
                {
                    USense = Sense(a[5]);
                }

                result = new Geom_TrimmedCurve(GC, u1, u2, USense);
            }
            else if (GC2d is not null)
            {
                if (n > 5)
                {
                    USense = Sense(a[5]);
                }

                result2d = new Geom2d_TrimmedCurve(GC2d, u1, u2, USense);
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

            result = new Geom_RectangularTrimmedSurface(GS, u1, u2, Utrim, USense);
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
            DrawTrSurf.Set(theDI, a[1], new Geom2d_OffsetCurve(C2d, theDI.Atof(a[3])));
            return 0;
        }

        Geom_Curve? GC = DrawTrSurf.GetCurve(theDI, a[2]);
        Geom_Surface? GS = null;
        if (GC is null)
        {
            GS = DrawTrSurf.GetSurface(theDI, a[2]);
            if (GS is null)
            {
                return 1;
            }
        }

        double dist = theDI.Atof(a[3]);
        Geom_Geometry result;
        if (GS is not null)
        {
            result = new Geom_OffsetSurface(GS, dist);
        }
        else
        {
            if (n < 7)
            {
                return 1;
            }

            var D = new gp_Dir(theDI.Atof(a[4]), theDI.Atof(a[5]), theDI.Atof(a[6]));
            result = new Geom_OffsetCurve(GC!, dist, D);
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

        Geom_Surface? GS = DrawTrSurf.GetSurface(theDI, a[1]);
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

        gp_Pnt P;
        if (n >= 13)
        {
            gp_Vec DU, DV;
            if (n >= 22)
            {
                GS.D2(U, V, out P, out DU, out DV, out gp_Vec D2U, out gp_Vec D2V, out gp_Vec D2UV);
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

        Geom_Surface? aSurf = DrawTrSurf.GetSurface(theDI, theArgv[1]);
        if (aSurf is null)
        {
            return 1;
        }

        double aU = theDI.Atof(theArgv[2]);
        double aV = theDI.Atof(theArgv[3]);
        int aNu = theDI.Atoi(theArgv[4]);
        int aNv = theDI.Atoi(theArgv[5]);
        gp_Vec aDeriv = aSurf.DN(aU, aV, aNu, aNv);
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
            Geom_Surface? S = DrawTrSurf.GetSurface(theDI, a[1]);
            if (S is null)
            {
                di.Append("Unknown surface\n");
                return 1;
            }

            var P = new gp_Pnt(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]));
            double Tol = theDI.Atof(a[5]);
            bool res = GeomLib_Tool.Parameters(S, P, Tol, out double U, out double V);
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
            Geom_Curve? C = DrawTrSurf.GetCurve(theDI, a[1]);
            if (C is null)
            {
                di.Append("Unknown curve\n");
                return 1;
            }

            var P = new gp_Pnt(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]));
            double Tol = theDI.Atof(a[5]);
            bool res = GeomLib_Tool.Parameter(C, P, Tol, out double U);
            theDI.SetNumber(a[6], U);
            if (!res)
            {
                di.Append("Wrong point\n");
                return 1;
            }
        }
        else if (n == 6)
        {
            Geom2d_Curve? C = DrawTrSurf.GetCurve2d(theDI, a[1]);
            if (C is null)
            {
                di.Append("Unknown curve 2d\n");
                return 1;
            }

            var P = new gp_Pnt2d(theDI.Atof(a[2]), theDI.Atof(a[3]));
            double Tol = theDI.Atof(a[4]);
            bool res = GeomLib_Tool.Parameter(C, P, Tol, out double U);
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
            Geom_Curve? C3d = DrawTrSurf.GetCurve(theDI, a[1]);
            if (C3d is null)
            {
                Geom2d_Curve? C2d = DrawTrSurf.GetCurve2d(theDI, a[1]);
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
            Geom_Surface? S = DrawTrSurf.GetSurface(theDI, a[1]);
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

        Geom_Curve? GC = DrawTrSurf.GetCurve(theDI, a[1]);
        Geom2d_Curve? GC2d = DrawTrSurf.GetCurve2d(theDI, a[1]);
        double Tol = Precision.Confusion(), L;
        if (n == 3)
        {
            Tol = theDI.Atof(a[2]);
        }

        if (GC is not null)
        {
            L = GCPnts_AbscissaPoint.Length(new GeomAdaptor_Curve(GC), Tol);
        }
        else if (GC2d is not null)
        {
            L = GCPnts_AbscissaPoint.Length(new Geom2dAdaptor_Curve(GC2d), Tol);
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