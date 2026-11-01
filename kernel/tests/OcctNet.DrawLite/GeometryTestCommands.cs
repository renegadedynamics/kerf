// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// GeometryTest_ConstraintCommands.cxx's <c>gcarc</c> (segments and arcs of circles through points, GC),
/// <c>cirtang</c> and <c>lintan</c> (circles and lines under constraints, Geom2dGcc), <c>interpol</c> and
/// <c>tanginterpol</c> (GeomAPI_Interpolate and Geom2dAPI_Interpolate, unit 5W), and GeometryTest_APICommands.cxx's
/// <c>extrema</c> (GeomAPI_ExtremaCurveCurve, CurveSurface, SurfaceSurface) and <c>proj</c> (GeomAPI_ProjectPointOnCurve,
/// GeomAPI_ProjectPointOnSurf and Extrema_GenLocateExtPS, unit 7W).
/// </summary>
/// <remarks>
/// Draw's curve colour changes around the constraint commands' results are left out (no viewer). <c>interpol</c>'s
/// interactive mode (<c>interpol name</c>: points picked in a view) prints its prompt and makes nothing, as when no
/// view is picked; its file mode is ported.
/// </remarks>
internal static class GeometryTestCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "cirtang", Cirtang);
        DrawCxx.Add(theInterpreter, "lintan", Lintang);
        DrawCxx.Add(theInterpreter, "gcarc", GcArc);
        DrawCxx.Add(theInterpreter, "interpol", interpol);
        DrawCxx.Add(theInterpreter, "tanginterpol", tanginterpol);
        DrawCxx.Add(theInterpreter, "extrema", Extrema);
        DrawCxx.Add(theInterpreter, "proj", Proj);
    }

    private static int interpol(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n == 1)
        {
            di.Append("give a name to your curve !\n");
            return 0;
        }
        if (n == 2)
        {
            di.Append("Pick points \n");
            // dout.Select: Draw-lite has no view, so nothing is picked (id < 0)
            return 0;
        }
        else if (n == 3)
        {
            // lecture du fichier.
            // nbpoints, 2d ou 3d, puis valeurs.
            string nomfic = a[2];
            if (!theDI.Files.TryReadBytes(nomfic, out byte[] aBytes))
            {
                return 1;
            }
            using var iFile = new StringReader(System.Text.Encoding.Latin1.GetString(aBytes));
            int nbp = 0, i;
            double x, y, z;
            Standard_Stream.Read(iFile, ref nbp);
            const int aBufferSize = 3;
            string dimen = Standard_Stream.ReadWord(iFile, aBufferSize) ?? "";
            if (dimen == "3d")
            {
                var Point = new NCollection_HArray1<gp_Pnt>(1, nbp);
                for (i = 1; i <= nbp; i++)
                {
                    GeomTools.GetReal(iFile, out x);
                    GeomTools.GetReal(iFile, out y);
                    GeomTools.GetReal(iFile, out z);
                    Point.SetValue(i, new gp_Pnt(x, y, z));
                }
                var anInterpolator = new GeomAPI_Interpolate(Point, false, 1.0e-5);
                anInterpolator.Perform();
                if (anInterpolator.IsDone())
                {
                    Geom_BSplineCurve C = anInterpolator.Curve();
                    DrawTrSurf.Set(theDI, a[1], C);
                }
            }
            else if (dimen == "2d")
            {
                var PointPtr = new NCollection_HArray1<gp_Pnt2d>(1, nbp);
                for (i = 1; i <= nbp; i++)
                {
                    GeomTools.GetReal(iFile, out x);
                    GeomTools.GetReal(iFile, out y);
                    PointPtr.SetValue(i, new gp_Pnt2d(x, y));
                }
                var a2dInterpolator = new Geom2dAPI_Interpolate(PointPtr, false, 1.0e-5);
                a2dInterpolator.Perform();
                if (a2dInterpolator.IsDone())
                {
                    Geom2d_BSplineCurve C = a2dInterpolator.Curve();
                    DrawTrSurf.Set(theDI, a[1], C);
                }
            }
        }
        return 0;
    }

    private static int tanginterpol(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }

        int ii, jj, num_tangents, num_read, num_start, num_parameters;

        double tolerance;

        bool periodic_flag = false;
        gp_Pnt a_point = new();
        gp_Vec a_vector = new();
        tolerance = 1.0e-5;

        Geom_BSplineCurve NewCurvePtr;

        num_read = 2;
        if (a[num_read] == "p")
        {
            periodic_flag = true;
            num_read += 1;
        }
        num_parameters = theDI.Atoi(a[num_read]);

        if (num_parameters < 2)
        {
            num_parameters = 2;
        }
        if (n < num_parameters * 3 + num_read)
        {
            return 1;
        }
        var PointsArrayPtr = new NCollection_HArray1<gp_Pnt>(1, num_parameters);

        num_tangents = ((n - num_read) / 3) - num_parameters;
        num_tangents = Math.Max(0, num_tangents);
        num_tangents = Math.Min(num_parameters, num_tangents);
        ii = 1;
        num_start = num_read;
        num_read += 1;
        while (num_read <= num_parameters * 3 + num_start)
        {
            for (jj = 1; jj <= 3; jj++)
            {
                a_point.SetCoord(jj, theDI.Atof(a[num_read]));
                num_read += 1;
            }
            PointsArrayPtr.SetValue(ii, a_point);
            ii += 1;
        }
        var anInterpolator = new GeomAPI_Interpolate(PointsArrayPtr, periodic_flag, tolerance);

        if (num_tangents > 0)
        {
            var TangentsArray = new NCollection_Array1<gp_Vec>(1, num_parameters);
            var TangentFlagsPtr = new NCollection_HArray1<bool>(1, num_parameters);

            for (ii = 1; ii <= num_tangents; ii++)
            {
                TangentFlagsPtr.SetValue(ii, true);
            }
            for (ii = num_tangents + 1; ii <= num_parameters; ii++)
            {
                TangentFlagsPtr.SetValue(ii, false);
            }
            ii = 1;
            while (ii <= num_tangents)
            {
                for (jj = 1; jj <= 3; jj++)
                {
                    a_vector.SetCoord(jj, theDI.Atof(a[num_read]));
                    num_read += 1;
                }
                TangentsArray.SetValue(ii, a_vector);
                ii += 1;
            }

            anInterpolator.Load(TangentsArray, TangentFlagsPtr);
        }
        anInterpolator.Perform();
        if (anInterpolator.IsDone())
        {
            NewCurvePtr = anInterpolator.Curve();

            DrawTrSurf.Set(theDI, a[1], NewCurvePtr);
            di.Append(a[2]).Append(" ");
        }
        return 0;
    }

    private static void ShowProjSolution(DrawInterpreter theDI, DrawResult di, int i, in gp_Pnt P, in gp_Pnt P1, double U, double V,
                                         bool isSurface)
    {
        string name = $"ext_{i}";
        di.Append(name).Append(" ");
        if (P.Distance(P1) > Precision.Confusion())
        {
            var L = new Geom_Line(P, new gp_Dir(new gp_Vec(P, P1)));
            var CT = new Geom_TrimmedCurve(L, 0.0, P.Distance(P1));
            DrawTrSurf.Set(theDI, name, CT);
        }
        else
        {
            DrawTrSurf.Set(theDI, name, P1);
            if (isSurface)
            {
                di.Append(" Point on surface ");
            }
            else
            {
                di.Append(" Point on curve ");
            }
        }
        if (isSurface)
        {
            di.Append(" Parameters: ").Append(U).Append(" ").Append(V).Append("\n");
        }
        else
        {
            di.Append(" parameter ").Append(i).Append(" = ").Append(U).Append("\n");
        }
    }

    // proj curve/surf x y z [{extrema algo: g(grad)/t(tree)}|{u v}]
    private static int Proj(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 5)
        {
            // Message::SendFail (decision 405)
            theDI.Output.WriteLine(" Use proj curve/surf x y z [{extrema algo: g(grad)/t(tree)}|{u v}]");
            return 1;
        }

        var P = new gp_Pnt(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]));

        Geom_Curve? GC = DrawTrSurf.GetCurve(theDI, a[1]);
        Geom_Surface? GS;
        Extrema_ExtAlgo aProjAlgo = Extrema_ExtAlgo.Extrema_ExtAlgo_Grad;

        if (n == 6 && a[5].Length > 0 && a[5][0] == 't')
        {
            aProjAlgo = Extrema_ExtAlgo.Extrema_ExtAlgo_Tree;
        }

        if (GC is null)
        {
            GS = DrawTrSurf.GetSurface(theDI, a[1]);

            if (GS is null)
            {
                return 1;
            }

            if (n <= 6)
            {
                GS.Bounds(out double U1, out double U2, out double V1, out double V2);

                var proj = new GeomAPI_ProjectPointOnSurf(P, GS, U1, U2, V1, V2, aProjAlgo);
                if (!proj.IsDone())
                {
                    di.Append("projection failed.");
                    return 0;
                }

                for (int i = 1; i <= proj.NbPoints(); i++)
                {
                    gp_Pnt P1 = proj.Point(i);
                    proj.Parameters(i, out double UU, out double VV);
                    ShowProjSolution(theDI, di, i, P, P1, UU, VV, true);
                }
            }
            else if (n == 7)
            {
                var aP2d = new gp_XY(theDI.Atof(a[5]), theDI.Atof(a[6]));
                var aGAS = new GeomAdaptor_Surface(GS);
                var aProjector = new Extrema_GenLocateExtPS(aGAS, Precision.PConfusion(), Precision.PConfusion());
                aProjector.Perform(P, aP2d.X(), aP2d.Y(), false);
                if (!aProjector.IsDone())
                {
                    di.Append("projection failed.");
                    return 0;
                }

                Extrema_POnSurf aP = aProjector.Point();
                aP.Parameter(out double UU, out double VV);
                ShowProjSolution(theDI, di, 1, P, aP.Value(), UU, VV, true);
            }
        }
        else
        {
            var proj = new GeomAPI_ProjectPointOnCurve(P, GC, GC.FirstParameter(), GC.LastParameter());

            if (proj.NbPoints() == 0)
            {
                // std::cout (decision 405)
                theDI.Output.Write("No project point was found.\n");
                return 0;
            }

            for (int i = 1; i <= proj.NbPoints(); i++)
            {
                gp_Pnt P1 = proj.Point(i);
                double UU = proj.Parameter(i);
                ShowProjSolution(theDI, di, i, P, P1, UU, UU, false);
            }
        }

        return 0;
    }

    private static int Extrema(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        Geom_Surface? GS1 = null, GS2 = null;
        bool isInfinitySolutions = false;
        double aMinDist = double.MaxValue;
        double U1f, U1l, U2f, U2l, V1f = 0.0, V1l = 0.0, V2f = 0.0, V2l = 0.0;

        Geom_Curve? GC1 = DrawTrSurf.GetCurve(theDI, a[1]);
        if (GC1 is null)
        {
            GS1 = DrawTrSurf.GetSurface(theDI, a[1]);
            if (GS1 is null)
            {
                return 1;
            }

            GS1.Bounds(out U1f, out U1l, out V1f, out V1l);
        }
        else
        {
            U1f = GC1.FirstParameter();
            U1l = GC1.LastParameter();
        }

        Geom_Curve? GC2 = DrawTrSurf.GetCurve(theDI, a[2]);
        if (GC2 is null)
        {
            GS2 = DrawTrSurf.GetSurface(theDI, a[2]);
            if (GS2 is null)
            {
                return 1;
            }

            GS2.Bounds(out U2f, out U2l, out V2f, out V2l);
        }
        else
        {
            U2f = GC2.FirstParameter();
            U2l = GC2.LastParameter();
        }

        var aPnts1 = new List<gp_Pnt>();
        var aPnts2 = new List<gp_Pnt>();
        List<double>[] aPrms = [[], [], [], []];
        if (GC1 is not null && GC2 is not null)
        {
            var Ex = new GeomAPI_ExtremaCurveCurve(GC1, GC2, U1f, U1l, U2f, U2l);
            isInfinitySolutions = Ex.Extrema().IsParallel();
            if (isInfinitySolutions)
            {
                aMinDist = Ex.LowerDistance();
            }
            else
            {
                for (int aJ = 1; aJ <= Ex.NbExtrema(); ++aJ)
                {
                    Ex.Points(aJ, out gp_Pnt aP1, out gp_Pnt aP2);
                    aPnts1.Add(aP1);
                    aPnts2.Add(aP2);
                    Ex.Parameters(aJ, out double aU1, out double aU2);
                    aPrms[0].Add(aU1);
                    aPrms[2].Add(aU2);
                }
            }
        }
        else if (GC1 is not null && GS2 is not null)
        {
            var Ex = new GeomAPI_ExtremaCurveSurface(GC1, GS2, U1f, U1l, U2f, U2l, V2f, V2l);
            isInfinitySolutions = Ex.Extrema().IsParallel();
            if (isInfinitySolutions)
            {
                aMinDist = Ex.LowerDistance();
            }
            else
            {
                for (int aJ = 1; aJ <= Ex.NbExtrema(); ++aJ)
                {
                    Ex.Points(aJ, out gp_Pnt aP1, out gp_Pnt aP2);
                    aPnts1.Add(aP1);
                    aPnts2.Add(aP2);
                    Ex.Parameters(aJ, out double aU1, out double aU2, out double aV2);
                    aPrms[0].Add(aU1);
                    aPrms[2].Add(aU2);
                    aPrms[3].Add(aV2);
                }
            }
        }
        else if (GS1 is not null && GC2 is not null)
        {
            var Ex = new GeomAPI_ExtremaCurveSurface(GC2, GS1, U2f, U2l, U1f, U1l, V1f, V1l);
            isInfinitySolutions = Ex.Extrema().IsParallel();
            if (isInfinitySolutions)
            {
                aMinDist = Ex.LowerDistance();
            }
            else
            {
                for (int aJ = 1; aJ <= Ex.NbExtrema(); ++aJ)
                {
                    Ex.Points(aJ, out gp_Pnt aP2, out gp_Pnt aP1);
                    aPnts1.Add(aP1);
                    aPnts2.Add(aP2);
                    Ex.Parameters(aJ, out double aU2, out double aU1, out double aV1);
                    aPrms[0].Add(aU1);
                    aPrms[1].Add(aV1);
                    aPrms[2].Add(aU2);
                }
            }
        }
        else if (GS1 is not null && GS2 is not null)
        {
            var Ex = new GeomAPI_ExtremaSurfaceSurface(GS1, GS2, U1f, U1l, V1f, V1l, U2f, U2l, V2f, V2l);
            isInfinitySolutions = Ex.Extrema().IsParallel();
            if (isInfinitySolutions)
            {
                aMinDist = Ex.LowerDistance();
            }
            else
            {
                for (int aJ = 1; aJ <= Ex.NbExtrema(); ++aJ)
                {
                    Ex.Points(aJ, out gp_Pnt aP1, out gp_Pnt aP2);
                    aPnts1.Add(aP1);
                    aPnts2.Add(aP2);
                    Ex.Parameters(aJ, out double aU1, out double aV1, out double aU2, out double aV2);
                    aPrms[0].Add(aU1);
                    aPrms[1].Add(aV1);
                    aPrms[2].Add(aU2);
                    aPrms[3].Add(aV2);
                }
            }
        }

        int aPntCount = aPnts1.Count;
        if (aPntCount == 0 || isInfinitySolutions)
        {
            // Infinity solutions flag may be set with 0 number of solutions in analytic extrema Curve/Curve
            if (isInfinitySolutions)
            {
                di.Append("Infinite number of extremas, distance = ").Append(aMinDist).Append("\n");
            }
            else
            {
                di.Append("No solutions!\n");
            }
        }

        for (int aJ = 1; aJ <= aPntCount; aJ++)
        {
            gp_Pnt aP1 = aPnts1[aJ - 1], aP2 = aPnts2[aJ - 1];
            if (aP1.Distance(aP2) < 1.0e-16)
            {
                di.Append("Extrema ").Append(aJ).Append(" is point : ").Append(aP1.X()).Append(" ").Append(aP1.Y()).Append(" ")
                  .Append(aP1.Z()).Append("\n");
                continue;
            }

            var aL = new Geom_Line(aP1, new gp_Dir(new gp_Vec(aP1, aP2)));
            var aCT = new Geom_TrimmedCurve(aL, 0.0, aP1.Distance(aP2));
            string aName = $"ext_{aJ}";
            DrawTrSurf.Set(theDI, aName, aCT);
            di.Append(aName).Append(" ");
        }

        if (n >= 4)
        {
            for (int aJ = 1; aJ <= aPntCount; aJ++)
            {
                DrawTrSurf.Set(theDI, $"ext_{aJ}_2", aPnts1[aJ - 1]);
                di.Append($"ext_{aJ}_2").Append(" ");
                DrawTrSurf.Set(theDI, $"ext_{aJ}_3", aPnts2[aJ - 1]);
                di.Append($"ext_{aJ}_3").Append(" ");
            }

            for (int aJ = 0; aJ < 4; ++aJ)
            {
                for (int aK = 0; aK < aPrms[aJ].Count; ++aK)
                {
                    string aName = $"prm_{aJ + 1}_{aK + 1}";
                    theDI.SetNumber(aName, aPrms[aJ][aK]);
                    di.Append(aName).Append(" ");
                }
            }
        }

        return 0;
    }

    // The three `solutions` helpers of GeometryTest_ConstraintCommands.cxx: each circle bound as name_i, the names
    // listed in the result.
    private static int Solutions(DrawInterpreter theDI, DrawResult di, bool theIsDone, Func<int> theNb, Func<int, gp_Circ2d> theSolution,
                                 string theName, string theNotDone)
    {
        if (theIsDone)
        {
            for (int i = 1; i <= theNb(); i++)
            {
                Geom2d_Circle C = new(theSolution(i));
                string solname = theName + "_" + i.ToString(CultureInfo.InvariantCulture);
                DrawTrSurf.Set(theDI, solname, C);
                di.Append(solname).Append(" ");
            }
            return 0;
        }
        else
        {
            di.Append(theNotDone);
            return 1;
        }
    }

    private static int Solutions(DrawInterpreter theDI, DrawResult di, Geom2dGcc_Circ2d2TanRad ct3, string name)
        => Solutions(theDI, di, ct3.IsDone(), ct3.NbSolutions, ct3.ThisSolution, name, "Circ2d2TanRad Not done");

    private static int Solutions(DrawInterpreter theDI, DrawResult di, Geom2dGcc_Circ2d3Tan ct3, string name)
        => Solutions(theDI, di, ct3.IsDone(), ct3.NbSolutions, ct3.ThisSolution, name, "Circ2d3Tan Not done");

    private static int Solutions(DrawInterpreter theDI, DrawResult theDIResult, Geom2dGcc_Circ2dTanCen theCt2, string theName)
        => Solutions(theDI, theDIResult, theCt2.IsDone(), theCt2.NbSolutions, theCt2.ThisSolution, theName, "Circ2dTanCen Not done");

    /// <summary>
    /// cirtang cname [-t Tolerance] -c curve -p point -r Radius...: circles tangent to up to three unqualified curves,
    /// through points, or of a radius (Geom2dGcc_Circ2d3Tan, Circ2d2TanRad, Circ2dTanCen), named cname_1, cname_2, ...
    /// An option at the end without its value reads an empty word, where OCCT reads Tcl's null argv[argc].
    /// </summary>
    private static int Cirtang(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVals)
    {
        int theNArgs = theArgVals.Count;
        if (theNArgs < 3)
        {
            theResult.Append("Use: ").Append(theArgVals[0]).Append("result [-t <Tolerance>] -c <curve> -p <point> -r <Radius>...\n");
            return 1;
        }

        string Word(int theI) => theI < theNArgs ? theArgVals[theI] : "";

        double aTol = Precision.Confusion();
        Geom2d_Curve?[] aC = new Geom2d_Curve?[3];
        gp_Pnt2d[] aP = [new(), new(), new()];
        double aRadius = -1.0;

        int aNbCurves = 0, aNbPnts = 0;

        for (int anArgID = 2; anArgID < theNArgs; anArgID++)
        {
            if (!theArgVals[anArgID].StartsWith('-'))
            {
                theResult.Append("Cannot interpret the argument #").Append(anArgID).Append(" (").Append(theArgVals[anArgID]).Append(")\n");
                return 1;
            }
            else if (theArgVals[anArgID] == "-c")
            {
                if (aNbCurves >= 3)
                {
                    theResult.Append("A lot of curves are given (not greater than 3 ones are expected)\n");
                    return 1;
                }

                aC[aNbCurves] = DrawTrSurf.GetCurve2d(theDI, Word(++anArgID));
                if (aC[aNbCurves] is null)
                {
                    theResult.Append("Error: ").Append(Word(anArgID)).Append(" is not a curve\n");
                    return 1;
                }

                aNbCurves++;
            }
            else if (theArgVals[anArgID] == "-p")
            {
                if (aNbPnts >= 3)
                {
                    theResult.Append("A lot of points are given (not greater than 3 ones are expected)\n");
                    return 1;
                }

                if (!DrawTrSurf.GetPoint2d(theDI, Word(++anArgID), ref aP[aNbPnts]))
                {
                    theResult.Append("Error: ").Append(Word(anArgID)).Append(" is not a point\n");
                    return 1;
                }

                aNbPnts++;
            }
            else if (theArgVals[anArgID] == "-r")
            {
                aRadius = theDI.Atof(Word(++anArgID));
            }
            else if (theArgVals[anArgID] == "-t")
            {
                aTol = theDI.Atof(Word(++anArgID));
            }
            else
            {
                theResult.Append("Unknown option ").Append(theArgVals[anArgID]).Append("\n");
                return 1;
            }
        }

        static Geom2dGcc_QualifiedCurve Unqualified(Geom2d_Curve? theC) => Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(theC!));

        if (aNbCurves == 3)
        {
            // C-C-C
            Geom2dGcc_Circ2d3Tan aCt3 = new(Unqualified(aC[0]), Unqualified(aC[1]), Unqualified(aC[2]), aTol, 0, 0, 0);
            theResult.Append("Solution of type C-C-C is: ");
            return Solutions(theDI, theResult, aCt3, theArgVals[1]);
        }
        else if (aNbCurves == 2)
        {
            if (aNbPnts >= 1)
            {
                // C-C-P
                Geom2dGcc_Circ2d3Tan aCt3 = new(Unqualified(aC[0]), Unqualified(aC[1]), new Geom2d_CartesianPoint(aP[0]), aTol, 0, 0);
                theResult.Append("Solution of type C-C-P is: ");
                return Solutions(theDI, theResult, aCt3, theArgVals[1]);
            }
            else if (aRadius > 0)
            {
                // C-C-R
                Geom2dGcc_Circ2d2TanRad aCt3 = new(Unqualified(aC[0]), Unqualified(aC[1]), aRadius, aTol);
                theResult.Append("Solution of type C-C-R is: ");
                return Solutions(theDI, theResult, aCt3, theArgVals[1]);
            }

            theResult.Append("Error: Unsupported set of input data!\n");
            return 1;
        }
        else if (aNbCurves == 1)
        {
            if (aNbPnts == 2)
            {
                // C-P-P
                Geom2dGcc_Circ2d3Tan aCt3 = new(Unqualified(aC[0]), new Geom2d_CartesianPoint(aP[0]), new Geom2d_CartesianPoint(aP[1]),
                                                aTol, 0);
                theResult.Append("Solution of type C-P-P is: ");
                return Solutions(theDI, theResult, aCt3, theArgVals[1]);
            }
            else if (aNbPnts == 1)
            {
                if (aRadius > 0.0)
                {
                    // C-P-R
                    Geom2dGcc_Circ2d2TanRad aCt3 = new(Unqualified(aC[0]), new Geom2d_CartesianPoint(aP[0]), aRadius, aTol);
                    theResult.Append("Solution of type C-P-R is: ");
                    return Solutions(theDI, theResult, aCt3, theArgVals[1]);
                }
                else
                {
                    // C-P
                    Geom2dGcc_Circ2dTanCen aCt2 = new(Unqualified(aC[0]), new Geom2d_CartesianPoint(aP[0]), aTol);
                    theResult.Append("Solution of type C-P is: ");
                    return Solutions(theDI, theResult, aCt2, theArgVals[1]);
                }
            }

            theResult.Append("Error: Unsupported set of input data!\n");
            return 1;
        }
        else if (aNbPnts >= 2)
        {
            if (aNbPnts == 3)
            {
                // P-P-P
                Geom2dGcc_Circ2d3Tan aCt3 = new(new Geom2d_CartesianPoint(aP[0]), new Geom2d_CartesianPoint(aP[1]),
                                                new Geom2d_CartesianPoint(aP[2]), aTol);
                theResult.Append("Solution of type P-P-P is: ");
                return Solutions(theDI, theResult, aCt3, theArgVals[1]);
            }
            else if (aRadius > 0)
            {
                // P-P-R
                Geom2dGcc_Circ2d2TanRad aCt3 = new(new Geom2d_CartesianPoint(aP[0]), new Geom2d_CartesianPoint(aP[1]), aRadius, aTol);
                theResult.Append("Solution of type P-P-R is: ");
                return Solutions(theDI, theResult, aCt3, theArgVals[1]);
            }

            theResult.Append("Error: Unsupported set of input data!\n");
            return 1;
        }

        theResult.Append("Error: Unsupported set of input data!\n");
        return 1;
    }

    /// <summary>
    /// lintan lname curve1 curve2 [angle]: the lines tangent to two curves (Geom2dGcc_Lin2d2Tan, started at the
    /// middles of their parameter ranges), or tangent to curve1 at an angle in degrees to the line curve2
    /// (Geom2dGcc_Lin2dTanObl), named lname_1, lname_2, ... "Not done" doesn't fail the command, as in OCCT.
    /// </summary>
    private static int Lintang(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }

        Geom2d_Curve? C1 = DrawTrSurf.GetCurve2d(theDI, a[2]);
        Geom2d_Curve? C2 = DrawTrSurf.GetCurve2d(theDI, a[3]);

        if (C1 is null || C2 is null)
        {
            return 1;
        }

        if (n >= 5)
        {
            if (C2 is not Geom2d_Line L)
            {
                di.Append("Second argument must be a line");
                return 1;
            }
            double                ang = theDI.Atof(a[4]) * (Math.PI / 180.0);
            Geom2dGcc_Lin2dTanObl ct3 = new(Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(C1)),
                                            L.Lin2d(),
                                            Precision.Angular(),
                                            (C1.FirstParameter() + C1.LastParameter()) / 2.0,
                                            ang);
            if (ct3.IsDone())
            {
                for (int i = 1; i <= ct3.NbSolutions(); i++)
                {
                    Geom2d_Line LS = new(ct3.ThisSolution(i));
                    string solname = a[1] + "_" + i.ToString(CultureInfo.InvariantCulture);
                    DrawTrSurf.Set(theDI, solname, LS);
                    di.Append(solname).Append(" ");
                }
            }
            else
            {
                di.Append("Lin2dTanObl Not done\n");
            }
        }
        else
        {
            Geom2dGcc_Lin2d2Tan ct3 = new(Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(C1)),
                                          Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(C2)),
                                          Precision.Angular(),
                                          (C1.FirstParameter() + C1.LastParameter()) / 2.0,
                                          (C2.FirstParameter() + C2.LastParameter()) / 2.0);
            if (ct3.IsDone())
            {
                for (int i = 1; i <= ct3.NbSolutions(); i++)
                {
                    Geom2d_Line LS = new(ct3.ThisSolution(i));
                    string solname = a[1] + "_" + i.ToString(CultureInfo.InvariantCulture);
                    DrawTrSurf.Set(theDI, solname, LS);
                    di.Append(solname).Append(" ");
                }
            }
            else
            {
                di.Append("Lin2d2Tan Not done\n");
            }
        }

        return 0;
    }

    /// <summary>
    /// gcarc name seg|cir p1 p2 [p3 [p4]]. OCCT returns 1 after binding the curve, so a successful gcarc is a Tcl
    /// error with an empty message (scripts call it inside <c>catch</c>); the usage is the result of a failure, with
    /// status 0. Both are kept.
    /// </summary>
    private static int GcArc(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n >= 5)
        {
            gp_Pnt P1 = new(), P2 = new(), P3 = new(), P4 = new();
            if (a[2] == "seg")
            {
                if (DrawTrSurf.GetPoint(theDI, a[3], ref P1) && DrawTrSurf.GetPoint(theDI, a[4], ref P2))
                {
                    DrawTrSurf.Set(theDI, a[1], new GC_MakeSegment(P1, P2).Value());
                    // OCCT
                    return 1;
                }
            }
            else if (a[2] == "cir")
            {
                // OCCT reads a[5] without checking the count: Tcl's null argv[argc], no point
                if (DrawTrSurf.GetPoint(theDI, a[3], ref P1) && DrawTrSurf.GetPoint(theDI, a[4], ref P2)
                    && DrawTrSurf.GetPoint(theDI, n > 5 ? a[5] : "", ref P3))
                {
                    if (n > 6)
                    {
                        DrawTrSurf.GetPoint(theDI, a[6], ref P4);
                        var V1 = new gp_Vec(P2, P3);
                        DrawTrSurf.Set(theDI, a[1], new GC_MakeArcOfCircle(P1, V1, P4).Value());
                        // OCCT
                        return 1;
                    }

                    DrawTrSurf.Set(theDI, a[1], new GC_MakeArcOfCircle(P1, P2, P3).Value());
                    // OCCT
                    return 1;
                }
            }
        }

        di.Append("give a name for arc and the type seg or cir then\n");
        di.Append("give passing points p1 p2 for seg    p1 p2 p3 or p1 p2 p3 p4 for cir (p2 p3 is a tgtvec)!\n");
        return 0;
    }
}
