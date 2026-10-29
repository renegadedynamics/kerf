// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// GeometryTest_ConstraintCommands.cxx's <c>gcarc</c> (segments and arcs of circles through points, GC) and
/// GeometryTest_APICommands.cxx's <c>extrema</c> (GeomAPI_ExtremaCurveCurve, CurveSurface, SurfaceSurface).
/// </summary>
internal static class GeometryTestCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "gcarc", GcArc);
        DrawCxx.Add(theInterpreter, "extrema", Extrema);
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
