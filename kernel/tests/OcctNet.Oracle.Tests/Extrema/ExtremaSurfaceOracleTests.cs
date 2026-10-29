#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using static Occt.Extrema_ExtAlgo;
using static Occt.Extrema_ExtFlag;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The surface extrema (unit 4A) against native OCCT on random surfaces of every kind built from the same raw numbers
/// (AdaptorOracleInputs' generators): Extrema_ExtPS (with Extrema_GenExtPS, ExtPExtS, ExtPRevS and ExtPElS under it)
/// through GeomAPI_ProjectPointOnSurf and BRepExtrema_DistShapeShape between a vertex and a face, Extrema_GenExtCS
/// through DistShapeShape between an edge and a face, and Extrema_ExtSS (with GenExtSS) through DistShapeShape between
/// two faces. The port's side mirrors each native call path.
/// </summary>
public class ExtremaSurfaceOracleTests(ITestOutputHelper output)
{
    // Runs the native and the port computations; both must throw, or neither.
    private static bool Both(Oracle theO, Action theNative, Action thePort, string theWhat)
    {
        Exception? aNativeError = null, aPortError = null;
        try
        {
            theNative();
        }
        catch (N.OcctException anE)
        {
            aNativeError = anE;
        }
        try
        {
            thePort();
        }
        catch (Standard_Failure anE)
        {
            aPortError = anE;
        }
        theO.Same(aNativeError is not null, aPortError is not null,
            $"{theWhat} throws (native: {aNativeError?.Message}; port: {aPortError?.Message})");
        return aNativeError is null;
    }

    // A finite sub-range of [theFirst, theLast] (a random window where the bound is infinite).
    private static (double, double) Window(Oracle theO, double theFirst, double theLast, bool theWhole)
    {
        if (Precision.IsInfinite(theFirst) || Precision.IsInfinite(theLast))
        {
            double a = theO.Real(-10, 5);
            return (a, a + theO.Real(1, 12));
        }
        if (theWhole)
        {
            return (theFirst, theLast);
        }
        double aD = theLast - theFirst;
        double u = theO.Real(theFirst, theFirst + 0.45 * aD);
        return (u, theO.Real(theFirst + 0.55 * aD, theLast));
    }

    // A point near the surface (on it, or at some distance from a point of it) or anywhere.
    private static gp_Pnt RandomPoint(Oracle theO, Geom_Surface theS, double theU1, double theU2, double theV1, double theV2)
    {
        int aKind = theO.Int(0, 4);
        if (aKind == 4)
        {
            return new gp_Pnt(theO.Real(-30, 30), theO.Real(-30, 30), theO.Real(-30, 30));
        }
        gp_Pnt aP = theS.Value(theO.Real(theU1, theU2), theO.Real(theV1, theV2));
        if (aKind == 0)
        {
            return aP;
        }
        double aScale = aKind == 1 ? theO.Real(1e-3, 1) : theO.Real(1, 20);
        return new gp_Pnt(aP.X() + aScale * theO.Real(-1, 1), aP.Y() + aScale * theO.Real(-1, 1),
            aP.Z() + aScale * theO.Real(-1, 1));
    }

    /// <summary>
    /// Extrema_ExtPS through GeomAPI_ProjectPointOnSurf, on every surface kind, whole (infinite bounds become ±1e10)
    /// or on a sub-range, with OCCT's default tolerances or a random one, the gradient or the tree algorithm, and the
    /// three flags (SetExtremaFlag). One projector is used for several points, so the grid or tree built for the first
    /// point is reused. GeomAPI loads a GeomAdaptor_Surface on the range and calls SetAlgo, Initialize and Perform on a
    /// default Extrema_ExtPS: the port's side does the same. Every extremum is compared: parameters, point, distance.
    /// </summary>
    [Fact]
    public void ExtPS_through_GeomAPI_ProjectPointOnSurf()
    {
        var anOracle = new Oracle(output, 20261101);
        int aCases = 0, anExtrema = 0;
        int[] aByKind = new int[AdaptorOracleInputs.SurfaceKinds.Length];
        for (int c = 0; c < 1100; c++)
        {
            int aKind = c % AdaptorOracleInputs.SurfaceKinds.Length;
            (N.Geom_Surface aNS, Geom_Surface aPS) = AdaptorOracleInputs.RandomSurface(anOracle, aKind);
            using (aNS)
            {
                aPS.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                bool aWhole = anOracle.Int(0, 2) == 0;
                (double aWU1, double aWU2) = Window(anOracle, aU1, aU2, aWhole);
                (double aWV1, double aWV2) = Window(anOracle, aV1, aV2, aWhole);
                bool aWithTol = anOracle.Bool();
                double aTol = aWithTol ? Math.Pow(10, anOracle.Real(-9, -4)) : 0.0;
                Extrema_ExtAlgo anAlgo = anOracle.Int(0, 3) == 0 ? Extrema_ExtAlgo_Tree : Extrema_ExtAlgo_Grad;
                Extrema_ExtFlag aFlag = anOracle.Int(0, 3) switch
                {
                    0 => Extrema_ExtFlag_MIN,
                    1 => Extrema_ExtFlag_MAX,
                    _ => Extrema_ExtFlag_MINMAX,
                };
                string aW = $"{AdaptorOracleInputs.SurfaceKinds[aKind]} #{c} ({(aWhole ? "whole" : "range")}, {anAlgo}, {aFlag}, tol {aTol:R})";

                using var aN = new N.GeomAPI_ProjectPointOnSurf();
                aN.SetExtremaFlag((N.Extrema_ExtFlag)(int)aFlag);
                var aNAlgo = (N.Extrema_ExtAlgo)(int)anAlgo;

                // The port's side of GeomAPI_ProjectPointOnSurf::Init
                var anAdaptor = new GeomAdaptor_Surface();
                var aPort = new Extrema_ExtPS();
                aPort.SetFlag(aFlag);
                double aLU1 = aWhole ? aU1 : aWU1, aLU2 = aWhole ? aU2 : aWU2;
                double aLV1 = aWhole ? aV1 : aWV1, aLV2 = aWhole ? aV2 : aWV2;
                double aPortTol = aWithTol ? aTol : (aWhole ? Precision.Confusion() : Precision.PConfusion());

                int aNbPoints = anOracle.Int(1, 3);
                for (int p = 0; p < aNbPoints; p++)
                {
                    gp_Pnt aP = RandomPoint(anOracle, aPS, aWU1, aWU2, aWV1, aWV2);
                    using N.gp_Pnt aNP = Oracle.ToNative(aP);
                    string aWP = $"{aW} point {p} {aP}";
                    bool anOk = Both(anOracle,
                        () =>
                        {
                            if (p > 0)
                            {
                                aN.Perform(aNP);
                            }
                            else if (aWhole && aWithTol)
                            {
                                aN.Init(aNP, aNS, aTol, aNAlgo);
                            }
                            else if (aWhole)
                            {
                                aN.Init(aNP, aNS, aNAlgo);
                            }
                            else if (aWithTol)
                            {
                                aN.Init(aNP, aNS, aWU1, aWU2, aWV1, aWV2, aTol, aNAlgo);
                            }
                            else
                            {
                                aN.Init(aNP, aNS, aWU1, aWU2, aWV1, aWV2, aNAlgo);
                            }
                        },
                        () =>
                        {
                            if (p == 0)
                            {
                                anAdaptor.Load(aPS, aLU1, aLU2, aLV1, aLV2);
                                aPort.SetAlgo(anAlgo);
                                aPort.Initialize(anAdaptor, aLU1, aLU2, aLV1, aLV2, aPortTol, aPortTol);
                            }
                            aPort.Perform(aP);
                        },
                        aWP);
                    if (!anOk)
                    {
                        break;
                    }
                    aCases++;
                    bool aPortDone = aPort.IsDone() && aPort.NbExt() > 0;
                    anOracle.Same(aN.IsDone(), aPortDone, $"{aWP} IsDone");
                    int aNb = aN.NbPoints();
                    anOracle.Same(aNb, aPortDone ? aPort.NbExt() : 0, $"{aWP} NbPoints");
                    for (int i = 1; i <= aNb; i++)
                    {
                        double aNU = 0.0, aNV = 0.0;
                        aN.Parameters(i, ref aNU, ref aNV);
                        Extrema_POnSurf aPOnS = aPort.Point(i);
                        aPOnS.Parameter(out double aU, out double aV);
                        anOracle.Same(aNU, aU, $"{aWP} extremum {i} U");
                        anOracle.Same(aNV, aV, $"{aWP} extremum {i} V");
                        using N.gp_Pnt aNPi = aN.Point(i);
                        anOracle.Same(aNPi, aPOnS.Value(), $"{aWP} extremum {i} point");
                        anOracle.Same(aN.Distance(i), Math.Sqrt(aPort.SquareDistance(i)), $"{aWP} extremum {i} distance");
                        anExtrema++;
                        aByKind[aKind]++;
                    }
                }
            }
        }
        output.WriteLine($"{aCases} projections, {anExtrema} extrema compared");
        for (int k = 0; k < aByKind.Length; k++)
        {
            output.WriteLine($"  {AdaptorOracleInputs.SurfaceKinds[k]}: {aByKind[k]} extrema");
            Assert.True(aByKind[k] > 0, $"no extremum compared on {AdaptorOracleInputs.SurfaceKinds[k]}");
        }
        anOracle.Report("Extrema_ExtPS through GeomAPI_ProjectPointOnSurf");
    }

    // A face on a random untrimmed surface (BRepLib_MakeFace would replace a trimmed surface by its basis) over a
    // window of its parameters; null when MakeFace fails.
    private static (N.TopoDS_Face Face, Geom_Surface Surface, string Kind)? RandomFace(Oracle theO, int theKind)
    {
        (N.Geom_Surface aNS, Geom_Surface aPS) = AdaptorOracleInputs.RandomSurface(theO, theKind);
        using (aNS)
        {
            aPS.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
            bool aWhole = theO.Int(0, 3) == 0;
            (double aWU1, double aWU2) = Window(theO, aU1, aU2, aWhole);
            (double aWV1, double aWV2) = Window(theO, aV1, aV2, aWhole);
            N.TopoDS_Face? aFace = MakeFace(aNS, aWU1, aWU2, aWV1, aWV2);
            if (aFace is null)
            {
                return null;
            }
            return (aFace, StoredSurface(aPS, aWU1, aWU2, aWV1, aWV2), AdaptorOracleInputs.SurfaceKinds[theKind]);
        }
    }

    // DistShapeShape's Value() is the least distance of all its solutions (within its epsilon); a solution's own is
    // the distance between its points, which the port's stored square distance must equal.
    private static void SameDistance(Oracle theO, N.BRepExtrema_DistShapeShape theN, N.gp_Pnt theNP1, N.gp_Pnt theNP2,
        double thePortSqDist, string theWhat)
    {
        theO.Same(Oracle.FromNative(theNP1).SquareDistance(Oracle.FromNative(theNP2)), thePortSqDist,
            $"{theWhat} square distance");
        Assert.True(Math.Abs(Math.Sqrt(thePortSqDist) - theN.Value()) < 1e-7, $"{theWhat} distance");
    }

    // BRepBuilderAPI_MakeFace(S, UMin, UMax, VMin, VMax, Confusion), or null when it fails or throws (offset faces go
    // through BRepLib::SameParameter, outside this unit).
    private static N.TopoDS_Face? MakeFace(N.Geom_Surface theS, double theU1, double theU2, double theV1, double theV2)
    {
        try
        {
            using var aMF = new N.BRepBuilderAPI_MakeFace(theS, theU1, theU2, theV1, theV2, Precision.Confusion());
            return aMF.IsDone() ? aMF.Face() : null;
        }
        catch (N.OcctException)
        {
            return null;
        }
    }

    // The surface BRepLib_MakeFace stores for theS: an offset of a swept surface is trimmed (on both directions when
    // the offset's U bounds are infinite, else in V for an extrusion; in V for a revolution with infinite V bounds).
    private static Geom_Surface StoredSurface(Geom_Surface theS, double theU1, double theU2, double theV1, double theV2)
    {
        if (theS is Geom_OffsetSurface anOS)
        {
            anOS.Bounds(out double umin, out double umax, out double vmin, out double vmax);
            Geom_Surface aBase = anOS.BasisSurface();
            if (aBase is Geom_SurfaceOfLinearExtrusion)
            {
                return Precision.IsInfinite(umin) || Precision.IsInfinite(umax)
                    ? new Geom_RectangularTrimmedSurface(anOS, theU1, theU2, theV1, theV2)
                    : new Geom_RectangularTrimmedSurface(anOS, theV1, theV2, false);
            }
            if (aBase is Geom_SurfaceOfRevolution && (Precision.IsInfinite(vmin) || Precision.IsInfinite(vmax)))
            {
                return new Geom_RectangularTrimmedSurface(anOS, theV1, theV2, false);
            }
        }
        return theS;
    }

    private static (double U1, double U2, double V1, double V2) UVBounds(N.TopoDS_Face theF)
    {
        double aU1 = 0, aU2 = 0, aV1 = 0, aV2 = 0;
        N.BRepTools.UVBounds(theF, ref aU1, ref aU2, ref aV1, ref aV2);
        return (aU1, aU2, aV1, aV2);
    }

    /// <summary>
    /// Extrema_ExtPS through BRepExtrema_DistShapeShape between a vertex and a face. BRepExtrema_ExtPF loads the face's
    /// whole surface (BRepAdaptor_Surface without restriction), sets the tolerances max(Resolution(min(face tolerance,
    /// Confusion)), PConfusion) and initialises Extrema_ExtPS on the face's UV bounds with DistShapeShape's flag
    /// (MINMAX) and algorithm (Grad); the port is driven the same way, with BRepTools::UVBounds and the tolerance
    /// read from the native face. Each native solution inside the face is one of Extrema_ExtPS's extrema, unchanged:
    /// it must be one of the port's (parameters, point, distance).
    /// </summary>
    [Fact]
    public void ExtPS_through_BRepExtrema_DistShapeShape_VertexFace()
    {
        var anOracle = new Oracle(output, 20261102);
        int aCompared = 0, aCases = 0;
        for (int c = 0; c < 600; c++)
        {
            int aKind = c % 10;
            var aFace = RandomFace(anOracle, aKind);
            if (aFace is null)
            {
                continue;
            }
            (N.TopoDS_Face aNF, Geom_Surface aPS, string aKindName) = aFace.Value;
            using (aNF)
            {
                (double aU1, double aU2, double aV1, double aV2) = UVBounds(aNF);
                gp_Pnt aP = RandomPoint(anOracle, aPS, aU1, aU2, aV1, aV2);
                string aW = $"{aKindName} #{c} [{aU1:R}, {aU2:R}] x [{aV1:R}, {aV2:R}], point {aP}";
                using N.gp_Pnt aNP = Oracle.ToNative(aP);
                using var aMV = new N.BRepBuilderAPI_MakeVertex(aNP);
                using N.TopoDS_Vertex aNV = aMV.Vertex();
                using var aN = new N.BRepExtrema_DistShapeShape(aNV, aNF);

                // The port's side of BRepExtrema_ExtPF(V, F, MINMAX, Grad)
                var aSurf = new GeomAdaptor_TransformedSurface(aPS, new gp_Trsf()); // BRepAdaptor_Surface
                double aTol = Math.Min(N.BRep_Tool.Tolerance(aNF), Precision.Confusion());
                double aTolU = Math.Max(aSurf.UResolution(aTol), Precision.PConfusion());
                double aTolV = Math.Max(aSurf.VResolution(aTol), Precision.PConfusion());
                var aPort = new Extrema_ExtPS();
                aPort.SetFlag(Extrema_ExtFlag_MINMAX);
                aPort.SetAlgo(Extrema_ExtAlgo_Grad);
                aPort.Initialize(aSurf, aU1, aU2, aV1, aV2, aTolU, aTolV);
                aPort.Perform(aP);
                aCases++;

                if (!aN.IsDone())
                {
                    continue;
                }
                int aNbPort = aPort.IsDone() ? aPort.NbExt() : 0;
                for (int i = 1; i <= aN.NbSolution(); i++)
                {
                    if (aN.SupportTypeShape2(i) != N.BRepExtrema_SupportType.BRepExtrema_IsInFace)
                    {
                        continue;
                    }
                    double aNU = 0.0, aNV2 = 0.0;
                    aN.ParOnFaceS2(i, ref aNU, ref aNV2);
                    int aMatch = 0;
                    double aBest = double.MaxValue;
                    for (int j = 1; j <= aNbPort; j++)
                    {
                        aPort.Point(j).Parameter(out double aU, out double aV);
                        double aD = Math.Abs(aU - aNU) + Math.Abs(aV - aNV2);
                        if (aD < aBest)
                        {
                            aBest = aD;
                            aMatch = j;
                        }
                    }
                    Assert.True(aMatch != 0 && aBest < 1e-6,
                        $"{aW}: native solution ({aNU:R}, {aNV2:R}) is not an extremum of the port's Extrema_ExtPS");
                    Extrema_POnSurf aQ = aPort.Point(aMatch);
                    aQ.Parameter(out double aQU, out double aQV);
                    anOracle.Same(aNU, aQU, $"{aW} U");
                    anOracle.Same(aNV2, aQV, $"{aW} V");
                    using N.gp_Pnt aNP2 = aN.PointOnShape2(i);
                    anOracle.Same(aNP2, aQ.Value(), $"{aW} point");
                    SameDistance(anOracle, aN, aNP, aNP2, aPort.SquareDistance(aMatch), aW);
                    aCompared++;
                }
            }
        }
        output.WriteLine($"{aCases} vertex/face pairs, {aCompared} solutions in faces compared");
        Assert.True(aCompared > aCases / 3, "too few solutions compared");
        anOracle.Report("Extrema_ExtPS through BRepExtrema_DistShapeShape (vertex/face)");
    }

    /// <summary>
    /// Extrema_GenExtCS through BRepExtrema_DistShapeShape between an edge on a Bezier or B-spline curve (smooth: no C1
    /// break, so Extrema_ExtCS adds no sharp points) and a face. BRepExtrema_ExtCF loads the face's surface on its UV
    /// bounds and initialises Extrema_ExtCS with the tolerances max(Resolution(min(tolerance, Confusion)),
    /// PConfusion); for these curves Extrema_ExtCS calls Extrema_GenExtCS (Initialize(S, 10, 10, TolS), Perform(C, 13
    /// for a B-spline and 12 for a Bezier curve, TolC)) and keeps each extremum inside the ranges after putting its
    /// surface parameters in their periods (AddSolution). The port's side does the same; each native solution inside
    /// the edge and the face must be one of the port's.
    /// </summary>
    [Fact]
    public void GenExtCS_through_BRepExtrema_DistShapeShape_EdgeFace()
    {
        var anOracle = new Oracle(output, 20261103);
        int aCompared = 0, aCases = 0;
        for (int c = 0; c < 400; c++)
        {
            int aKind = c % 10;
            var aFace = RandomFace(anOracle, aKind);
            if (aFace is null)
            {
                continue;
            }
            (N.TopoDS_Face aNF, Geom_Surface aPS, string aKindName) = aFace.Value;
            N.Geom_Curve aNC;
            Geom_Curve aPC;
            bool aBezier = anOracle.Bool();
            if (aBezier)
            {
                (N.Geom_BezierCurve aNB, Geom_BezierCurve aPB) = AdaptorOracleInputs.RandomBezierCurve(anOracle, 5);
                (aNC, aPC) = (aNB, aPB);
            }
            else
            {
                (N.Geom_BSplineCurve aNB, Geom_BSplineCurve aPB) =
                    AdaptorOracleInputs.RandomBSplineCurve(anOracle, false, 2, 5, 4);
                (aNC, aPC) = (aNB, aPB);
            }
            using (aNF)
            using (aNC)
            {
                (double aU1, double aU2, double aV1, double aV2) = UVBounds(aNF);
                double aF = aPC.FirstParameter(), aL = aPC.LastParameter();
                if (anOracle.Bool())
                {
                    double a = anOracle.Real(aF, aF + 0.4 * (aL - aF));
                    aL = anOracle.Real(aF + 0.6 * (aL - aF), aL);
                    aF = a;
                }
                // Move the curve so that it passes near (or through) a point of the face
                gp_Pnt aOnS = aPS.Value(anOracle.Real(aU1, aU2), anOracle.Real(aV1, aV2));
                gp_Pnt aOnC = aPC.Value(anOracle.Real(aF, aL));
                double aGap = anOracle.Bool() ? 0.0 : anOracle.Real(0.01, 3.0);
                gp_Vec aMove = new(aOnC, aOnS);
                aMove += new gp_Vec(aGap * anOracle.Real(-1, 1), aGap * anOracle.Real(-1, 1), aGap * anOracle.Real(-1, 1));
                using (N.gp_Vec aNMove = Oracle.ToNative(aMove))
                {
                    aNC.Translate(aNMove);
                }
                aPC.Translate(aMove);
                string aW = $"{(aBezier ? "Bezier" : "B-spline")} [{aF:R}, {aL:R}] / {aKindName} #{c}";

                // The port's side of BRepExtrema_ExtCF(E, F) and Extrema_ExtCS's generic path
                var aCurve = new GeomAdaptor_Curve(aPC, aF, aL);
                if (aCurve.NbIntervals(GeomAbs_Shape.GeomAbs_C1) != 1)
                {
                    continue;
                }
                using var aME = new N.BRepBuilderAPI_MakeEdge(aNC, aF, aL);
                if (!aME.IsDone())
                {
                    continue;
                }
                using N.TopoDS_Edge aNE = aME.Edge();
                using var aN = new N.BRepExtrema_DistShapeShape(aNE, aNF);

                var aSurf = new GeomAdaptor_TransformedSurface(aPS, aU1, aU2, aV1, aV2, new gp_Trsf()); // BRepAdaptor_Surface
                double aTolS = Math.Min(N.BRep_Tool.Tolerance(aNF), Precision.Confusion());
                aTolS = Math.Min(aSurf.UResolution(aTolS), aSurf.VResolution(aTolS));
                aTolS = Math.Max(aTolS, Precision.PConfusion());
                double aTolC = Math.Min(N.BRep_Tool.Tolerance(aNE), Precision.Confusion());
                aTolC = aCurve.Resolution(aTolC);
                aTolC = Math.Max(aTolC, Precision.PConfusion());
                var aGen = new Extrema_GenExtCS();
                aGen.Initialize(aSurf, 10, 10, aTolS);
                aGen.Perform(aCurve, aBezier ? 12 : 13, aTolC);
                aCases++;
                List<(double T, double U, double V, gp_Pnt PC, gp_Pnt PS, double D2)> aSols = [];
                if (aGen.IsDone())
                {
                    for (int j = 1; j <= aGen.NbExt(); j++)
                    {
                        Extrema_POnCurv aPOnC = aGen.PointOnCurve(j);
                        Extrema_POnSurf aPOnS = aGen.PointOnSurface(j);
                        aPOnS.Parameter(out double aU, out double aV);
                        // Extrema_ExtCS::AddSolution (the curve is not periodic)
                        if (aSurf.IsUPeriodic())
                        {
                            aU = ElCLib.InPeriod(aU, aU1, aU1 + aSurf.UPeriod());
                        }
                        if (aSurf.IsVPeriodic())
                        {
                            aV = ElCLib.InPeriod(aV, aV1, aV1 + aSurf.VPeriod());
                        }
                        aSols.Add((aPOnC.Parameter(), aU, aV, aPOnC.Value(), aPOnS.Value(), aGen.SquareDistance(j)));
                    }
                }

                if (!aN.IsDone())
                {
                    continue;
                }
                for (int i = 1; i <= aN.NbSolution(); i++)
                {
                    if (aN.SupportTypeShape1(i) != N.BRepExtrema_SupportType.BRepExtrema_IsOnEdge
                        || aN.SupportTypeShape2(i) != N.BRepExtrema_SupportType.BRepExtrema_IsInFace)
                    {
                        continue;
                    }
                    double aNT = 0.0, aNU = 0.0, aNV = 0.0;
                    aN.ParOnEdgeS1(i, ref aNT);
                    aN.ParOnFaceS2(i, ref aNU, ref aNV);
                    int aMatch = -1;
                    double aBest = double.MaxValue;
                    for (int j = 0; j < aSols.Count; j++)
                    {
                        double aD = Math.Abs(aSols[j].T - aNT) + Math.Abs(aSols[j].U - aNU) + Math.Abs(aSols[j].V - aNV);
                        if (aD < aBest)
                        {
                            aBest = aD;
                            aMatch = j;
                        }
                    }
                    Assert.True(aMatch >= 0 && aBest < 1e-6,
                        $"{aW}: native solution ({aNT:R}; {aNU:R}, {aNV:R}) is not an extremum of the port's Extrema_GenExtCS");
                    var aQ = aSols[aMatch];
                    anOracle.Same(aNT, aQ.T, $"{aW} T");
                    anOracle.Same(aNU, aQ.U, $"{aW} U");
                    anOracle.Same(aNV, aQ.V, $"{aW} V");
                    using N.gp_Pnt aNP1 = aN.PointOnShape1(i);
                    using N.gp_Pnt aNP2 = aN.PointOnShape2(i);
                    anOracle.Same(aNP1, aQ.PC, $"{aW} point on edge");
                    anOracle.Same(aNP2, aQ.PS, $"{aW} point on face");
                    SameDistance(anOracle, aN, aNP1, aNP2, aQ.D2, aW);
                    aCompared++;
                }
            }
        }
        output.WriteLine($"{aCases} edge/face pairs, {aCompared} solutions inside the edge and the face compared");
        Assert.True(aCompared > aCases / 5, "too few solutions compared");
        anOracle.Report("Extrema_GenExtCS through BRepExtrema_DistShapeShape (edge/face)");
    }

    /// <summary>
    /// Extrema_ExtSS through BRepExtrema_DistShapeShape between two faces. BRepExtrema_ExtFF initialises Extrema_ExtSS
    /// with face 2 (its surface on its UV bounds, tolerance max(min(UResolution, VResolution)(min(tolerance,
    /// Confusion)), PConfusion)), then performs it with face 1 the same way; the port is driven the same way. Each
    /// native solution inside both faces is one of Extrema_ExtSS's extrema, unchanged, and must be one of the port's.
    /// </summary>
    [Fact]
    public void ExtSS_through_BRepExtrema_DistShapeShape_FaceFace()
    {
        var anOracle = new Oracle(output, 20261104);
        int aCompared = 0, aCases = 0;
        for (int c = 0; c < 600; c++)
        {
            var aFace1 = RandomFace(anOracle, c % 10);
            if (aFace1 is null)
            {
                continue;
            }
            (N.TopoDS_Face aNF1, Geom_Surface aPS1, string aKind1) = aFace1.Value;
            using (aNF1)
            {
                (double aU11, double aU12, double aV11, double aV12) = UVBounds(aNF1);
                // Surface 2, moved near a point of face 1 before its face is made
                int aKind2 = anOracle.Int(0, 9);
                (N.Geom_Surface aNS2, Geom_Surface aPS2) = AdaptorOracleInputs.RandomSurface(anOracle, aKind2);
                using (aNS2)
                {
                    aPS2.Bounds(out double aB1, out double aB2, out double aB3, out double aB4);
                    (double aWU1, double aWU2) = Window(anOracle, aB1, aB2, false);
                    (double aWV1, double aWV2) = Window(anOracle, aB3, aB4, false);
                    double aS1U = anOracle.Real(aU11, aU12), aS1V = anOracle.Real(aV11, aV12);
                    double aS2U = anOracle.Real(aWU1, aWU2), aS2V = anOracle.Real(aWV1, aWV2);
                    aPS1.D1(aS1U, aS1V, out gp_Pnt aOn1, out gp_Vec aD1U1, out gp_Vec aD1V1);
                    aPS2.D1(aS2U, aS2V, out gp_Pnt aOn2, out gp_Vec aD1U2, out gp_Vec aD1V2);
                    gp_Vec aN1 = aD1U1 ^ aD1V1, aN2 = aD1U2 ^ aD1V2;
                    double aGap = anOracle.Real(0.01, 3.0);
                    gp_Vec aMove;
                    if (anOracle.Int(0, 3) != 0 && aN1.Magnitude() > 1e-6 && aN2.Magnitude() > 1e-6)
                    {
                        // Turn surface 2 so that its normal at aOn2 is opposite to surface 1's at aOn1, and put aOn2 on
                        // that normal: the two points are then a local extremum of the distance, often inside both faces
                        aN1.Normalize();
                        aN2.Normalize();
                        gp_Vec anAxis = aN2 ^ aN1.Reversed();
                        if (anAxis.Magnitude() > 1e-6)
                        {
                            double anAngle = aN2.Angle(aN1.Reversed());
                            var anAx1 = new gp_Ax1(aOn2, new gp_Dir(anAxis.X(), anAxis.Y(), anAxis.Z()));
                            using var aNOn2 = new N.gp_Pnt(aOn2.X(), aOn2.Y(), aOn2.Z());
                            using var aNDir = new N.gp_Dir(anAxis.X(), anAxis.Y(), anAxis.Z());
                            using var aNAx1 = new N.gp_Ax1(aNOn2, aNDir);
                            aNS2.Rotate(aNAx1, anAngle);
                            aPS2.Rotate(anAx1, anAngle);
                        }
                        aMove = new gp_Vec(aOn2, aOn1) + aGap * aN1;
                    }
                    else
                    {
                        aMove = new gp_Vec(aOn2, aOn1);
                        aMove += new gp_Vec(aGap * anOracle.Real(-1, 1), aGap * anOracle.Real(-1, 1), aGap * anOracle.Real(-1, 1));
                    }
                    using (N.gp_Vec aNMove = Oracle.ToNative(aMove))
                    {
                        aNS2.Translate(aNMove);
                    }
                    aPS2.Translate(aMove);
                    N.TopoDS_Face? aNF2OrNull = MakeFace(aNS2, aWU1, aWU2, aWV1, aWV2);
                    if (aNF2OrNull is null)
                    {
                        continue;
                    }
                    using N.TopoDS_Face aNF2 = aNF2OrNull;
                    aPS2 = StoredSurface(aPS2, aWU1, aWU2, aWV1, aWV2);
                    (double aU21, double aU22, double aV21, double aV22) = UVBounds(aNF2);
                    string aW = $"{aKind1} / {AdaptorOracleInputs.SurfaceKinds[aKind2]} #{c}";
                    N.BRepExtrema_DistShapeShape aN;
                    try
                    {
                        aN = new N.BRepExtrema_DistShapeShape(aNF1, aNF2);
                    }
                    catch (N.OcctException anE)
                    {
                        // In code outside this unit (bounding boxes of the faces, ...)
                        output.WriteLine($"{aW}: native DistShapeShape throws {anE.Message}, skipped");
                        continue;
                    }
                    using var aNOwner = aN;

                    // The port's side of BRepExtrema_ExtFF(F1, F2)
                    var aSurf2 = new GeomAdaptor_TransformedSurface(aPS2, aU21, aU22, aV21, aV22, new gp_Trsf()); // BRepAdaptor_Surface
                    double aTol2 = Math.Min(N.BRep_Tool.Tolerance(aNF2), Precision.Confusion());
                    aTol2 = Math.Min(aSurf2.UResolution(aTol2), aSurf2.VResolution(aTol2));
                    aTol2 = Math.Max(aTol2, Precision.PConfusion());
                    var aPort = new Extrema_ExtSS();
                    aPort.Initialize(aSurf2, aU21, aU22, aV21, aV22, aTol2);
                    var aSurf1 = new GeomAdaptor_TransformedSurface(aPS1, aU11, aU12, aV11, aV12, new gp_Trsf());
                    double aTol1 = Math.Min(N.BRep_Tool.Tolerance(aNF1), Precision.Confusion());
                    aTol1 = Math.Min(aSurf1.UResolution(aTol1), aSurf1.VResolution(aTol1));
                    aTol1 = Math.Max(aTol1, Precision.PConfusion());
                    aPort.Perform(aSurf1, aU11, aU12, aV11, aV12, aTol1);
                    aCases++;

                    if (!aN.IsDone())
                    {
                        continue;
                    }
                    int aNbPort = aPort.IsDone() && !aPort.IsParallel() ? aPort.NbExt() : 0;
                    for (int i = 1; i <= aN.NbSolution(); i++)
                    {
                        if (aN.SupportTypeShape1(i) != N.BRepExtrema_SupportType.BRepExtrema_IsInFace
                            || aN.SupportTypeShape2(i) != N.BRepExtrema_SupportType.BRepExtrema_IsInFace)
                        {
                            continue;
                        }
                        double aNU1 = 0, aNV1 = 0, aNU2 = 0, aNV2 = 0;
                        aN.ParOnFaceS1(i, ref aNU1, ref aNV1);
                        aN.ParOnFaceS2(i, ref aNU2, ref aNV2);
                        int aMatch = 0;
                        double aBest = double.MaxValue;
                        for (int j = 1; j <= aNbPort; j++)
                        {
                            aPort.Points(j, out Extrema_POnSurf aP1, out Extrema_POnSurf aP2);
                            aP1.Parameter(out double u1, out double v1);
                            aP2.Parameter(out double u2, out double v2);
                            double aD = Math.Abs(u1 - aNU1) + Math.Abs(v1 - aNV1) + Math.Abs(u2 - aNU2) + Math.Abs(v2 - aNV2);
                            if (aD < aBest)
                            {
                                aBest = aD;
                                aMatch = j;
                            }
                        }
                        Assert.True(aMatch != 0 && aBest < 1e-6,
                            $"{aW}: native solution ({aNU1:R}, {aNV1:R}; {aNU2:R}, {aNV2:R}) is not an extremum of the port's Extrema_ExtSS");
                        aPort.Points(aMatch, out Extrema_POnSurf aQ1, out Extrema_POnSurf aQ2);
                        aQ1.Parameter(out double aQU1, out double aQV1);
                        aQ2.Parameter(out double aQU2, out double aQV2);
                        anOracle.Same(aNU1, aQU1, $"{aW} U1");
                        anOracle.Same(aNV1, aQV1, $"{aW} V1");
                        anOracle.Same(aNU2, aQU2, $"{aW} U2");
                        anOracle.Same(aNV2, aQV2, $"{aW} V2");
                        using N.gp_Pnt aNP1 = aN.PointOnShape1(i);
                        using N.gp_Pnt aNP2 = aN.PointOnShape2(i);
                        anOracle.Same(aNP1, aQ1.Value(), $"{aW} point on 1");
                        anOracle.Same(aNP2, aQ2.Value(), $"{aW} point on 2");
                        SameDistance(anOracle, aN, aNP1, aNP2, aPort.SquareDistance(aMatch), aW);
                        aCompared++;
                    }
                }
            }
        }
        output.WriteLine($"{aCases} face pairs, {aCompared} solutions inside both faces compared");
        Assert.True(aCompared > 30, "too few solutions compared");
        anOracle.Report("Extrema_ExtSS through BRepExtrema_DistShapeShape (face/face)");
    }
}
#endif
