// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Api;
using Kerf.Geometry;
using Xunit.Abstractions;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// Geom2dAPI_InterCurveCurve (and through it Geom2dInt_GInter, IntCurve and IntImpParGen) and
/// Geom2dAPI_ProjectPointOnCurve against native OCCT, on random lines, segments, circles, arcs, ellipses, elliptic
/// arcs and B-splines (whole or trimmed; rational, periodic, with C0 breaks), built on both sides from the same raw
/// numbers. Second curves are drawn independently, or through, tangent to or along a point of the first curve, so
/// that crossings, tangencies and overlaps all occur.
/// </summary>
/// <remarks>
/// The native bindings expose only NbPoints, Point and NbSegments of Geom2dAPI_InterCurveCurve: the parameters and
/// transitions of the intersection points are compared through the points (each one evaluated on the curve at its
/// parameter). Native Geom2d has no Bezier, parabola or hyperbola bindings; the unit tests cover those.
/// </remarks>
public class Geom2dAPIOracleTests(ITestOutputHelper output)
{
    private const int Pairs = 1500;

    /// <summary>A 2D curve definition, built identically on both sides.</summary>
    private sealed class CurveSpec
    {
        public int Kind; // 0 line, 1 circle, 2 ellipse, 3 B-spline
        public double Px, Py, Dx, Dy, R1, R2;
        public bool Sense = true;
        public BSplineSpec? BSpline;
        public bool Trimmed;
        public double U1, U2;
        public string Name = "";

        public (N.Geom2d_Curve Native, Curve2D Port) Build()
        {
            N.Geom2d_Curve aN;
            Curve2D aP;
            switch (Kind)
            {
                case 0:
                {
                    using var aNP = new N.gp_Pnt2d(Px, Py);
                    using var aND = new N.gp_Dir2d(Dx, Dy);
                    aN = new N.Geom2d_Line(aNP, aND);
                    aP = new LineCurve2D(new Point2D(Px, Py), new Direction2D(Dx, Dy));
                    break;
                }
                case 1:
                case 2:
                {
                    using var aNP = new N.gp_Pnt2d(Px, Py);
                    using var aND = new N.gp_Dir2d(Dx, Dy);
                    using var aNA = new N.gp_Ax2d(aNP, aND);
                    Axis2D anA = new(new Point2D(Px, Py), new Direction2D(Dx, Dy));
                    if (Kind == 1)
                    {
                        aN = new N.Geom2d_Circle(aNA, R1, Sense);
                        aP = new CircleCurve2D(anA, R1, Sense);
                    }
                    else
                    {
                        aN = new N.Geom2d_Ellipse(aNA, R1, R2, Sense);
                        aP = new EllipseCurve2D(anA, R1, R2, Sense);
                    }
                    break;
                }
                default:
                    aN = Geom2dOracle.ToNative(BSpline!);
                    aP = Geom2dOracle.ToPort(BSpline!);
                    break;
            }
            if (Trimmed)
            {
                N.Geom2d_Curve aNT = new N.Geom2d_TrimmedCurve(aN, U1, U2);
                aN.Dispose();
                return (aNT, new TrimmedCurve2D(aP, U1, U2));
            }
            return (aN, aP);
        }
    }

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
        catch (Exception anE) when (Refusal.IsFailure(anE))
        {
            aPortError = anE;
        }
        theO.Same(aNativeError is not null, aPortError is not null,
            $"{theWhat} throws (native: {aNativeError?.Message}; port: {aPortError?.Message})");
        return aNativeError is null;
    }

    private static (double X, double Y) RawDir(Oracle theO)
    {
        Xy aV = GeomOracleInputs.RawDir2d(theO);
        return (aV.X(), aV.Y());
    }

    private static CurveSpec RandomCurve(Oracle theO, double theSize)
    {
        var aS = new CurveSpec { Kind = theO.Int(0, 3) };
        aS.Px = theO.Real(-theSize, theSize);
        aS.Py = theO.Real(-theSize, theSize);
        (aS.Dx, aS.Dy) = RawDir(theO);
        aS.Sense = theO.Int(0, 3) != 0;
        switch (aS.Kind)
        {
            case 0:
                aS.Name = "line";
                break;
            case 1:
                aS.R1 = theO.Real(0.5, theSize);
                aS.Name = "circle";
                break;
            case 2:
                aS.R1 = theO.Real(0.5, theSize);
                aS.R2 = theO.Real(0.2, aS.R1);
                aS.Name = "ellipse";
                break;
            default:
                aS.BSpline = Geom2dOracle.RandomBSpline(theO);
                aS.Name = $"bspline deg {aS.BSpline.Degree}{(aS.BSpline.Periodic ? " periodic" : "")}"
                          + $"{(aS.BSpline.Weights is null ? "" : " rational")}";
                break;
        }
        // Lines are always trimmed half of the time; other curves a third of the time.
        if (aS.Kind == 0 ? theO.Bool() : theO.Int(0, 2) == 0)
        {
            Trim(theO, aS);
        }
        return aS;
    }

    private static void Trim(Oracle theO, CurveSpec theS)
    {
        double aFirst, aLast;
        switch (theS.Kind)
        {
            case 0:
                aFirst = -15;
                aLast = 15;
                break;
            case 1:
            case 2:
                aFirst = 0;
                aLast = 2 * Math.PI;
                break;
            default:
            {
                BSplineCurve2D aC = Geom2dOracle.ToPort(theS.BSpline!);
                aFirst = aC.FirstParameter;
                aLast = aC.LastParameter;
                break;
            }
        }
        double a = theO.Real(aFirst, aLast), b = theO.Real(aFirst, aLast);
        if (Math.Abs(a - b) < 1e-3 * (aLast - aFirst))
        {
            b = a + 0.5 * (aLast - aFirst);
        }
        theS.Trimmed = true;
        if (theS.Kind == 3 && !theS.BSpline!.Periodic)
        {
            (theS.U1, theS.U2) = a < b ? (a, b) : (b, a);
            if (theS.U2 > aLast)
            {
                theS.U2 = aLast;
            }
        }
        else
        {
            (theS.U1, theS.U2) = (a, b);
        }
        theS.Name = "trimmed " + theS.Name;
    }

    /// <summary>
    /// A second curve: independent, or through a point of the first one, tangent to it there, or the first one
    /// itself (whole, or trimmed differently) so that it overlaps.
    /// </summary>
    private static CurveSpec SecondCurve(Oracle theO, CurveSpec theFirst, Curve2D thePort)
    {
        int aMode = theO.Int(0, 9);
        if (aMode <= 3)
        {
            return RandomCurve(theO, 10);
        }
        if (aMode == 9)
        {
            var aSame = new CurveSpec
            {
                Kind = theFirst.Kind, Px = theFirst.Px, Py = theFirst.Py, Dx = theFirst.Dx, Dy = theFirst.Dy,
                R1 = theFirst.R1, R2 = theFirst.R2, Sense = theFirst.Sense, BSpline = theFirst.BSpline,
                Name = "same " + theFirst.Name
            };
            if (theO.Bool())
            {
                Trim(theO, aSame);
            }
            return aSame;
        }

        double aF = thePort.FirstParameter, aL = thePort.LastParameter;
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aF) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aL))
        {
            (aF, aL) = (-10, 10);
        }
        thePort.D1(theO.Real(aF, aL), out Point2D aP, out Vector2D aV);
        bool isTangent = aMode >= 7 && aV.Magnitude > 1e-6;
        double aDx, aDy;
        if (isTangent)
        {
            (aDx, aDy) = (aV.X, aV.Y);
        }
        else
        {
            (aDx, aDy) = RawDir(theO);
        }
        var aS = new CurveSpec { Dx = aDx, Dy = aDy };
        if (theO.Bool())
        {
            aS.Kind = 0;
            aS.Px = aP.X;
            aS.Py = aP.Y;
            aS.Name = isTangent ? "tangent line" : "line through";
            if (theO.Bool())
            {
                Trim(theO, aS);
            }
        }
        else
        {
            // A circle through P, centred along the normal (tangent) or a random direction.
            double aR = theO.Real(0.5, 10);
            double aNx = isTangent ? -aDy : aDx, aNy = isTangent ? aDx : aDy;
            double aLen = Math.Sqrt(aNx * aNx + aNy * aNy);
            aS.Kind = 1;
            aS.R1 = aR;
            aS.Px = aP.X + aR * aNx / aLen;
            aS.Py = aP.Y + aR * aNy / aLen;
            (aS.Dx, aS.Dy) = RawDir(theO);
            aS.Sense = theO.Bool();
            aS.Name = isTangent ? "tangent circle" : "circle through";
            if (theO.Int(0, 2) == 0)
            {
                Trim(theO, aS);
            }
        }
        return aS;
    }

    private static readonly double[] Tolerances = [1.0e-6, 1.0e-7, 1.0e-4, 1.0e-9];

    private static void SameIntersections(Oracle theO, N.Geom2dAPI_InterCurveCurve theNative,
                                          CurveCurveIntersection2D thePort, string theWhat)
    {
        theO.Same(theNative.NbPoints(), thePort.NbPoints(), $"{theWhat} NbPoints");
        theO.Same(theNative.NbSegments(), thePort.NbSegments(), $"{theWhat} NbSegments");
        for (int i = 1; i <= thePort.NbPoints(); i++)
        {
            using N.gp_Pnt2d aNP = theNative.Point(i);
            theO.Same(aNP, thePort.Point(i), $"{theWhat} Point({i})");
        }
    }

    [Fact]
    public void Geom2dAPI_InterCurveCurve_TwoCurves()
    {
        var anOracle = new Oracle(output, 20261031);
        int aPoints = 0, aSegments = 0;
        var aKinds = new Dictionary<string, int>();
        for (int c = 0; c < Pairs; c++)
        {
            CurveSpec aS1 = RandomCurve(anOracle, 10);
            (N.Geom2d_Curve aN1, Curve2D aP1) = aS1.Build();
            CurveSpec aS2 = SecondCurve(anOracle, aS1, aP1);
            (N.Geom2d_Curve aN2, Curve2D aP2) = aS2.Build();
            double aTol = Tolerances[anOracle.Int(0, Tolerances.Length - 1)];
            bool aSwap = anOracle.Bool();
            if (aSwap)
            {
                (aN1, aN2) = (aN2, aN1);
                (aP1, aP2) = (aP2, aP1);
            }
            string aW = $"#{c} {(aSwap ? aS2.Name : aS1.Name)} x {(aSwap ? aS1.Name : aS2.Name)} tol {aTol}";
            string aKey = aSwap ? $"{aS2.Name} x {aS1.Name}" : $"{aS1.Name} x {aS2.Name}";
            aKinds[aKey] = aKinds.GetValueOrDefault(aKey) + 1;
            using (aN1)
            using (aN2)
            {
                N.Geom2dAPI_InterCurveCurve? aN = null;
                CurveCurveIntersection2D? aP = null;
                if (!Both(anOracle, () => aN = new N.Geom2dAPI_InterCurveCurve(aN1, aN2, aTol),
                          () => aP = new CurveCurveIntersection2D(aP1, aP2, aTol), aW))
                {
                    continue;
                }
                using (aN)
                {
                    SameIntersections(anOracle, aN!, aP!, aW);
                    aPoints += aP!.NbPoints();
                    aSegments += aP.NbSegments();
                }
            }
        }
        output.WriteLine($"{Pairs} pairs, {aPoints} points, {aSegments} segments, {aKinds.Count} kinds of pairs");
        anOracle.Report("Geom2dAPI_InterCurveCurve");
    }

    [Fact]
    public void Geom2dAPI_InterCurveCurve_SelfIntersection()
    {
        var anOracle = new Oracle(output, 20261101);
        int aPoints = 0, aSegments = 0;
        for (int c = 0; c < 400; c++)
        {
            CurveSpec aS = RandomCurve(anOracle, 10);
            if (anOracle.Int(0, 3) != 0)
            {
                // Mostly B-splines, which can cross themselves.
                aS = new CurveSpec { Kind = 3, BSpline = Geom2dOracle.RandomBSpline(anOracle), Name = "bspline" };
                if (anOracle.Int(0, 3) == 0)
                {
                    Trim(anOracle, aS);
                }
            }
            (N.Geom2d_Curve aNC, Curve2D aPC) = aS.Build();
            double aTol = Tolerances[anOracle.Int(0, Tolerances.Length - 1)];
            string aW = $"#{c} {aS.Name} tol {aTol}";
            using (aNC)
            {
                N.Geom2dAPI_InterCurveCurve? aN = null;
                CurveCurveIntersection2D? aP = null;
                if (!Both(anOracle, () => aN = new N.Geom2dAPI_InterCurveCurve(aNC, aTol),
                          () => aP = new CurveCurveIntersection2D(aPC, aTol), aW))
                {
                    continue;
                }
                using (aN)
                {
                    SameIntersections(anOracle, aN!, aP!, aW);
                    aPoints += aP!.NbPoints();
                    aSegments += aP.NbSegments();
                }
            }
        }
        output.WriteLine($"{aPoints} points, {aSegments} segments");
        anOracle.Report("Geom2dAPI_InterCurveCurve (self-intersection)");
    }

    [Fact]
    public void Geom2dAPI_ProjectPointOnCurve_Port()
    {
        var anOracle = new Oracle(output, 20261102);
        int aProjections = 0;
        var aPort = new ProjectPointOnCurve2D();
        for (int c = 0; c < 600; c++)
        {
            CurveSpec aS = RandomCurve(anOracle, 10);
            (N.Geom2d_Curve aNC, Curve2D aPC) = aS.Build();
            using (aNC)
            {
                double aF = aPC.FirstParameter, aL = aPC.LastParameter;
                bool aWithRange = anOracle.Bool() || global::Kerf.Engine.Foundation.Precision.IsInfinite(aF) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aL);
                if (aWithRange)
                {
                    (aF, aL) = global::Kerf.Engine.Foundation.Precision.IsInfinite(aF) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aL)
                        ? (anOracle.Real(-20, 0), anOracle.Real(0.5, 20))
                        : (anOracle.Real(aF, aL), anOracle.Real(aF, aL));
                    if (aF > aL)
                    {
                        (aF, aL) = (aL, aF);
                    }
                }
                for (int k = 0; k < 3; k++)
                {
                    Point2D aP = aPC.Value(anOracle.Real(aF, aL));
                    if (k > 0)
                    {
                        double aScale = k == 1 ? anOracle.Real(1e-3, 1) : anOracle.Real(1, 30);
                        aP = new Point2D(aP.X + aScale * anOracle.Real(-1, 1), aP.Y + aScale * anOracle.Real(-1, 1));
                    }
                    string aW = $"#{c} {aS.Name} [{aF:R}, {aL:R}] point {k}";
                    using N.gp_Pnt2d aNP = Oracle.ToNative(aP);
                    N.Geom2dAPI_ProjectPointOnCurve? aN = null;
                    // The port's object is reused through Init, as an algorithm object may be.
                    if (!Both(anOracle,
                            () => aN = aWithRange ? new N.Geom2dAPI_ProjectPointOnCurve(aNP, aNC, aF, aL)
                                                  : new N.Geom2dAPI_ProjectPointOnCurve(aNP, aNC),
                            () =>
                            {
                                if (aWithRange)
                                {
                                    aPort.Init(aP, aPC, aF, aL);
                                }
                                else
                                {
                                    aPort.Init(aP, aPC);
                                }
                            }, aW))
                    {
                        continue;
                    }
                    using (aN)
                    {
                        int aNb = aPort.NbPoints();
                        anOracle.Same(aN!.NbPoints(), aNb, $"{aW} NbPoints");
                        for (int j = 1; j <= aNb; j++)
                        {
                            using N.gp_Pnt2d aNPt = aN.Point(j);
                            anOracle.Same(aNPt, aPort.Point(j), $"{aW} Point({j})");
                            anOracle.Same(aN.Parameter(j), aPort.Parameter(j), $"{aW} Parameter({j})");
                            anOracle.Same(aN.Distance(j), aPort.Distance(j), $"{aW} Distance({j})");
                            aProjections++;
                        }
                        if (aNb > 0)
                        {
                            using N.gp_Pnt2d aNNear = aN.NearestPoint();
                            anOracle.Same(aNNear, aPort.NearestPoint(), $"{aW} NearestPoint");
                            anOracle.Same(aN.LowerDistanceParameter(), aPort.LowerDistanceParameter(),
                                $"{aW} LowerDistanceParameter");
                            anOracle.Same(aN.LowerDistance(), aPort.LowerDistance(), $"{aW} LowerDistance");
                        }
                    }
                }
            }
        }
        output.WriteLine($"{aProjections} projections");
        anOracle.Report("Geom2dAPI_ProjectPointOnCurve");
    }
}
