// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Geometry.ParametrizationType;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// AppDef against native OCCT, through GeomAPI_PointsToBSpline, the only native entry point in the bindings. Each
/// test repeats what GeomAPI_PointsToBSpline.cxx does around AppDef (V8_0_1): the AppDef_BSplineCompute settings
/// it derives from the tolerance (squares and two iterations for Tol3D &lt;= 1e-3, else the gradient), the
/// continuity, the parameters it normalises to [0, 1] and the knots it reparametrises back; and, for the weighted
/// form, the AppDef_Variational settings (1000 segments, no min-max, its iteration count, the criterion weights).
/// The poles, knots, multiplicities and degree of the resulting curve are compared.
/// </summary>
public class AppDefOracleTests(ITestOutputHelper output)
{
    private const int Cases = 300;

    // A point set: samples of a random smooth curve, with random noise and random parameter spacing.
    internal static NCollection_Array1<Point3D> Points(Oracle theOracle)
    {
        int aNb = theOracle.Int(2, 40);
        int aKind = theOracle.Int(0, 3);
        double aNoise = theOracle.Bool() ? 0.0 : theOracle.Real(0.0, 0.5);
        double[] aC = new double[12];
        for (int k = 0; k < aC.Length; k++)
        {
            aC[k] = theOracle.Real(-10, 10);
        }
        NCollection_Array1<Point3D> aPoints = new(1, aNb);
        double t = 0.0;
        for (int i = 1; i <= aNb; i++)
        {
            t += theOracle.Real(0.2, 1.0) / aNb;
            Point3D aP = aKind switch
            {
                0 => new Point3D(theOracle.Real(), theOracle.Real(), theOracle.Real()), // random points
                1 => new Point3D(aC[0] + aC[1] * t + aC[2] * t * t + aC[3] * t * t * t,
                                aC[4] + aC[5] * t + aC[6] * t * t,
                                aC[7] + aC[8] * t * t * t), // polynomial
                2 => new Point3D(aC[0] + 10 * Math.Cos(5 * t), aC[1] + 10 * Math.Sin(5 * t), aC[2] * t), // helix
                _ => new Point3D(10 * t, aC[3] * Math.Sin(aC[4] * t), aC[5] * Math.Cos(aC[6] * t)), // sines
            };
            if (aNoise > 0.0)
            {
                aP = new Point3D(aP.X + theOracle.Real(-aNoise, aNoise), aP.Y + theOracle.Real(-aNoise, aNoise),
                                aP.Z + theOracle.Real(-aNoise, aNoise));
            }
            aPoints[i] = aP;
        }
        return aPoints;
    }

    internal static Continuity Continuity(Oracle theOracle) => theOracle.Int(0, 3) switch
    {
        0 => global::Kerf.Geometry.Continuity.C0,
        1 => global::Kerf.Geometry.Continuity.C1,
        2 => global::Kerf.Geometry.Continuity.C2,
        _ => global::Kerf.Geometry.Continuity.C3,
    };

    internal static double Tolerance(Oracle theOracle) => theOracle.Int(0, 4) switch
    {
        0 => 1.0e-6,
        1 => 1.0e-3,
        2 => 1.0e-2,
        3 => 0.1,
        _ => 1.0,
    };

    // GeomAPI_PointsToBSpline::Init's switch on the continuity.
    private static void SetContinuity(BSplineCompute theComputer, Continuity theContinuity)
    {
        switch (theContinuity)
        {
            case global::Kerf.Geometry.Continuity.C0:
                theComputer.SetContinuity(0);
                break;
            case global::Kerf.Geometry.Continuity.G1:
            case global::Kerf.Geometry.Continuity.C1:
                theComputer.SetContinuity(1);
                break;
            case global::Kerf.Geometry.Continuity.G2:
            case global::Kerf.Geometry.Continuity.C2:
                theComputer.SetContinuity(2);
                break;
            default:
                theComputer.SetContinuity(3);
                break;
        }
    }

    private static void Compare(Oracle theOracle, N.Geom_BSplineCurve theNative, MultiBSpCurve theCurve,
                                NCollection_Array1<double> theKnots, string theWhat, double theTolerance)
    {
        theOracle.Same(theNative.Degree(), theCurve.Degree(), theWhat + " degree");
        theOracle.Same(theNative.NbPoles(), theCurve.NbPoles(), theWhat + " NbPoles");
        theOracle.Same(theNative.NbKnots(), theKnots.Length(), theWhat + " NbKnots");
        NCollection_Array1<Point3D> aPoles = new(1, theCurve.NbPoles());
        theCurve.Curve(1, aPoles);
        for (int i = 1; i <= aPoles.Length(); i++)
        {
            using N.gp_Pnt aNP = theNative.Pole(i);
            theOracle.Same(aNP, aPoles[i], $"{theWhat} pole {i}", theTolerance);
        }
        NCollection_Array1<int> aMults = theCurve.Multiplicities();
        for (int i = 1; i <= theKnots.Length(); i++)
        {
            theOracle.Same(theNative.Knot(i), theKnots[i], $"{theWhat} knot {i}", theTolerance);
            theOracle.Same(theNative.Multiplicity(i), aMults[i], $"{theWhat} mult {i}");
        }
    }

    [Theory]
    [InlineData(ChordLength)]
    [InlineData(Centripetal)]
    [InlineData(IsoParametric)]
    public void PointsToBSpline_ParametrizationType(global::Kerf.Geometry.ParametrizationType theParType)
    {
        Tape.Row(theParType);
        Oracle anOracle = new(output, 20261029 + (int)theParType);
        int aCompared = 0;
        for (int aCase = 0; aCase < Cases; aCase++)
        {
            NCollection_Array1<Point3D> aPoints = Points(anOracle);
            int aDegMin = anOracle.Int(1, 4);
            int aDegMax = anOracle.Int(aDegMin, 8);
            Continuity aCont = Continuity(anOracle);
            double aTol3D = Tolerance(anOracle);

            using N.TColgp_Array1OfPnt aNPoints = Oracle.ToNative(aPoints);
            using N.GeomAPI_PointsToBSpline aNative = new(aNPoints, (N.Approx_ParametrizationType)(int)theParType,
                                                          aDegMin, aDegMax, (N.GeomAbs_Shape)(int)aCont, aTol3D);

            // GeomAPI_PointsToBSpline::Init(Points, ParType, DegMin, DegMax, Continuity, Tol3D)
            double Tol2D = 0.0;
            int nbit = 2;
            bool UseSquares = false;
            if (aTol3D <= 1.0e-3)
            {
                UseSquares = true;
            }
            BSplineCompute TheComputer = new(aDegMin, aDegMax, aTol3D, Tol2D, nbit, true, theParType, UseSquares);
            SetContinuity(TheComputer, aCont);
            TheComputer.Perform(new MultiLine(aPoints));
            MultiBSpCurve TheCurve = TheComputer.Value();

            Assert.True(aNative.IsDone());
            using N.Geom_BSplineCurve? aNCurve = aNative.Curve();
            Assert.NotNull(aNCurve);
            // Squares is a direct solve; the gradient (BFGS) iterates on the parameters and amplifies rounding.
            double aTol = UseSquares ? 1.0e-9 : 1.0e-6;
            Compare(anOracle, aNCurve, TheCurve, TheCurve.Knots(), $"case {aCase}", aTol);
            aCompared++;
        }
        anOracle.Report($"GeomAPI_PointsToBSpline {theParType}, {aCompared} point sets");
    }

    [Fact]
    public void PointsToBSpline_Parameters()
    {
        Oracle anOracle = new(output, 20261030);
        for (int aCase = 0; aCase < Cases; aCase++)
        {
            NCollection_Array1<Point3D> aPoints = Points(anOracle);
            int Nbp = aPoints.Length();
            NCollection_Array1<double> Params = new(1, Nbp);
            double u = anOracle.Real(-5, 5);
            for (int i = 1; i <= Nbp; i++)
            {
                Params[i] = u;
                u += anOracle.Real(0.01, 2.0);
            }
            int aDegMin = anOracle.Int(1, 4);
            int aDegMax = anOracle.Int(aDegMin, 8);
            Continuity aCont = Continuity(anOracle);
            double aTol3D = Tolerance(anOracle);

            using N.TColgp_Array1OfPnt aNPoints = Oracle.ToNative(aPoints);
            using N.TColStd_Array1OfReal aNParams = Oracle.ToNative(Params);
            using N.GeomAPI_PointsToBSpline aNative = new(aNPoints, aNParams, aDegMin, aDegMax, (N.GeomAbs_Shape)(int)aCont,
                                                          aTol3D);

            // GeomAPI_PointsToBSpline::Init(Points, Params, DegMin, DegMax, Continuity, Tol3D)
            double Tol2D = 0.0;
            math_Vector theParams = new(1, Nbp);
            theParams[1] = 0.0;
            theParams[Nbp] = 1.0;
            double Uf = Params[Params.Lower()];
            double Ul = Params[Params.Upper()] - Uf;
            for (int i = 2; i < Nbp; i++)
            {
                theParams[i] = (Params[i] - Uf) / Ul;
            }
            BSplineCompute TheComputer = new(aDegMin, aDegMax, aTol3D, Tol2D, 0, true, IsoParametric, true);
            TheComputer.SetParameters(theParams);
            SetContinuity(TheComputer, aCont);
            TheComputer.Perform(new MultiLine(aPoints));
            MultiBSpCurve TheCurve = TheComputer.Value();
            NCollection_Array1<double> Knots = TheCurve.Knots();
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(Params[Params.Lower()], Params[Params.Upper()], Knots);

            Assert.True(aNative.IsDone());
            using N.Geom_BSplineCurve? aNCurve = aNative.Curve();
            Assert.NotNull(aNCurve);
            Compare(anOracle, aNCurve, TheCurve, Knots, $"case {aCase}", 1.0e-9);
        }
        anOracle.Report("GeomAPI_PointsToBSpline with parameters");
    }

    [Fact]
    public void PointsToBSpline_Variational()
    {
        Oracle anOracle = new(output, 20261031);
        int aDone = 0;
        for (int aCase = 0; aCase < Cases; aCase++)
        {
            NCollection_Array1<Point3D> aPoints = Points(anOracle);
            if (aPoints.Length() < 3)
            {
                continue; // OCCT reads past its points for two points (occt-issues.md)
            }
            double W1 = anOracle.Real(0, 1), W2 = anOracle.Real(0, 1), W3 = anOracle.Real(0, 1);
            if (W1 + W2 + W3 < 0.01)
            {
                W1 = 1.0;
            }
            int aDegMax = anOracle.Int(3, 8);
            // Not C0: at C0, FEmTool_Curve::ReduceDegree can leave elements of degree 1, whose second derivative
            // OCCT reads from outside the element's coefficients, for the first element from uninitialised memory,
            // so native results change from run to run (occt-issues.md). The unit tests cover C0.
            Continuity aCont = anOracle.Bool() ? global::Kerf.Geometry.Continuity.C1 : global::Kerf.Geometry.Continuity.C2;
            double aTol3D = Tolerance(anOracle);

            using N.TColgp_Array1OfPnt aNPoints = Oracle.ToNative(aPoints);
            N.GeomAPI_PointsToBSpline? aNative = null;
            bool aNativeThrew = false;
            try
            {
                // OCCT catches failures in Approximate only: the setup (WorkDegree too small for the continuity) throws
                lock (Oracle.NativeVariational)
                {
                    aNative = new(aNPoints, W1, W2, W3, aDegMax, (N.GeomAbs_Shape)(int)aCont, aTol3D);
                }
            }
            catch (N.OcctException)
            {
                aNativeThrew = true;
            }
            using N.GeomAPI_PointsToBSpline? aNativeToDispose = aNative;

            // GeomAPI_PointsToBSpline::Init(Points, W1, W2, W3, DegMax, Continuity, Tol3D)
            int NbPoint = aPoints.Length(), i;
            int nbit = 2;
            if (aTol3D <= 1.0e-3)
            {
                nbit = 0;
            }
            MultiLine multL = new(NbPoint);
            for (i = 1; i <= NbPoint; ++i)
            {
                MultiPointConstraint mpc = new(1, 0);
                mpc.SetPoint(1, aPoints.Value(aPoints.Lower() + i - 1));
                multL.SetValue(i, mpc);
            }
            NCollection_HArray1<ConstraintCouple> TABofCC = new(1, NbPoint);
            Constraint Constraint = global::Kerf.Engine.Approximation.Constraint.NoConstraint;
            for (i = 1; i <= NbPoint; ++i)
            {
                ConstraintCouple ACC = new(i, Constraint);
                TABofCC.SetValue(i, ACC);
            }
            Variational? Variation = null;
            bool aPortThrew = false;
            try
            {
                Variation = new(multL, 1, NbPoint, TABofCC);
                int theMaxSegments = 1000;
                bool theWithMinMax = false;
                Variation.SetMaxDegree(aDegMax);
                Variation.SetContinuity(aCont);
                Variation.SetMaxSegment(theMaxSegments);
                Variation.SetTolerance(aTol3D);
                Variation.SetWithMinMax(theWithMinMax);
                Variation.SetNbIterations(nbit);
                Variation.SetCriteriumWeight(W1, W2, W3);
            }
            catch (FailureException)
            {
                aPortThrew = true;
            }
            anOracle.Same(aNativeThrew, aPortThrew, $"case {aCase} setup throws");
            if (aPortThrew)
            {
                continue;
            }
            bool aPortDone = false;
            MultiBSpCurve? TheCurve = null;
            if (Variation!.IsCreated && !Variation.IsOverConstrained)
            {
                try
                {
                    Variation.Approximate();
                    if (Variation.IsDone)
                    {
                        TheCurve = Variation.Value();
                        aPortDone = true;
                    }
                }
                catch (FailureException)
                {
                }
            }

            anOracle.Same(aNative!.IsDone(), aPortDone, $"case {aCase} IsDone");
            if (!aPortDone)
            {
                continue;
            }
            aDone++;
            using N.Geom_BSplineCurve? aNCurve = aNative.Curve();
            Assert.NotNull(aNCurve);
            // The variational solve iterates (Newton on the parameters, conjugate gradients in FEmTool).
            // The variational solve iterates (Newton on the parameters, conjugate gradients in FEmTool).
            Compare(anOracle, aNCurve, TheCurve!, TheCurve!.Knots(), $"case {aCase}", 1.0e-6);
        }
        anOracle.Report($"GeomAPI_PointsToBSpline variational, {aDone} curves");
    }
}
