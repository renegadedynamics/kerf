// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Approximation.Advanced.CriterionRepartition;
using static Kerf.Engine.Approximation.Advanced.CriterionType;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// AdvApp2Var_ApproxAFunc2Var on surfaces with known results (OCCT's GTests cover only the working classes): planes
/// and polynomial surfaces are reproduced exactly, a sphere patch and sin(u)cos(v) are approximated within their
/// tolerances, under each continuity, favoured iso, precision code and criterion, and in parallel.
/// </summary>
public class AdvApp2Var_ApproxAFunc2VarTests
{
    /// <summary>F(u, v) and its partial derivative of order (du, dv), as three coordinates.</summary>
    internal delegate void Function(double u, double v, int du, int dv, Span<double> xyz);

    /// <summary>An evaluator over a <see cref="Function"/>, counting its calls.</summary>
    internal sealed class Evaluator(Function theF) : EvaluatorFunc2Var
    {
        public int NbCalls;

        public override void Evaluate(int theDimension, ReadOnlySpan<double> theUStartEnd,
                                      ReadOnlySpan<double> theVStartEnd, int theFavorIso, double theConstParam,
                                      int theNbParams, ReadOnlySpan<double> theParameters, int theUOrder,
                                      int theVOrder, Span<double> theResult, out int theErrorCode)
        {
            Interlocked.Increment(ref NbCalls);
            theErrorCode = 0;
            if (theDimension != 3)
            {
                theErrorCode = 1;
                return;
            }
            for (int j = 0; j < theNbParams; j++)
            {
                double u = theFavorIso == 1 ? theConstParam : theParameters[j];
                double v = theFavorIso == 1 ? theParameters[j] : theConstParam;
                theF(u, v, theUOrder, theVOrder, theResult.Slice(3 * j, 3));
            }
        }
    }

    // A plane z = 1 + 0.5u - 0.3v
    internal static void Plane(double u, double v, int du, int dv, Span<double> r)
    {
        r[0] = (du, dv) switch { (0, 0) => u, (1, 0) => 1, _ => 0 };
        r[1] = (du, dv) switch { (0, 0) => v, (0, 1) => 1, _ => 0 };
        r[2] = (du, dv) switch { (0, 0) => 1 + 0.5 * u - 0.3 * v, (1, 0) => 0.5, (0, 1) => -0.3, _ => 0 };
    }

    // x = u, y = v, z = u^3 v^2 - 2uv + u^2 + 1 (degrees 3 in u, 2 in v)
    internal static void Poly(double u, double v, int du, int dv, Span<double> r)
    {
        r[0] = (du, dv) switch { (0, 0) => u, (1, 0) => 1, _ => 0 };
        r[1] = (du, dv) switch { (0, 0) => v, (0, 1) => 1, _ => 0 };
        double pu = du switch { 0 => u * u * u, 1 => 3 * u * u, 2 => 6 * u, 3 => 6, _ => 0 };
        double pv = dv switch { 0 => v * v, 1 => 2 * v, 2 => 2, _ => 0 };
        double qu = du switch { 0 => u, 1 => 1, _ => 0 };
        double qv = dv switch { 0 => v, 1 => 1, _ => 0 };
        double su = du switch { 0 => u * u, 1 => 2 * u, 2 => 2, _ => 0 };
        double sv = dv == 0 ? 1 : 0;
        r[2] = pu * pv - 2 * qu * qv + su * sv + (du == 0 && dv == 0 ? 1 : 0);
    }

    private static double SinD(double t, int d) => (d % 4) switch
    {
        0 => Math.Sin(t),
        1 => Math.Cos(t),
        2 => -Math.Sin(t),
        _ => -Math.Cos(t),
    };

    private static double CosD(double t, int d) => SinD(t + Math.PI / 2, d);

    // x = u, y = v, z = sin(u) cos(v)
    internal static void SinCos(double u, double v, int du, int dv, Span<double> r)
    {
        r[0] = (du, dv) switch { (0, 0) => u, (1, 0) => 1, _ => 0 };
        r[1] = (du, dv) switch { (0, 0) => v, (0, 1) => 1, _ => 0 };
        r[2] = SinD(u, du) * CosD(v, dv);
    }

    // A sphere of radius 2: (2 cos u cos v, 2 sin u cos v, 2 sin v)
    internal static void Sphere(double u, double v, int du, int dv, Span<double> r)
    {
        r[0] = 2 * CosD(u, du) * CosD(v, dv);
        r[1] = 2 * SinD(u, du) * CosD(v, dv);
        r[2] = du == 0 ? 2 * SinD(v, dv) : 0;
    }

    internal static NCollection_HArray1<double> Tol1(double theTol) => new(1, 1, theTol);

    internal static NCollection_HArray2<double> Tol2(double theTol) => new(1, 1, 1, 4, theTol);

    internal static ApproxAFunc2Var Approximate(Function theF, double theU0, double theU1, double theV0,
                                                          double theV1, double theTol, Continuity theUCont,
                                                          Continuity theVCont, int theMaxDegU = 11,
                                                          int theMaxDegV = 11, int theMaxPatch = 50,
                                                          int thePrecisCode = 1,
                                                          IsoType theIso = IsoV,
                                                          Criterion? theCrit = null)
    {
        Evaluator anEval = new(theF);
        DichoCutting aCut = new();
        NCollection_HArray1<double> aNul1 = Tol1(0.0);
        NCollection_HArray2<double> aNul2 = Tol2(0.0);
        return theCrit is null
            ? new ApproxAFunc2Var(0, 0, 1, aNul1, aNul1, Tol1(theTol), aNul2, aNul2, Tol2(theTol), theU0,
                                             theU1, theV0, theV1, theIso, theUCont, theVCont, thePrecisCode, theMaxDegU,
                                             theMaxDegV, theMaxPatch, anEval, aCut, aCut)
            : new ApproxAFunc2Var(0, 0, 1, aNul1, aNul1, Tol1(theTol), aNul2, aNul2, Tol2(theTol), theU0,
                                             theU1, theV0, theV1, theIso, theUCont, theVCont, thePrecisCode, theMaxDegU,
                                             theMaxDegV, theMaxPatch, anEval, theCrit, aCut, aCut);
    }

    /// <summary>The largest distance between the surface and F on an n x n grid of the domain.</summary>
    internal static double MaxDeviation(Surface theS, Function theF, double theU0, double theU1, double theV0,
                                        double theV1, int theN = 23)
    {
        double aMax = 0;
        Span<double> r = stackalloc double[3];
        for (int i = 0; i <= theN; i++)
        {
            for (int j = 0; j <= theN; j++)
            {
                double u = theU0 + (theU1 - theU0) * i / theN, v = theV0 + (theV1 - theV0) * j / theN;
                theF(u, v, 0, 0, r);
                Point3D p = theS.Value(u, v);
                aMax = Math.Max(aMax, p.DistanceTo(new Point3D(r[0], r[1], r[2])));
            }
        }
        return aMax;
    }

    [Fact]
    public void Plane_IsReproduced()
    {
        ApproxAFunc2Var anApprox = Approximate(Plane, -1, 2, 0, 3, 1e-6, C1, C1);
        Assert.True(anApprox.IsDone);
        Assert.True(anApprox.HasResult);
        BSplineSurface aS = anApprox.Surface(1);
        Assert.True(aS.IsFrozen);
        Assert.True(anApprox.MaxError(3, 1) < 1e-12);
        Assert.True(MaxDeviation(aS, Plane, -1, 2, 0, 3) < 1e-12);
        Assert.Equal(1, aS.UKnotCount - 1);
        Assert.Equal(1, aS.VKnotCount - 1);
    }

    [Fact]
    public void Polynomial_IsReproduced()
    {
        ApproxAFunc2Var anApprox = Approximate(Poly, -1, 1.5, -2, 1, 1e-7, C2, C1);
        Assert.True(anApprox.IsDone);
        BSplineSurface aS = anApprox.Surface(1);
        Assert.True(MaxDeviation(aS, Poly, -1, 1.5, -2, 1) < 1e-11, anApprox.ToString());
        Assert.Equal(1, aS.UKnotCount - 1);
    }

    [Theory]
    [InlineData(C1, C1, 1e-5, IsoV)]
    [InlineData(C2, C2, 1e-6, IsoU)]
    [InlineData(C1, C2, 1e-7, NoneIso)]
    [InlineData(C2, C0, 1e-5, IsoV)]
    public void SinCos_IsWithinTolerance(Continuity theUCont, Continuity theVCont, double theTol,
                                         object theIsoValue)
    {
        var theIso = (IsoType)theIsoValue; // the theory's data as object: the engine type is internal
        double u1 = 2 * Math.PI, v1 = 2 * Math.PI;
        ApproxAFunc2Var anApprox =
            Approximate(SinCos, 0, u1, 0, v1, theTol, theUCont, theVCont, theIso: theIso);
        Assert.True(anApprox.IsDone, anApprox.ToString());
        BSplineSurface aS = anApprox.Surface(1);
        Assert.True(anApprox.MaxError(3, 1) <= theTol);
        Assert.True(MaxDeviation(aS, SinCos, 0, u1, 0, v1) <= theTol, anApprox.ToString());
        AssertContinuity(aS, theUCont, theVCont);
    }

    /// <summary>
    /// OCCT's error estimate leaves out the U truncation (occt-issues.md: mma2ce1_ never computes the Jacobi maxima
    /// in U, which stay zero): here it reports success, with a maximum error of 1e-5, for a surface of degree 1 in V
    /// that is far from sin(u)cos(v). The values are native OCCT's.
    /// </summary>
    [Fact]
    public void SinCos_C0_ReproducesOcctsUnderestimatedError()
    {
        double u1 = 2 * Math.PI, v1 = 2 * Math.PI;
        ApproxAFunc2Var anApprox =
            Approximate(SinCos, 0, u1, 0, v1, 1e-4, C0, C0, theIso: IsoV);
        Assert.True(anApprox.IsDone);
        Assert.Equal(1.0346934213069106E-05, anApprox.MaxError(3, 1));
        Assert.Equal(0.8660254037952803, anApprox.AverageError(3, 1));
        Assert.Equal(9, anApprox.UDegree());
        Assert.Equal(1, anApprox.VDegree());
        Assert.True(MaxDeviation(anApprox.Surface(1), SinCos, 0, u1, 0, v1) > 1);
    }

    /// <summary>The knots' multiplicities give at least the requested continuity.</summary>
    internal static void AssertContinuity(BSplineSurface theS, Continuity theUCont, Continuity theVCont)
    {
        int anOrdU = theUCont == C0 ? 0 : theUCont == C1 ? 1 : 2;
        int anOrdV = theVCont == C0 ? 0 : theVCont == C1 ? 1 : 2;
        for (int i = 2; i < theS.UKnotCount; i++)
        {
            Assert.True(theS.UDegree - theS.UMultiplicity(i) >= anOrdU);
        }
        for (int i = 2; i < theS.VKnotCount; i++)
        {
            Assert.True(theS.VDegree - theS.VMultiplicity(i) >= anOrdV);
        }
    }

    [Theory]
    [InlineData(C1, C1, 1e-5, 1)]
    [InlineData(C2, C1, 1e-6, 2)]
    [InlineData(C2, C2, 1e-7, 3)]
    public void SpherePatch_IsWithinTolerance(Continuity theUCont, Continuity theVCont, double theTol,
                                              int thePrecisCode)
    {
        ApproxAFunc2Var anApprox =
            Approximate(Sphere, 0, Math.PI, -Math.PI / 3, Math.PI / 3, theTol, theUCont, theVCont,
                        thePrecisCode: thePrecisCode);
        Assert.True(anApprox.IsDone, anApprox.ToString());
        BSplineSurface aS = anApprox.Surface(1);
        Assert.True(MaxDeviation(aS, Sphere, 0, Math.PI, -Math.PI / 3, Math.PI / 3) <= theTol, anApprox.ToString());
        AssertContinuity(aS, theUCont, theVCont);
    }

    /// <summary>
    /// A U degree much larger than the V degree, where OCCT writes past its work zone (occt-issues.md); the port
    /// completes with OCCT's values (decision 198).
    /// </summary>
    [Fact]
    public void LargeUDegree_SmallVDegree_Completes()
    {
        ApproxAFunc2Var anApprox =
            Approximate(Plane, -1, 2, 0, 3, 1e-6, C0, C2, theMaxDegU: 14, theMaxDegV: 5);
        Assert.True(anApprox.IsDone);
        Assert.True(MaxDeviation(anApprox.Surface(1), Plane, -1, 2, 0, 3) < 1e-12);
    }

    /// <summary>A criterion on the patches' maximum error.</summary>
    internal sealed class MaxErrorCriterion : Criterion
    {
        public MaxErrorCriterion(double theMax, CriterionType theType,
                                 CriterionRepartition theRepartition)
        {
            _maxValue = theMax;
            _type = theType;
            _repartition = theRepartition;
        }

        public override void Value(Patch P, Context C) =>
            P.SetCritValue(P.MaxErrors()!.Value(1));

        public override bool IsSatisfied(Patch P) => P.CritValue() < _maxValue;
    }

    [Theory]
    [InlineData(Absolute, Regular)]
    [InlineData(Absolute, Incremental)]
    [InlineData(Relative, Regular)]
    [InlineData(Relative, Incremental)]
    public void Criterion_DrivesTheCutting(object theTypeValue,
                                                  object theRepartitionValue)
    {
        var theType = (CriterionType)theTypeValue; // the theory's data as object: the engine type is internal
        var theRepartition = (CriterionRepartition)theRepartitionValue; // the theory's data as object: the engine type is internal
        const double aCrit = 1e-6;
        MaxErrorCriterion aCriterion = new(aCrit, theType, theRepartition);
        Assert.Equal(aCrit, aCriterion.MaxValue());
        Assert.Equal(theType, aCriterion.Type());
        Assert.Equal(theRepartition, aCriterion.Repartition());
        ApproxAFunc2Var anApprox = Approximate(SinCos, 0, 4, 0, 4, 1e-5, C1, C1,
                                                          theMaxDegU: 7, theMaxDegV: 7, theMaxPatch: 400,
                                                          theCrit: aCriterion);
        Assert.True(anApprox.HasResult);
        // the criterion is the patches' maximum error, read after their errors are complete: its maximum over the
        // patches is the approximation's maximum error
        Assert.Equal(anApprox.MaxError(3, 1), anApprox.CritError(3, 1));
        BSplineSurface aS = anApprox.Surface(1);
        Assert.True(aS.UKnotCount > 2, anApprox.ToString());
        if (theRepartition == Regular)
        {
            // a regular grid: NbInt x NbInt equal intervals
            Assert.Equal(aS.UKnotCount, aS.VKnotCount);
            double aStep = 4.0 / (aS.UKnotCount - 1);
            for (int i = 1; i <= aS.UKnotCount; i++)
            {
                Assert.Equal((i - 1) * aStep, aS.UKnot(i), 1e-14);
            }
        }
        Assert.True(MaxDeviation(aS, SinCos, 0, 4, 0, 4) < 1e-5, anApprox.ToString());
    }

    [Fact]
    public void Approximations_RunInParallel_AsSequentially()
    {
        (Function F, double U1, double V1, Continuity C, double Tol)[] aCases =
        [
            (SinCos, 2 * Math.PI, 2 * Math.PI, C1, 1e-6),
            (Sphere, Math.PI, 1.0, C2, 1e-7),
            (Poly, 1.5, 1.0, C2, 1e-8),
            (SinCos, 3.0, 5.0, C2, 1e-7),
        ];
        BSplineSurface Run(int i)
        {
            var c = aCases[i % aCases.Length];
            return Approximate(c.F, 0, c.U1, 0, c.V1, c.Tol, c.C, c.C).Surface(1);
        }
        BSplineSurface[] aSequential = new BSplineSurface[aCases.Length];
        for (int i = 0; i < aCases.Length; i++)
        {
            aSequential[i] = Run(i);
        }
        BSplineSurface[] aParallel = new BSplineSurface[64];
        Parallel.For(0, aParallel.Length, new ParallelOptions { MaxDegreeOfParallelism = 8 },
                     i => aParallel[i] = Run(i));
        for (int i = 0; i < aParallel.Length; i++)
        {
            BSplineSurface aRef = aSequential[i % aCases.Length], aS = aParallel[i];
            Assert.Equal(aRef.UPoleCount, aS.UPoleCount);
            Assert.Equal(aRef.VPoleCount, aS.VPoleCount);
            for (int iu = 1; iu <= aRef.UPoleCount; iu++)
            {
                for (int iv = 1; iv <= aRef.VPoleCount; iv++)
                {
                    Assert.Equal(aRef.Pole(iu, iv), aS.Pole(iu, iv));
                }
            }
        }
    }

    [Fact]
    public void Accessors_ReturnCopies_AndNullForMissingDimensions()
    {
        ApproxAFunc2Var anApprox = Approximate(Sphere, 0, 1, 0, 1, 1e-6, C1, C1);
        Assert.Equal(0, anApprox.NumSubSpaces(1));
        Assert.Equal(0, anApprox.NumSubSpaces(2));
        Assert.Equal(1, anApprox.NumSubSpaces(3));
        Assert.Null(anApprox.MaxError(1));
        Assert.Null(anApprox.AverageError(2));
        Assert.Null(anApprox.UFrontError(1));
        Assert.Null(anApprox.VFrontError(2));
        NCollection_HArray1<double> aMax = anApprox.MaxError(3)!;
        Assert.Equal(anApprox.MaxError(3, 1), aMax.Value(1));
        aMax.SetValue(1, 42);
        Assert.NotEqual(42, anApprox.MaxError(3, 1));
        Assert.Equal(anApprox.AverageError(3, 1), anApprox.AverageError(3)!.Value(1));
        Assert.Equal(anApprox.UFrontError(3, 1), anApprox.UFrontError(3)!.Value(1));
        Assert.Equal(anApprox.VFrontError(3, 1), anApprox.VFrontError(3)!.Value(1));
        Assert.Equal(0.0, anApprox.CritError(3, 1));
        Assert.Same(anApprox.Surface(1), anApprox.Surface(1));
        Assert.Equal(anApprox.UDegree(), anApprox.Surface(1).UDegree);
        EngineAssert.ThrowsAny<OutOfRangeException>(() => anApprox.MaxError(4));
        EngineAssert.ThrowsAny<OutOfRangeException>(() => anApprox.VFrontError(0));
        EngineAssert.ThrowsAny<OutOfRangeException>(() => anApprox.MaxError(2, 1));
        EngineAssert.ThrowsAny<OutOfRangeException>(() => anApprox.CritError(3, 2));
        EngineAssert.ThrowsAny<ImmutableObjectException>(() => anApprox.Surface(1).SetPole(1, 1, new Point3D()));
        string aDump = anApprox.ToString();
        Assert.Contains("There is a result within the requested tolerance 1E-06", aDump);
        Assert.Contains("Number of knots in U : ", aDump);
    }

    [Fact]
    public void InvalidContinuity_Throws()
    {
        EngineAssert.ThrowsAny<ConstructionErrorException>(
            () => Approximate(Plane, 0, 1, 0, 1, 1e-6, C3, C1));
        EngineAssert.ThrowsAny<ConstructionErrorException>(
            () => Approximate(Plane, 0, 1, 0, 1, 1e-6, C1, G1));
    }

    [Fact]
    public void FailingEvaluator_Throws()
    {
        // one 1D subspace, which the evaluator refuses (it answers only in dimension 3): the iso approximation fails
        Evaluator anEval = new(Plane);
        DichoCutting aCut = new();
        EngineAssert.ThrowsAny<ConstructionErrorException>(
            () => new ApproxAFunc2Var(1, 0, 0, Tol1(1e-6), null, null, Tol2(1e-6), null, null, 0, 1, 0, 1,
                                                 IsoV, C1, C1, 1, 8, 8, 4, anEval, aCut,
                                                 aCut));
    }
}
