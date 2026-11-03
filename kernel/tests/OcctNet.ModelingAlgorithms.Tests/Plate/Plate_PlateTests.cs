// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Plate;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTest Plate_Plate (names kept) and unit tests of Plate_Plate.</summary>
public class Plate_PlateTests
{
    // ---- OCCT's GTests (src/ModelingAlgorithms/TKGeomAlgo/GTests/Plate_Plate_Test.cxx) ----

    [Fact]
    public void Init_ClearsState()
    {
        var aPlate = new PlateSolver();

        // Add a constraint and solve.
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 0.0), new Xyz(0.0, 0.0, 1.0), 0, 0));
        aPlate.Load(new PinpointConstraint(new Xy(1.0, 0.0), new Xyz(0.0, 0.0, 0.0), 0, 0));
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 1.0), new Xyz(0.0, 0.0, 0.0), 0, 0));
        aPlate.SolveTI(2);

        // After Init(), constraints and solution are cleared.
        aPlate.Init();
        // OCCT's TODO: IsDone() returns true after Init(), which is semantically wrong, but changing it caused
        // regressions in the blend/filling Draw tests.
        Assert.True(aPlate.IsDone);
    }

    [Fact]
    public void Init_EvaluateReturnsZero()
    {
        var aPlate = new PlateSolver();
        aPlate.Init();

        Xyz aResult = aPlate.Evaluate(new Xy(0.5, 0.5));
        Assert.Equal(0.0, aResult.X());
        Assert.Equal(0.0, aResult.Y());
        Assert.Equal(0.0, aResult.Z());
    }

    // ---- unit tests ----

    private static readonly Xy[] s_points =
    [
        new(0.0, 0.0), new(1.0, 0.0), new(0.0, 1.0), new(1.0, 1.0), new(0.5, 0.3), new(0.2, 0.8), new(0.9, 0.6),
        new(0.4, 0.55),
    ];

    private static Xyz Bump(in Xy p)
        => new(0.1 * Math.Sin(3.0 * p.X()), 0.05 * p.Y() * p.Y(), Math.Exp(-((p.X() - 0.5) * (p.X() - 0.5)) - p.Y()));

    private static PlateSolver SolvedBump(int theOrder, double theAniso = 1.0)
    {
        var aPlate = new PlateSolver();
        foreach (Xy p in s_points)
        {
            aPlate.Load(new PinpointConstraint(p, Bump(p)));
        }
        aPlate.SolveTI(theOrder, theAniso);
        Assert.True(aPlate.IsDone);
        return aPlate;
    }

    private static void AssertNear(in Xyz theExpected, in Xyz theActual, double theTol)
    {
        Assert.True(theExpected.IsEqual(theActual, theTol), $"expected {theExpected}, got {theActual}");
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    public void SolveTI_InterpolatesPinpointConstraints(int theOrder)
    {
        PlateSolver aPlate = SolvedBump(theOrder);
        foreach (Xy p in s_points)
        {
            AssertNear(Bump(p), aPlate.Evaluate(p), 1.0e-9);
        }
        Assert.Equal(2 * theOrder - 3, aPlate.Continuity());
    }

    [Theory]
    [InlineData(0.5)]
    [InlineData(2.0)]
    public void SolveTI_WithAnisotropy_InterpolatesPinpointConstraints(double theAniso)
    {
        PlateSolver aPlate = SolvedBump(3, theAniso);
        foreach (Xy p in s_points)
        {
            AssertNear(Bump(p), aPlate.Evaluate(p), 1.0e-9);
        }
    }

    [Fact]
    public void SolveTI_ReproducesAffineData()
    {
        // a thin-plate spline of order 2 reproduces polynomials of degree 1: the plate is the affine map itself
        static Xyz Affine(in Xy p) => new(1.0 + 2.0 * p.X() - p.Y(), 0.5 * p.X(), -3.0 + 0.25 * p.Y());
        var aPlate = new PlateSolver();
        foreach (Xy p in s_points)
        {
            aPlate.Load(new PinpointConstraint(p, Affine(p)));
        }
        aPlate.SolveTI(2);
        Assert.True(aPlate.IsDone);
        foreach (Xy p in new Xy[] { new(-0.5, 0.25), new(0.33, 0.77), new(2.0, -1.0) })
        {
            AssertNear(Affine(p), aPlate.Evaluate(p), 1.0e-9);
        }
    }

    [Fact]
    public void EvaluateDerivative_MatchesFiniteDifferences()
    {
        PlateSolver aPlate = SolvedBump(3);
        var p = new Xy(0.37, 0.41);
        const double h = 1.0e-5;
        Xyz du = (aPlate.Evaluate(new Xy(p.X() + h, p.Y())) - aPlate.Evaluate(new Xy(p.X() - h, p.Y()))) / (2.0 * h);
        Xyz dv = (aPlate.Evaluate(new Xy(p.X(), p.Y() + h)) - aPlate.Evaluate(new Xy(p.X(), p.Y() - h))) / (2.0 * h);
        AssertNear(du, aPlate.EvaluateDerivative(p, 1, 0), 1.0e-6);
        AssertNear(dv, aPlate.EvaluateDerivative(p, 0, 1), 1.0e-6);
        Xyz duv = (aPlate.EvaluateDerivative(new Xy(p.X(), p.Y() + h), 1, 0)
                      - aPlate.EvaluateDerivative(new Xy(p.X(), p.Y() - h), 1, 0)) / (2.0 * h);
        AssertNear(duv, aPlate.EvaluateDerivative(p, 1, 1), 1.0e-5);
        Assert.Equal(aPlate.Evaluate(p), aPlate.EvaluateDerivative(p, 0, 0));
    }

    [Fact]
    public void DerivativeConstraints_AreMet()
    {
        var aPlate = new PlateSolver();
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 0.0), new Xyz(0.0, 0.0, 0.0)));
        aPlate.Load(new PinpointConstraint(new Xy(1.0, 0.0), new Xyz(0.0, 0.0, 1.0)));
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 1.0), new Xyz(0.0, 0.0, 0.5)));
        aPlate.Load(new PinpointConstraint(new Xy(0.5, 0.5), new Xyz(0.1, 0.2, 0.3), 1, 0));
        aPlate.Load(new PinpointConstraint(new Xy(0.5, 0.5), new Xyz(-0.3, 0.0, 0.7), 0, 1));
        aPlate.Load(new PinpointConstraint(new Xy(0.2, 0.7), new Xyz(0.0, 1.0, 0.0), 1, 1));
        aPlate.SolveTI(4);
        Assert.True(aPlate.IsDone);
        AssertNear(new Xyz(0.0, 0.0, 1.0), aPlate.Evaluate(new Xy(1.0, 0.0)), 1.0e-9);
        AssertNear(new Xyz(0.1, 0.2, 0.3), aPlate.EvaluateDerivative(new Xy(0.5, 0.5), 1, 0), 1.0e-9);
        AssertNear(new Xyz(-0.3, 0.0, 0.7), aPlate.EvaluateDerivative(new Xy(0.5, 0.5), 0, 1), 1.0e-9);
        AssertNear(new Xyz(0.0, 1.0, 0.0), aPlate.EvaluateDerivative(new Xy(0.2, 0.7), 1, 1), 1.0e-9);
        Assert.Equal(2 * 4 - 3 - 2, aPlate.Continuity()); // the highest constraint order is 2
    }

    [Theory]
    [InlineData(1, 1.0)]
    [InlineData(10, 1.0)]
    [InlineData(3, 1.0e-7)]
    [InlineData(3, 1.0e7)]
    public void SolveTI_RejectsBadArguments(int theOrder, double theAniso)
    {
        var aPlate = new PlateSolver();
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 0.0), new Xyz(0.0, 0.0, 0.0)));
        aPlate.Load(new PinpointConstraint(new Xy(1.0, 1.0), new Xyz(0.0, 0.0, 1.0)));
        aPlate.SolveTI(theOrder, theAniso);
        Assert.False(aPlate.IsDone);
        Assert.Equal(new Xyz(0.0, 0.0, 0.0), aPlate.Evaluate(new Xy(0.5, 0.5)));
    }

    [Fact]
    public void SolveTI_WithoutConstraints_IsNotDone()
    {
        var aPlate = new PlateSolver();
        aPlate.SolveTI();
        Assert.False(aPlate.IsDone);
    }

    [Fact]
    public void UVBox_WidensAFlatBoxTo1em3()
    {
        var aPlate = new PlateSolver();
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 0.0), new Xyz(0.0, 0.0, 0.0)));
        aPlate.Load(new PinpointConstraint(new Xy(1.0, 0.0), new Xyz(0.0, 0.0, 1.0)));
        aPlate.UVBox(out double u1, out double u2, out double v1, out double v2);
        Assert.Equal((0.0, 1.0), (u1, u2));
        Assert.Equal(-0.5e-3, v1, 1.0e-18);
        Assert.Equal(0.5e-3, v2, 1.0e-18);
    }

    [Fact]
    public void UVConstraints_ListsOrderZeroPinpointsOnly()
    {
        var aPlate = new PlateSolver();
        aPlate.Load(new PinpointConstraint(new Xy(0.1, 0.2), new Xyz(0.0, 0.0, 0.0)));
        aPlate.Load(new PinpointConstraint(new Xy(0.3, 0.4), new Xyz(0.0, 0.0, 0.0), 1, 0));
        aPlate.Load(new PinpointConstraint(new Xy(0.5, 0.6), new Xyz(0.0, 0.0, 0.0)));
        var aSeq = new NCollection_Sequence<Xy>();
        aPlate.UVConstraints(aSeq);
        Assert.Equal(2, aSeq.Length());
        Assert.Equal(new Xy(0.1, 0.2), aSeq.Value(1));
        Assert.Equal(new Xy(0.5, 0.6), aSeq.Value(2));
    }

    [Fact]
    public void CopyConstructor_AndCopy_AreIndependent()
    {
        PlateSolver aPlate = SolvedBump(3);
        var p = new Xy(0.31, 0.62);
        Xyz aValue = aPlate.Evaluate(p);

        var aCopy = new PlateSolver(aPlate);
        var anAssigned = new PlateSolver();
        Assert.Same(anAssigned, anAssigned.Copy(aPlate));
        aPlate.Init();
        Assert.Equal(new Xyz(0.0, 0.0, 0.0), aPlate.Evaluate(p));
        Assert.True(aCopy.IsDone);
        Assert.Equal(aValue, aCopy.Evaluate(p));
        Assert.Equal(aValue, anAssigned.Evaluate(p));
        Assert.Equal(aValue.X(), anAssigned.EvaluateDerivative(p, 0, 0).X());

        // the copy kept the constraints: solving it again gives the same plate
        aCopy.SolveTI(3);
        Assert.Equal(aValue, aCopy.Evaluate(p));
    }

    [Fact]
    public void PolynomialPartOnly_AndCoefPol_AgreeOnThePolynomialPart()
    {
        PlateSolver aPlate = SolvedBump(3);
        var p = new Xy(0.7, 0.2);
        Xyz aFull = aPlate.Evaluate(p);
        aPlate.SetPolynomialPartOnly();
        Xyz aPoly = aPlate.Evaluate(p);
        Assert.False(aFull.IsEqual(aPoly, 1.0e-6));

        NCollection_HArray2<Xyz> aCoefs = aPlate.CoefPol();
        Assert.Equal((0, 2, 0, 2), (aCoefs.LowerRow(), aCoefs.UpperRow(), aCoefs.LowerCol(), aCoefs.UpperCol()));
        Xyz aSum = new(0.0, 0.0, 0.0);
        for (int iu = 0; iu < 3; iu++)
        {
            for (int iv = 0; iu + iv < 3; iv++)
            {
                aSum += aCoefs.Value(iu, iv) * (Math.Pow(p.X(), iu) * Math.Pow(p.Y(), iv));
            }
        }
        AssertNear(aPoly, aSum, 1.0e-12);
        aPlate.SetPolynomialPartOnly(false);
        Assert.Equal(aFull, aPlate.Evaluate(p));
    }

    [Fact]
    public void GlobalTranslationConstraint_MovesThePointsTogether()
    {
        // SolveTI2: one pinpoint and a translation constraint on three more points
        var aPlate = new PlateSolver();
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 0.0), new Xyz(0.0, 0.0, 1.0)));
        aPlate.Load(new PinpointConstraint(new Xy(1.0, 1.0), new Xyz(0.0, 0.0, 0.0)));
        var aSeq = new NCollection_Sequence<Xy>();
        aSeq.Append(new Xy(1.0, 0.0));
        aSeq.Append(new Xy(0.0, 1.0));
        aSeq.Append(new Xy(0.5, 0.5));
        var aGT = new GlobalTranslationConstraint(aSeq);
        Assert.Equal(2, aGT.LXYZC().Coeff().ColLength());
        Assert.Equal(3, aGT.LXYZC().Coeff().RowLength());
        aPlate.Load(aGT);
        aPlate.SolveTI(3);
        Assert.True(aPlate.IsDone);
        AssertNear(new Xyz(0.0, 0.0, 1.0), aPlate.Evaluate(new Xy(0.0, 0.0)), 1.0e-9);
        Xyz a = aPlate.Evaluate(new Xy(1.0, 0.0));
        AssertNear(a, aPlate.Evaluate(new Xy(0.0, 1.0)), 1.0e-9);
        AssertNear(a, aPlate.Evaluate(new Xy(0.5, 0.5)), 1.0e-9);
    }

    [Fact]
    public void SampledCurveConstraint_MatchesTheAveragedValues()
    {
        var aSeq = new NCollection_Sequence<PinpointConstraint>();
        for (int i = 0; i < 7; i++)
        {
            double t = i / 6.0;
            aSeq.Append(new PinpointConstraint(new Xy(t, 0.5 + 0.2 * t), new Xyz(0.0, 0.0, Math.Sin(3.0 * t))));
        }
        var aSC = new SampledCurveConstraint(aSeq, 3);
        LinearXyzConstraint aL = aSC.LXYZC();
        Assert.Equal(3, aL.Coeff().ColLength());
        Assert.Equal(7, aL.Coeff().RowLength());
        // B0(ratio * j - i), ratio = 4/8
        Assert.Equal(1.0, aL.Coeff()[1, 2]);
        Assert.Equal(0.5, aL.Coeff()[1, 1]);
        Assert.Equal(0.0, aL.Coeff()[1, 5]);

        var aPlate = new PlateSolver();
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 0.0), new Xyz(0.0, 0.0, 0.0)));
        aPlate.Load(new PinpointConstraint(new Xy(1.0, 0.0), new Xyz(0.0, 0.0, 0.0)));
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 1.0), new Xyz(0.0, 0.0, 0.0)));
        aPlate.Load(new PinpointConstraint(new Xy(1.0, 1.0), new Xyz(0.0, 0.0, 0.0)));
        aPlate.Load(aSC);
        aPlate.SolveTI(3);
        Assert.True(aPlate.IsDone);
        for (int i = 1; i <= 3; i++)
        {
            double aWanted = 0.0, aGot = 0.0;
            for (int j = 1; j <= 7; j++)
            {
                PinpointConstraint aP = aL.GetPPC()[j];
                aWanted += aL.Coeff()[i, j] * aP.Value().Z();
                aGot += aL.Coeff()[i, j] * aPlate.Evaluate(aP.Pnt2d()).Z();
            }
            Assert.Equal(aWanted, aGot, 1.0e-9);
        }
        Assert.Throws<DimensionMismatchException>(() => new SampledCurveConstraint(aSeq, 8));
    }

    [Fact]
    public void PlaneAndLineConstraints_AreMet()
    {
        // SolveTI3: pinpoints with linear scalar constraints
        var aPln = new Plane3D(new Point3D(0.0, 0.0, 2.0), new Direction3D(0.0, 1.0, 1.0));
        var aLin = new Line3D(new Point3D(1.0, 2.0, 3.0), new Direction3D(1.0, 1.0, 0.0));
        var aPlate = new PlateSolver();
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 0.0), new Xyz(0.0, 0.0, 0.0)));
        aPlate.Load(new PinpointConstraint(new Xy(1.0, 0.0), new Xyz(0.0, 0.0, 1.0)));
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 1.0), new Xyz(0.0, 0.0, 0.0)));
        aPlate.Load(new PlaneConstraint(new Xy(0.5, 0.5), aPln));
        aPlate.Load(new LineConstraint(new Xy(0.9, 0.8), aLin));
        aPlate.SolveTI(3);
        Assert.True(aPlate.IsDone);
        Xyz aOnPlane = aPlate.Evaluate(new Xy(0.5, 0.5));
        Assert.Equal(0.0, aPln.Distance(new Point3D(aOnPlane)), 1.0e-9);
        Xyz aOnLine = aPlate.Evaluate(new Xy(0.9, 0.8));
        Assert.Equal(0.0, aLin.Distance(new Point3D(aOnLine)), 1.0e-9);
        AssertNear(new Xyz(0.0, 0.0, 1.0), aPlate.Evaluate(new Xy(1.0, 0.0)), 1.0e-9);
    }

    [Fact]
    public void MixedLinearConstraints_AreMet()
    {
        // SolveTI3 with both kinds of linear constraints
        var aPln = new Plane3D(new Point3D(0.0, 0.0, 0.5), new Direction3D(0.0, 0.0, 1.0));
        var aPlate = new PlateSolver();
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 0.0), new Xyz(0.0, 0.0, 0.0)));
        aPlate.Load(new PinpointConstraint(new Xy(1.0, 1.0), new Xyz(0.0, 0.0, 1.0)));
        var aSeq = new NCollection_Sequence<Xy>();
        aSeq.Append(new Xy(1.0, 0.0));
        aSeq.Append(new Xy(0.0, 1.0));
        aPlate.Load(new GlobalTranslationConstraint(aSeq));
        aPlate.Load(new PlaneConstraint(new Xy(0.3, 0.6), aPln));
        aPlate.SolveTI(3);
        Assert.True(aPlate.IsDone);
        AssertNear(aPlate.Evaluate(new Xy(1.0, 0.0)), aPlate.Evaluate(new Xy(0.0, 1.0)), 1.0e-9);
        Assert.Equal(0.5, aPlate.Evaluate(new Xy(0.3, 0.6)).Z(), 1.0e-9);
        AssertNear(new Xyz(0.0, 0.0, 1.0), aPlate.Evaluate(new Xy(1.0, 1.0)), 1.0e-9);
    }

    [Fact]
    public void SolveTI_ProgressCancellation_LeavesThePlateNotDone()
    {
        using var aSource = new CancellationTokenSource();
        aSource.Cancel();
        var anIndicator = new ProgressIndicator(aSource.Token);
        var aPlate = new PlateSolver();
        foreach (Xy p in s_points)
        {
            aPlate.Load(new PinpointConstraint(p, Bump(p)));
        }
        aPlate.SolveTI(3, 1.0, anIndicator.Start());
        Assert.False(aPlate.IsDone);
    }
}
