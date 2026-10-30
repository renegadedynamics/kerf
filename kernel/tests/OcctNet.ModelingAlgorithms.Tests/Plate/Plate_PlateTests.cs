// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTest Plate_Plate (names kept) and unit tests of Plate_Plate.</summary>
public class Plate_PlateTests
{
    // ---- OCCT's GTests (src/ModelingAlgorithms/TKGeomAlgo/GTests/Plate_Plate_Test.cxx) ----

    [Fact]
    public void Init_ClearsState()
    {
        var aPlate = new Plate_Plate();

        // Add a constraint and solve.
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.0, 0.0), new gp_XYZ(0.0, 0.0, 1.0), 0, 0));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(1.0, 0.0), new gp_XYZ(0.0, 0.0, 0.0), 0, 0));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.0, 1.0), new gp_XYZ(0.0, 0.0, 0.0), 0, 0));
        aPlate.SolveTI(2);

        // After Init(), constraints and solution are cleared.
        aPlate.Init();
        // OCCT's TODO: IsDone() returns true after Init(), which is semantically wrong, but changing it caused
        // regressions in the blend/filling Draw tests.
        Assert.True(aPlate.IsDone());
    }

    [Fact]
    public void Init_EvaluateReturnsZero()
    {
        var aPlate = new Plate_Plate();
        aPlate.Init();

        gp_XYZ aResult = aPlate.Evaluate(new gp_XY(0.5, 0.5));
        Assert.Equal(0.0, aResult.X());
        Assert.Equal(0.0, aResult.Y());
        Assert.Equal(0.0, aResult.Z());
    }

    // ---- unit tests ----

    private static readonly gp_XY[] s_points =
    [
        new(0.0, 0.0), new(1.0, 0.0), new(0.0, 1.0), new(1.0, 1.0), new(0.5, 0.3), new(0.2, 0.8), new(0.9, 0.6),
        new(0.4, 0.55),
    ];

    private static gp_XYZ Bump(in gp_XY p)
        => new(0.1 * Math.Sin(3.0 * p.X()), 0.05 * p.Y() * p.Y(), Math.Exp(-((p.X() - 0.5) * (p.X() - 0.5)) - p.Y()));

    private static Plate_Plate SolvedBump(int theOrder, double theAniso = 1.0)
    {
        var aPlate = new Plate_Plate();
        foreach (gp_XY p in s_points)
        {
            aPlate.Load(new Plate_PinpointConstraint(p, Bump(p)));
        }
        aPlate.SolveTI(theOrder, theAniso);
        Assert.True(aPlate.IsDone());
        return aPlate;
    }

    private static void AssertNear(in gp_XYZ theExpected, in gp_XYZ theActual, double theTol)
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
        Plate_Plate aPlate = SolvedBump(theOrder);
        foreach (gp_XY p in s_points)
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
        Plate_Plate aPlate = SolvedBump(3, theAniso);
        foreach (gp_XY p in s_points)
        {
            AssertNear(Bump(p), aPlate.Evaluate(p), 1.0e-9);
        }
    }

    [Fact]
    public void SolveTI_ReproducesAffineData()
    {
        // a thin-plate spline of order 2 reproduces polynomials of degree 1: the plate is the affine map itself
        static gp_XYZ Affine(in gp_XY p) => new(1.0 + 2.0 * p.X() - p.Y(), 0.5 * p.X(), -3.0 + 0.25 * p.Y());
        var aPlate = new Plate_Plate();
        foreach (gp_XY p in s_points)
        {
            aPlate.Load(new Plate_PinpointConstraint(p, Affine(p)));
        }
        aPlate.SolveTI(2);
        Assert.True(aPlate.IsDone());
        foreach (gp_XY p in new gp_XY[] { new(-0.5, 0.25), new(0.33, 0.77), new(2.0, -1.0) })
        {
            AssertNear(Affine(p), aPlate.Evaluate(p), 1.0e-9);
        }
    }

    [Fact]
    public void EvaluateDerivative_MatchesFiniteDifferences()
    {
        Plate_Plate aPlate = SolvedBump(3);
        var p = new gp_XY(0.37, 0.41);
        const double h = 1.0e-5;
        gp_XYZ du = (aPlate.Evaluate(new gp_XY(p.X() + h, p.Y())) - aPlate.Evaluate(new gp_XY(p.X() - h, p.Y()))) / (2.0 * h);
        gp_XYZ dv = (aPlate.Evaluate(new gp_XY(p.X(), p.Y() + h)) - aPlate.Evaluate(new gp_XY(p.X(), p.Y() - h))) / (2.0 * h);
        AssertNear(du, aPlate.EvaluateDerivative(p, 1, 0), 1.0e-6);
        AssertNear(dv, aPlate.EvaluateDerivative(p, 0, 1), 1.0e-6);
        gp_XYZ duv = (aPlate.EvaluateDerivative(new gp_XY(p.X(), p.Y() + h), 1, 0)
                      - aPlate.EvaluateDerivative(new gp_XY(p.X(), p.Y() - h), 1, 0)) / (2.0 * h);
        AssertNear(duv, aPlate.EvaluateDerivative(p, 1, 1), 1.0e-5);
        Assert.Equal(aPlate.Evaluate(p), aPlate.EvaluateDerivative(p, 0, 0));
    }

    [Fact]
    public void DerivativeConstraints_AreMet()
    {
        var aPlate = new Plate_Plate();
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.0, 0.0), new gp_XYZ(0.0, 0.0, 0.0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(1.0, 0.0), new gp_XYZ(0.0, 0.0, 1.0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.0, 1.0), new gp_XYZ(0.0, 0.0, 0.5)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.5, 0.5), new gp_XYZ(0.1, 0.2, 0.3), 1, 0));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.5, 0.5), new gp_XYZ(-0.3, 0.0, 0.7), 0, 1));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.2, 0.7), new gp_XYZ(0.0, 1.0, 0.0), 1, 1));
        aPlate.SolveTI(4);
        Assert.True(aPlate.IsDone());
        AssertNear(new gp_XYZ(0.0, 0.0, 1.0), aPlate.Evaluate(new gp_XY(1.0, 0.0)), 1.0e-9);
        AssertNear(new gp_XYZ(0.1, 0.2, 0.3), aPlate.EvaluateDerivative(new gp_XY(0.5, 0.5), 1, 0), 1.0e-9);
        AssertNear(new gp_XYZ(-0.3, 0.0, 0.7), aPlate.EvaluateDerivative(new gp_XY(0.5, 0.5), 0, 1), 1.0e-9);
        AssertNear(new gp_XYZ(0.0, 1.0, 0.0), aPlate.EvaluateDerivative(new gp_XY(0.2, 0.7), 1, 1), 1.0e-9);
        Assert.Equal(2 * 4 - 3 - 2, aPlate.Continuity()); // the highest constraint order is 2
    }

    [Theory]
    [InlineData(1, 1.0)]
    [InlineData(10, 1.0)]
    [InlineData(3, 1.0e-7)]
    [InlineData(3, 1.0e7)]
    public void SolveTI_RejectsBadArguments(int theOrder, double theAniso)
    {
        var aPlate = new Plate_Plate();
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.0, 0.0), new gp_XYZ(0.0, 0.0, 0.0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(1.0, 1.0), new gp_XYZ(0.0, 0.0, 1.0)));
        aPlate.SolveTI(theOrder, theAniso);
        Assert.False(aPlate.IsDone());
        Assert.Equal(new gp_XYZ(0.0, 0.0, 0.0), aPlate.Evaluate(new gp_XY(0.5, 0.5)));
    }

    [Fact]
    public void SolveTI_WithoutConstraints_IsNotDone()
    {
        var aPlate = new Plate_Plate();
        aPlate.SolveTI();
        Assert.False(aPlate.IsDone());
    }

    [Fact]
    public void UVBox_WidensAFlatBoxTo1em3()
    {
        var aPlate = new Plate_Plate();
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.0, 0.0), new gp_XYZ(0.0, 0.0, 0.0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(1.0, 0.0), new gp_XYZ(0.0, 0.0, 1.0)));
        aPlate.UVBox(out double u1, out double u2, out double v1, out double v2);
        Assert.Equal((0.0, 1.0), (u1, u2));
        Assert.Equal(-0.5e-3, v1, 1.0e-18);
        Assert.Equal(0.5e-3, v2, 1.0e-18);
    }

    [Fact]
    public void UVConstraints_ListsOrderZeroPinpointsOnly()
    {
        var aPlate = new Plate_Plate();
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.1, 0.2), new gp_XYZ(0.0, 0.0, 0.0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.3, 0.4), new gp_XYZ(0.0, 0.0, 0.0), 1, 0));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.5, 0.6), new gp_XYZ(0.0, 0.0, 0.0)));
        var aSeq = new NCollection_Sequence<gp_XY>();
        aPlate.UVConstraints(aSeq);
        Assert.Equal(2, aSeq.Length());
        Assert.Equal(new gp_XY(0.1, 0.2), aSeq.Value(1));
        Assert.Equal(new gp_XY(0.5, 0.6), aSeq.Value(2));
    }

    [Fact]
    public void CopyConstructor_AndCopy_AreIndependent()
    {
        Plate_Plate aPlate = SolvedBump(3);
        var p = new gp_XY(0.31, 0.62);
        gp_XYZ aValue = aPlate.Evaluate(p);

        var aCopy = new Plate_Plate(aPlate);
        var anAssigned = new Plate_Plate();
        Assert.Same(anAssigned, anAssigned.Copy(aPlate));
        aPlate.Init();
        Assert.Equal(new gp_XYZ(0.0, 0.0, 0.0), aPlate.Evaluate(p));
        Assert.True(aCopy.IsDone());
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
        Plate_Plate aPlate = SolvedBump(3);
        var p = new gp_XY(0.7, 0.2);
        gp_XYZ aFull = aPlate.Evaluate(p);
        aPlate.SetPolynomialPartOnly();
        gp_XYZ aPoly = aPlate.Evaluate(p);
        Assert.False(aFull.IsEqual(aPoly, 1.0e-6));

        NCollection_HArray2<gp_XYZ> aCoefs = aPlate.CoefPol();
        Assert.Equal((0, 2, 0, 2), (aCoefs.LowerRow(), aCoefs.UpperRow(), aCoefs.LowerCol(), aCoefs.UpperCol()));
        gp_XYZ aSum = new(0.0, 0.0, 0.0);
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
        var aPlate = new Plate_Plate();
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.0, 0.0), new gp_XYZ(0.0, 0.0, 1.0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(1.0, 1.0), new gp_XYZ(0.0, 0.0, 0.0)));
        var aSeq = new NCollection_Sequence<gp_XY>();
        aSeq.Append(new gp_XY(1.0, 0.0));
        aSeq.Append(new gp_XY(0.0, 1.0));
        aSeq.Append(new gp_XY(0.5, 0.5));
        var aGT = new Plate_GlobalTranslationConstraint(aSeq);
        Assert.Equal(2, aGT.LXYZC().Coeff().ColLength());
        Assert.Equal(3, aGT.LXYZC().Coeff().RowLength());
        aPlate.Load(aGT);
        aPlate.SolveTI(3);
        Assert.True(aPlate.IsDone());
        AssertNear(new gp_XYZ(0.0, 0.0, 1.0), aPlate.Evaluate(new gp_XY(0.0, 0.0)), 1.0e-9);
        gp_XYZ a = aPlate.Evaluate(new gp_XY(1.0, 0.0));
        AssertNear(a, aPlate.Evaluate(new gp_XY(0.0, 1.0)), 1.0e-9);
        AssertNear(a, aPlate.Evaluate(new gp_XY(0.5, 0.5)), 1.0e-9);
    }

    [Fact]
    public void SampledCurveConstraint_MatchesTheAveragedValues()
    {
        var aSeq = new NCollection_Sequence<Plate_PinpointConstraint>();
        for (int i = 0; i < 7; i++)
        {
            double t = i / 6.0;
            aSeq.Append(new Plate_PinpointConstraint(new gp_XY(t, 0.5 + 0.2 * t), new gp_XYZ(0.0, 0.0, Math.Sin(3.0 * t))));
        }
        var aSC = new Plate_SampledCurveConstraint(aSeq, 3);
        Plate_LinearXYZConstraint aL = aSC.LXYZC();
        Assert.Equal(3, aL.Coeff().ColLength());
        Assert.Equal(7, aL.Coeff().RowLength());
        // B0(ratio * j - i), ratio = 4/8
        Assert.Equal(1.0, aL.Coeff()[1, 2]);
        Assert.Equal(0.5, aL.Coeff()[1, 1]);
        Assert.Equal(0.0, aL.Coeff()[1, 5]);

        var aPlate = new Plate_Plate();
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.0, 0.0), new gp_XYZ(0.0, 0.0, 0.0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(1.0, 0.0), new gp_XYZ(0.0, 0.0, 0.0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.0, 1.0), new gp_XYZ(0.0, 0.0, 0.0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(1.0, 1.0), new gp_XYZ(0.0, 0.0, 0.0)));
        aPlate.Load(aSC);
        aPlate.SolveTI(3);
        Assert.True(aPlate.IsDone());
        for (int i = 1; i <= 3; i++)
        {
            double aWanted = 0.0, aGot = 0.0;
            for (int j = 1; j <= 7; j++)
            {
                Plate_PinpointConstraint aP = aL.GetPPC()[j];
                aWanted += aL.Coeff()[i, j] * aP.Value().Z();
                aGot += aL.Coeff()[i, j] * aPlate.Evaluate(aP.Pnt2d()).Z();
            }
            Assert.Equal(aWanted, aGot, 1.0e-9);
        }
        Assert.Throws<Standard_DimensionMismatch>(() => new Plate_SampledCurveConstraint(aSeq, 8));
    }

    [Fact]
    public void PlaneAndLineConstraints_AreMet()
    {
        // SolveTI3: pinpoints with linear scalar constraints
        var aPln = new gp_Pln(new gp_Pnt(0.0, 0.0, 2.0), new gp_Dir(0.0, 1.0, 1.0));
        var aLin = new gp_Lin(new gp_Pnt(1.0, 2.0, 3.0), new gp_Dir(1.0, 1.0, 0.0));
        var aPlate = new Plate_Plate();
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.0, 0.0), new gp_XYZ(0.0, 0.0, 0.0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(1.0, 0.0), new gp_XYZ(0.0, 0.0, 1.0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.0, 1.0), new gp_XYZ(0.0, 0.0, 0.0)));
        aPlate.Load(new Plate_PlaneConstraint(new gp_XY(0.5, 0.5), aPln));
        aPlate.Load(new Plate_LineConstraint(new gp_XY(0.9, 0.8), aLin));
        aPlate.SolveTI(3);
        Assert.True(aPlate.IsDone());
        gp_XYZ aOnPlane = aPlate.Evaluate(new gp_XY(0.5, 0.5));
        Assert.Equal(0.0, aPln.Distance(new gp_Pnt(aOnPlane)), 1.0e-9);
        gp_XYZ aOnLine = aPlate.Evaluate(new gp_XY(0.9, 0.8));
        Assert.Equal(0.0, aLin.Distance(new gp_Pnt(aOnLine)), 1.0e-9);
        AssertNear(new gp_XYZ(0.0, 0.0, 1.0), aPlate.Evaluate(new gp_XY(1.0, 0.0)), 1.0e-9);
    }

    [Fact]
    public void MixedLinearConstraints_AreMet()
    {
        // SolveTI3 with both kinds of linear constraints
        var aPln = new gp_Pln(new gp_Pnt(0.0, 0.0, 0.5), new gp_Dir(0.0, 0.0, 1.0));
        var aPlate = new Plate_Plate();
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.0, 0.0), new gp_XYZ(0.0, 0.0, 0.0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(1.0, 1.0), new gp_XYZ(0.0, 0.0, 1.0)));
        var aSeq = new NCollection_Sequence<gp_XY>();
        aSeq.Append(new gp_XY(1.0, 0.0));
        aSeq.Append(new gp_XY(0.0, 1.0));
        aPlate.Load(new Plate_GlobalTranslationConstraint(aSeq));
        aPlate.Load(new Plate_PlaneConstraint(new gp_XY(0.3, 0.6), aPln));
        aPlate.SolveTI(3);
        Assert.True(aPlate.IsDone());
        AssertNear(aPlate.Evaluate(new gp_XY(1.0, 0.0)), aPlate.Evaluate(new gp_XY(0.0, 1.0)), 1.0e-9);
        Assert.Equal(0.5, aPlate.Evaluate(new gp_XY(0.3, 0.6)).Z(), 1.0e-9);
        AssertNear(new gp_XYZ(0.0, 0.0, 1.0), aPlate.Evaluate(new gp_XY(1.0, 1.0)), 1.0e-9);
    }

    [Fact]
    public void SolveTI_ProgressCancellation_LeavesThePlateNotDone()
    {
        using var aSource = new CancellationTokenSource();
        aSource.Cancel();
        var anIndicator = new Message_ProgressIndicator(aSource.Token);
        var aPlate = new Plate_Plate();
        foreach (gp_XY p in s_points)
        {
            aPlate.Load(new Plate_PinpointConstraint(p, Bump(p)));
        }
        aPlate.SolveTI(3, 1.0, anIndicator.Start());
        Assert.False(aPlate.IsDone());
    }
}
