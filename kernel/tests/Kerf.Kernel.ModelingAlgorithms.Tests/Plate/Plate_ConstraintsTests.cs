// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Plate;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>Plate's constraint classes and derivative values.</summary>
public class Plate_ConstraintsTests
{
    private static readonly Xyz s_zero = new(0.0, 0.0, 0.0);

    [Fact]
    public void PinpointConstraint_IsAValue()
    {
        var a = new PinpointConstraint(new Xy(1.0, 2.0), new Xyz(3.0, 4.0, 5.0), 1, 2);
        Assert.Equal(new Xy(1.0, 2.0), a.Pnt2d());
        Assert.Equal(new Xyz(3.0, 4.0, 5.0), a.Value());
        Assert.Equal((1, 2), (a.Idu(), a.Idv()));
        Assert.Equal(a, new PinpointConstraint(new Xy(1.0, 2.0), new Xyz(3.0, 4.0, 5.0), 1, 2));
        Assert.NotEqual(a, new PinpointConstraint(new Xy(1.0, 2.0), new Xyz(3.0, 4.0, 5.0), 1, 1));
        Assert.Equal(new PinpointConstraint(), default);
        Assert.Equal((0, 0), (new PinpointConstraint(new Xy(), s_zero).Idu(), new PinpointConstraint(new Xy(), s_zero).Idv()));
    }

    [Fact]
    public void Derivatives_AreValues()
    {
        var d1 = new D1(new Xyz(1.0, 0.0, 0.0), new Xyz(0.0, 1.0, 0.0));
        Assert.Equal(new Xyz(1.0, 0.0, 0.0), d1.DU());
        Assert.Equal(new Xyz(0.0, 1.0, 0.0), d1.DV());
        Assert.Equal(d1, new D1(d1));
        var d2 = new D2(s_zero, new Xyz(1.0, 1.0, 1.0), s_zero);
        Assert.Equal(d2, new D2(d2));
        Assert.NotEqual(d2, new D2(s_zero, s_zero, s_zero));
        var d3 = new D3(s_zero, s_zero, s_zero, new Xyz(0.0, 0.0, 2.0));
        Assert.Equal(d3, new D3(d3));
    }

    [Fact]
    public void LinearXYZConstraint_FromArrays()
    {
        var aPPC = new NCollection_Array1<PinpointConstraint>(1, 2);
        aPPC[1] = new PinpointConstraint(new Xy(0.0, 0.0), s_zero);
        aPPC[2] = new PinpointConstraint(new Xy(1.0, 0.0), s_zero);
        var aCoeff = new NCollection_Array1<double>(5, 6);
        aCoeff[5] = 1.0;
        aCoeff[6] = -1.0;
        var aC = new LinearXyzConstraint(aPPC, aCoeff);
        Assert.Equal(1, aC.Coeff().ColLength());
        Assert.Equal(2, aC.Coeff().RowLength());
        Assert.Equal(1.0, aC.Coeff()[1, 1]);
        Assert.Equal(-1.0, aC.Coeff()[1, 2]);
        Assert.Equal(aPPC[2], aC.GetPPC()[2]);
        aC.SetCoeff(1, 2, 3.0);
        Assert.Equal(3.0, aC.Coeff()[1, 2]);

        var a2 = new NCollection_Array2<double>(1, 2, 1, 2);
        a2.Init(0.5);
        var aC2 = new LinearXyzConstraint(aPPC, a2);
        Assert.Equal(2, aC2.Coeff().ColLength());
        EngineAssert.Throws<DimensionMismatchException>(() => new LinearXyzConstraint(aPPC, new NCollection_Array1<double>(1, 3)));
        EngineAssert.Throws<DimensionMismatchException>(() => new LinearXyzConstraint(aPPC, new NCollection_Array2<double>(1, 2, 1, 3)));

        var aC3 = new LinearXyzConstraint(2, 3);
        Assert.Equal(0.0, aC3.Coeff()[2, 3]);
        Assert.Equal(new PinpointConstraint(), aC3.GetPPC()[3]);
    }

    [Fact]
    public void LinearScalarConstraint_FromArrays()
    {
        var aP = new PinpointConstraint(new Xy(0.5, 0.5), new Xyz(1.0, 2.0, 3.0));
        var aC = new LinearScalarConstraint(aP, new Xyz(0.0, 0.0, 1.0));
        Assert.Equal(aP, aC.GetPPC()[1]);
        Assert.Equal(new Xyz(0.0, 0.0, 1.0), aC.Coeff()[1, 1]);

        var aPPC = new NCollection_Array1<PinpointConstraint>(1, 2);
        var aCoeff = new NCollection_Array1<Xyz>(1, 2);
        aCoeff[2] = new Xyz(1.0, 0.0, 0.0);
        var aC2 = new LinearScalarConstraint(aPPC, aCoeff);
        Assert.Equal(new Xyz(1.0, 0.0, 0.0), aC2.Coeff()[1, 2]);
        EngineAssert.Throws<DimensionMismatchException>(() => new LinearScalarConstraint(aPPC, new NCollection_Array2<Xyz>(1, 1, 1, 3)));
        var aC3 = new LinearScalarConstraint(2, 1);
        aC3.SetCoeff(2, 1, new Xyz(0.0, 1.0, 0.0));
        aC3.SetPPC(1, aP);
        Assert.Equal(new Xyz(0.0, 1.0, 0.0), aC3.Coeff()[2, 1]);
        Assert.Equal(s_zero, aC3.Coeff()[1, 1]);
        Assert.Equal(aP, aC3.GetPPC()[1]);
    }

    [Fact]
    public void LineConstraint_HasTwoNormalsOfTheLine()
    {
        var aDir = new Direction3D(1.0, 2.0, 3.0);
        var aC = new LineConstraint(new Xy(0.2, 0.3), new Line3D(new Point3D(1.0, 1.0, 1.0), aDir), 0, 1);
        LinearScalarConstraint aL = aC.LSC();
        Assert.Equal((2, 1), (aL.Coeff().ColLength(), aL.Coeff().RowLength()));
        Xyz n1 = aL.Coeff()[1, 1], n2 = aL.Coeff()[2, 1];
        Assert.Equal(1.0, n1.Modulus(), 1.0e-15);
        Assert.Equal(1.0, n2.Modulus(), 1.0e-15);
        Assert.Equal(0.0, n1 * aDir.XYZ(), 1.0e-15);
        Assert.Equal(0.0, n2 * aDir.XYZ(), 1.0e-15);
        Assert.Equal(0.0, n1 * n2, 1.0e-15);
        Assert.Equal((0, 1), (aL.GetPPC()[1].Idu(), aL.GetPPC()[1].Idv()));
        Assert.Equal(new Xyz(1.0, 1.0, 1.0), aL.GetPPC()[1].Value());
    }

    [Fact]
    public void PlaneConstraint_HasThePlaneNormal()
    {
        var aC = new PlaneConstraint(new Xy(0.2, 0.3), new Plane3D(new Point3D(0.0, 0.0, 4.0), new Direction3D(0.0, 0.0, 1.0)));
        Assert.Equal(new Xyz(0.0, 0.0, 1.0), aC.LSC().Coeff()[1, 1]);
        Assert.Equal(new Xyz(0.0, 0.0, 4.0), aC.LSC().GetPPC()[1].Value());
    }

    [Fact]
    public void GlobalTranslationConstraint_Coefficients()
    {
        var aSeq = new NCollection_Sequence<Xy>();
        for (int i = 0; i < 4; i++)
        {
            aSeq.Append(new Xy(i, 1.0));
        }
        LinearXyzConstraint aL = new GlobalTranslationConstraint(aSeq).LXYZC();
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                Assert.Equal(j == 1 ? -1.0 : j == i + 1 ? 1.0 : 0.0, aL.Coeff()[i, j]);
            }
        }
    }

    // A target tangent plane tilted from the surface's (the xy plane)
    private static readonly D1 s_flat = new(new Xyz(1.0, 0.0, 0.0), new Xyz(0.0, 1.0, 0.0));
    private static readonly D1 s_tilted = new(new Xyz(1.0, 0.0, 0.3), new Xyz(0.0, 1.0, -0.2));
    private static readonly D2 s_d2s = new(new Xyz(0.0, 0.0, 0.1), s_zero, s_zero);
    private static readonly D2 s_d2t = new(s_zero, new Xyz(0.0, 0.0, 0.2), new Xyz(0.1, 0.0, 0.0));
    private static readonly D3 s_d3s = new(s_zero, s_zero, s_zero, s_zero);
    private static readonly D3 s_d3t = new(new Xyz(0.0, 0.0, 1.0), s_zero, s_zero, s_zero);

    [Fact]
    public void GtoCConstraint_MakesTheSurfaceTangentToTheTarget()
    {
        var p = new Xy(0.4, 0.6);
        var aG1 = new GtoCConstraint(p, s_flat, s_tilted);
        Assert.Equal(2, aG1.nb_PPC());
        Assert.Equal(s_flat, aG1.D1SurfInit());
        // the corrected derivatives lie in the target's tangent plane
        Xyz aN = (s_tilted.DU() ^ s_tilted.DV()).Normalized();
        Assert.Equal(0.0, (s_flat.DU() + aG1.GetPPC(0).Value()) * aN, 1.0e-15);
        Assert.Equal(0.0, (s_flat.DV() + aG1.GetPPC(1).Value()) * aN, 1.0e-15);
        Assert.Equal((1, 0), (aG1.GetPPC(0).Idu(), aG1.GetPPC(0).Idv()));
        Assert.Equal((0, 1), (aG1.GetPPC(1).Idu(), aG1.GetPPC(1).Idv()));

        Assert.Equal(5, new GtoCConstraint(p, s_flat, s_tilted, s_d2s, s_d2t).nb_PPC());
        Assert.Equal(9, new GtoCConstraint(p, s_flat, s_tilted, s_d2s, s_d2t, s_d3s, s_d3t).nb_PPC());
        var aNP = new Xyz(1.0, 0.0, 0.0);
        Assert.Equal(2, new GtoCConstraint(p, s_flat, s_tilted, aNP).nb_PPC());
        Assert.Equal(5, new GtoCConstraint(p, s_flat, s_tilted, s_d2s, s_d2t, aNP).nb_PPC());
        var aG3 = new GtoCConstraint(p, s_flat, s_tilted, s_d2s, s_d2t, s_d3s, s_d3t);
        var aG3P = new GtoCConstraint(p, s_flat, s_tilted, s_d2s, s_d2t, s_d3s, s_d3t, aNP);
        for (int i = 0; i < 9; i++)
        {
            Assert.Equal(aG3.GetPPC(i), aG3P.GetPPC(i)); // OCCT ignores nP in this variant
        }

        var aCopy = new GtoCConstraint(aG3);
        Assert.Equal(9, aCopy.nb_PPC());
        Assert.Equal(aG3.GetPPC(8), aCopy.GetPPC(8));
    }

    [Fact]
    public void GtoCConstraint_DegenerateOrOrthogonalNormals_GiveNoConstraint()
    {
        var p = new Xy(0.0, 0.0);
        var aDegenerate = new D1(new Xyz(1.0, 0.0, 0.0), new Xyz(2.0, 0.0, 0.0));
        Assert.Equal(0, new GtoCConstraint(p, s_flat, aDegenerate).nb_PPC());
        Assert.Equal(0, new GtoCConstraint(p, aDegenerate, s_flat).nb_PPC());
        var anOrthogonal = new D1(new Xyz(1.0, 0.0, 0.0), new Xyz(0.0, 0.0, 1.0));
        Assert.Equal(0, new GtoCConstraint(p, s_flat, anOrthogonal).nb_PPC());
    }

    [Fact]
    public void FreeGtoCConstraint_ImposesNormalComponents()
    {
        var p = new Xy(0.4, 0.6);
        var aG1 = new FreeGtoCConstraint(p, s_flat, s_tilted);
        Assert.Equal((0, 2), (aG1.nb_PPC(), aG1.nb_LSC()));
        Xyz aN = (s_tilted.DU() ^ s_tilted.DV()).Normalized();
        Assert.True(aG1.LSC(0).Coeff()[1, 1].IsEqual(aN, 1.0e-15));
        Assert.Equal(s_flat.DU() * -1.0, aG1.LSC(0).GetPPC()[1].Value());

        var aG2 = new FreeGtoCConstraint(p, s_flat, s_tilted, s_d2s, s_d2t);
        Assert.Equal((2, 3), (aG2.nb_PPC(), aG2.nb_LSC()));
        var aG3 = new FreeGtoCConstraint(p, s_flat, s_tilted, s_d2s, s_d2t, s_d3s, s_d3t);
        Assert.Equal((5, 4), (aG3.nb_PPC(), aG3.nb_LSC()));

        // an incremental load rotates the target's normal part of the way towards the surface's
        var aHalf = new FreeGtoCConstraint(p, s_flat, s_tilted, 0.5);
        Assert.Equal(2, aHalf.nb_LSC());
        Xyz aNHalf = aHalf.LSC(0).Coeff()[1, 1];
        double aAngleFull = Math.Acos(aN.Z()), aAngleHalf = Math.Acos(aNHalf.Z());
        Assert.True(aAngleHalf < aAngleFull && aAngleHalf > 0.0);

        // a degenerate surface normal: the G1 constraints only
        var aDegenerate = new D1(new Xyz(1.0, 0.0, 0.0), new Xyz(2.0, 0.0, 0.0));
        var aDeg = new FreeGtoCConstraint(p, aDegenerate, s_tilted, s_d2s, s_d2t);
        Assert.Equal((0, 2), (aDeg.nb_PPC(), aDeg.nb_LSC()));
        Assert.Equal((0, 0), (new FreeGtoCConstraint(p, aDegenerate, s_tilted, s_d2s, s_d2t, 0.5).nb_PPC(),
                              new FreeGtoCConstraint(p, aDegenerate, s_tilted, s_d2s, s_d2t, 0.5).nb_LSC()));
    }

    [Fact]
    public void GtoCConstraint_InAPlate_MakesItTangent()
    {
        var aPlate = new PlateSolver();
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 0.0), s_zero));
        aPlate.Load(new PinpointConstraint(new Xy(1.0, 0.0), s_zero));
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 1.0), s_zero));
        aPlate.Load(new PinpointConstraint(new Xy(1.0, 1.0), s_zero));
        var p = new Xy(0.5, 0.5);
        aPlate.Load(new PinpointConstraint(p, s_zero));
        var aG = new GtoCConstraint(p, s_flat, s_tilted);
        aPlate.Load(aG);
        aPlate.SolveTI(3);
        Assert.True(aPlate.IsDone);
        // the surface (the xy plane plus the deformation) is tangent to the target there
        Xyz aDu = s_flat.DU() + aPlate.EvaluateDerivative(p, 1, 0);
        Xyz aDv = s_flat.DV() + aPlate.EvaluateDerivative(p, 0, 1);
        Xyz aN = (s_tilted.DU() ^ s_tilted.DV()).Normalized();
        Assert.Equal(0.0, aDu * aN, 1.0e-9);
        Assert.Equal(0.0, aDv * aN, 1.0e-9);

        var aFree = new PlateSolver();
        aFree.Load(new PinpointConstraint(new Xy(0.0, 0.0), s_zero));
        aFree.Load(new PinpointConstraint(new Xy(1.0, 0.0), s_zero));
        aFree.Load(new PinpointConstraint(new Xy(0.0, 1.0), s_zero));
        aFree.Load(new PinpointConstraint(p, s_zero));
        aFree.Load(new FreeGtoCConstraint(p, s_flat, s_tilted));
        aFree.SolveTI(3);
        Assert.True(aFree.IsDone);
        Assert.Equal(0.0, (s_flat.DU() + aFree.EvaluateDerivative(p, 1, 0)) * aN, 1.0e-9);
        Assert.Equal(0.0, (s_flat.DV() + aFree.EvaluateDerivative(p, 0, 1)) * aN, 1.0e-9);
    }
}
