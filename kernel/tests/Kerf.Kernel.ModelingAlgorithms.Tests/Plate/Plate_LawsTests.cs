// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Plate;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The plate's derivatives and its free tangency (decision 1753): every derivative a plate can give is the rate of
/// change of the one below it, whatever the order of the solve and the anisotropy, and a solved plate meets the normal
/// part of what its constraints ask for at each order.
/// </summary>
public class Plate_LawsTests
{
    private static readonly Xyz s_zero = new(0.0, 0.0, 0.0);
    private static readonly D1 s_flat = new(new Xyz(1.0, 0.0, 0.0), new Xyz(0.0, 1.0, 0.0));
    private static readonly D1 s_tilted = new(new Xyz(1.0, 0.0, 0.3), new Xyz(0.0, 1.0, -0.2));
    private static readonly D2 s_d2s = new(new Xyz(0.0, 0.0, 0.1), s_zero, s_zero);
    private static readonly D2 s_d2t = new(s_zero, new Xyz(0.0, 0.0, 0.2), new Xyz(0.1, 0.0, 0.0));
    private static readonly D3 s_d3s = new(s_zero, s_zero, s_zero, s_zero);
    private static readonly D3 s_d3t = new(new Xyz(0.0, 0.0, 1.0), s_zero, s_zero, s_zero);

    private static readonly Xy[] s_points =
    [
        new(0.0, 0.0), new(1.0, 0.0), new(0.0, 1.0), new(1.0, 1.0), new(0.5, 0.3), new(0.2, 0.8), new(0.9, 0.6),
        new(0.4, 0.55),
    ];

    private static Xyz Bump(in Xy p) =>
        new(0.1 * Math.Sin(3.0 * p.X()), 0.05 * p.Y() * p.Y(), Math.Exp(-((p.X() - 0.5) * (p.X() - 0.5)) - ((p.Y() - 0.5) * (p.Y() - 0.5))));

    private static PlateSolver Solved(int theOrder, double theAniso)
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

    public static TheoryData<int, double, int, int> Derivatives()
    {
        var aData = new TheoryData<int, double, int, int>();
        foreach (int anOrder in new[] { 2, 3, 4 })
        {
            foreach (double anAniso in new[] { 1.0, 2.5 })
            {
                // the plate's solution is of class C^(2 order - 3) or so away from the points: as far as it goes
                for (int iu = 0; iu <= 2 * anOrder - 2; iu++)
                {
                    for (int iv = 0; iu + iv <= 2 * anOrder - 2; iv++)
                    {
                        if (iu + iv < 6)
                        {
                            aData.Add(anOrder, anAniso, iu, iv);
                        }
                    }
                }
            }
        }
        return aData;
    }

    [Theory]
    [MemberData(nameof(Derivatives))]
    public void Each_derivative_is_the_rate_of_change_of_the_one_below_it(int theOrder, double theAniso, int theIu, int theIv)
    {
        if (theIu + theIv == 0)
        {
            return;
        }
        PlateSolver aPlate = Solved(theOrder, theAniso);
        var p = new Xy(0.37, 0.41);
        const double h = 1.0e-5;
        Xyz anExpected;
        if (theIu > 0)
        {
            anExpected = (aPlate.EvaluateDerivative(new Xy(p.X() + h, p.Y()), theIu - 1, theIv)
                          - aPlate.EvaluateDerivative(new Xy(p.X() - h, p.Y()), theIu - 1, theIv)) / (2.0 * h);
        }
        else
        {
            anExpected = (aPlate.EvaluateDerivative(new Xy(p.X(), p.Y() + h), theIu, theIv - 1)
                          - aPlate.EvaluateDerivative(new Xy(p.X(), p.Y() - h), theIu, theIv - 1)) / (2.0 * h);
        }
        Xyz anActual = aPlate.EvaluateDerivative(p, theIu, theIv);
        double aScale = Math.Max(1.0, anExpected.Modulus());
        Assert.True(anExpected.IsEqual(anActual, 1.0e-4 * aScale), $"expected {anExpected}, got {anActual}");
    }

    private static PlateSolver WithFree(FreeGtoCConstraint theFree, Xy theP)
    {
        var aPlate = new PlateSolver();
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 0.0), s_zero));
        aPlate.Load(new PinpointConstraint(new Xy(1.0, 0.0), s_zero));
        aPlate.Load(new PinpointConstraint(new Xy(0.0, 1.0), s_zero));
        aPlate.Load(new PinpointConstraint(theP, s_zero));
        aPlate.Load(theFree);
        aPlate.SolveTI(4);
        Assert.True(aPlate.IsDone);
        return aPlate;
    }

    [Theory]
    [InlineData(1.0, 0)]
    [InlineData(1.0, 1)]
    [InlineData(1.0, -1)]
    [InlineData(0.5, 0)]
    public void Free_tangency_of_the_second_order_meets_its_normal_part(double theLoad, int theOrientation)
    {
        var p = new Xy(0.5, 0.5);
        var aFree = new FreeGtoCConstraint(p, s_flat, s_tilted, s_d2s, s_d2t, theLoad, theOrientation);
        Assert.Equal(3, aFree.nb_LSC());
        Assert.Equal(2, aFree.nb_PPC());
        PlateSolver aPlate = WithFree(aFree, p);
        // the normal the constraint uses is its first scalar constraint's
        Xyz aN = aFree.LSC(0).Coeff()[1, 1];
        Assert.Equal(0.0, (s_flat.DU() + aPlate.EvaluateDerivative(p, 1, 0)) * aN, 1.0e-8);
        Assert.Equal(0.0, (s_flat.DV() + aPlate.EvaluateDerivative(p, 0, 1)) * aN, 1.0e-8);
    }

    [Theory]
    [InlineData(1.0, 0)]
    [InlineData(1.0, 1)]
    [InlineData(1.0, -1)]
    [InlineData(0.5, 0)]
    public void Free_tangency_of_the_third_order_meets_its_normal_part(double theLoad, int theOrientation)
    {
        var p = new Xy(0.5, 0.5);
        var aFree = new FreeGtoCConstraint(p, s_flat, s_tilted, s_d2s, s_d2t, s_d3s, s_d3t, theLoad, theOrientation);
        Assert.Equal(4, aFree.nb_LSC());
        Assert.Equal(5, aFree.nb_PPC());
        Assert.Equal(p.X(), aFree.GetPPC(0).Pnt2d().X());
        PlateSolver aPlate = WithFree(aFree, p);
        Xyz aN = aFree.LSC(0).Coeff()[1, 1];
        Assert.Equal(0.0, (s_flat.DU() + aPlate.EvaluateDerivative(p, 1, 0)) * aN, 1.0e-8);
        Assert.Equal(0.0, (s_flat.DV() + aPlate.EvaluateDerivative(p, 0, 1)) * aN, 1.0e-8);
    }

    [Fact]
    public void Free_tangency_of_a_degenerate_target_asks_for_nothing()
    {
        var p = new Xy(0.5, 0.5);
        var aDegenerate = new D1(new Xyz(1.0, 0.0, 0.0), new Xyz(2.0, 0.0, 0.0));
        Assert.Equal(0, new FreeGtoCConstraint(p, s_flat, aDegenerate).nb_LSC());
        Assert.Equal(0, new FreeGtoCConstraint(p, s_flat, aDegenerate, s_d2s, s_d2t).nb_LSC());
        Assert.Equal(0, new FreeGtoCConstraint(p, s_flat, aDegenerate, s_d2s, s_d2t, s_d3s, s_d3t).nb_LSC());
        Assert.Equal(0, new FreeGtoCConstraint(p, aDegenerate, s_flat, 0.5).nb_LSC());
        Assert.Equal(0, new FreeGtoCConstraint(p, aDegenerate, s_tilted, s_d2s, s_d2t, s_d3s, s_d3t, 0.5).nb_LSC());
        // a degenerate surface normal keeps the first order alone
        Assert.Equal(2, new FreeGtoCConstraint(p, aDegenerate, s_tilted, s_d2s, s_d2t, s_d3s, s_d3t).nb_LSC());
    }

    [Fact]
    public void The_box_holds_every_constraint_point()
    {
        var aPlate = new PlateSolver();
        aPlate.Load(new PinpointConstraint(new Xy(0.5, 0.2), s_zero));
        aPlate.Load(new PinpointConstraint(new Xy(-0.5, 1.2), s_zero));
        aPlate.Load(new PinpointConstraint(new Xy(2.5, -0.8), s_zero));
        aPlate.Load(new PinpointConstraint(new Xy(1.0, 3.0), s_zero));
        aPlate.UVBox(out double aUMin, out double aUMax, out double aVMin, out double aVMax);
        Assert.Equal(-0.5, aUMin);
        Assert.Equal(2.5, aUMax);
        Assert.Equal(-0.8, aVMin);
        Assert.Equal(3.0, aVMax);
    }

    [Fact]
    public void A_destroyed_plate_is_empty_and_answers_zero()
    {
        PlateSolver aPlate = Solved(3, 1.0);
        aPlate.Destroy();
        Assert.Equal(0.0, aPlate.Evaluate(new Xy(0.5, 0.5)).Modulus());
    }
}
