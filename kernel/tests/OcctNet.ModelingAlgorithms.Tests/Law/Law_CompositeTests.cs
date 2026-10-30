// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Law_Composite: choosing the elementary law, the junctions, periodicity, intervals and Trim.</summary>
public class Law_CompositeTests
{
    private static Law_Linear Linear(double p1, double v1, double p2, double v2)
    {
        var aLaw = new Law_Linear();
        aLaw.Set(p1, v1, p2, v2);
        return aLaw;
    }

    private static Law_Composite ThreeLaws(double theFirst, double theLast, double theTol)
    {
        var aComp = new Law_Composite(theFirst, theLast, theTol);
        aComp.ChangeLaws().Append(Linear(0, 0, 1, 1));    // slope 1
        aComp.ChangeLaws().Append(Linear(1, 1, 2, 3));    // slope 2
        aComp.ChangeLaws().Append(Linear(2, 3, 4, -1));   // slope -2
        return aComp;
    }

    [Fact]
    public void Evaluation_UsesTheLawOfTheParameter()
    {
        Law_Composite aComp = ThreeLaws(0, 4, 1.0e-9);
        // bounds are known after the first evaluation, as in OCCT
        aComp.Bounds(out double f, out double l);
        Assert.Equal((-1.0e100, 1.0e100), (f, l));
        Assert.Equal(0.5, aComp.Value(0.5));
        aComp.Bounds(out f, out l);
        Assert.Equal((0.0, 4.0), (f, l));
        aComp.D1(1.5, out double F, out double D);
        Assert.Equal((2.0, 2.0), (F, D));
        aComp.D2(3.0, out F, out D, out double D2);
        Assert.Equal((1.0, -2.0, 0.0), (F, D, D2));
        // outside the bounds: the end laws
        aComp.D1(-1.0, out F, out D);
        Assert.Equal(1.0, D);
        aComp.D1(5.0, out F, out D);
        Assert.Equal(-2.0, D);
    }

    [Fact]
    public void Junctions_GoToTheInnerLawNearEachEnd()
    {
        // near the first bound Eps = +Tol: a junction belongs to the next law
        Law_Composite aComp = ThreeLaws(0, 4, 1.0e-6);
        aComp.D1(1.0, out _, out double D);
        Assert.Equal(2.0, D);
        Assert.IsType<Law_Linear>(aComp.ChangeElementaryLaw(1.0));
        // nearer the last bound Eps = -Tol: a junction belongs to the previous law
        aComp.D1(2.0, out _, out D);
        Assert.Equal(2.0, D);
        aComp.D1(2.5, out _, out D);
        Assert.Equal(-2.0, D);
        ref Law_Function? aCur = ref aComp.ChangeElementaryLaw(2.5);
        aCur!.Bounds(out double f, out double l);
        Assert.Equal((2.0, 4.0), (f, l));
    }

    [Fact]
    public void Periodic_BringsTheParameterIntoTheRange()
    {
        Law_Composite aComp = ThreeLaws(0, 4, 0.0);
        Assert.False(aComp.IsPeriodic());
        aComp.SetPeriodic();
        Assert.True(aComp.IsPeriodic());
        Assert.Equal(aComp.Value(0.5), aComp.Value(4.5), 1.0e-14);
        Assert.Equal(aComp.Value(3.0), aComp.Value(-1.0), 1.0e-14);
    }

    [Fact]
    public void IntervalsAndTrim()
    {
        Law_Composite aComp = ThreeLaws(0, 4, 0.0);
        Assert.Equal(3, aComp.NbIntervals(GeomAbs_Shape.GeomAbs_C2));
        var aT = new NCollection_Array1<double>(1, 4);
        aComp.Intervals(aT, GeomAbs_Shape.GeomAbs_C2);
        Assert.Equal([0.0, 1.0, 2.0, 4.0], aT.AsSpan().ToArray());
        Assert.Throws<Standard_NotImplemented>(() => aComp.Continuity());

        var aTrim = Assert.IsType<Law_Composite>(aComp.Trim(0.5, 3.5, 1.0e-9));
        Assert.Equal(3, aTrim.ChangeLaws().Extent());
        Assert.Same(aComp.ChangeLaws().First(), aTrim.ChangeLaws().First());
        Assert.NotSame(aComp.ChangeLaws(), aTrim.ChangeLaws());
        Assert.Equal(aComp.Value(3.0), aTrim.Value(3.0));
    }
}
