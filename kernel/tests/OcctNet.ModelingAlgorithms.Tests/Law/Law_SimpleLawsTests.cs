// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Law_Constant, Law_Linear and Law_S: closed forms, Trim, Bounds and Intervals.</summary>
public class Law_SimpleLawsTests
{
    [Fact]
    public void Law_Constant_IsConstant()
    {
        var aLaw = new Constant();
        aLaw.Set(2.5, -1.0, 4.0);
        Assert.Equal(2.5, aLaw.Value(17.0));
        aLaw.D1(0.3, out double F, out double D);
        Assert.Equal((2.5, 0.0), (F, D));
        aLaw.D2(0.3, out F, out D, out double D2);
        Assert.Equal((2.5, 0.0, 0.0), (F, D, D2));
        Assert.Equal(Continuity.CN, aLaw.Continuity());
        Assert.Equal(1, aLaw.NbIntervals(Continuity.CN));
        var aT = new NCollection_Array1<double>(1, 2);
        aLaw.Intervals(aT, Continuity.CN);
        Assert.Equal([-1.0, 4.0], aT.AsSpan().ToArray());
        var aTrim = Assert.IsType<Constant>(aLaw.Trim(0.0, 1.0, 1.0e-7));
        aTrim.Bounds(out double f, out double l);
        Assert.Equal((0.0, 1.0), (f, l));
        Assert.Equal(2.5, aTrim.Value(0.5));
        aLaw.Bounds(out f, out l);
        Assert.Equal((-1.0, 4.0), (f, l));
    }

    [Fact]
    public void Law_Linear_InterpolatesItsEnds()
    {
        var aLaw = new Linear();
        aLaw.Set(1.0, 3.0, 5.0, -1.0);
        Assert.Equal(3.0, aLaw.Value(1.0));
        Assert.Equal(-1.0, aLaw.Value(5.0));
        Assert.Equal(1.0, aLaw.Value(3.0));
        aLaw.D2(2.0, out double F, out double D, out double D2);
        Assert.Equal(2.0, F);
        Assert.Equal(-1.0, D);
        Assert.Equal(0.0, D2);
        aLaw.D1(2.0, out double F1, out double D1);
        Assert.Equal((F, D), (F1, D1));
        var aTrim = Assert.IsType<Linear>(aLaw.Trim(2.0, 4.0, 0.0));
        aTrim.Bounds(out double f, out double l);
        Assert.Equal((2.0, 4.0), (f, l));
        Assert.Equal(2.0, aTrim.Value(2.0));
        Assert.Equal(0.0, aTrim.Value(4.0));
        var aT = new NCollection_Array1<double>(1, 2);
        aLaw.Intervals(aT, Continuity.C2);
        Assert.Equal([1.0, 5.0], aT.AsSpan().ToArray());
        Assert.Equal(1, aLaw.NbIntervals(Continuity.C2));
    }

    [Fact]
    public void Law_S_HasTheGivenEndValuesAndDerivatives()
    {
        var aLaw = new S();
        aLaw.Set(0.0, 1.0, 2.0, 5.0);
        aLaw.D1(0.0, out double F, out double D);
        Assert.Equal(1.0, F, 1.0e-15);
        Assert.Equal(0.0, D, 1.0e-14);
        aLaw.D1(2.0, out F, out D);
        Assert.Equal(5.0, F, 1.0e-15);
        Assert.Equal(0.0, D, 1.0e-14);
        Assert.Equal(3.0, aLaw.Value(1.0), 1.0e-14); // symmetric S

        aLaw.Set(1.0, 0.0, 2.0, 3.0, 1.0, -1.0);
        aLaw.D2(1.0, out F, out D, out double D2);
        Assert.Equal(0.0, F, 1.0e-15);
        Assert.Equal(2.0, D, 1.0e-13);
        aLaw.D1(3.0, out F, out D);
        Assert.Equal(1.0, F, 1.0e-15);
        Assert.Equal(-1.0, D, 1.0e-13);
        aLaw.Bounds(out double f, out double l);
        Assert.Equal((1.0, 3.0), (f, l));
        Assert.True(aLaw.Curve()!.IsFrozen);
        Assert.Equal(3, aLaw.Curve()!.Degree());
        // the Hermite cubic, against finite differences
        double h = 1.0e-6;
        for (double x = 1.1; x < 3.0; x += 0.3)
        {
            aLaw.D2(x, out F, out D, out D2);
            Assert.Equal((aLaw.Value(x + h) - aLaw.Value(x - h)) / (2 * h), D, 1.0e-7);
            aLaw.D1(x + h, out _, out double Dp);
            aLaw.D1(x - h, out _, out double Dm);
            Assert.Equal((Dp - Dm) / (2 * h), D2, 1.0e-6);
        }
    }
}
