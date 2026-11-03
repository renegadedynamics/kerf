// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Law_Interpolate and Law_Interpol: interpolation through values, with and without tangents, periodic or not.</summary>
public class Law_InterpolateTests
{
    private static NCollection_HArray1<double> Array(params double[] theValues)
    {
        var anArray = new NCollection_HArray1<double>(1, theValues.Length);
        for (int i = 0; i < theValues.Length; i++)
        {
            anArray[i + 1] = theValues[i];
        }
        return anArray;
    }

    [Theory]
    [InlineData(2, 1)]
    [InlineData(3, 2)]
    [InlineData(4, 3)]
    [InlineData(7, 3)]
    public void NonPeriodic_PassesThroughThePoints(int theNb, int theDegree)
    {
        var aRnd = new Random(theNb);
        var aValues = new double[theNb];
        var aParams = new double[theNb];
        for (int i = 0; i < theNb; i++)
        {
            aValues[i] = aRnd.NextDouble() * 10 - 5;
            aParams[i] = i == 0 ? 0.0 : aParams[i - 1] + 0.5 + aRnd.NextDouble();
        }
        var anInter = new Law_Interpolate(Array(aValues), Array(aParams), false, 1.0e-9);
        Assert.False(anInter.IsDone());
        Assert.Throws<NotDoneException>(() => anInter.Curve());
        anInter.Perform();
        Assert.True(anInter.IsDone());
        Law_BSpline aCurve = anInter.Curve();
        Assert.True(aCurve.IsFrozen);
        Assert.Equal(theDegree, aCurve.Degree());
        for (int i = 0; i < theNb; i++)
        {
            Assert.Equal(aValues[i], aCurve.Value(aParams[i]), 1.0e-12);
        }
    }

    [Fact]
    public void NonPeriodic_WithEndTangents()
    {
        double[] aValues = [0.0, 1.0, 0.5, 2.0];
        double[] aParams = [0.0, 1.0, 2.5, 3.0];
        var anInter = new Law_Interpolate(Array(aValues), Array(aParams), false, 1.0e-9);
        anInter.Load(3.0, -1.0);
        anInter.Perform();
        Law_BSpline aCurve = anInter.Curve();
        for (int i = 0; i < 4; i++)
        {
            Assert.Equal(aValues[i], aCurve.Value(aParams[i]), 1.0e-12);
        }
        aCurve.D1(0.0, out _, out double D0);
        aCurve.D1(3.0, out _, out double D3);
        Assert.Equal(3.0, D0, 1.0e-11);
        Assert.Equal(-1.0, D3, 1.0e-11);
    }

    [Fact]
    public void NonPeriodic_WithInnerTangents()
    {
        double[] aValues = [0.0, 1.0, 0.5, 2.0, 1.0];
        double[] aParams = [0.0, 1.0, 2.0, 3.0, 4.0];
        var aFlags = new NCollection_HArray1<bool>(1, 5);
        aFlags.Init(false);
        aFlags[3] = true;
        var aTangents = new NCollection_Array1<double>(1, 5);
        aTangents[3] = 4.0;
        var anInter = new Law_Interpolate(Array(aValues), Array(aParams), false, 1.0e-9);
        anInter.Load(aTangents, aFlags);
        anInter.Perform();
        Law_BSpline aCurve = anInter.Curve();
        for (int i = 0; i < 5; i++)
        {
            Assert.Equal(aValues[i], aCurve.Value(aParams[i]), 1.0e-12);
        }
        aCurve.D1(2.0, out _, out double D);
        Assert.Equal(4.0, D, 1.0e-11);
        // the end tangents computed by Lagrange interpolation are written into the caller's flags, as in OCCT
        Assert.True(aFlags[1]);
        Assert.True(aFlags[5]);
    }

    [Fact]
    public void Periodic_ClosesOnTheFirstPoint()
    {
        double[] aValues = [1.0, 3.0, 2.0, 0.0];
        double[] aParams = [0.0, 1.0, 2.0, 3.0, 4.0];
        var anInter = new Law_Interpolate(Array(aValues), Array(aParams), true, 1.0e-9);
        anInter.Perform();
        Law_BSpline aCurve = anInter.Curve();
        Assert.True(aCurve.IsPeriodic());
        for (int i = 0; i < 4; i++)
        {
            Assert.Equal(aValues[i], aCurve.Value(aParams[i]), 1.0e-12);
        }
        Assert.Equal(aValues[0], aCurve.Value(4.0), 1.0e-12);
        aCurve.D1(0.0, out _, out double D0);
        aCurve.D1(4.0, out _, out double D4);
        Assert.Equal(D0, D4, 1.0e-10);
        Assert.Throws<ConstructionErrorException>(() => new Law_Interpolate(Array(aValues), Array(aParams), false, 1.0e-9)
                                                        .Load(new NCollection_Array1<double>(1, 3), new NCollection_HArray1<bool>(1, 3)));
    }

    [Fact]
    public void ComputedParameters_FollowTheValues()
    {
        // parameters from the differences of the values: 0, 2, 3, 7
        var anInter = new Law_Interpolate(Array(1.0, 3.0, 2.0, 6.0), false, 1.0e-9);
        anInter.Perform();
        Law_BSpline aCurve = anInter.Curve();
        Assert.Equal(0.0, aCurve.FirstParameter());
        Assert.Equal(7.0, aCurve.LastParameter());
        Assert.Equal(3.0, aCurve.Value(2.0), 1.0e-12);
        Assert.Equal(2.0, aCurve.Value(3.0), 1.0e-12);
        Assert.Throws<ConstructionErrorException>(() => new Law_Interpolate(Array(1.0, 2.0), Array(0.0, 0.0), false, 1.0e-9));
    }

    [Fact]
    public void Law_Interpol_SetAndSetInRelative()
    {
        var aPnts = new NCollection_Array1<Point2D>(1, 4);
        aPnts[1] = new Point2D(0.0, 1.0);
        aPnts[2] = new Point2D(1.0, 2.0);
        aPnts[3] = new Point2D(2.0, 1.5);
        aPnts[4] = new Point2D(4.0, 3.0);
        var aLaw = new Law_Interpol();
        aLaw.Set(aPnts);
        for (int i = 1; i <= 4; i++)
        {
            Assert.Equal(aPnts[i].Y(), aLaw.Value(aPnts[i].X()), 1.0e-7);
        }
        aLaw.Set(aPnts, 1.0, -2.0);
        aLaw.D1(0.0, out _, out double D);
        Assert.Equal(1.0, D, 1.0e-10);
        aLaw.D1(4.0, out _, out D);
        Assert.Equal(-2.0, D, 1.0e-10);

        // relative: the parameters mapped onto [10, 12]
        aLaw.SetInRelative(aPnts, 10.0, 12.0);
        aLaw.Bounds(out double f, out double l);
        Assert.Equal((10.0, 12.0), (f, l));
        Assert.Equal(2.0, aLaw.Value(10.5), 1.0e-7);
        aLaw.SetInRelative(aPnts, 10.0, 12.0, 0.5, 0.25);
        aLaw.D1(10.0, out _, out D);
        Assert.Equal(0.5, D, 1.0e-10);

        // periodic: the last value is the first
        aPnts[4] = new Point2D(4.0, 1.0);
        aLaw.Set(aPnts, true);
        Assert.True(aLaw.Curve()!.IsPeriodic());
        Assert.Equal(1.0, aLaw.Value(4.0), 1.0e-7);
        aLaw.SetInRelative(aPnts, 0.0, 1.0, true);
        Assert.Equal(2.0, aLaw.Value(0.25), 1.0e-7);
    }
}
