// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Law_BSpFunc: evaluation through its curve (one-sided at its bounds), intervals, Trim and freezing.</summary>
public class Law_BSpFuncTests
{
    /// <summary>A C0 cubic on [0, 3] with a triple knot at 1 and a simple knot at 2.</summary>
    private static BSpline C0Curve()
    {
        var aKnots = new NCollection_Array1<double>(1, 4);
        aKnots[1] = 0;
        aKnots[2] = 1;
        aKnots[3] = 2;
        aKnots[4] = 3;
        var aMults = new NCollection_Array1<int>(1, 4);
        aMults[1] = 4;
        aMults[2] = 3;
        aMults[3] = 1;
        aMults[4] = 4;
        int aNb = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(3, false, aMults);
        var aPoles = new NCollection_Array1<double>(1, aNb);
        for (int i = 1; i <= aNb; i++)
        {
            aPoles[i] = Math.Sin(1.3 * i) * 3.0 + 0.1 * i * i;
        }
        return new BSpline(aPoles, aKnots, aMults, 3);
    }

    [Fact]
    public void SetCurve_FreezesAndKeepsTheCurve()
    {
        BSpline aCurve = C0Curve();
        var aLaw = new BSpFunc();
        Assert.Null(aLaw.Curve());
        aLaw.SetCurve(aCurve);
        Assert.True(aCurve.IsFrozen);
        Assert.Same(aCurve, aLaw.Curve());
        aLaw.Bounds(out double f, out double l);
        Assert.Equal(0.0, f);
        Assert.Equal(3.0, l);
        Assert.Equal(Continuity.C0, aLaw.Continuity());

        var aTrimmed = Assert.IsType<BSpFunc>(aLaw.Trim(0.5, 2.5, 1.0e-9));
        Assert.Same(aCurve, aTrimmed.Curve());
        aTrimmed.Bounds(out f, out l);
        Assert.Equal((0.5, 2.5), (f, l));

        BSpline anOther = C0Curve();
        var aLaw2 = new BSpFunc(anOther, 1.0, 2.0);
        Assert.True(anOther.IsFrozen);
    }

    [Fact]
    public void Evaluation_IsOneSidedAtTheBounds()
    {
        BSpline aCurve = C0Curve();
        // a law ending at the C0 knot evaluates there on its own side
        var aLeft = new BSpFunc(aCurve, 0.0, 1.0);
        var aRight = new BSpFunc(aCurve, 1.0, 3.0);
        aLeft.D2(1.0, out double FL, out double DL, out double D2L);
        aRight.D2(1.0, out double FR, out double DR, out double D2R);
        aCurve.LocalD2(1.0, 1, 2, out double P1, out double V1, out double W1);
        aCurve.LocalD2(1.0, 2, 3, out double P2, out double V2, out double W2);
        Assert.Equal(P1, FL);
        Assert.Equal(V1, DL);
        Assert.Equal(W1, D2L);
        Assert.Equal(P2, FR);
        Assert.Equal(V2, DR);
        Assert.Equal(W2, D2R);
        Assert.NotEqual(DL, DR, 1.0e-6);
        Assert.Equal(FL, aLeft.Value(1.0), 1.0e-14);
        aLeft.D1(1.0, out double FL1, out double DL1);
        Assert.Equal((FL, DL), (FL1, DL1));
        aRight.D1(1.0, out double FR1, out double DR1);
        Assert.Equal((FR, DR), (FR1, DR1));
        // inside, the curve's own evaluation
        aRight.D2(1.7, out double F, out double D, out double D2);
        aCurve.D2(1.7, out double CP, out double CV, out double CW);
        Assert.Equal((CP, CV, CW), (F, D, D2));
        Assert.Equal(CP, aRight.Value(1.7), 1.0e-14);
    }

    [Fact]
    public void Intervals_FollowTheKnotsInsideTheBounds()
    {
        BSpline aCurve = C0Curve();
        var aLaw = new BSpFunc(aCurve, 0.5, 2.5);
        Assert.Equal(1, aLaw.NbIntervals(Continuity.C0));
        Assert.Equal(2, aLaw.NbIntervals(Continuity.C1));
        var aT = new NCollection_Array1<double>(1, 3);
        aLaw.Intervals(aT, Continuity.C1);
        Assert.Equal([0.5, 1.0, 2.5], aT.AsSpan().ToArray());
        // the simple knot at 2 breaks C3 only
        Assert.Equal(3, aLaw.NbIntervals(Continuity.C3));
        var aT3 = new NCollection_Array1<double>(1, 4);
        aLaw.Intervals(aT3, Continuity.C3);
        Assert.Equal([0.5, 1.0, 2.0, 2.5], aT3.AsSpan().ToArray());
        Assert.Throws<DomainErrorException>(() => aLaw.NbIntervals(Continuity.G1));
        // a law inside one span
        var anInner = new BSpFunc(aCurve, 1.2, 1.8);
        Assert.Equal(1, anInner.NbIntervals(Continuity.CN));
        var aT1 = new NCollection_Array1<double>(1, 2);
        anInner.Intervals(aT1, Continuity.CN);
        Assert.Equal([1.2, 1.8], aT1.AsSpan().ToArray());
    }
}
