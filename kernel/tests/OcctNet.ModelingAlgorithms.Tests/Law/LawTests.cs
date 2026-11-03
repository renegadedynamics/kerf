// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>The Law package functions, Law_BSplineKnotSplitting and GeomFill_TgtField's defaults.</summary>
public class LawTests
{
    [Fact]
    public void MixBnd_FollowsTheLinearLawWithFlatEnds()
    {
        var aLin = new Law_Linear();
        aLin.Set(0.0, 1.0, 2.0, 3.0);
        Law_BSpFunc aMix = Law.MixBnd(aLin);
        aMix.Bounds(out double f, out double l);
        Assert.Equal((0.0, 2.0), (f, l));
        Law_BSpline aCurve = aMix.Curve()!;
        Assert.True(aCurve.IsFrozen);
        Assert.Equal(3, aCurve.Degree());
        Assert.Equal(6, aCurve.NbPoles());
        Assert.Equal(aCurve.Pole(1), aCurve.Pole(2));
        Assert.Equal(aCurve.Pole(5), aCurve.Pole(6));
        Assert.Equal(1.0, aMix.Value(0.0), 1.0e-14);
        Assert.Equal(3.0, aMix.Value(2.0), 1.0e-14);
        Assert.Equal(2.0, aMix.Value(1.0), 1.0e-12); // symmetric
        aMix.D1(0.0, out _, out double D);
        Assert.Equal(0.0, D, 1.0e-12);

        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 2.0;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = aMults[2] = 3;
        NCollection_HArray1<double> aPoles = Law.MixBnd(2, aKnots, aMults, aLin);
        // three poles: a quadratic Bezier interpolating a line at 0, 1, 2 is that line
        Assert.Equal(3, aPoles.Length());
        Assert.Equal([1.0, 2.0, 3.0], aPoles.AsSpan().ToArray().Select(x => Math.Round(x, 12)).ToArray());
    }

    [Fact]
    public void MixTgt_VanishesOnOneSide()
    {
        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        aKnots[3] = 2.0;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = aMults[3] = 4;
        aMults[2] = 1;
        foreach (bool aRight in new[] { true, false })
        {
            NCollection_HArray1<double> aPoles = Law.MixTgt(3, aKnots, aMults, aRight, 2);
            Assert.Equal(5, aPoles.Length());
            var aCurve = new Law_BSpline(aPoles, aKnots, aMults, 3);
            // interpolates eval1 at the Schoenberg points: 0 at the far end
            Assert.Equal(0.0, aCurve.Value(aRight ? 2.0 : 0.0), 1.0e-14);
            Assert.Equal(0.0, aCurve.Value(aRight ? 0.0 : 2.0), 1.0e-14);
        }
    }

    [Fact]
    public void Scale_AndScaleCub_GoFromTheEndValuesToOne()
    {
        Law_BSpline aScale = Law.Scale(0.0, 4.0, true, true, 2.0, 0.5);
        Assert.False(aScale.IsFrozen);
        Assert.Equal(2.0, aScale.Value(0.0), 1.0e-12);
        Assert.Equal(0.5, aScale.Value(4.0), 1.0e-12);
        Assert.Equal(1.0, aScale.Value(2.0), 1.0e-12);
        Assert.Equal(6, aScale.NbPoles());
        Law_BSpline aNone = Law.Scale(0.0, 4.0, false, false, 2.0, 0.5);
        for (double x = 0.0; x <= 4.0; x += 0.5)
        {
            Assert.Equal(1.0, aNone.Value(x), 1.0e-12);
        }

        Law_BSpline aCub = Law.ScaleCub(1.0, 3.0, true, false, 3.0, 7.0);
        Assert.Equal(3.0, aCub.Value(1.0), 1.0e-14);
        Assert.Equal(1.0, aCub.Value(3.0), 1.0e-14);
        aCub.D1(1.0, out _, out double D);
        Assert.Equal(0.0, D, 1.0e-13);
        Assert.Equal(5, aCub.NbPoles());
        Law_BSpline aCub2 = Law.ScaleCub(1.0, 3.0, false, true, 3.0, 7.0);
        Assert.Equal(1.0, aCub2.Value(1.0), 1.0e-14);
        Assert.Equal(7.0, aCub2.Value(3.0), 1.0e-14);
    }

    [Fact]
    public void Reparametrize_ByArcLength()
    {
        // a parabola y = x^2 on [0, 1]: the law maps [0, 1] onto x by approximate arc length
        var aPoles = new NCollection_Array1<Point3D>(1, 3);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(0.5, 0, 0);
        aPoles[3] = new Point3D(1, 1, 0);
        var aCurve = new GeomAdaptor_Curve(new Geom_BezierCurve(aPoles));
        Law_BSpline aLaw = Law.Reparametrize(aCurve, 0.0, 1.0, false, false, 0.0, 0.0, false, 21);
        Assert.False(aLaw.IsFrozen);
        Assert.Equal(0.0, aLaw.Value(0.0), 1.0e-12);
        Assert.Equal(1.0, aLaw.Value(1.0), 1.0e-12);
        // equal steps of the law give nearly equal chords
        Point3D aPrev = aCurve.Value(aLaw.Value(0.0));
        double aMin = double.MaxValue, aMax = 0.0;
        for (int i = 1; i <= 10; i++)
        {
            Point3D aP = aCurve.Value(aLaw.Value(i / 10.0));
            double d = aP.Distance(aPrev);
            aMin = Math.Min(aMin, d);
            aMax = Math.Max(aMax, d);
            aPrev = aP;
        }
        Assert.True(aMax - aMin < 1.0e-2 * aMax, $"chords {aMin} to {aMax}");

        // reversed, with end derivatives
        Law_BSpline aRev = Law.Reparametrize(aCurve, 0.0, 2.0, true, true, 0.25, 0.75, true, 11);
        Assert.Equal(1.0, aRev.Value(0.0), 1.0e-12);
        Assert.Equal(0.0, aRev.Value(2.0), 1.0e-12);
        aRev.D1(0.0, out _, out double D0);
        aRev.D1(2.0, out _, out double D2);
        Assert.Equal(-0.25, D0, 1.0e-10);
        Assert.Equal(-0.75, D2, 1.0e-10);
    }

    [Fact]
    public void KnotSplitting_FindsTheKnotsBelowTheContinuity()
    {
        var aKnots = new NCollection_Array1<double>(1, 5);
        var aMults = new NCollection_Array1<int>(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aKnots[i] = i;
        }
        aMults[1] = aMults[5] = 4;
        aMults[2] = 1; // C2
        aMults[3] = 2; // C1
        aMults[4] = 3; // C0
        var aPoles = new NCollection_Array1<double>(1, global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(3, false, aMults));
        var aCurve = new Law_BSpline(aPoles, aKnots, aMults, 3);
        int[] Splits(int theCont)
        {
            var aSplit = new Law_BSplineKnotSplitting(aCurve, theCont);
            var anArray = new NCollection_Array1<int>(1, aSplit.NbSplits());
            aSplit.Splitting(anArray);
            for (int i = 1; i <= aSplit.NbSplits(); i++)
            {
                Assert.Equal(anArray[i], aSplit.SplitValue(i));
            }
            return anArray.AsSpan().ToArray();
        }
        Assert.Equal([1, 5], Splits(0));
        Assert.Equal([1, 4, 5], Splits(1));
        Assert.Equal([1, 3, 4, 5], Splits(2));
        Assert.Equal([1, 2, 3, 4, 5], Splits(3));
        Assert.Throws<RangeErrorException>(() => new Law_BSplineKnotSplitting(aCurve, -1));
        Assert.Throws<RangeErrorException>(() => new Law_BSplineKnotSplitting(aCurve, 1).SplitValue(4));
    }

    private sealed class ConstantField : GeomFill_TgtField
    {
        public override Vector3D Value(double W) => new Vector3D(1, 0, 0);

        public override Vector3D D1(double W) => new Vector3D();

        public override void D1(double W, out Vector3D V, out Vector3D DV)
        {
            V = Value(W);
            DV = D1(W);
        }
    }

    [Fact]
    public void TgtField_IsNotScalableByDefault()
    {
        var aField = new ConstantField();
        Assert.False(aField.IsScalable());
        aField.Scale(Law.ScaleCub(0, 1, true, true, 2, 2)); // does nothing
        Assert.Equal(1.0, aField.Value(0.5).X());
    }
}
