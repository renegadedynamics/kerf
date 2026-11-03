// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.Intersection;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of HLRAlgo's projector (parallel and perspective, the transforms, the projections and the shooting) and
/// edge status (hiding intervals, the visible and hidden parts). TKHLR has no GTests; the native comparisons are
/// <see cref="HlrNativeTests"/>.
/// </summary>
public class HLRAlgoTests
{
    private const double Tol = 1e-12;

    private static Frame3D View() => new(new Point3D(1, 2, 3), new Direction3D(0.3, -0.7, 0.5), new Direction3D(0.7, 0.3, 0.0));

    [Fact]
    public void Projector_Default_IsIdentityAndParallel()
    {
        AlgorithmProjector P = new();
        Assert.False(P.Perspective());
        Point2D aP = new();
        P.Project(new Point3D(1, 2, 3), ref aP);
        Assert.Equal(1.0, aP.X(), Tol);
        Assert.Equal(2.0, aP.Y(), Tol);
        P.Project(new Point3D(1, 2, 3), out double X, out double Y, out double Z);
        Assert.Equal((1.0, 2.0, 3.0), (X, Y, Z));
        Assert.ThrowsAny<NoSuchObjectException>(() => P.Focus());
    }

    [Fact]
    public void Projector_Parallel_DropsTheTranslationAndProjectsAlongTheViewDirection()
    {
        AlgorithmProjector P = new(View());
        // the translation is dropped (Scaled(false)): the view's origin does not move the projection
        Point3D aOrigin = new(0, 0, 0);
        P.Project(aOrigin, out double X0, out double Y0, out double Z0);
        Assert.Equal(0.0, X0, Tol);
        Assert.Equal(0.0, Y0, Tol);
        Assert.Equal(0.0, Z0, Tol);
        // moving a point along the view direction changes only the depth
        Direction3D aZ = View().Direction();
        Point3D aA = new(3, -1, 2);
        Point3D aB = aA.Translated(new Vector3D(aZ).Multiplied(5.0));
        P.Project(aA, out double XA, out double YA, out double ZA);
        P.Project(aB, out double XB, out double YB, out double ZB);
        Assert.Equal(XA, XB, 1e-12);
        Assert.Equal(YA, YB, 1e-12);
        Assert.Equal(ZA + 5.0, ZB, 1e-12);
        // the inverted transformation undoes the active one
        Point3D aT = aA;
        P.Transform(ref aT);
        Assert.True(aT.Transformed(P.InvertedTransformation()).IsEqual(aA, 1e-12));
    }

    [Fact]
    public void Projector_Perspective_DividesByTheDepth()
    {
        const double f = 50.0;
        AlgorithmProjector P = new(View(), f);
        Assert.True(P.Perspective());
        Assert.Equal(f, P.Focus());
        Point3D aA = new(3, -1, 2);
        Point3D aT = aA;
        P.Transform(ref aT);
        Point2D aP = new();
        P.Project(aA, ref aP);
        double R = 1.0 - aT.Z() / f;
        Assert.Equal(aT.X() / R, aP.X(), Tol);
        Assert.Equal(aT.Y() / R, aP.Y(), Tol);
        P.Project(aA, out double X, out double Y, out double Z);
        Assert.Equal(aP.X(), X, Tol);
        Assert.Equal(aP.Y(), Y, Tol);
        Assert.Equal(aT.Z(), Z, Tol);

        // the derivative of the projection, against a finite difference
        Vector3D aD = new(0.4, -1.2, 0.9);
        P.Project(aA, aD, out Point2D aP1, out Vector2D aD1);
        const double h = 1e-6;
        Point2D aPh = new();
        P.Project(aA.Translated(aD.Multiplied(h)), ref aPh);
        Assert.Equal((aPh.X() - aP1.X()) / h, aD1.X(), 1e-4);
        Assert.Equal((aPh.Y() - aP1.Y()) / h, aD1.Y(), 1e-4);
    }

    [Fact]
    public void Projector_Shoot_GoesThroughTheProjectedPoint()
    {
        foreach (AlgorithmProjector P in new[] { new AlgorithmProjector(View()), new AlgorithmProjector(View(), 40.0) })
        {
            Point3D aA = new(3, -1, 2);
            Point2D aP = new();
            P.Project(aA, ref aP);
            Line3D aL = P.Shoot(aP.X(), aP.Y());
            Assert.True(aL.Distance(aA) < 1e-9, $"{aL.Distance(aA)}");
        }
    }

    [Fact]
    public void Projector_TopView_UsesTheHardCodedFormula()
    {
        AlgorithmProjector P = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        Point2D aP = new();
        P.Project(new Point3D(4, -5, 6), ref aP);
        Assert.Equal((4.0, -5.0), (aP.X(), aP.Y()));
    }

    [Fact]
    public void Projector_FirstAxonometricView_LeavesThePointUnset_AsOcct()
    {
        // OCCT's type-0 view matrix: Project(P, Pout) computes X and Y but does not write them (decision 1078)
        AlgorithmProjector P = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 1), new Direction3D(1.4142135623730951, -1, 1)));
        Point2D aP = new(-7, -7);
        P.Project(new Point3D(1, 2, 3), ref aP);
        Assert.Equal((-7.0, -7.0), (aP.X(), aP.Y()));
        P.Project(new Point3D(1, 2, 3), out double X, out double Y, out _);
        Assert.NotEqual(-7.0, X);
        Assert.NotEqual(-7.0, Y);
    }

    [Fact]
    public void Projector_Scaled_KeepsTheScaleAndTranslation()
    {
        Transform3D aT = new();
        aT.SetTransformation(new CoordinateSystem(View()));
        Transform3D aS = new();
        aS.SetScale(new Point3D(0, 0, 0), 2.0);
        Transform3D aTS = aS * aT;
        AlgorithmProjector P = new(aTS, false, 0.0);
        AlgorithmProjector PS = P.WithScaled(true);
        Assert.NotEqual(P, PS);
        Assert.Equal(1.0, P.Transformation().ScaleFactor(), Tol);
        Assert.Equal(2.0, PS.Transformation().ScaleFactor(), Tol);
        Assert.Equal(aTS, PS.FullTransformation());
        Assert.Equal(PS, P.WithScaled(true));
    }

    [Fact]
    public void EdgeStatus_HidesIntervals()
    {
        EdgeStatus S = new(0.0, 1e-5f, 1.0, 1e-5f);
        Assert.True(S.AllVisible());
        Assert.Equal(1, S.NbVisiblePart());

        S.Hide(0.2, 1e-5f, 0.4, 1e-5f, false, false);
        Assert.False(S.AllVisible());
        Assert.Equal(2, S.NbVisiblePart());
        S.VisiblePart(1, out double a, out _, out double b, out _);
        Assert.Equal((0.0, 0.2), (a, b));
        S.VisiblePart(2, out a, out _, out b, out _);
        Assert.Equal((0.4, 1.0), (a, b));

        // on the face: nothing hidden
        S.Hide(0.6, 1e-5f, 0.7, 1e-5f, true, false);
        Assert.Equal(2, S.NbVisiblePart());

        // the hidden parts are the complement of the visible ones
        EdgeIterator It = new();
        List<(double, double)> aHidden = [];
        for (It.InitHidden(S); It.MoreHidden(); It.NextHidden())
        {
            It.Hidden(out a, out _, out b, out _);
            aHidden.Add((a, b));
        }
        Assert.Equal([(0.2, 0.4)], aHidden);

        S.Hide(-1.0, 1e-5f, 2.0, 1e-5f, false, false);
        Assert.True(S.AllHidden());
        Assert.Equal(0, S.NbVisiblePart());
        aHidden.Clear();
        for (It.InitHidden(S); It.MoreHidden(); It.NextHidden())
        {
            It.Hidden(out a, out _, out b, out _);
            aHidden.Add((a, b));
        }
        Assert.Equal([(0.0, 1.0)], aHidden);
    }

    [Fact]
    public void EdgeStatus_CopyAndAssign_AreDeep()
    {
        EdgeStatus S = new(0.0, 1e-5f, 1.0, 1e-5f);
        S.Hide(0.2, 1e-5f, 0.4, 1e-5f, false, false);
        EdgeStatus C = new(S);
        EdgeStatus A = new();
        A.Assign(S);
        S.Hide(0.5, 1e-5f, 0.6, 1e-5f, false, false);
        Assert.Equal(3, S.NbVisiblePart());
        Assert.Equal(2, C.NbVisiblePart());
        Assert.Equal(2, A.NbVisiblePart());
    }

    [Fact]
    public void Intervals_UniteSubtractIntersect()
    {
        Intervals I = new(new IntersectionInterval(0.0, 10.0));
        I.Subtract(new IntersectionInterval(2.0, 3.0));
        Assert.Equal(2, I.IntervalCount);
        I.Unite(new IntersectionInterval(2.5, 2.75));
        Assert.Equal(3, I.IntervalCount);
        I.Intersect(new IntersectionInterval(1.0, 2.6));
        Assert.Equal(2, I.IntervalCount);
        Assert.Equal((1.0, 2.0), (I.Value(1).Start(), I.Value(1).End()));
        Assert.Equal((2.5, 2.6), (I.Value(2).Start(), I.Value(2).End()));
    }

    [Fact]
    public void MinMax_EncodeDecode_RoundTrips()
    {
        EdgesBlock.MinMaxIndices aMin = new(), aMax = new(), aMM = new(), aMin2 = new(), aMax2 = new();
        for (int i = 0; i < 8; i++)
        {
            aMin.Min[i] = 10 * i + 1;
            aMin.Max[i] = 10 * i + 2;
            aMax.Min[i] = 1000 + 10 * i + 3;
            aMax.Max[i] = 1000 + 10 * i + 4;
        }
        HlrAlgo.EncodeMinMax(ref aMin, ref aMax, ref aMM);
        HlrAlgo.DecodeMinMax(aMM, ref aMin2, ref aMax2);
        Assert.Equal(aMin, aMin2);
        Assert.Equal(aMax, aMax2);
    }
}
