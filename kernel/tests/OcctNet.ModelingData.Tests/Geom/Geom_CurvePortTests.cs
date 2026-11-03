// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Evaluation;
using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own tests of the Geom curves: freezing, the non-mutating twins, the returned arrays, and the OCCT
/// behaviours listed in occt-issues.md.
/// </summary>
public class Geom_CurvePortTests
{
    private static BSplineCurve3D Cubic(bool theRational = false)
    {
        var aPoles = NCollection_Array1<Point3D>.From([new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(2, -1, 1),
                                                      new Point3D(3, 1, 0), new Point3D(4, 0, 2)]);
        var aKnots = NCollection_Array1<double>.From([0.0, 0.5, 1.0]);
        var aMults = NCollection_Array1<int>.From([4, 1, 4]);
        if (!theRational)
        {
            return new BSplineCurve3D(aPoles, aKnots, aMults, 3);
        }
        var aWeights = NCollection_Array1<double>.From([1.0, 2.0, 0.5, 1.0, 1.5]);
        return new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, 3);
    }

    [Fact]
    public void Frozen_curves_refuse_mutators_but_give_changed_copies()
    {
        BSplineCurve3D aC = Cubic();
        aC.Freeze();
        Assert.Throws<global::Kerf.Engine.Foundation.ImmutableObjectException>(() => aC.InsertKnot(0.25));
        Assert.Throws<global::Kerf.Engine.Foundation.ImmutableObjectException>(() => aC.SetPole(1, new Point3D(5, 5, 5)));
        Assert.Throws<global::Kerf.Engine.Foundation.ImmutableObjectException>(() => aC.Reverse());
        Assert.Throws<global::Kerf.Engine.Foundation.ImmutableObjectException>(() => aC.Transform(Transform3D.Translation(new Vector3D(1, 0, 0))));
        Assert.Throws<global::Kerf.Engine.Foundation.ImmutableObjectException>(() => aC.ClearEvalRepresentation());

        BSplineCurve3D aK = aC.KnotInserted(0.25, 2);
        Assert.False(aK.IsFrozen);
        Assert.Equal(4, aK.KnotCount);
        Assert.Equal(3, aC.KnotCount);

        BSplineCurve3D aP = aC.WithPole(1, new Point3D(5, 5, 5));
        Assert.Equal(new Point3D(5, 5, 5), aP.Pole(1));
        Assert.Equal(new Point3D(0, 0, 0), aC.Pole(1));

        // Resolution caches a value but is not a mutator
        aC.Resolution(1e-3, out double aTol);
        Assert.True(aTol > 0);

        var aL = new LineCurve3D(new Point3D(), new Direction3D(1, 0, 0));
        aL.Freeze();
        Assert.Throws<global::Kerf.Engine.Foundation.ImmutableObjectException>(() => aL.SetLocation(new Point3D(1, 1, 1)));
        Assert.Equal(new Point3D(1, 1, 1), aL.WithLocation(new Point3D(1, 1, 1)).Position().Location());
    }

    [Fact]
    public void Returned_arrays_are_copies()
    {
        BSplineCurve3D aC = Cubic(true);
        PackedList<Point3D> aPoles = aC.Poles();
        Assert.Equal(5, aPoles.Count);
        Assert.Equal(aC.Pole(2), aPoles[1]);
        PackedList<double>? aWeights = aC.Weights();
        Assert.NotNull(aWeights);
        Assert.Equal(2.0, aWeights[1]);
        Assert.Equal(new[] { 4, 1, 4 }, aC.Multiplicities().AsSpan().ToArray());
        Assert.Equal(9, aC.KnotSequence().Count);

        BSplineCurve3D aChanged = aC.WithPole(2, new Point3D(9, 9, 9));
        Assert.Equal(new Point3D(1, 2, 0), aPoles[1]);
        Assert.Equal(new Point3D(9, 9, 9), aChanged.Poles()[1]);

        Assert.Null(Cubic().Weights());
        Assert.Equal(5, Cubic().WeightsArray().Count);
    }

    [Fact]
    public void Twins_match_the_mutators()
    {
        BSplineCurve3D aC = Cubic(true);
        BSplineCurve3D aM = (BSplineCurve3D)aC.Copy();
        aM.IncreaseDegree(5);
        BSplineCurve3D aT = aC.DegreeIncreased(5);
        Assert.True(aM.IsEqual(aT, 0.0));

        Assert.Null(aC.KnotRemoved(2, 0, 1e-12));
        BSplineCurve3D aInserted = aC.KnotInserted(0.75);
        BSplineCurve3D? aRemoved = aInserted.KnotRemoved(3, 0, 1e-9);
        Assert.NotNull(aRemoved);
        Assert.Equal(3, aRemoved.KnotCount);

        BSplineCurve3D aSeg = aC.Segmented(0.2, 0.8);
        Assert.Equal(0.2, aSeg.FirstParameter());
        Assert.Equal(0.8, aSeg.LastParameter());
        Assert.True(aSeg.Value(0.5).IsEqual(aC.Value(0.5), 1e-12));
    }

    [Fact]
    public void Segment_default_tolerance_is_PConfusion()
    {
        // The default argument of Segment is the constant 1e-7 * 0.01, which must be Precision::PConfusion().
        BSplineCurve3D aC = Cubic();
        BSplineCurve3D aA = aC.Segmented(0.1, 0.9);
        BSplineCurve3D aB = aC.Segmented(0.1, 0.9, global::Kerf.Engine.Foundation.Precision.PConfusion());
        Assert.True(aA.IsEqual(aB, 0.0));
        Assert.Equal(1e-7 * 0.01, global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    [Fact]
    public void Eval_representation_is_copied_in()
    {
        BSplineCurve3D aC = Cubic();
        var aRep = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var aDesc = new RepCurveDesc.Full { Representation = aRep };
        BSplineCurve3D aWith = aC.WithEvalRepresentation(aDesc);
        Assert.True(aWith.HasEvalRepresentation);
        Assert.False(aC.HasEvalRepresentation);
        Assert.Equal(new Point3D(0, 0, 2), aWith.Value(2));

        // changing the caller's descriptor afterwards doesn't reach the curve
        aDesc.Representation = new LineCurve3D(new Point3D(1, 1, 1), new Direction3D(1, 0, 0));
        Assert.Equal(new Point3D(0, 0, 2), aWith.Value(2));

        // a modification drops it, as in OCCT
        Assert.False(aWith.WithPole(1, new Point3D(1, 1, 1)).HasEvalRepresentation);
        Assert.False(aWith.WithEvalRepresentation(null).HasEvalRepresentation);
    }

    [Fact]
    public void Bezier_InsertPoleAfter_refuses_a_25_pole_curve()
    {
        // OCCT: InsertPoleAfter throws for 25 poles, though a curve of 26 poles (degree 25) can be constructed
        var aPoles = new NCollection_Array1<Point3D>(1, 25);
        for (int i = 1; i <= 25; i++)
        {
            aPoles[i] = new Point3D(i, i * i % 7, 0);
        }
        var aC = new BezierCurve3D(aPoles);
        Assert.Throws<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => aC.PoleInsertedAfter(1, new Point3D(0, 1, 0)));
        Assert.Equal(25, aC.Increased(25).Degree());
    }

    [Fact]
    public void Trimmed_and_offset_curves_copy_their_basis()
    {
        var aCircle = new CircleCurve3D(new Frame3D(), 2.0);
        var aT = new TrimmedCurve3D(aCircle, 0.0, M_PI);
        Assert.NotSame(aCircle, aT.BasisCurve());
        var aT2 = new TrimmedCurve3D(aT, 0.5, 1.0);
        Assert.IsType<CircleCurve3D>(aT2.BasisCurve());

        var anOffset = new OffsetCurve3D(aT, 1.0, new Direction3D(0, 0, 1));
        Assert.IsType<TrimmedCurve3D>(anOffset.BasisCurve());
        var anOffset2 = new OffsetCurve3D(anOffset, 0.5, new Direction3D(0, 0, 1));
        Assert.Equal(1.5, anOffset2.Offset(), 15);
        Assert.IsType<TrimmedCurve3D>(anOffset2.BasisCurve());
        Assert.True(anOffset2.Value(0.0).IsEqual(new Point3D(3.5, 0, 0), 1e-12));
    }

    // Decision 127: shared basis curves are frozen; their owner copies on write

    [Fact]
    public void TrimmedCurve_BasisCurve_frozen_and_copied_on_write()
    {
        BSplineCurve3D aCaller = Cubic();
        var aTrim = new TrimmedCurve3D(aCaller, 0.2, 0.8);
        // The constructor copies the basis: the caller's curve isn't shared nor frozen
        Assert.False(aCaller.IsFrozen);

        Curve3D aBasis = aTrim.BasisCurve();
        Assert.True(aBasis.IsFrozen);
        Assert.NotSame(aCaller, aBasis);
        Assert.Same(aBasis, aTrim.BasisCurve());
        Point3D aBasisPoint = aBasis.Value(0.5);

        // The trimmed curve is still mutable: it copies its basis first
        aTrim.Transform(Transform3D.Translation(new Vector3D(10, 0, 0)));
        Assert.True(aTrim.Value(0.5).IsEqual(new Point3D(aBasisPoint.X() + 10, aBasisPoint.Y(), aBasisPoint.Z()), 1e-12));
        Assert.Equal(aBasisPoint, aBasis.Value(0.5));
        Assert.NotSame(aBasis, aTrim.BasisCurve());

        Curve3D aBasis2 = aTrim.BasisCurve();
        Point3D aStart = aTrim.StartPoint();
        Point3D aBasis2Point = aBasis2.Value(0.5);
        aTrim.Reverse();
        Assert.True(aStart.IsEqual(aTrim.EndPoint(), 1e-12));
        Assert.Equal(aBasis2Point, aBasis2.Value(0.5));

        // Sense = false reverses the (unshared) basis in the constructor
        var aReversed = new TrimmedCurve3D(aCaller, 0.2, 0.8, false);
        Assert.True(aReversed.StartPoint().IsEqual(aCaller.Value(0.8), 1e-12));

        // A trimmed curve of a trimmed curve doesn't freeze the inner basis
        var anInner = new TrimmedCurve3D(Cubic(), 0.2, 0.8);
        var anOuter = new TrimmedCurve3D(anInner, 0.3, 0.6);
        Assert.IsType<BSplineCurve3D>(anOuter.BasisCurve());
        anInner.Transform(Transform3D.Translation(new Vector3D(1, 1, 1))); // would throw if the constructor had frozen its basis
    }

    [Fact]
    public void OffsetCurve_BasisCurve_frozen_and_copied_on_write()
    {
        BSplineCurve3D aCaller = Cubic();
        var anOffset = new OffsetCurve3D(aCaller, 0.5, new Direction3D(0, 0, 1));
        // Unlike Geom2d, OCCT's Geom_OffsetCurve copies the caller's curve: it isn't shared nor frozen
        Assert.False(aCaller.IsFrozen);
        Assert.NotSame(aCaller, anOffset.BasisCurve());

        Curve3D aBasis = anOffset.BasisCurve();
        Assert.True(aBasis.IsFrozen);
        Assert.Same(aBasis, anOffset.BasisCurve());
        Point3D aBasisPoint = aBasis.Value(0.5);
        Point3D anOffsetPoint = anOffset.Value(0.5);

        anOffset.Transform(Transform3D.Translation(new Vector3D(0, 5, 0)));
        Assert.True(anOffset.Value(0.5).IsEqual(new Point3D(anOffsetPoint.X(), anOffsetPoint.Y() + 5, anOffsetPoint.Z()), 1e-12));
        Assert.Equal(aBasisPoint, aBasis.Value(0.5));
        Assert.NotSame(aBasis, anOffset.BasisCurve());

        Curve3D aBasis2 = anOffset.BasisCurve();
        Assert.True(aBasis2.IsFrozen);
        Point3D aBasis2Point = aBasis2.Value(0.5);
        anOffset.Reverse();
        Assert.Equal(-0.5, anOffset.Offset());
        Assert.Equal(aBasis2Point, aBasis2.Value(0.5));

        // An offset of a trimmed curve rebuilds the trimmed curve: the caller's isn't frozen
        var aTrim = new TrimmedCurve3D(Cubic(), 0.2, 0.8);
        var anOffsetOfTrim = new OffsetCurve3D(aTrim, 0.5, new Direction3D(0, 0, 1));
        Assert.False(aTrim.IsFrozen);
        Assert.NotSame(aTrim, anOffsetOfTrim.BasisCurve());
        aTrim.Transform(Transform3D.Translation(new Vector3D(1, 1, 1)));

        // An offset of an offset adds the offset vectors (-0.5 Z + 0.25 Z = 0.25 along -Z)
        var anOffsetOfOffset = new OffsetCurve3D(anOffset, 0.25, new Direction3D(0, 0, 1));
        Assert.Equal(0.25, anOffsetOfOffset.Offset(), 15);
        Assert.True(anOffsetOfOffset.Direction().IsEqual(new Direction3D(0, 0, -1), 0.0));
        anOffset.Transform(Transform3D.Translation(new Vector3D(1, 1, 1)));
    }

    // Reproductions of OCCT issues (occt-issues.md)

    /// <summary>
    /// occt-issues.md, Geom_OffsetCurveUtils::CalculateD3 (the same code as Geom2d's): the D2 it returns subtracts
    /// the Ndir term that CalculateD2 adds, and its D3 has wrong DNdir and Ndir coefficients, so EvalD3 disagrees
    /// with EvalD2 (and with the derivative of EvalD2) wherever the basis curve's speed varies. On a circle they agree.
    /// </summary>
    [Fact]
    public void OcctIssue_OffsetCurveEvalD3_wrong_where_basis_speed_varies()
    {
        var anEllipse = new EllipseCurve3D(new Frame3D(), 3.0, 1.0);
        var anOffset = new OffsetCurve3D(anEllipse, 0.5, new Direction3D(0, 0, 1));
        double aU = 0.7, h = 1e-4;

        Curve3D.ResD2 aD2 = anOffset.EvalD2(aU);
        Curve3D.ResD3 aD3 = anOffset.EvalD3(aU);
        Vector3D aFdD3 = (anOffset.EvalD2(aU + h).D2 - anOffset.EvalD2(aU - h).D2) / (2.0 * h);
        Vector3D aFdD2 = (anOffset.EvalD1(aU + h).D1 - anOffset.EvalD1(aU - h).D1) / (2.0 * h);

        // EvalD2's D2 is right; point and D1 agree between EvalD2 and EvalD3
        Assert.True((aFdD2 - aD2.D2).Magnitude() <= 1e-6);
        Assert.True(aD2.Point.IsEqual(aD3.Point, 1e-15));
        Assert.True((aD2.D1 - aD3.D1).Magnitude() <= 1e-15);
        // OCCT: EvalD3's D2 and D3 are wrong (the values of occt-issues.md: offset 0.5 of the ellipse (3, 1) at 0.7)
        Assert.True((aD3.D2 - aD2.D2).Magnitude() > 1e-3);
        Assert.True((aD3.D3 - aFdD3).Magnitude() > 1e-3);
        Assert.True((aD3.D2 - new Vector3D(-2.597, -3.131, 0)).Magnitude() < 1e-3);
        Assert.True((aD2.D2 - new Vector3D(-1.794, -1.102, 0)).Magnitude() < 1e-3);

        // The same wrong values as Geom2d_OffsetCurve (Ndir = D1 ^ Z is Geom2d's normal)
        var anOffset2d = new OffsetCurve2D(new EllipseCurve2D(new Axis2D(), 3.0, 1.0), 0.5);
        Curve2D.ResD3 aD3_2d = anOffset2d.EvalD3(aU);
        Assert.True(Math.Abs(aD3.D2.X() - aD3_2d.D2.X()) < 1e-12 && Math.Abs(aD3.D2.Y() - aD3_2d.D2.Y()) < 1e-12);
        Assert.True(Math.Abs(aD3.D3.X() - aD3_2d.D3.X()) < 1e-12 && Math.Abs(aD3.D3.Y() - aD3_2d.D3.Y()) < 1e-12);

        // On a circle they agree
        var aCircleOffset = new OffsetCurve3D(new CircleCurve3D(new Frame3D(), 3.0), 0.5, new Direction3D(0, 0, 1));
        Curve3D.ResD2 aCD2 = aCircleOffset.EvalD2(aU);
        Curve3D.ResD3 aCD3 = aCircleOffset.EvalD3(aU);
        Vector3D aCFdD3 = (aCircleOffset.EvalD2(aU + h).D2 - aCircleOffset.EvalD2(aU - h).D2) / (2.0 * h);
        Assert.True((aCD2.D2 - aCD3.D2).Magnitude() <= 1e-12);
        Assert.True((aCFdD3 - aCD3.D3).Magnitude() <= 1e-6);
    }

    /// <summary>
    /// occt-issues.md: every Geom_BSplineCurve modifier drops an attached evaluation representation (unlike
    /// Geom2d_BSplineCurve's); Geom_BezierCurve drops it only in Transform and init (constructors, Increase, pole
    /// insertion and removal) and keeps it through SetPole, SetWeight, Reverse and Segment, as Geom2d_BezierCurve.
    /// </summary>
    [Fact]
    public void OcctIssue_eval_representation_kept_or_dropped_as_in_OCCT()
    {
        static RepCurveDesc.Full Rep(Curve3D theC) => new() { Representation = (Curve3D)theC.Copy() };
        Transform3D aMove = Transform3D.Translation(new Vector3D(10, 0, 0));

        BSplineCurve3D aB = Cubic(true);
        aB.SetEvalRepresentation(Rep(aB));
        aB.SetPole(2, new Point3D(1, 5, 0));
        Assert.False(aB.HasEvalRepresentation);
        aB.SetEvalRepresentation(Rep(aB));
        aB.InsertKnot(0.25);
        Assert.False(aB.HasEvalRepresentation);
        aB.SetEvalRepresentation(Rep(aB));
        aB.Reverse();
        Assert.False(aB.HasEvalRepresentation);
        aB.SetEvalRepresentation(Rep(aB));
        aB.Transform(aMove);
        Assert.False(aB.HasEvalRepresentation);

        var aPoles = NCollection_Array1<Point3D>.From([new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(2, -1, 1),
                                                      new Point3D(3, 1, 0)]);
        var aZ = new BezierCurve3D(aPoles);
        Point3D aBefore = aZ.Value(0.3);
        aZ.SetEvalRepresentation(Rep(aZ));
        aZ.SetPole(2, new Point3D(1, 5, 0));
        aZ.SetWeight(3, 2.0);
        aZ.Reverse();
        aZ.Segment(0.1, 0.9);
        // OCCT: still evaluated through the old representation
        Assert.True(aZ.HasEvalRepresentation);
        Assert.Equal(aBefore, aZ.EvalD0(0.3));
        aZ.Transform(aMove);
        Assert.False(aZ.HasEvalRepresentation);
        Assert.True(aZ.EvalD0(0.3).X() > 9.0);

        var aZ2 = new BezierCurve3D(aPoles);
        aZ2.SetEvalRepresentation(Rep(aZ2));
        aZ2.Increase(4);
        Assert.False(aZ2.HasEvalRepresentation);

        BezierCurve3D aWith = new BezierCurve3D(aPoles).WithEvalRepresentation(Rep(aZ2));
        Assert.True(aWith.HasEvalRepresentation);
        Assert.False(aWith.WithoutEvalRepresentation().HasEvalRepresentation);
    }
}
