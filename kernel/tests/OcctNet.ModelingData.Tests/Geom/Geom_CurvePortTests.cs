// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own tests of the Geom curves: freezing, the non-mutating twins, the returned arrays, and the OCCT
/// behaviours listed in occt-issues.md.
/// </summary>
public class Geom_CurvePortTests
{
    private static Geom_BSplineCurve Cubic(bool theRational = false)
    {
        var aPoles = NCollection_Array1<gp_Pnt>.From([new gp_Pnt(0, 0, 0), new gp_Pnt(1, 2, 0), new gp_Pnt(2, -1, 1),
                                                      new gp_Pnt(3, 1, 0), new gp_Pnt(4, 0, 2)]);
        var aKnots = NCollection_Array1<double>.From([0.0, 0.5, 1.0]);
        var aMults = NCollection_Array1<int>.From([4, 1, 4]);
        if (!theRational)
        {
            return new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
        }
        var aWeights = NCollection_Array1<double>.From([1.0, 2.0, 0.5, 1.0, 1.5]);
        return new Geom_BSplineCurve(aPoles, aWeights, aKnots, aMults, 3);
    }

    [Fact]
    public void Frozen_curves_refuse_mutators_but_give_changed_copies()
    {
        Geom_BSplineCurve aC = Cubic();
        aC.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => aC.InsertKnot(0.25));
        Assert.Throws<Standard_ImmutableObject>(() => aC.SetPole(1, new gp_Pnt(5, 5, 5)));
        Assert.Throws<Standard_ImmutableObject>(() => aC.Reverse());
        Assert.Throws<Standard_ImmutableObject>(() => aC.Transform(gp_Trsf.Translation(new gp_Vec(1, 0, 0))));
        Assert.Throws<Standard_ImmutableObject>(() => aC.ClearEvalRepresentation());

        Geom_BSplineCurve aK = aC.KnotInserted(0.25, 2);
        Assert.False(aK.IsFrozen());
        Assert.Equal(4, aK.NbKnots());
        Assert.Equal(3, aC.NbKnots());

        Geom_BSplineCurve aP = aC.WithPole(1, new gp_Pnt(5, 5, 5));
        Assert.Equal(new gp_Pnt(5, 5, 5), aP.Pole(1));
        Assert.Equal(new gp_Pnt(0, 0, 0), aC.Pole(1));

        // Resolution caches a value but is not a mutator
        aC.Resolution(1e-3, out double aTol);
        Assert.True(aTol > 0);

        var aL = new Geom_Line(new gp_Pnt(), new gp_Dir(1, 0, 0));
        aL.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => aL.SetLocation(new gp_Pnt(1, 1, 1)));
        Assert.Equal(new gp_Pnt(1, 1, 1), aL.WithLocation(new gp_Pnt(1, 1, 1)).Position().Location());
    }

    [Fact]
    public void Returned_arrays_are_copies()
    {
        Geom_BSplineCurve aC = Cubic(true);
        PackedList<gp_Pnt> aPoles = aC.Poles();
        Assert.Equal(5, aPoles.Count);
        Assert.Equal(aC.Pole(2), aPoles[1]);
        PackedList<double>? aWeights = aC.Weights();
        Assert.NotNull(aWeights);
        Assert.Equal(2.0, aWeights[1]);
        Assert.Equal(new[] { 4, 1, 4 }, aC.Multiplicities().AsSpan().ToArray());
        Assert.Equal(9, aC.KnotSequence().Count);

        Geom_BSplineCurve aChanged = aC.WithPole(2, new gp_Pnt(9, 9, 9));
        Assert.Equal(new gp_Pnt(1, 2, 0), aPoles[1]);
        Assert.Equal(new gp_Pnt(9, 9, 9), aChanged.Poles()[1]);

        Assert.Null(Cubic().Weights());
        Assert.Equal(5, Cubic().WeightsArray().Count);
    }

    [Fact]
    public void Twins_match_the_mutators()
    {
        Geom_BSplineCurve aC = Cubic(true);
        Geom_BSplineCurve aM = (Geom_BSplineCurve)aC.Copy();
        aM.IncreaseDegree(5);
        Geom_BSplineCurve aT = aC.DegreeIncreased(5);
        Assert.True(aM.IsEqual(aT, 0.0));

        Assert.Null(aC.KnotRemoved(2, 0, 1e-12));
        Geom_BSplineCurve aInserted = aC.KnotInserted(0.75);
        Geom_BSplineCurve? aRemoved = aInserted.KnotRemoved(3, 0, 1e-9);
        Assert.NotNull(aRemoved);
        Assert.Equal(3, aRemoved.NbKnots());

        Geom_BSplineCurve aSeg = aC.Segmented(0.2, 0.8);
        Assert.Equal(0.2, aSeg.FirstParameter());
        Assert.Equal(0.8, aSeg.LastParameter());
        Assert.True(aSeg.Value(0.5).IsEqual(aC.Value(0.5), 1e-12));
    }

    [Fact]
    public void Segment_default_tolerance_is_PConfusion()
    {
        // The default argument of Segment is the constant 1e-7 * 0.01, which must be Precision::PConfusion().
        Geom_BSplineCurve aC = Cubic();
        Geom_BSplineCurve aA = aC.Segmented(0.1, 0.9);
        Geom_BSplineCurve aB = aC.Segmented(0.1, 0.9, Precision.PConfusion());
        Assert.True(aA.IsEqual(aB, 0.0));
        Assert.Equal(1e-7 * 0.01, Precision.PConfusion());
    }

    [Fact]
    public void Eval_representation_is_copied_in()
    {
        Geom_BSplineCurve aC = Cubic();
        var aRep = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        var aDesc = new GeomEval_RepCurveDesc.Full { Representation = aRep };
        Geom_BSplineCurve aWith = aC.WithEvalRepresentation(aDesc);
        Assert.True(aWith.HasEvalRepresentation());
        Assert.False(aC.HasEvalRepresentation());
        Assert.Equal(new gp_Pnt(0, 0, 2), aWith.Value(2));

        // changing the caller's descriptor afterwards doesn't reach the curve
        aDesc.Representation = new Geom_Line(new gp_Pnt(1, 1, 1), new gp_Dir(1, 0, 0));
        Assert.Equal(new gp_Pnt(0, 0, 2), aWith.Value(2));

        // a modification drops it, as in OCCT
        Assert.False(aWith.WithPole(1, new gp_Pnt(1, 1, 1)).HasEvalRepresentation());
        Assert.False(aWith.WithEvalRepresentation(null).HasEvalRepresentation());
    }

    [Fact]
    public void Bezier_InsertPoleAfter_refuses_a_25_pole_curve()
    {
        // OCCT: InsertPoleAfter throws for 25 poles, though a curve of 26 poles (degree 25) can be constructed
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 25);
        for (int i = 1; i <= 25; i++)
        {
            aPoles[i] = new gp_Pnt(i, i * i % 7, 0);
        }
        var aC = new Geom_BezierCurve(aPoles);
        Assert.Throws<Standard_ConstructionError>(() => aC.PoleInsertedAfter(1, new gp_Pnt(0, 1, 0)));
        Assert.Equal(25, aC.Increased(25).Degree());
    }

    [Fact]
    public void Trimmed_and_offset_curves_copy_their_basis()
    {
        var aCircle = new Geom_Circle(new gp_Ax2(), 2.0);
        var aT = new Geom_TrimmedCurve(aCircle, 0.0, M_PI);
        Assert.NotSame(aCircle, aT.BasisCurve());
        var aT2 = new Geom_TrimmedCurve(aT, 0.5, 1.0);
        Assert.IsType<Geom_Circle>(aT2.BasisCurve());

        var anOffset = new Geom_OffsetCurve(aT, 1.0, new gp_Dir(0, 0, 1));
        Assert.IsType<Geom_TrimmedCurve>(anOffset.BasisCurve());
        var anOffset2 = new Geom_OffsetCurve(anOffset, 0.5, new gp_Dir(0, 0, 1));
        Assert.Equal(1.5, anOffset2.Offset(), 15);
        Assert.IsType<Geom_TrimmedCurve>(anOffset2.BasisCurve());
        Assert.True(anOffset2.Value(0.0).IsEqual(new gp_Pnt(3.5, 0, 0), 1e-12));
    }
}
