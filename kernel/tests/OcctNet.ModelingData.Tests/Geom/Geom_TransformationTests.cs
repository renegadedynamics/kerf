// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gp_TrsfForm;

namespace OcctNet.ModelingData.Tests;

/// <summary>Geom_Transformation against gp_Trsf, and its freezing and non-mutating twins.</summary>
public class Geom_TransformationTests
{
    private static readonly gp_Ax1 Axis = new(new gp_Pnt(1, 2, 3), new gp_Dir(1, 1, 0));

    private static void AssertSame(in gp_Trsf theExpected, Geom_Transformation theActual)
    {
        Assert.Equal(theExpected, theActual.Trsf());
        for (int r = 1; r <= 3; ++r)
        {
            for (int c = 1; c <= 4; ++c)
            {
                Assert.Equal(theExpected.Value(r, c), theActual.Value(r, c));
            }
        }
        Assert.Equal(theExpected.Form(), theActual.Form());
        Assert.Equal(theExpected.ScaleFactor(), theActual.ScaleFactor());
        Assert.Equal(theExpected.IsNegative(), theActual.IsNegative());
    }

    [Fact]
    public void Default_is_identity()
    {
        Geom_Transformation T = new();
        Assert.Equal(gp_Identity, T.Form());
        AssertSame(new gp_Trsf(), T);
    }

    [Fact]
    public void Setters_match_gp_Trsf()
    {
        gp_Ax3 aFrom = new(new gp_Pnt(1, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0));
        gp_Ax3 aTo = new(new gp_Pnt(0, 5, 0), new gp_Dir(0, 1, 0), new gp_Dir(0, 0, 1));

        AssertSame(gp_Trsf.Rotation(Axis, 0.7), Geom_Transformation.Rotation(Axis, 0.7));
        AssertSame(gp_Trsf.Mirror(new gp_Pnt(1, 2, 3)), Geom_Transformation.Mirror(new gp_Pnt(1, 2, 3)));
        AssertSame(gp_Trsf.Mirror(Axis), Geom_Transformation.Mirror(Axis));
        AssertSame(gp_Trsf.Mirror(new gp_Ax2()), Geom_Transformation.Mirror(new gp_Ax2()));
        AssertSame(gp_Trsf.Scale(new gp_Pnt(1, 1, 1), -2.5), Geom_Transformation.Scale(new gp_Pnt(1, 1, 1), -2.5));
        AssertSame(gp_Trsf.Translation(new gp_Vec(1, 2, 3)), Geom_Transformation.Translation(new gp_Vec(1, 2, 3)));
        AssertSame(gp_Trsf.Translation(new gp_Pnt(1, 2, 3), new gp_Pnt(0, 0, 1)),
                   Geom_Transformation.Translation(new gp_Pnt(1, 2, 3), new gp_Pnt(0, 0, 1)));
        AssertSame(gp_Trsf.Transformation(aFrom, aTo), Geom_Transformation.Transformation(aFrom, aTo));
        AssertSame(gp_Trsf.Transformation(aTo), Geom_Transformation.Transformation(aTo));

        // The internal mutators, as ported code calls them
        Geom_Transformation T = new();
        T.SetRotation(Axis, 0.7);
        AssertSame(gp_Trsf.Rotation(Axis, 0.7), T);
        T.SetTrsf(gp_Trsf.Scale(new gp_Pnt(0, 0, 0), 3));
        AssertSame(gp_Trsf.Scale(new gp_Pnt(0, 0, 0), 3), T);
    }

    [Fact]
    public void Composition_matches_gp_Trsf()
    {
        gp_Trsf R = gp_Trsf.Rotation(Axis, 0.3);
        gp_Trsf S = gp_Trsf.Translation(new gp_Vec(4, -1, 2)).Multiplied(gp_Trsf.Scale(new gp_Pnt(1, 0, 0), 2));
        Geom_Transformation GR = new(R), GS = new(S);

        AssertSame(R.Multiplied(S), GR.Multiplied(GS));
        AssertSame(R.PreMultiplied(S), GR.PreMultiplied(GS));
        AssertSame(S.Inverted(), GS.Inverted());
        AssertSame(R.Powered(3), GR.Powered(3));
        AssertSame(R.Powered(-2), GR.Powered(-2));

        Geom_Transformation M = GR.Copy();
        M.Multiply(GS);
        AssertSame(R.Multiplied(S), M);
        M = GR.Copy();
        M.PreMultiply(GS);
        AssertSame(R.PreMultiplied(S), M);
        M = GS.Copy();
        M.Invert();
        AssertSame(S.Inverted(), M);
        M = GR.Copy();
        M.Power(4);
        AssertSame(R.Powered(4), M);
    }

    [Fact]
    public void Transforms_matches_gp_Trsf()
    {
        gp_Trsf R = gp_Trsf.Rotation(Axis, 1.1);
        double x1 = 1, y1 = -2, z1 = 0.5, x2 = 1, y2 = -2, z2 = 0.5;
        R.Transforms(ref x1, ref y1, ref z1);
        new Geom_Transformation(R).Transforms(ref x2, ref y2, ref z2);
        Assert.Equal(x1, x2);
        Assert.Equal(y1, y2);
        Assert.Equal(z1, z2);
    }

    [Fact]
    public void Frozen_transformation_rejects_mutators()
    {
        Geom_Transformation T = Geom_Transformation.Rotation(Axis, 0.5);
        T.Freeze();
        Assert.True(T.IsFrozen());
        Assert.Throws<Standard_ImmutableObject>(() => T.SetTranslation(new gp_Vec(1, 0, 0)));
        Assert.Throws<Standard_ImmutableObject>(() => T.Invert());
        Assert.Throws<Standard_ImmutableObject>(() => T.Multiply(new Geom_Transformation()));
        Assert.Throws<Standard_ImmutableObject>(() => T.PreMultiply(new Geom_Transformation()));
        Assert.Throws<Standard_ImmutableObject>(() => T.Power(2));
        Assert.Throws<Standard_ImmutableObject>(() => T.SetTrsf(new gp_Trsf()));
        AssertSame(gp_Trsf.Rotation(Axis, 0.5), T);

        // Non-mutating forms still work on a frozen object and return unfrozen ones
        Geom_Transformation aCopy = T.Copy();
        Assert.False(aCopy.IsFrozen());
        aCopy.SetTranslation(new gp_Vec(1, 0, 0));
        Assert.False(T.Inverted().IsFrozen());
        Assert.False(T.WithScale(new gp_Pnt(0, 0, 0), 2).IsFrozen());
    }

    [Fact]
    public void With_methods_leave_the_original_unchanged()
    {
        Geom_Transformation T = Geom_Transformation.Rotation(Axis, 0.5);
        Geom_Transformation T2 = T.WithTranslation(new gp_Vec(0, 0, 7));
        AssertSame(gp_Trsf.Rotation(Axis, 0.5), T);
        AssertSame(gp_Trsf.Translation(new gp_Vec(0, 0, 7)), T2);
        AssertSame(gp_Trsf.Mirror(Axis), T.WithMirror(Axis));
        AssertSame(gp_Trsf.Scale(new gp_Pnt(1, 2, 3), 4), T.WithScale(new gp_Pnt(1, 2, 3), 4));
        AssertSame(gp_Trsf.Rotation(Axis, 0.2), T.WithRotation(Axis, 0.2));
        AssertSame(gp_Trsf.Rotation(Axis, 0.5), T);
    }
}
