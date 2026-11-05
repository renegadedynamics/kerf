// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>Geom_Transformation against gp_Trsf, and its freezing and non-mutating twins.</summary>
public class Geom_TransformationTests
{
    private static readonly Axis3D Axis = new(new Point3D(1, 2, 3), new Direction3D(1, 1, 0));

    private static void AssertSame(in Transform3D theExpected, GeometryTransformation theActual)
    {
        Assert.Equal(theExpected, theActual.Trsf());
        for (int r = 1; r <= 3; ++r)
        {
            for (int c = 1; c <= 4; ++c)
            {
                Assert.Equal(theExpected.Value(r, c), theActual.Value(r, c));
            }
        }
        Assert.Equal(theExpected.Form, theActual.Form());
        Assert.Equal(theExpected.ScaleFactor, theActual.ScaleFactor());
        Assert.Equal(theExpected.IsNegative, theActual.IsNegative);
    }

    [Fact]
    public void Default_is_identity()
    {
        GeometryTransformation T = new();
        Assert.Equal(Identity, T.Form());
        AssertSame(new Transform3D(), T);
    }

    [Fact]
    public void Setters_match_gp_Trsf()
    {
        CoordinateSystem aFrom = new(new Point3D(1, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));
        CoordinateSystem aTo = new(new Point3D(0, 5, 0), new Direction3D(0, 1, 0), new Direction3D(0, 0, 1));

        AssertSame(Transform3D.Rotation(Axis, 0.7), GeometryTransformation.Rotation(Axis, 0.7));
        AssertSame(Transform3D.Mirror(new Point3D(1, 2, 3)), GeometryTransformation.Mirror(new Point3D(1, 2, 3)));
        AssertSame(Transform3D.Mirror(Axis), GeometryTransformation.Mirror(Axis));
        AssertSame(Transform3D.Mirror(new Frame3D()), GeometryTransformation.Mirror(new Frame3D()));
        AssertSame(Transform3D.Scale(new Point3D(1, 1, 1), -2.5), GeometryTransformation.Scale(new Point3D(1, 1, 1), -2.5));
        AssertSame(Transform3D.Translation(new Vector3D(1, 2, 3)), GeometryTransformation.Translation(new Vector3D(1, 2, 3)));
        AssertSame(Transform3D.Translation(new Point3D(1, 2, 3), new Point3D(0, 0, 1)),
                   GeometryTransformation.Translation(new Point3D(1, 2, 3), new Point3D(0, 0, 1)));
        AssertSame(Transform3D.Transformation(aFrom, aTo), GeometryTransformation.Transformation(aFrom, aTo));
        AssertSame(Transform3D.Transformation(aTo), GeometryTransformation.Transformation(aTo));

        // The internal mutators, as ported code calls them
        GeometryTransformation T = new();
        T.SetRotation(Axis, 0.7);
        AssertSame(Transform3D.Rotation(Axis, 0.7), T);
        T.SetTrsf(Transform3D.Scale(new Point3D(0, 0, 0), 3));
        AssertSame(Transform3D.Scale(new Point3D(0, 0, 0), 3), T);
    }

    [Fact]
    public void Composition_matches_gp_Trsf()
    {
        Transform3D R = Transform3D.Rotation(Axis, 0.3);
        Transform3D S = Transform3D.Translation(new Vector3D(4, -1, 2)).Multiplied(Transform3D.Scale(new Point3D(1, 0, 0), 2));
        GeometryTransformation GR = new(R), GS = new(S);

        AssertSame(R.Multiplied(S), GR.Multiplied(GS));
        AssertSame(R.PreMultiplied(S), GR.PreMultiplied(GS));
        AssertSame(S.Inverted(), GS.Inverted());
        AssertSame(R.Powered(3), GR.Powered(3));
        AssertSame(R.Powered(-2), GR.Powered(-2));

        GeometryTransformation M = GR.Copy();
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
        Transform3D R = Transform3D.Rotation(Axis, 1.1);
        double x1 = 1, y1 = -2, z1 = 0.5, x2 = 1, y2 = -2, z2 = 0.5;
        R.Transforms(ref x1, ref y1, ref z1);
        new GeometryTransformation(R).Transforms(ref x2, ref y2, ref z2);
        Assert.Equal(x1, x2);
        Assert.Equal(y1, y2);
        Assert.Equal(z1, z2);
    }

    [Fact]
    public void Frozen_transformation_rejects_mutators()
    {
        GeometryTransformation T = GeometryTransformation.Rotation(Axis, 0.5);
        T.Freeze();
        Assert.True(T.IsFrozen);
        EngineAssert.Throws<ImmutableObjectException>(() => T.SetTranslation(new Vector3D(1, 0, 0)));
        EngineAssert.Throws<ImmutableObjectException>(() => T.Invert());
        EngineAssert.Throws<ImmutableObjectException>(() => T.Multiply(new GeometryTransformation()));
        EngineAssert.Throws<ImmutableObjectException>(() => T.PreMultiply(new GeometryTransformation()));
        EngineAssert.Throws<ImmutableObjectException>(() => T.Power(2));
        EngineAssert.Throws<ImmutableObjectException>(() => T.SetTrsf(new Transform3D()));
        AssertSame(Transform3D.Rotation(Axis, 0.5), T);

        // Non-mutating forms still work on a frozen object and return unfrozen ones
        GeometryTransformation aCopy = T.Copy();
        Assert.False(aCopy.IsFrozen);
        aCopy.SetTranslation(new Vector3D(1, 0, 0));
        Assert.False(T.Inverted().IsFrozen);
        Assert.False(T.WithScale(new Point3D(0, 0, 0), 2).IsFrozen);
    }

    [Fact]
    public void With_methods_leave_the_original_unchanged()
    {
        GeometryTransformation T = GeometryTransformation.Rotation(Axis, 0.5);
        GeometryTransformation T2 = T.WithTranslation(new Vector3D(0, 0, 7));
        AssertSame(Transform3D.Rotation(Axis, 0.5), T);
        AssertSame(Transform3D.Translation(new Vector3D(0, 0, 7)), T2);
        AssertSame(Transform3D.Mirror(Axis), T.WithMirror(Axis));
        AssertSame(Transform3D.Scale(new Point3D(1, 2, 3), 4), T.WithScale(new Point3D(1, 2, 3), 4));
        AssertSame(Transform3D.Rotation(Axis, 0.2), T.WithRotation(Axis, 0.2));
        AssertSame(Transform3D.Rotation(Axis, 0.5), T);
    }
}
