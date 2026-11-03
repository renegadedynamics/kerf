// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;
using Kerf.Geometry;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Axes, frames, coordinate systems and transforms (docs/api/geometry.md).</summary>
public class AxisFrameTransformTests
{
    [Fact]
    public void Axes_and_frames_have_their_parts_as_properties()
    {
        var axis = new Axis3D(new Point3D(1, 2, 3), Direction3D.UnitZ);
        Assert.Equal(new Point3D(1, 2, 3), axis.Location);
        Assert.Equal(Direction3D.UnitZ, axis.Direction);

        Frame3D world = Frame3D.World;
        Assert.Equal(Point3D.Origin, world.Location);
        Assert.Equal(Direction3D.UnitZ, world.Direction);
        Assert.Equal(Direction3D.UnitX, world.XDirection);
        Assert.Equal(Direction3D.UnitY, world.YDirection);
        Assert.Equal(Axis3D.ZAxis, world.Axis);
        Assert.Equal(Direction3D.UnitX, Frame3D.YZ.Direction);
        Assert.Equal(Direction3D.UnitY, Frame3D.ZX.Direction);
    }

    [Fact]
    public void The_constants_are_gps()
    {
        Assert.Equal(Direction3D.UnitX, Axis3D.XAxis.Direction);
        Assert.Equal(Direction3D.UnitY, Axis3D.YAxis.Direction);
        Assert.Equal(Point3D.Origin, Axis3D.XAxis.Location);
        Assert.Equal(Direction2D.UnitX, Axis2D.XAxis.Direction);
        Assert.Equal(Direction2D.UnitY, Axis2D.YAxis.Direction);
        Assert.Equal(Point2D.Origin, Frame2D.World.Location);
        Assert.Equal(Direction2D.UnitY, Frame2D.World.YDirection);
    }

    [Fact]
    public void A_coordinate_system_knows_its_handedness()
    {
        CoordinateSystem world = CoordinateSystem.World;
        Assert.True(world.IsDirect);
        Assert.False(world.YReversed().IsDirect);
        Assert.Equal(Frame3D.World, world.Frame);
        Assert.Equal(Direction3D.UnitX, world.XDirection);
    }

    [Fact]
    public void Transforms_compose_and_apply_with_operators()
    {
        Transform3D rotate = Transform3D.Rotation(Axis3D.ZAxis, Math.PI / 2);
        Transform3D move = Transform3D.Translation(new Vector3D(10, 0, 0));
        Point3D p = (move * rotate) * new Point3D(1, 0, 0);
        Assert.True(p.IsEqual(new Point3D(10, 1, 0), 1e-12));

        Vector3D v = move * new Vector3D(1, 0, 0);
        Assert.Equal(new Vector3D(1, 0, 0), v);
        Assert.True((rotate * Direction3D.UnitX).IsEqual(Direction3D.UnitY, 1e-12));
        Assert.Equal(new Vector3D(10, 0, 0), move.TranslationPart);
        Assert.Equal(TransformForm.Translation, move.Form);
        Assert.Equal(1, rotate.ScaleFactor);
        Assert.False(rotate.IsNegative);
    }

    [Fact]
    public void The_identity_is_a_static_and_inverts_to_itself()
    {
        Transform3D identity = Transform3D.Identity;
        Assert.Equal(TransformForm.Identity, identity.Form);
        Assert.Equal(new Point3D(1, 2, 3), identity * new Point3D(1, 2, 3));
        Transform3D scale = Transform3D.Scale(Point3D.Origin, 2);
        Assert.True((scale * scale.Inverted() * new Point3D(1, 2, 3)).IsEqual(new Point3D(1, 2, 3), 1e-12));
        Assert.Equal(Transform2D.Identity * new Point2D(1, 2), new Point2D(1, 2));
    }

    [Fact]
    public void Euler_angles_round_trip()
    {
        Transform3D t = Transform3D.Rotation(EulerSequence.YawPitchRoll, 0.3, 0.2, 0.1);
        (double alpha, double beta, double gamma) = t.GetEulerAngles(EulerSequence.YawPitchRoll);
        Assert.Equal(0.3, alpha, 12);
        Assert.Equal(0.2, beta, 12);
        Assert.Equal(0.1, gamma, 12);
    }

    [Fact]
    public void A_transform_converts_to_a_System_Numerics_matrix()
    {
        Transform3D t = Transform3D.Translation(new Vector3D(1, 2, 3)) * Transform3D.Rotation(Axis3D.ZAxis, Math.PI / 2);
        Matrix4x4 m = t.ToMatrix4x4();
        Vector3 moved = Vector3.Transform(new Vector3(1, 0, 0), m);
        Point3D expected = t * new Point3D(1, 0, 0);
        Assert.Equal((float)expected.X, moved.X, 5);
        Assert.Equal((float)expected.Y, moved.Y, 5);
        Assert.Equal((float)expected.Z, moved.Z, 5);
    }

    [Fact]
    public void The_2D_transform_applies_with_operators()
    {
        Transform2D rotate = new Transform2D().WithRotation(Point2D.Origin, Math.PI / 2);
        Assert.True((rotate * new Point2D(1, 0)).IsEqual(new Point2D(0, 1), 1e-12));
        Assert.True((rotate * new Vector2D(1, 0)).IsEqual(new Vector2D(0, 1), 1e-12, 1e-12));
        Assert.True((rotate * Direction2D.UnitX).IsEqual(Direction2D.UnitY, 1e-12));
        Transform2D move = new Transform2D().WithTranslation(new Vector2D(2, 3));
        Assert.Equal(new Vector2D(2, 3), move.TranslationPart);
    }

    [Fact]
    public void A_matrix_has_its_determinant_as_a_property()
    {
        Matrix3x3 identity = Matrix3x3.Identity;
        Assert.Equal(1, identity.Determinant);
        Assert.False(identity.IsSingular);
        Assert.Equal(8, (identity * 2).Determinant);
    }
}
