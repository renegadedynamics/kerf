// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>Geom_CartesianPoint, Geom_Direction and Geom_VectorWithMagnitude, with freezing.</summary>
public class Geom_PointVectorTests
{
    private const double Tol = 1e-12;
    private static readonly Axis3D Axis = new(new Point3D(1, 2, 3), new Direction3D(0, 1, 1));

    private static void AssertPnt(in Point3D theExpected, in Point3D theActual, double theTol = 0.0)
    {
        Assert.Equal(theExpected.X, theActual.X, theTol);
        Assert.Equal(theExpected.Y, theActual.Y, theTol);
        Assert.Equal(theExpected.Z, theActual.Z, theTol);
    }

    [Fact]
    public void CartesianPoint_transforms_as_gp_Pnt()
    {
        Point3D P = new(3, -1, 2);
        CartesianPoint G = new(P);
        Transform3D T = Transform3D.Rotation(Axis, 0.9).Multiplied(Transform3D.Scale(new Point3D(0, 1, 0), 1.5));

        AssertPnt(P.Transformed(T), ((CartesianPoint)G.Transformed(T)).Pnt());
        AssertPnt(P.Mirrored(new Point3D(1, 1, 1)), ((Point)G.Mirrored(new Point3D(1, 1, 1))).Pnt());
        AssertPnt(P.Mirrored(Axis), ((Point)G.Mirrored(Axis)).Pnt());
        AssertPnt(P.Mirrored(new Frame3D()), ((Point)G.Mirrored(new Frame3D())).Pnt());
        AssertPnt(P.Rotated(Axis, 0.4), ((Point)G.Rotated(Axis, 0.4)).Pnt());
        AssertPnt(P.Scaled(new Point3D(1, 1, 1), -3), ((Point)G.Scaled(new Point3D(1, 1, 1), -3)).Pnt());
        AssertPnt(P.Translated(new Vector3D(1, 2, 3)), ((Point)G.Translated(new Vector3D(1, 2, 3))).Pnt());
        AssertPnt(P.Translated(new Point3D(1, 1, 1), new Point3D(2, 0, 5)),
                  ((Point)G.Translated(new Point3D(1, 1, 1), new Point3D(2, 0, 5))).Pnt());

        // The original is unchanged
        AssertPnt(P, G.Pnt());

        // The internal mutators, as ported code calls them
        G.Rotate(Axis, 0.4);
        AssertPnt(P.Rotated(Axis, 0.4), G.Pnt());
    }

    [Fact]
    public void CartesianPoint_queries_and_twins()
    {
        CartesianPoint G = new(1, 2, 3);
        G.Coord(out double x, out double y, out double z);
        Assert.Equal((1.0, 2.0, 3.0), (x, y, z));
        Assert.Equal(5.0, G.Distance(new CartesianPoint(1, 5, 7)));
        Assert.Equal(25.0, G.SquareDistance(new CartesianPoint(1, 5, 7)));

        CartesianPoint G2 = G.WithX(10).WithY(20).WithZ(30);
        AssertPnt(new Point3D(10, 20, 30), G2.Pnt());
        AssertPnt(new Point3D(1, 2, 3), G.Pnt());
        AssertPnt(new Point3D(4, 5, 6), G.WithCoord(4, 5, 6).Pnt());
        AssertPnt(new Point3D(7, 8, 9), G.WithPnt(new Point3D(7, 8, 9)).Pnt());
    }

    [Fact]
    public void Frozen_point_rejects_every_mutator()
    {
        CartesianPoint G = new(1, 2, 3);
        G.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => G.SetX(0));
        EngineAssert.Throws<ImmutableObjectException>(() => G.SetCoord(0, 0, 0));
        EngineAssert.Throws<ImmutableObjectException>(() => G.SetPnt(new Point3D()));
        EngineAssert.Throws<ImmutableObjectException>(() => G.Transform(new Transform3D()));
        EngineAssert.Throws<ImmutableObjectException>(() => G.Mirror(new Point3D()));
        EngineAssert.Throws<ImmutableObjectException>(() => G.Rotate(Axis, 1));
        EngineAssert.Throws<ImmutableObjectException>(() => G.Scale(new Point3D(), 2));
        EngineAssert.Throws<ImmutableObjectException>(() => G.Translate(new Vector3D(1, 0, 0)));
        EngineAssert.Throws<ImmutableObjectException>(() => G.Translate(new Point3D(), new Point3D(1, 0, 0)));
        AssertPnt(new Point3D(1, 2, 3), G.Pnt());

        // Copy is unfrozen; the public non-mutating forms work on a frozen object
        CartesianPoint aCopy = (CartesianPoint)G.Copy();
        Assert.False(aCopy.IsFrozen);
        aCopy.SetX(5);
        Assert.Equal(5.0, aCopy.X());
        Geometry3D aMoved = G.Translated(new Vector3D(1, 0, 0));
        Assert.False(aMoved.IsFrozen);
        AssertPnt(new Point3D(2, 2, 3), ((Point)aMoved).Pnt());
        Assert.Equal(9.0, G.WithX(9).X());
    }

    [Fact]
    public void Direction_normalizes_and_rejects_null()
    {
        Direction D = new(3, 0, 4);
        Assert.Equal(0.6, D.X(), Tol);
        Assert.Equal(0.8, D.Z(), Tol);
        Assert.Equal(1.0, D.Magnitude());
        EngineAssert.Throws<ConstructionErrorException>(() => new Direction(0, 0, 0));
        EngineAssert.Throws<ConstructionErrorException>(() => new Direction(1, 0, 0).WithX(0));

        // A scale doesn't change a direction's length
        Direction S = (Direction)D.Scaled(new Point3D(1, 1, 1), -2);
        Assert.Equal(-0.6, S.X(), Tol);
        Assert.Equal(-0.8, S.Z(), Tol);

        Direction DX = new(new Direction3D(1, 0, 0));
        Direction DY = new(new Direction3D(0, 1, 0));
        GeometryVector3D C = DX.Crossed(DY);
        Assert.IsType<Direction>(C);
        Assert.Equal(1.0, C.Z(), Tol);
        EngineAssert.Throws<ConstructionErrorException>(() => DX.Crossed(new Direction(-1, 0, 0)));
        Assert.Equal(M_PI / 2, DX.Angle(DY), Tol);
        Assert.Equal(-M_PI / 2, DY.AngleWithRef(DX, new Direction(0, 0, 1)), Tol);

        // Cross in place, as ported code would
        Direction M = (Direction)DX.Copy();
        M.Cross(DY);
        Assert.Equal(1.0, M.Z(), Tol);
        Assert.Equal(1.0, DX.X());
        M.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => M.Cross(DX));
        EngineAssert.Throws<ImmutableObjectException>(() => M.Reverse());
        Assert.Equal(-1.0, M.Reversed().Z(), Tol);
    }

    [Fact]
    public void VectorWithMagnitude_arithmetic()
    {
        VectorWithMagnitude V = new(1, 2, 2);
        VectorWithMagnitude W = new(new Point3D(1, 1, 1), new Point3D(1, 1, 3));
        Assert.Equal(3.0, V.Magnitude());
        Assert.Equal(9.0, V.SquareMagnitude());
        Assert.Equal(new Vector3D(1, 2, 4), V.Added(W).Vec());
        Assert.Equal(new Vector3D(1, 2, 0), V.Subtracted(W).Vec());
        Assert.Equal(new Vector3D(2, 4, 4), V.Multiplied(2).Vec());
        Assert.Equal(new Vector3D(0.5, 1, 1), V.Divided(2).Vec());
        Assert.Equal(1.0, V.Normalized().Magnitude(), Tol);
        Assert.Equal(new Vector3D(4, -2, 0), V.Crossed(W).Vec());
        Assert.Equal(4.0, V.Dot(W));
        Assert.Equal(V.Vec().CrossCrossed(W.Vec(), new Vector3D(1, 0, 0)), V.CrossCrossed(W, new VectorWithMagnitude(1, 0, 0)).Vec());
        Assert.Equal(new Vector3D(1, 2, 2), V.Vec());

        // A vector with magnitude is scaled by a transformation, unlike a direction
        VectorWithMagnitude S = (VectorWithMagnitude)V.Scaled(new Point3D(5, 5, 5), 2);
        Assert.Equal(6.0, S.Magnitude(), Tol);

        VectorWithMagnitude M = (VectorWithMagnitude)V.Copy();
        M.Add(W);
        M.Multiply(2);
        Assert.Equal(new Vector3D(2, 4, 8), M.Vec());
        M.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => M.Normalize());
        EngineAssert.Throws<ImmutableObjectException>(() => M.Subtract(W));
        EngineAssert.Throws<ImmutableObjectException>(() => M.Divide(2));
        EngineAssert.Throws<ImmutableObjectException>(() => M.SetVec(new Vector3D()));
        Assert.Equal(new Vector3D(0, 4, 8), M.WithX(0).Vec());
        Assert.Equal(new Vector3D(2, 4, 8), M.Vec());
    }
}
