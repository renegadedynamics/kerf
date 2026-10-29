// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>Geom_CartesianPoint, Geom_Direction and Geom_VectorWithMagnitude, with freezing.</summary>
public class Geom_PointVectorTests
{
    private const double Tol = 1e-12;
    private static readonly gp_Ax1 Axis = new(new gp_Pnt(1, 2, 3), new gp_Dir(0, 1, 1));

    private static void AssertPnt(in gp_Pnt theExpected, in gp_Pnt theActual, double theTol = 0.0)
    {
        Assert.Equal(theExpected.X(), theActual.X(), theTol);
        Assert.Equal(theExpected.Y(), theActual.Y(), theTol);
        Assert.Equal(theExpected.Z(), theActual.Z(), theTol);
    }

    [Fact]
    public void CartesianPoint_transforms_as_gp_Pnt()
    {
        gp_Pnt P = new(3, -1, 2);
        Geom_CartesianPoint G = new(P);
        gp_Trsf T = gp_Trsf.Rotation(Axis, 0.9).Multiplied(gp_Trsf.Scale(new gp_Pnt(0, 1, 0), 1.5));

        AssertPnt(P.Transformed(T), ((Geom_CartesianPoint)G.Transformed(T)).Pnt());
        AssertPnt(P.Mirrored(new gp_Pnt(1, 1, 1)), ((Geom_Point)G.Mirrored(new gp_Pnt(1, 1, 1))).Pnt());
        AssertPnt(P.Mirrored(Axis), ((Geom_Point)G.Mirrored(Axis)).Pnt());
        AssertPnt(P.Mirrored(new gp_Ax2()), ((Geom_Point)G.Mirrored(new gp_Ax2())).Pnt());
        AssertPnt(P.Rotated(Axis, 0.4), ((Geom_Point)G.Rotated(Axis, 0.4)).Pnt());
        AssertPnt(P.Scaled(new gp_Pnt(1, 1, 1), -3), ((Geom_Point)G.Scaled(new gp_Pnt(1, 1, 1), -3)).Pnt());
        AssertPnt(P.Translated(new gp_Vec(1, 2, 3)), ((Geom_Point)G.Translated(new gp_Vec(1, 2, 3))).Pnt());
        AssertPnt(P.Translated(new gp_Pnt(1, 1, 1), new gp_Pnt(2, 0, 5)),
                  ((Geom_Point)G.Translated(new gp_Pnt(1, 1, 1), new gp_Pnt(2, 0, 5))).Pnt());

        // The original is unchanged
        AssertPnt(P, G.Pnt());

        // The internal mutators, as ported code calls them
        G.Rotate(Axis, 0.4);
        AssertPnt(P.Rotated(Axis, 0.4), G.Pnt());
    }

    [Fact]
    public void CartesianPoint_queries_and_twins()
    {
        Geom_CartesianPoint G = new(1, 2, 3);
        G.Coord(out double x, out double y, out double z);
        Assert.Equal((1.0, 2.0, 3.0), (x, y, z));
        Assert.Equal(5.0, G.Distance(new Geom_CartesianPoint(1, 5, 7)));
        Assert.Equal(25.0, G.SquareDistance(new Geom_CartesianPoint(1, 5, 7)));

        Geom_CartesianPoint G2 = G.WithX(10).WithY(20).WithZ(30);
        AssertPnt(new gp_Pnt(10, 20, 30), G2.Pnt());
        AssertPnt(new gp_Pnt(1, 2, 3), G.Pnt());
        AssertPnt(new gp_Pnt(4, 5, 6), G.WithCoord(4, 5, 6).Pnt());
        AssertPnt(new gp_Pnt(7, 8, 9), G.WithPnt(new gp_Pnt(7, 8, 9)).Pnt());
    }

    [Fact]
    public void Frozen_point_rejects_every_mutator()
    {
        Geom_CartesianPoint G = new(1, 2, 3);
        G.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => G.SetX(0));
        Assert.Throws<Standard_ImmutableObject>(() => G.SetCoord(0, 0, 0));
        Assert.Throws<Standard_ImmutableObject>(() => G.SetPnt(new gp_Pnt()));
        Assert.Throws<Standard_ImmutableObject>(() => G.Transform(new gp_Trsf()));
        Assert.Throws<Standard_ImmutableObject>(() => G.Mirror(new gp_Pnt()));
        Assert.Throws<Standard_ImmutableObject>(() => G.Rotate(Axis, 1));
        Assert.Throws<Standard_ImmutableObject>(() => G.Scale(new gp_Pnt(), 2));
        Assert.Throws<Standard_ImmutableObject>(() => G.Translate(new gp_Vec(1, 0, 0)));
        Assert.Throws<Standard_ImmutableObject>(() => G.Translate(new gp_Pnt(), new gp_Pnt(1, 0, 0)));
        AssertPnt(new gp_Pnt(1, 2, 3), G.Pnt());

        // Copy is unfrozen; the public non-mutating forms work on a frozen object
        Geom_CartesianPoint aCopy = (Geom_CartesianPoint)G.Copy();
        Assert.False(aCopy.IsFrozen());
        aCopy.SetX(5);
        Assert.Equal(5.0, aCopy.X());
        Geom_Geometry aMoved = G.Translated(new gp_Vec(1, 0, 0));
        Assert.False(aMoved.IsFrozen());
        AssertPnt(new gp_Pnt(2, 2, 3), ((Geom_Point)aMoved).Pnt());
        Assert.Equal(9.0, G.WithX(9).X());
    }

    [Fact]
    public void Direction_normalizes_and_rejects_null()
    {
        Geom_Direction D = new(3, 0, 4);
        Assert.Equal(0.6, D.X(), Tol);
        Assert.Equal(0.8, D.Z(), Tol);
        Assert.Equal(1.0, D.Magnitude());
        Assert.Throws<Standard_ConstructionError>(() => new Geom_Direction(0, 0, 0));
        Assert.Throws<Standard_ConstructionError>(() => new Geom_Direction(1, 0, 0).WithX(0));

        // A scale doesn't change a direction's length
        Geom_Direction S = (Geom_Direction)D.Scaled(new gp_Pnt(1, 1, 1), -2);
        Assert.Equal(-0.6, S.X(), Tol);
        Assert.Equal(-0.8, S.Z(), Tol);

        Geom_Direction DX = new(new gp_Dir(1, 0, 0));
        Geom_Direction DY = new(new gp_Dir(0, 1, 0));
        Geom_Vector C = DX.Crossed(DY);
        Assert.IsType<Geom_Direction>(C);
        Assert.Equal(1.0, C.Z(), Tol);
        Assert.Throws<Standard_ConstructionError>(() => DX.Crossed(new Geom_Direction(-1, 0, 0)));
        Assert.Equal(M_PI / 2, DX.Angle(DY), Tol);
        Assert.Equal(-M_PI / 2, DY.AngleWithRef(DX, new Geom_Direction(0, 0, 1)), Tol);

        // Cross in place, as ported code would
        Geom_Direction M = (Geom_Direction)DX.Copy();
        M.Cross(DY);
        Assert.Equal(1.0, M.Z(), Tol);
        Assert.Equal(1.0, DX.X());
        M.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => M.Cross(DX));
        Assert.Throws<Standard_ImmutableObject>(() => M.Reverse());
        Assert.Equal(-1.0, M.Reversed().Z(), Tol);
    }

    [Fact]
    public void VectorWithMagnitude_arithmetic()
    {
        Geom_VectorWithMagnitude V = new(1, 2, 2);
        Geom_VectorWithMagnitude W = new(new gp_Pnt(1, 1, 1), new gp_Pnt(1, 1, 3));
        Assert.Equal(3.0, V.Magnitude());
        Assert.Equal(9.0, V.SquareMagnitude());
        Assert.Equal(new gp_Vec(1, 2, 4), V.Added(W).Vec());
        Assert.Equal(new gp_Vec(1, 2, 0), V.Subtracted(W).Vec());
        Assert.Equal(new gp_Vec(2, 4, 4), V.Multiplied(2).Vec());
        Assert.Equal(new gp_Vec(0.5, 1, 1), V.Divided(2).Vec());
        Assert.Equal(1.0, V.Normalized().Magnitude(), Tol);
        Assert.Equal(new gp_Vec(4, -2, 0), V.Crossed(W).Vec());
        Assert.Equal(4.0, V.Dot(W));
        Assert.Equal(V.Vec().CrossCrossed(W.Vec(), new gp_Vec(1, 0, 0)), V.CrossCrossed(W, new Geom_VectorWithMagnitude(1, 0, 0)).Vec());
        Assert.Equal(new gp_Vec(1, 2, 2), V.Vec());

        // A vector with magnitude is scaled by a transformation, unlike a direction
        Geom_VectorWithMagnitude S = (Geom_VectorWithMagnitude)V.Scaled(new gp_Pnt(5, 5, 5), 2);
        Assert.Equal(6.0, S.Magnitude(), Tol);

        Geom_VectorWithMagnitude M = (Geom_VectorWithMagnitude)V.Copy();
        M.Add(W);
        M.Multiply(2);
        Assert.Equal(new gp_Vec(2, 4, 8), M.Vec());
        M.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => M.Normalize());
        Assert.Throws<Standard_ImmutableObject>(() => M.Subtract(W));
        Assert.Throws<Standard_ImmutableObject>(() => M.Divide(2));
        Assert.Throws<Standard_ImmutableObject>(() => M.SetVec(new gp_Vec()));
        Assert.Equal(new gp_Vec(0, 4, 8), M.WithX(0).Vec());
        Assert.Equal(new gp_Vec(2, 4, 8), M.Vec());
    }
}
