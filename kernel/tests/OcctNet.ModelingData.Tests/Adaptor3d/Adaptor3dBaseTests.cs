// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.TopAbs_Orientation;

namespace OcctNet.ModelingData.Tests;

/// <summary>The adaptor bases, Adaptor3d_HVertex and Adaptor3d_InterFunc.</summary>
public class Adaptor3dBaseTests
{
    /// <summary>An adaptor that redefines only the evaluation, as a derived adaptor may.</summary>
    private sealed class LineAdaptor : Adaptor3d_Curve
    {
        public override Point3D EvalD0(double theU) => new Point3D(theU, 0, 0);

        public override Geom_Curve.ResD1 EvalD1(double theU) =>
            new Geom_Curve.ResD1 { Point = EvalD0(theU), D1 = new Vector3D(1, 0, 0) };
    }

    [Fact]
    public void Base_adaptors_throw_NotImplemented()
    {
        Adaptor3d_Curve C = new();
        Assert.Throws<MethodNotImplementedException>(() => C.FirstParameter());
        Assert.Throws<MethodNotImplementedException>(() => C.Value(0));
        Assert.Throws<MethodNotImplementedException>(() => C.GetType());
        Assert.Throws<MethodNotImplementedException>(() => C.BSpline());
        Assert.Equal(typeof(Adaptor3d_Curve), C.DynamicType());

        Adaptor3d_Surface S = new();
        Assert.Throws<MethodNotImplementedException>(() => S.UResolution(1));
        Assert.Throws<MethodNotImplementedException>(() => S.D1(0, 0, out _, out _, out _));
        Assert.Throws<MethodNotImplementedException>(() => S.BasisSurface());

        Adaptor2d_Curve2d C2 = new();
        Assert.Throws<MethodNotImplementedException>(() => C2.EvalD1(0)); // goes through D1
        Assert.Equal(20, C2.NbSamples());
    }

    [Fact]
    public void Derived_adaptor_gets_D_methods_from_Eval()
    {
        LineAdaptor L = new();
        L.D1(2.5, out Point3D P, out Vector3D V);
        Assert.Equal(new Point3D(2.5, 0, 0), P);
        Assert.Equal(new Vector3D(1, 0, 0), V);
        Assert.Throws<MethodNotImplementedException>(() => L.D2(0, out _, out _, out _));
    }

    [Fact]
    public void Line2d_Eval_goes_through_D()
    {
        Adaptor2d_Line2d L = new(new Point2D(1, 1), new Direction2D(0, 1), 0, 5);
        Geom2d_Curve.ResD2 R = L.EvalD2(2);
        Assert.Equal(new Point2D(1, 3), R.Point);
        Assert.Equal(new Vector2D(0, 1), R.D1);
        Assert.Equal(new Vector2D(0, 0), R.D2);
        Assert.Equal(new Vector2D(0, 0), L.EvalDN(2, 3));
        Assert.Throws<OutOfRangeException>(() => L.DN(2, 0));
    }

    [Fact]
    public void HVertex_and_InterFunc()
    {
        Adaptor2d_Line2d L = new(new Point2D(1, 1), new Direction2D(0, 1), 0, 5);
        Adaptor3d_HVertex V = new(new Point2D(1, 4), TopAbs_REVERSED, 1e-3);
        Assert.Equal(3.0, V.Parameter(L));
        Assert.Equal(1e-3, V.Resolution(L));
        Assert.Equal(TopAbs_REVERSED, V.Orientation());
        Assert.True(V.IsSame(new Adaptor3d_HVertex(new Point2D(1, 4 + 1e-8), TopAbs_FORWARD, 0)));
        Assert.False(V.IsSame(new Adaptor3d_HVertex()));

        Adaptor3d_InterFunc F = new(L, 2.5, 2); // V(t) - 2.5 = 1 + t - 2.5
        Assert.True(F.Values(1, out double f, out double d));
        Assert.Equal(-0.5, f);
        Assert.Equal(1.0, d);
        Assert.True(F.Value(3, out f));
        Assert.Equal(1.5, f);
        Assert.Throws<ConstructionErrorException>(() => new Adaptor3d_InterFunc(L, 0, 3));
    }
}
