// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// The non-abstract behaviour of Geom_Curve and Geom_Surface (D0 to DN over the Eval methods, Reversed, Period,
/// parametric transformation defaults), and of the evaluation representations (GeomEval_RepUtils), on the test
/// geometry of TestGeometry.cs.
/// </summary>
public class Geom_CurveSurfaceBaseTests
{
    [Fact]
    public void Curve_D_methods_go_through_Eval()
    {
        TestLine L = new(new gp_Pnt(1, 2, 3), new gp_Vec(1, 0, 2));
        L.D0(2, out gp_Pnt P);
        Assert.Equal(new gp_Pnt(3, 2, 7), P);
        Assert.Equal(P, L.Value(2));
        L.D1(2, out P, out gp_Vec V1);
        Assert.Equal(new gp_Vec(1, 0, 2), V1);
        L.D2(2, out P, out V1, out gp_Vec V2);
        Assert.Equal(new gp_Vec(0, 0, 0), V2);
        L.D3(2, out P, out V1, out V2, out gp_Vec V3);
        Assert.Equal(new gp_Vec(0, 0, 0), V3);
        Assert.Equal(new gp_Vec(1, 0, 2), L.DN(2, 1));

        Assert.Equal(5.0, L.TransformedParameter(5.0, gp_Trsf.Scale(new gp_Pnt(), 3)));
        Assert.Equal(1.0, L.ParametricTransformation(gp_Trsf.Scale(new gp_Pnt(), 3)));
        Assert.Throws<Standard_NoSuchObject>(() => L.Period());
    }

    [Fact]
    public void Curve_Reversed_copies_even_when_frozen()
    {
        TestLine L = new(new gp_Pnt(0, 0, 0), new gp_Vec(1, 0, 0));
        L.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => L.Mirror(new gp_Pnt()));
        Geom_Curve R = L.Reversed();
        Assert.False(R.IsFrozen());
        Assert.Equal(new gp_Pnt(-2, 0, 0), R.Value(2));
        Assert.Equal(L.Value(-2), R.Value(L.ReversedParameter(-2)));
        Assert.Equal(new gp_Pnt(2, 0, 0), L.Value(2));

        TestLine T = (TestLine)L.Translated(new gp_Vec(0, 1, 0));
        Assert.Equal(new gp_Pnt(0, 1, 0), T.Origin());
    }

    [Fact]
    public void Surface_D_methods_and_defaults()
    {
        TestSaddle S = new();
        S.D1(2, 3, out gp_Pnt P, out gp_Vec D1U, out gp_Vec D1V);
        Assert.Equal(new gp_Pnt(2, 6, 6), P);
        Assert.Equal(new gp_Vec(1, 0, 3), D1U);
        Assert.Equal(new gp_Vec(0, 2, 2), D1V);
        S.D2(2, 3, out P, out D1U, out D1V, out gp_Vec D2U, out gp_Vec D2V, out gp_Vec D2UV);
        Assert.Equal(new gp_Vec(0, 0, 1), D2UV);
        S.D3(2, 3, out P, out D1U, out D1V, out D2U, out D2V, out D2UV, out gp_Vec D3U, out _, out _, out _);
        Assert.Equal(new gp_Vec(0, 0, 0), D3U);
        Assert.Equal(new gp_Vec(0, 0, 1), S.DN(2, 3, 1, 1));

        double U = 1, V = 2;
        S.TransformParameters(ref U, ref V, gp_Trsf.Scale(new gp_Pnt(), 2));
        Assert.Equal((1.0, 2.0), (U, V));
        Assert.Equal(new gp_GTrsf2d().Form(), S.ParametricTransformation(new gp_Trsf()).Form());
        Assert.Throws<Standard_NoSuchObject>(() => S.UPeriod());
        Assert.Throws<Standard_NoSuchObject>(() => S.VPeriod());
    }

    [Fact]
    public void RepUtils_curve_mapped_scales_derivatives()
    {
        TestLine aRep = new(new gp_Pnt(0, 0, 0), new gp_Vec(1, 1, 0));
        TestLine anOwner = new(new gp_Pnt(5, 5, 5), new gp_Vec(0, 0, 1));
        GeomEval_RepCurveDesc.Mapped aDesc = new()
        {
            Representation = aRep,
            ParamMap = new GeomEval_RepCurveDesc.Map1d { Scale = 2.0, Offset = 1.0 },
            Domain = new GeomEval_RepCurveDesc.Domain1d { First = 0.0, Last = 10.0 },
        };
        GeomEval_RepUtils.ValidateCurveDesc(aDesc, anOwner);

        Assert.True(GeomEval_RepUtils.TryEvalCurveD0(aDesc, 2.0, out gp_Pnt P));
        Assert.Equal(new gp_Pnt(5, 5, 0), P);
        Assert.True(GeomEval_RepUtils.TryEvalCurveD1(aDesc, 2.0, out Geom_Curve.ResD1 R1));
        Assert.Equal(new gp_Vec(2, 2, 0), R1.D1);
        Assert.True(GeomEval_RepUtils.TryEvalCurveDN(aDesc, 2.0, 1, out gp_Vec VN));
        Assert.Equal(new gp_Vec(2, 2, 0), VN);

        // Outside the domain (u = 6 maps to 13) and beyond MaxDerivOrder, the owner must evaluate itself
        Assert.False(GeomEval_RepUtils.TryEvalCurveD0(aDesc, 6.0, out _));
        aDesc.MaxDerivOrder = 1;
        Assert.False(GeomEval_RepUtils.TryEvalCurveD2(aDesc, 2.0, out _));
        Assert.False(GeomEval_RepUtils.TryEvalCurveD0(null, 2.0, out _));

        GeomEval_RepCurveDesc.Mapped aClone = (GeomEval_RepCurveDesc.Mapped)GeomEval_RepUtils.CloneCurveDesc(aDesc)!;
        Assert.NotSame(aRep, aClone.Representation);
        Assert.Equal(2.0, aClone.ParamMap.Scale);
        Assert.Equal(10.0, aClone.Domain!.Value.Last);
        Assert.Equal(1, aClone.MaxDerivOrder);

        // Invalid descriptors
        Assert.Throws<Standard_ProgramError>(() => GeomEval_RepUtils.ValidateCurveDesc(aDesc, aRep));
        Assert.Throws<Standard_ProgramError>(() =>
            GeomEval_RepUtils.ValidateCurveDesc(new GeomEval_RepCurveDesc.Full(), anOwner));
        aDesc.ParamMap = new GeomEval_RepCurveDesc.Map1d { Scale = 0.0 };
        Assert.Throws<Standard_ProgramError>(() => GeomEval_RepUtils.ValidateCurveDesc(aDesc, anOwner));
        Assert.True(new GeomEval_RepCurveDesc.Map1d().IsIdentity());
    }

    [Fact]
    public void RepUtils_surface_mapped_with_swap()
    {
        TestSaddle aRep = new();
        GeomEval_RepSurfaceDesc.Mapped aDesc = new()
        {
            Representation = aRep,
            ParamMap = new GeomEval_RepSurfaceDesc.Map2d { ScaleU = 2.0, OffsetU = 1.0, ScaleV = 3.0, OffsetV = -1.0, SwapUV = true },
        };
        GeomEval_RepUtils.ValidateSurfaceDesc(aDesc, new TestSaddle());
        double u = 0.5, v = 0.25;

        // S'(u, v) = S(2v + 1, 3u - 1)
        double uR = 2 * v + 1, vR = 3 * u - 1;
        Assert.True(GeomEval_RepUtils.TryEvalSurfaceD0(aDesc, u, v, out gp_Pnt P));
        Assert.Equal(aRep.Value(uR, vR), P);

        Assert.True(GeomEval_RepUtils.TryEvalSurfaceD2(aDesc, u, v, out Geom_Surface.ResD2 R2));
        Geom_Surface.ResD2 aRef = aRep.EvalD2(uR, vR);
        Assert.Equal(aRef.D1V * 3.0, R2.D1U); // d/du = 3 d/dvRep
        Assert.Equal(aRef.D1U * 2.0, R2.D1V); // d/dv = 2 d/duRep
        Assert.Equal(aRef.D2UV * 6.0, R2.D2UV);

        Assert.True(GeomEval_RepUtils.TryEvalSurfaceDN(aDesc, u, v, 1, 0, out gp_Vec DN));
        Assert.Equal(aRep.DN(uR, vR, 0, 1) * 3.0, DN);
        Assert.False(GeomEval_RepUtils.TryEvalSurfaceDN(aDesc, u, v, 0, 0, out _));

        GeomEval_RepSurfaceDesc.Base aClone = GeomEval_RepUtils.CloneSurfaceDesc(aDesc)!;
        Assert.Equal(GeomEval_RepSurfaceDesc.Base.Kind.Mapped, aClone.GetKind());
        Assert.True(((GeomEval_RepSurfaceDesc.Mapped)aClone).ParamMap.SwapUV);
    }
}
