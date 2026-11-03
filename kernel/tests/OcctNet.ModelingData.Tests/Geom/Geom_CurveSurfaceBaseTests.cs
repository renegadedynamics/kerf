// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Evaluation;
using Kerf.Geometry;

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
        TestLine L = new(new Point3D(1, 2, 3), new Vector3D(1, 0, 2));
        L.D0(2, out Point3D P);
        Assert.Equal(new Point3D(3, 2, 7), P);
        Assert.Equal(P, L.Value(2));
        L.D1(2, out P, out Vector3D V1);
        Assert.Equal(new Vector3D(1, 0, 2), V1);
        L.D2(2, out P, out V1, out Vector3D V2);
        Assert.Equal(new Vector3D(0, 0, 0), V2);
        L.D3(2, out P, out V1, out V2, out Vector3D V3);
        Assert.Equal(new Vector3D(0, 0, 0), V3);
        Assert.Equal(new Vector3D(1, 0, 2), L.DN(2, 1));

        Assert.Equal(5.0, L.TransformedParameter(5.0, Transform3D.Scale(new Point3D(), 3)));
        Assert.Equal(1.0, L.ParametricTransformation(Transform3D.Scale(new Point3D(), 3)));
        Assert.Throws<NoSuchObjectException>(() => L.Period());
    }

    [Fact]
    public void Curve_Reversed_copies_even_when_frozen()
    {
        TestLine L = new(new Point3D(0, 0, 0), new Vector3D(1, 0, 0));
        L.Freeze();
        Assert.Throws<ImmutableObjectException>(() => L.Mirror(new Point3D()));
        Curve3D R = L.Reversed();
        Assert.False(R.IsFrozen);
        Assert.Equal(new Point3D(-2, 0, 0), R.Value(2));
        Assert.Equal(L.Value(-2), R.Value(L.ReversedParameter(-2)));
        Assert.Equal(new Point3D(2, 0, 0), L.Value(2));

        TestLine T = (TestLine)L.Translated(new Vector3D(0, 1, 0));
        Assert.Equal(new Point3D(0, 1, 0), T.Origin());
    }

    [Fact]
    public void Surface_D_methods_and_defaults()
    {
        TestSaddle S = new();
        S.D1(2, 3, out Point3D P, out Vector3D D1U, out Vector3D D1V);
        Assert.Equal(new Point3D(2, 6, 6), P);
        Assert.Equal(new Vector3D(1, 0, 3), D1U);
        Assert.Equal(new Vector3D(0, 2, 2), D1V);
        S.D2(2, 3, out P, out D1U, out D1V, out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV);
        Assert.Equal(new Vector3D(0, 0, 1), D2UV);
        S.D3(2, 3, out P, out D1U, out D1V, out D2U, out D2V, out D2UV, out Vector3D D3U, out _, out _, out _);
        Assert.Equal(new Vector3D(0, 0, 0), D3U);
        Assert.Equal(new Vector3D(0, 0, 1), S.DN(2, 3, 1, 1));

        double U = 1, V = 2;
        S.TransformParameters(ref U, ref V, Transform3D.Scale(new Point3D(), 2));
        Assert.Equal((1.0, 2.0), (U, V));
        Assert.Equal(new GeneralTransform2D().Form(), S.ParametricTransformation(new Transform3D()).Form());
        Assert.Throws<NoSuchObjectException>(() => S.UPeriod());
        Assert.Throws<NoSuchObjectException>(() => S.VPeriod());
    }

    [Fact]
    public void RepUtils_curve_mapped_scales_derivatives()
    {
        TestLine aRep = new(new Point3D(0, 0, 0), new Vector3D(1, 1, 0));
        TestLine anOwner = new(new Point3D(5, 5, 5), new Vector3D(0, 0, 1));
        RepCurveDesc.Mapped aDesc = new()
        {
            Representation = aRep,
            ParamMap = new RepCurveDesc.Map1d { Scale = 2.0, Offset = 1.0 },
            Domain = new RepCurveDesc.Domain1d { First = 0.0, Last = 10.0 },
        };
        RepUtils.ValidateCurveDesc(aDesc, anOwner);

        Assert.True(RepUtils.TryEvalCurveD0(aDesc, 2.0, out Point3D P));
        Assert.Equal(new Point3D(5, 5, 0), P);
        Assert.True(RepUtils.TryEvalCurveD1(aDesc, 2.0, out Curve3D.ResD1 R1));
        Assert.Equal(new Vector3D(2, 2, 0), R1.D1);
        Assert.True(RepUtils.TryEvalCurveDN(aDesc, 2.0, 1, out Vector3D VN));
        Assert.Equal(new Vector3D(2, 2, 0), VN);

        // Outside the domain (u = 6 maps to 13) and beyond MaxDerivOrder, the owner must evaluate itself
        Assert.False(RepUtils.TryEvalCurveD0(aDesc, 6.0, out _));
        aDesc.MaxDerivOrder = 1;
        Assert.False(RepUtils.TryEvalCurveD2(aDesc, 2.0, out _));
        Assert.False(RepUtils.TryEvalCurveD0(null, 2.0, out _));

        RepCurveDesc.Mapped aClone = (RepCurveDesc.Mapped)RepUtils.CloneCurveDesc(aDesc)!;
        Assert.NotSame(aRep, aClone.Representation);
        Assert.Equal(2.0, aClone.ParamMap.Scale);
        Assert.Equal(10.0, aClone.Domain!.Value.Last);
        Assert.Equal(1, aClone.MaxDerivOrder);

        // Invalid descriptors
        Assert.Throws<ProgramErrorException>(() => RepUtils.ValidateCurveDesc(aDesc, aRep));
        Assert.Throws<ProgramErrorException>(() =>
            RepUtils.ValidateCurveDesc(new RepCurveDesc.Full(), anOwner));
        aDesc.ParamMap = new RepCurveDesc.Map1d { Scale = 0.0 };
        Assert.Throws<ProgramErrorException>(() => RepUtils.ValidateCurveDesc(aDesc, anOwner));
        Assert.True(new RepCurveDesc.Map1d().IsIdentity);
    }

    [Fact]
    public void RepUtils_surface_mapped_with_swap()
    {
        TestSaddle aRep = new();
        RepSurfaceDesc.Mapped aDesc = new()
        {
            Representation = aRep,
            ParamMap = new RepSurfaceDesc.Map2d { ScaleU = 2.0, OffsetU = 1.0, ScaleV = 3.0, OffsetV = -1.0, SwapUV = true },
        };
        RepUtils.ValidateSurfaceDesc(aDesc, new TestSaddle());
        double u = 0.5, v = 0.25;

        // S'(u, v) = S(2v + 1, 3u - 1)
        double uR = 2 * v + 1, vR = 3 * u - 1;
        Assert.True(RepUtils.TryEvalSurfaceD0(aDesc, u, v, out Point3D P));
        Assert.Equal(aRep.Value(uR, vR), P);

        Assert.True(RepUtils.TryEvalSurfaceD2(aDesc, u, v, out Surface.ResD2 R2));
        Surface.ResD2 aRef = aRep.EvalD2(uR, vR);
        Assert.Equal(aRef.D1V * 3.0, R2.D1U); // d/du = 3 d/dvRep
        Assert.Equal(aRef.D1U * 2.0, R2.D1V); // d/dv = 2 d/duRep
        Assert.Equal(aRef.D2UV * 6.0, R2.D2UV);

        Assert.True(RepUtils.TryEvalSurfaceDN(aDesc, u, v, 1, 0, out Vector3D DN));
        Assert.Equal(aRep.DN(uR, vR, 0, 1) * 3.0, DN);
        Assert.False(RepUtils.TryEvalSurfaceDN(aDesc, u, v, 0, 0, out _));

        RepSurfaceDesc.Base aClone = RepUtils.CloneSurfaceDesc(aDesc)!;
        Assert.Equal(RepSurfaceDesc.Base.Kind.Mapped, aClone.GetKind());
        Assert.True(((RepSurfaceDesc.Mapped)aClone).ParamMap.SwapUV);
    }
}
