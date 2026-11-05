// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own tests of Geom_OffsetSurface: freezing of the shared basis, the twins, the osculating surfaces at
/// degenerate iso-curves, the trimmed offset of Geom_RectangularTrimmedSurface, and the OCCT behaviours listed in
/// occt-issues.md.
/// </summary>
public class Geom_OffsetSurfacePortTests
{
    private static CylindricalSurface Cylinder(double theR = 2.0) =>
        new(new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)), theR);

    /// <summary>A bicubic B-spline dome whose iso-curve V = 0 collapses to the apex (0, 0, 2).</summary>
    private static BSplineSurface Dome()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            double anAng = 0.3 + 0.7 * (i - 1);
            for (int j = 1; j <= 4; j++)
            {
                double aR = (j - 1) * 1.5;
                aPoles[i, j] = j == 1 ? new Point3D(0, 0, 2) : new Point3D(aR * Math.Cos(anAng), aR * Math.Sin(anAng), 2 - 0.6 * (j - 1));
            }
        }
        var aKnots = NCollection_Array1<double>.From([0.0, 1.0]);
        var aMults = NCollection_Array1<int>.From([4, 4]);
        return new BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
    }

    [Fact]
    public void BasisSurface_Is_Frozen_Shared_And_Copied_On_Write()
    {
        BSplineSurface aDome = Dome();
        OffsetSurface anOff = new(aDome, 0.5);
        Assert.False(aDome.IsFrozen); // the constructor copies the caller's surface
        Surface aBasis = anOff.BasisSurface;
        Assert.True(aBasis.IsFrozen);
        Assert.Same(aBasis, anOff.BasisSurface);
        EngineAssert.Throws<ImmutableObjectException>(() => aBasis.UReverse());

        Point3D aBefore = aBasis.Value(0.3, 0.6);
        Point3D anExpected = anOff.Value(0.3, 0.6);
        double aU = anOff.UReversedParameter(0.3);
        anOff.UReverse();
        Assert.NotSame(aBasis, anOff.BasisSurface);
        Assert.True(aBasis.Value(0.3, 0.6).IsEqual(aBefore, 0.0));
        Assert.True(anOff.Value(aU, 0.6).IsEqual(anExpected, 1e-12));
        Assert.Equal(-0.5, anOff.Offset);

        Surface aBasis2 = anOff.BasisSurface;
        anOff.Transform(Transform3D.Translation(new Vector3D(1, 2, 3)));
        Assert.NotSame(aBasis2, anOff.BasisSurface);
        Assert.True(anOff.Value(aU, 0.6).IsEqual(anExpected.Translated(new Vector3D(1, 2, 3)), 1e-12));
    }

    [Fact]
    public void Frozen_Offset_Refuses_Mutators_But_Gives_Changed_Copies()
    {
        OffsetSurface anOff = new(Cylinder(), 1.0);
        anOff.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => anOff.SetOffsetValue(2.0));
        EngineAssert.Throws<ImmutableObjectException>(() => anOff.SetBasisSurface(Cylinder()));
        EngineAssert.Throws<ImmutableObjectException>(() => anOff.UReverse());
        EngineAssert.Throws<ImmutableObjectException>(() => anOff.Transform(new Transform3D()));
        EngineAssert.Throws<ImmutableObjectException>(() => anOff.ClearEvalRepresentation());

        OffsetSurface aCopy = anOff.WithOffsetValue(2.0);
        Assert.False(aCopy.IsFrozen);
        Assert.Equal(2.0, aCopy.Offset);
        Assert.Equal(1.0, anOff.Offset);
        Assert.True(aCopy.HasEvalRepresentation); // the equivalent cylinder of radius 4
        Assert.Equal(4.0, ((CylindricalSurface)aCopy.Surface()!).Radius);

        OffsetSurface aNoRep = anOff.WithoutEvalRepresentation();
        Assert.False(aNoRep.HasEvalRepresentation);
        Assert.True(aNoRep.Value(0.7, 1.5).IsEqual(anOff.Value(0.7, 1.5), 1e-12)); // general evaluation agrees

        OffsetSurface aRebased = anOff.WithBasisSurface(Dome());
        Assert.IsType<BSplineSurface>(aRebased.BasisSurface);
        Assert.Null(aRebased.Surface());
    }

    [Fact]
    public void ZeroOffset_Surface_Is_The_Frozen_Basis()
    {
        OffsetSurface anOff = new(Cylinder(), 0.0);
        Surface? anEquiv = anOff.Surface();
        Assert.Same(anOff.BasisSurface, anEquiv);
        Assert.True(anEquiv!.IsFrozen);
    }

    [Fact]
    public void Nested_Offsets_And_Trimmed_Bases_Are_Flattened()
    {
        TrimmedSurface aTrim = new(Cylinder(), 0.5, 2.0, -1.0, 3.0);
        OffsetSurface anInner = new(aTrim, 0.5);
        OffsetSurface anOuter = new(anInner, 0.25);
        Assert.Equal(0.75, anOuter.Offset);
        TrimmedSurface aBasis = Assert.IsType<TrimmedSurface>(anOuter.BasisSurface);
        Assert.IsType<CylindricalSurface>(aBasis.BasisSurface);
        anOuter.ParameterBounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal((0.5, 2.0, -1.0, 3.0), (aU1, aU2, aV1, aV2));
        Assert.Equal(2.75, ((CylindricalSurface)((TrimmedSurface)anOuter.Surface()!).BasisSurface).Radius);
    }

    [Fact]
    public void Trimming_An_Offset_Gives_The_Offset_Of_A_Trimmed_Basis()
    {
        // The former TODO(2A) of Geom_RectangularTrimmedSurface: an offset basis becomes an offset of a trimmed
        // surface, in both constructors.
        OffsetSurface anOff = new(Dome(), 0.3);
        TrimmedSurface aBoth = new(anOff, 0.1, 0.9, 0.2, 0.8);
        OffsetSurface aB1 = Assert.IsType<OffsetSurface>(aBoth.BasisSurface);
        Assert.IsType<TrimmedSurface>(aB1.BasisSurface);
        Assert.Equal(0.3, aB1.Offset);
        Assert.True(aBoth.Value(0.5, 0.5).IsEqual(anOff.Value(0.5, 0.5), 1e-12));

        TrimmedSurface aU = new(anOff, 0.1, 0.9, true);
        OffsetSurface aB2 = Assert.IsType<OffsetSurface>(aU.BasisSurface);
        aB2.ParameterBounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal((0.1, 0.9, 0.0, 1.0), (aU1, aU2, aV1, aV2));
        Assert.True(aU.Value(0.4, 0.7).IsEqual(anOff.Value(0.4, 0.7), 1e-12));
    }

    [Fact]
    public void Collapsed_Iso_Curve_Has_An_Osculating_Surface()
    {
        OffsetSurface anOff = new(Dome(), 0.4);
        bool isOpposite = false;
        BSplineSurface? anOsc = null;
        Assert.True(anOff.UOsculatingSurface(0.5, 0.0, ref isOpposite, ref anOsc));
        Assert.NotNull(anOsc);
        Assert.True(anOsc!.IsFrozen);
        Assert.False(isOpposite);
        bool isOppositeV = false;
        BSplineSurface? anOscV = null;
        Assert.False(anOff.VOsculatingSurface(0.5, 0.0, ref isOppositeV, ref anOscV));
        Assert.Null(anOscV);

        // At the apex the offset point is the limit of the regular ones along the iso-curve U = 0.5 (the dome is
        // cone-like there, so the limit depends on U)
        Point3D anApex = anOff.Value(0.5, 0.0);
        Assert.True(anApex.IsEqual(anOff.Value(0.5, 1e-7), 1e-5));
        Assert.False(anApex.IsEqual(new Point3D(0, 0, 2), 0.39));
        anOff.D1(0.5, 0.0, out Point3D aP, out Vector3D aD1U, out Vector3D aD1V);
        Assert.True(aP.IsEqual(anApex, 1e-12));
        Assert.True(aD1V.Magnitude > 0.1);

        // A copy shares the (frozen) osculating surfaces
        var aCopy = (OffsetSurface)anOff.Copy();
        BSplineSurface? aCopyOsc = null;
        Assert.True(aCopy.UOsculatingSurface(0.5, 0.0, ref isOpposite, ref aCopyOsc));
        Assert.Same(anOsc, aCopyOsc);
    }

    [Fact]
    public void C0_Basis_Is_Rejected_Unless_Not_Checked()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 3, 1, 2);
        aPoles[1, 1] = new Point3D(0, 0, 0);
        aPoles[2, 1] = new Point3D(1, 0, 0);
        aPoles[3, 1] = new Point3D(1, 1, 0);
        aPoles[1, 2] = new Point3D(0, 0, 1);
        aPoles[2, 2] = new Point3D(1, 0, 1);
        aPoles[3, 2] = new Point3D(1, 1, 1);
        var aUKnots = NCollection_Array1<double>.From([0.0, 1.0, 2.0]);
        var aUMults = NCollection_Array1<int>.From([2, 1, 2]);
        var aVKnots = NCollection_Array1<double>.From([0.0, 1.0]);
        var aVMults = NCollection_Array1<int>.From([2, 2]);
        BSplineSurface aCorner = new(aPoles, aUKnots, aVKnots, aUMults, aVMults, 1, 1); // a sharp corner at U = 1
        EngineAssert.Throws<ConstructionErrorException>(() => new OffsetSurface(aCorner, 0.1));
        OffsetSurface anOff = new(aCorner, 0.1, true);
        Assert.Equal(Continuity.C0, anOff.GetBasisSurfContinuity());
    }

    [Fact]
    public void TransformParameters_Are_The_Parameters_Of_The_Transformed_Point()
    {
        // occt-issues.md, Geom_OffsetSurface::TransformParameters (Kerf decision 1489): OCCT transformed the parameters
        // by the basis surface, then again by the equivalent surface (U and V scaled by 4 for a plane and a scale of
        // 2). They are the basis's: the transformed surface's point at them is the transformed point, with or without
        // the equivalent surface (a reversal drops it), as ParametricTransformation says.
        Transform3D aScale = new Transform3D().WithScale(new Point3D(1, -2, 0.5), 2.0);
        Surface[] aBases =
        [
            new PlaneSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1))),
            new CylindricalSurface(new CoordinateSystem(new Point3D(0, 1, 0), new Direction3D(1, 1, 1)), 3.0),
            new SphericalSurface(new CoordinateSystem(new Point3D(2, 0, 0), new Direction3D(0, 1, 0)), 2.0),
        ];
        foreach (Surface aBase in aBases)
        {
            OffsetSurface anOff = new(aBase, 1.0);
            foreach (OffsetSurface aSurface in new[] { anOff, (OffsetSurface)anOff.UReversed().UReversed() })
            {
                var aMoved = (OffsetSurface)aSurface.Transformed(aScale);
                double U = 1.5, V = -0.5;
                Point3D anExpected = aSurface.Value(U, V).Transformed(aScale);
                aSurface.TransformParameters(ref U, ref V, aScale);
                Assert.True(anExpected.DistanceTo(aMoved.Value(U, V)) < 1e-12, $"{aBase.GetType().Name}: {U}, {V}");
                var aUV = new global::Kerf.Engine.Geometry.Xy(1.5, -0.5);
                aSurface.ParametricTransformation(aScale).Transforms(ref aUV);
                Assert.Equal((aUV.X(), aUV.Y()), (U, V));
            }
        }
    }
}
