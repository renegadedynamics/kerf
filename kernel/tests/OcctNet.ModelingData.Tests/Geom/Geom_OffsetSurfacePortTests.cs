// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own tests of Geom_OffsetSurface: freezing of the shared basis, the twins, the osculating surfaces at
/// degenerate iso-curves, the trimmed offset of Geom_RectangularTrimmedSurface, and the OCCT behaviours listed in
/// occt-issues.md.
/// </summary>
public class Geom_OffsetSurfacePortTests
{
    private static Geom_CylindricalSurface Cylinder(double theR = 2.0) =>
        new(new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)), theR);

    /// <summary>A bicubic B-spline dome whose iso-curve V = 0 collapses to the apex (0, 0, 2).</summary>
    private static Geom_BSplineSurface Dome()
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
        return new Geom_BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
    }

    [Fact]
    public void BasisSurface_Is_Frozen_Shared_And_Copied_On_Write()
    {
        Geom_BSplineSurface aDome = Dome();
        Geom_OffsetSurface anOff = new(aDome, 0.5);
        Assert.False(aDome.IsFrozen); // the constructor copies the caller's surface
        Geom_Surface aBasis = anOff.BasisSurface();
        Assert.True(aBasis.IsFrozen);
        Assert.Same(aBasis, anOff.BasisSurface());
        Assert.Throws<ImmutableObjectException>(() => aBasis.UReverse());

        Point3D aBefore = aBasis.Value(0.3, 0.6);
        Point3D anExpected = anOff.Value(0.3, 0.6);
        double aU = anOff.UReversedParameter(0.3);
        anOff.UReverse();
        Assert.NotSame(aBasis, anOff.BasisSurface());
        Assert.True(aBasis.Value(0.3, 0.6).IsEqual(aBefore, 0.0));
        Assert.True(anOff.Value(aU, 0.6).IsEqual(anExpected, 1e-12));
        Assert.Equal(-0.5, anOff.Offset());

        Geom_Surface aBasis2 = anOff.BasisSurface();
        anOff.Transform(Transform3D.Translation(new Vector3D(1, 2, 3)));
        Assert.NotSame(aBasis2, anOff.BasisSurface());
        Assert.True(anOff.Value(aU, 0.6).IsEqual(anExpected.Translated(new Vector3D(1, 2, 3)), 1e-12));
    }

    [Fact]
    public void Frozen_Offset_Refuses_Mutators_But_Gives_Changed_Copies()
    {
        Geom_OffsetSurface anOff = new(Cylinder(), 1.0);
        anOff.Freeze();
        Assert.Throws<ImmutableObjectException>(() => anOff.SetOffsetValue(2.0));
        Assert.Throws<ImmutableObjectException>(() => anOff.SetBasisSurface(Cylinder()));
        Assert.Throws<ImmutableObjectException>(() => anOff.UReverse());
        Assert.Throws<ImmutableObjectException>(() => anOff.Transform(new Transform3D()));
        Assert.Throws<ImmutableObjectException>(() => anOff.ClearEvalRepresentation());

        Geom_OffsetSurface aCopy = anOff.WithOffsetValue(2.0);
        Assert.False(aCopy.IsFrozen);
        Assert.Equal(2.0, aCopy.Offset());
        Assert.Equal(1.0, anOff.Offset());
        Assert.True(aCopy.HasEvalRepresentation()); // the equivalent cylinder of radius 4
        Assert.Equal(4.0, ((Geom_CylindricalSurface)aCopy.Surface()!).Radius());

        Geom_OffsetSurface aNoRep = anOff.WithoutEvalRepresentation();
        Assert.False(aNoRep.HasEvalRepresentation());
        Assert.True(aNoRep.Value(0.7, 1.5).IsEqual(anOff.Value(0.7, 1.5), 1e-12)); // general evaluation agrees

        Geom_OffsetSurface aRebased = anOff.WithBasisSurface(Dome());
        Assert.IsType<Geom_BSplineSurface>(aRebased.BasisSurface());
        Assert.Null(aRebased.Surface());
    }

    [Fact]
    public void ZeroOffset_Surface_Is_The_Frozen_Basis()
    {
        Geom_OffsetSurface anOff = new(Cylinder(), 0.0);
        Geom_Surface? anEquiv = anOff.Surface();
        Assert.Same(anOff.BasisSurface(), anEquiv);
        Assert.True(anEquiv!.IsFrozen);
    }

    [Fact]
    public void Nested_Offsets_And_Trimmed_Bases_Are_Flattened()
    {
        Geom_RectangularTrimmedSurface aTrim = new(Cylinder(), 0.5, 2.0, -1.0, 3.0);
        Geom_OffsetSurface anInner = new(aTrim, 0.5);
        Geom_OffsetSurface anOuter = new(anInner, 0.25);
        Assert.Equal(0.75, anOuter.Offset());
        Geom_RectangularTrimmedSurface aBasis = Assert.IsType<Geom_RectangularTrimmedSurface>(anOuter.BasisSurface());
        Assert.IsType<Geom_CylindricalSurface>(aBasis.BasisSurface());
        anOuter.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal((0.5, 2.0, -1.0, 3.0), (aU1, aU2, aV1, aV2));
        Assert.Equal(2.75, ((Geom_CylindricalSurface)((Geom_RectangularTrimmedSurface)anOuter.Surface()!).BasisSurface()).Radius());
    }

    [Fact]
    public void Trimming_An_Offset_Gives_The_Offset_Of_A_Trimmed_Basis()
    {
        // The former TODO(2A) of Geom_RectangularTrimmedSurface: an offset basis becomes an offset of a trimmed
        // surface, in both constructors.
        Geom_OffsetSurface anOff = new(Dome(), 0.3);
        Geom_RectangularTrimmedSurface aBoth = new(anOff, 0.1, 0.9, 0.2, 0.8);
        Geom_OffsetSurface aB1 = Assert.IsType<Geom_OffsetSurface>(aBoth.BasisSurface());
        Assert.IsType<Geom_RectangularTrimmedSurface>(aB1.BasisSurface());
        Assert.Equal(0.3, aB1.Offset());
        Assert.True(aBoth.Value(0.5, 0.5).IsEqual(anOff.Value(0.5, 0.5), 1e-12));

        Geom_RectangularTrimmedSurface aU = new(anOff, 0.1, 0.9, true);
        Geom_OffsetSurface aB2 = Assert.IsType<Geom_OffsetSurface>(aU.BasisSurface());
        aB2.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal((0.1, 0.9, 0.0, 1.0), (aU1, aU2, aV1, aV2));
        Assert.True(aU.Value(0.4, 0.7).IsEqual(anOff.Value(0.4, 0.7), 1e-12));
    }

    [Fact]
    public void Collapsed_Iso_Curve_Has_An_Osculating_Surface()
    {
        Geom_OffsetSurface anOff = new(Dome(), 0.4);
        bool isOpposite = false;
        Geom_BSplineSurface? anOsc = null;
        Assert.True(anOff.UOsculatingSurface(0.5, 0.0, ref isOpposite, ref anOsc));
        Assert.NotNull(anOsc);
        Assert.True(anOsc!.IsFrozen);
        Assert.False(isOpposite);
        bool isOppositeV = false;
        Geom_BSplineSurface? anOscV = null;
        Assert.False(anOff.VOsculatingSurface(0.5, 0.0, ref isOppositeV, ref anOscV));
        Assert.Null(anOscV);

        // At the apex the offset point is the limit of the regular ones along the iso-curve U = 0.5 (the dome is
        // cone-like there, so the limit depends on U)
        Point3D anApex = anOff.Value(0.5, 0.0);
        Assert.True(anApex.IsEqual(anOff.Value(0.5, 1e-7), 1e-5));
        Assert.False(anApex.IsEqual(new Point3D(0, 0, 2), 0.39));
        anOff.D1(0.5, 0.0, out Point3D aP, out Vector3D aD1U, out Vector3D aD1V);
        Assert.True(aP.IsEqual(anApex, 1e-12));
        Assert.True(aD1V.Magnitude() > 0.1);

        // A copy shares the (frozen) osculating surfaces
        var aCopy = (Geom_OffsetSurface)anOff.Copy();
        Geom_BSplineSurface? aCopyOsc = null;
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
        Geom_BSplineSurface aCorner = new(aPoles, aUKnots, aVKnots, aUMults, aVMults, 1, 1); // a sharp corner at U = 1
        Assert.Throws<ConstructionErrorException>(() => new Geom_OffsetSurface(aCorner, 0.1));
        Geom_OffsetSurface anOff = new(aCorner, 0.1, true);
        Assert.Equal(Continuity.C0, anOff.GetBasisSurfContinuity());
    }

    [Fact]
    public void TransformParameters_Transforms_Twice_With_An_Equivalent_Surface()
    {
        // OCCT (occt-issues.md): the parameters are transformed by the basis surface, then again by the
        // equivalent surface. For a plane and a scale of 2, U and V are scaled by 4; without the equivalent
        // surface (after a reversal drops it), by 2.
        Geom_Plane aPlane = new(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1)));
        Geom_OffsetSurface anOff = new(aPlane, 1.0);
        Transform3D aScale = new();
        aScale.SetScale(new Point3D(), 2.0);
        double U = 1.5, V = -0.5;
        anOff.TransformParameters(ref U, ref V, aScale);
        Assert.Equal((6.0, -2.0), (U, V));

        var aRev = (Geom_OffsetSurface)anOff.UReversed().UReversed();
        Assert.False(aRev.HasEvalRepresentation());
        U = 1.5;
        V = -0.5;
        aRev.TransformParameters(ref U, ref V, aScale);
        Assert.Equal((3.0, -1.0), (U, V));
    }
}
