// Port of: src/ModelingData/TKG3d/GTests/Geom_OffsetCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom_OffsetCurve_Test
{
    private readonly OffsetCurve3D myOriginalCurve;

    public Geom_OffsetCurve_Test()
    {
        // Create a circle as basis curve
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 5.0);
        CircleCurve3D aBasisCurve = new(aCircle);

        // Create offset curve
        Direction3D anOffsetDir = new(Direction3D.D.Z);
        double anOffsetValue = 2.0;
        myOriginalCurve = new OffsetCurve3D(aBasisCurve, anOffsetValue, anOffsetDir);
    }

    [Fact]
    public void CopyConstructorBasicProperties()
    {
        // Test copy constructor
        OffsetCurve3D aCopiedCurve = new(myOriginalCurve);

        // Verify basic properties are identical
        Assert.Equal(myOriginalCurve.Offset, aCopiedCurve.Offset);
        Assert.True(myOriginalCurve.Direction.IsEqual(aCopiedCurve.Direction, 1e-10));
        Assert.Equal(myOriginalCurve.IsPeriodic, aCopiedCurve.IsPeriodic);
        Assert.Equal(myOriginalCurve.IsClosed, aCopiedCurve.IsClosed);
    }

    [Fact]
    public void CopyConstructorBasisCurve()
    {
        OffsetCurve3D aCopiedCurve = new(myOriginalCurve);

        // Verify basis curves are equivalent but independent
        Curve3D anOrigBasis = myOriginalCurve.BasisCurve;
        Curve3D aCopyBasis = aCopiedCurve.BasisCurve;

        // They should be different objects
        Assert.NotSame(anOrigBasis, aCopyBasis);

        // But functionally equivalent
        Assert.Equal(anOrigBasis.FirstParameter, aCopyBasis.FirstParameter);
        Assert.Equal(anOrigBasis.LastParameter, aCopyBasis.LastParameter);
    }

    [Fact]
    public void CopyMethodUsesOptimizedConstructor()
    {
        // Test that Copy() method uses the optimized copy constructor
        OffsetCurve3D? aCopiedCurve = myOriginalCurve.Copy() as OffsetCurve3D;
        Assert.NotNull(aCopiedCurve);

        // Verify the copy is functionally identical
        Assert.Equal(myOriginalCurve.Offset, aCopiedCurve.Offset);

        // Test evaluation at several points
        double anUFirst = myOriginalCurve.FirstParameter;
        double anULast = myOriginalCurve.LastParameter;
        double aStep = (anULast - anUFirst) / 4.0;

        for (double u = anUFirst; u <= anULast; u += aStep)
        {
            Point3D anOrigPnt = myOriginalCurve.Value(u);
            Point3D aCopyPnt = aCopiedCurve.Value(u);
            Assert.True(anOrigPnt.IsEqual(aCopyPnt, 1e-10));
        }
    }

    [Fact]
    public void CopyIndependence()
    {
        OffsetCurve3D aCopiedCurve = new(myOriginalCurve);

        // Store original offset value
        double anOrigOffset = aCopiedCurve.Offset;

        // Modify the original curve
        myOriginalCurve.SetOffsetValue(10.0);

        // Verify the copied curve is not affected
        Assert.Equal(anOrigOffset, aCopiedCurve.Offset);
        Assert.NotEqual(myOriginalCurve.Offset, aCopiedCurve.Offset);
    }
}
