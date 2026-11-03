// Port of: src/ModelingData/TKG2d/GTests/Geom2d_OffsetCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom2d_OffsetCurve_Test
{
    private readonly OffsetCurve2D myOriginalCurve;

    public Geom2d_OffsetCurve_Test()
    {
        // Create a circle as basis curve
        Circle2D aCircle = new Circle2D(new Axis2D(new Point2D(0, 0), new Direction2D(Direction2D.D.X)), 5.0);
        CircleCurve2D aBasisCurve = new CircleCurve2D(aCircle);

        // Create offset curve
        double anOffsetValue = 2.0;
        myOriginalCurve = new OffsetCurve2D(aBasisCurve, anOffsetValue);
    }

    [Fact]
    public void CopyConstructorBasicProperties()
    {
        // Test copy constructor
        OffsetCurve2D aCopiedCurve = new OffsetCurve2D(myOriginalCurve);

        // Verify basic properties are identical
        Assert.Equal(myOriginalCurve.Offset, aCopiedCurve.Offset);
        Assert.Equal(myOriginalCurve.IsPeriodic, aCopiedCurve.IsPeriodic);
        Assert.Equal(myOriginalCurve.IsClosed, aCopiedCurve.IsClosed);
    }

    [Fact]
    public void CopyConstructorBasisCurve()
    {
        OffsetCurve2D aCopiedCurve = new OffsetCurve2D(myOriginalCurve);

        // Verify basis curves are equivalent but independent
        Curve2D anOrigBasis = myOriginalCurve.BasisCurve;
        Curve2D aCopyBasis = aCopiedCurve.BasisCurve;

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
        Geometry2D aCopiedGeom = myOriginalCurve.Copy();
        OffsetCurve2D? aCopiedCurve = aCopiedGeom as OffsetCurve2D;

        Assert.NotNull(aCopiedCurve);

        // Verify the copy is functionally identical
        Assert.Equal(myOriginalCurve.Offset, aCopiedCurve.Offset);

        // Test evaluation at several points
        double anUFirst = myOriginalCurve.FirstParameter;
        double anULast = myOriginalCurve.LastParameter;
        double aStep = (anULast - anUFirst) / 4.0;

        for (double u = anUFirst; u <= anULast; u += aStep)
        {
            Point2D anOrigPnt = myOriginalCurve.Value(u);
            Point2D aCopyPnt = aCopiedCurve.Value(u);
            Assert.True(anOrigPnt.IsEqual(aCopyPnt, 1e-10));
        }
    }

    [Fact]
    public void CopyIndependence()
    {
        OffsetCurve2D aCopiedCurve = new OffsetCurve2D(myOriginalCurve);

        // Store original offset value
        double anOrigOffset = aCopiedCurve.Offset;

        // Modify the original curve
        myOriginalCurve.SetOffsetValue(10.0);

        // Verify the copied curve is not affected
        Assert.Equal(anOrigOffset, aCopiedCurve.Offset);
        Assert.NotEqual(myOriginalCurve.Offset, aCopiedCurve.Offset);
    }
}
