// Port of: src/ModelingData/TKG2d/GTests/Geom2d_OffsetCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Geom2d_OffsetCurve_Test
{
    private readonly Geom2d_OffsetCurve myOriginalCurve;

    public Geom2d_OffsetCurve_Test()
    {
        // Create a circle as basis curve
        gp_Circ2d aCircle = new gp_Circ2d(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(gp_Dir2d.D.X)), 5.0);
        Geom2d_Circle aBasisCurve = new Geom2d_Circle(aCircle);

        // Create offset curve
        double anOffsetValue = 2.0;
        myOriginalCurve = new Geom2d_OffsetCurve(aBasisCurve, anOffsetValue);
    }

    [Fact]
    public void CopyConstructorBasicProperties()
    {
        // Test copy constructor
        Geom2d_OffsetCurve aCopiedCurve = new Geom2d_OffsetCurve(myOriginalCurve);

        // Verify basic properties are identical
        Assert.Equal(myOriginalCurve.Offset(), aCopiedCurve.Offset());
        Assert.Equal(myOriginalCurve.IsPeriodic(), aCopiedCurve.IsPeriodic());
        Assert.Equal(myOriginalCurve.IsClosed(), aCopiedCurve.IsClosed());
    }

    [Fact]
    public void CopyConstructorBasisCurve()
    {
        Geom2d_OffsetCurve aCopiedCurve = new Geom2d_OffsetCurve(myOriginalCurve);

        // Verify basis curves are equivalent but independent
        Geom2d_Curve anOrigBasis = myOriginalCurve.BasisCurve();
        Geom2d_Curve aCopyBasis = aCopiedCurve.BasisCurve();

        // They should be different objects
        Assert.NotSame(anOrigBasis, aCopyBasis);

        // But functionally equivalent
        Assert.Equal(anOrigBasis.FirstParameter(), aCopyBasis.FirstParameter());
        Assert.Equal(anOrigBasis.LastParameter(), aCopyBasis.LastParameter());
    }

    [Fact]
    public void CopyMethodUsesOptimizedConstructor()
    {
        // Test that Copy() method uses the optimized copy constructor
        Geom2d_Geometry aCopiedGeom = myOriginalCurve.Copy();
        Geom2d_OffsetCurve? aCopiedCurve = aCopiedGeom as Geom2d_OffsetCurve;

        Assert.NotNull(aCopiedCurve);

        // Verify the copy is functionally identical
        Assert.Equal(myOriginalCurve.Offset(), aCopiedCurve.Offset());

        // Test evaluation at several points
        double anUFirst = myOriginalCurve.FirstParameter();
        double anULast = myOriginalCurve.LastParameter();
        double aStep = (anULast - anUFirst) / 4.0;

        for (double u = anUFirst; u <= anULast; u += aStep)
        {
            gp_Pnt2d anOrigPnt = myOriginalCurve.Value(u);
            gp_Pnt2d aCopyPnt = aCopiedCurve.Value(u);
            Assert.True(anOrigPnt.IsEqual(aCopyPnt, 1e-10));
        }
    }

    [Fact]
    public void CopyIndependence()
    {
        Geom2d_OffsetCurve aCopiedCurve = new Geom2d_OffsetCurve(myOriginalCurve);

        // Store original offset value
        double anOrigOffset = aCopiedCurve.Offset();

        // Modify the original curve
        myOriginalCurve.SetOffsetValue(10.0);

        // Verify the copied curve is not affected
        Assert.Equal(anOrigOffset, aCopiedCurve.Offset());
        Assert.NotEqual(myOriginalCurve.Offset(), aCopiedCurve.Offset());
    }
}
