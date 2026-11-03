// Port of: src/ModelingData/TKGeomBase/GTests/GCPnts_AbscissaPoint_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

// GeomAdaptor_Curve is not ported yet (unit 2A): TestGeomAdaptor_Curve stands in for it.
public class GCPnts_AbscissaPointTest
{
    [Fact]
    public void LineLength()
    {
        Geom_Line aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        TestGeomAdaptor_Curve anAdaptor = new(aLine, 0.0, 10.0);

        double aLength = GCPnts_AbscissaPoint.Length(anAdaptor);

        Assert.Equal(10.0, aLength, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void CircleArcLength()
    {
        Geom_Circle aCircle = new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 1.0);
        TestGeomAdaptor_Curve anAdaptor = new(aCircle, 0.0, M_PI);

        double aLength = GCPnts_AbscissaPoint.Length(anAdaptor);

        Assert.Equal(M_PI, aLength, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FullCircleLength()
    {
        Geom_Circle aCircle = new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 5.0);
        TestGeomAdaptor_Curve anAdaptor = new(aCircle, 0.0, 2.0 * M_PI);

        double aLength = GCPnts_AbscissaPoint.Length(anAdaptor);

        Assert.Equal(2.0 * M_PI * 5.0, aLength, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ParameterAtAbscissa_Line()
    {
        Geom_Line aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        TestGeomAdaptor_Curve anAdaptor = new(aLine, 0.0, 10.0);

        GCPnts_AbscissaPoint anAbscissa = new(anAdaptor, 5.0, 0.0);

        Assert.True(anAbscissa.IsDone());
        double aParam = anAbscissa.Parameter();
        Assert.Equal(5.0, aParam, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ParameterAtAbscissa_Circle()
    {
        Geom_Circle aCircle = new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 1.0);
        TestGeomAdaptor_Curve anAdaptor = new(aCircle, 0.0, 2.0 * M_PI);

        GCPnts_AbscissaPoint anAbscissa = new(anAdaptor, M_PI / 2.0, 0.0);

        Assert.True(anAbscissa.IsDone());
        double aParam = anAbscissa.Parameter();
        Assert.Equal(M_PI / 2.0, aParam, global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
