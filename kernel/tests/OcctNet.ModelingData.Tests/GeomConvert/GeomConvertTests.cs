// Port of: src/ModelingData/TKGeomBase/GTests/GeomConvert_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

public class GeomConvertTest
{
    [Fact]
    public void CircleToBSpline()
    {
        CircleCurve3D aCircle = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5.0);
        BSplineCurve3D aBSpline = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aCircle);
        Assert.NotNull(aBSpline);

        aCircle.D0(0.0, out Point3D aOrigPnt);
        aBSpline.D0(0.0, out Point3D aBSplinePnt);
        Assert.Equal(0.0, aOrigPnt.Distance(aBSplinePnt), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void LineToBSpline()
    {
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        TrimmedCurve3D aTrimmed = new(aLine, 0.0, 10.0);
        BSplineCurve3D aBSpline = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aTrimmed);
        Assert.NotNull(aBSpline);

        aBSpline.D0(5.0, out Point3D aPnt);
        Assert.Equal(0.0, aPnt.Distance(new Point3D(5, 0, 0)), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void PlaneToBSplineSurface()
    {
        PlaneSurface aPlane = new(new CoordinateSystem(Constants.XOY()));
        TrimmedSurface aTrimmed = new(aPlane, 0.0, 10.0, 0.0, 10.0);
        BSplineSurface aBSurf = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(aTrimmed);
        Assert.NotNull(aBSurf);

        aBSurf.D0(5.0, 5.0, out Point3D aPnt);
        Assert.Equal(0.0, aPnt.Distance(new Point3D(5, 5, 0)), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
