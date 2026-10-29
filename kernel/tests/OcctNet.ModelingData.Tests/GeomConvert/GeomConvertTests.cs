// Port of: src/ModelingData/TKGeomBase/GTests/GeomConvert_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class GeomConvertTest
{
    [Fact]
    public void CircleToBSpline()
    {
        Geom_Circle aCircle = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 5.0);
        Geom_BSplineCurve aBSpline = GeomConvert.CurveToBSplineCurve(aCircle);
        Assert.NotNull(aBSpline);

        aCircle.D0(0.0, out gp_Pnt aOrigPnt);
        aBSpline.D0(0.0, out gp_Pnt aBSplinePnt);
        Assert.Equal(0.0, aOrigPnt.Distance(aBSplinePnt), Precision.Confusion());
    }

    [Fact]
    public void LineToBSpline()
    {
        Geom_Line aLine = new(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        Geom_TrimmedCurve aTrimmed = new(aLine, 0.0, 10.0);
        Geom_BSplineCurve aBSpline = GeomConvert.CurveToBSplineCurve(aTrimmed);
        Assert.NotNull(aBSpline);

        aBSpline.D0(5.0, out gp_Pnt aPnt);
        Assert.Equal(0.0, aPnt.Distance(new gp_Pnt(5, 0, 0)), Precision.Confusion());
    }

    [Fact]
    public void PlaneToBSplineSurface()
    {
        Geom_Plane aPlane = new(new gp_Ax3(gp.XOY()));
        Geom_RectangularTrimmedSurface aTrimmed = new(aPlane, 0.0, 10.0, 0.0, 10.0);
        Geom_BSplineSurface aBSurf = GeomConvert.SurfaceToBSplineSurface(aTrimmed);
        Assert.NotNull(aBSurf);

        aBSurf.D0(5.0, 5.0, out gp_Pnt aPnt);
        Assert.Equal(0.0, aPnt.Distance(new gp_Pnt(5, 5, 0)), Precision.Confusion());
    }
}
