// Port of: src/ModelingData/TKGeomBase/GTests/GeomLib_CheckCurveOnSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class GeomLib_CheckCurveOnSurfaceTest
{
    // Circle of radius theRadius at height theHeight on a cylinder of radius
    // theCylRadius around OZ; the pcurve is the corresponding iso line in UV.
    private static GeomLib_CheckCurveOnSurface makeCircleOnCylinderCheck(double theCircRadius, double theCylRadius,
                                                                         double theHeight)
    {
        double aFirst = 0.0, aLast = 2.0 * Math.PI;

        Geom_Circle aCirc = new(
            new gp_Ax2(new gp_Pnt(0.0, 0.0, theHeight), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(1.0, 0.0, 0.0)),
            theCircRadius);
        GeomAdaptor_Curve aC3d = new(aCirc, aFirst, aLast);

        Geom_CylindricalSurface aCyl = new(
            new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(1.0, 0.0, 0.0)),
            theCylRadius);
        Geom2d_Line aPLine = new(new gp_Pnt2d(0.0, theHeight), new gp_Dir2d(1.0, 0.0));

        Geom2dAdaptor_Curve aC2d = new(aPLine, aFirst, aLast);
        GeomAdaptor_Surface aSurf = new(aCyl);
        Adaptor3d_CurveOnSurface aCoS = new(aC2d, aSurf);

        GeomLib_CheckCurveOnSurface aCheck = new();
        aCheck.Init(aC3d, Precision.PConfusion());
        aCheck.Perform(aCoS);
        return aCheck;
    }

    private static GeomLib_CheckCurveOnSurface makeLineOnPlaneCheck(gp_Dir theCurveDirection,
                                                                    gp_Dir2d thePCurveDirection,
                                                                    double theFirst, double theLast)
    {
        Geom_Line aLine = new(new gp_Pnt(0.0, 0.0, 0.0), theCurveDirection);
        GeomAdaptor_Curve aC3d = new(aLine, theFirst, theLast);

        Geom_Plane aPlane =
            new(new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(1.0, 0.0, 0.0)));
        Geom2d_Line aPLine = new(new gp_Pnt2d(0.0, 0.0), thePCurveDirection);
        Geom2dAdaptor_Curve aC2d = new(aPLine, theFirst, theLast);
        GeomAdaptor_Surface aSurf = new(aPlane);
        Adaptor3d_CurveOnSurface aCoS = new(aC2d, aSurf);

        GeomLib_CheckCurveOnSurface aCheck = new();
        aCheck.Init(aC3d, Precision.PConfusion());
        aCheck.Perform(aCoS);
        return aCheck;
    }

    private static GeomLib_CheckCurveOnSurface makeCircleAndEllipseCheck(double theMajorRadius, double theMinorRadius,
                                                                         double theFirst, double theLast)
    {
        Geom_Circle aCircle =
            new(new gp_Ax2(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(1.0, 0.0, 0.0)), 2.0);
        GeomAdaptor_Curve aC3d = new(aCircle, theFirst, theLast);

        Geom2d_Ellipse anEllipse =
            new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), theMajorRadius, theMinorRadius);
        Geom2dAdaptor_Curve aC2d = new(anEllipse, theFirst, theLast);
        Geom_Plane aPlane =
            new(new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(1.0, 0.0, 0.0)));
        GeomAdaptor_Surface aSurf = new(aPlane);
        Adaptor3d_CurveOnSurface aCoS = new(aC2d, aSurf);

        GeomLib_CheckCurveOnSurface aCheck = new();
        aCheck.Init(aC3d, Precision.PConfusion());
        aCheck.Perform(aCoS);
        return aCheck;
    }

    [Fact]
    public void AnalyticCoincident_ReportsZeroDeviation()
    {
        GeomLib_CheckCurveOnSurface aCheck = makeCircleOnCylinderCheck(2.0, 2.0, 5.0);

        Assert.True(aCheck.IsDone());
        Assert.True(aCheck.MaxDistance() <= Precision.Confusion());
    }

    [Fact]
    public void AnalyticDeviating_ReportsActualDeviation()
    {
        // The 3D circle is 0.05 smaller than the cylinder carrying the pcurve: the
        // deviation is constant and equal to the radius difference. This guards the
        // constant-distance shortcut against swallowing real deviations.
        GeomLib_CheckCurveOnSurface aCheck = makeCircleOnCylinderCheck(1.95, 2.0, 5.0);

        Assert.True(aCheck.IsDone());
        Assert.Equal(0.05, aCheck.MaxDistance(), 1.0e-9);
    }

    [Fact]
    public void LineOnPlane_ReportsZeroDeviation()
    {
        GeomLib_CheckCurveOnSurface aCheck =
            makeLineOnPlaneCheck(new gp_Dir(1.0, 0.0, 0.0), new gp_Dir2d(1.0, 0.0), -10.0, 10.0);

        Assert.True(aCheck.IsDone());
        Assert.True(aCheck.MaxDistance() <= Precision.Confusion());
    }

    [Fact]
    public void DeviatingLines_ReportsEndpointMaximum()
    {
        GeomLib_CheckCurveOnSurface aCheck =
            makeLineOnPlaneCheck(new gp_Dir(1.0, 0.0, 0.0), new gp_Dir2d(0.0, 1.0), -2.0, 3.0);

        Assert.True(aCheck.IsDone());
        Assert.Equal(Math.Sqrt(18.0), aCheck.MaxDistance(), Precision.Confusion());
        Assert.Equal(3.0, aCheck.MaxParameter());
    }

    [Fact]
    public void CircleAndEllipse_ReportsAnalyticMaximum()
    {
        GeomLib_CheckCurveOnSurface aCheck = makeCircleAndEllipseCheck(4.0, 1.0, 0.0, 2.0 * Math.PI);

        Assert.True(aCheck.IsDone());
        Assert.Equal(2.0, aCheck.MaxDistance(), Precision.Confusion());
    }

    [Fact]
    public void NegativeTrim_ReportsInteriorMaximum()
    {
        GeomLib_CheckCurveOnSurface aCheck = makeCircleAndEllipseCheck(2.0, 1.0, -2.0, -1.0);

        Assert.True(aCheck.IsDone());
        Assert.Equal(1.0, aCheck.MaxDistance(), Precision.Confusion());
        Assert.Equal(-0.5 * Math.PI, aCheck.MaxParameter(), Precision.PConfusion());
    }

    [Fact]
    public void AnalyticAliasing_ReportsActualDeviation()
    {
        double aFirst = 0.0, aLast = 80.0 * Math.PI;

        Geom_Circle aCircle =
            new(new gp_Ax2(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(1.0, 0.0, 0.0)), 4.0);
        GeomAdaptor_Curve aC3d = new(aCircle, aFirst, aLast);

        Geom_ToroidalSurface aTorus = new(
            new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(1.0, 0.0, 0.0)), 3.0, 1.0);
        Geom2d_Line aPLine = new(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(3.0, 4.0));
        Geom2dAdaptor_Curve aC2d = new(aPLine, aFirst, aLast);
        GeomAdaptor_Surface aSurf = new(aTorus);
        Adaptor3d_CurveOnSurface aCoS = new(aC2d, aSurf);

        GeomLib_CheckCurveOnSurface aCheck = new();
        aCheck.Init(aC3d, Precision.PConfusion());
        aCheck.Perform(aCoS);

        Assert.True(aCheck.IsDone());
        Assert.Equal(8.0, aCheck.MaxDistance(), Precision.Confusion());
    }
}
