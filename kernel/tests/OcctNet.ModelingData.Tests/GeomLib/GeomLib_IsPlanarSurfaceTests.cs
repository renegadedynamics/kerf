// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class GeomLib_IsPlanarSurfaceTests
{
    private static readonly gp_Ax3 TheAx3 = new(new gp_Pnt(1, 2, 3), new gp_Dir(1, 1, 1), new gp_Dir(1, -1, 0));

    private static Geom_BezierSurface PlanarBezier(double theNoise)
    {
        var aPoles = new NCollection_Array2<gp_Pnt>(1, 4, 1, 3);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                double aZ = (i == 2 && j == 2) ? theNoise : 0.0;
                gp_XYZ aP = TheAx3.Location().XYZ() + (i + 0.1 * j) * TheAx3.XDirection().XYZ()
                            + (j * 1.5 - 0.2 * i) * TheAx3.YDirection().XYZ() + aZ * TheAx3.Direction().XYZ();
                aPoles[i, j] = new gp_Pnt(aP);
            }
        }
        return new Geom_BezierSurface(aPoles);
    }

    private static void AssertNormal(gp_Pln thePln, in gp_Dir theExpected)
    {
        Assert.True(thePln.Axis().Direction().IsParallel(theExpected, 1e-9));
    }

    [Fact]
    public void Plane_IsPlanar()
    {
        var aPlane = new Geom_Plane(TheAx3);
        var anIs = new GeomLib_IsPlanarSurface(aPlane);
        Assert.True(anIs.IsPlanar());
        Assert.Equal(aPlane.Pln(), anIs.Plan());
    }

    [Fact]
    public void Quadrics_AreNotPlanar()
    {
        Geom_Surface[] aSurfaces =
        [
            new Geom_CylindricalSurface(TheAx3, 2), new Geom_ConicalSurface(TheAx3, 0.3, 2),
            new Geom_SphericalSurface(TheAx3, 2), new Geom_ToroidalSurface(TheAx3, 5, 1),
        ];
        foreach (Geom_Surface aS in aSurfaces)
        {
            var anIs = new GeomLib_IsPlanarSurface(aS, 1.0);
            Assert.False(anIs.IsPlanar());
            Assert.ThrowsAny<StdFail_NotDone>(() => anIs.Plan());
        }
    }

    [Fact]
    public void PlanarBSpline_IsPlanar_WithItsNormal()
    {
        var aTrimmed = new Geom_RectangularTrimmedSurface(new Geom_Plane(TheAx3), -3, 4, -1, 2);
        Geom_BSplineSurface aBS = GeomConvert.SurfaceToBSplineSurface(aTrimmed);
        var anIs = new GeomLib_IsPlanarSurface(aBS);
        Assert.True(anIs.IsPlanar());
        AssertNormal(anIs.Plan(), TheAx3.Direction());
        Assert.True(anIs.Plan().Distance(TheAx3.Location()) < 1e-12);
    }

    [Fact]
    public void NearlyPlanarBezier_DependsOnTolerance()
    {
        Geom_BezierSurface aBz = PlanarBezier(1e-3);
        Assert.False(new GeomLib_IsPlanarSurface(aBz).IsPlanar());
        var anIs = new GeomLib_IsPlanarSurface(aBz, 1e-2);
        Assert.True(anIs.IsPlanar());
        Assert.True(anIs.Plan().Axis().Direction().Angle(TheAx3.Direction()) < 1e-3
                    || anIs.Plan().Axis().Direction().Angle(TheAx3.Direction()) > M_PI - 1e-3);
        Assert.True(new GeomLib_IsPlanarSurface(PlanarBezier(0.0)).IsPlanar());
    }

    [Fact]
    public void PlanarBezier_XDirectionFollowsDU()
    {
        var anIs = new GeomLib_IsPlanarSurface(PlanarBezier(0.0));
        Assert.True(anIs.IsPlanar());
        PlanarBezier(0.0).D1(0.5, 0.5, out gp_Pnt _, out gp_Vec DU, out gp_Vec DV);
        gp_Ax3 aPos = anIs.Plan().Position();
        Assert.True(aPos.XDirection().Angle(new gp_Dir(DU)) <= M_PI / 2);
        Assert.True(aPos.YDirection().Angle(new gp_Dir(DV)) <= M_PI / 2);
    }

    [Fact]
    public void RevolutionOfPerpendicularSegment_IsPlanar()
    {
        var anAxis = new gp_Ax1(new gp_Pnt(0, 0, 1), new gp_Dir(0, 0, 1));
        var aSeg = new Geom_TrimmedCurve(new Geom_Line(new gp_Pnt(0, 0, 1), new gp_Dir(1, 0, 0)), 1, 3);
        var anIs = new GeomLib_IsPlanarSurface(new Geom_SurfaceOfRevolution(aSeg, anAxis));
        Assert.True(anIs.IsPlanar());
        AssertNormal(anIs.Plan(), new gp_Dir(0, 0, 1));
        Assert.Equal(0.0, anIs.Plan().Distance(new gp_Pnt(5, 7, 1)), 1e-12);

        // A slightly tilted segment sweeps a flat cone: planar within a loose tolerance only
        var aTilted = new Geom_TrimmedCurve(new Geom_Line(new gp_Pnt(0, 0, 1), new gp_Dir(1, 0, 1e-4)), 1, 3);
        var aCone = new Geom_SurfaceOfRevolution(aTilted, anAxis);
        Assert.False(new GeomLib_IsPlanarSurface(aCone).IsPlanar());
        Assert.True(new GeomLib_IsPlanarSurface(aCone, 1e-2).IsPlanar());
    }

    [Fact]
    public void RevolutionOfCircle_IsNotPlanar()
    {
        var aCircle = new Geom_Circle(new gp_Ax2(new gp_Pnt(5, 0, 0), new gp_Dir(0, 1, 0)), 1);
        var aTorus = new Geom_SurfaceOfRevolution(aCircle, new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)));
        Assert.False(new GeomLib_IsPlanarSurface(aTorus).IsPlanar());
    }

    [Fact]
    public void ExtrusionOfPlanarCurve_InItsPlane_IsPlanar()
    {
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 4);
        aPoles[1] = new gp_Pnt(0, 0, 2);
        aPoles[2] = new gp_Pnt(1, 3, 2);
        aPoles[3] = new gp_Pnt(3, -1, 2);
        aPoles[4] = new gp_Pnt(4, 2, 2);
        var aCurve = new Geom_BezierCurve(aPoles);
        var anExt = new Geom_SurfaceOfLinearExtrusion(aCurve, new gp_Dir(1, 1, 0));
        var aBounded = new Geom_RectangularTrimmedSurface(anExt, -1, 1, false);
        var anIs = new GeomLib_IsPlanarSurface(aBounded);
        // A trimmed extrusion is not GeomAbs_SurfaceOfExtrusion for the adaptor: the default (sampling) branch
        Assert.True(anIs.IsPlanar());
        AssertNormal(anIs.Plan(), new gp_Dir(0, 0, 1));
        var anIs2 = new GeomLib_IsPlanarSurface(anExt);
        Assert.True(anIs2.IsPlanar());
        AssertNormal(anIs2.Plan(), new gp_Dir(0, 0, 1));

        // Out of the plane: a cylinder-like surface
        var anUp = new Geom_RectangularTrimmedSurface(new Geom_SurfaceOfLinearExtrusion(aCurve, new gp_Dir(0, 0, 1)),
                                                      -1, 1, false);
        Assert.False(new GeomLib_IsPlanarSurface(anUp).IsPlanar());
    }

    [Fact]
    public void ExtrusionOfLine_IsPlanar()
    {
        var aLine = new Geom_TrimmedCurve(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 0, 5);
        var anExt = new Geom_RectangularTrimmedSurface(
            new Geom_SurfaceOfLinearExtrusion(aLine, new gp_Dir(0, 1, 1)), -2, 2, false);
        var anIs = new GeomLib_IsPlanarSurface(anExt);
        Assert.True(anIs.IsPlanar());
        AssertNormal(anIs.Plan(), new gp_Dir(0, -1, 1));
    }

    [Fact]
    public void OffsetOfPlanarBSpline_IsPlanar()
    {
        var aTrimmed = new Geom_RectangularTrimmedSurface(new Geom_Plane(TheAx3), -3, 4, -1, 2);
        Geom_BSplineSurface aBS = GeomConvert.SurfaceToBSplineSurface(aTrimmed);
        var anIs = new GeomLib_IsPlanarSurface(new Geom_OffsetSurface(aBS, 2.5));
        Assert.True(anIs.IsPlanar());
        AssertNormal(anIs.Plan(), TheAx3.Direction());
        Assert.Equal(2.5, anIs.Plan().Distance(TheAx3.Location()), 1e-9);
    }

    [Fact]
    public void CollinearSamples_AreNotPlanar()
    {
        // A degenerate surface: every pole on one line (the second inertia is below Confusion)
        var aPoles = new NCollection_Array2<gp_Pnt>(1, 2, 1, 2);
        aPoles[1, 1] = new gp_Pnt(0, 0, 0);
        aPoles[1, 2] = new gp_Pnt(1, 0, 0);
        aPoles[2, 1] = new gp_Pnt(2, 0, 0);
        aPoles[2, 2] = new gp_Pnt(3, 0, 0);
        Assert.False(new GeomLib_IsPlanarSurface(new Geom_BezierSurface(aPoles)).IsPlanar());
    }
}
