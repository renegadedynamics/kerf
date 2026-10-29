// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static Occt.GeomConvert_ConvType;

namespace OcctNet.ModelingData.Tests;

public class GeomConvert_SurfToAnaSurfTests
{
    private static readonly gp_Ax3 TheAx3 = new(new gp_Pnt(1, 2, 3), new gp_Dir(1, 1, 1), new gp_Dir(1, -1, 0));

    private static Geom_BSplineSurface AsBSpline(Geom_Surface theS, double U1, double U2, double V1, double V2) =>
        GeomConvert.SurfaceToBSplineSurface(new Geom_RectangularTrimmedSurface(theS, U1, U2, V1, V2));

    private static Geom_Surface? Convert(Geom_Surface theS, double theTol, out double theGap,
                                         GeomConvert_ConvType theType = GeomConvert_Simplest,
                                         GeomAbs_SurfaceType theTarget = GeomAbs_Plane)
    {
        var aConv = new GeomConvert_SurfToAnaSurf(theS);
        aConv.SetConvType(theType);
        aConv.SetTarget(theTarget);
        Geom_Surface? aRes = aConv.ConvertToAnalytical(theTol);
        theGap = aConv.Gap();
        return aRes;
    }

    /// <summary>Largest distance from samples of theS (on its bounds) to theAna.</summary>
    private static double Deviation(Geom_Surface theS, Geom_Surface theAna)
    {
        theS.Bounds(out double U1, out double U2, out double V1, out double V2);
        double aMax = 0;
        for (int i = 0; i <= 10; i++)
        {
            for (int j = 0; j <= 10; j++)
            {
                gp_Pnt aP = theS.Value(U1 + (U2 - U1) * i / 10, V1 + (V2 - V1) * j / 10);
                var anExt = new GeomAPI_ProjectPointOnSurfLite(aP, theAna);
                aMax = Math.Max(aMax, anExt.Distance);
            }
        }
        return aMax;
    }

    [Fact]
    public void ElementarySurface_IsCopied_WithZeroGap()
    {
        var aCyl = new Geom_CylindricalSurface(TheAx3, 2);
        Geom_Surface? aRes = Convert(aCyl, 1e-3, out double aGap);
        var aResCyl = Assert.IsType<Geom_CylindricalSurface>(aRes);
        Assert.NotSame(aCyl, aResCyl);
        Assert.Equal(0.0, aGap);
        Assert.Equal(2.0, aResCyl.Radius());
        Assert.Equal(TheAx3, aResCyl.Position());
    }

    [Fact]
    public void BSplinePlane_BecomesPlane()
    {
        Geom_BSplineSurface aBS = AsBSpline(new Geom_Plane(TheAx3), -3, 4, -1, 2);
        Geom_Surface? aRes = Convert(aBS, 1e-6, out double aGap);
        var aPlane = Assert.IsType<Geom_Plane>(aRes);
        Assert.True(aGap < 1e-9);
        Assert.True(aPlane.Pln().Axis().Direction().IsParallel(TheAx3.Direction(), 1e-9));
        Assert.True(aPlane.Pln().Distance(TheAx3.Location()) < 1e-9);
    }

    [Fact]
    public void BSplineCylinder_BecomesCylinder()
    {
        Geom_BSplineSurface aBS = AsBSpline(new Geom_CylindricalSurface(TheAx3, 2.5), 0.3, 2.5, -1, 3);
        Geom_Surface? aRes = Convert(aBS, 1e-4, out double aGap);
        var aCyl = Assert.IsType<Geom_CylindricalSurface>(aRes);
        Assert.Equal(2.5, aCyl.Radius(), 1e-6);
        Assert.True(aCyl.Axis().Direction().IsParallel(TheAx3.Direction(), 1e-6));
        Assert.True(aGap <= 1e-4);
        Assert.True(Deviation(aBS, aCyl) < 1e-4);
    }

    [Fact]
    public void BSplineCone_BecomesCone()
    {
        Geom_BSplineSurface aBS = AsBSpline(new Geom_ConicalSurface(TheAx3, 0.4, 2), 0.1, 3, 0.5, 4);
        Geom_Surface? aRes = Convert(aBS, 1e-4, out double aGap);
        var aCone = Assert.IsType<Geom_ConicalSurface>(aRes);
        Assert.Equal(0.4, aCone.SemiAngle(), 1e-6);
        Assert.True(aGap <= 1e-4);
        Assert.True(Deviation(aBS, aCone) < 1e-4);
    }

    [Fact]
    public void BSplineSphere_BecomesSphere()
    {
        Geom_BSplineSurface aBS = AsBSpline(new Geom_SphericalSurface(TheAx3, 3), 0.2, 2.2, -0.8, 0.9);
        Geom_Surface? aRes = Convert(aBS, 1e-4, out double aGap);
        var aSph = Assert.IsType<Geom_SphericalSurface>(aRes);
        Assert.Equal(3.0, aSph.Radius(), 1e-6);
        Assert.True(aSph.Location().Distance(TheAx3.Location()) < 1e-6);
        Assert.True(aGap <= 1e-4);
    }

    [Fact]
    public void BSplineTorus_BecomesTorus()
    {
        Geom_BSplineSurface aBS = AsBSpline(new Geom_ToroidalSurface(TheAx3, 6, 1.5), 0.2, 2.2, 0.3, 2.0);
        Geom_Surface? aRes = Convert(aBS, 1e-4, out double aGap);
        var aTor = Assert.IsType<Geom_ToroidalSurface>(aRes);
        Assert.Equal(6.0, aTor.MajorRadius(), 1e-6);
        Assert.Equal(1.5, aTor.MinorRadius(), 1e-6);
        Assert.True(aGap <= 1e-4);
    }

    [Fact]
    public void RevolutionOfParallelLine_BecomesCylinder_AndTiltedLine_Cone()
    {
        var anAxis = new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        var aPar = new Geom_TrimmedCurve(new Geom_Line(new gp_Pnt(2, 0, 0), new gp_Dir(0, 0, 1)), 0, 5);
        var aCyl = Assert.IsType<Geom_CylindricalSurface>(
            Convert(new Geom_SurfaceOfRevolution(aPar, anAxis), 1e-5, out double aGap1));
        Assert.Equal(2.0, aCyl.Radius(), 1e-9);
        Assert.True(aGap1 < 1e-9);

        var aTilted = new Geom_TrimmedCurve(new Geom_Line(new gp_Pnt(2, 0, 0), new gp_Dir(0.3, 0, 1)), 0, 5);
        var aCone = Assert.IsType<Geom_ConicalSurface>(
            Convert(new Geom_SurfaceOfRevolution(aTilted, anAxis), 1e-5, out double aGap2));
        Assert.Equal(Math.Atan(0.3), aCone.SemiAngle(), 1e-9);
        Assert.True(aGap2 < 1e-9);
    }

    [Fact]
    public void RevolutionOfCircle_BecomesTorus_OrSphere()
    {
        var anAxis = new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        var aCircle = new Geom_Circle(new gp_Ax2(new gp_Pnt(5, 0, 0), new gp_Dir(0, 1, 0)), 1);
        var aRev = new Geom_RectangularTrimmedSurface(new Geom_SurfaceOfRevolution(aCircle, anAxis), 0.1, 2, 0.2, 2.5);
        var aTor = Assert.IsType<Geom_ToroidalSurface>(Convert(aRev, 1e-5, out _));
        Assert.Equal(5.0, aTor.MajorRadius(), 1e-9);
        Assert.Equal(1.0, aTor.MinorRadius(), 1e-9);

        var aCentred = new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 0)), 2);
        var aSphRev = new Geom_RectangularTrimmedSurface(new Geom_SurfaceOfRevolution(aCentred, anAxis),
                                                         0.1, 2, 0.2, 1.3);
        var aSph = Assert.IsType<Geom_SphericalSurface>(Convert(aSphRev, 1e-5, out _));
        Assert.Equal(2.0, aSph.Radius(), 1e-9);
    }

    [Fact]
    public void ExtrusionOfCircle_BecomesCylinder()
    {
        var aCircle = new Geom_Circle(new gp_Ax2(new gp_Pnt(1, 1, 0), new gp_Dir(0, 0, 1)), 3);
        var anExt = new Geom_RectangularTrimmedSurface(new Geom_SurfaceOfLinearExtrusion(aCircle, new gp_Dir(0, 0, 1)),
                                                       0.5, 2.5, -1, 2);
        var aCyl = Assert.IsType<Geom_CylindricalSurface>(Convert(anExt, 1e-5, out double aGap));
        Assert.Equal(3.0, aCyl.Radius(), 1e-9);
        Assert.True(aGap < 1e-9);
    }

    [Fact]
    public void FreeFormBSpline_IsNotConverted()
    {
        var aPoles = new NCollection_Array2<gp_Pnt>(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aPoles[i, j] = new gp_Pnt(i, j, Math.Sin(i * 1.7) * Math.Cos(j * 2.3));
            }
        }
        var aConv = new GeomConvert_SurfToAnaSurf(new Geom_BezierSurface(aPoles));
        Assert.Null(aConv.ConvertToAnalytical(1e-3));
        Assert.Equal(-1.0, aConv.Gap());
    }

    [Fact]
    public void Target_OtherType_GivesNull()
    {
        Geom_BSplineSurface aBS = AsBSpline(new Geom_CylindricalSurface(TheAx3, 2.5), 0.3, 2.5, -1, 3);
        Assert.Null(Convert(aBS, 1e-4, out _, GeomConvert_Target, GeomAbs_Plane));
        Assert.Null(Convert(aBS, 1e-4, out double aGap, GeomConvert_Target, GeomAbs_Sphere));
        Assert.Equal(RealLast(), aGap);
        Assert.IsType<Geom_CylindricalSurface>(Convert(aBS, 1e-4, out _, GeomConvert_Target, GeomAbs_Cylinder));
        Assert.IsType<Geom_CylindricalSurface>(Convert(aBS, 1e-4, out _, GeomConvert_MinGap));
    }

    [Fact]
    public void Range_OutsideTheSurface_GivesNull()
    {
        Geom_BSplineSurface aBS = AsBSpline(new Geom_Plane(TheAx3), -3, 4, -1, 2);
        var aConv = new GeomConvert_SurfToAnaSurf(aBS);
        Assert.Null(aConv.ConvertToAnalytical(1e-6, -10, 4, -1, 2));
    }

    [Fact]
    public void SubRange_SegmentsACopy_NotTheCallersSurface()
    {
        Geom_BSplineSurface aBS = AsBSpline(new Geom_CylindricalSurface(TheAx3, 2.5), 0.3, 2.5, -1, 3);
        aBS.Bounds(out double U1, out double U2, out double V1, out double V2);
        aBS.Freeze();
        var aConv = new GeomConvert_SurfToAnaSurf(aBS);
        var aCyl = Assert.IsType<Geom_CylindricalSurface>(aConv.ConvertToAnalytical(1e-4, U1 + 0.2, U2 - 0.3, V1 + 0.5, V2));
        Assert.Equal(2.5, aCyl.Radius(), 1e-6);
        aBS.Bounds(out double U1b, out double U2b, out double V1b, out double V2b);
        Assert.Equal((U1, U2, V1, V2), (U1b, U2b, V1b, V2b));
    }

    [Fact]
    public void IsSame_And_IsCanonical()
    {
        var aCyl1 = new Geom_CylindricalSurface(TheAx3, 2);
        var aCyl2 = new Geom_CylindricalSurface(TheAx3.Translated(new gp_Vec(TheAx3.Direction()) * 5), 2);
        var aCyl3 = new Geom_CylindricalSurface(TheAx3, 2.1);
        Assert.True(GeomConvert_SurfToAnaSurf.IsSame(aCyl1, aCyl2, 1e-7));
        Assert.False(GeomConvert_SurfToAnaSurf.IsSame(aCyl1, aCyl3, 1e-7));
        Assert.True(GeomConvert_SurfToAnaSurf.IsSame(new Geom_Plane(TheAx3), new Geom_Plane(TheAx3), 1e-7));
        Assert.False(GeomConvert_SurfToAnaSurf.IsSame(aCyl1, new Geom_Plane(TheAx3), 1e-7));
        Geom_BSplineSurface aBS = AsBSpline(new Geom_Plane(TheAx3), -3, 4, -1, 2);
        Assert.False(GeomConvert_SurfToAnaSurf.IsSame(aBS, aBS, 1e-7));

        Assert.True(GeomConvert_SurfToAnaSurf.IsCanonical(aCyl1));
        Assert.True(GeomConvert_SurfToAnaSurf.IsCanonical(new Geom_ToroidalSurface(TheAx3, 3, 1)));
        Assert.False(GeomConvert_SurfToAnaSurf.IsCanonical(aBS));
        Assert.False(GeomConvert_SurfToAnaSurf.IsCanonical(null));
    }

    // A tiny projection helper (Extrema_ExtPS through GeomAdaptor_Surface), to measure deviations.
    private readonly struct GeomAPI_ProjectPointOnSurfLite
    {
        public readonly double Distance;

        public GeomAPI_ProjectPointOnSurfLite(in gp_Pnt theP, Geom_Surface theS)
        {
            var anAd = new GeomAdaptor_Surface(theS, -100, 100, -100, 100);
            var anExt = new Extrema_ExtPS(theP, anAd, 1e-9, 1e-9);
            double aMin = double.MaxValue;
            for (int i = 1; anExt.IsDone() && i <= anExt.NbExt(); i++)
            {
                aMin = Math.Min(aMin, anExt.SquareDistance(i));
            }
            Distance = Math.Sqrt(aMin);
        }
    }
}
