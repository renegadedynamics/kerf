// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Geometry.Conversion.ConvType;

namespace Kerf.Kernel.ModelingData.Tests;

public class GeomConvert_SurfToAnaSurfTests
{
    private static readonly CoordinateSystem TheAx3 = new(new Point3D(1, 2, 3), new Direction3D(1, 1, 1), new Direction3D(1, -1, 0));

    private static global::Kerf.Geometry.BSplineSurface AsBSpline(Surface theS, double U1, double U2, double V1, double V2) =>
        global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(new TrimmedSurface(theS, U1, U2, V1, V2));

    private static Surface? Convert(Surface theS, double theTol, out double theGap,
                                         global::Kerf.Engine.Geometry.Conversion.ConvType theType = Simplest,
                                         SurfaceKind theTarget = Plane)
    {
        var aConv = new global::Kerf.Engine.Geometry.Conversion.SurfToAnaSurf(theS);
        aConv.SetConvType(theType);
        aConv.SetTarget(theTarget);
        Surface? aRes = aConv.ConvertToAnalytical(theTol);
        theGap = aConv.Gap();
        return aRes;
    }

    /// <summary>Largest distance from samples of theS (on its bounds) to theAna.</summary>
    private static double Deviation(Surface theS, Surface theAna)
    {
        theS.Bounds(out double U1, out double U2, out double V1, out double V2);
        double aMax = 0;
        for (int i = 0; i <= 10; i++)
        {
            for (int j = 0; j <= 10; j++)
            {
                Point3D aP = theS.Value(U1 + (U2 - U1) * i / 10, V1 + (V2 - V1) * j / 10);
                var anExt = new GeomAPI_ProjectPointOnSurfLite(aP, theAna);
                aMax = Math.Max(aMax, anExt.Distance);
            }
        }
        return aMax;
    }

    [Fact]
    public void ElementarySurface_IsCopied_WithZeroGap()
    {
        var aCyl = new CylindricalSurface(TheAx3, 2);
        Surface? aRes = Convert(aCyl, 1e-3, out double aGap);
        var aResCyl = Assert.IsType<CylindricalSurface>(aRes);
        Assert.NotSame(aCyl, aResCyl);
        Assert.Equal(0.0, aGap);
        Assert.Equal(2.0, aResCyl.Radius());
        Assert.Equal(TheAx3, aResCyl.Position());
    }

    [Fact]
    public void BSplinePlane_BecomesPlane()
    {
        global::Kerf.Geometry.BSplineSurface aBS = AsBSpline(new PlaneSurface(TheAx3), -3, 4, -1, 2);
        Surface? aRes = Convert(aBS, 1e-6, out double aGap);
        var aPlane = Assert.IsType<PlaneSurface>(aRes);
        Assert.True(aGap < 1e-9);
        Assert.True(aPlane.Pln().Axis().Direction().IsParallel(TheAx3.Direction(), 1e-9));
        Assert.True(aPlane.Pln().Distance(TheAx3.Location()) < 1e-9);
    }

    [Fact]
    public void BSplineCylinder_BecomesCylinder()
    {
        global::Kerf.Geometry.BSplineSurface aBS = AsBSpline(new CylindricalSurface(TheAx3, 2.5), 0.3, 2.5, -1, 3);
        Surface? aRes = Convert(aBS, 1e-4, out double aGap);
        var aCyl = Assert.IsType<CylindricalSurface>(aRes);
        Assert.Equal(2.5, aCyl.Radius(), 1e-6);
        Assert.True(aCyl.Axis().Direction().IsParallel(TheAx3.Direction(), 1e-6));
        Assert.True(aGap <= 1e-4);
        Assert.True(Deviation(aBS, aCyl) < 1e-4);
    }

    [Fact]
    public void BSplineCone_BecomesCone()
    {
        global::Kerf.Geometry.BSplineSurface aBS = AsBSpline(new ConicalSurface(TheAx3, 0.4, 2), 0.1, 3, 0.5, 4);
        Surface? aRes = Convert(aBS, 1e-4, out double aGap);
        var aCone = Assert.IsType<ConicalSurface>(aRes);
        Assert.Equal(0.4, aCone.SemiAngle(), 1e-6);
        Assert.True(aGap <= 1e-4);
        Assert.True(Deviation(aBS, aCone) < 1e-4);
    }

    [Fact]
    public void BSplineSphere_BecomesSphere()
    {
        global::Kerf.Geometry.BSplineSurface aBS = AsBSpline(new SphericalSurface(TheAx3, 3), 0.2, 2.2, -0.8, 0.9);
        Surface? aRes = Convert(aBS, 1e-4, out double aGap);
        var aSph = Assert.IsType<SphericalSurface>(aRes);
        Assert.Equal(3.0, aSph.Radius(), 1e-6);
        Assert.True(aSph.Location().Distance(TheAx3.Location()) < 1e-6);
        Assert.True(aGap <= 1e-4);
    }

    [Fact]
    public void BSplineTorus_BecomesTorus()
    {
        global::Kerf.Geometry.BSplineSurface aBS = AsBSpline(new ToroidalSurface(TheAx3, 6, 1.5), 0.2, 2.2, 0.3, 2.0);
        Surface? aRes = Convert(aBS, 1e-4, out double aGap);
        var aTor = Assert.IsType<ToroidalSurface>(aRes);
        Assert.Equal(6.0, aTor.MajorRadius(), 1e-6);
        Assert.Equal(1.5, aTor.MinorRadius(), 1e-6);
        Assert.True(aGap <= 1e-4);
    }

    [Fact]
    public void RevolutionOfParallelLine_BecomesCylinder_AndTiltedLine_Cone()
    {
        var anAxis = new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var aPar = new TrimmedCurve3D(new LineCurve3D(new Point3D(2, 0, 0), new Direction3D(0, 0, 1)), 0, 5);
        var aCyl = Assert.IsType<CylindricalSurface>(
            Convert(new RevolutionSurface(aPar, anAxis), 1e-5, out double aGap1));
        Assert.Equal(2.0, aCyl.Radius(), 1e-9);
        Assert.True(aGap1 < 1e-9);

        var aTilted = new TrimmedCurve3D(new LineCurve3D(new Point3D(2, 0, 0), new Direction3D(0.3, 0, 1)), 0, 5);
        var aCone = Assert.IsType<ConicalSurface>(
            Convert(new RevolutionSurface(aTilted, anAxis), 1e-5, out double aGap2));
        Assert.Equal(Math.Atan(0.3), aCone.SemiAngle(), 1e-9);
        Assert.True(aGap2 < 1e-9);
    }

    [Fact]
    public void RevolutionOfCircle_BecomesTorus_OrSphere()
    {
        var anAxis = new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(5, 0, 0), new Direction3D(0, 1, 0)), 1);
        var aRev = new TrimmedSurface(new RevolutionSurface(aCircle, anAxis), 0.1, 2, 0.2, 2.5);
        var aTor = Assert.IsType<ToroidalSurface>(Convert(aRev, 1e-5, out _));
        Assert.Equal(5.0, aTor.MajorRadius(), 1e-9);
        Assert.Equal(1.0, aTor.MinorRadius(), 1e-9);

        var aCentred = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 0)), 2);
        var aSphRev = new TrimmedSurface(new RevolutionSurface(aCentred, anAxis),
                                                         0.1, 2, 0.2, 1.3);
        var aSph = Assert.IsType<SphericalSurface>(Convert(aSphRev, 1e-5, out _));
        Assert.Equal(2.0, aSph.Radius(), 1e-9);
    }

    [Fact]
    public void ExtrusionOfCircle_BecomesCylinder()
    {
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(1, 1, 0), new Direction3D(0, 0, 1)), 3);
        var anExt = new TrimmedSurface(new ExtrusionSurface(aCircle, new Direction3D(0, 0, 1)),
                                                       0.5, 2.5, -1, 2);
        var aCyl = Assert.IsType<CylindricalSurface>(Convert(anExt, 1e-5, out double aGap));
        Assert.Equal(3.0, aCyl.Radius(), 1e-9);
        Assert.True(aGap < 1e-9);
    }

    [Fact]
    public void FreeFormBSpline_IsNotConverted()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aPoles[i, j] = new Point3D(i, j, Math.Sin(i * 1.7) * Math.Cos(j * 2.3));
            }
        }
        var aConv = new global::Kerf.Engine.Geometry.Conversion.SurfToAnaSurf(new global::Kerf.Geometry.BezierSurface(aPoles));
        Assert.Null(aConv.ConvertToAnalytical(1e-3));
        Assert.Equal(-1.0, aConv.Gap());
    }

    [Fact]
    public void Target_OtherType_GivesNull()
    {
        global::Kerf.Geometry.BSplineSurface aBS = AsBSpline(new CylindricalSurface(TheAx3, 2.5), 0.3, 2.5, -1, 3);
        Assert.Null(Convert(aBS, 1e-4, out _, Target, Plane));
        Assert.Null(Convert(aBS, 1e-4, out double aGap, Target, SurfaceKind.Sphere));
        Assert.Equal(RealLast(), aGap);
        Assert.IsType<CylindricalSurface>(Convert(aBS, 1e-4, out _, Target, SurfaceKind.Cylinder));
        Assert.IsType<CylindricalSurface>(Convert(aBS, 1e-4, out _, MinGap));
    }

    [Fact]
    public void Range_OutsideTheSurface_GivesNull()
    {
        global::Kerf.Geometry.BSplineSurface aBS = AsBSpline(new PlaneSurface(TheAx3), -3, 4, -1, 2);
        var aConv = new global::Kerf.Engine.Geometry.Conversion.SurfToAnaSurf(aBS);
        Assert.Null(aConv.ConvertToAnalytical(1e-6, -10, 4, -1, 2));
    }

    [Fact]
    public void SubRange_SegmentsACopy_NotTheCallersSurface()
    {
        global::Kerf.Geometry.BSplineSurface aBS = AsBSpline(new CylindricalSurface(TheAx3, 2.5), 0.3, 2.5, -1, 3);
        aBS.Bounds(out double U1, out double U2, out double V1, out double V2);
        aBS.Freeze();
        var aConv = new global::Kerf.Engine.Geometry.Conversion.SurfToAnaSurf(aBS);
        var aCyl = Assert.IsType<CylindricalSurface>(aConv.ConvertToAnalytical(1e-4, U1 + 0.2, U2 - 0.3, V1 + 0.5, V2));
        Assert.Equal(2.5, aCyl.Radius(), 1e-6);
        aBS.Bounds(out double U1b, out double U2b, out double V1b, out double V2b);
        Assert.Equal((U1, U2, V1, V2), (U1b, U2b, V1b, V2b));
    }

    [Fact]
    public void IsSame_And_IsCanonical()
    {
        var aCyl1 = new CylindricalSurface(TheAx3, 2);
        var aCyl2 = new CylindricalSurface(TheAx3.Translated(new Vector3D(TheAx3.Direction()) * 5), 2);
        var aCyl3 = new CylindricalSurface(TheAx3, 2.1);
        Assert.True(global::Kerf.Engine.Geometry.Conversion.SurfToAnaSurf.IsSame(aCyl1, aCyl2, 1e-7));
        Assert.False(global::Kerf.Engine.Geometry.Conversion.SurfToAnaSurf.IsSame(aCyl1, aCyl3, 1e-7));
        Assert.True(global::Kerf.Engine.Geometry.Conversion.SurfToAnaSurf.IsSame(new PlaneSurface(TheAx3), new PlaneSurface(TheAx3), 1e-7));
        Assert.False(global::Kerf.Engine.Geometry.Conversion.SurfToAnaSurf.IsSame(aCyl1, new PlaneSurface(TheAx3), 1e-7));
        global::Kerf.Geometry.BSplineSurface aBS = AsBSpline(new PlaneSurface(TheAx3), -3, 4, -1, 2);
        Assert.False(global::Kerf.Engine.Geometry.Conversion.SurfToAnaSurf.IsSame(aBS, aBS, 1e-7));

        Assert.True(global::Kerf.Engine.Geometry.Conversion.SurfToAnaSurf.IsCanonical(aCyl1));
        Assert.True(global::Kerf.Engine.Geometry.Conversion.SurfToAnaSurf.IsCanonical(new ToroidalSurface(TheAx3, 3, 1)));
        Assert.False(global::Kerf.Engine.Geometry.Conversion.SurfToAnaSurf.IsCanonical(aBS));
        Assert.False(global::Kerf.Engine.Geometry.Conversion.SurfToAnaSurf.IsCanonical(null));
    }

    // A tiny projection helper (Extrema_ExtPS through GeomAdaptor_Surface), to measure deviations.
    private readonly struct GeomAPI_ProjectPointOnSurfLite
    {
        public readonly double Distance;

        public GeomAPI_ProjectPointOnSurfLite(in Point3D theP, Surface theS)
        {
            var anAd = new SurfaceAdaptor(theS, -100, 100, -100, 100);
            var anExt = new ExtPS(theP, anAd, 1e-9, 1e-9);
            double aMin = double.MaxValue;
            for (int i = 1; anExt.IsDone && i <= anExt.ExtCount; i++)
            {
                aMin = Math.Min(aMin, anExt.SquareDistance(i));
            }
            Distance = Math.Sqrt(aMin);
        }
    }
}
