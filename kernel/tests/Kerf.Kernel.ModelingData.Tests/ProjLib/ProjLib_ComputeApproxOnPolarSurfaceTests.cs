// Port of: src/ModelingData/TKGeomBase/GTests/ProjLib_ComputeApproxOnPolarSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class ProjLib_ComputeApproxOnPolarSurfaceTest
{
    private static void checkProjectionAtParam(BSplineCurve2D thePCurve,
                                               CylindricalSurface theSurface,
                                               CircleCurve3D theCurve,
                                               double theParam,
                                               double theTol)
    {
        Point2D aUV = thePCurve.Value(theParam);
        Point3D aProjP = theSurface.Value(aUV.X(), aUV.Y());
        Point3D aRefP = theCurve.Value(theParam);
        Assert.True(aProjP.Distance(aRefP) <= theTol);
    }

    [Fact]
    public void BuildInitialCurveAndProjectOnCylinder_Done()
    {
        CircleCurve3D aCurve =
            new(new Frame3D(new Point3D(0.0, 0.0, 2.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0)), 5.0);
        CylindricalSurface aSurface =
            new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0)), 5.0);

        CurveAdaptorBase aCurveAdaptor = new CurveAdaptor(aCurve);
        SurfaceAdaptorBase aSurfAdaptor = new SurfaceAdaptor(aSurface);

        ComputeApproxOnPolarSurface aProjector = new();
        aProjector.SetTolerance(1.0e-6);

        Curve2DAdaptorBase? anInitCurve2d = aProjector.BuildInitialCurve2d(aCurveAdaptor, aSurfAdaptor);
        Assert.True(aProjector.IsDone);
        Assert.NotNull(anInitCurve2d);

        BSplineCurve2D? aPCurve =
            aProjector.ProjectUsingInitialCurve2d(aCurveAdaptor, aSurfAdaptor, anInitCurve2d);
        Assert.True(aProjector.IsDone);
        Assert.NotNull(aPCurve);

        double aFirst = Max(aCurve.FirstParameter(), aPCurve.FirstParameter());
        double aLast = Min(aCurve.LastParameter(), aPCurve.LastParameter());
        Assert.True(aFirst < aLast);

        checkProjectionAtParam(aPCurve, aSurface, aCurve, aFirst, 1.0e-3);
        checkProjectionAtParam(aPCurve, aSurface, aCurve, 0.5 * (aFirst + aLast), 1.0e-3);
        checkProjectionAtParam(aPCurve, aSurface, aCurve, aLast, 1.0e-3);
    }
}
