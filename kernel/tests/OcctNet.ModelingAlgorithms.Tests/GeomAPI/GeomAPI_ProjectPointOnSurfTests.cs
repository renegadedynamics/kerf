// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/GeomAPI_ProjectPointOnSurf_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GeomAPI_ProjectPointOnSurf GTest.</summary>
public class GeomAPI_ProjectPointOnSurfTest
{
    // Bug OCC867: calling Init() with explicit UV bounds then Perform() must not throw.
    // The bounds passed to Init() are intentionally tighter (U:[0,3]) than the trimmed surface
    // (U:[0,4]) to reproduce the original bug scenario.
    [Fact]
    public void Bug867_InitWithTightBoundsNoException()
    {
        Geom_CylindricalSurface aCyl = new(new CoordinateSystem(Constants.XOY()), 10.0);
        Geom_RectangularTrimmedSurface aTrimSurf = new(aCyl, 0.0, 4.0, 0.0, 2.0);

        Point3D aPoint = new(30.0, 30.0, 30.0);

        GeomAPI_ProjectPointOnSurf aProjector = new();
        Exception? anEx = Record.Exception(() =>
        {
            aProjector.Init(aTrimSurf, 0.0, 3.0, 0.0, 2.0);
            aProjector.Perform(aPoint);
        });
        Assert.Null(anEx);
    }
}
