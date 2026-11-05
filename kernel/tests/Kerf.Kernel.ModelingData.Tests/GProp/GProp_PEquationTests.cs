// Port of: src/ModelingData/TKGeomBase/GTests/GProp_PEquation_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class GProp_PEquationTest
{
    private static readonly double Conf = global::Kerf.Engine.Foundation.Precision.Confusion();

    [Fact]
    public void CoincidentPoints()
    {
        var aPnts = new NCollection_Array1<Point3D>(1, 3);
        aPnts[1] = new Point3D(1.0, 2.0, 3.0);
        aPnts[2] = new Point3D(1.0, 2.0, 3.0);
        aPnts[3] = new Point3D(1.0, 2.0, 3.0);

        var anEq = new PEquation(aPnts, 1e-6);
        Assert.True(anEq.IsPoint);

        Point3D aP = anEq.Point();
        Assert.Equal(1.0, aP.X, Conf);
        Assert.Equal(2.0, aP.Y, Conf);
        Assert.Equal(3.0, aP.Z, Conf);
    }

    [Fact]
    public void CollinearPoints()
    {
        var aPnts = new NCollection_Array1<Point3D>(1, 3);
        aPnts[1] = new Point3D(0.0, 0.0, 0.0);
        aPnts[2] = new Point3D(1.0, 0.0, 0.0);
        aPnts[3] = new Point3D(2.0, 0.0, 0.0);

        var anEq = new PEquation(aPnts, 1e-6);
        Assert.True(anEq.IsLinear);

        Line3D aLin = anEq.Line();
        Assert.Equal(1.0, Math.Abs(aLin.Direction.X), Conf);
    }

    [Fact]
    public void CoplanarPoints()
    {
        var aPnts = new NCollection_Array1<Point3D>(1, 4);
        aPnts[1] = new Point3D(0.0, 0.0, 0.0);
        aPnts[2] = new Point3D(1.0, 0.0, 0.0);
        aPnts[3] = new Point3D(0.0, 1.0, 0.0);
        aPnts[4] = new Point3D(1.0, 1.0, 0.0);

        var anEq = new PEquation(aPnts, 1e-6);
        Assert.True(anEq.IsPlanar);

        Plane3D aPln = anEq.Plane();
        Assert.Equal(1.0, Math.Abs(aPln.Axis.Direction.Z), Conf);
    }

    [Fact]
    public void SpacePoints()
    {
        var aPnts = new NCollection_Array1<Point3D>(1, 4);
        aPnts[1] = new Point3D(0.0, 0.0, 0.0);
        aPnts[2] = new Point3D(1.0, 0.0, 0.0);
        aPnts[3] = new Point3D(0.0, 1.0, 0.0);
        aPnts[4] = new Point3D(0.0, 0.0, 1.0);

        var anEq = new PEquation(aPnts, 1e-6);
        Assert.True(anEq.IsSpace);

        anEq.Box(out Point3D _, out Vector3D aV1, out Vector3D aV2, out Vector3D aV3);
        Assert.True(aV1.Magnitude > Conf);
        Assert.True(aV2.Magnitude > Conf);
        Assert.True(aV3.Magnitude > Conf);
    }

    // OCCT.NET additions.

    [Fact]
    public void WrongAccessorThrows()
    {
        var aPnts = new NCollection_Array1<Point3D>(1, 3);
        aPnts[1] = new Point3D(0.0, 0.0, 0.0);
        aPnts[2] = new Point3D(1.0, 0.0, 0.0);
        aPnts[3] = new Point3D(2.0, 0.0, 0.0);

        var anEq = new PEquation(aPnts, 1e-6);
        Assert.Equal(PEquation.Type.Line, anEq.GetType());
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => anEq.Plane());
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => anEq.Point());
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => anEq.Box(out _, out _, out _, out _));
        Assert.Equal(1.0, anEq.Barycentre().X, Conf);
        Assert.Equal(2.0, anEq.Extent(3), Conf); // sorted by decreasing moment: the line is the last axis
    }
}
