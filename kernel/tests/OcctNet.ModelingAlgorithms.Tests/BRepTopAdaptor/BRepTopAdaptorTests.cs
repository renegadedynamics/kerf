// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using OcctNet.ModelingData.Tests;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepTopAdaptor: the fast 2D classifier on a holed square and on a periodic cylinder face (with the
/// points brought back into the face's period), its test on the restriction, and the topological tool and its vertices.
/// </summary>
public class BRepTopAdaptorTests
{
    private static TopoDS_Face Holed() => TestShapes.Frozen(ClassificationTestShapes.HoledSquare());

    [Theory]
    [InlineData(2.0, 2.0, TopAbs_IN)]
    [InlineData(5.0, 5.0, TopAbs_OUT)]
    [InlineData(11.0, 2.0, TopAbs_OUT)]
    [InlineData(0.0, 5.0, TopAbs_ON)]
    [InlineData(4.0, 5.0, TopAbs_ON)]
    [InlineData(10.0, 10.0, TopAbs_ON)]
    public void FClass2d_on_holed_square(double u, double v, TopAbs_State theExpected)
    {
        var aClass = new BRepTopAdaptor_FClass2d(Holed(), 1e-7);
        Assert.Equal(theExpected, aClass.Perform(new Point2D(u, v)));
        Assert.Equal(theExpected, aClass.Perform(new Point2D(u, v), false));
        Assert.Equal(TopAbs_OUT, aClass.PerformInfinitePoint());
    }

    [Fact]
    public void FClass2d_test_on_restriction()
    {
        var aClass = new BRepTopAdaptor_FClass2d(Holed(), 1e-7);
        Assert.Equal(TopAbs_ON, aClass.TestOnRestriction(new Point2D(1e-4, 5), 1e-3));
        Assert.Equal(TopAbs_IN, aClass.TestOnRestriction(new Point2D(2, 2), 1e-3));
        Assert.Equal(TopAbs_OUT, aClass.TestOnRestriction(new Point2D(5, 5), 1e-3));
    }

    [Fact]
    public void FClass2d_on_periodic_cylinder_brings_points_into_the_period()
    {
        TopoDS_Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        var aClass = new BRepTopAdaptor_FClass2d(aFace, 1e-7);
        Assert.Equal(TopAbs_IN, aClass.Perform(new Point2D(Math.PI, 1.0)));
        // a period further: IN when brought back, OUT when not
        Assert.Equal(TopAbs_IN, aClass.Perform(new Point2D(Math.PI + 2.0 * Math.PI, 1.0)));
        Assert.Equal(TopAbs_IN, aClass.Perform(new Point2D(Math.PI - 4.0 * Math.PI, 1.0)));
        Assert.Equal(TopAbs_OUT, aClass.Perform(new Point2D(Math.PI + 2.0 * Math.PI, 1.0), false));
        Assert.Equal(TopAbs_OUT, aClass.Perform(new Point2D(Math.PI, 5.0)));
        Assert.Equal(TopAbs_ON, aClass.Perform(new Point2D(Math.PI, 3.0)));
    }

    [Fact]
    public void FClass2d_natural_face_is_all_in()
    {
        TopoDS_Face aFace = TestShapes.Frozen(BRepGPropTestShapes.NaturalFace(
            new Geom_SphericalSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1.0)));
        var aClass = new BRepTopAdaptor_FClass2d(aFace, 1e-7);
        Assert.Equal(TopAbs_IN, aClass.Perform(new Point2D(100, 100)));
    }

    [Fact]
    public void TopolTool_explores_the_face()
    {
        TopoDS_Face aFace = Holed();
        var aSurf = new BRepAdaptor_Surface(aFace);
        var aTool = new BRepTopAdaptor_TopolTool(aSurf);
        Assert.True(aTool.Has3d());
        Assert.False(aTool.DomainIsInfinite());

        int aNbArcs = 0;
        for (aTool.Init(); aTool.More(); aTool.Next())
        {
            Adaptor2d_Curve2d anArc = aTool.Value();
            Assert.IsType<BRepAdaptor_Curve2d>(anArc);
            Assert.Equal(TopAbs_FORWARD, aTool.Orientation(anArc));
            Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aTool.Tol3d(anArc));
            Assert.IsType<TopoDS_Edge>(aTool.Edge());

            // the vertices of the arc
            aTool.Initialize(anArc);
            int aNbVertices = 0;
            for (aTool.InitVertexIterator(); aTool.MoreVertex(); aTool.NextVertex())
            {
                Adaptor3d_HVertex aV = aTool.Vertex();
                Assert.IsType<BRepTopAdaptor_HVertex>(aV);
                Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aTool.Tol3d(aV));
                Point3D aP = aTool.Pnt(aV);
                Assert.Equal(0.0, aP.Z(), 1e-12);
                double aParam = aV.Parameter(anArc);
                Assert.True(aParam >= anArc.FirstParameter() - 1e-9 && aParam <= anArc.LastParameter() + 1e-9);
                Assert.True(aV.Resolution(anArc) > 0.0);
                Assert.True(aV.IsSame(aV));
                Assert.Equal(RealFirst(), aV.Value().X());
                aNbVertices++;
            }

            Assert.Equal(2, aNbVertices);
            aNbArcs++;
        }

        Assert.Equal(8, aNbArcs);
        Assert.Equal(TopAbs_IN, aTool.Classify(new Point2D(2, 2), 1e-7));
        Assert.Equal(TopAbs_OUT, aTool.Classify(new Point2D(5, 5), 1e-7));
        Assert.True(aTool.IsThePointOn(new Point2D(0, 5), 1e-7));
        Assert.False(aTool.IsThePointOn(new Point2D(2, 2), 1e-7));

        // the samples of a plane: at least 10 by 10
        Assert.Equal(10, aTool.NbSamplesU());
        Assert.Equal(10, aTool.NbSamplesV());
        Assert.Equal(100, aTool.NbSamples());
        aTool.SamplePoint(0, out Point2D aP2d, out Point3D aP3d);
        Assert.Equal(aP2d.X(), aP3d.X(), 1e-12);
        Assert.Equal(aP2d.Y(), aP3d.Y(), 1e-12);
    }

    [Fact]
    public void TopolTool_needs_brep_adaptors()
    {
        var aTool = new BRepTopAdaptor_TopolTool();
        var aGeomSurf = new GeomAdaptor_Surface(new Geom_Plane(new CoordinateSystem()));
        Assert.Throws<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => aTool.Initialize(aGeomSurf));
        Assert.Throws<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => aTool.Initialize(new Geom2dAdaptor_Curve(new Geom2d_Line(new Point2D(), new Direction2D()))));
        Assert.Throws<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aTool.Initialize());
        Assert.Equal(TopAbs_UNKNOWN, aTool.Classify(new Point2D(), 1e-7));
    }

    [Fact]
    public void TopolTool_samples_a_bspline_surface_by_its_poles()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 6, 1, 6);
        for (int i = 1; i <= 6; i++)
        {
            for (int j = 1; j <= 6; j++)
            {
                aPoles[i, j] = new Point3D(i, j, ((i + j) % 2) * 0.5);
            }
        }

        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 6;
        aMults[2] = 6;
        var aSurface = new Geom_BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 5, 5);
        TopoDS_Face aFace = TestShapes.Frozen(BRepGPropTestShapes.NaturalFace(aSurface));
        var aTool = new BRepTopAdaptor_TopolTool(new BRepAdaptor_Surface(aFace));
        Assert.Equal(BSplineSurface, new BRepAdaptor_Surface(aFace).GetType());
        Assert.True(aTool.NbSamplesU() >= 10);
        Assert.True(aTool.NbSamplesV() >= 10);
    }

    [Fact]
    public void Tool_keeps_the_surface_and_its_topological_tool()
    {
        TopoDS_Face aFace = Holed();
        var aTool = new BRepTopAdaptor_Tool(aFace, 1e-7);
        Assert.IsType<BRepAdaptor_Surface>(aTool.GetSurface());
        Assert.Equal(TopAbs_IN, aTool.GetTopolTool().Classify(new Point2D(2, 2), 1e-7));

        var anEmpty = new BRepTopAdaptor_Tool();
        Assert.Null(anEmpty.GetSurface());
        anEmpty.Init(aFace, 1e-7);
        Assert.NotNull(anEmpty.GetSurface());
        var anOther = new BRepTopAdaptor_TopolTool();
        anEmpty.SetTopolTool(anOther);
        Assert.Same(anOther, anEmpty.GetTopolTool());
    }
}
