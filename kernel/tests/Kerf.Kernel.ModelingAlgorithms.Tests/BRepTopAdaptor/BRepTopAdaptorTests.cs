// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepTopAdaptor: the fast 2D classifier on a holed square and on a periodic cylinder face (with the
/// points brought back into the face's period), its test on the restriction, and the topological tool and its vertices.
/// </summary>
public class BRepTopAdaptorTests
{
    private static Face Holed() => TestShapes.Frozen(ClassificationTestShapes.HoledSquare());

    [Theory]
    [InlineData(2.0, 2.0, In)]
    [InlineData(5.0, 5.0, Out)]
    [InlineData(11.0, 2.0, Out)]
    [InlineData(0.0, 5.0, On)]
    [InlineData(4.0, 5.0, On)]
    [InlineData(10.0, 10.0, On)]
    public void FClass2d_on_holed_square(double u, double v, Containment theExpected)
    {
        var aClass = new FClass2d(Holed(), 1e-7);
        Assert.Equal(theExpected, aClass.Perform(new Point2D(u, v)));
        Assert.Equal(theExpected, aClass.Perform(new Point2D(u, v), false));
        Assert.Equal(Out, aClass.PerformInfinitePoint());
    }

    [Fact]
    public void FClass2d_test_on_restriction()
    {
        var aClass = new FClass2d(Holed(), 1e-7);
        Assert.Equal(On, aClass.TestOnRestriction(new Point2D(1e-4, 5), 1e-3));
        Assert.Equal(In, aClass.TestOnRestriction(new Point2D(2, 2), 1e-3));
        Assert.Equal(Out, aClass.TestOnRestriction(new Point2D(5, 5), 1e-3));
    }

    [Fact]
    public void FClass2d_on_periodic_cylinder_brings_points_into_the_period()
    {
        Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        var aClass = new FClass2d(aFace, 1e-7);
        Assert.Equal(In, aClass.Perform(new Point2D(Math.PI, 1.0)));
        // a period further: IN when brought back, OUT when not
        Assert.Equal(In, aClass.Perform(new Point2D(Math.PI + 2.0 * Math.PI, 1.0)));
        Assert.Equal(In, aClass.Perform(new Point2D(Math.PI - 4.0 * Math.PI, 1.0)));
        Assert.Equal(Out, aClass.Perform(new Point2D(Math.PI + 2.0 * Math.PI, 1.0), false));
        Assert.Equal(Out, aClass.Perform(new Point2D(Math.PI, 5.0)));
        Assert.Equal(On, aClass.Perform(new Point2D(Math.PI, 3.0)));
    }

    [Fact]
    public void FClass2d_natural_face_is_all_in()
    {
        Face aFace = TestShapes.Frozen(BRepGPropTestShapes.NaturalFace(
            new SphericalSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1.0)));
        var aClass = new FClass2d(aFace, 1e-7);
        Assert.Equal(In, aClass.Perform(new Point2D(100, 100)));
    }

    [Fact]
    public void TopolTool_explores_the_face()
    {
        Face aFace = Holed();
        var aSurf = new FaceSurfaceAdaptor(aFace);
        var aTool = new BrepTopolTool(aSurf);
        Assert.True(aTool.Has3d());
        Assert.False(aTool.DomainIsInfinite());

        int aNbArcs = 0;
        for (aTool.Init(); aTool.More(); aTool.Next())
        {
            Curve2DAdaptorBase anArc = aTool.Value();
            Assert.IsType<EdgeCurve2DAdaptor>(anArc);
            Assert.Equal(Forward, aTool.Orientation(anArc));
            Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aTool.Tol3d(anArc));
            Assert.IsType<Edge>(aTool.Edge());

            // the vertices of the arc
            aTool.Initialize(anArc);
            int aNbVertices = 0;
            for (aTool.InitVertexIterator(); aTool.MoreVertex(); aTool.NextVertex())
            {
                HVertex aV = aTool.Vertex();
                Assert.IsType<TopologyHVertex>(aV);
                Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aTool.Tol3d(aV));
                Point3D aP = aTool.Pnt(aV);
                Assert.Equal(0.0, aP.Z, 1e-12);
                double aParam = aV.Parameter(anArc);
                Assert.True(aParam >= anArc.FirstParameter() - 1e-9 && aParam <= anArc.LastParameter() + 1e-9);
                Assert.True(aV.Resolution(anArc) > 0.0);
                Assert.True(aV.IsSame(aV));
                Assert.Equal(RealFirst(), aV.Value().X);
                aNbVertices++;
            }

            Assert.Equal(2, aNbVertices);
            aNbArcs++;
        }

        Assert.Equal(8, aNbArcs);
        Assert.Equal(In, aTool.Classify(new Point2D(2, 2), 1e-7));
        Assert.Equal(Out, aTool.Classify(new Point2D(5, 5), 1e-7));
        Assert.True(aTool.IsThePointOn(new Point2D(0, 5), 1e-7));
        Assert.False(aTool.IsThePointOn(new Point2D(2, 2), 1e-7));

        // the samples of a plane: at least 10 by 10
        Assert.Equal(10, aTool.NbSamplesU());
        Assert.Equal(10, aTool.NbSamplesV());
        Assert.Equal(100, aTool.NbSamples());
        aTool.SamplePoint(0, out Point2D aP2d, out Point3D aP3d);
        Assert.Equal(aP2d.X, aP3d.X, 1e-12);
        Assert.Equal(aP2d.Y, aP3d.Y, 1e-12);
    }

    [Fact]
    public void TopolTool_needs_brep_adaptors()
    {
        var aTool = new BrepTopolTool();
        var aGeomSurf = new SurfaceAdaptor(new PlaneSurface(new CoordinateSystem()));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => aTool.Initialize(aGeomSurf));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => aTool.Initialize(new CurveAdaptor2D(new LineCurve2D(new Point2D(), new Direction2D()))));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aTool.Initialize());
        Assert.Equal(Unknown, aTool.Classify(new Point2D(), 1e-7));
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
        var aSurface = new global::Kerf.Geometry.BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 5, 5);
        Face aFace = TestShapes.Frozen(BRepGPropTestShapes.NaturalFace(aSurface));
        var aTool = new BrepTopolTool(new FaceSurfaceAdaptor(aFace));
        Assert.Equal(SurfaceKind.BSplineSurface, new FaceSurfaceAdaptor(aFace).GetType());
        Assert.True(aTool.NbSamplesU() >= 10);
        Assert.True(aTool.NbSamplesV() >= 10);
    }

    [Fact]
    public void Tool_keeps_the_surface_and_its_topological_tool()
    {
        Face aFace = Holed();
        var aTool = new AdaptorTool(aFace, 1e-7);
        Assert.IsType<FaceSurfaceAdaptor>(aTool.GetSurface());
        Assert.Equal(In, aTool.GetTopolTool().Classify(new Point2D(2, 2), 1e-7));

        var anEmpty = new AdaptorTool();
        Assert.Null(anEmpty.GetSurface());
        anEmpty.Init(aFace, 1e-7);
        Assert.NotNull(anEmpty.GetSurface());
        var anOther = new BrepTopolTool();
        anEmpty.SetTopolTool(anOther);
        Assert.Same(anOther, anEmpty.GetTopolTool());
    }
}
