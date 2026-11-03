// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingData.Tests;

/// <summary>BRepLProp's continuity, BRepLProp_CLProps, BRepLProp_SLProps and BRepLProp_SurfaceTool.</summary>
public class BRepLPropTests
{
    private static CircleCurve3D Circle(double theR) => new(new Frame3D(new Point3D(1, 1, 0), new Direction3D(0, 0, 1)), theR);

    [Fact]
    public void CLProps_OnACircleEdge()
    {
        Edge anE = TestShapes.Frozen(TestShapes.Edge(Circle(2.0), 0.0, Math.PI));
        var aC = new EdgeCurveAdaptor(anE);
        var aProps = new global::Kerf.Engine.Topology.Properties.CLProps(aC, 0.5 * Math.PI, 2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aProps.Value().Distance(new Point3D(1, 3, 0)) < 1e-14);
        Assert.Equal(0.5, aProps.Curvature(), 1e-14);
        aProps.CentreOfCurvature(out Point3D aCentre);
        Assert.True(aCentre.Distance(new Point3D(1, 1, 0)) < 1e-14);
        Assert.True(aProps.IsTangentDefined());
        aProps.Tangent(out Direction3D aT);
        Assert.True(aT.IsEqual(new Direction3D(-1, 0, 0), 1e-14));

        // the properties keep their own copy of the adaptor
        aC.Initialize(TestShapes.Frozen(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0))));
        aProps.SetParameter(0.0);
        Assert.True(aProps.Value().Distance(new Point3D(3, 1, 0)) < 1e-14);

        var aLater = new global::Kerf.Engine.Topology.Properties.CLProps(1, global::Kerf.Engine.Foundation.Precision.Confusion());
        aLater.SetCurve(aC);
        aLater.SetParameter(0.25);
        Assert.Equal(new Point3D(0.25, 0, 0), aLater.Value());
    }

    [Fact]
    public void SLProps_OnACylinderFace()
    {
        Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        var aS = new FaceSurfaceAdaptor(aFace);
        var aProps = new global::Kerf.Engine.Topology.Properties.SLProps(aS, 1.0, 1.5, 2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aProps.IsCurvatureDefined());
        Assert.Equal(0.5, Math.Abs(aProps.MaxCurvature()) + Math.Abs(aProps.MinCurvature()), 1e-14);
        Assert.Equal(0.0, aProps.GaussianCurvature(), 1e-14);
        Assert.True(aProps.IsNormalDefined);

        var aLater = new global::Kerf.Engine.Topology.Properties.SLProps(1, global::Kerf.Engine.Foundation.Precision.Confusion());
        aLater.SetSurface(aS);
        aLater.SetParameters(1.0, 1.5);
        Assert.Equal(aProps.Value(), aLater.Value());
        Assert.Equal(aProps.D1U(), aLater.D1U());
    }

    [Fact]
    public void SurfaceTool()
    {
        Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        var aS = new FaceSurfaceAdaptor(aFace);
        Assert.Equal(3, global::Kerf.Engine.Topology.Properties.SurfaceTool.Continuity(aS));
        global::Kerf.Engine.Topology.Properties.SurfaceTool.Bounds(aS, out double U1, out double V1, out double U2, out double V2);
        Assert.Equal(0.0, U1);
        Assert.Equal(0.0, V1);
        Assert.Equal(2 * Math.PI, U2, 1e-15);
        Assert.Equal(3.0, V2);
        global::Kerf.Engine.Topology.Properties.SurfaceTool.Value(aS, 0.5, 1.0, out Point3D aP);
        global::Kerf.Engine.Topology.Properties.SurfaceTool.D2(aS, 0.5, 1.0, out Point3D aP2, out Vector3D aD1U, out _, out _, out _, out _);
        global::Kerf.Engine.Topology.Properties.SurfaceTool.D1(aS, 0.5, 1.0, out _, out Vector3D aD1U1, out _);
        Assert.Equal(aP, aP2);
        Assert.Equal(aD1U, aD1U1);
        Assert.Equal(aD1U, global::Kerf.Engine.Topology.Properties.SurfaceTool.DN(aS, 0.5, 1.0, 1, 0));
    }

    [Fact]
    public void Continuity_OfJunctions()
    {
        // two halves of a circle: C2 at their junction
        CircleCurve3D aCircle = Circle(1.0);
        Vertex aV0 = TestShapes.Vertex(aCircle.Value(0.0));
        Vertex aV1 = TestShapes.Vertex(aCircle.Value(Math.PI));
        Edge aHalf1 = TestShapes.Frozen(TestShapes.Edge(aCircle, 0.0, Math.PI, aV0, aV1));
        Edge aHalf2 = TestShapes.Frozen(TestShapes.Edge(aCircle, Math.PI, 2 * Math.PI, aV1, aV0));
        var aC1 = new EdgeCurveAdaptor(aHalf1);
        var aC2 = new EdgeCurveAdaptor(aHalf2);
        Assert.Equal(C2, global::Kerf.Engine.Topology.Properties.BRepLProp.Continuity(aC1, aC2, Math.PI, Math.PI));

        // a full circle with itself across its seam: CN
        Edge aFull = TestShapes.Frozen(TestShapes.Edge(aCircle, 0.0, 2 * Math.PI));
        var aF = new EdgeCurveAdaptor(aFull);
        Assert.Equal(CN, global::Kerf.Engine.Topology.Properties.BRepLProp.Continuity(aF, aF, 2 * Math.PI, 0.0));

        // a corner of a box: C0
        Shape aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Wire aW = ShapeCasts.Wire(new Explorer(aBox, ShapeType.Wire).Current());
        var anExp = new WireExplorer(aW);
        Edge anE1 = anExp.Current();
        anExp.Next();
        Edge anE2 = anExp.Current();
        var aB1 = new EdgeCurveAdaptor(anE1);
        var aB2 = new EdgeCurveAdaptor(anE2);
        Tool.Range(anE1, out double f1, out double l1);
        Tool.Range(anE2, out double f2, out double l2);
        double u1 = anE1.Orientation() == Reversed ? f1 : l1;
        double u2 = anE2.Orientation() == Reversed ? l2 : f2;
        Assert.Equal(C0, global::Kerf.Engine.Topology.Properties.BRepLProp.Continuity(aB1, aB2, u1, u2));

        // points that don't coincide
        Assert.Throws<global::Kerf.Engine.Foundation.FailureException>(() => global::Kerf.Engine.Topology.Properties.BRepLProp.Continuity(aB1, aB2, u1 == f1 ? l1 : f1, u2));
    }
}
