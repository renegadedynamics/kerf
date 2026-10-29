// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>BRepLProp's continuity, BRepLProp_CLProps, BRepLProp_SLProps and BRepLProp_SurfaceTool.</summary>
public class BRepLPropTests
{
    private static Geom_Circle Circle(double theR) => new(new gp_Ax2(new gp_Pnt(1, 1, 0), new gp_Dir(0, 0, 1)), theR);

    [Fact]
    public void CLProps_OnACircleEdge()
    {
        TopoDS_Edge anE = TestShapes.Frozen(TestShapes.Edge(Circle(2.0), 0.0, Math.PI));
        var aC = new BRepAdaptor_Curve(anE);
        var aProps = new BRepLProp_CLProps(aC, 0.5 * Math.PI, 2, Precision.Confusion());
        Assert.True(aProps.Value().Distance(new gp_Pnt(1, 3, 0)) < 1e-14);
        Assert.Equal(0.5, aProps.Curvature(), 1e-14);
        aProps.CentreOfCurvature(out gp_Pnt aCentre);
        Assert.True(aCentre.Distance(new gp_Pnt(1, 1, 0)) < 1e-14);
        Assert.True(aProps.IsTangentDefined());
        aProps.Tangent(out gp_Dir aT);
        Assert.True(aT.IsEqual(new gp_Dir(-1, 0, 0), 1e-14));

        // the properties keep their own copy of the adaptor
        aC.Initialize(TestShapes.Frozen(TestShapes.Edge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0))));
        aProps.SetParameter(0.0);
        Assert.True(aProps.Value().Distance(new gp_Pnt(3, 1, 0)) < 1e-14);

        var aLater = new BRepLProp_CLProps(1, Precision.Confusion());
        aLater.SetCurve(aC);
        aLater.SetParameter(0.25);
        Assert.Equal(new gp_Pnt(0.25, 0, 0), aLater.Value());
    }

    [Fact]
    public void SLProps_OnACylinderFace()
    {
        TopoDS_Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        var aS = new BRepAdaptor_Surface(aFace);
        var aProps = new BRepLProp_SLProps(aS, 1.0, 1.5, 2, Precision.Confusion());
        Assert.True(aProps.IsCurvatureDefined());
        Assert.Equal(0.5, Math.Abs(aProps.MaxCurvature()) + Math.Abs(aProps.MinCurvature()), 1e-14);
        Assert.Equal(0.0, aProps.GaussianCurvature(), 1e-14);
        Assert.True(aProps.IsNormalDefined());

        var aLater = new BRepLProp_SLProps(1, Precision.Confusion());
        aLater.SetSurface(aS);
        aLater.SetParameters(1.0, 1.5);
        Assert.Equal(aProps.Value(), aLater.Value());
        Assert.Equal(aProps.D1U(), aLater.D1U());
    }

    [Fact]
    public void SurfaceTool()
    {
        TopoDS_Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        var aS = new BRepAdaptor_Surface(aFace);
        Assert.Equal(3, BRepLProp_SurfaceTool.Continuity(aS));
        BRepLProp_SurfaceTool.Bounds(aS, out double U1, out double V1, out double U2, out double V2);
        Assert.Equal(0.0, U1);
        Assert.Equal(0.0, V1);
        Assert.Equal(2 * Math.PI, U2, 1e-15);
        Assert.Equal(3.0, V2);
        BRepLProp_SurfaceTool.Value(aS, 0.5, 1.0, out gp_Pnt aP);
        BRepLProp_SurfaceTool.D2(aS, 0.5, 1.0, out gp_Pnt aP2, out gp_Vec aD1U, out _, out _, out _, out _);
        BRepLProp_SurfaceTool.D1(aS, 0.5, 1.0, out _, out gp_Vec aD1U1, out _);
        Assert.Equal(aP, aP2);
        Assert.Equal(aD1U, aD1U1);
        Assert.Equal(aD1U, BRepLProp_SurfaceTool.DN(aS, 0.5, 1.0, 1, 0));
    }

    [Fact]
    public void Continuity_OfJunctions()
    {
        // two halves of a circle: C2 at their junction
        Geom_Circle aCircle = Circle(1.0);
        TopoDS_Vertex aV0 = TestShapes.Vertex(aCircle.Value(0.0));
        TopoDS_Vertex aV1 = TestShapes.Vertex(aCircle.Value(Math.PI));
        TopoDS_Edge aHalf1 = TestShapes.Frozen(TestShapes.Edge(aCircle, 0.0, Math.PI, aV0, aV1));
        TopoDS_Edge aHalf2 = TestShapes.Frozen(TestShapes.Edge(aCircle, Math.PI, 2 * Math.PI, aV1, aV0));
        var aC1 = new BRepAdaptor_Curve(aHalf1);
        var aC2 = new BRepAdaptor_Curve(aHalf2);
        Assert.Equal(GeomAbs_C2, BRepLProp.Continuity(aC1, aC2, Math.PI, Math.PI));

        // a full circle with itself across its seam: CN
        TopoDS_Edge aFull = TestShapes.Frozen(TestShapes.Edge(aCircle, 0.0, 2 * Math.PI));
        var aF = new BRepAdaptor_Curve(aFull);
        Assert.Equal(GeomAbs_CN, BRepLProp.Continuity(aF, aF, 2 * Math.PI, 0.0));

        // a corner of a box: C0
        TopoDS_Shape aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Wire aW = TopoDS.Wire(new TopExp_Explorer(aBox, TopAbs_WIRE).Current());
        var anExp = new BRepTools_WireExplorer(aW);
        TopoDS_Edge anE1 = anExp.Current();
        anExp.Next();
        TopoDS_Edge anE2 = anExp.Current();
        var aB1 = new BRepAdaptor_Curve(anE1);
        var aB2 = new BRepAdaptor_Curve(anE2);
        BRep_Tool.Range(anE1, out double f1, out double l1);
        BRep_Tool.Range(anE2, out double f2, out double l2);
        double u1 = anE1.Orientation() == TopAbs_REVERSED ? f1 : l1;
        double u2 = anE2.Orientation() == TopAbs_REVERSED ? l2 : f2;
        Assert.Equal(GeomAbs_C0, BRepLProp.Continuity(aB1, aB2, u1, u2));

        // points that don't coincide
        Assert.Throws<Standard_Failure>(() => BRepLProp.Continuity(aB1, aB2, u1 == f1 ? l1 : f1, u2));
    }
}
