// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using OcctNet.ModelingData.Tests;
using static Occt.ChFiDS_TypeOfConcavity;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepOffset_Analyse and ChFi3d's package functions (unit 7E) on primitives and Boolean results; the native
/// comparison over many more shapes is <see cref="BRepOffsetNativeTests"/>.
/// </summary>
public class BRepOffset_AnalyseTests
{
    private static List<TopoDS_Shape> SubShapes(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return [.. aMap];
    }

    private static Dictionary<ChFiDS_TypeOfConcavity, int> Types(BRepOffset_Analyse theA, in TopoDS_Shape theShape)
    {
        var aCounts = new Dictionary<ChFiDS_TypeOfConcavity, int>();
        foreach (TopoDS_Shape anE in SubShapes(theShape, TopAbs_EDGE))
        {
            foreach (BRepOffset_Interval anI in theA.Type(TopoDS.Edge(anE)))
            {
                aCounts[anI.Type()] = aCounts.GetValueOrDefault(anI.Type()) + 1;
            }
        }
        return aCounts;
    }

    // A stadium (two half discs joined by straight sides) extruded: planes tangent to half cylinders
    private static TopoDS_Shape Stadium()
    {
        TopoDS_Edge e1 = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        TopoDS_Edge e2 = new BRepBuilderAPI_MakeEdge(new GC_MakeArcOfCircle(new Point3D(10, 0, 0), new Point3D(15, 5, 0), new Point3D(10, 10, 0)).Value()).Edge();
        TopoDS_Edge e3 = new BRepBuilderAPI_MakeEdge(new Point3D(10, 10, 0), new Point3D(0, 10, 0)).Edge();
        TopoDS_Edge e4 = new BRepBuilderAPI_MakeEdge(new GC_MakeArcOfCircle(new Point3D(0, 10, 0), new Point3D(-5, 5, 0), new Point3D(0, 0, 0)).Value()).Edge();
        TopoDS_Face aF = new BRepBuilderAPI_MakeFace(new BRepBuilderAPI_MakeWire(e1, e2, e3, e4).Wire()).Face();
        return new BRepPrimAPI_MakePrism(aF, new Vector3D(0, 0, 4)).Shape();
    }

    [Fact]
    public void Box_HasTwelveConvexEdges_AndExplodesIntoItsFaces()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        var anA = new BRepOffset_Analyse(aBox, 0.01);
        Assert.True(anA.IsDone());
        Dictionary<ChFiDS_TypeOfConcavity, int> aTypes = Types(anA, aBox);
        Assert.Equal(12, aTypes[ChFiDS_Convex]);
        Assert.Single(aTypes);

        var aL = new NCollection_List<TopoDS_Shape>();
        anA.Explode(aL, ChFiDS_Tangential);
        Assert.Equal(6, aL.Extent());
        anA.Explode(aL, ChFiDS_Convex);
        Assert.Equal(1, aL.Extent());

        TopoDS_Vertex aV = TopoDS.Vertex(SubShapes(aBox, TopAbs_VERTEX)[0]);
        Assert.Equal(3, anA.Ancestors(aV).Extent());
        anA.Edges(aV, ChFiDS_Convex, aL);
        Assert.Equal(3, aL.Extent());
        Assert.Empty(anA.NewFaces());
    }

    [Fact]
    public void LShape_HasOneConcaveEdge()
    {
        TopoDS_Shape aB1 = new BRepPrimAPI_MakeBox(new Point3D(0, 0, 0), new Point3D(10, 10, 10)).Shape();
        TopoDS_Shape aB2 = new BRepPrimAPI_MakeBox(new Point3D(0, 0, 0), new Point3D(10, 20, 5)).Shape();
        TopoDS_Shape aL = new BRepAlgoAPI_Fuse(aB1, aB2).Shape();
        var aU = new ShapeUpgrade_UnifySameDomain(aL, true, true, true);
        aU.Build();
        TopoDS_Shape aShape = aU.Shape();
        var anA = new BRepOffset_Analyse(aShape, 0.01);
        Dictionary<ChFiDS_TypeOfConcavity, int> aTypes = Types(anA, aShape);
        Assert.Equal(1, aTypes[ChFiDS_Concave]);
        TopoDS_Edge aConcave = TopoDS.Edge(SubShapes(aShape, TopAbs_EDGE).Single(e => anA.Type(TopoDS.Edge(e)).First().Type() == ChFiDS_Concave));
        var aC = new BRepAdaptor_Curve(aConcave);
        Point3D aMid = aC.Value(0.5 * (aC.FirstParameter() + aC.LastParameter()));
        Assert.Equal(10.0, aMid.Y(), 9);
        Assert.Equal(5.0, aMid.Z(), 9);
    }

    [Fact]
    public void Stadium_HasTangentEdges_AndChFi3dAgrees()
    {
        TopoDS_Shape aS = Stadium();
        var anA = new BRepOffset_Analyse(aS, 0.01);
        Dictionary<ChFiDS_TypeOfConcavity, int> aTypes = Types(anA, aS);
        Assert.Equal(4, aTypes[ChFiDS_Tangential]); // the four vertical edges between sides and half cylinders
        Assert.Equal(8, aTypes[ChFiDS_Convex]);     // top and bottom boundaries
        var aL = new NCollection_List<TopoDS_Shape>();
        anA.Explode(aL, ChFiDS_Tangential);
        Assert.Equal(3, aL.Extent()); // the side band, the top and the bottom

        foreach (TopoDS_Shape anE in SubShapes(aS, TopAbs_EDGE))
        {
            NCollection_List<TopoDS_Shape> aFaces = anA.Ancestors(anE);
            TopoDS_Face aF1 = TopoDS.Face(aFaces.First()), aF2 = TopoDS.Face(aFaces.Last());
            bool isTangent = anA.Type(TopoDS.Edge(anE)).First().Type() == ChFiDS_Tangential;
            Assert.Equal(isTangent, ChFi3d.IsTangentFaces(TopoDS.Edge(anE), aF1, aF2));
            if (!isTangent)
            {
                Assert.Equal(ChFiDS_Convex, ChFi3d.DefineConnectType(TopoDS.Edge(anE), aF1, aF2, Math.Sin(0.01), false));
            }
        }
    }

    [Fact]
    public void TangentFaces_WithDifferentOffsets_OnAFrozenShape_LeaveItUnchanged()
    {
        TopoDS_Shape aS = Stadium();
        Assert.True(aS.TShape()!.IsFrozen);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aS);
        var aMap = new NCollection_DataMap<TopoDS_Shape, double>(TopTools_ShapeMapHasher.Instance);
        foreach (TopoDS_Shape aF in SubShapes(aS, TopAbs_FACE))
        {
            if (BRep_Tool.Surface(TopoDS.Face(aF)) is Geom_CylindricalSurface)
            {
                aMap.Bind(aF, 2.0);
            }
        }
        var anA = new BRepOffset_Analyse();
        anA.SetOffsetValue(1.0);
        anA.SetFaceOffsetMap(aMap);
        anA.Perform(aS, 0.01);
        aSnap.AssertUnchanged(aS);
        Assert.True(anA.IsDone());
        Assert.Equal(4, anA.NewFaces().Extent());
        foreach (TopoDS_Shape anE in SubShapes(aS, TopAbs_EDGE))
        {
            if (anA.HasGenerated(anE))
            {
                Assert.Equal(TopAbs_FACE, anA.Generated(anE).ShapeType());
            }
        }
    }

    [Fact]
    public void ConcaveSide_OnABoxEdge()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        var anA = new BRepOffset_Analyse(aBox, 0.01);
        TopoDS_Edge anE = TopoDS.Edge(SubShapes(aBox, TopAbs_EDGE)[0]);
        NCollection_List<TopoDS_Shape> aFaces = anA.Ancestors(anE);
        int aChoice = ChFi3d.ConcaveSide(new BRepAdaptor_Surface(TopoDS.Face(aFaces.First())),
                                         new BRepAdaptor_Surface(TopoDS.Face(aFaces.Last())), anE,
                                         out TopAbs_Orientation anOr1, out TopAbs_Orientation anOr2);
        // a convex edge: the fillet lies on the side opposite to both outward normals
        Assert.Equal(TopAbs_Orientation.TopAbs_REVERSED, anOr1);
        Assert.Equal(TopAbs_Orientation.TopAbs_REVERSED, anOr2);
        Assert.True(aChoice is 5 or 6);
        TopAbs_Orientation o1 = TopAbs_Orientation.TopAbs_FORWARD, o2 = TopAbs_Orientation.TopAbs_REVERSED;
        Assert.Equal(3 + (aChoice % 2 == 0 ? 1 : 0), ChFi3d.NextSide(ref o1, ref o2, anOr1, anOr2, aChoice));
        Assert.Equal(TopAbs_Orientation.TopAbs_REVERSED, o1);
        Assert.Equal(TopAbs_Orientation.TopAbs_FORWARD, o2);
        Assert.True(ChFi3d.SameSide(TopAbs_Orientation.TopAbs_FORWARD, anOr1, anOr2, TopAbs_Orientation.TopAbs_FORWARD,
                                    TopAbs_Orientation.TopAbs_FORWARD));
    }
}
