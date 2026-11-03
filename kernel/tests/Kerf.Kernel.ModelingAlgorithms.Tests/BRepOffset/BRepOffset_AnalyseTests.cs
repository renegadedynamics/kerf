// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Offsets;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Fillets.Data.TypeOfConcavity;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BRepOffset_Analyse and ChFi3d's package functions (unit 7E) on primitives and Boolean results; the native
/// comparison over many more shapes is <see cref="BRepOffsetNativeTests"/>.
/// </summary>
public class BRepOffset_AnalyseTests
{
    private static List<Shape> SubShapes(in Shape theShape, ShapeType theType)
    {
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return [.. aMap];
    }

    private static Dictionary<TypeOfConcavity, int> Types(Analyse theA, in Shape theShape)
    {
        var aCounts = new Dictionary<TypeOfConcavity, int>();
        foreach (Shape anE in SubShapes(theShape, ShapeType.Edge))
        {
            foreach (Interval anI in theA.Type(ShapeCasts.Edge(anE)))
            {
                aCounts[anI.Type()] = aCounts.GetValueOrDefault(anI.Type()) + 1;
            }
        }
        return aCounts;
    }

    // A stadium (two half discs joined by straight sides) extruded: planes tangent to half cylinders
    private static Shape Stadium()
    {
        Edge e1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        Edge e2 = new MakeEdge(new MakeArcOfCircle(new Point3D(10, 0, 0), new Point3D(15, 5, 0), new Point3D(10, 10, 0)).Value()).Edge();
        Edge e3 = new MakeEdge(new Point3D(10, 10, 0), new Point3D(0, 10, 0)).Edge();
        Edge e4 = new MakeEdge(new MakeArcOfCircle(new Point3D(0, 10, 0), new Point3D(-5, 5, 0), new Point3D(0, 0, 0)).Value()).Edge();
        Face aF = new MakeFace(new MakeWire(e1, e2, e3, e4).Wire()).Face();
        return new ApiMakePrism(aF, new Vector3D(0, 0, 4)).Shape();
    }

    [Fact]
    public void Box_HasTwelveConvexEdges_AndExplodesIntoItsFaces()
    {
        Shape aBox = new MakeBox(10, 20, 30).Shape();
        var anA = new Analyse(aBox, 0.01);
        Assert.True(anA.IsDone);
        Dictionary<TypeOfConcavity, int> aTypes = Types(anA, aBox);
        Assert.Equal(12, aTypes[TypeOfConcavity.Convex]);
        Assert.Single(aTypes);

        var aL = new NCollection_List<Shape>();
        anA.Explode(aL, TypeOfConcavity.Tangential);
        Assert.Equal(6, aL.Extent());
        anA.Explode(aL, TypeOfConcavity.Convex);
        Assert.Equal(1, aL.Extent());

        Vertex aV = ShapeCasts.Vertex(SubShapes(aBox, ShapeType.Vertex)[0]);
        Assert.Equal(3, anA.Ancestors(aV).Extent());
        anA.Edges(aV, TypeOfConcavity.Convex, aL);
        Assert.Equal(3, aL.Extent());
        Assert.Empty(anA.NewFaces());
    }

    [Fact]
    public void LShape_HasOneConcaveEdge()
    {
        Shape aB1 = new MakeBox(new Point3D(0, 0, 0), new Point3D(10, 10, 10)).Shape();
        Shape aB2 = new MakeBox(new Point3D(0, 0, 0), new Point3D(10, 20, 5)).Shape();
        Shape aL = new Fuse(aB1, aB2).Shape();
        var aU = new UnifySameDomain(aL, true, true, true);
        aU.Build();
        Shape aShape = aU.Shape();
        var anA = new Analyse(aShape, 0.01);
        Dictionary<TypeOfConcavity, int> aTypes = Types(anA, aShape);
        Assert.Equal(1, aTypes[TypeOfConcavity.Concave]);
        Edge aConcave = ShapeCasts.Edge(SubShapes(aShape, ShapeType.Edge).Single(e => anA.Type(ShapeCasts.Edge(e)).First().Type() == TypeOfConcavity.Concave));
        var aC = new EdgeCurveAdaptor(aConcave);
        Point3D aMid = aC.Value(0.5 * (aC.FirstParameter() + aC.LastParameter()));
        Assert.Equal(10.0, aMid.Y(), 9);
        Assert.Equal(5.0, aMid.Z(), 9);
    }

    [Fact]
    public void Stadium_HasTangentEdges_AndChFi3dAgrees()
    {
        Shape aS = Stadium();
        var anA = new Analyse(aS, 0.01);
        Dictionary<TypeOfConcavity, int> aTypes = Types(anA, aS);
        Assert.Equal(4, aTypes[TypeOfConcavity.Tangential]); // the four vertical edges between sides and half cylinders
        Assert.Equal(8, aTypes[TypeOfConcavity.Convex]);     // top and bottom boundaries
        var aL = new NCollection_List<Shape>();
        anA.Explode(aL, TypeOfConcavity.Tangential);
        Assert.Equal(3, aL.Extent()); // the side band, the top and the bottom

        foreach (Shape anE in SubShapes(aS, ShapeType.Edge))
        {
            NCollection_List<Shape> aFaces = anA.Ancestors(anE);
            Face aF1 = ShapeCasts.Face(aFaces.First()), aF2 = ShapeCasts.Face(aFaces.Last());
            bool isTangent = anA.Type(ShapeCasts.Edge(anE)).First().Type() == TypeOfConcavity.Tangential;
            Assert.Equal(isTangent, global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(ShapeCasts.Edge(anE), aF1, aF2));
            if (!isTangent)
            {
                Assert.Equal(TypeOfConcavity.Convex, global::Kerf.Engine.Fillets.ChFi3d.DefineConnectType(ShapeCasts.Edge(anE), aF1, aF2, Math.Sin(0.01), false));
            }
        }
    }

    [Fact]
    public void TangentFaces_WithDifferentOffsets_OnAFrozenShape_LeaveItUnchanged()
    {
        Shape aS = Stadium();
        Assert.True(aS.TShape()!.IsFrozen);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aS);
        var aMap = new NCollection_DataMap<Shape, double>(ShapeHasher.Instance);
        foreach (Shape aF in SubShapes(aS, ShapeType.Face))
        {
            if (Tool.Surface(ShapeCasts.Face(aF)) is CylindricalSurface)
            {
                aMap.Bind(aF, 2.0);
            }
        }
        var anA = new Analyse();
        anA.SetOffsetValue(1.0);
        anA.SetFaceOffsetMap(aMap);
        anA.Perform(aS, 0.01);
        aSnap.AssertUnchanged(aS);
        Assert.True(anA.IsDone);
        Assert.Equal(4, anA.NewFaces().Extent());
        foreach (Shape anE in SubShapes(aS, ShapeType.Edge))
        {
            if (anA.HasGenerated(anE))
            {
                Assert.Equal(ShapeType.Face, anA.Generated(anE).ShapeType());
            }
        }
    }

    [Fact]
    public void ConcaveSide_OnABoxEdge()
    {
        Shape aBox = new MakeBox(10, 20, 30).Shape();
        var anA = new Analyse(aBox, 0.01);
        Edge anE = ShapeCasts.Edge(SubShapes(aBox, ShapeType.Edge)[0]);
        NCollection_List<Shape> aFaces = anA.Ancestors(anE);
        int aChoice = global::Kerf.Engine.Fillets.ChFi3d.ConcaveSide(new FaceSurfaceAdaptor(ShapeCasts.Face(aFaces.First())),
                                         new FaceSurfaceAdaptor(ShapeCasts.Face(aFaces.Last())), anE,
                                         out Orientation anOr1, out Orientation anOr2);
        // a convex edge: the fillet lies on the side opposite to both outward normals
        Assert.Equal(Orientation.Reversed, anOr1);
        Assert.Equal(Orientation.Reversed, anOr2);
        Assert.True(aChoice is 5 or 6);
        Orientation o1 = Orientation.Forward, o2 = Orientation.Reversed;
        Assert.Equal(3 + (aChoice % 2 == 0 ? 1 : 0), global::Kerf.Engine.Fillets.ChFi3d.NextSide(ref o1, ref o2, anOr1, anOr2, aChoice));
        Assert.Equal(Orientation.Reversed, o1);
        Assert.Equal(Orientation.Forward, o2);
        Assert.True(global::Kerf.Engine.Fillets.ChFi3d.SameSide(Orientation.Forward, anOr1, anOr2, Orientation.Forward,
                                    Orientation.Forward));
    }
}
