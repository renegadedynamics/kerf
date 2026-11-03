// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Shapes for the BinTools tests, built with BRep_Builder: every representation the binary formats store (points on
/// curves, on surfaces and on curves on surfaces; 3D curves; curves on surfaces and on closed surfaces; regularity;
/// 3D polygons; polygons on triangulations, closed or not; triangulations with or without UV nodes and normals;
/// faces with only a triangulation), elementary and complex locations, and every shape type. Unfrozen.
/// </summary>
internal static class BinTestShapes
{
    private static readonly BRep_Builder theBuilder = new();

    /// <summary>The shapes of the round-trip tests, with names.</summary>
    public static IEnumerable<(string Name, TopoDS_Shape Shape)> All()
    {
        yield return ("vertex", TestShapes.Vertex(new Point3D(1, 2, 3)));
        yield return ("edge", TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 1, 1)));
        yield return ("box", TestShapes.Box(1, 2, 3));
        yield return ("cylinder face", TestShapes.CylinderFace(2, 5));
        yield return ("meshed box", MeshedBox(false, false));
        yield return ("meshed box with UV and normals", MeshedBox(true, true));
        yield return ("triangulation-only face", TriangulationOnlyFace());
        yield return ("meshed cylinder", MeshedCylinder());
        yield return ("vertex representations", VertexRepresentations(includePointOnSurface: true));
        yield return ("compound with locations", CompoundWithLocations());
        yield return ("compsolid", CompSolid());
    }

    /// <summary>A box whose first edge is G1 between its two faces, with a 3D polygon on its second edge.</summary>
    public static TopoDS_Shape BoxWithRegularityAndPolygon()
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 2, 3);
        TopoDS_Edge anEdge = FirstEdge(aBox);
        var aFaces = FacesOf(aBox, anEdge);
        theBuilder.Continuity(anEdge, aFaces[0], aFaces[1], Continuity.G1);

        TopoDS_Edge aSecond = NthEdge(aBox, 2);
        var aNodes = new NCollection_Array1<Point3D>(1, 3);
        aNodes[1] = new Point3D(0, 0, 0);
        aNodes[2] = new Point3D(0.5, 0, 0);
        aNodes[3] = new Point3D(1, 0, 0);
        var aParams = new NCollection_Array1<double>(1, 3);
        aParams[1] = 0;
        aParams[2] = 0.5;
        aParams[3] = 1;
        var aPolygon = new Polygon3D(aNodes, aParams);
        aPolygon.Deflection(0.01);
        theBuilder.UpdateEdge(aSecond, aPolygon);

        TopoDS_Edge aThird = NthEdge(aBox, 3);
        theBuilder.UpdateEdge(aThird, new Polygon3D(aNodes));
        return aBox;
    }

    /// <summary>
    /// A box with a vertex holding a point on a curve, a point on a curve on a surface and (if asked) a point on a
    /// surface.
    /// </summary>
    public static TopoDS_Shape VertexRepresentations(bool includePointOnSurface)
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 2, 3);
        TopoDS_Edge anEdge = FirstEdge(aBox);
        TopoDS_Face aFace = FacesOf(aBox, anEdge)[0];
        TopoDS_Vertex aVertex = TopoDS.Vertex(new TopExp_Explorer(anEdge, TopAbs_VERTEX).Current());
        TopoDS_Vertex aFree = TestShapes.Vertex(new Point3D(0.25, 0.5, 0.75));
        theBuilder.UpdateVertex(aFree, 0.25, anEdge, global::Kerf.Engine.Foundation.Precision.Confusion());
        theBuilder.UpdateVertex(aFree, 0.5, anEdge, aFace, global::Kerf.Engine.Foundation.Precision.Confusion());
        if (includePointOnSurface)
        {
            theBuilder.UpdateVertex(aFree, 0.125, 0.375, aFace, global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        TopoDS_Compound aCompound = new();
        theBuilder.MakeCompound(ref aCompound);
        theBuilder.Add(aCompound, aBox);
        theBuilder.Add(aCompound, aFree);
        theBuilder.Add(aCompound, aVertex);
        return aCompound;
    }

    /// <summary>
    /// A compound of a box under several locations: elementary, complex (a product of two datums, with powers), and
    /// reversed, sharing its sub-shapes.
    /// </summary>
    public static TopoDS_Shape CompoundWithLocations()
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 1, 1);
        var aT1 = new Transform3D();
        aT1.SetTranslation(new Vector3D(5, 0, 0));
        var aT2 = new Transform3D();
        aT2.SetRotation(new Axis3D(new Point3D(), new Direction3D(0, 0, 1)), 0.3);
        var aL1 = new Placement(aT1);
        var aL2 = new Placement(aT2);
        Placement aComplex = aL1 * aL2.Powered(2);
        Placement aComplex2 = aL2.Powered(-3) * aL1 * aL2;

        TopoDS_Compound aCompound = new();
        theBuilder.MakeCompound(ref aCompound);
        theBuilder.Add(aCompound, aBox);
        theBuilder.Add(aCompound, aBox.Moved(aL1));
        theBuilder.Add(aCompound, aBox.Moved(aComplex).Reversed());
        theBuilder.Add(aCompound, aBox.Moved(aComplex2));
        TopoDS_Compound anInner = new();
        theBuilder.MakeCompound(ref anInner);
        theBuilder.Add(anInner, FirstEdge(aBox).Moved(aL2));
        theBuilder.Add(aCompound, anInner.Moved(aL1));
        return aCompound;
    }

    /// <summary>A compsolid of two boxes.</summary>
    public static TopoDS_Shape CompSolid()
    {
        TopoDS_CompSolid aCS = new();
        theBuilder.MakeCompSolid(ref aCS);
        theBuilder.Add(aCS, TestShapes.Box(1, 1, 1));
        var aT = new Transform3D();
        aT.SetTranslation(new Vector3D(1, 0, 0));
        theBuilder.Add(aCS, TestShapes.Box(1, 1, 1).Moved(new Placement(aT)));
        return aCS;
    }

    /// <summary>
    /// A box whose faces carry a two-triangle triangulation and whose edges carry polygons on it (one edge's polygons
    /// with parameters).
    /// </summary>
    public static TopoDS_Shape MeshedBox(bool theWithUV, bool theWithNormals)
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 2, 3);
        int aFaceIndex = 0;
        for (var aFaces = new TopExp_Explorer(aBox, TopAbs_FACE); aFaces.More(); aFaces.Next(), aFaceIndex++)
        {
            TopoDS_Face aFace = TopoDS.Face(aFaces.Current());
            var aVertices = new List<TopoDS_Vertex>();
            for (var aV = new TopExp_Explorer(aFace, TopAbs_VERTEX); aV.More(); aV.Next())
            {
                TopoDS_Vertex aVertex = TopoDS.Vertex(aV.Current());
                if (!aVertices.Exists(theOther => theOther.IsSame(aVertex)))
                {
                    aVertices.Add(aVertex);
                }
            }

            var aTri = new Triangulation(aVertices.Count, 2, theWithUV, theWithNormals);
            for (int i = 0; i < aVertices.Count; i++)
            {
                aTri.SetNode(i + 1, BRep_Tool.Pnt(aVertices[i]));
                if (theWithUV)
                {
                    aTri.SetUVNode(i + 1, new Point2D(i, 0.5 * i));
                }

                if (theWithNormals)
                {
                    aTri.SetNormal(i + 1, new NCollection_Vec3<float>(0.0f, 0.6f, 0.8f));
                }
            }

            aTri.SetTriangle(1, new Triangle1(1, 2, 3));
            aTri.SetTriangle(2, new Triangle1(1, 3, 4));
            aTri.Deflection(0.001 * (aFaceIndex + 1));
            theBuilder.UpdateFace(aFace, aTri);

            int anEdgeIndex = 0;
            for (var anE = new TopExp_Explorer(aFace, TopAbs_EDGE); anE.More(); anE.Next(), anEdgeIndex++)
            {
                TopoDS_Edge anEdge = TopoDS.Edge(anE.Current());
                var aNodes = new NCollection_Array1<int>(1, 2);
                aNodes[1] = 1 + anEdgeIndex;
                aNodes[2] = 1 + (anEdgeIndex + 1) % 4;
                PolygonOnTriangulation aPoly;
                if (anEdgeIndex == 0)
                {
                    var aParams = new NCollection_Array1<double>(1, 2);
                    aParams[1] = 0;
                    aParams[2] = 1;
                    aPoly = new PolygonOnTriangulation(aNodes, aParams);
                }
                else
                {
                    aPoly = new PolygonOnTriangulation(aNodes);
                }

                aPoly.Deflection(0.5);
                theBuilder.UpdateEdge(anEdge, aPoly, aTri, new Placement());
            }
        }

        return aBox;
    }

    /// <summary>The cylinder face, meshed, with its seam on two polygons of the triangulation (closed).</summary>
    public static TopoDS_Shape MeshedCylinder()
    {
        TopoDS_Face aFace = TestShapes.CylinderFace(2, 5);
        var aTri = new Triangulation(4, 2, true, false);
        aTri.SetNode(1, new Point3D(2, 0, 0));
        aTri.SetNode(2, new Point3D(-2, 0, 0));
        aTri.SetNode(3, new Point3D(-2, 0, 5));
        aTri.SetNode(4, new Point3D(2, 0, 5));
        aTri.SetUVNode(1, new Point2D(0, 0));
        aTri.SetUVNode(2, new Point2D(Math.PI, 0));
        aTri.SetUVNode(3, new Point2D(Math.PI, 5));
        aTri.SetUVNode(4, new Point2D(0, 5));
        aTri.SetTriangle(1, new Triangle1(1, 2, 3));
        aTri.SetTriangle(2, new Triangle1(1, 3, 4));
        theBuilder.UpdateFace(aFace, aTri);
        for (var anE = new TopExp_Explorer(aFace, TopAbs_EDGE); anE.More(); anE.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anE.Current());
            if (BRep_Tool.IsClosed(anEdge, aFace))
            {
                var aNodes1 = new NCollection_Array1<int>(1, 2);
                aNodes1[1] = 1;
                aNodes1[2] = 4;
                var aNodes2 = new NCollection_Array1<int>(1, 2);
                aNodes2[1] = 1;
                aNodes2[2] = 4;
                theBuilder.UpdateEdge(anEdge,
                                      new PolygonOnTriangulation(aNodes1),
                                      new PolygonOnTriangulation(aNodes2),
                                      aTri,
                                      new Placement());
                break;
            }
        }

        return aFace;
    }

    /// <summary>A face with a triangulation and no surface, whose normals are always written.</summary>
    public static TopoDS_Shape TriangulationOnlyFace()
    {
        var aTri = new Triangulation(3, 1, false, true);
        aTri.SetNode(1, new Point3D(0, 0, 0));
        aTri.SetNode(2, new Point3D(1, 0, 0));
        aTri.SetNode(3, new Point3D(0, 1, 0));
        aTri.SetTriangle(1, new Triangle1(1, 2, 3));
        for (int i = 1; i <= 3; i++)
        {
            aTri.SetNormal(i, new NCollection_Vec3<float>(0, 0, 1));
        }

        TopoDS_Face aFace = new();
        theBuilder.MakeFace(ref aFace, aTri);
        return aFace;
    }

    public static TopoDS_Edge FirstEdge(in TopoDS_Shape theShape) => NthEdge(theShape, 1);

    public static TopoDS_Edge NthEdge(in TopoDS_Shape theShape, int theN)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, TopAbs_EDGE, aMap);
        return TopoDS.Edge(aMap[theN]);
    }

    public static List<TopoDS_Face> FacesOf(in TopoDS_Shape theShape, TopoDS_Edge theEdge)
    {
        var aFaces = new List<TopoDS_Face>();
        for (var aF = new TopExp_Explorer(theShape, TopAbs_FACE); aF.More(); aF.Next())
        {
            for (var anE = new TopExp_Explorer(aF.Current(), TopAbs_EDGE); anE.More(); anE.Next())
            {
                if (anE.Current().IsSame(theEdge))
                {
                    aFaces.Add(TopoDS.Face(aF.Current()));
                    break;
                }
            }
        }

        return aFaces;
    }
}
