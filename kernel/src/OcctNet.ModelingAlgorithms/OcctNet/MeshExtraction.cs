// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// An OCCT.NET addition, not a port: the managed equivalent of build123d.NET's native mesh extraction
// (native/shim/src/b3d_mesh.cpp, b3d_mesh_extract), which fills Bento's viewer's buffers. Open question 12.

using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace OcctNet;

/// <summary>
/// Every face's triangulation and every edge's polyline of a meshed shape, in a few flat buffers, for a viewer: the
/// managed equivalent of build123d.NET's native <c>Extras.ExtractMesh</c>, float for float.
/// </summary>
/// <remarks>
/// <para>
/// The shape is meshed first (<see cref="BRepMesh_IncrementalMesh"/>); extraction only reads it, so it is safe from any
/// thread on a frozen shape, and a shape shared by several callers is never written. Native OCCT computes the nodal
/// normals into the face's own triangulation (<see cref="BRepLib_ToolTriangulatedShape.ComputeNormals(in Face, Triangulation?)"/>);
/// here they are computed on an unattached copy (phase3-design section 5), with the same values.
/// </para>
/// <para>
/// A face without a triangulation gives no nodes. An edge is taken from the polygon on triangulation of the first of
/// its faces (in <see cref="Exploration.MapShapesAndAncestors"/> order) that carries one, so it lies exactly on the shaded
/// surface; else from the mesher's 3D polygon of a free edge; else it is discretised with
/// <see cref="TangentialDeflection"/> to the given deflection and angle. An edge's points run the way the edge
/// does.
/// </para>
/// </remarks>
public static class MeshExtraction
{
    /// <summary>
    /// The buffers of <paramref name="theFaces"/> and <paramref name="theEdges"/>, sub-shapes of the meshed
    /// <paramref name="theRoot"/>, in the order given (b3d_mesh_extract's arguments).
    /// </summary>
    /// <param name="theRoot">The meshed shape: its faces are the edges' ancestors.</param>
    /// <param name="theFaces">The faces to extract, with their locations and orientations.</param>
    /// <param name="theEdges">The edges to extract, with their locations and orientations.</param>
    /// <param name="theDeflection">The linear deflection for edges without a polygon.</param>
    /// <param name="theAngle">The angular deflection, in radians, for edges without a polygon.</param>
    public static MeshBuffers Extract(in Shape theRoot, IReadOnlyList<Shape> theFaces,
                                      IReadOnlyList<Shape> theEdges, double theDeflection, double theAngle)
    {
        ArgumentNullException.ThrowIfNull(theFaces);
        ArgumentNullException.ThrowIfNull(theEdges);

        var anEdgeFaces = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theRoot, ShapeType.Edge, ShapeType.Face, anEdgeFaces);

        // Sizes first, so each buffer is allocated once at its final length
        var aTris = new Triangulation?[theFaces.Count];
        var aLocations = new Placement[theFaces.Count];
        long aNbNodes = 0, aNbIndices = 0;
        for (int i = 0; i < theFaces.Count; i++)
        {
            aTris[i] = Tool.Triangulation(ShapeCasts.Face(theFaces[i]), out aLocations[i]);
            if (aTris[i] is { } aTri)
            {
                aNbNodes += aTri.NodeCount;
                aNbIndices += 3L * aTri.TriangleCount;
            }
        }

        var aBuilder = new Builder(checked((int)(3 * aNbNodes)), checked((int)aNbIndices), theFaces.Count, theEdges.Count);
        for (int i = 0; i < theFaces.Count; i++)
        {
            aBuilder.AddFace(ShapeCasts.Face(theFaces[i]), aTris[i], aLocations[i]);
        }
        aBuilder.FaceNodes[theFaces.Count] = aBuilder.NodeCount;
        aBuilder.FaceTriangles[theFaces.Count] = aBuilder.IndexCount;

        for (int i = 0; i < theEdges.Count; i++)
        {
            aBuilder.AddEdge(ShapeCasts.Edge(theEdges[i]), anEdgeFaces, theDeflection, theAngle);
        }
        return aBuilder.Finish();
    }

    /// <summary>
    /// The buffers of every face and every edge of the meshed <paramref name="theRoot"/>, in the order of
    /// <see cref="Faces"/> and <see cref="Edges"/>: build123d.NET's <c>ShapeMesh</c>.
    /// </summary>
    public static MeshBuffers Extract(in Shape theRoot, double theDeflection, double theAngle)
        => Extract(theRoot, Faces(theRoot), Edges(theRoot), theDeflection, theAngle);

    /// <summary>
    /// The distinct faces of a shape in exploration order, as build123d.NET's <c>Shape.Faces()</c>: each face once
    /// (<see cref="Shape.IsSame"/>), at its first place, with the orientation of its last occurrence.
    /// </summary>
    public static List<Shape> Faces(in Shape theShape) => Entities(theShape, ShapeType.Face);

    /// <summary>The distinct edges of a shape, as <see cref="Faces"/>: build123d.NET's <c>Shape.Edges()</c>.</summary>
    public static List<Shape> Edges(in Shape theShape) => Entities(theShape, ShapeType.Edge);

    private static List<Shape> Entities(in Shape theShape, ShapeType theType)
    {
        var anIndex = new Dictionary<Shape, int>(ShapeHasher.Instance);
        var aFound = new List<Shape>();
        if (theShape.IsNull)
        {
            return aFound;
        }
        for (var anExp = new Explorer(theShape, theType); anExp.More(); anExp.Next())
        {
            Shape anItem = anExp.Current();
            if (anIndex.TryGetValue(anItem, out int i))
            {
                aFound[i] = anItem;
            }
            else
            {
                anIndex.Add(anItem, aFound.Count);
                aFound.Add(anItem);
            }
        }
        return aFound;
    }

    /// <summary>Fills the buffers: b3d_mesh.cpp's addFace and addEdge.</summary>
    private sealed class Builder(int theNodeFloats, int theIndices, int theFaces, int theEdges)
    {
        public readonly float[] Positions = new float[theNodeFloats];
        public readonly float[] Normals = new float[theNodeFloats];
        public readonly int[] Triangles = new int[theIndices];
        public readonly long[] FaceNodes = new long[theFaces + 1];
        public readonly long[] FaceTriangles = new long[theFaces + 1];
        public readonly int[] FaceTypes = new int[theFaces];
        public readonly long[] EdgePoints = new long[theEdges + 1];
        public readonly int[] EdgeTypes = new int[theEdges];
        private readonly List<float> myPoints = new(Math.Max(16, 4 * theNodeFloats / 10));
        private int myNodeFloats, myIndices, myFace, myEdge;

        public long NodeCount => myNodeFloats / 3;

        public long IndexCount => myIndices;

        public void AddFace(in Face theFace, Triangulation? theTri, in Placement theLocation)
        {
            FaceNodes[myFace] = NodeCount;
            FaceTriangles[myFace] = myIndices;
            FaceTypes[myFace++] = (int)new FaceSurfaceAdaptor(theFace, false).GetType();
            if (theTri is null)
            {
                return;
            }

            // The surface's normals at the nodes, on a copy: the face's own triangulation is never written
            Triangulation aTri = BRepLib_ToolTriangulatedShape.NormalsComputed(theFace, theTri)!;
            Transform3D aTrsf = theLocation.Transformation();
            bool isReversed = theFace.Orientation() == Reversed;
            // Winding follows the surface's natural normal: flipped for a reversed face, and again by a mirroring location
            bool toFlip = isReversed != aTrsf.IsNegative;
            for (int i = 1; i <= aTri.NodeCount; ++i)
            {
                Point3D aP = aTri.Node(i).Transformed(aTrsf);
                Positions[myNodeFloats] = (float)aP.X();
                Positions[myNodeFloats + 1] = (float)aP.Y();
                Positions[myNodeFloats + 2] = (float)aP.Z();
                Direction3D aN = aTri.Normal(i).Transformed(aTrsf);
                if (isReversed)
                {
                    aN = aN.Reversed();
                }
                Normals[myNodeFloats] = (float)aN.X();
                Normals[myNodeFloats + 1] = (float)aN.Y();
                Normals[myNodeFloats + 2] = (float)aN.Z();
                myNodeFloats += 3;
            }
            for (int i = 1; i <= aTri.TriangleCount; ++i)
            {
                aTri.Triangle(i).Get(out int a, out int b, out int c);
                Triangles[myIndices] = a - 1;
                Triangles[myIndices + 1] = toFlip ? c - 1 : b - 1;
                Triangles[myIndices + 2] = toFlip ? b - 1 : c - 1;
                myIndices += 3;
            }
        }

        private void AddPoint(in Point3D theP)
        {
            myPoints.Add((float)theP.X());
            myPoints.Add((float)theP.Y());
            myPoints.Add((float)theP.Z());
        }

        public void AddEdge(in Edge theEdge, NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theEdgeFaces,
                            double theDeflection, double theAngle)
        {
            EdgePoints[myEdge] = myPoints.Count / 3;
            var aCurve = new EdgeCurveAdaptor(theEdge);
            EdgeTypes[myEdge++] = (int)aCurve.GetType();
            int aStart = myPoints.Count;

            // On a face's triangulation, so the edge sits exactly on the shaded surface
            bool isDone = false;
            if (theEdgeFaces.FindFromKey(theEdge, out NCollection_List<Shape> aFaces))
            {
                foreach (Shape aShape in aFaces)
                {
                    Triangulation? aTri = Tool.Triangulation(ShapeCasts.Face(aShape), out Placement aLocation);
                    if (aTri is null)
                    {
                        continue;
                    }
                    PolygonOnTriangulation? aPolygon = Tool.PolygonOnTriangulation(theEdge, aTri, aLocation);
                    if (aPolygon is null)
                    {
                        continue;
                    }
                    Transform3D aTrsf = aLocation.Transformation();
                    for (int i = 1; i <= aPolygon.NodeCount; ++i)
                    {
                        AddPoint(aTri.Node(aPolygon.Node(i)).Transformed(aTrsf));
                    }
                    isDone = true;
                    break;
                }
            }
            // A free edge's own polygon, from the mesher
            if (!isDone)
            {
                Polygon3D? aPolygon = Tool.Polygon3D(theEdge, out Placement aLocation);
                if (aPolygon is not null)
                {
                    Transform3D aTrsf = aLocation.Transformation();
                    NCollection_Array1<Point3D> aNodes = aPolygon.Nodes();
                    for (int i = 1; i <= aPolygon.NodeCount; ++i)
                    {
                        AddPoint(aNodes.Value(i).Transformed(aTrsf));
                    }
                    isDone = true;
                }
            }
            // Else discretised to the same tolerances
            if (!isDone)
            {
                var aDiscretizer = new TangentialDeflection(aCurve, theAngle, theDeflection);
                for (int i = 1; i <= aDiscretizer.PointCount; ++i)
                {
                    AddPoint(aDiscretizer.Value(i));
                }
            }
            // Polygons follow the curve's parameter: reversed to run the way the edge does
            if (theEdge.Orientation() == Reversed)
            {
                int aCount = (myPoints.Count - aStart) / 3;
                for (int i = 0; i < aCount / 2; ++i)
                {
                    for (int k = 0; k < 3; ++k)
                    {
                        int p = aStart + 3 * i + k, q = aStart + 3 * (aCount - 1 - i) + k;
                        (myPoints[p], myPoints[q]) = (myPoints[q], myPoints[p]);
                    }
                }
            }
        }

        public MeshBuffers Finish()
        {
            EdgePoints[myEdge] = myPoints.Count / 3;
            return new MeshBuffers(Positions, Normals, Triangles, FaceNodes, FaceTriangles, FaceTypes, [.. myPoints],
                                   EdgePoints, EdgeTypes);
        }
    }
}

/// <summary>
/// The buffers <see cref="MeshExtraction.Extract(in Shape, IReadOnlyList{Shape}, IReadOnlyList{Shape}, double, double)"/>
/// fills, laid out as build123d.NET's <c>MeshBuffers</c>: every face's nodes (x, y, z each, located), their unit
/// normals (facing out of a reversed face too) and its triangles (index triples into the face's own nodes, wound to
/// face out) end to end, then every edge's polyline. The offsets have one entry per face or edge and a final total.
/// </summary>
/// <remarks>The arrays are the extraction's own; nothing else holds them.</remarks>
public sealed class MeshBuffers
{
    internal MeshBuffers(float[] thePositions, float[] theNormals, int[] theTriangles, long[] theFaceNodes, long[] theFaceTriangles,
                         int[] theFaceTypes, float[] thePoints, long[] theEdgePoints, int[] theEdgeTypes)
    {
        Positions = thePositions;
        Normals = theNormals;
        Triangles = theTriangles;
        FaceNodes = theFaceNodes;
        FaceTriangles = theFaceTriangles;
        FaceTypes = theFaceTypes;
        Points = thePoints;
        EdgePoints = theEdgePoints;
        EdgeTypes = theEdgeTypes;
    }

    /// <summary>The faces' nodes, x, y, z each, with their locations applied.</summary>
    public float[] Positions { get; }

    /// <summary>The unit normal at each node, x, y, z each, facing out of the face as oriented.</summary>
    public float[] Normals { get; }

    /// <summary>The triangles: three indices each into their face's own nodes (from 0), counter-clockwise seen from outside.</summary>
    public int[] Triangles { get; }

    /// <summary>Each face's first node, then the total.</summary>
    public long[] FaceNodes { get; }

    /// <summary>Each face's first triangle index (three per triangle), then the total.</summary>
    public long[] FaceTriangles { get; }

    /// <summary>Each face's <see cref="SurfaceKind"/>.</summary>
    public int[] FaceTypes { get; }

    /// <summary>The edges' polylines, x, y, z per point, with their locations applied.</summary>
    public float[] Points { get; }

    /// <summary>Each edge's first point, then the total.</summary>
    public long[] EdgePoints { get; }

    /// <summary>Each edge's <see cref="CurveKind"/>.</summary>
    public int[] EdgeTypes { get; }
}
