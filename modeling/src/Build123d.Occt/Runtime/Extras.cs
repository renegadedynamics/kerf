using System.Runtime.InteropServices;

namespace Build123d.Occt;

public partial class TopoDS_Shape
{
    /// <summary>
    /// <c>std::hash&lt;TopoDS_Shape&gt;</c>: equal for shapes that are <c>IsSame</c> (the same TShape at the
    /// same Location), whatever their orientation.
    /// </summary>
    public ulong StdHash()
    {
        var h = Extras.N.b3d_TopoDS_Shape_hash(PtrAs_TopoDS_Shape());
        GC.KeepAlive(this);
        return h;
    }
}

/// <summary>Hand-written shim functions (native/shim/src/b3d_extras.cpp) for what OcctGen can't bind.</summary>
public static partial class Extras
{
    /// <summary>
    /// Whether the edge's own 3D curve lies in a plane normal to <paramref name="normal"/>
    /// (<c>ShapeAnalysis_Curve::IsPlanar</c> on the curve in <c>BRep_TEdge::Curves()</c>), or null if it has none.
    /// </summary>
    public static bool? EdgeCurve3DIsPlanar(TopoDS_Edge edge, gp_XYZ normal, double tolerance)
    {
        ArgumentNullException.ThrowIfNull(edge);
        ArgumentNullException.ThrowIfNull(normal);
        var r = N.b3d_edge_curve3d_is_planar(edge.PtrAs_TopoDS_Edge(), normal.PtrAs_gp_XYZ(), tolerance, out var err);
        GC.KeepAlive(edge);
        GC.KeepAlive(normal);
        OcctException.ThrowIfError(err);
        return r < 0 ? null : r == 1;
    }

    /// <summary>
    /// A copy of a history with its maps copied as they are. <c>Merge</c> into an empty history isn't a copy: it drops
    /// the removal of a shape that also generated or was modified.
    /// </summary>
    public static BRepTools_History CopyHistory(BRepTools_History history)
    {
        ArgumentNullException.ThrowIfNull(history);
        var r = N.b3d_BRepTools_History_copy(history.PtrAs_BRepTools_History(), out var err);
        GC.KeepAlive(history);
        OcctException.ThrowIfError(err);
        return new BRepTools_History(r);
    }

    /// <summary>A shape in OCCT's BRep text format, with its triangulations if <paramref name="withTriangles"/>.</summary>
    public static byte[] WriteBRep(TopoDS_Shape shape, bool withTriangles = false)
    {
        ArgumentNullException.ThrowIfNull(shape);
        var data = N.b3d_brep_write(shape.PtrAs_TopoDS_Shape(), withTriangles ? (byte)1 : (byte)0, out var size, out var err);
        GC.KeepAlive(shape);
        OcctException.ThrowIfError(err);
        try
        {
            var bytes = new byte[size];
            Marshal.Copy(data, bytes, 0, checked((int)size));
            return bytes;
        }
        finally
        {
            N.b3d_free_bytes(data);
        }
    }

    /// <summary>A shape read from OCCT's BRep text format.</summary>
    public static TopoDS_Shape ReadBRep(ReadOnlySpan<byte> data)
    {
        IntPtr r, err;
        unsafe
        {
            fixed (byte* p = data)
                r = N.b3d_brep_read((IntPtr)p, data.Length, out err);
        }
        OcctException.ThrowIfError(err);
        return new TopoDS_Shape(r);
    }

    /// <summary>
    /// The triangulations of <paramref name="faces"/> and the polylines of <paramref name="edges"/>, sub-shapes of the
    /// meshed <paramref name="root"/>, copied out in bulk. Edges come from a face's triangulation where they lie on
    /// one, else from the mesher's own polygon, else are discretised to <paramref name="deflection"/> and
    /// <paramref name="angle"/>.
    /// </summary>
    public static MeshBuffers ExtractMesh(TopoDS_Shape root, IReadOnlyList<TopoDS_Shape> faces, IReadOnlyList<TopoDS_Shape> edges,
                                          double deflection, double angle)
    {
        ArgumentNullException.ThrowIfNull(root);
        var facePtrs = faces.Select(f => f.PtrAs_TopoDS_Shape()).ToArray();
        var edgePtrs = edges.Select(e => e.PtrAs_TopoDS_Shape()).ToArray();
        IntPtr mesh, err;
        unsafe
        {
            fixed (IntPtr* f = facePtrs)
            fixed (IntPtr* e = edgePtrs)
                mesh = N.b3d_mesh_extract(root.PtrAs_TopoDS_Shape(), (IntPtr)f, facePtrs.Length, (IntPtr)e, edgePtrs.Length, deflection, angle, out err);
        }
        GC.KeepAlive(root);
        GC.KeepAlive(faces);
        GC.KeepAlive(edges);
        OcctException.ThrowIfError(err);
        try
        {
            N.b3d_mesh_sizes(mesh, out var nodeFloats, out var triangleIndices, out var pointFloats);
            var buffers = new MeshBuffers(checked((int)nodeFloats), checked((int)triangleIndices), checked((int)pointFloats),
                                          facePtrs.Length, edgePtrs.Length);
            unsafe
            {
                fixed (float* positions = buffers.Positions)
                fixed (float* normals = buffers.Normals)
                fixed (int* triangles = buffers.Triangles)
                fixed (long* faceNodes = buffers.FaceNodes)
                fixed (long* faceTriangles = buffers.FaceTriangles)
                fixed (int* faceTypes = buffers.FaceTypes)
                fixed (float* points = buffers.Points)
                fixed (long* edgePoints = buffers.EdgePoints)
                fixed (int* edgeTypes = buffers.EdgeTypes)
                    N.b3d_mesh_copy(mesh, (IntPtr)positions, (IntPtr)normals, (IntPtr)triangles, (IntPtr)faceNodes, (IntPtr)faceTriangles,
                                    (IntPtr)faceTypes, (IntPtr)points, (IntPtr)edgePoints, (IntPtr)edgeTypes);
            }
            return buffers;
        }
        finally
        {
            N.b3d_mesh_free(mesh);
        }
    }

    internal static partial class N
    {
        [LibraryImport("b3docct")]
        internal static partial IntPtr b3d_mesh_extract(IntPtr root, IntPtr faces, int faceCount, IntPtr edges, int edgeCount,
                                                        double deflection, double angle, out IntPtr err);

        [LibraryImport("b3docct")]
        internal static partial void b3d_mesh_sizes(IntPtr mesh, out long nodeFloats, out long triangleIndices, out long pointFloats);

        [LibraryImport("b3docct")]
        internal static partial void b3d_mesh_copy(IntPtr mesh, IntPtr positions, IntPtr normals, IntPtr triangles, IntPtr faceNodes,
                                                   IntPtr faceTriangles, IntPtr faceTypes, IntPtr points, IntPtr edgePoints, IntPtr edgeTypes);

        [LibraryImport("b3docct")]
        internal static partial void b3d_mesh_free(IntPtr mesh);

        [LibraryImport("b3docct")]
        internal static partial IntPtr b3d_brep_write(IntPtr shape, byte withTriangles, out long size, out IntPtr err);

        [LibraryImport("b3docct")]
        internal static partial IntPtr b3d_brep_read(IntPtr data, long size, out IntPtr err);

        [LibraryImport("b3docct")]
        internal static partial void b3d_free_bytes(IntPtr data);

        [LibraryImport("b3docct")]
        internal static partial IntPtr b3d_BRepTools_History_copy(IntPtr history, out IntPtr err);

        [LibraryImport("b3docct")]
        internal static partial ulong b3d_TopoDS_Shape_hash(IntPtr shape);

        [LibraryImport("b3docct")]
        internal static partial int b3d_edge_curve3d_is_planar(IntPtr edge, IntPtr normal, double tolerance, out IntPtr err);
    }
}

/// <summary>
/// The buffers <see cref="Extras.ExtractMesh"/> fills: every face's nodes (x, y, z each, located), their normals and
/// the triangles (index triples into each face's own nodes, wound to face out) end to end, then every edge's
/// polyline. The offsets have one entry per face or edge and a final total.
/// </summary>
public sealed class MeshBuffers
{
    internal MeshBuffers(int nodeFloats, int triangleIndices, int pointFloats, int faces, int edges)
    {
        Positions = new float[nodeFloats];
        Normals = new float[nodeFloats];
        Triangles = new int[triangleIndices];
        FaceNodes = new long[faces + 1];
        FaceTriangles = new long[faces + 1];
        FaceTypes = new int[faces];
        Points = new float[pointFloats];
        EdgePoints = new long[edges + 1];
        EdgeTypes = new int[edges];
    }

    public float[] Positions { get; }
    public float[] Normals { get; }
    public int[] Triangles { get; }
    /// <summary>Each face's first node, then the total.</summary>
    public long[] FaceNodes { get; }
    /// <summary>Each face's first triangle index (three per triangle), then the total.</summary>
    public long[] FaceTriangles { get; }
    /// <summary>Each face's <c>GeomAbs_SurfaceType</c>.</summary>
    public int[] FaceTypes { get; }
    public float[] Points { get; }
    /// <summary>Each edge's first point, then the total.</summary>
    public long[] EdgePoints { get; }
    /// <summary>Each edge's <c>GeomAbs_CurveType</c>.</summary>
    public int[] EdgeTypes { get; }
}
