using System.Text;
using O = global::Occt;

namespace Build123d.Occt;

/// <summary>
/// What the native binding's hand-written shim functions do (native/shim/src/b3d_extras.cpp, b3d_mesh.cpp), over
/// OCCT.NET.
/// </summary>
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
        try
        {
            if (edge.V.TShape() is not O.BRep_TEdge tedge)
                return null;
            foreach (var rep in tedge.CurveRepresentations())
                if (rep.IsCurve3D())
                    {
                    var n = normal.V;   // OCCT computes a normal into a null one; the caller's stays as it is
                    return O.ShapeAnalysis_Curve.IsPlanar(rep.Curve3D()!, ref n, tolerance);
                }
            return null;
        }
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }

    /// <summary>
    /// A copy of a history with its maps copied as they are. <c>Merge</c> into an empty history isn't a copy: it drops
    /// the removal of a shape that also generated or was modified.
    /// </summary>
    public static BRepTools_History CopyHistory(BRepTools_History history)
    {
        ArgumentNullException.ThrowIfNull(history);
        var source = history.Impl;
        var copy = new O.BRepTools_History();
        static void CopyMap(O.NCollection_DataMap<O.TopoDS_Shape, O.NCollection_List<O.TopoDS_Shape>> from,
                            O.NCollection_DataMap<O.TopoDS_Shape, O.NCollection_List<O.TopoDS_Shape>> to)
        {
            foreach (var (key, shapes) in from)
            {
                var list = new O.NCollection_List<O.TopoDS_Shape>();
                foreach (var s in shapes)
                    list.Append(s);
                to.Bind(key, list);
            }
        }
        CopyMap(Internals.Modified(source), Internals.Modified(copy));
        CopyMap(Internals.Generated(source), Internals.Generated(copy));
        var removed = Internals.Removed(copy);
        foreach (var s in Internals.Removed(source))
            removed.Add(s);
        return new BRepTools_History(copy);
    }

    /// <summary>A shape in OCCT's BRep text format, with its triangulations if <paramref name="withTriangles"/>.</summary>
    public static byte[] WriteBRep(TopoDS_Shape shape, bool withTriangles = false)
    {
        ArgumentNullException.ThrowIfNull(shape);
        try
        {
            var writer = new StringWriter(System.Globalization.CultureInfo.InvariantCulture) { NewLine = "\n" };
            O.BRepTools.Write(shape.V, writer, withTriangles, false, O.TopTools_FormatVersion.TopTools_FormatVersion_CURRENT);
            return Encoding.ASCII.GetBytes(writer.ToString());
        }
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }

    /// <summary>A shape read from OCCT's BRep text format.</summary>
    public static TopoDS_Shape ReadBRep(ReadOnlySpan<byte> data)
    {
        // The native reader overruns on long words and loops at the end of truncated data, so the native binding checks
        // first; OCCT.NET's reader is safe, but the same input is refused the same way
        const int longestWord = 200;
        int word = 0;
        foreach (var c in data)
        {
            word = c is (byte)' ' or (byte)'\t' or (byte)'\n' or (byte)'\r' or (byte)'\v' or (byte)'\f' ? 0 : word + 1;
            if (word >= longestWord)
                throw new OcctException("Storage_StreamFormatError", "BRep data has a word too long to be BRep");
            if (c == 0)
                throw new OcctException("Storage_StreamFormatError", "BRep data is text, without NULs");
        }
        var text = Encoding.ASCII.GetString(data);
        O.TopoDS_Shape shape;
        try
        {
            var reader = new StringReader(text);
            O.Standard_Stream.SetInputLength(reader, text.Length);
            O.BRepTools.Read(out shape, reader);
        }
        catch (O.Standard_Failure e)
        {
            throw e is O.Storage_StreamFormatError ? OcctException.From(e)
                : new OcctException("Storage_StreamFormatError", $"BRep data is truncated or malformed ({e.OcctType})");
        }
        catch (Exception e) when (e is FormatException or EndOfStreamException or OverflowException or IndexOutOfRangeException)
        {
            throw new OcctException("Storage_StreamFormatError", "BRep data is truncated or malformed");
        }
        if (shape.IsNull())
            throw new OcctException("Storage_StreamFormatError", "BRep data holds no shape");
        return new TopoDS_Shape(shape);
    }

    /// <summary>
    /// The triangulations of <paramref name="faces"/> and the polylines of <paramref name="edges"/>, sub-shapes of the
    /// meshed <paramref name="root"/>, copied out in bulk (OCCT.NET's <c>MeshExtraction</c>, float for float the shim's).
    /// </summary>
    public static MeshBuffers ExtractMesh(TopoDS_Shape root, IReadOnlyList<TopoDS_Shape> faces, IReadOnlyList<TopoDS_Shape> edges,
                                          double deflection, double angle)
    {
        ArgumentNullException.ThrowIfNull(root);
        try
        {
            var m = global::OcctNet.MeshExtraction.Extract(root.V, faces.Select(f => f.V).ToList(), edges.Select(e => e.V).ToList(), deflection, angle);
            return new MeshBuffers(m.Positions, m.Normals, m.Triangles, m.FaceNodes, m.FaceTriangles, m.FaceTypes,
                                   m.Points, m.EdgePoints, m.EdgeTypes);
        }
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }

    /// <summary>
    /// A rough estimate of the memory a shape holds, in bytes: its sub-shapes, curves, surfaces and triangulations,
    /// each counted once however often it's shared (the native shim's estimate, for the same shapes).
    /// </summary>
    public static long EstimateMemory(TopoDS_Shape shape)
    {
        ArgumentNullException.ThrowIfNull(shape);
        try
        {
            return new MemoryEstimate().Of(shape.V);
        }
        catch (O.Standard_Failure)
        {
            return 0;
        }
    }

    /// <summary>
    /// Under the native backend, tells the GC about the native memory a shape holds. OCCT.NET's memory is managed, so
    /// the GC already counts it: nothing is added.
    /// </summary>
    public static void AddMemoryPressure(TopoDS_Shape shape) => ArgumentNullException.ThrowIfNull(shape);

    /// <summary>The native memory a wrapper has told the GC about: none under OCCT.NET.</summary>
    public static long MemoryPressureOf(OcctObject wrapper) => wrapper.MemoryPressure;

    /// <summary>
    /// The names of the errors an algorithm reported ("BOPAlgo_AlertTooFewArguments, BOPAlgo_AlertSelfInterferingShape"),
    /// or null if it reported none.
    /// </summary>
    public static string? AlgoErrors(BRepAlgoAPI_Algo algo)
    {
        ArgumentNullException.ThrowIfNull(algo);
        try
        {
            var names = new List<string>();
            foreach (var alert in algo.Impl.GetReport().GetAlerts(O.Message_Gravity.Message_Fail))
            {
                var name = alert.DynamicType().Name;
                if (!names.Contains(name))
                    names.Add(name);
            }
            return names.Count == 0 ? null : string.Join(", ", names);
        }
        catch (Exception)
        {
            return null; // diagnostics mustn't mask the failure they explain
        }
    }

    /// <summary>The shim's MemoryEstimate, over OCCT.NET's representations.</summary>
    private sealed class MemoryEstimate
    {
        private readonly HashSet<object> _seen = new(ReferenceEqualityComparer.Instance);
        private long _bytes;

        public long Of(O.TopoDS_Shape shape)
        {
            if (!shape.IsNull())
                Visit(shape);
            return _bytes;
        }

        private bool First(object? o) => o is not null && _seen.Add(o);

        private void Visit(O.TopoDS_Shape shape)
        {
            var tshape = shape.TShape();
            if (!First(tshape))
                return;
            _bytes += 96;
            if (tshape is O.BRep_TEdge tedge)
                foreach (var rep in tedge.CurveRepresentations())
                {
                    _bytes += 64;
                    if (rep.IsCurve3D())
                        Curve(rep.Curve3D());
                    else if (rep.IsCurveOnSurface())
                    {
                        Curve2d(rep.PCurve());
                        if (rep.IsCurveOnClosedSurface())
                            Curve2d(rep.PCurve2());
                    }
                    else if (rep.IsPolygon3D() && rep.Polygon3D() is { } p && First(p))
                        _bytes += 32L * p.NbNodes();
                    else if (rep.IsPolygonOnTriangulation() && rep.PolygonOnTriangulation() is { } pt && First(pt))
                        _bytes += 12L * pt.NbNodes();
                }
            else if (tshape is O.BRep_TFace tface)
            {
                Surface(tface.Surface());
                foreach (var tri in tface.TriangulationList())
                    if (First(tri))
                        _bytes += (long)tri.NbNodes() * (24 + (tri.HasUVNodes() ? 16 : 0) + (tri.HasNormals() ? 12 : 0)) + 12L * tri.NbTriangles();
            }
            for (var it = new O.TopoDS_Iterator(shape, false, false); it.More(); it.Next())
                Visit(it.Value());
        }

        private void Curve(O.Geom_Curve? c)
        {
            if (!First(c))
                return;
            _bytes += 80;
            switch (c)
            {
                case O.Geom_BSplineCurve b: _bytes += 32L * b.NbPoles() + 12L * b.NbKnots(); break;
                case O.Geom_BezierCurve z: _bytes += 32L * z.NbPoles(); break;
                case O.Geom_TrimmedCurve t: Curve(t.BasisCurve()); break;
                case O.Geom_OffsetCurve o: Curve(o.BasisCurve()); break;
            }
        }

        private void Curve2d(O.Geom2d_Curve? c)
        {
            if (!First(c))
                return;
            _bytes += 64;
            switch (c)
            {
                case O.Geom2d_BSplineCurve b: _bytes += 24L * b.NbPoles() + 12L * b.NbKnots(); break;
                case O.Geom2d_BezierCurve z: _bytes += 24L * z.NbPoles(); break;
                case O.Geom2d_TrimmedCurve t: Curve2d(t.BasisCurve()); break;
                case O.Geom2d_OffsetCurve o: Curve2d(o.BasisCurve()); break;
            }
        }

        private void Surface(O.Geom_Surface? s)
        {
            if (!First(s))
                return;
            _bytes += 120;
            switch (s)
            {
                case O.Geom_BSplineSurface b: _bytes += 32L * b.NbUPoles() * b.NbVPoles() + 12L * (b.NbUKnots() + b.NbVKnots()); break;
                case O.Geom_BezierSurface z: _bytes += 32L * z.NbUPoles() * z.NbVPoles(); break;
                case O.Geom_RectangularTrimmedSurface t: Surface(t.BasisSurface()); break;
                case O.Geom_OffsetSurface o: Surface(o.BasisSurface()); break;
                case O.Geom_SweptSurface w: Curve(w.BasisCurve()); break;
            }
        }
    }
}

/// <summary>
/// The buffers <see cref="Extras.ExtractMesh"/> fills: every face's nodes (x, y, z each, located), their normals and
/// the triangles (index triples into each face's own nodes, wound to face out) end to end, then every edge's
/// polyline. The offsets have one entry per face or edge and a final total.
/// </summary>
public sealed class MeshBuffers
{
    internal MeshBuffers(float[] positions, float[] normals, int[] triangles, long[] faceNodes, long[] faceTriangles,
                         int[] faceTypes, float[] points, long[] edgePoints, int[] edgeTypes)
    {
        Positions = positions;
        Normals = normals;
        Triangles = triangles;
        FaceNodes = faceNodes;
        FaceTriangles = faceTriangles;
        FaceTypes = faceTypes;
        Points = points;
        EdgePoints = edgePoints;
        EdgeTypes = edgeTypes;
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
