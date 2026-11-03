using System.Text;
using Occt;

namespace Build123d;

/// <summary>
/// The modelling layer's helpers over OCCT.NET that OCCT has no single call for (once the native shim's hand-written
/// functions, b3d_extras.cpp): BRep text as bytes, a shape's memory estimate, an algorithm's errors, an edge's own curve.
/// </summary>
internal static partial class Extras
{
    /// <summary>
    /// Whether the edge's own 3D curve lies in a plane normal to <paramref name="normal"/>
    /// (<c>ShapeAnalysis_Curve::IsPlanar</c> on the curve in <c>BRep_TEdge::Curves()</c>), or null if it has none.
    /// </summary>
    public static bool? EdgeCurve3DIsPlanar(TopoDS_Edge edge, gp_XYZ normal, double tolerance)
    {
        if (edge.TShape() is not BRep_TEdge tedge)
            return null;
        foreach (var rep in tedge.CurveRepresentations())
            if (rep.IsCurve3D())
            {
                var n = normal;   // OCCT computes a normal into a null one; the caller's stays as it is
                return ShapeAnalysis_Curve.IsPlanar(rep.Curve3D()!, ref n, tolerance);
            }
        return null;
    }

    /// <summary>A shape in OCCT's BRep text format, with its triangulations if <paramref name="withTriangles"/>.</summary>
    public static byte[] WriteBRep(TopoDS_Shape shape, bool withTriangles = false)
    {
        var writer = new StringWriter(System.Globalization.CultureInfo.InvariantCulture) { NewLine = "\n" };
        BRepTools.Write(shape, writer, withTriangles, false, TopTools_FormatVersion.TopTools_FormatVersion_CURRENT);
        return Encoding.ASCII.GetBytes(writer.ToString());
    }

    /// <summary>
    /// A shape read from OCCT's BRep text format. Data that isn't BRep, or is truncated or malformed, throws
    /// <see cref="Storage_StreamFormatError"/>.
    /// </summary>
    public static TopoDS_Shape ReadBRep(ReadOnlySpan<byte> data)
    {
        // The native reader overran on long words and looped at the end of truncated data, so the native binding checked
        // first; OCCT.NET's reader is safe, but the same input is refused the same way
        const int longestWord = 200;
        int word = 0;
        foreach (var c in data)
        {
            word = c is (byte)' ' or (byte)'\t' or (byte)'\n' or (byte)'\r' or (byte)'\v' or (byte)'\f' ? 0 : word + 1;
            if (word >= longestWord)
                throw new Storage_StreamFormatError("BRep data has a word too long to be BRep");
            if (c == 0)
                throw new Storage_StreamFormatError("BRep data is text, without NULs");
        }
        var text = Encoding.ASCII.GetString(data);
        TopoDS_Shape shape;
        try
        {
            var reader = new StringReader(text);
            Standard_Stream.SetInputLength(reader, text.Length);
            BRepTools.Read(out shape, reader);
        }
        catch (Standard_Failure e) when (e is not Storage_StreamFormatError)
        {
            throw new Storage_StreamFormatError($"BRep data is truncated or malformed ({e.OcctType})");
        }
        catch (Exception e) when (e is FormatException or EndOfStreamException or OverflowException or IndexOutOfRangeException)
        {
            throw new Storage_StreamFormatError("BRep data is truncated or malformed");
        }
        if (shape.IsNull())
            throw new Storage_StreamFormatError("BRep data holds no shape");
        return shape;
    }

    /// <summary>
    /// A rough estimate of the memory a shape holds, in bytes: its sub-shapes, curves, surfaces and triangulations,
    /// each counted once however often it's shared (the native shim's estimate, for the same shapes).
    /// </summary>
    public static long EstimateMemory(TopoDS_Shape shape)
    {
        try
        {
            return new MemoryEstimate().Of(shape);
        }
        catch (Standard_Failure)
        {
            return 0;
        }
    }

    /// <summary>
    /// The names of the errors an algorithm reported ("BOPAlgo_AlertTooFewArguments, BOPAlgo_AlertSelfInterferingShape"),
    /// or null if it reported none.
    /// </summary>
    public static string? AlgoErrors(BRepAlgoAPI_Algo algo)
    {
        try
        {
            var names = new List<string>();
            foreach (var alert in algo.GetReport().GetAlerts(Message_Gravity.Message_Fail))
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

        public long Of(TopoDS_Shape shape)
        {
            if (!shape.IsNull())
                Visit(shape);
            return _bytes;
        }

        private bool First(object? o) => o is not null && _seen.Add(o);

        private void Visit(TopoDS_Shape shape)
        {
            var tshape = shape.TShape();
            if (!First(tshape))
                return;
            _bytes += 96;
            if (tshape is BRep_TEdge tedge)
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
            else if (tshape is BRep_TFace tface)
            {
                Surface(tface.Surface());
                foreach (var tri in tface.TriangulationList())
                    if (First(tri))
                        _bytes += (long)tri.NbNodes() * (24 + (tri.HasUVNodes() ? 16 : 0) + (tri.HasNormals() ? 12 : 0)) + 12L * tri.NbTriangles();
            }
            for (var it = new TopoDS_Iterator(shape, false, false); it.More(); it.Next())
                Visit(it.Value());
        }

        private void Curve(Geom_Curve? c)
        {
            if (!First(c))
                return;
            _bytes += 80;
            switch (c)
            {
                case Geom_BSplineCurve b: _bytes += 32L * b.NbPoles() + 12L * b.NbKnots(); break;
                case Geom_BezierCurve z: _bytes += 32L * z.NbPoles(); break;
                case Geom_TrimmedCurve t: Curve(t.BasisCurve()); break;
                case Geom_OffsetCurve o: Curve(o.BasisCurve()); break;
            }
        }

        private void Curve2d(Geom2d_Curve? c)
        {
            if (!First(c))
                return;
            _bytes += 64;
            switch (c)
            {
                case Geom2d_BSplineCurve b: _bytes += 24L * b.NbPoles() + 12L * b.NbKnots(); break;
                case Geom2d_BezierCurve z: _bytes += 24L * z.NbPoles(); break;
                case Geom2d_TrimmedCurve t: Curve2d(t.BasisCurve()); break;
                case Geom2d_OffsetCurve o: Curve2d(o.BasisCurve()); break;
            }
        }

        private void Surface(Geom_Surface? s)
        {
            if (!First(s))
                return;
            _bytes += 120;
            switch (s)
            {
                case Geom_BSplineSurface b: _bytes += 32L * b.NbUPoles() * b.NbVPoles() + 12L * (b.NbUKnots() + b.NbVKnots()); break;
                case Geom_BezierSurface z: _bytes += 32L * z.NbUPoles() * z.NbVPoles(); break;
                case Geom_RectangularTrimmedSurface t: Surface(t.BasisSurface()); break;
                case Geom_OffsetSurface o: Surface(o.BasisSurface()); break;
                case Geom_SweptSurface w: Curve(w.BasisCurve()); break;
            }
        }
    }
}
