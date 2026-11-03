using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
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
    public static bool? EdgeCurve3DIsPlanar(global::Kerf.Topology.Edge edge, Xyz normal, double tolerance)
    {
        if (edge.TShape() is not BrepTEdge tedge)
            return null;
        foreach (var rep in tedge.CurveRepresentations())
            if (rep.IsCurve3D)
            {
                var n = normal;   // OCCT computes a normal into a null one; the caller's stays as it is
                return ShapeAnalysis_Curve.IsPlanar(rep.Curve3D()!, ref n, tolerance);
            }
        return null;
    }

    /// <summary>A shape in OCCT's BRep text format, with its triangulations if <paramref name="withTriangles"/>.</summary>
    public static byte[] WriteBRep(global::Kerf.Topology.Shape shape, bool withTriangles = false)
    {
        var writer = new StringWriter(System.Globalization.CultureInfo.InvariantCulture) { NewLine = "\n" };
        BrepTools.Write(shape, writer, withTriangles, false, FormatVersion.FormatVersion_CURRENT);
        return Encoding.ASCII.GetBytes(writer.ToString());
    }

    /// <summary>
    /// A shape read from OCCT's BRep text format. Data that isn't BRep, or is truncated or malformed, throws
    /// <see cref="StreamFormatErrorException"/>.
    /// </summary>
    public static global::Kerf.Topology.Shape ReadBRep(ReadOnlySpan<byte> data)
    {
        // The native reader overran on long words and looped at the end of truncated data, so the native binding checked
        // first; OCCT.NET's reader is safe, but the same input is refused the same way
        const int longestWord = 200;
        int word = 0;
        foreach (var c in data)
        {
            word = c is (byte)' ' or (byte)'\t' or (byte)'\n' or (byte)'\r' or (byte)'\v' or (byte)'\f' ? 0 : word + 1;
            if (word >= longestWord)
                throw new StreamFormatErrorException("BRep data has a word too long to be BRep");
            if (c == 0)
                throw new StreamFormatErrorException("BRep data is text, without NULs");
        }
        var text = Encoding.ASCII.GetString(data);
        global::Kerf.Topology.Shape shape;
        try
        {
            var reader = new StringReader(text);
            StreamUtilities.SetInputLength(reader, text.Length);
            BrepTools.Read(out shape, reader);
        }
        catch (FailureException e) when (e is not StreamFormatErrorException)
        {
            throw new StreamFormatErrorException($"BRep data is truncated or malformed ({e.OcctType})");
        }
        catch (Exception e) when (e is FormatException or EndOfStreamException or OverflowException or IndexOutOfRangeException)
        {
            throw new StreamFormatErrorException("BRep data is truncated or malformed");
        }
        if (shape.IsNull)
            throw new StreamFormatErrorException("BRep data holds no shape");
        return shape;
    }

    /// <summary>
    /// A rough estimate of the memory a shape holds, in bytes: its sub-shapes, curves, surfaces and triangulations,
    /// each counted once however often it's shared (the native shim's estimate, for the same shapes).
    /// </summary>
    public static long EstimateMemory(global::Kerf.Topology.Shape shape)
    {
        try
        {
            return new MemoryEstimate().Of(shape);
        }
        catch (FailureException)
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
            foreach (var alert in algo.GetReport().GetAlerts(Gravity.Fail))
            {
                var name = alert.DynamicTypeName();
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

        public long Of(global::Kerf.Topology.Shape shape)
        {
            if (!shape.IsNull)
                Visit(shape);
            return _bytes;
        }

        private bool First(object? o) => o is not null && _seen.Add(o);

        private void Visit(global::Kerf.Topology.Shape shape)
        {
            var tshape = shape.TShape();
            if (!First(tshape))
                return;
            _bytes += 96;
            if (tshape is BrepTEdge tedge)
                foreach (var rep in tedge.CurveRepresentations())
                {
                    _bytes += 64;
                    if (rep.IsCurve3D)
                        Curve(rep.Curve3D());
                    else if (rep.IsCurveOnSurface())
                    {
                        Curve2d(rep.PCurve());
                        if (rep.IsCurveOnClosedSurface)
                            Curve2d(rep.PCurve2());
                    }
                    else if (rep.IsPolygon3D && rep.Polygon3D() is { } p && First(p))
                        _bytes += 32L * p.NodeCount;
                    else if (rep.IsPolygonOnTriangulation() && rep.PolygonOnTriangulation() is { } pt && First(pt))
                        _bytes += 12L * pt.NodeCount;
                }
            else if (tshape is BrepTFace tface)
            {
                Surface(tface.Surface());
                foreach (var tri in tface.TriangulationList())
                    if (First(tri))
                        _bytes += (long)tri.NodeCount * (24 + (tri.HasUVNodes ? 16 : 0) + (tri.HasNormals ? 12 : 0)) + 12L * tri.TriangleCount;
            }
            for (var it = new Iterator(shape, false, false); it.More(); it.Next())
                Visit(it.Value());
        }

        private void Curve(Curve3D? c)
        {
            if (!First(c))
                return;
            _bytes += 80;
            switch (c)
            {
                case BSplineCurve3D b: _bytes += 32L * b.PoleCount + 12L * b.KnotCount; break;
                case BezierCurve3D z: _bytes += 32L * z.PoleCount; break;
                case TrimmedCurve3D t: Curve(t.BasisCurve()); break;
                case OffsetCurve3D o: Curve(o.BasisCurve()); break;
            }
        }

        private void Curve2d(Curve2D? c)
        {
            if (!First(c))
                return;
            _bytes += 64;
            switch (c)
            {
                case BSplineCurve2D b: _bytes += 24L * b.PoleCount + 12L * b.KnotCount; break;
                case BezierCurve2D z: _bytes += 24L * z.PoleCount; break;
                case TrimmedCurve2D t: Curve2d(t.BasisCurve()); break;
                case OffsetCurve2D o: Curve2d(o.BasisCurve()); break;
            }
        }

        private void Surface(Surface? s)
        {
            if (!First(s))
                return;
            _bytes += 120;
            switch (s)
            {
                case BSplineSurface b: _bytes += 32L * b.UPoleCount * b.VPoleCount + 12L * (b.UKnotCount + b.VKnotCount); break;
                case BezierSurface z: _bytes += 32L * z.UPoleCount * z.VPoleCount; break;
                case TrimmedSurface t: Surface(t.BasisSurface()); break;
                case OffsetSurface o: Surface(o.BasisSurface()); break;
                case SweptSurface w: Curve(w.BasisCurve()); break;
            }
        }
    }
}
