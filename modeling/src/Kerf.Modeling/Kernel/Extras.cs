using Kerf.Exchange;

namespace Kerf.Modeling;

/// <summary>
/// The modelling layer's helpers over the kernel's public API that build123d's code needs in its own terms: BRep text as bytes, read guardedly, and an edge's own curve.
/// </summary>
internal static class Extras
{
    // A BRep reader's failure, as Kerf.Exchange reports one
    private static readonly DiagnosticCode Malformed = new("exchange.malformed");

    /// <summary>
    /// Whether the edge's own 3D curve, unplaced, lies in a plane normal to <paramref name="normal"/>
    /// (<c>ShapeAnalysis_Curve::IsPlanar</c> on the edge's curve), or null if it has none.
    /// </summary>
    public static bool? EdgeCurve3DIsPlanar(Edge edge, Direction3D normal, double tolerance) =>
        edge.TryGetLocalCurve(out var curve, out _, out _) ? curve.IsPlanar(normal, tolerance) : null;

    /// <summary>A shape in OCCT's BRep text format, with its triangulations if <paramref name="withTriangles"/>.</summary>
    public static byte[] WriteBRep(Shape shape, bool withTriangles = false)
    {
        using var stream = new MemoryStream();
        Failures.Check(Brep.Write(shape, stream, new BrepWriteOptions { WithTriangles = withTriangles }), () => "write BRep");
        return stream.ToArray();
    }

    /// <summary>
    /// The shape of BRep text, refused (a failure, <c>exchange.malformed</c>) if it holds NULs or a word too long to be
    /// BRep, or if <see cref="Brep.Read"/> finds it truncated or malformed.
    /// </summary>
    public static Result<Shape> ReadBRep(ReadOnlySpan<byte> data)
    {
        // The native reader overran on long words and looped at the end of truncated data, so the native binding checked
        // first; the kernel's reader is safe, but the same input is refused the same way
        const int longestWord = 200;
        int word = 0;
        foreach (var c in data)
        {
            word = c is (byte)' ' or (byte)'\t' or (byte)'\n' or (byte)'\r' or (byte)'\v' or (byte)'\f' ? 0 : word + 1;
            if (word >= longestWord)
                return Refused("BRep data has a word too long to be BRep");
            if (c == 0)
                return Refused("BRep data is text, without NULs");
        }
        using var stream = new MemoryStream(data.ToArray(), writable: false);
        return Brep.Read(stream);
    }

    private static Result<Shape> Refused(string message) =>
        Result<Shape>.Failure(Diagnostic.Error(Malformed, message));
}
