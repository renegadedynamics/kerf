// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Geometry;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Drawing;

/// <summary>What a projected edge is, as OCCT's hidden-line removal classifies it (<c>HLRBRep_TypeOfResultingEdge</c>).</summary>
public enum LineKind
{
    /// <summary>An edge where two faces meet at an angle, or a free edge.</summary>
    Sharp,

    /// <summary>An edge where two faces meet tangentially (G1), such as a fillet's boundary.</summary>
    Smooth,

    /// <summary>An edge inside one smooth surface (CN), such as a cylinder's seam.</summary>
    Sewn,

    /// <summary>A silhouette: where a curved face turns away from the view. It isn't an edge of the shape.</summary>
    Outline,

    /// <summary>An isoparametric line of a face, drawn to show its shape (<see cref="DrawingOptions.IsoLineCount"/>).</summary>
    IsoLine,
}

/// <summary>The hidden-line algorithm (OCCT's two).</summary>
public enum HiddenLineMethod
{
    /// <summary>OCCT's <c>HLRBRep_Algo</c> on the exact geometry: slower, but the lines are true curves (lines, arcs,
    /// ellipses, B-splines).</summary>
    Exact,

    /// <summary>OCCT's <c>HLRBRep_PolyAlgo</c> on a mesh of the shapes: faster on large models, but every line is a
    /// polyline, as fine as the mesh. Orthographic cameras only.</summary>
    Polygonal,
}

/// <summary>How <see cref="DrawingOperations.ProjectLines{TShape}"/> projects.</summary>
public sealed record DrawingOptions
{
    /// <summary>The exact algorithm, hidden lines kept, no isolines.</summary>
    public static DrawingOptions Default { get; } = new();

    /// <summary>The hidden-line algorithm, exact by default.</summary>
    public HiddenLineMethod Method { get; init; } = HiddenLineMethod.Exact;

    /// <summary>Whether the hidden lines are in the drawing as well as the visible ones.</summary>
    public bool IncludeHidden { get; init; } = true;

    /// <summary>The number of isolines drawn in each direction of each face (exact algorithm only); zero for none.</summary>
    public int IsoLineCount
    {
        get;
        init => field = value >= 0 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The count must be zero or more.");
    }

    /// <summary>
    /// How far a projected curve that is neither a line, an arc, an ellipse nor a polynomial spline (a circle seen in
    /// perspective, an edge seen edge-on) may be from the polyline drawn for it, in model units; null (the default) for a
    /// two-thousandth of the shapes' size.
    /// </summary>
    public double? Tolerance
    {
        get;
        init => field = value is null or > 0 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The tolerance must be more than zero.");
    }

    /// <summary>The mesh of the polygonal algorithm; null for a mesh whose deflection is a thousandth of the shapes'
    /// size.</summary>
    public MeshOptions? Mesh { get; init; }
}

/// <summary>
/// A projected edge of a <see cref="LineDrawing"/>: a bounded 2D curve in the view plane (X right, Y up), its kind,
/// whether it is visible, and the sub-shape it comes from.
/// </summary>
/// <param name="Curve">The curve, bounded: from its <c>FirstParameter</c> to its <c>LastParameter</c>. Lines, arcs and
/// ellipses are <see cref="TrimmedCurve2D"/>s of <see cref="LineCurve2D"/>, <see cref="CircleCurve2D"/> and
/// <see cref="EllipseCurve2D"/>; splines are <see cref="BSplineCurve2D"/> or <see cref="BezierCurve2D"/>; polylines are
/// B-splines of degree 1.</param>
/// <param name="Kind">The kind of line.</param>
/// <param name="IsVisible">True for a visible line, false for a hidden one.</param>
/// <param name="Source">The edge the line is a part of (a sharp, smooth or sewn line), or the face it is drawn on (an
/// outline or an isoline); a null shape where the engine doesn't know it (a line on an edge the outliner split).</param>
public sealed record DrawnEdge(Curve2D Curve, LineKind Kind, bool IsVisible, Shape Source)
{
    /// <summary>The curve's start.</summary>
    public Point2D Start => Curve.Evaluate(Curve.FirstParameter);

    /// <summary>The curve's end.</summary>
    public Point2D End => Curve.Evaluate(Curve.LastParameter);
}

/// <summary>
/// A line drawing: shapes projected by a <see cref="Camera"/>, as visible and hidden 2D edges (OCCT's hidden-line
/// removal; docs/api/drawing.md). <see cref="Svg"/> writes one.
/// </summary>
/// <remarks>The edges are in the engine's order: by kind (sharp, smooth, sewn, outline, isoline), visible before hidden,
/// then by the shapes' faces and edges. The same shapes and options always give the same drawing.</remarks>
public sealed class LineDrawing
{
    /// <summary>A drawing of the given edges, as seen by the camera (for drawings made or completed by hand).</summary>
    /// <exception cref="ArgumentNullException">The camera or the edges are null, or an edge is.</exception>
    public LineDrawing(Camera camera, IEnumerable<DrawnEdge> edges)
    {
        KernelArgumentNullException.ThrowIfNull(camera);
        KernelArgumentNullException.ThrowIfNull(edges);
        Camera = camera;
        Edges = [.. edges];
        foreach (DrawnEdge edge in Edges)
        {
            if (edge is null)
            {
                throw new KernelArgumentNullException(nameof(edges), "An edge is null.");
            }
        }
    }

    /// <summary>The camera.</summary>
    public Camera Camera { get; }

    /// <summary>The edges, visible and hidden.</summary>
    public ImmutableArray<DrawnEdge> Edges { get; }

    /// <summary>The visible edges.</summary>
    public IEnumerable<DrawnEdge> Visible => Edges.Where(e => e.IsVisible);

    /// <summary>The hidden edges.</summary>
    public IEnumerable<DrawnEdge> Hidden => Edges.Where(e => !e.IsVisible);

    /// <summary>The edges of a kind, visible or hidden.</summary>
    public IEnumerable<DrawnEdge> Of(LineKind kind, bool visible) => Edges.Where(e => e.Kind == kind && e.IsVisible == visible);

    /// <summary>The drawing's extent in the view plane, from points sampled on its edges; null for an empty drawing.</summary>
    /// <param name="includeHidden">Whether the hidden edges count.</param>
    public (Point2D Min, Point2D Max)? GetBounds(bool includeHidden = true)
    {
        double xMin = double.PositiveInfinity, yMin = double.PositiveInfinity, xMax = double.NegativeInfinity, yMax = double.NegativeInfinity;
        foreach (DrawnEdge edge in Edges)
        {
            if (!edge.IsVisible && !includeHidden)
            {
                continue;
            }

            Curve2D curve = edge.Curve;
            double u0 = curve.FirstParameter, u1 = curve.LastParameter;
            int samples = curve is TrimmedCurve2D { BasisCurve: LineCurve2D } ? 1 : 64;
            for (int i = 0; i <= samples; i++)
            {
                Point2D p = curve.Evaluate(i == samples ? u1 : u0 + (u1 - u0) * i / samples);
                xMin = Math.Min(xMin, p.X);
                yMin = Math.Min(yMin, p.Y);
                xMax = Math.Max(xMax, p.X);
                yMax = Math.Max(yMax, p.Y);
            }
        }

        return xMin <= xMax ? (new Point2D(xMin, yMin), new Point2D(xMax, yMax)) : null;
    }
}
