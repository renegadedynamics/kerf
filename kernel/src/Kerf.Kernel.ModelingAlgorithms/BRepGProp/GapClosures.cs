// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Buffers;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The arcs that close a face's boundary in its parameters where it's open inside a vertex's tolerance (decision 2257).
/// </summary>
/// <remarks>
/// A vertex whose tolerance has grown covers the ends of the edges that meet at it, which is all validity asks; their curves
/// on the face can still end apart, by up to twice the tolerance. The integrals over a face are taken round its boundary in
/// its parameters, and read such a gap as part of the face missing or added: the gap's length times how far it is from the
/// face's lowest U. The mesher and the classifiers join the two ends straight, in the parameters, and so does each arc here:
/// a line from where one edge's curve ends to where the next one's starts, as an edge with that curve on the face and
/// nothing else.
/// </remarks>
internal static class GapClosures
{
    // Where an edge's curve on the face arrives at a vertex or leaves it
    private struct End(Vertex vertex, bool arrives, Point2D at)
    {
        public readonly Vertex Vertex = vertex;
        public readonly bool Arrives = arrives;
        public readonly Point2D At = at;
        public bool Paired;
    }

    /// <summary>
    /// The closing arcs of <paramref name="face"/>, forward, or null if it has none: no vertex past the default tolerance,
    /// or no gap longer than that.
    /// </summary>
    public static List<Edge>? Of(in Face face)
    {
        double confusion = global::Kerf.Engine.Foundation.Precision.Confusion();
        Face forward = ShapeCasts.Face(face.Oriented(Forward));
        // Rented, as nearly every face with a grown vertex has no gap and so nothing to keep
        End[]? ends = null;
        int count = 0;
        try
        {
            for (var edges = new Explorer(forward, ShapeType.Edge); edges.More(); edges.Next())
            {
                Edge edge = ShapeCasts.Edge(edges.Current());
                bool reversed = edge.Orientation == Reversed;
                if (!reversed && edge.Orientation != Forward)
                {
                    continue;
                }

                Exploration.Vertices(edge, out Vertex start, out Vertex end, cumOri: true);
                bool startGrown = !start.IsNull && Tool.Tolerance(start) > confusion;
                bool endGrown = !end.IsNull && Tool.Tolerance(end) > confusion;
                if (!startGrown && !endGrown)
                {
                    continue;
                }

                Curve2D? curve = Tool.CurveOnSurfaceToRead(edge, forward, out double first, out double last);
                if (curve is null)
                {
                    continue;
                }

                ends ??= ArrayPool<End>.Shared.Rent(32);
                if (count + 2 > ends.Length)
                {
                    End[] longer = ArrayPool<End>.Shared.Rent(2 * ends.Length);
                    Array.Copy(ends, longer, count);
                    ArrayPool<End>.Shared.Return(ends, clearArray: true);
                    ends = longer;
                }

                if (startGrown)
                {
                    ends[count++] = new End(start, false, curve.Value(reversed ? last : first));
                }

                if (endGrown)
                {
                    ends[count++] = new End(end, true, curve.Value(reversed ? first : last));
                }
            }

            if (ends is null || Tool.Surface(face, out Placement placement) is not { } surface)
            {
                return null;
            }

            List<Edge>? closures = null;
            Span<End> all = ends.AsSpan(0, count);
            for (int i = 0; i < all.Length; i++)
            {
                if (all[i].Paired)
                {
                    continue;
                }

                // Each arrival at this vertex with the departure nearest it along the surface, the nearest pair first
                Vertex vertex = all[i].Vertex;
                while (true)
                {
                    int bestIn = -1, bestOut = -1, candidates = 0;
                    double best = double.MaxValue;
                    for (int a = i; a < all.Length; a++)
                    {
                        if (all[a].Paired || !all[a].Arrives || !all[a].Vertex.IsSame(vertex))
                        {
                            continue;
                        }

                        for (int d = i; d < all.Length; d++)
                        {
                            if (all[d].Paired || all[d].Arrives || !all[d].Vertex.IsSame(vertex))
                            {
                                continue;
                            }

                            candidates++;
                            double along = candidates == 1 ? 0.0 : Along(surface, all[a].At, all[d].At);
                            if (candidates == 2)
                            {
                                // The first pair wasn't measured, as one alone needn't be
                                best = Along(surface, all[bestIn].At, all[bestOut].At);
                            }

                            if (along < best)
                            {
                                (best, bestIn, bestOut) = (along, a, d);
                            }
                        }
                    }

                    if (bestIn < 0)
                    {
                        break;
                    }

                    all[bestIn].Paired = true;
                    all[bestOut].Paired = true;
                    if (Arc(surface, placement, vertex, all[bestIn].At, all[bestOut].At, confusion) is { } arc)
                    {
                        (closures ??= []).Add(arc);
                    }
                }

                // An end with nothing to pair with at its vertex stays as it is
                all[i].Paired = true;
            }

            return closures;
        }
        finally
        {
            if (ends is not null)
            {
                ArrayPool<End>.Shared.Return(ends, clearArray: true);
            }
        }
    }

    // The arc from one end to the other, or null: not a gap worth an arc; or not one this vertex covers, as the two sides of
    // a seam aren't: a period apart in the parameters, and the line between them is round the surface
    private static Edge? Arc(Surface surface, in Placement placement, in Vertex vertex, in Point2D from, in Point2D to, double confusion)
    {
        double length = from.DistanceTo(to);
        if (length <= 0.0)
        {
            return null;
        }

        Point3D a = On(surface, placement, from), b = On(surface, placement, to);
        if (a.DistanceTo(b) <= confusion)
        {
            return null;
        }

        double tolerance = Tool.Tolerance(vertex), reach = 2.0 * tolerance + confusion;
        Point3D at = Tool.Pnt(vertex);
        Point3D middle = On(surface, placement, new Point2D((from.X + to.X) / 2.0, (from.Y + to.Y) / 2.0));
        if (a.DistanceTo(at) > reach || b.DistanceTo(at) > reach || middle.DistanceTo(at) > reach)
        {
            return null;
        }

        var line = new LineCurve2D(from, new Direction2D(to.X - from.X, to.Y - from.Y));
        var builder = new BrepBuilder();
        Edge arc = new();
        builder.MakeEdge(ref arc);
        builder.UpdateEdge(arc, line, surface, placement, tolerance);
        builder.Range(arc, 0.0, length);
        return arc;
    }

    // The length in space of the line between two points of the parameters, through its middle
    private static double Along(Surface surface, in Point2D from, in Point2D to)
    {
        Point3D a = surface.Value(from.X, from.Y), b = surface.Value(to.X, to.Y);
        Point3D middle = surface.Value((from.X + to.X) / 2.0, (from.Y + to.Y) / 2.0);
        return a.DistanceTo(middle) + middle.DistanceTo(b);
    }

    private static Point3D On(Surface surface, in Placement placement, in Point2D at)
    {
        Point3D point = surface.Value(at.X, at.Y);
        return placement.IsIdentity ? point : point.Transformed(placement.Transformation);
    }
}
