// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Kerf: where to put a node that splits a triangle on a frontier link (decision 2057). Delaunay skips a free node nearer
/// a frontier link than a fifth of the link's length (<see cref="Delaun.Contains"/>), so a node meant for such a triangle,
/// at its centre or nearest the surface, may never go in, and the triangle stays further from the surface than the
/// deflection. Moved along the line from the link's middle to the triangle's third node, to 0.3 of the link's length from
/// the link, it goes in, and the triangle it makes on the link is obtuse there, so no further from the surface than the
/// link itself, which the edge's own deflection set.
/// </summary>
internal static class FrontierClearance
{
    // Within this share of a frontier link's length from it, a node is moved (Delaunay's limit is a fifth)
    private const double Near = 0.25;

    // How far from the link a moved node goes, as a share of its length (under a half, so the triangle is obtuse there)
    private const double Height = 0.3;

    /// <summary>
    /// The node, in the face's scaled parameters, or the point it is moved to if it is near a frontier link of the
    /// triangle: <paramref name="corners"/> are its nodes, and link i (of nodes i and i + 1) is a frontier if
    /// <paramref name="frontier"/>[i] is.
    /// </summary>
    public static Point2D Clear(in Point2D node, ReadOnlySpan<Point2D> corners, ReadOnlySpan<bool> frontier)
    {
        for (int i = 0; i < 3; i++)
        {
            if (!frontier[i])
            {
                continue;
            }

            Point2D a = corners[i], b = corners[(i + 1) % 3], o = corners[(i + 2) % 3];
            double lx = b.X - a.X, ly = b.Y - a.Y;
            double length = Math.Sqrt(lx * lx + ly * ly);
            if (length <= 0)
            {
                continue;
            }

            double height = Math.Abs(lx * (node.Y - a.Y) - ly * (node.X - a.X)) / length;
            if (height >= Near * length)
            {
                continue;
            }

            double third = Math.Abs(lx * (o.Y - a.Y) - ly * (o.X - a.X)) / length;
            if (third <= Height * length)
            {
                // The triangle is already obtuse at its third node, or nearly
                return node;
            }

            double mx = (a.X + b.X) / 2, my = (a.Y + b.Y) / 2, s = Height * length / third;
            return new Point2D(mx + s * (o.X - mx), my + s * (o.Y - my));
        }

        return node;
    }
}
