// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;

namespace Kerf.Engine.Meshing;

/// <summary>
/// For a sphere, whose grid alone OCCT trusts (decision 1953): after the grid's nodes, measures
/// each triangle's and link's distance from the sphere exactly, and inserts a node at the point of the sphere nearest the
/// worst point of each that is further than the deflection asked, for up to 11 passes. The cause of a sphere's misses was
/// the scaling of its parameters, which <see cref="SphereRangeSplitter"/> now takes at the latitude nearest the equator;
/// this is the check that the grid then meets the deflection, and adds nothing where it does, as on every sphere surveyed.
/// OCCT's control tests a parameter midpoint, which overstates the distance near the poles and refines a sphere already
/// within it. At a relative deflection the limit is the face's own, as the torus's (decision 2056).
/// </summary>
internal sealed class SphereDeflectionMeshAlgo : DelaunayNodeInsertionMeshAlgo<SphereRangeSplitter>
{
    /// <inheritdoc/>
    protected override void PostProcessMesh(Delaun mesher, ProgressRange range)
    {
        using ProgressScope ps = new(range, "Post process mesh", 2);
        base.PostProcessMesh(mesher, ps.Next());
        if (!ps.More())
        {
            return;
        }

        if (GetParameters().ControlSurfaceDeflection && GetStructure().ElementsOfDomain().Extent() > 0)
        {
            OptimizeMesh(mesher, ps.Next());
        }
        else
        {
            ps.Next();
        }
    }

    private void OptimizeMesh(Delaun mesher, ProgressRange range)
    {
        Sphere3D sphere = GetDFace().GetSurface().Sphere();
        Point3D centre = sphere.Location;
        double radius = sphere.Radius;
        double deflection = Deflection.SurfaceLimit(GetDFace(), GetParameters());
        double minSize = GetParameters().MinSize;
        double sqMinSize = minSize * minSize;
        double angle = GetParameters().AngleInterior;
        var links = new LinkMarks();
        var nodes = new NCollection_List<Point2D>();
        Span<int> indices = stackalloc int[3];
        Span<Point3D> points = stackalloc Point3D[3];
        Span<Point2D> uvs = stackalloc Point2D[3];
        Span<Point2D> scaled = stackalloc Point2D[3];
        Span<bool> frontier = stackalloc bool[3];
        double worst = -1;

        const int passes = 11;
        using ProgressScope ps = new(range, "Iteration", passes);
        bool inserted = true;
        for (int pass = 1; pass <= passes && inserted; ++pass)
        {
            if (!ps.More())
            {
                return;
            }

            worst = -1;
            links.Reset(GetStructure().LinkCount);
            nodes.Clear();
            for (NCollection_PackedMapOfInteger.Iterator it = new(GetStructure().ElementsOfDomain()); it.More(); it.Next())
            {
                Triangle triangle = GetStructure().GetElement(it.Key());
                if (triangle.Movability() == DegreeOfFreedom.Deleted)
                {
                    continue;
                }

                GetStructure().ElementNodes(triangle, indices);
                for (int i = 0; i < 3; i++)
                {
                    MeshVertex vertex = GetStructure().GetNode(indices[i]);
                    scaled[i] = new Point2D(vertex.Coord());
                    uvs[i] = GetRangeSplitter().Scale(scaled[i], false);
                    points[i] = GetNodesMap()[vertex.Location3d()];
                }

                Triangle.Int3 edges = triangle._edges;
                for (int i = 0; i < 3; i++)
                {
                    frontier[i] = GetStructure().GetLink(edges[i]).Movability() == DegreeOfFreedom.Frontier;
                }

                // The triangle's nearest point to the centre, where it is furthest inside the sphere, when that is inside it
                Vector3D normal = new Vector3D(points[0], points[1]).Crossed(new Vector3D(points[0], points[2]));
                double sqNormal = normal.SquareMagnitude;
                if (sqNormal > 1e-24)
                {
                    double along = normal.Dot(new Vector3D(points[0], centre)) / sqNormal;
                    Point3D foot = centre.Translated(normal.Multiplied(-along));
                    if (Inside(points, foot, normal))
                    {
                        double distance = radius - centre.DistanceTo(foot);
                        worst = Math.Max(worst, distance);
                        if (distance > deflection && Use(sphere, centre, foot, uvs, points, sqMinSize, out Point2D uv))
                        {
                            // Off a frontier link, where Delaunay would skip it (decision 2057)
                            Point2D node = GetRangeSplitter().Scale(uv, true);
                            Point2D cleared = FrontierClearance.Clear(node, scaled, frontier);
                            nodes.Append(cleared.X == node.X && cleared.Y == node.Y ? uv : GetRangeSplitter().Scale(cleared, false));
                        }
                    }
                }

                // Its links, each once (a link's index is its pair of nodes while the structure doesn't change): the chord's
                // middle is its furthest point from the sphere
                for (int i = 0; i < 3; i++)
                {
                    if (frontier[i])
                    {
                        continue;
                    }

                    int j = (i + 1) % 3;
                    if (!links.Add(Math.Abs(edges[i])))
                    {
                        continue;
                    }

                    var middle = new Point3D((points[i].X + points[j].X) / 2, (points[i].Y + points[j].Y) / 2, (points[i].Z + points[j].Z) / 2);
                    double sagitta = radius - centre.DistanceTo(middle);
                    worst = Math.Max(worst, sagitta);
                    bool split = sagitta > deflection;
                    if (!split && angle > 0)
                    {
                        // As OCCT's control: the normals at its ends, which on a sphere are the directions from the centre
                        Vector3D a = new(centre, points[i]), b = new(centre, points[j]);
                        split = a.SquareMagnitude > 0 && b.SquareMagnitude > 0 && a.AngleTo(b) > angle;
                    }

                    if (split)
                    {
                        Span<Point2D> ends = [uvs[i], uvs[j]];
                        Span<Point3D> ends3d = [points[i], points[j]];
                        if (Use(sphere, centre, middle, ends, ends3d, sqMinSize, out Point2D uv))
                        {
                            nodes.Append(uv);
                        }
                    }
                }
            }

            inserted = InsertNodes(nodes, mesher, ps.Next());
        }

        if (worst >= 0)
        {
            GetDFace().SetDeflection(worst);
        }
    }

    // Whether the point, in the triangle's plane, is inside the triangle
    private static bool Inside(ReadOnlySpan<Point3D> points, in Point3D point, in Vector3D normal)
    {
        for (int i = 0; i < 3; i++)
        {
            Point3D a = points[i], b = points[(i + 1) % 3];
            if (new Vector3D(a, b).Crossed(new Vector3D(a, point)).Dot(normal) < 0)
            {
                return false;
            }
        }

        return true;
    }

    // The parameters of the sphere's point nearest the given one, its u taken to the period of the points', unless it
    // would come nearer to one of them than the smallest size
    private static bool Use(in Sphere3D sphere, in Point3D centre, in Point3D point, ReadOnlySpan<Point2D> uvs,
                            ReadOnlySpan<Point3D> points, double sqMinSize, out Point2D node)
    {
        node = default;
        if (centre.SquareDistanceTo(point) < 1e-24)
        {
            return false;
        }

        Point3D onSphere = centre.Translated(new Vector3D(centre, point).Normalized().Multiplied(sphere.Radius));
        for (int i = 0; i < points.Length; i++)
        {
            if (onSphere.SquareDistanceTo(points[i]) < sqMinSize)
            {
                return false;
            }
        }

        ElSLib.Parameters(sphere, onSphere, out double u, out double v);
        double reference = 0;
        for (int i = 0; i < uvs.Length; i++)
        {
            reference += uvs[i].X;
        }

        reference /= uvs.Length;
        u += 2 * Math.PI * Math.Round((reference - u) / (2 * Math.PI));
        node = new Point2D(u, v);
        return true;
    }
}
