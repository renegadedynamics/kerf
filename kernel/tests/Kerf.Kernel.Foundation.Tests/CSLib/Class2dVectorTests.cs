// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Class2d = Kerf.Engine.Geometry.Class2d;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// Unit 14E: the point-in-polygon tests by vectors of four edges give the scalar loops' answers, point for point: on
/// random polygons, convex and not, of 3 to 300 points, with points at and beside their vertices and edges, on grids
/// through them, and at random.
/// </summary>
public class Class2dVectorTests
{
    [Fact]
    public void VectorAndScalarAgree()
    {
        var random = new Random(1405);
        int compared = 0, inside = 0, uncertain = 0;
        foreach (int count in (int[])[3, 4, 5, 7, 8, 9, 11, 12, 13, 16, 17, 31, 64, 65, 100, 299])
        {
            for (int polygon = 0; polygon < 12; polygon++)
            {
                (Class2d classifier, List<Point2D> points) = Polygon(random, count, polygon);
                foreach (Point2D p in Queries(random, points))
                {
                    bool scalar = classifier.InternalSiDansScalar(p.X, p.Y);
                    Assert.Equal(scalar, classifier.InternalSiDansVector(p.X, p.Y));
                    Class2d.Result onScalar = classifier.InternalSiDansOuOnScalar(p.X, p.Y);
                    Assert.Equal(onScalar, classifier.InternalSiDansOuOnVector(p.X, p.Y));
                    compared++;
                    inside += scalar ? 1 : 0;
                    uncertain += onScalar == Class2d.Result.Result_Uncertain ? 1 : 0;
                }
            }
        }

        // Each answer was met many times
        Assert.True(compared > 50_000 && inside > compared / 10 && uncertain > compared / 20, $"{compared} {inside} {uncertain}");
    }

    // A polygon in the unit square, as the classifier holds it (normalized): a star of random radii, a circle, or a
    // zigzag with many vertical and horizontal edges and repeated points; tolerances from none to large
    private static (Class2d, List<Point2D>) Polygon(Random random, int count, int kind)
    {
        var points = new List<Point2D>();
        for (int i = 0; i < count; i++)
        {
            double angle = 2 * Math.PI * i / count;
            switch (kind % 3)
            {
                case 0:
                    double r = 0.1 + 0.4 * random.NextDouble();
                    points.Add(new Point2D(0.5 + r * Math.Cos(angle), 0.5 + r * Math.Sin(angle)));
                    break;
                case 1:
                    points.Add(new Point2D(0.5 + 0.5 * Math.Cos(angle), 0.5 + 0.5 * Math.Sin(angle)));
                    break;
                default:
                    // On a coarse grid, so that edges are vertical, horizontal or repeated
                    points.Add(new Point2D(Math.Round(8 * random.NextDouble()) / 8, Math.Round(8 * random.NextDouble()) / 8));
                    break;
            }
        }

        double tol = (kind / 3) switch { 0 => 0.0, 1 => 1e-12, 2 => 1e-4, _ => 0.02 };
        return (new Class2d(points, tol, tol, 0.0, 0.0, 1.0, 1.0), points);
    }

    private static IEnumerable<Point2D> Queries(Random random, List<Point2D> points)
    {
        for (int i = 0; i < points.Count; i++)
        {
            Point2D a = points[i], b = points[(i + 1) % points.Count];
            yield return a;
            yield return new Point2D(0.5 * (a.X + b.X), 0.5 * (a.Y + b.Y));
            yield return new Point2D(a.X + 1e-9, a.Y);
            yield return new Point2D(a.X, a.Y - 1e-5);
            yield return new Point2D(a.X - 0.01, a.Y + 0.01);
            yield return new Point2D(a.X + 0.3 * (b.X - a.X), a.Y + 0.3 * (b.Y - a.Y));
        }

        for (int i = 0; i <= 16; i++)
        {
            for (int j = 0; j <= 16; j++)
            {
                yield return new Point2D(i / 16.0, j / 16.0);
            }
        }

        for (int i = 0; i < 60; i++)
        {
            yield return new Point2D(1.2 * random.NextDouble() - 0.1, 1.2 * random.NextDouble() - 0.1);
        }
    }
}
