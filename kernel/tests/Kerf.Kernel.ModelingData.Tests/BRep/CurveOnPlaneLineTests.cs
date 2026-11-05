// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Unit 11B's short cut in <c>BRep_Tool::CurveOnPlane</c> for a line in the plane has to give, to the bit, what OCCT's
/// projection gives.
/// </summary>
public class CurveOnPlaneLineTests
{
    private static Direction3D RandomDirection(Random random)
    {
        while (true)
        {
            var v = new Vector3D(random.NextDouble() - 0.5, random.NextDouble() - 0.5, random.NextDouble() - 0.5);
            if (v.Magnitude > 0.01)
            {
                return new Direction3D(v);
            }
        }
    }

    [Fact]
    public void LinesInPlanes_AreTheProjectionsToTheBit()
    {
        var random = new Random(11);
        for (int i = 0; i < 2000; i++)
        {
            var origin = new Point3D(100 * random.NextDouble(), -50 * random.NextDouble(), 1000 * random.NextDouble());
            var position = new CoordinateSystem(origin, RandomDirection(random));
            if (i % 2 == 1)
            {
                position = position.YReversed(); // A left-handed plane, as a reversed face can have
            }

            var plane = new PlaneSurface(position);

            // A direction in the plane, and a point on it, as an edge of the face has them, with the rounding that comes with that
            double angle = 2 * Math.PI * random.NextDouble();
            var inPlane = new Vector3D(position.XDirection) * Math.Cos(angle) + new Vector3D(position.YDirection) * Math.Sin(angle);
            Point3D start = origin.Translated(new Vector3D(position.XDirection) * (200 * random.NextDouble() - 100)
                                              + new Vector3D(position.YDirection) * (200 * random.NextDouble() - 100));
            var line = new LineCurve3D(new Line3D(start, new Direction3D(inPlane)));
            double first = 100 * random.NextDouble() - 50;
            double last = first + 0.001 + 50 * random.NextDouble();

            Assert.True(ProjectOnPlane.TryProjectLineKeepingParameters(position, position.Direction, line.Line, out Line3D onPlane));
            Line2D shortCut = PlaneProjector.ProjectLine(plane.Plane, onPlane);

            var projected = Assert.IsType<LineCurve2D>(Tool.CurveOnPlaneByProjection(line, first, last, plane));
            Line2D expected = projected.Line;

            Assert.Equal(expected.Location.X, shortCut.Location.X);
            Assert.Equal(expected.Location.Y, shortCut.Location.Y);
            Assert.Equal(expected.Direction.X, shortCut.Direction.X);
            Assert.Equal(expected.Direction.Y, shortCut.Direction.Y);
        }
    }

    [Fact]
    public void ALineThatIsNotInThePlane_IsLeftToTheProjection()
    {
        var position = new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var slanted = new Line3D(new Point3D(1, 2, 3), new Direction3D(1, 0, 1));
        Assert.False(ProjectOnPlane.TryProjectLineKeepingParameters(position, position.Direction, slanted, out _));

        Curve2D projected = Tool.CurveOnPlaneByProjection(new LineCurve3D(slanted), 0.0, 1.0, new PlaneSurface(position));
        Assert.IsType<BSplineCurve2D>(projected);
    }
}
