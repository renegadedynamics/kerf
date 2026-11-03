// Ported from build123d tests/test_direct_api/test_projection.py.
// Omitted:
// - flat_projection, text_projection: need Compound.make_text (text isn't ported yet); text_projection also needs
//   Solid.project_faces.
namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class ProjectionTests
{
    [Fact]
    public void MultipleOutputWires()
    {
        var target = CompoundObjects.Box(10, 10, 4) - new Pos(0, 0, 2) * CompoundObjects.Box(5, 5, 2);
        var circle = Wire.MakeCircle(3, Plane.XY.Offset(10));
        var projection = circle.ProjectToShape(target, (0, 0, -1));
        var bbox = projection[0].BoundingBox();
        Check.AlmostEqual((-3, -3, 1), bbox.Min, 2);
        Check.AlmostEqual((3, 3, 2), bbox.Max, 2);
        bbox = projection[1].BoundingBox();
        Check.AlmostEqual((-3, -3, -2), bbox.Min, 2);
        Check.AlmostEqual((3, 3, -2), bbox.Max, 2);
    }

    [Fact]
    public void ErrorHandling()
    {
        var sphere = Solid.MakeSphere(50);
        var circle = Wire.MakeCircle(1);
        Assert.Throws<ArgumentException>(() => circle.ProjectToShape(sphere, center: null, direction: null));
    }

    [Fact]
    public void ProjectEdge()
    {
        var projection = Edge.MakeCircle(1, Plane.XY.Offset(-5)).ProjectToShape(Solid.MakeBox(1, 1, 1), (0, 0, 1));
        Check.AlmostEqual((1, 0, 0), projection[0].PositionAt(1), 5);
        Check.AlmostEqual((0, 1, 0), projection[0].PositionAt(0), 5);
        Check.AlmostEqual((0, 0, 0), projection[0].ArcCenter, 5);
    }

    /// <summary>Verify that the projected edge result is one edge - wire was cleaned.</summary>
    [Fact]
    public void ProjectCleanWire()
    {
        double radiusX = 45, radiusY = 30, width = 25, openingAngle = 80;
        var centerArc = Edge.MakeEllipse(radiusX, radiusY, startAngle: 270 + openingAngle / 2, endAngle: 270 - openingAngle / 2);
        var centerSurface = (Face)(-Face.Extrude(centerArc, (0, 0, 2 * width)).Moved(new Location((0, 0, -width), (0, 0, 180))));
        var tipCenterLoc = centerSurface.LocationAt(centerArc.PositionAt(1), xDir: (1, 0, 0));
        var normalAtTipCenter = tipCenterLoc.ZAxis.Direction;

        var planarTipArc = Edge.MakeCircle(width / 2, startAngle: 270, endAngle: 450).Located(tipCenterLoc);
        var tipArc = planarTipArc.ProjectToShape(centerSurface, -normalAtTipCenter)[0];
        Assert.IsAssignableFrom<Edge>(tipArc);
        Check.AlmostEqual(planarTipArc.PositionAt(0), tipArc.PositionAt(0), 5);
        Check.AlmostEqual(planarTipArc.PositionAt(1), tipArc.PositionAt(1), 5);
    }
}
