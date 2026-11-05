// Ported from build123d tests/test_direct_api/test_shape_placement.py.
// Omitted:
// - shallow_copy, deep_copy, pickle, copies_keep_the_colour, copies_follow_the_original: copy.copy, copy.deepcopy
//   and pickle have no counterpart (shapes are immutable; Located/Moved are the copies).
// - move: tests that Python's in-place move returns self and changes the shape; Moved is covered by Moved.
// - relocated_copies_stand_outside_the_assembly: reads .parent; children hold no parent link.
// - relocated_copies_get_their_own_joints: joints aren't ported yet.
// - relocated_assembly_keeps_its_tree: ported without the .parent and identity (assertIsNot) checks, and without the
//   copy.copy case.
// A child's global location is read as the assembly's location times the child's, as there is no parent link.
namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class ShapePlacementTests
{
    // Where every shape under test is built, and where it is sent
    static readonly Location Placed = new(new Point3D(10, 20, 30), new Vector3D(0, 0, 90));
    static readonly Location Elsewhere = new(new Point3D(-5, 1, 2), new Vector3D(90, 0, 0));

    static Assembly MakeAssembly(Location location) =>
        new Assembly([CompoundObjects.Box(1, 1, 1), new Pos(4, 0, 0) * CompoundObjects.Box(1, 1, 1)]).WithLocation(location);

    // Each kind builds a fresh shape at the given location, so a shape made at one location can be compared with the
    // same shape relocated to another
    static readonly (string Name, Func<Location, Shape> Make)[] Kinds =
    [
        ("vertex", loc => Vertex.Make(1, 2, 3).Moved(loc)),
        ("vertex of a moved wire", loc =>
            Wire.MakePolygon(new Point3D[] { new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0) }, close: false).Moved(loc).Vertices()[1]),
        ("edge", loc => Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Moved(loc)),
        ("edge of a moved face", loc => Face.MakeRect(2, 1).Moved(loc).Edges()[0]),
        ("wire", loc => Wire.MakePolygon(new Point3D[] { new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0) }).Moved(loc)),
        ("face", loc => Face.MakeRect(2, 1).Moved(loc)),
        ("face with a hole", loc => Face.MakeRect(4, 4).MakeHoles([Wire.MakeCircle(1)]).Moved(loc)),
        ("shell", loc => Solid.MakeBox(1, 2, 3).Shells()[0].Moved(loc)),
        ("solid", loc => Solid.MakeBox(1, 2, 3).Moved(loc)),
        ("labelled solid", loc => Solid.MakeBox(1, 2, 3).Moved(loc).WithLabel("brick")),
        ("compound", loc => Compound.Make([Solid.MakeBox(1, 1, 1), Solid.MakeSphere(0.5).Moved(new Pos(3, 0, 0))]).Moved(loc)),
        ("assembly", loc => MakeAssembly(loc)),
        ("part", loc => loc * CompoundObjects.Box(1, 2, 3)),
        ("sketch", loc => loc * CompoundObjects.Rectangle(1, 2)),
        ("curve", loc => loc * Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0))),
    ];

    static double R4(double value) => Math.Round(value, 4) + 0.0;   // + 0.0 turns -0 into 0

    static string Rounded(Point3D v) => $"({R4(v.X)}, {R4(v.Y)}, {R4(v.Z)})";

    static IReadOnlyList<Shape> ChildrenOf(Shape shape) => shape.IsCompound() ? shape.ChildShapes() : [];

    /// <summary>Everything about where a shape is: location, extent, vertices, children.</summary>
    sealed record Placement(Location Location, string Min, string Max, string Vertices, string Children);

    static Placement PlacementOf(Shape shape)
    {
        var bbox = shape.BoundBox();
        var vertices = shape.Vertices()
            .Select(v => (R4(v.X), R4(v.Y), R4(v.Z))).Order().Select(t => t.ToString());
        var children = ChildrenOf(shape).Select(c => Rounded((Check.LocationOf(shape) * Check.LocationOf(c)).Position));
        return new Placement(Check.LocationOf(shape), Rounded(bbox.Min), Rounded(bbox.Max),
                             string.Join(" ", vertices), string.Join(" ", children));
    }

    static void AssertPlacedLike(Shape shape, Shape reference, string kind) =>
        Assert.True(PlacementOf(shape) == PlacementOf(reference),
                    $"{kind}: {PlacementOf(shape)} != {PlacementOf(reference)}");

    /// <summary>The copy holds the original's geometry; an assembly's children do.</summary>
    static void AssertSharesGeometry(Shape shape, Shape original)
    {
        var originalChildren = ChildrenOf(original);
        if (originalChildren.Count > 0)
            foreach (var (child, source) in ChildrenOf(shape).Zip(originalChildren))
                Assert.True(child.IsPartner(source));
        else
            Assert.True(shape.IsPartner(original));
    }

    /// <summary>Every kind reports the location it was built at.</summary>
    [Fact]
    public void KindsArePlaced()
    {
        foreach (var (name, make) in Kinds)
        {
            Assert.True(Check.LocationOf(make(Placed)) == Placed, name);
            Assert.True(PlacementOf(make(Placed)) != PlacementOf(make(Elsewhere)), name);
        }
    }

    /// <summary>Located replaces the location on a copy that shares the geometry.</summary>
    [Fact]
    public void Located()
    {
        foreach (var (name, make) in Kinds)
        {
            var original = make(Placed);
            var before = PlacementOf(original);
            var relocated = original.Located(Elsewhere);
            Assert.True(Check.LocationOf(relocated) == Elsewhere, name);
            AssertPlacedLike(relocated, make(Elsewhere), name);
            AssertSharesGeometry(relocated, original);
            Assert.Equal(before, PlacementOf(original));
        }
    }

    /// <summary>Moved composes the location on a copy that shares the geometry.</summary>
    [Fact]
    public void Moved()
    {
        foreach (var (name, make) in Kinds)
        {
            var original = make(Placed);
            var before = PlacementOf(original);
            var moved = original.Moved(Elsewhere);
            Assert.True(Check.LocationOf(moved) == Elsewhere * Placed, name);
            AssertPlacedLike(moved, make(Elsewhere * Placed), name);
            AssertSharesGeometry(moved, original);
            Assert.Equal(before, PlacementOf(original));
        }
    }

    /// <summary>Sharing the geometry keeps a relocated copy's place in its topology.</summary>
    [Fact]
    public void RelocatedCopiesKeepTheirProvenance()
    {
        var cross = (CompoundObjects.Rectangle(8, 2) + CompoundObjects.Rectangle(2, 8)).SingleFace();
        var solid = Solid.Extrude(cross, new Vector3D(0, 0, 2));   // extrude(cross, 2)
        foreach (var (name, relocated) in new[] { ("located", solid.Located(Elsewhere)), ("moved", solid.Moved(Elsewhere)) })
        {
            var interior = relocated.Edges().Where(Convexity.Concave);
            Assert.True(interior.Count == 4, name);
        }
    }

    /// <summary>Location * shape is moved.</summary>
    [Fact]
    public void PlacementOperator()
    {
        foreach (var (name, make) in Kinds)
        {
            var original = make(Placed);
            var before = PlacementOf(original);
            AssertPlacedLike(Elsewhere * original, make(Elsewhere * Placed), name);
            Assert.Equal(before, PlacementOf(original));
        }
    }

    /// <summary>A copy of an assembly has its own children, sharing the originals' geometry.</summary>
    [Fact]
    public void RelocatedAssemblyKeepsItsTree()
    {
        var original = MakeAssembly(Placed);
        foreach (var (name, relocated) in new[] { ("located", original.Located(Elsewhere)), ("moved", original.Moved(Elsewhere)) })
        {
            Assert.True(relocated.Children.Count == 2, name);
            foreach (var (child, source) in relocated.Children.Zip(original.Children))
            {
                Assert.True(child.Shape.IsPartner(source.Shape), name);
                Assert.True(child.Location == source.Location, name);
            }
            // the copy's compound was rebuilt from its children at its location
            Assert.Equal(Rounded(MakeAssembly(Check.LocationOf(relocated)).BoundBox().Max), Rounded(relocated.BoundBox().Max));
        }
    }
}
