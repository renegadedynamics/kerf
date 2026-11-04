// Ported from build123d tests/test_direct_api/test_compound.py.
// Omitted:
// - TestCompound.make_text, triad: text and make_triad aren't ported yet.
// - TestCompound.remove: Compound._remove is private and has no counterpart.
// - TestCompound.repr: the simple case compares a repr with a memory address; only the assembly's label and child
//   count are checked, against the port's ToString.
// - TestCompound.constructor: a TypeError for an unknown keyword, which the compiler rules out.
// - TestCompound.unwrap_of_a_custom_object_keeps_its_topology_class: needs BasePartObject subclasses, BuildPart and insert.
// - TestDoChildrenIntersect.child_parts_of_a_part: sets a Solid's parent; children hold no parent link.
// - TestCompoundTreeValidation: _pre_attach and _pre_attach_children belong to the parent-link tree.
// - TestCompoundTreeLocation: ported with WithAssemblyChildren in place of setting .parent and .children; the marker's
//   global location is the assembly's location times its own, as there is no parent link to read it through.
namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

internal static class CompoundObjects
{
    /// <summary>build123d's Box(l, w, h): a Part of a box centred on the origin.</summary>
    public static Part Box(double l, double w, double h) =>
        new([Solid.MakeBox(l, w, h).Moved(new Location(new Vector(-l / 2, -w / 2, -h / 2)))]);

    /// <summary>build123d's Cylinder(r, h): a Part of a cylinder centred on the origin, or standing on it (<paramref name="onBase"/>).</summary>
    public static Part Cylinder(double r, double h, bool onBase = false) =>
        new([onBase ? Solid.MakeCylinder(r, h) : Solid.MakeCylinder(r, h).Moved(new Location(new Vector(0, 0, -h / 2)))]);

    /// <summary>build123d's Sphere(r): a Part of a sphere centred on the origin.</summary>
    public static Part Sphere(double r) => new([Solid.MakeSphere(r)]);

    /// <summary>build123d's Rectangle(w, h): a Sketch of a rectangle centred on the origin.</summary>
    public static Sketch Rectangle(double w, double h) => new([Face.MakeRect(w, h)]);

    /// <summary>build123d's Circle(r): a Sketch of a disc.</summary>
    public static Sketch Circle(double r) => new([new Face(Wire.MakeCircle(r))]);

    /// <summary>GridLocations(10, 10, 2, 2) * Circle(1) added to an empty Sketch.</summary>
    public static Shape GridOfCircles()
    {
        Location[] grid = [new Pos(-5, -5), new Pos(-5, 5), new Pos(5, -5), new Pos(5, 5)];
        return new Sketch() + grid.Select(l => (Shape)(l * Circle(1)));
    }
}

public class CompoundTests
{
    [Fact]
    public void Fuse()
    {
        var box1 = Solid.MakeBox(1, 1, 1);
        var box2 = Solid.MakeBox(1, 1, 1, new Plane(new Vector(1, 0, 0)));
        var combined = new Compound([box1]).Fuse(new Shape[] { box2 }, glue: true);
        Assert.True(combined.IsValid);
        Check.AlmostEqual(2, combined.Volume, 5);
        var fuzzy = new Compound([box1]).Fuse(new Shape[] { box2 }, tolerance: 1e-6);
        Assert.True(fuzzy.IsValid);
        Check.AlmostEqual(2, fuzzy.Volume, 5);
    }

    [Fact]
    public void Repr()
    {
        var assembly = Compound.Assembly([Solid.MakeBox(1, 1, 1)], label: "test");
        Assert.EndsWith("label(test), #children(1)", assembly.ToString());
    }

    [Fact]
    public void Center()
    {
        var testCompound = new Compound(
        [
            Solid.MakeBox(2, 2, 2).Located(new Location(new Vector(-1, -1, -1))),
            Solid.MakeBox(1, 1, 1).Located(new Location(new Vector(8.5, -0.5, -0.5))),
        ]);
        Check.AlmostEqual(new Vector(1, 0, 0), testCompound.Center(CenterOf.Mass), 5);
        Check.AlmostEqual(new Vector(4.25, 0, 0), testCompound.Center(CenterOf.BoundingBox), 5);
        Assert.Throws<ArgumentException>(() => testCompound.Center(CenterOf.Geometry));
    }

    [Fact]
    public void Volume()
    {
        var e = Edge.MakeLine(new Vector(0, 0), new Vector(1, 1));
        Check.AlmostEqual(0, e.Volume, 5);

        var f = Face.MakeRect(1, 1);
        Check.AlmostEqual(0, f.Volume, 5);

        var b = Solid.MakeBox(1, 1, 1);
        Check.AlmostEqual(1, b.Volume, 5);

        var bb = CompoundObjects.Box(1, 1, 1);
        Check.AlmostEqual(1, bb.Volume, 5);

        var c = Compound.Assembly([e, f, b, bb, b.Translate(new Vector(0, 5, 0))]);
        Check.AlmostEqual(3, c.Volume, 5);
        // N.B. b and bb overlap but still add to Compound volume
    }

    [Fact]
    public void Len()
    {
        Assert.Equal(0, new Compound().Count);
        var skt = Assert.IsAssignableFrom<Compound>(CompoundObjects.GridOfCircles());
        Assert.Equal(4, skt.Count);
    }

    [Fact]
    public void Iteration()
    {
        var skt = Assert.IsAssignableFrom<Compound>(CompoundObjects.GridOfCircles());
        var children = skt.Children().ToList();
        for (int i = 0; i < children.Count; i++)
            for (int j = i + 1; j < children.Count; j++)
                Assert.True((children[i].Position - children[j].Position).Length >= 10);
    }

    [Fact]
    public void Unwrap()
    {
        var skt = CompoundObjects.GridOfCircles();
        var skt2 = Compound.Assembly([skt]);
        Assert.Equal(1, skt2.Count);
        var skt3 = Assert.IsAssignableFrom<Compound>(skt2.Unwrap(fully: false));
        Assert.Equal(4, skt3.Count);

        var comp1 = Assert.IsAssignableFrom<Compound>(new Compound().Unwrap());
        Assert.Equal(0, comp1.Count);
        var comp2 = Compound.Assembly([Face.MakeRect(1, 1)]);
        var comp3 = Compound.Assembly([comp2]);
        Assert.Equal(1, comp3.Count);
        Assert.IsAssignableFrom<Compound>(comp3.Children().First());
        var comp4 = comp3.Unwrap(fully: true);
        Assert.IsAssignableFrom<Face>(comp4);
    }

    [Fact]
    public void GetTopLevelShapes()
    {
        // PolarLocations(15, 20): 20 locations round a circle of radius 15, each turned to face outwards
        var locations = Enumerable.Range(0, 20).Select(i =>
        {
            var angle = i * 360.0 / 20;
            var (x, y) = TopologyUtils.Polar(15, angle);
            return new Location(new Vector(x, y, 0), new Vector(0, 0, angle));
        });
        var baseShapes = Compound.Assembly(locations.Select(l => (Shape)(l * CompoundObjects.Box(4, 4, 4))));
        var fls = baseShapes.GetTopLevelShapes();
        Assert.IsType<ShapeList<Shape>>(fls);
        Assert.Equal(20, fls.Count);
        Assert.All(fls, s => Assert.IsAssignableFrom<Solid>(s));

        var b1 = CompoundObjects.Box(1, 1, 1).SingleSolid();
        Assert.True(b1.GetTopLevelShapes()[0] == b1);
    }
}

/// <summary>Compound's operators, which Part/Sketch/Curve inherit.</summary>
public class CompoundAlgebraTests
{
    [Fact]
    public void AddNoneReturnsSelf()
    {
        var part = new Part(Solid.MakeBox(1, 1, 1).KernelShape);
        Assert.Same(part, part + Array.Empty<Shape>());
        Assert.Throws<ArgumentNullException>(() => part + (Shape)null!);
    }

    /// <summary>Compound holds the 1D branch; Curve inherits Mixin1D.__add__ instead.</summary>
    [Fact]
    public void Adding1dContentThatFusesToOneEdge()
    {
        var collinear = new Compound([Edge.MakeLine(new Vector(0, 0), new Vector(1, 0))]);
        var result = collinear + Edge.MakeLine(new Vector(1, 0), new Vector(2, 0));

        Assert.IsAssignableFrom<Curve>(result);
        Assert.Single(result.Edges());
        Check.AlmostEqual(2, result.Edges()[0].Length, 5);
    }

    [Fact]
    public void Adding1dContentThatStaysApart()
    {
        var separate = new Compound([Edge.MakeLine(new Vector(0, 0), new Vector(1, 0))]);
        var result = separate + Edge.MakeLine(new Vector(5, 5), new Vector(6, 5));
        Assert.Equal(2, result.Edges().Count);
    }

    [Fact]
    public void IntersectionWithNoOverlapIsEmpty()
    {
        var left = new Compound([Solid.MakeBox(1, 1, 1)]);
        var right = new Compound([Solid.MakeBox(1, 1, 1, new Plane(new Vector(10, 0, 0)))]);
        var result = left & right;

        Assert.IsAssignableFrom<Compound>(result);
        Assert.True(result.IsEmpty);
    }
}

public class CompoundAccessorsTests
{
    [Fact]
    public void CompoundAndCompounds()
    {
        var wrapped = new Compound([Solid.MakeBox(1, 1, 1)]);
        Assert.Single(wrapped.Compounds());
        Assert.IsAssignableFrom<Compound>(wrapped.SingleCompound());
    }

    /// <summary>A Part can wrap a bare Solid rather than a TopoDS_Compound.</summary>
    [Fact]
    public void NoCompoundsWhenNotWrappingOne()
    {
        var part = new Part(Solid.MakeBox(1, 1, 1).KernelShape);
        Assert.Empty(part.Compounds());
        var error = Assert.Throws<InvalidOperationException>(() => part.SingleCompound());
        Assert.Contains("found 0", error.Message);
    }
}

public class DoChildrenIntersectTests
{
    [Fact]
    public void OverlappingChildren()
    {
        var overlapping = Compound.Assembly(
            [Solid.MakeBox(1, 1, 1), Solid.MakeBox(1, 1, 1, new Plane(new Vector(0.5, 0, 0)))], label: "asm");
        var (intersects, _, volume) = overlapping.DoChildrenIntersect();
        Assert.True(intersects);
        Assert.True(volume > 0);
    }

    [Fact]
    public void ChildrenThatStayApart()
    {
        var apart = Compound.Assembly(
            [Solid.MakeBox(1, 1, 1), Solid.MakeBox(1, 1, 1, new Plane(new Vector(5, 0, 0)))], label: "asm");
        Assert.False(apart.DoChildrenIntersect().Intersect);
    }

    /// <summary>Three children: the first pair is clear, the second is not.</summary>
    [Fact]
    public void OnlyOnePairOfSeveralOverlaps()
    {
        var assembly = Compound.Assembly(
        [
            Solid.MakeBox(1, 1, 1),
            Solid.MakeBox(1, 1, 1, new Plane(new Vector(5, 0, 0))),
            Solid.MakeBox(1, 1, 1, new Plane(new Vector(5.5, 0, 0))),
        ], label: "asm");
        var (intersects, pair, _) = assembly.DoChildrenIntersect();
        Assert.True(intersects);
        Assert.False(pair.A == assembly || pair.B == assembly);
    }

    /// <summary>
    /// The parent's own geometry contains its children, so including it always reports an intersection - between
    /// parent and child.
    /// </summary>
    [Fact]
    public void IncludingTheParent()
    {
        var assembly = Compound.Assembly(
            [Solid.MakeBox(1, 1, 1), Solid.MakeBox(1, 1, 1, new Plane(new Vector(5, 0, 0)))], label: "asm");
        Assert.False(assembly.DoChildrenIntersect().Intersect);

        var (intersects, pair, _) = assembly.DoChildrenIntersect(includeParent: true);
        Assert.True(intersects);
        Assert.True(pair.A == assembly || pair.B == assembly);
    }

    /// <summary>A sub-assembly contains its own parts, which is not a collision.</summary>
    [Fact]
    public void NestedSubAssembliesThatStayApart()
    {
        var assembly = Compound.Assembly(
        [
            Compound.Assembly([Solid.MakeBox(1, 1, 1)], label: "a"),
            Compound.Assembly([Solid.MakeBox(1, 1, 1, new Plane(new Vector(5, 0, 0)))], label: "b"),
        ], label: "asm");
        Assert.False(assembly.DoChildrenIntersect().Intersect);
    }

    /// <summary>Parts are compared where their sub-assemblies place them.</summary>
    [Fact]
    public void NestedPartsThatOverlapAfterPlacement()
    {
        var partA = Solid.MakeBox(1, 1, 1);
        var partB = Solid.MakeBox(1, 1, 1, new Plane(new Vector(5, 0, 0)));
        var subAssembly = Compound.Assembly([partB], label: "b").WithLocation(new Location(new Vector(-4.5, 0, 0)));
        var assembly = Compound.Assembly([Compound.Assembly([partA], label: "a"), subAssembly], label: "asm");

        var (intersects, pair, volume) = assembly.DoChildrenIntersect();
        Assert.True(intersects);
        Assert.True((pair.A == partA && pair.B == partB) || (pair.A == partB && pair.B == partA));
        Check.AlmostEqual(0.5, volume, 5);
    }

    /// <summary>Parts that only overlap in their local frames do not collide.</summary>
    [Fact]
    public void NestedPartsApartAfterPlacement()
    {
        var subAssembly = Compound.Assembly([Solid.MakeBox(1, 1, 1, new Plane(new Vector(0.5, 0, 0)))], label: "b")
            .WithLocation(new Location(new Vector(5, 0, 0)));
        var assembly = Compound.Assembly(
            [Compound.Assembly([Solid.MakeBox(1, 1, 1)], label: "a"), subAssembly], label: "asm");
        Assert.False(assembly.DoChildrenIntersect().Intersect);
    }
}

/// <summary>@ and % were covered; ^ was not.</summary>
public class CurveOperatorsTests
{
    // BuildLine with Polyline((0, 0), (5, 0), (5, 5)): the builder's line is a Curve of the two edges
    readonly Curve _curve = new([Edge.MakeLine(new Vector(0, 0), new Vector(5, 0)), Edge.MakeLine(new Vector(5, 0), new Vector(5, 5))]);

    [Fact]
    public void LocationOperator()
    {
        var location = _curve.LocationAt(0.5);
        Assert.IsAssignableFrom<Location>(location);
        Check.AlmostEqual(new Vector(5, 0, 0), location.Position, 5);
    }

    [Fact]
    public void MatchesPositionAndTangent()
    {
        Check.AlmostEqual(_curve.PositionAt(0.5), (_curve.LocationAt(0.5)).Position, 5);
        Check.AlmostEqual(_curve.TangentAt(0.5), (_curve.LocationAt(0.5)).ZAxis.Direction, 5);
    }
}

/// <summary>Changing the children of an assembly leaves the assembly where it is.</summary>
public class CompoundTreeLocationTests
{
    static readonly Location Placed = new(new Vector(10, 0, 0), new Vector(0, 0, 90));

    /// <summary>An assembly at x=10 turned by 90°, with a marker child at x=4 in it.</summary>
    static (Compound Assembly, Part Marker) MovedAssembly()
    {
        var marker = new Pos(4, 0, 0) * CompoundObjects.Box(1, 1, 1);
        var assembly = Compound.Assembly([CompoundObjects.Box(1, 1, 1), marker]).WithLocation(Placed);
        return (assembly, marker);
    }

    static void AssertInPlace(Compound assembly, Part marker)
    {
        Assert.Equal(Placed, assembly.Location);
        Assert.Contains(assembly.AssemblyChildren, c => ReferenceEquals(c, marker));
        Check.AlmostEqual(new Vector(10, 4, 0), (assembly.Location * marker.Location).Position, 5);
        Check.AlmostEqual(4.5, assembly.BoundingBox().Max.Y, 5);
    }

    [Fact]
    public void AttachingAChild()
    {
        var (assembly, marker) = MovedAssembly();
        assembly = assembly.WithAssemblyChildren([.. assembly.AssemblyChildren, CompoundObjects.Box(1, 1, 1)]);
        AssertInPlace(assembly, marker);
        Assert.Equal(3, assembly.AssemblyChildren.Count);
    }

    [Fact]
    public void DetachingAChild()
    {
        var (assembly, marker) = MovedAssembly();
        assembly = assembly.WithAssemblyChildren(assembly.AssemblyChildren.Skip(1));
        AssertInPlace(assembly, marker);
    }

    [Fact]
    public void ReplacingTheChildren()
    {
        var (assembly, marker) = MovedAssembly();
        assembly = assembly.WithAssemblyChildren([marker]);
        AssertInPlace(assembly, marker);
    }

    [Fact]
    public void MovingAChildToAnotherAssembly()
    {
        var (source, sourceMarker) = MovedAssembly();
        var (target, targetMarker) = MovedAssembly();
        var moving = source.AssemblyChildren[0];
        source = source.WithAssemblyChildren(source.AssemblyChildren.Skip(1));
        target = target.WithAssemblyChildren([.. target.AssemblyChildren, moving]);
        AssertInPlace(source, sourceMarker);
        AssertInPlace(target, targetMarker);
    }
}
