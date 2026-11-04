// Ported from build123d tests/test_direct_api/test_history.py.
// Omitted or changed:
// - ShapeHistory.relocation_history: the final check reads the private OCCT history (HasModified); here every face
//   of the record of an unmoved shape maps to nothing instead.
// - ShapeHistory.recorded_placements_preserve_last_and_new_geometry: the BuildPart/insert comparison and the
//   cold-cache check (which clears the private _traces) are left out.
// - ShapeHistory.deep_copy_drops_the_record: copy.deepcopy has no counterpart.
// - ShapeHistory.merge_returns_self_and_tolerates_none: records are immutable, so merging an empty record returns a
//   new record rather than self; only the None case returns the same record.
using Kerf.Geometry;
using Ops = Kerf.Modeling.Operations;

namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class ShapeHistoryTests
{
    readonly Solid _box = Solid.MakeBox(10, 10, 10);
    readonly Face _top;

    public ShapeHistoryTests() => _top = _box.Faces().OrderBy(Axis.Z)[^1];

    /// <summary>
    /// A cylinder standing on the box: the top is rebuilt around the cylinder's own bottom circle, the cylinder's
    /// bottom cap is removed, everything else is left alone and nothing is generated.
    /// </summary>
    [Fact]
    public void BooleanRecordsUntouchedModifiedRemovedAndGenerated()
    {
        var cylinder = Solid.MakeCylinder(2, 5, new Plane((5, 5, 10)));
        var fused = _box.Fuse(cylinder);
        var history = fused.History;
        Assert.IsType<ShapeHistory>(history);
        Assert.Single(history.Modified(_top.Wrapped));
        foreach (var side in _box.Faces().NotParallelTo(Axis.Z))
        {
            Assert.Empty(history.Modified(side.Wrapped));
            Assert.False(history.IsRemoved(side.Wrapped));
        }
        var bottomCap = cylinder.Faces().OrderBy(Axis.Z)[0];
        Assert.True(history.IsRemoved(bottomCap.Wrapped));
        Assert.Empty(history.Generated(_top.Wrapped));

        // A cylinder through the box: the circles where it pierces the top and bottom are generated from those faces
        var through = _box.Fuse(Solid.MakeCylinder(2, 20, new Plane((5, 5, -5))));
        history = through.History!;
        Assert.NotEmpty(history.Generated(_top.Wrapped));
        Assert.Single(history.Modified(_top.Wrapped));
    }

    /// <summary>After a fuse and a clean the original face still maps to one face.</summary>
    [Fact]
    public void CleanHistoryIsMergedIntoTheBoolean()
    {
        var slab = Solid.MakeBox(10, 10, 5, new Plane((0, 0, 10)));
        var fused = _box.Fuse(slab);   // coplanar sides are unified by the clean
        var history = fused.History!;
        var side = _box.Faces().OrderBy(Axis.X)[0];
        var merged = history.Modified(side.Wrapped);
        Assert.Single(merged);
        Check.AlmostEqual(150, new Face(((global::Kerf.Topology.Face)(merged[0]))).Area, 5);
    }

    [Fact]
    public void TraceAnswersFromTheResultSide()
    {
        var cylinder = Solid.MakeCylinder(2, 20, new Plane((5, 5, -5)));
        var fused = _box.Fuse(cylinder);
        var trace = fused.History!.TraceOf([_box.Wrapped]);
        int untouched = 0, modified = 0, generated = 0, other = 0;
        foreach (var face in fused.Faces())
        {
            if (trace.IsUntouched(face.Wrapped))
                untouched++;
            else if (trace.IsModified(face.Wrapped))
                modified++;
            else if (trace.IsGenerated(face.Wrapped))
                generated++;
            else
                other++;
        }
        // four box sides untouched, top and bottom rebuilt, and the cylinder's faces are not the box's at all
        Assert.Equal((4, 2, 0, 4), (untouched, modified, generated, other));
        var circles = fused.Edges().Where(e => trace.IsGenerated(e.Wrapped)).ToList();
        Assert.True(circles.Count >= 2);
        Assert.All(circles, e => Assert.Equal(GeomType.Circle, e.GeomType));
    }

    [Fact]
    public void AlgorithmHistoryFromAFillet()
    {
        var rounded = _box.Fillet(1, _box.Edges().ParallelTo(Axis.Z));
        var trace = rounded.History!.TraceOf([_box.Wrapped]);
        var faces = rounded.Faces();
        Assert.Equal(6, faces.Count(f => trace.IsModified(f.Wrapped)));
        Assert.Equal(4, faces.Count(f => trace.IsGenerated(f.Wrapped)));
        Assert.Equal(10, faces.Count(f => trace.IsDescendant(f.Wrapped)));
        // the filleted edges are gone and the fillet faces came from them
        foreach (var edge in _box.Edges().ParallelTo(Axis.Z))
        {
            Assert.True(rounded.History.IsRemoved(edge.Wrapped));
            Assert.Single(rounded.History.Generated(edge.Wrapped));
        }
    }

    /// <summary>An extrusion generates faces from edges and edges from vertices.</summary>
    [Fact]
    public void AlgorithmHistoryFromAPrism()
    {
        var square = Face.MakeRect(4, 4);
        var vec = new Vector3D(0, 0, 5);
        var prism = global::Kerf.Construction.Sweeps.Extrude(square.Wrapped, vec).Value;
        var history = ShapeHistory.FromAlgorithm(s => prism.History.Modified(s), s => prism.History.Generated(s), [square.Wrapped], prism.Shape);
        // The kernel's extrusion history also generates each sub-shape's copy in the top cap (decision 1558)
        foreach (var edge in square.Edges())
            Assert.Single(history.Generated(edge.Wrapped), g => g.Type == global::Kerf.Topology.ShapeType.Face);
        foreach (var vertex in square.Vertices())
            Assert.Single(history.Generated(vertex.Wrapped), g => g.Type == global::Kerf.Topology.ShapeType.Edge);
        Assert.False(history.IsRemoved(square.Wrapped));
    }

    /// <summary>Sewing rebuilds both faces and merges the shared edge.</summary>
    [Fact]
    public void SewingHistory()
    {
        var left = Face.MakeRect(4, 4);
        var right = Face.MakeRect(4, 4, new Plane((4, 0, 0)));
        var sewn = global::Kerf.Healing.HealingOperations.Sew([left.Wrapped, right.Wrapped], new global::Kerf.Healing.SewOptions { Tolerance = 1e-6 }).Value;
        var history = ShapeHistory.FromHistory(sewn.History, [left.Wrapped, right.Wrapped]);
        foreach (var face in new[] { left, right })
            Assert.Single(history.Modified(face.Wrapped));
        // the two coincident edges were merged into one edge of the shell
        var leftEdge = left.Edges().OrderBy(Axis.X)[^1];
        var rightEdge = right.Edges().OrderBy(Axis.X)[0];
        var mergedLeft = history.Modified(leftEdge.Wrapped);
        var mergedRight = history.Modified(rightEdge.Wrapped);
        Assert.Single(mergedLeft);
        Assert.True(mergedLeft[0].IsSame(mergedRight[0]));
    }

    [Fact]
    public void RelocationHistory()
    {
        var moved = _box.Moved(new Plane((1, 2, 3)).Location);
        var history = ShapeHistory.FromRelocation(_box.Wrapped, moved.Wrapped);
        foreach (var face in _box.Faces())
        {
            var placed = history.Modified(face.Wrapped);
            Assert.Single(placed);
            Assert.True(placed[0].IsPartner(face.Wrapped));
            Assert.False(placed[0].IsSame(face.Wrapped));
        }
        // nothing moved, nothing recorded
        var unmoved = ShapeHistory.FromRelocation(_box.Wrapped, _box.Wrapped);
        foreach (var sub in ShapeHistory.TrackedSubshapes([_box.Wrapped]))
            Assert.Empty(unmoved.Modified(sub));
    }

    [Fact]
    public void RelocationHistoryPreservesRelativeLocations()
    {
        var initial = new Pos(3, -5, 7) * new Rotation(11, 23, 37);
        Location[] motions = [new Pos(13, -7, 5), new Rotation(17, 31, -23), new Pos(-4, 8, 2) * new Rotation(-29, 7, 41)];
        Solid[] startingShapes = [_box, _box, _box.Moved(initial)];

        foreach (var (before, motion) in startingShapes.Zip(motions))
        {
            var after = before.Moved(motion);
            var history = ShapeHistory.FromRelocation(before.Wrapped, after.Wrapped);
            foreach (var subshape in ShapeHistory.TrackedSubshapes([before.Wrapped]))
            {
                var mapped = history.Modified(subshape);
                Assert.Single(mapped);
                Assert.True(mapped[0].IsSame(subshape.Moved(motion.ToTopLoc())));
            }
        }

        var movedOccurrences = _box.Moved(new Pos(20, 0, 0)) + _box.Moved(new Pos(-20, 0, 0));
        Assert.IsType<ShapeHistory>(movedOccurrences.History);
        var motion2 = new Pos(1, 2, 3) * new Rotation(19, 13, -7);
        var relocated = movedOccurrences.Moved(motion2);
        var history2 = ShapeHistory.FromRelocation(movedOccurrences.Wrapped, relocated.Wrapped);
        foreach (var subshape in ShapeHistory.TrackedSubshapes([movedOccurrences.Wrapped]))
        {
            var mapped = history2.Modified(subshape);
            Assert.Single(mapped);
            Assert.True(mapped[0].IsSame(subshape.Moved(motion2.ToTopLoc())));
        }
    }

    static double R5(double value) => Math.Round(value, 5) + 0.0;   // + 0.0 turns -0 into 0, as Python's tuples compare

    static string SelectionSignature(Shape shape)
    {
        var c = shape.Center();
        var center = $"({R5(c.X)}, {R5(c.Y)}, {R5(c.Z)})";
        return shape switch
        {
            Face f => $"Face {f.GeomType} {R5(f.Area)} {center} {f.Edges().Count}",
            Edge e => $"Edge {e.GeomType} {R5(e.Length)} {center} {e.Vertices().Count}",
            Solid s => $"Solid {R5(s.Volume)} {center} {s.Faces().Count} {s.Edges().Count}",
            _ => $"Vertex {center}",
        };
    }

    static readonly (string Name, Func<Shape, Select, IEnumerable<Shape>> Of)[] Selectors =
    [
        ("faces", (s, sel) => s.Faces(sel)),
        ("edges", (s, sel) => s.Edges(sel)),
        ("vertices", (s, sel) => s.Vertices(sel)),
        ("solids", (s, sel) => s.Solids(sel)),
    ];

    static Dictionary<(string, Select), List<string>> Answers(Shape shape, Func<Shape, Shape>? restore = null)
    {
        var answers = new Dictionary<(string, Select), List<string>>();
        foreach (var (name, of) in Selectors)
            foreach (var selection in new[] { Select.Last, Select.New })
                answers[(name, selection)] = of(shape, selection)
                    .Select(s => SelectionSignature(restore is null ? s : restore(s))).Order().ToList();
        return answers;
    }

    static void AssertPlacedSelectionMatches(Shape original, Shape placed, Transform3D inverse)
    {
        var expected = Answers(original);
        var actual = Answers(placed, s =>
        {
            var transform = global::Kerf.Construction.ShapeTransforms.Transformed(s.Wrapped, inverse, copyGeometry: true).Value;
            return Shape.Cast(transform.Shape);
        });
        foreach (var key in expected.Keys)
            Assert.True(expected[key].SequenceEqual(actual[key]), $"{key}: expected [{string.Join("; ", expected[key])}] but got [{string.Join("; ", actual[key])}]");
    }

    [Fact]
    public void RecordedPlacementsPreserveLastAndNewGeometry()
    {
        static global::Kerf.Topology.Shape Snapshot(global::Kerf.Topology.Shape shape) => shape.Located(shape.Placement);

        var recorded = CompoundObjects.Box(10, 10, 10) - CompoundObjects.Cylinder(2, 20);
        var originalHistory = recorded.History;
        Assert.NotNull(originalHistory);
        var originalInputs = (originalHistory.Before.Select(Snapshot).ToList(), originalHistory.Brought.Select(Snapshot).ToList());
        var inputSubshapes = ShapeHistory.TrackedSubshapes([.. originalHistory.Before, .. originalHistory.Brought]);
        var rawMappings = inputSubshapes.Select(sub => (
            Original: Snapshot(sub),
            Modified: originalHistory.Modified(sub).Select(Snapshot).ToList(),
            Generated: originalHistory.Generated(sub).Select(Snapshot).ToList(),
            Removed: originalHistory.IsRemoved(sub))).ToList();
        var originalAnswers = Answers(recorded);

        var location = new Pos(12, -8, 5) * new Rotation(13, 0, 29);
        var inverseLocation = location.ToGpTrsf().Inverted();
        var operations = new List<(string Name, Shape Placed, Transform3D Inverse)>
        {
            ("positioning operator", location * recorded, inverseLocation),
            ("moved", recorded.Moved(location), inverseLocation),
            ("located", recorded.Located(location), inverseLocation),
        };

        var initialLocation = new Pos(4, -3, 8) * new Rotation(17, 29, -11);
        var starting = recorded.Located(initialLocation);
        var absoluteLocation = new Pos(-9, 6, 13) * new Rotation(-23, 7, 41);
        operations.Add(("absolute location from placed source", starting.Located(absoluteLocation),
                        absoluteLocation.ToGpTrsf().Inverted()));

        var firstLocation = new Pos(9, 2, -6) * new Rotation(5, 37, 19);
        var secondLocation = new Pos(-7, 12, 3) * new Rotation(-31, 13, 2);
        var repeated = recorded.Located(firstLocation).Located(secondLocation);
        operations.Add(("repeated absolute location", repeated, secondLocation.ToGpTrsf().Inverted()));

        operations.Add(("rotated copy", recorded.Rotate(Axis.Z, 31, transform: true), new Rotation(0, 0, -31).ToGpTrsf()));

        var mirrorTransform = new Transform3D();
        var origin = Plane.YZ.Origin.ToGpPnt();
        var normal = Plane.YZ.ZDir.ToGpDir();
        var ax2 = new Frame3D(origin, normal);
        mirrorTransform = mirrorTransform.WithMirror(ax2);
        operations.Add(("mirror copy", recorded.Mirror(Plane.YZ), mirrorTransform));

        var scaleInverse = new Transform3D();
        var centre = new Point3D(0, 0, 0);
        scaleInverse = scaleInverse.WithScale(centre, 0.5);
        operations.Add(("scale copy", recorded.Scale(2), scaleInverse));

        foreach (var (name, placed, inverse) in operations)
        {
            try
            {
                AssertPlacedSelectionMatches(recorded, placed, inverse);
            }
            catch (Exception e)
            {
                throw new Xunit.Sdk.XunitException($"placement={name}: {e.Message}");
            }
        }

        Assert.Same(originalHistory, recorded.History);
        Assert.Equal(originalInputs.Item1.Count, originalHistory.Before.Count);
        Assert.Equal(originalInputs.Item2.Count, originalHistory.Brought.Count);
        Assert.True(originalHistory.Before.Zip(originalInputs.Item1).All(p => p.First.IsEqual(p.Second)));
        Assert.True(originalHistory.Brought.Zip(originalInputs.Item2).All(p => p.First.IsEqual(p.Second)));
        foreach (var (original, modified, generated, removed) in rawMappings)
        {
            var currentModified = originalHistory.Modified(original);
            var currentGenerated = originalHistory.Generated(original);
            Assert.Equal(modified.Count, currentModified.Length);
            Assert.Equal(generated.Count, currentGenerated.Length);
            Assert.True(currentModified.Zip(modified).All(p => p.First.IsEqual(p.Second)));
            Assert.True(currentGenerated.Zip(generated).All(p => p.First.IsEqual(p.Second)));
            Assert.Equal(removed, originalHistory.IsRemoved(original));
        }
        var warmAnswers = Answers(recorded);
        foreach (var key in originalAnswers.Keys)
            Assert.Equal(originalAnswers[key], warmAnswers[key]);
    }

    [Fact]
    public void UnrecordedAndEmptyPlacementsKeepNoHistory()
    {
        Shape[] shapes = [CompoundObjects.Box(2, 3, 4), new Vertex(1, 2, 3), new Solid()];
        var location = new Pos(1, 2, 3);
        foreach (var shape in shapes)
            foreach (var placed in new[] { shape.Moved(location), shape.Located(location) })
                Assert.Null(placed.History);
    }

    [Fact]
    public void MergeReturnsSelfAndToleratesNone()
    {
        var history = new ShapeHistory();
        Assert.Same(history, history.Merge(null));
        Assert.NotNull(history.Merge(new ShapeHistory()));
    }

    [Fact]
    public void UnsupportedTypesAreNotTracked()
    {
        var history = new ShapeHistory();
        var wire = _top.OuterWire();
        Assert.Empty(history.Modified(wire.Wrapped));
        Assert.Empty(history.Generated(wire.Wrapped));
        Assert.False(history.IsRemoved(wire.Wrapped));
    }

    [Fact]
    public void Helpers()
    {
        var subs = ShapeHistory.TrackedSubshapes([_box.Wrapped, new global::Kerf.Topology.Shape()]);
        Assert.Equal(8 + 12 + 6 + 1, subs.Length);

    }
}

/// <summary>Select.Last and Select.New on the result of an Algebra operation.</summary>
public class AlgebraSelectTests
{
    [Fact]
    public void OperationsOtherThanBooleans()
    {
        var rounded = Ops.Fillet(new Box(10, 10, 10).Edges().ParallelTo(Axis.Z), 1);
        Assert.Equal(4, rounded.Faces(Select.Last).Count);
        Assert.Equal(16, rounded.Edges(Select.New).Count);
    }

    [Fact]
    public void FusedSketch()
    {
        Shape cross = CompoundObjects.Rectangle(8, 2);
        cross += CompoundObjects.Rectangle(2, 8);
        // the four points where the arms cross are new; the second rectangle's corners and the arms' surviving edges
        // are last
        Assert.Equal(4, cross.Vertices(Select.New).Count);
        Assert.Equal(8, cross.Vertices(Select.Last).Count);
        Assert.Empty(cross.Edges(Select.New));
        Assert.Equal(6, cross.Edges(Select.Last).Count);
        Assert.All(cross.Vertices(Select.New), v => Assert.True(Math.Abs(v.X) == 1 && Math.Abs(v.Y) == 1));
    }

    [Fact]
    public void FusedPart()
    {
        var part = CompoundObjects.Box(10, 10, 10) + new Pos(z: 5) * CompoundObjects.Cylinder(2, 5, onBase: true);
        var last = part.Faces(Select.Last);
        Assert.Equal(2, last.Count);
        Assert.All(last, f => Assert.True(f.Center().Z > 5));
        Assert.Single(part.Solids(Select.New));
    }

    [Fact]
    public void CutAndIntersectedParts()
    {
        var holed = CompoundObjects.Box(10, 10, 10) - CompoundObjects.Cylinder(1, 20);
        Assert.Single(holed.Faces(Select.Last));
        Assert.Equal(GeomType.Cylinder, holed.Faces(Select.Last)[0].GeomType);
        Assert.Equal(2, holed.Edges(Select.New).Count);
        var bar = CompoundObjects.Box(10, 10, 10) & CompoundObjects.Box(30, 2, 2);
        Assert.Equal(4, bar.Faces(Select.Last).Count);
        Assert.Empty(bar.Faces(Select.New));
    }

    [Fact]
    public void ChainedOperationsReportTheLastOne()
    {
        var three = CompoundObjects.Box(10, 10, 10)
                    + new Pos(z: 5) * CompoundObjects.Cylinder(1, 5, onBase: true)
                    + new Pos(x: 5) * CompoundObjects.Sphere(1);
        Assert.Single(three.Faces(Select.Last));
        Assert.Equal(GeomType.Sphere, three.Faces(Select.Last)[0].GeomType);
    }

    /// <summary>A multi-piece left operand: none of its untouched pieces is last.</summary>
    [Fact]
    public void EverythingOfSelfWasThereBefore()
    {
        var two = CompoundObjects.Rectangle(2, 2) + new Pos(x: 5) * CompoundObjects.Rectangle(2, 2);
        var plus = two + new Pos(x: 5) * CompoundObjects.Rectangle(1, 4);
        Assert.Equal(2, plus.Faces().Count);
        Assert.Single(plus.Faces(Select.Last));
    }

    /// <summary>Connected edges are joined without a boolean; the join records too.</summary>
    [Fact]
    public void JoinedLines()
    {
        var bent = Edge.MakeLine((0, 0), (1, 0)) + Edge.MakeLine((1, 0), (1, 1));
        Assert.Equal(2, bent.Edges().Count);
        Assert.Single(bent.Edges(Select.Last));
        Assert.Empty(bent.Edges(Select.New));
        // both ends of the new line, including the one it shares with the old
        Assert.Equal(2, bent.Vertices(Select.Last).Count);
        Assert.Empty(bent.Vertices(Select.New));
        // collinear lines are cleaned into one edge, which still carries the record
        var straight = Edge.MakeLine((0, 0), (1, 0)) + Edge.MakeLine((1, 0), (2, 0));
        Assert.IsAssignableFrom<Edge>(straight);
        Assert.Single(straight.Edges(Select.Last));
        Assert.Empty(straight.Vertices(Select.New));
        // three edges joined into an empty curve: everything was brought in
        var chain = new Curve() + new Shape[]
        {
            Edge.MakeLine((0, 0), (1, 0)),
            Edge.MakeLine((1, 0), (1, 1)),
            Edge.MakeLine((1, 1), (0, 1)),
        };
        Assert.Equal(3, chain.Edges(Select.Last).Count);
    }

    [Fact]
    public void AShapeWithNoRecordCannotAnswer()
    {
        var error = Assert.Throws<InvalidOperationException>(() => CompoundObjects.Rectangle(8, 2).Vertices(Select.New));
        Assert.Contains("no record", error.Message);
        Assert.Equal(4, CompoundObjects.Rectangle(8, 2).Vertices(Select.All).Count);
    }
}
