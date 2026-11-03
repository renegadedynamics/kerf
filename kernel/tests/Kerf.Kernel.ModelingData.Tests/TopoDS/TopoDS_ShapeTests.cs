// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// The value semantics of the shape structs (docs/phase3-design.md, sections 2 and 3): copies, the -ed forms,
/// identity, equality and hashing, the subtypes and their conversions, and TopTools_ShapeMapHasher.
/// </summary>
public class TopoDS_ShapeTests
{
    private static readonly Placement theLoc = new(Transform3D.Translation(new Vector3D(1, 2, 3)));

    [Fact]
    public void A_new_shape_is_OCCTs_null_shape()
    {
        Shape aShape = new();
        Assert.True(aShape.IsNull);
        Assert.Equal(External, aShape.Orientation);
        Assert.True(aShape.Placement.IsIdentity);
        Assert.Equal(0, aShape.ChildCount);
        Assert.Equal(External, new Edge().Orientation);
        // default differs only in the orientation
        Assert.Equal(Forward, default(Shape).Orientation);
        Assert.True(default(Face).IsNull);
        Assert.Equal("NULL", aShape.ToString());

        // queries that need the TShape throw on a null shape (OCCT: undefined behaviour)
        Assert.Throws<NullObjectException>(() => aShape.Type);
        Assert.Throws<NullObjectException>(() => aShape.Free());
    }

    [Fact]
    public void Copies_are_independent_values_sharing_the_TShape()
    {
        Edge anEdge = TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0));
        Edge aCopy = anEdge;
        aCopy.Reverse();
        aCopy.Move(theLoc);
        Assert.Equal(Forward, anEdge.Orientation);
        Assert.True(anEdge.Placement.IsIdentity);
        Assert.True(aCopy.IsPartner(anEdge));
        Assert.False(aCopy.IsSame(anEdge));

        // the -ed forms leave their shape unchanged
        Shape aRev = anEdge.Reversed();
        Assert.Equal(Reversed, aRev.Orientation);
        Assert.Equal(Forward, anEdge.Orientation);
        Assert.Equal(Internal, anEdge.Oriented(Internal).Orientation);
        Assert.Equal(External, anEdge.Oriented(Internal).Complemented().Orientation);
        Assert.Equal(Reversed, anEdge.Composed(Reversed).Orientation);
        Assert.Equal(theLoc, anEdge.Located(theLoc).Placement);
        Assert.Equal(theLoc * theLoc, anEdge.Moved(theLoc).Moved(theLoc).Placement);
        // a Moved shape's location is the new position times the old one
        var aRot = new Placement(Transform3D.Rotation(Constants.OZ(), 0.5));
        Assert.Equal(aRot * theLoc, anEdge.Located(theLoc).Moved(aRot).Placement);
    }

    [Fact]
    public void Scaled_or_mirrored_locations_are_refused_when_asked()
    {
        Vertex aV = TestShapes.Vertex(new Point3D());
        var aScale = new Placement(Transform3D.Scale(new Point3D(), 2.0));
        Assert.Throws<DomainErrorException>(() => aV.Located(aScale, true));
        Assert.Throws<DomainErrorException>(() => aV.Moved(aScale, true));
        Assert.Equal(aScale, aV.Located(aScale).Placement); // allowed without the check, as in OCCT
    }

    [Fact]
    public void Equality_is_IsEqual_and_the_hash_ignores_the_orientation()
    {
        Edge anEdge = TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0));
        Shape aShape = anEdge;
        Shape aReversed = anEdge.Reversed();
        Shape aMoved = anEdge.Moved(theLoc);
        Edge anOther = TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0));

        Assert.True(aShape == anEdge && aShape.Equals(anEdge) && anEdge.Equals(aShape) && aShape.IsEqual(anEdge));
        Assert.True(aShape != aReversed && aShape.IsSame(aReversed) && aShape.IsNotEqual(aReversed));
        Assert.False(aShape.IsSame(aMoved));
        Assert.True(aShape.IsPartner(aMoved));
        Assert.False(aShape.IsPartner(anOther)); // geometrically equal, but another TShape

        // the hash ignores the orientation, so it agrees with IsSame and IsEqual
        Assert.Equal(aShape.GetHashCode(), aReversed.GetHashCode());
        Assert.Equal(aShape.GetHashCode(), anEdge.GetHashCode());
        Assert.Equal(aShape.GetHashCode(), ShapeCasts.Edge(aReversed).GetHashCode());
        // equal locations built separately (from the same datum) hash the same
        Assert.Equal(aMoved.GetHashCode(), anEdge.Moved(theLoc).GetHashCode());
        Assert.True(aMoved == anEdge.Moved(theLoc));

        // boxed values compare across the shape types
        object aBoxedEdge = anEdge, aBoxedShape = aShape;
        Assert.True(aBoxedEdge.Equals(aBoxedShape) && aBoxedShape.Equals(aBoxedEdge));
        Assert.False(aBoxedEdge.Equals(aReversed));
        Assert.False(aBoxedEdge.Equals("EDGE"));
        Assert.Equal("EDGE FORWARD", aShape.ToString());
        Assert.Equal("EDGE REVERSED", aReversed.ToString());

        // null shapes are equal when their orientations are
        Assert.True(new Shape() == new Shape());
        Assert.True(new Shape() == new Face());
        Assert.True(new Shape().IsSame(default(Shape)));
    }

    [Fact]
    public void Map_hashers_choose_IsSame_or_IsEqual()
    {
        Edge anEdge = TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0));
        var aSame = new IndexedSet<Shape>(ShapeHasher.Instance);
        Assert.Equal(1, aSame.Add(anEdge) + 1);
        Assert.Equal(1, aSame.Add(anEdge.Reversed()) + 1);
        Assert.Equal(2, aSame.Add(anEdge.Moved(theLoc)) + 1);
        Assert.True(ShapeHasher.Instance.Equals(anEdge, anEdge.Reversed()));
        Assert.Equal(anEdge.GetHashCode(), ShapeHasher.Instance.GetHashCode(anEdge.Reversed()));

        var anEqual = new IndexedSet<Shape>(EqualityComparer<Shape>.Default);
        Assert.Equal(1, anEqual.Add(anEdge) + 1);
        Assert.Equal(2, anEqual.Add(anEdge.Reversed()) + 1);
    }

    [Fact]
    public void Narrowing_checks_the_type()
    {
        Shape anEdge = TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0));
        Edge aNarrowed = ShapeCasts.Edge(anEdge);
        Assert.True(aNarrowed.IsEqual(anEdge));
        Assert.Throws<TypeMismatchException>(() => ShapeCasts.Face(anEdge));
        Assert.Throws<TypeMismatchException>(() => ShapeCasts.Vertex(anEdge));
        // a null shape narrows to any type
        Assert.True(ShapeCasts.Solid(new Shape()).IsNull);
        Assert.Equal(External, ShapeCasts.Compound(new Shape()).Orientation);
    }

    [Fact]
    public void AsShapeRef_writes_through_to_the_subtype_variable()
    {
        Edge anEdge = TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0));
        ReverseInPlace(ref ShapeCasts.AsShapeRef(ref anEdge));
        Assert.Equal(Reversed, anEdge.Orientation);

        static void ReverseInPlace(ref Shape theShape) => theShape.Reverse();
    }

    [Fact]
    public void Each_subtype_converts_to_the_same_shape()
    {
        var aBuilder = new BrepBuilder();
        Wire aW = new();
        aBuilder.MakeWire(ref aW);
        Shell aSh = new();
        aBuilder.MakeShell(ref aSh);
        Solid aSo = new();
        aBuilder.MakeSolid(ref aSo);
        CompSolid aCs = new();
        aBuilder.MakeCompSolid(ref aCs);
        Compound aC = new();
        aBuilder.MakeCompound(ref aC);
        Face aF = new();
        aBuilder.MakeFace(ref aF);
        Shape[] aShapes = [TestShapes.Vertex(new Point3D()), TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0)), aW, aF, aSh, aSo, aCs, aC];
        ShapeType[] aTypes = [ShapeType.Vertex, ShapeType.Edge, ShapeType.Wire, ShapeType.Face, ShapeType.Shell, ShapeType.Solid, ShapeType.CompSolid, ShapeType.Compound];
        for (int i = 0; i < aShapes.Length; i++)
        {
            Assert.Equal(aTypes[i], aShapes[i].Type);
            Assert.Equal(Forward, aShapes[i].Orientation);
        }

        Assert.True(ShapeCasts.Wire(aShapes[2]) == aW);
        Assert.True(ShapeCasts.Face(aShapes[3]) == aF);
        Assert.True(ShapeCasts.Shell(aShapes[4]) == aSh);
        Assert.True(ShapeCasts.Solid(aShapes[5]) == aSo);
        Assert.True(ShapeCasts.CompSolid(aShapes[6]) == aCs);
        Assert.True(ShapeCasts.Compound(aShapes[7]) == aC);
    }

    [Fact]
    public void HShape_holds_a_shape()
    {
        Vertex aV = TestShapes.Vertex(new Point3D());
        var aH = new TopoDS_HShape(aV);
        Assert.True(aH.Shape() == aV);
        aH.ChangeShape().Reverse();
        Assert.Equal(Reversed, aH.Shape().Orientation);
        Assert.True(new TopoDS_HShape().Shape().IsNull);
    }

    [Fact]
    public void Generated_subtypes_are_up_to_date()
    {
        // tools/gen-topods-subtypes.cs copies TopoDS_Shape's members into the subtypes: each has them all, but for the
        // casts only Shape has (TryAs, As); the vertex's, edge's, wire's and face's own data come beside them (unit 10L1)
        string[] aShapeMembers = [.. Members(typeof(Shape)).Where(m => !m.Contains(" TryAs(") && !m.Contains(" As("))];
        foreach (Type aType in new[] { typeof(Vertex), typeof(Edge), typeof(Wire), typeof(Face),
                                       typeof(Shell), typeof(Solid), typeof(CompSolid), typeof(Compound) })
        {
            Assert.Equal(aShapeMembers, Members(aType).Intersect(aShapeMembers));
        }

        static string[] Members(Type theType)
            => [.. theType.GetMethods(System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic
                                     | System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.DeclaredOnly)
                          .Where(m => m.Name != "Equals" && !m.IsSpecialName)
                          .Select(m => (m.IsPublic ? "public " : m.IsAssembly ? "internal " : "private ") + m.ReturnType.Name + " " + m.Name + "("
                                       + string.Join(", ", m.GetParameters().Select(p => p.ParameterType.Name)) + ")")
                          .Order(StringComparer.Ordinal)];
    }
}
