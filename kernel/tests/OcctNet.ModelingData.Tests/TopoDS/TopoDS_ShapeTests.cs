// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

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
        TopoDS_Shape aShape = new();
        Assert.True(aShape.IsNull());
        Assert.Equal(TopAbs_EXTERNAL, aShape.Orientation());
        Assert.True(aShape.Location().IsIdentity);
        Assert.Equal(0, aShape.NbChildren());
        Assert.Equal(TopAbs_EXTERNAL, new TopoDS_Edge().Orientation());
        // default differs only in the orientation
        Assert.Equal(TopAbs_FORWARD, default(TopoDS_Shape).Orientation());
        Assert.True(default(TopoDS_Face).IsNull());
        Assert.Equal("NULL", aShape.ToString());

        // queries that need the TShape throw on a null shape (OCCT: undefined behaviour)
        Assert.Throws<NullObjectException>(() => aShape.ShapeType());
        Assert.Throws<NullObjectException>(() => aShape.Free());
    }

    [Fact]
    public void Copies_are_independent_values_sharing_the_TShape()
    {
        TopoDS_Edge anEdge = TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0));
        TopoDS_Edge aCopy = anEdge;
        aCopy.Reverse();
        aCopy.Move(theLoc);
        Assert.Equal(TopAbs_FORWARD, anEdge.Orientation());
        Assert.True(anEdge.Location().IsIdentity);
        Assert.True(aCopy.IsPartner(anEdge));
        Assert.False(aCopy.IsSame(anEdge));

        // the -ed forms leave their shape unchanged
        TopoDS_Shape aRev = anEdge.Reversed();
        Assert.Equal(TopAbs_REVERSED, aRev.Orientation());
        Assert.Equal(TopAbs_FORWARD, anEdge.Orientation());
        Assert.Equal(TopAbs_INTERNAL, anEdge.Oriented(TopAbs_INTERNAL).Orientation());
        Assert.Equal(TopAbs_EXTERNAL, anEdge.Oriented(TopAbs_INTERNAL).Complemented().Orientation());
        Assert.Equal(TopAbs_REVERSED, anEdge.Composed(TopAbs_REVERSED).Orientation());
        Assert.Equal(theLoc, anEdge.Located(theLoc).Location());
        Assert.Equal(theLoc * theLoc, anEdge.Moved(theLoc).Moved(theLoc).Location());
        // a Moved shape's location is the new position times the old one
        var aRot = new Placement(Transform3D.Rotation(Constants.OZ(), 0.5));
        Assert.Equal(aRot * theLoc, anEdge.Located(theLoc).Moved(aRot).Location());
    }

    [Fact]
    public void Scaled_or_mirrored_locations_are_refused_when_asked()
    {
        TopoDS_Vertex aV = TestShapes.Vertex(new Point3D());
        var aScale = new Placement(Transform3D.Scale(new Point3D(), 2.0));
        Assert.Throws<DomainErrorException>(() => aV.Located(aScale, true));
        Assert.Throws<DomainErrorException>(() => aV.Moved(aScale, true));
        Assert.Equal(aScale, aV.Located(aScale).Location()); // allowed without the check, as in OCCT
    }

    [Fact]
    public void Equality_is_IsEqual_and_the_hash_ignores_the_orientation()
    {
        TopoDS_Edge anEdge = TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0));
        TopoDS_Shape aShape = anEdge;
        TopoDS_Shape aReversed = anEdge.Reversed();
        TopoDS_Shape aMoved = anEdge.Moved(theLoc);
        TopoDS_Edge anOther = TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0));

        Assert.True(aShape == anEdge && aShape.Equals(anEdge) && anEdge.Equals(aShape) && aShape.IsEqual(anEdge));
        Assert.True(aShape != aReversed && aShape.IsSame(aReversed) && aShape.IsNotEqual(aReversed));
        Assert.False(aShape.IsSame(aMoved));
        Assert.True(aShape.IsPartner(aMoved));
        Assert.False(aShape.IsPartner(anOther)); // geometrically equal, but another TShape

        // the hash ignores the orientation, so it agrees with IsSame and IsEqual
        Assert.Equal(aShape.GetHashCode(), aReversed.GetHashCode());
        Assert.Equal(aShape.GetHashCode(), anEdge.GetHashCode());
        Assert.Equal(aShape.GetHashCode(), TopoDS.Edge(aReversed).GetHashCode());
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
        Assert.True(new TopoDS_Shape() == new TopoDS_Shape());
        Assert.True(new TopoDS_Shape() == new TopoDS_Face());
        Assert.True(new TopoDS_Shape().IsSame(default(TopoDS_Shape)));
    }

    [Fact]
    public void Map_hashers_choose_IsSame_or_IsEqual()
    {
        TopoDS_Edge anEdge = TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0));
        var aSame = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        Assert.Equal(1, aSame.Add(anEdge));
        Assert.Equal(1, aSame.Add(anEdge.Reversed()));
        Assert.Equal(2, aSame.Add(anEdge.Moved(theLoc)));
        Assert.True(TopTools_ShapeMapHasher.Instance.Equals(anEdge, anEdge.Reversed()));
        Assert.Equal(anEdge.GetHashCode(), TopTools_ShapeMapHasher.Instance.GetHashCode(anEdge.Reversed()));

        var anEqual = new NCollection_IndexedMap<TopoDS_Shape>(EqualityComparer<TopoDS_Shape>.Default);
        Assert.Equal(1, anEqual.Add(anEdge));
        Assert.Equal(2, anEqual.Add(anEdge.Reversed()));
    }

    [Fact]
    public void Narrowing_checks_the_type()
    {
        TopoDS_Shape anEdge = TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0));
        TopoDS_Edge aNarrowed = TopoDS.Edge(anEdge);
        Assert.True(aNarrowed.IsEqual(anEdge));
        Assert.Throws<TypeMismatchException>(() => TopoDS.Face(anEdge));
        Assert.Throws<TypeMismatchException>(() => TopoDS.Vertex(anEdge));
        // a null shape narrows to any type
        Assert.True(TopoDS.Solid(new TopoDS_Shape()).IsNull());
        Assert.Equal(TopAbs_EXTERNAL, TopoDS.Compound(new TopoDS_Shape()).Orientation());
    }

    [Fact]
    public void AsShapeRef_writes_through_to_the_subtype_variable()
    {
        TopoDS_Edge anEdge = TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0));
        ReverseInPlace(ref TopoDS.AsShapeRef(ref anEdge));
        Assert.Equal(TopAbs_REVERSED, anEdge.Orientation());

        static void ReverseInPlace(ref TopoDS_Shape theShape) => theShape.Reverse();
    }

    [Fact]
    public void Each_subtype_converts_to_the_same_shape()
    {
        var aBuilder = new BRep_Builder();
        TopoDS_Wire aW = new();
        aBuilder.MakeWire(ref aW);
        TopoDS_Shell aSh = new();
        aBuilder.MakeShell(ref aSh);
        TopoDS_Solid aSo = new();
        aBuilder.MakeSolid(ref aSo);
        TopoDS_CompSolid aCs = new();
        aBuilder.MakeCompSolid(ref aCs);
        TopoDS_Compound aC = new();
        aBuilder.MakeCompound(ref aC);
        TopoDS_Face aF = new();
        aBuilder.MakeFace(ref aF);
        TopoDS_Shape[] aShapes = [TestShapes.Vertex(new Point3D()), TestShapes.Edge(new Point3D(), new Point3D(1, 0, 0)), aW, aF, aSh, aSo, aCs, aC];
        TopAbs_ShapeEnum[] aTypes = [TopAbs_VERTEX, TopAbs_EDGE, TopAbs_WIRE, TopAbs_FACE, TopAbs_SHELL, TopAbs_SOLID, TopAbs_COMPSOLID, TopAbs_COMPOUND];
        for (int i = 0; i < aShapes.Length; i++)
        {
            Assert.Equal(aTypes[i], aShapes[i].ShapeType());
            Assert.Equal(TopAbs_FORWARD, aShapes[i].Orientation());
        }

        Assert.True(TopoDS.Wire(aShapes[2]) == aW);
        Assert.True(TopoDS.Face(aShapes[3]) == aF);
        Assert.True(TopoDS.Shell(aShapes[4]) == aSh);
        Assert.True(TopoDS.Solid(aShapes[5]) == aSo);
        Assert.True(TopoDS.CompSolid(aShapes[6]) == aCs);
        Assert.True(TopoDS.Compound(aShapes[7]) == aC);
    }

    [Fact]
    public void HShape_holds_a_shape()
    {
        TopoDS_Vertex aV = TestShapes.Vertex(new Point3D());
        var aH = new TopoDS_HShape(aV);
        Assert.True(aH.Shape() == aV);
        aH.ChangeShape().Reverse();
        Assert.Equal(TopAbs_REVERSED, aH.Shape().Orientation());
        Assert.True(new TopoDS_HShape().Shape().IsNull());
    }

    [Fact]
    public void Generated_subtypes_are_up_to_date()
    {
        // tools/gen-topods-subtypes.cs copies TopoDS_Shape's members into the subtypes: the members match
        string[] aShapeMembers = Members(typeof(TopoDS_Shape));
        foreach (Type aType in new[] { typeof(TopoDS_Vertex), typeof(TopoDS_Edge), typeof(TopoDS_Wire), typeof(TopoDS_Face),
                                       typeof(TopoDS_Shell), typeof(TopoDS_Solid), typeof(TopoDS_CompSolid), typeof(TopoDS_Compound) })
        {
            Assert.Equal(aShapeMembers, Members(aType));
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
