// Port of: src/ModelingData/TKBRep/TopTools/TopTools_ShapeMapHasher.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

using Occt;

namespace Kerf.Engine.Topology;

/// <summary>
/// The hasher of OCCT's maps of shapes: two shapes are the same key when they are <see cref="Shape.IsSame"/>
/// (same TShape and location, any orientation). The hash is the shape's own, which ignores the orientation.
/// </summary>
/// <remarks>
/// Every TopTools map typedef (<c>TopTools_MapOfShape</c>, <c>TopTools_IndexedMapOfShape</c>,
/// <c>TopTools_DataMapOfShapeShape</c>, ...) is written out in the port as the NCollection map with
/// <see cref="Instance"/> as its comparer. Analyzer rule KERF0002 (formerly OCCTNET0002) requires an explicit comparer on every map or set
/// keyed by a shape, because the default one compares with IsEqual (docs/phase3-design.md, section 3).
/// </remarks>
public sealed class ShapeHasher : IEqualityComparer<Shape>
{
    /// <summary>The comparer: the class has no state.</summary>
    public static ShapeHasher Instance { get; } = new();

    private ShapeHasher()
    {
    }

    /// <summary>OCCT's <c>operator()(S1, S2)</c>: <see cref="Shape.IsSame"/>.</summary>
    public bool Equals(Shape s1, Shape s2) => s1.IsSame(s2);

    /// <summary>OCCT's <c>operator()(S)</c>: the shape's hash, without its orientation.</summary>
    public int GetHashCode(Shape shape) => shape.GetHashCode();
}
