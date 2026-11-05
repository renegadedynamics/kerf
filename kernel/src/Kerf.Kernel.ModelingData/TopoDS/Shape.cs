// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_Shape.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_Shape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using System.Runtime.CompilerServices;

namespace Kerf.Topology;

/// <summary>
/// A shape: a reference to an underlying <c>TShape</c>, a location for it (its placement in the local
/// coordinate system) and an orientation (in terms of its geometry, as opposed to its orientation in relation to
/// other shapes).
/// </summary>
/// <remarks>
/// <para>
/// A value, as in OCCT: copying a shape copies its three fields, and the copies share the TShape. The forms that
/// return a changed shape (<see cref="Located"/>, <see cref="Oriented"/>, <see cref="Moved"/>, <see cref="Reversed"/>,
/// <see cref="Complemented"/>, <see cref="Composed"/>) are public; the in-place forms are internal to Kerf.
/// </para>
/// <para>
/// Equality is OCCT's <see cref="IsEqual"/>: the same TShape (by reference), the same location (by the identity of
/// its datums) and the same orientation. <see cref="GetHashCode"/> ignores the orientation, so it agrees with both
/// IsEqual and <see cref="IsSame"/>; <see cref="ShapeHasher"/> compares with IsSame.
/// </para>
/// <para>
/// <c>new TopoDS_Shape()</c> is OCCT's null shape, with orientation EXTERNAL; <c>default(TopoDS_Shape)</c> differs
/// from it only in the orientation (FORWARD). Ported code writes <c>new TopoDS_Shape()</c>.
/// </para>
/// <para>
/// TopoDS_Vertex, TopoDS_Edge, TopoDS_Wire, TopoDS_Face, TopoDS_Shell, TopoDS_Solid, TopoDS_CompSolid and
/// TopoDS_Compound have the same fields and members (generated from this file by tools/gen-topods-subtypes.cs), and
/// convert implicitly to TopoDS_Shape. <c>TopoDS.Edge(S)</c> and the others narrow a shape,
/// checking its type.
/// </para>
/// </remarks>
public partial struct Shape : IEquatable<Shape>
{
    private TShape? _tShape;
    private Placement _location;
    private Orientation _orient;

    /// <summary>A null shape, referring to nothing (orientation EXTERNAL, as OCCT's).</summary>
    public Shape()
    {
        _tShape = null;
        _location = new Placement();
        _orient = global::Kerf.Topology.Orientation.External;
    }

    /// <summary>A shape with the given fields: the conversions between the shape types.</summary>
    internal Shape(TShape? tShape, in Placement location, Orientation orient)
    {
        _tShape = tShape;
        _location = location;
        _orient = orient;
    }

    // <generated-members> Copied into the eight subtypes by tools/gen-topods-subtypes.cs: edit here, then regenerate.

    /// <summary>True if this shape is null: it references no underlying shape.</summary>
    public readonly bool IsNull => _tShape is null;

    /// <summary>Makes this shape null.</summary>
    internal void Nullify()
    {
        _tShape = null;
        _location = new Placement();
        _orient = global::Kerf.Topology.Orientation.External;
    }

    /// <summary>The shape's local coordinate system.</summary>
    public readonly Placement Placement => _location;

    /// <summary>Sets the shape's local coordinate system.</summary>
    /// <param name="loc">The new local coordinate system.</param>
    /// <param name="raiseExc">Throw if the transformation has a scale or is negative.</param>
    internal void Location(in Placement loc, bool raiseExc = false)
    {
        Transform3D trsf = loc.Transformation;
        if (raiseExc)
        {
            ValidateTransformation(trsf);
        }

        _location = loc;
    }

    /// <summary>This shape with its local coordinate system set to <paramref name="loc"/>.</summary>
    /// <param name="loc">The new local coordinate system.</param>
    /// <param name="raiseExc">Throw if the transformation has a scale or is negative.</param>
    public readonly Shape Located(in Placement loc, bool raiseExc = false)
    {
        Shape shape = this;
        shape.Location(loc, raiseExc);
        return shape;
    }

    /// <summary>The shape's orientation.</summary>
    public readonly Orientation Orientation => _orient;

    /// <summary>Sets the shape's orientation.</summary>
    internal void Orient(Orientation orient) => _orient = orient;

    /// <summary>This shape with the given orientation.</summary>
    public readonly Shape Oriented(Orientation orient)
    {
        Shape shape = this;
        shape.Orient(orient);
        return shape;
    }

    /// <summary>The underlying shape (null for a null shape).</summary>
    internal readonly TShape? TShape() => _tShape;

    /// <summary>The type of the shape: VERTEX, EDGE, and so on.</summary>
    /// <exception cref="NullObjectException">The shape is null.</exception>
    public readonly ShapeType Type => NonNullTShape().ShapeType();

    /// <summary>The free flag of the underlying shape.</summary>
    internal readonly bool Free() => NonNullTShape().Free();

    /// <summary>Sets the free flag of the underlying shape (shared by every shape referring to it).</summary>
    internal readonly void Free(bool isFree) => NonNullTShape().Free(isFree);

    /// <summary>The locked flag of the underlying shape: true for every frozen shape.</summary>
    internal readonly bool Locked() => NonNullTShape().Locked();

    /// <summary>Sets the locked flag of the underlying shape.</summary>
    internal readonly void Locked(bool isLocked) => NonNullTShape().Locked(isLocked);

    /// <summary>The modification flag of the underlying shape.</summary>
    internal readonly bool Modified() => NonNullTShape().Modified();

    /// <summary>Sets the modification flag of the underlying shape.</summary>
    internal readonly void Modified(bool isModified) => NonNullTShape().Modified(isModified);

    /// <summary>The checked flag of the underlying shape.</summary>
    internal readonly bool Checked() => NonNullTShape().Checked();

    /// <summary>Sets the checked flag of the underlying shape.</summary>
    internal readonly void Checked(bool isChecked) => NonNullTShape().Checked(isChecked);

    /// <summary>The orientability flag of the underlying shape.</summary>
    internal readonly bool OrientableFlag => NonNullTShape().Orientable();

    /// <summary>Sets the orientability flag of the underlying shape.</summary>
    internal readonly void Orientable(bool isOrientable) => NonNullTShape().Orientable(isOrientable);

    /// <summary>The closedness flag OCCT stores on the underlying shape, as its builder set it; see the public <c>IsClosed</c>, which looks.</summary>
    internal readonly bool ClosedFlag => NonNullTShape().Closed();

    /// <summary>Sets the closedness flag of the underlying shape.</summary>
    internal readonly void Closed(bool isClosed) => NonNullTShape().Closed(isClosed);

    /// <summary>The infinity flag of the underlying shape.</summary>
    internal readonly bool InfiniteFlag => NonNullTShape().Infinite();

    /// <summary>Sets the infinity flag of the underlying shape.</summary>
    internal readonly void Infinite(bool isInfinite) => NonNullTShape().Infinite(isInfinite);

    /// <summary>The convexness flag of the underlying shape.</summary>
    internal readonly bool ConvexFlag => NonNullTShape().Convex();

    /// <summary>Sets the convexness flag of the underlying shape.</summary>
    internal readonly void Convex(bool isConvex) => NonNullTShape().Convex(isConvex);

    /// <summary>Multiplies the shape's location by <paramref name="position"/>.</summary>
    /// <param name="position">The transformation to apply.</param>
    /// <param name="raiseExc">Throw if the transformation has a scale or is negative.</param>
    internal void Move(in Placement position, bool raiseExc = false)
    {
        Transform3D trsf = position.Transformation;
        if (raiseExc)
        {
            ValidateTransformation(trsf);
        }

        _location = position * _location;
    }

    /// <summary>This shape with its location multiplied by <paramref name="position"/>.</summary>
    /// <param name="position">The transformation to apply.</param>
    /// <param name="raiseExc">Throw if the transformation has a scale or is negative.</param>
    public readonly Shape Moved(in Placement position, bool raiseExc = false)
    {
        Shape shape = this;
        shape.Move(position, raiseExc);
        return shape;
    }

    /// <summary>Reverses the orientation (TopAbs::Reverse).</summary>
    internal void Reverse() => _orient = global::Kerf.Engine.Topology.TopAbs.Reverse(_orient);

    /// <summary>This shape with the orientation reversed (TopAbs::Reverse).</summary>
    public readonly Shape Reversed()
    {
        Shape shape = this;
        shape.Reverse();
        return shape;
    }

    /// <summary>Complements the orientation (TopAbs::Complement).</summary>
    internal void Complement() => _orient = global::Kerf.Engine.Topology.TopAbs.Complement(_orient);

    /// <summary>This shape with the orientation complemented (TopAbs::Complement).</summary>
    public readonly Shape Complemented()
    {
        Shape shape = this;
        shape.Complement();
        return shape;
    }

    /// <summary>Composes the orientation with <paramref name="orient"/> (TopAbs::Compose).</summary>
    internal void Compose(Orientation orient) => _orient = global::Kerf.Engine.Topology.TopAbs.Compose(_orient, orient);

    /// <summary>This shape with the orientation composed with <paramref name="orient"/> (TopAbs::Compose).</summary>
    public readonly Shape Composed(Orientation orient)
    {
        Shape shape = this;
        shape.Compose(orient);
        return shape;
    }

    /// <summary>The number of direct sub-shapes (children); 0 for a null shape.</summary>
    public readonly int ChildCount => _tShape is null ? 0 : _tShape.ChildCount;

    /// <summary>True if the two shapes share the same TShape. Locations and orientations may differ.</summary>
    public readonly bool IsPartner(in Shape other) => ReferenceEquals(_tShape, other.TShape());

    /// <summary>True if the two shapes share the same TShape and location. Orientations may differ.</summary>
    public readonly bool IsSame(in Shape other)
        => ReferenceEquals(_tShape, other.TShape()) && _location.IsEqual(other.Placement);

    /// <summary>True if the two shapes share the same TShape, location and orientation.</summary>
    public readonly bool IsEqual(in Shape other)
        => ReferenceEquals(_tShape, other.TShape()) && _location.IsEqual(other.Placement)
           && _orient == other.Orientation;

    /// <summary>The negation of <see cref="IsEqual"/>.</summary>
    internal readonly bool IsNotEqual(in Shape other) => !IsEqual(other);

    /// <summary>
    /// Replaces the TShape by a new one with the same geometry and no sub-shapes, keeping the orientation and location.
    /// </summary>
    internal void EmptyCopy() => _tShape = NonNullTShape().EmptyCopy();

    /// <summary>
    /// This shape with a new TShape with the same geometry and no sub-shapes. Internal: the new TShape isn't frozen,
    /// so ported code can fill it (docs/decisions.md, 3103).
    /// </summary>
    internal readonly Shape EmptyCopied()
    {
        Shape shape = this;
        shape.EmptyCopy();
        return shape;
    }

    /// <summary>Sets the underlying shape.</summary>
    internal void TShape(TShape? tShape) => _tShape = tShape;

    // Checks if the transformation contains scaling or negative values.
    private static void ValidateTransformation(in Transform3D trsf)
    {
        if (Math.Abs(Math.Abs(trsf.ScaleFactor) - 1.0) > Placement.ScalePrec() || trsf.IsNegative)
        {
            // Exception
            throw Refusal.Argument(new DomainErrorException("Transformation with scaling transformation is forbidden"));
        }
    }

    // OCCT dereferences the null handle (undefined behaviour); the port throws Standard_NullObject.
    private readonly TShape NonNullTShape()
        => _tShape ?? throw Refusal.State(new NullObjectException("TopoDS_Shape: the shape is null"));

    // </generated-members>

    /// <summary>OCCT's <c>operator==</c>: <see cref="IsEqual"/>.</summary>
    public readonly bool Equals(Shape other) => IsEqual(other);

    /// <summary>
    /// <see cref="IsEqual"/> with any of the nine shape types, so a boxed TopoDS_Edge equals the boxed TopoDS_Shape that
    /// IsEqual matches.
    /// </summary>
    public override readonly bool Equals(object? obj) => EqualsObject(this, obj);

    /// <summary>
    /// The TShape's identity hash combined with the location's hash, without the orientation, as OCCT's
    /// <c>std::hash&lt;TopoDS_Shape&gt;</c>: consistent with both <see cref="IsSame"/> and <see cref="IsEqual"/>.
    /// Process-specific, as OCCT's (an address there).
    /// </summary>
    public override readonly int GetHashCode() => HashCodeOf(_tShape, _location);

    public static bool operator ==(in Shape left, in Shape right) => left.IsEqual(right);

    public static bool operator !=(in Shape left, in Shape right) => !left.IsEqual(right);

    /// <summary>The type and orientation, or "NULL".</summary>
    public override readonly string ToString() => ToStringOf(this);

    internal static bool EqualsObject(in Shape shape, object? obj) => obj switch
    {
        Shape s => shape.IsEqual(s),
        Vertex s => shape.IsEqual(s),
        Edge s => shape.IsEqual(s),
        Wire s => shape.IsEqual(s),
        Face s => shape.IsEqual(s),
        Shell s => shape.IsEqual(s),
        Solid s => shape.IsEqual(s),
        CompSolid s => shape.IsEqual(s),
        Compound s => shape.IsEqual(s),
        _ => false,
    };

    internal static int HashCodeOf(TShape? tShape, in Placement location)
    {
        int hl = location.HashCode();
        int ht = tShape?._hash ?? 0;
        return hl == 0 ? ht : HashCode.Combine(ht, hl);
    }

    internal static string ToStringOf(in Shape shape) => shape.IsNull
        ? "NULL"
        : global::Kerf.Engine.Topology.TopAbs.ShapeTypeToString(shape.Type) + " " + global::Kerf.Engine.Topology.TopAbs.ShapeOrientationToString(shape.Orientation);
}
