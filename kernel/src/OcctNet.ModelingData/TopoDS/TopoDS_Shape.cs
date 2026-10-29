// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_Shape.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_Shape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// A shape: a reference to an underlying <see cref="TopoDS_TShape"/>, a location for it (its placement in the local
/// coordinate system) and an orientation (in terms of its geometry, as opposed to its orientation in relation to
/// other shapes).
/// </summary>
/// <remarks>
/// <para>
/// A value, as in OCCT: copying a shape copies its three fields, and the copies share the TShape. The forms that
/// return a changed shape (<see cref="Located"/>, <see cref="Oriented"/>, <see cref="Moved"/>, <see cref="Reversed"/>,
/// <see cref="Complemented"/>, <see cref="Composed"/>) are public; the in-place forms are internal to OCCT.NET.
/// </para>
/// <para>
/// Equality is OCCT's <see cref="IsEqual"/>: the same TShape (by reference), the same location (by the identity of
/// its datums) and the same orientation. <see cref="GetHashCode"/> ignores the orientation, so it agrees with both
/// IsEqual and <see cref="IsSame"/>; <see cref="TopTools_ShapeMapHasher"/> compares with IsSame.
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
public partial struct TopoDS_Shape : IEquatable<TopoDS_Shape>
{
    private TopoDS_TShape? myTShape;
    private TopLoc_Location myLocation;
    private TopAbs_Orientation myOrient;

    /// <summary>A null shape, referring to nothing (orientation EXTERNAL, as OCCT's).</summary>
    public TopoDS_Shape()
    {
        myTShape = null;
        myLocation = new TopLoc_Location();
        myOrient = TopAbs_Orientation.TopAbs_EXTERNAL;
    }

    /// <summary>A shape with the given fields: the conversions between the shape types.</summary>
    internal TopoDS_Shape(TopoDS_TShape? theTShape, in TopLoc_Location theLocation, TopAbs_Orientation theOrient)
    {
        myTShape = theTShape;
        myLocation = theLocation;
        myOrient = theOrient;
    }

    // <generated-members> Copied into the eight subtypes by tools/gen-topods-subtypes.cs: edit here, then regenerate.

    /// <summary>True if this shape is null: it references no underlying shape.</summary>
    public readonly bool IsNull() => myTShape is null;

    /// <summary>Makes this shape null.</summary>
    internal void Nullify()
    {
        myTShape = null;
        myLocation.Clear();
        myOrient = TopAbs_Orientation.TopAbs_EXTERNAL;
    }

    /// <summary>The shape's local coordinate system.</summary>
    public readonly TopLoc_Location Location() => myLocation;

    /// <summary>Sets the shape's local coordinate system.</summary>
    /// <param name="theLoc">The new local coordinate system.</param>
    /// <param name="theRaiseExc">Throw if the transformation has a scale or is negative.</param>
    internal void Location(in TopLoc_Location theLoc, bool theRaiseExc = false)
    {
        gp_Trsf aTrsf = theLoc.Transformation();
        if (theRaiseExc)
        {
            validateTransformation(aTrsf);
        }

        myLocation = theLoc;
    }

    /// <summary>This shape with its local coordinate system set to <paramref name="theLoc"/>.</summary>
    /// <param name="theLoc">The new local coordinate system.</param>
    /// <param name="theRaiseExc">Throw if the transformation has a scale or is negative.</param>
    public readonly TopoDS_Shape Located(in TopLoc_Location theLoc, bool theRaiseExc = false)
    {
        TopoDS_Shape aShape = this;
        aShape.Location(theLoc, theRaiseExc);
        return aShape;
    }

    /// <summary>The shape's orientation.</summary>
    public readonly TopAbs_Orientation Orientation() => myOrient;

    /// <summary>Sets the shape's orientation.</summary>
    internal void Orientation(TopAbs_Orientation theOrient) => myOrient = theOrient;

    /// <summary>This shape with the given orientation.</summary>
    public readonly TopoDS_Shape Oriented(TopAbs_Orientation theOrient)
    {
        TopoDS_Shape aShape = this;
        aShape.Orientation(theOrient);
        return aShape;
    }

    /// <summary>The underlying shape (null for a null shape).</summary>
    public readonly TopoDS_TShape? TShape() => myTShape;

    /// <summary>The type of the shape: VERTEX, EDGE, and so on.</summary>
    /// <exception cref="Standard_NullObject">The shape is null.</exception>
    public readonly TopAbs_ShapeEnum ShapeType() => nonNullTShape().ShapeType();

    /// <summary>The free flag of the underlying shape.</summary>
    public readonly bool Free() => nonNullTShape().Free();

    /// <summary>Sets the free flag of the underlying shape (shared by every shape referring to it).</summary>
    internal readonly void Free(bool theIsFree) => nonNullTShape().Free(theIsFree);

    /// <summary>The locked flag of the underlying shape: true for every frozen shape.</summary>
    public readonly bool Locked() => nonNullTShape().Locked();

    /// <summary>Sets the locked flag of the underlying shape.</summary>
    internal readonly void Locked(bool theIsLocked) => nonNullTShape().Locked(theIsLocked);

    /// <summary>The modification flag of the underlying shape.</summary>
    public readonly bool Modified() => nonNullTShape().Modified();

    /// <summary>Sets the modification flag of the underlying shape.</summary>
    internal readonly void Modified(bool theIsModified) => nonNullTShape().Modified(theIsModified);

    /// <summary>The checked flag of the underlying shape.</summary>
    public readonly bool Checked() => nonNullTShape().Checked();

    /// <summary>Sets the checked flag of the underlying shape.</summary>
    internal readonly void Checked(bool theIsChecked) => nonNullTShape().Checked(theIsChecked);

    /// <summary>The orientability flag of the underlying shape.</summary>
    public readonly bool Orientable() => nonNullTShape().Orientable();

    /// <summary>Sets the orientability flag of the underlying shape.</summary>
    internal readonly void Orientable(bool theIsOrientable) => nonNullTShape().Orientable(theIsOrientable);

    /// <summary>The closedness flag of the underlying shape.</summary>
    public readonly bool Closed() => nonNullTShape().Closed();

    /// <summary>Sets the closedness flag of the underlying shape.</summary>
    internal readonly void Closed(bool theIsClosed) => nonNullTShape().Closed(theIsClosed);

    /// <summary>The infinity flag of the underlying shape.</summary>
    public readonly bool Infinite() => nonNullTShape().Infinite();

    /// <summary>Sets the infinity flag of the underlying shape.</summary>
    internal readonly void Infinite(bool theIsInfinite) => nonNullTShape().Infinite(theIsInfinite);

    /// <summary>The convexness flag of the underlying shape.</summary>
    public readonly bool Convex() => nonNullTShape().Convex();

    /// <summary>Sets the convexness flag of the underlying shape.</summary>
    internal readonly void Convex(bool theIsConvex) => nonNullTShape().Convex(theIsConvex);

    /// <summary>Multiplies the shape's location by <paramref name="thePosition"/>.</summary>
    /// <param name="thePosition">The transformation to apply.</param>
    /// <param name="theRaiseExc">Throw if the transformation has a scale or is negative.</param>
    internal void Move(in TopLoc_Location thePosition, bool theRaiseExc = false)
    {
        gp_Trsf aTrsf = thePosition.Transformation();
        if (theRaiseExc)
        {
            validateTransformation(aTrsf);
        }

        myLocation = thePosition * myLocation;
    }

    /// <summary>This shape with its location multiplied by <paramref name="thePosition"/>.</summary>
    /// <param name="thePosition">The transformation to apply.</param>
    /// <param name="theRaiseExc">Throw if the transformation has a scale or is negative.</param>
    public readonly TopoDS_Shape Moved(in TopLoc_Location thePosition, bool theRaiseExc = false)
    {
        TopoDS_Shape aShape = this;
        aShape.Move(thePosition, theRaiseExc);
        return aShape;
    }

    /// <summary>Reverses the orientation (TopAbs::Reverse).</summary>
    internal void Reverse() => myOrient = TopAbs.Reverse(myOrient);

    /// <summary>This shape with the orientation reversed (TopAbs::Reverse).</summary>
    public readonly TopoDS_Shape Reversed()
    {
        TopoDS_Shape aShape = this;
        aShape.Reverse();
        return aShape;
    }

    /// <summary>Complements the orientation (TopAbs::Complement).</summary>
    internal void Complement() => myOrient = TopAbs.Complement(myOrient);

    /// <summary>This shape with the orientation complemented (TopAbs::Complement).</summary>
    public readonly TopoDS_Shape Complemented()
    {
        TopoDS_Shape aShape = this;
        aShape.Complement();
        return aShape;
    }

    /// <summary>Composes the orientation with <paramref name="theOrient"/> (TopAbs::Compose).</summary>
    internal void Compose(TopAbs_Orientation theOrient) => myOrient = TopAbs.Compose(myOrient, theOrient);

    /// <summary>This shape with the orientation composed with <paramref name="theOrient"/> (TopAbs::Compose).</summary>
    public readonly TopoDS_Shape Composed(TopAbs_Orientation theOrient)
    {
        TopoDS_Shape aShape = this;
        aShape.Compose(theOrient);
        return aShape;
    }

    /// <summary>The number of direct sub-shapes (children); 0 for a null shape.</summary>
    public readonly int NbChildren() => myTShape is null ? 0 : myTShape.NbChildren();

    /// <summary>True if the two shapes share the same TShape. Locations and orientations may differ.</summary>
    public readonly bool IsPartner(in TopoDS_Shape theOther) => ReferenceEquals(myTShape, theOther.TShape());

    /// <summary>True if the two shapes share the same TShape and location. Orientations may differ.</summary>
    public readonly bool IsSame(in TopoDS_Shape theOther)
        => ReferenceEquals(myTShape, theOther.TShape()) && myLocation == theOther.Location();

    /// <summary>True if the two shapes share the same TShape, location and orientation.</summary>
    public readonly bool IsEqual(in TopoDS_Shape theOther)
        => ReferenceEquals(myTShape, theOther.TShape()) && myLocation == theOther.Location()
           && myOrient == theOther.Orientation();

    /// <summary>The negation of <see cref="IsEqual"/>.</summary>
    public readonly bool IsNotEqual(in TopoDS_Shape theOther) => !IsEqual(theOther);

    /// <summary>
    /// Replaces the TShape by a new one with the same geometry and no sub-shapes, keeping the orientation and location.
    /// </summary>
    internal void EmptyCopy() => myTShape = nonNullTShape().EmptyCopy();

    /// <summary>
    /// This shape with a new TShape with the same geometry and no sub-shapes. Internal: the new TShape isn't frozen,
    /// so ported code can fill it (docs/decisions.md, 3103).
    /// </summary>
    internal readonly TopoDS_Shape EmptyCopied()
    {
        TopoDS_Shape aShape = this;
        aShape.EmptyCopy();
        return aShape;
    }

    /// <summary>Sets the underlying shape.</summary>
    internal void TShape(TopoDS_TShape? theTShape) => myTShape = theTShape;

    // Checks if the transformation contains scaling or negative values.
    private static void validateTransformation(in gp_Trsf theTrsf)
    {
        if (Math.Abs(Math.Abs(theTrsf.ScaleFactor()) - 1.0) > TopLoc_Location.ScalePrec() || theTrsf.IsNegative())
        {
            // Exception
            throw new Standard_DomainError("Transformation with scaling transformation is forbidden");
        }
    }

    // OCCT dereferences the null handle (undefined behaviour); the port throws Standard_NullObject.
    private readonly TopoDS_TShape nonNullTShape()
        => myTShape ?? throw new Standard_NullObject("TopoDS_Shape: the shape is null");

    // </generated-members>

    /// <summary>OCCT's <c>operator==</c>: <see cref="IsEqual"/>.</summary>
    public readonly bool Equals(TopoDS_Shape other) => IsEqual(other);

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
    public override readonly int GetHashCode() => HashCodeOf(myTShape, myLocation);

    public static bool operator ==(in TopoDS_Shape theLeft, in TopoDS_Shape theRight) => theLeft.IsEqual(theRight);

    public static bool operator !=(in TopoDS_Shape theLeft, in TopoDS_Shape theRight) => !theLeft.IsEqual(theRight);

    /// <summary>The type and orientation, or "NULL".</summary>
    public override readonly string ToString() => ToStringOf(this);

    internal static bool EqualsObject(in TopoDS_Shape theShape, object? theObj) => theObj switch
    {
        TopoDS_Shape aS => theShape.IsEqual(aS),
        TopoDS_Vertex aS => theShape.IsEqual(aS),
        TopoDS_Edge aS => theShape.IsEqual(aS),
        TopoDS_Wire aS => theShape.IsEqual(aS),
        TopoDS_Face aS => theShape.IsEqual(aS),
        TopoDS_Shell aS => theShape.IsEqual(aS),
        TopoDS_Solid aS => theShape.IsEqual(aS),
        TopoDS_CompSolid aS => theShape.IsEqual(aS),
        TopoDS_Compound aS => theShape.IsEqual(aS),
        _ => false,
    };

    internal static int HashCodeOf(TopoDS_TShape? theTShape, in TopLoc_Location theLocation)
    {
        int aHL = theLocation.HashCode();
        int aHT = RuntimeHelpers.GetHashCode(theTShape);
        return aHL == 0 ? aHT : HashCode.Combine(aHT, aHL);
    }

    internal static string ToStringOf(in TopoDS_Shape theShape) => theShape.IsNull()
        ? "NULL"
        : TopAbs.ShapeTypeToString(theShape.ShapeType()) + " " + TopAbs.ShapeOrientationToString(theShape.Orientation());
}
