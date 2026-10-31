// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_CoupleOfShape.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A pair of shapes.</summary>
/// <remarks>
/// A value: a struct with internal setters and <c>With</c> twins (translation rules, section 3). Equality compares the
/// two shapes with <see cref="TopoDS_Shape"/>'s equality (same TShape, location and orientation).
/// </remarks>
public partial struct BOPTools_CoupleOfShape : IEquatable<BOPTools_CoupleOfShape>
{
    private TopoDS_Shape myShape1;
    private TopoDS_Shape myShape2;

    /// <summary>Sets the first shape.</summary>
    internal void SetShape1(in TopoDS_Shape theShape) => myShape1 = theShape;

    /// <summary>The first shape.</summary>
    public readonly TopoDS_Shape Shape1() => myShape1;

    /// <summary>Sets the second shape.</summary>
    internal void SetShape2(in TopoDS_Shape theShape) => myShape2 = theShape;

    /// <summary>The second shape.</summary>
    public readonly TopoDS_Shape Shape2() => myShape2;

    /// <summary>A copy with the first shape replaced.</summary>
    public readonly BOPTools_CoupleOfShape WithShape1(in TopoDS_Shape theShape)
    {
        BOPTools_CoupleOfShape aCopy = this;
        aCopy.SetShape1(theShape);
        return aCopy;
    }

    /// <summary>A copy with the second shape replaced.</summary>
    public readonly BOPTools_CoupleOfShape WithShape2(in TopoDS_Shape theShape)
    {
        BOPTools_CoupleOfShape aCopy = this;
        aCopy.SetShape2(theShape);
        return aCopy;
    }

    /// <inheritdoc/>
    public readonly bool Equals(BOPTools_CoupleOfShape theOther) => myShape1.Equals(theOther.myShape1) && myShape2.Equals(theOther.myShape2);

    /// <inheritdoc/>
    public override readonly bool Equals(object? theObj) => theObj is BOPTools_CoupleOfShape aC && Equals(aC);

    /// <inheritdoc/>
    public override readonly int GetHashCode() => HashCode.Combine(myShape1, myShape2);

    /// <summary>Field-by-field equality.</summary>
    public static bool operator ==(BOPTools_CoupleOfShape theLeft, BOPTools_CoupleOfShape theRight) => theLeft.Equals(theRight);

    /// <summary>Field-by-field inequality.</summary>
    public static bool operator !=(BOPTools_CoupleOfShape theLeft, BOPTools_CoupleOfShape theRight) => !theLeft.Equals(theRight);

    /// <inheritdoc/>
    public override readonly string ToString() => $"BOPTools_CoupleOfShape({myShape1}, {myShape2})";
}
