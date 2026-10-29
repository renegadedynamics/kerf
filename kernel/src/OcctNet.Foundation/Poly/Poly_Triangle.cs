// Port of: src/FoundationClasses/TKMath/Poly/Poly_Triangle.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using System.Globalization;

namespace Occt;

/// <summary>
/// A triangle of a triangulation (<see cref="Poly_Triangulation"/>): three node indices, each in
/// [1, Poly_Triangulation.NbNodes()].
/// </summary>
/// <remarks>
/// Three <c>int</c>s and nothing else, so an array of triangles reads as an array of 3 × n indices
/// (<see cref="Poly_Triangulation.TriangleIndices"/>). The default triangle has all three indices 0, as OCCT's.
/// </remarks>
public partial struct Poly_Triangle : IEquatable<Poly_Triangle>
{
    private NCollection_Buffer3<int> myNodes;

    /// <summary>A triangle with the three node indices.</summary>
    public Poly_Triangle(int theN1, int theN2, int theN3)
    {
        myNodes[0] = theN1;
        myNodes[1] = theN2;
        myNodes[2] = theN3;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the three node indices.</summary>
    internal void Set(int theN1, int theN2, int theN3)
    {
        myNodes[0] = theN1;
        myNodes[1] = theN2;
        myNodes[2] = theN3;
    }

    /// <summary>Sets the node index at position 1, 2 or 3; throws <see cref="Standard_OutOfRange"/> otherwise.</summary>
    internal void Set(int theIndex, int theNode)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > 3, "Poly_Triangle::Set(), invalid index");
        myNodes[theIndex - 1] = theNode;
    }

    /// <summary>The node index at position 1, 2 or 3, for writing.</summary>
    [UnscopedRef]
    internal ref int ChangeValue(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > 3, "Poly_Triangle::ChangeValue(), invalid index");
        return ref myNodes[theIndex - 1];
    }

    // Non-mutating twins

    /// <summary>This triangle with the node index at position 1, 2 or 3 replaced (the twin of <c>Set(index, node)</c>).</summary>
    public readonly Poly_Triangle WithNode(int theIndex, int theNode)
    {
        Poly_Triangle aCopy = this;
        aCopy.Set(theIndex, theNode);
        return aCopy;
    }

    /// <summary>The three node indices.</summary>
    public readonly void Get(out int theN1, out int theN2, out int theN3)
    {
        theN1 = myNodes[0];
        theN2 = myNodes[1];
        theN3 = myNodes[2];
    }

    /// <summary>The node index at position 1, 2 or 3; throws <see cref="Standard_OutOfRange"/> otherwise.</summary>
    public readonly int Value(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > 3, "Poly_Triangle::Value(), invalid index");
        return myNodes[theIndex - 1];
    }

    /// <summary>The node index at position 1, 2 or 3 (OCCT's <c>operator()</c>).</summary>
    public readonly int this[int theIndex] => Value(theIndex);

    // Equality and formatting (OCCT.NET additions)

    public readonly bool Equals(Poly_Triangle other) => myNodes[0] == other.myNodes[0] && myNodes[1] == other.myNodes[1] && myNodes[2] == other.myNodes[2];

    public override readonly bool Equals(object? obj) => obj is Poly_Triangle other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myNodes[0], myNodes[1], myNodes[2]);

    public static bool operator ==(in Poly_Triangle theLeft, in Poly_Triangle theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Poly_Triangle theLeft, in Poly_Triangle theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Poly_Triangle({myNodes[0]}, {myNodes[1]}, {myNodes[2]})");
}
