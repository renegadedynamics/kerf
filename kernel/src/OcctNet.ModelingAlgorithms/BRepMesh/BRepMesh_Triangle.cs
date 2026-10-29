// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Triangle.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;
using static Occt.BRepMesh_DegreeOfFreedom;

namespace Occt;

/// <summary>A triangle of the mesh: three links (by index) with their orientations, and a movability.</summary>
/// <remarks>
/// OCCT's public arrays <c>myEdges[3]</c> and <c>myOrientations[3]</c> are internal inline arrays here, read by ported
/// code as in C++ (<c>var e = aElement.myEdges; e[i]</c>); callers use <see cref="Edges"/>. The default triangle has
/// links 0, 0, 0, orientations false and is free, as OCCT's; <c>default(T)</c> is the same.
/// <see cref="Equals(BRepMesh_Triangle)"/> and <c>==</c> compare the fields exactly (decision 4402); OCCT's
/// <c>operator==</c> (the same links in the same cyclic order, never a deleted triangle) is <see cref="IsEqual"/>,
/// and maps in ported code take <see cref="Hasher"/>.
/// </remarks>
public partial struct BRepMesh_Triangle : IEquatable<BRepMesh_Triangle>
{
    /// <summary>Three link indices (C++'s <c>int[3]</c>).</summary>
    [InlineArray(3)]
    internal struct Int3
    {
        private int myElement0;
    }

    /// <summary>Three orientations (C++'s <c>bool[3]</c>).</summary>
    [InlineArray(3)]
    internal struct Bool3
    {
        private bool myElement0;
    }

    internal Int3 myEdges;
    internal Bool3 myOrientations;
    internal BRepMesh_DegreeOfFreedom myMovability;

    /// <summary>A triangle of the given three links (by index) and orientations.</summary>
    public BRepMesh_Triangle(ReadOnlySpan<int> theEdges, ReadOnlySpan<bool> theOrientations, BRepMesh_DegreeOfFreedom theMovability)
    {
        Initialize(theEdges, theOrientations, theMovability);
    }

    /// <summary>Sets the links, the orientations and the movability.</summary>
    internal void Initialize(ReadOnlySpan<int> theEdges, ReadOnlySpan<bool> theOrientations, BRepMesh_DegreeOfFreedom theMovability)
    {
        theEdges[..3].CopyTo(myEdges);
        theOrientations[..3].CopyTo(myOrientations);
        myMovability = theMovability;
    }

    /// <summary>The links and their orientations, each into a span of three.</summary>
    public readonly void Edges(Span<int> theEdges, Span<bool> theOrientations)
    {
        ((ReadOnlySpan<int>)myEdges).CopyTo(theEdges);
        ((ReadOnlySpan<bool>)myOrientations).CopyTo(theOrientations);
    }

    /// <summary>The index of the link at position 0, 1 or 2.</summary>
    public readonly int Edge(int theIndex) => myEdges[theIndex];

    /// <summary>The orientation of the link at position 0, 1 or 2.</summary>
    public readonly bool Orientation(int theIndex) => myOrientations[theIndex];

    /// <summary>The movability.</summary>
    public readonly BRepMesh_DegreeOfFreedom Movability() => myMovability;

    /// <summary>Sets the movability.</summary>
    internal void SetMovability(BRepMesh_DegreeOfFreedom theMovability) => myMovability = theMovability;

    /// <summary>This triangle with another movability (the twin of <c>SetMovability</c>).</summary>
    public readonly BRepMesh_Triangle WithMovability(BRepMesh_DegreeOfFreedom theMovability)
    {
        BRepMesh_Triangle aCopy = this;
        aCopy.SetMovability(theMovability);
        return aCopy;
    }

    /// <summary>OCCT's equality: false if either triangle is deleted, else whether they have the same links in the
    /// same cyclic order.</summary>
    public readonly bool IsEqual(in BRepMesh_Triangle theOther)
    {
        if (myMovability == BRepMesh_Deleted || theOther.myMovability == BRepMesh_Deleted)
        {
            return false;
        }

        if (myEdges[0] == theOther.myEdges[0] && myEdges[1] == theOther.myEdges[1]
            && myEdges[2] == theOther.myEdges[2])
        {
            return true;
        }

        if (myEdges[0] == theOther.myEdges[1] && myEdges[1] == theOther.myEdges[2]
            && myEdges[2] == theOther.myEdges[0])
        {
            return true;
        }

        if (myEdges[0] == theOther.myEdges[2] && myEdges[1] == theOther.myEdges[0]
            && myEdges[2] == theOther.myEdges[1])
        {
            return true;
        }

        return false;
    }

    /// <summary>
    /// OCCT's hasher: <c>operator==</c> (<see cref="IsEqual"/>) and a hash of the sorted link indices (OCCT hashes their
    /// bytes; the value differs, which only changes the order of a non-indexed map).
    /// </summary>
    public sealed class Hasher : IEqualityComparer<BRepMesh_Triangle>
    {
        public static readonly Hasher Instance = new();

        public bool Equals(BRepMesh_Triangle theT1, BRepMesh_Triangle theT2) => theT1.IsEqual(theT2);

        public int GetHashCode(BRepMesh_Triangle theTriangle)
        {
            Span<int> aCombination = [theTriangle.myEdges[0], theTriangle.myEdges[1], theTriangle.myEdges[2]];
            aCombination.Sort();
            return HashCode.Combine(aCombination[0], aCombination[1], aCombination[2]);
        }
    }

    public readonly bool Equals(BRepMesh_Triangle other)
        => myEdges[0] == other.myEdges[0] && myEdges[1] == other.myEdges[1] && myEdges[2] == other.myEdges[2]
           && myOrientations[0] == other.myOrientations[0] && myOrientations[1] == other.myOrientations[1]
           && myOrientations[2] == other.myOrientations[2] && myMovability == other.myMovability;

    public override readonly bool Equals(object? obj) => obj is BRepMesh_Triangle other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(myEdges[0], myEdges[1], myEdges[2], myOrientations[0], myOrientations[1], myOrientations[2], myMovability);

    public static bool operator ==(in BRepMesh_Triangle theLeft, in BRepMesh_Triangle theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in BRepMesh_Triangle theLeft, in BRepMesh_Triangle theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"BRepMesh_Triangle({(myOrientations[0] ? "" : "-")}{myEdges[0]}, {(myOrientations[1] ? "" : "-")}{myEdges[1]}, {(myOrientations[2] ? "" : "-")}{myEdges[2]}, {myMovability})";
}
