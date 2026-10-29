// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Edge.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An edge between two IntPolyh points (by index), linked to two IntPolyh triangles (by index).</summary>
/// <remarks>
/// OCCT's default edge has every index -1; <c>default(IntPolyh_Edge)</c> has zeros, so ported code writes
/// <c>new IntPolyh_Edge()</c>.
/// </remarks>
public partial struct IntPolyh_Edge : IEquatable<IntPolyh_Edge>
{
    private int myPoint1;
    private int myPoint2;
    private int myTriangle1;
    private int myTriangle2;

    /// <summary>An edge with no points and no triangles (every index -1).</summary>
    public IntPolyh_Edge()
    {
        myPoint1 = -1;
        myPoint2 = -1;
        myTriangle1 = -1;
        myTriangle2 = -1;
    }

    public IntPolyh_Edge(int thePoint1, int thePoint2, int theTriangle1, int theTriangle2)
    {
        myPoint1 = thePoint1;
        myPoint2 = thePoint2;
        myTriangle1 = theTriangle1;
        myTriangle2 = theTriangle2;
    }

    public readonly int FirstPoint() => myPoint1;

    public readonly int SecondPoint() => myPoint2;

    public readonly int FirstTriangle() => myTriangle1;

    public readonly int SecondTriangle() => myTriangle2;

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetFirstPoint(int thePoint) => myPoint1 = thePoint;

    internal void SetSecondPoint(int thePoint) => myPoint2 = thePoint;

    internal void SetFirstTriangle(int theTriangle) => myTriangle1 = theTriangle;

    internal void SetSecondTriangle(int theTriangle) => myTriangle2 = theTriangle;

    public readonly IntPolyh_Edge WithFirstPoint(int thePoint)
    {
        IntPolyh_Edge aCopy = this;
        aCopy.SetFirstPoint(thePoint);
        return aCopy;
    }

    public readonly IntPolyh_Edge WithSecondPoint(int thePoint)
    {
        IntPolyh_Edge aCopy = this;
        aCopy.SetSecondPoint(thePoint);
        return aCopy;
    }

    public readonly IntPolyh_Edge WithFirstTriangle(int theTriangle)
    {
        IntPolyh_Edge aCopy = this;
        aCopy.SetFirstTriangle(theTriangle);
        return aCopy;
    }

    public readonly IntPolyh_Edge WithSecondTriangle(int theTriangle)
    {
        IntPolyh_Edge aCopy = this;
        aCopy.SetSecondTriangle(theTriangle);
        return aCopy;
    }

    /// <summary>Writes the edge with its index, as OCCT's <c>Dump(i)</c> prints it.</summary>
    public readonly void Dump(int i, TextWriter theStream)
    {
        theStream.Write($"\nEdge({IntPolyh_Format.D(i, 3)}) : P1:{IntPolyh_Format.D(myPoint1, 5)}   "
                        + $"P2:{IntPolyh_Format.D(myPoint2, 5)}   T1:{IntPolyh_Format.D(myTriangle1, 5)}   "
                        + $"T2:{IntPolyh_Format.D(myTriangle2, 5)}\n");
    }

    public readonly bool Equals(IntPolyh_Edge other)
        => myPoint1 == other.myPoint1 && myPoint2 == other.myPoint2 && myTriangle1 == other.myTriangle1
           && myTriangle2 == other.myTriangle2;

    public override readonly bool Equals(object? obj) => obj is IntPolyh_Edge other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myPoint1, myPoint2, myTriangle1, myTriangle2);

    public static bool operator ==(in IntPolyh_Edge theLeft, in IntPolyh_Edge theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntPolyh_Edge theLeft, in IntPolyh_Edge theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"IntPolyh_Edge(P {myPoint1}, {myPoint2}; T {myTriangle1}, {myTriangle2})";
}
