// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShape.hxx
// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShape.lxx
// Port of: src/ModelingAlgorithms/TKPrim/Sweep/Sweep_NumShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// An indexed edge or vertex: the directing shape of a sweep. For an edge, the index is the number of vertices (0, 1
/// or 2), with flags for a closed edge and an edge infinite at its beginning or end; for a vertex, the index is its
/// rank on the edge (1 or 2) and the closed flag is the edge's.
/// </summary>
/// <remarks>A value type (OCCT's class used by value). <c>default</c> has the type TopAbs_COMPOUND (0): ported code writes
/// <c>new Sweep_NumShape()</c>, OCCT's dummy shape of type TopAbs_SHAPE.</remarks>
internal struct Sweep_NumShape : IEquatable<Sweep_NumShape>
{
    private ShapeType myType;
    private int myIndex;
    private bool myClosed;
    private bool myBegInf;
    private bool myEndInf;

    /// <summary>A dummy indexed edge.</summary>
    public Sweep_NumShape()
    {
        myType = ShapeType.Shape;
        myIndex = 0;
        myClosed = false;
        myBegInf = false;
        myEndInf = false;
    }

    /// <summary>A simple indexed edge or vertex.</summary>
    public Sweep_NumShape(int Index, ShapeType Type, bool Closed = false, bool BegInf = false, bool EndInf = false)
    {
        myType = Type;
        myIndex = Index;
        myClosed = Closed;
        myBegInf = BegInf;
        myEndInf = EndInf;
    }

    /// <summary>Reinitializes the indexed edge or vertex.</summary>
    internal void Init(int Index, ShapeType Type, bool Closed = false, bool BegInf = false, bool EndInf = false)
    {
        myIndex = Index;
        myType = Type;
        myClosed = Closed;
        myBegInf = BegInf;
        myEndInf = EndInf;
    }

    public readonly int Index() => myIndex;

    public readonly ShapeType Type() => myType;

    public readonly bool Closed() => myClosed;

    public readonly bool BegInfinite() => myBegInf;

    public readonly bool EndInfinite() => myEndInf;

    public readonly Orientation Orientation()
    {
        if (myType == ShapeType.Edge)
        {
            return Forward;
        }
        else
        {
            if (myIndex == 2)
            {
                return Forward;
            }
            else
            {
                return Reversed;
            }
        }
    }

    public readonly bool Equals(Sweep_NumShape other) =>
        myType == other.myType && myIndex == other.myIndex && myClosed == other.myClosed && myBegInf == other.myBegInf
        && myEndInf == other.myEndInf;

    public override readonly bool Equals(object? obj) => obj is Sweep_NumShape other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myType, myIndex, myClosed, myBegInf, myEndInf);

    public static bool operator ==(Sweep_NumShape left, Sweep_NumShape right) => left.Equals(right);

    public static bool operator !=(Sweep_NumShape left, Sweep_NumShape right) => !left.Equals(right);
}
