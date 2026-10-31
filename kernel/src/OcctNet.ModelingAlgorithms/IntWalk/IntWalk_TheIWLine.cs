// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheIWLineOfTheIWalking.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheIWLineOfTheIWalking_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A line computed by IntWalk_IWalking: its points (an <see cref="IntSurf_LineOn2S"/>), whether it is closed, its
/// first and last start points, the passing points it goes through, and a tangent vector.
/// </summary>
/// <remarks>
/// OCCT has no generic file for it: IntPatch_TheIWLineOfTheIWalking and Contap_TheIWLineOfTheIWalking are two copies
/// of the same class, named by the macro <c>IntWalk_TheIWLine</c> in IntWalk_IWalking.gxx. The port writes the class
/// once under the macro's name, and each instantiation is a sealed subclass with OCCT's name (decision 686), which
/// <see cref="IntWalk_IWalking{TheIWFunction, TheIWLine}"/> builds. A working object (decision 609): its mutators are
/// public, as OCCT's, and <see cref="Line"/> is the line itself, not a copy. <see cref="Value"/>,
/// <see cref="FirstPoint"/> and <see cref="LastPoint"/> return copies (OCCT's <c>const&amp;</c>).
/// </remarks>
public class IntWalk_TheIWLine : Standard_Transient
{
    private readonly IntSurf_LineOn2S line;
    private readonly NCollection_Sequence<IntSurf_Couple> couple = new();
    private bool closed;
    private bool hasFirst;
    private bool hasLast;
    private int firstIndex;
    private int lastIndex;
    private IntSurf_PathPoint theFirstPoint;
    private IntSurf_PathPoint theLastPoint;
    private int indextg;
    private gp_Vec vcttg;
    private bool istgtbeg;
    private bool istgtend;

    /// <summary>An empty line (OCCT's allocator argument is dropped).</summary>
    public IntWalk_TheIWLine()
    {
        line = new IntSurf_LineOn2S();
        closed = false;
        hasFirst = false;
        hasLast = false;
        firstIndex = -1;
        lastIndex = -1;
        theFirstPoint = new IntSurf_PathPoint();
        theLastPoint = new IntSurf_PathPoint();
        indextg = -1;
        vcttg = new gp_Vec();
        istgtbeg = false;
        istgtend = false;
    }

    /// <summary>Reverses the points and renumbers the passing points.</summary>
    public void Reverse()
    {
        line.Reverse();
        int N = line.NbPoints();
        int NbCouple = couple.Length();
        for (int i = 1; i <= NbCouple; i++)
        {
            couple.SetValue(i, new IntSurf_Couple(N - couple.Value(i).First() + 1, couple.Value(i).Second()));
        }
    }

    /// <summary>Removes the points from <paramref name="Index"/> on.</summary>
    public void Cut(int Index)
    {
        _ = line.Split(Index);
    }

    public void AddPoint(in IntSurf_PntOn2S P) => line.Add(P);

    public void AddStatusFirst(bool Closed, bool HasFirst)
    {
        closed = Closed;
        hasFirst = HasFirst;
    }

    public void AddStatusFirst(bool Closed, bool HasFirst, int Index, in IntSurf_PathPoint P)
    {
        closed = Closed;
        hasFirst = HasFirst;
        firstIndex = Index;
        theFirstPoint = P;
    }

    public void AddStatusFirstLast(bool Closed, bool HasFirst, bool HasLast)
    {
        closed = Closed;
        hasFirst = HasFirst;
        hasLast = HasLast;
    }

    public void AddStatusLast(bool HasLast) => hasLast = HasLast;

    public void AddStatusLast(bool HasLast, int Index, in IntSurf_PathPoint P)
    {
        hasLast = HasLast;
        lastIndex = Index;
        theLastPoint = P;
    }

    /// <summary>Records that the next point added is the passing point <paramref name="Index"/>.</summary>
    public void AddIndexPassing(int Index) => couple.Append(new IntSurf_Couple(line.NbPoints() + 1, Index));

    public void SetTangentVector(in gp_Vec V, int Index)
    {
        indextg = Index;
        vcttg = V;
    }

    public void SetTangencyAtBegining(bool IsTangent) => istgtbeg = IsTangent;

    public void SetTangencyAtEnd(bool IsTangent) => istgtend = IsTangent;

    public int NbPoints() => line.NbPoints();

    public IntSurf_PntOn2S Value(int Index) => line.Value(Index);

    /// <summary>The line's points (the line itself).</summary>
    public IntSurf_LineOn2S Line() => line;

    public bool IsClosed() => closed;

    public bool HasFirstPoint() => hasFirst;

    public bool HasLastPoint() => hasLast;

    /// <summary>The first start point. Throws Standard_DomainError if there is none.</summary>
    public IntSurf_PathPoint FirstPoint()
    {
        if (!hasFirst)
        {
            throw new Standard_DomainError();
        }
        return theFirstPoint;
    }

    public int FirstPointIndex()
    {
        if (!hasFirst)
        {
            throw new Standard_DomainError();
        }
        return firstIndex;
    }

    /// <summary>The last start point. Throws Standard_DomainError if there is none.</summary>
    public IntSurf_PathPoint LastPoint()
    {
        if (!hasLast)
        {
            throw new Standard_DomainError();
        }
        return theLastPoint;
    }

    public int LastPointIndex()
    {
        if (!hasLast)
        {
            throw new Standard_DomainError();
        }
        return lastIndex;
    }

    public int NbPassingPoint() => couple.Length();

    public void PassingPoint(int Index, out int IndexLine, out int IndexPnts)
    {
        IndexLine = couple.Value(Index).First();
        IndexPnts = couple.Value(Index).Second();
    }

    /// <summary>The tangent vector and, in <paramref name="Index"/>, the index of the point it is at.</summary>
    public gp_Vec TangentVector(out int Index)
    {
        Index = indextg;
        return vcttg;
    }

    public bool IsTangentAtBegining() => istgtbeg;

    public bool IsTangentAtEnd() => istgtend;
}
