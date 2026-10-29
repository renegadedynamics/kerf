// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_SectionLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_SectionLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A chain of start points: a line of the intersection of two triangulations.</summary>
/// <remarks>
/// <para>
/// The sequence always ends with one spare start point once initialized: <see cref="NbStartPoints"/> is one less than
/// <see cref="GetN"/>, and appending writes the spare item then adds a new spare (OCCT's
/// <c>SectionLine[NbStartPoints()] = SP; IncrementNbStartPoints();</c>).
/// </para>
/// <para>
/// A class: OCCT copies section lines by value (copy constructor, <c>operator=</c>), which ported code writes as
/// <c>new IntPolyh_SectionLine(other)</c> or <see cref="Copy"/>. A working object of the intersection: readers public,
/// mutators internal.
/// </para>
/// </remarks>
public sealed class IntPolyh_SectionLine : IntPolyh_IDump
{
    private readonly NCollection_Sequence<IntPolyh_StartPoint> mySeqOfSPoints = new();

    /// <summary>An empty section line (no spare item: call <see cref="Init"/>).</summary>
    public IntPolyh_SectionLine()
    {
    }

    /// <summary>An initialized section line (the size is ignored, as in OCCT).</summary>
    public IntPolyh_SectionLine(int N) => Init(N);

    /// <summary>A copy of the other section line.</summary>
    public IntPolyh_SectionLine(IntPolyh_SectionLine theOther) => Copy(theOther);

    /// <summary>Adds the spare start point if the line is empty (the size is ignored, as in OCCT).</summary>
    internal void Init(int N)
    {
        _ = N;
        if (mySeqOfSPoints.Length() == 0)
        {
            IncrementNbStartPoints();
        }
    }

    /// <summary>The start point at <paramref name="Index"/>, from 0.</summary>
    public IntPolyh_StartPoint Value(int Index) => mySeqOfSPoints.Value(Index + 1);

    /// <summary>The start point at <paramref name="Index"/>, from 0, for writing.</summary>
    internal ref IntPolyh_StartPoint ChangeValue(int Index) => ref mySeqOfSPoints.ChangeValue(Index + 1);

    /// <summary>C++'s <c>operator[]</c>.</summary>
    internal ref IntPolyh_StartPoint this[int Index] => ref ChangeValue(Index);

    /// <summary>Copies the other line's start points; C++'s <c>operator=</c>.</summary>
    internal IntPolyh_SectionLine Copy(IntPolyh_SectionLine Other)
    {
        mySeqOfSPoints.Assign(Other.mySeqOfSPoints);
        return this;
    }

    /// <summary>The number of items, the spare one included.</summary>
    public int GetN() => mySeqOfSPoints.Length();

    /// <summary>The number of start points.</summary>
    public int NbStartPoints() => mySeqOfSPoints.Length() - 1;

    /// <summary>Appends a default start point.</summary>
    internal void IncrementNbStartPoints()
    {
        IntPolyh_StartPoint aSP = new();
        mySeqOfSPoints.Append(aSP);
    }

    /// <summary>Does nothing (OCCT's leftover of an array implementation).</summary>
    internal void Destroy()
    {
    }

    /// <summary>Writes the start points as OCCT's <c>Dump()</c> prints them.</summary>
    public void Dump(TextWriter theStream)
    {
        theStream.Write($"\n SectionLine 0-> {IntPolyh_Format.D(NbStartPoints() - 1)}");
        for (int i = 0; i < NbStartPoints(); i++)
        {
            Value(i).Dump(i, theStream);
        }

        theStream.Write("\n");
    }

    /// <summary>Inserts the start point at the beginning.</summary>
    internal void Prepend(in IntPolyh_StartPoint SP) => mySeqOfSPoints.Prepend(SP);
}
