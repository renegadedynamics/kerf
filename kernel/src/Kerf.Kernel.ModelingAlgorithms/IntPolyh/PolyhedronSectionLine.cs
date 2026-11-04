// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_SectionLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_SectionLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Intersection.Polyhedra;

/// <summary>A chain of start points: a line of the intersection of two triangulations.</summary>
/// <remarks>
/// <para>
/// The sequence always ends with one spare start point once initialized: <see cref="StartPointCount"/> is one less than
/// <see cref="GetN"/>, and appending writes the spare item then adds a new spare (OCCT's
/// <c>SectionLine[NbStartPoints()] = SP; IncrementNbStartPoints();</c>).
/// </para>
/// <para>
/// A class: OCCT copies section lines by value (copy constructor, <c>operator=</c>), which ported code writes as
/// <c>new IntPolyh_SectionLine(other)</c> or <see cref="Copy"/>. A working object of the intersection: readers public,
/// mutators internal.
/// </para>
/// </remarks>
internal sealed class PolyhedronSectionLine : IPolyhedronDump
{
    private readonly NCollection_Sequence<PolyhedronStartPoint> _seqOfSPoints = new();

    /// <summary>An empty section line (no spare item: call <see cref="Init"/>).</summary>
    public PolyhedronSectionLine()
    {
    }

    /// <summary>An initialized section line (the size is ignored, as in OCCT).</summary>
    public PolyhedronSectionLine(int n) => Init(n);

    /// <summary>A copy of the other section line.</summary>
    public PolyhedronSectionLine(PolyhedronSectionLine other) => Copy(other);

    /// <summary>Adds the spare start point if the line is empty (the size is ignored, as in OCCT).</summary>
    internal void Init(int n)
    {
        _ = n;
        if (_seqOfSPoints.Length() == 0)
        {
            IncrementNbStartPoints();
        }
    }

    /// <summary>The start point at <paramref name="index"/>, from 0.</summary>
    public PolyhedronStartPoint Value(int index) => _seqOfSPoints.Value(index + 1);

    /// <summary>The start point at <paramref name="index"/>, from 0, for writing.</summary>
    internal ref PolyhedronStartPoint ChangeValue(int index) => ref _seqOfSPoints.ChangeValue(index + 1);

    /// <summary>C++'s <c>operator[]</c>.</summary>
    internal ref PolyhedronStartPoint this[int index] => ref ChangeValue(index);

    /// <summary>Copies the other line's start points; C++'s <c>operator=</c>.</summary>
    internal PolyhedronSectionLine Copy(PolyhedronSectionLine other)
    {
        _seqOfSPoints.Assign(other._seqOfSPoints);
        return this;
    }

    /// <summary>The number of items, the spare one included.</summary>
    public int GetN() => _seqOfSPoints.Length();

    /// <summary>The number of start points.</summary>
    public int StartPointCount => _seqOfSPoints.Length() - 1;

    /// <summary>Appends a default start point.</summary>
    internal void IncrementNbStartPoints()
    {
        PolyhedronStartPoint sp = new();
        _seqOfSPoints.Append(sp);
    }

    /// <summary>Does nothing (OCCT's leftover of an array implementation).</summary>
    internal void Destroy()
    {
    }

    /// <summary>Writes the start points as OCCT's <c>Dump()</c> prints them.</summary>
    public void Dump(TextWriter stream)
    {
        stream.Write($"\n SectionLine 0-> {Format.D(StartPointCount - 1)}");
        for (int i = 0; i < StartPointCount; i++)
        {
            Value(i).Dump(i, stream);
        }

        stream.Write("\n");
    }

    /// <summary>Inserts the start point at the beginning.</summary>
    internal void Prepend(in PolyhedronStartPoint sp) => _seqOfSPoints.Prepend(sp);
}
