// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_FindContigousEdges.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_FindContigousEdges.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;

namespace Kerf.Engine.Healing;

/// <summary>
/// Finds the contiguous edges of the shapes added (edges shared by two faces within the tolerance), with
/// <see cref="Sewing"/> in its analysis-only mode (no sewing, cutting by default).
/// </summary>
/// <remarks>
/// A working object over the sewing, which copies the shapes it is given (decision 583). OCCT declares
/// <c>NbEdges</c> without defining it; it is left out.
/// </remarks>
internal sealed class FindContiguousEdges
{
    private readonly Sewing _sewing;

    //=================================================================================================

    /// <summary>
    /// The tool with the tolerance <paramref name="tolerance"/>; <paramref name="option"/> true analyses the
    /// degenerated shapes too.
    /// </summary>
    public FindContiguousEdges(double tolerance = 1.0e-06, bool option = true)
    {
        _sewing = new Sewing();
        Init(tolerance, option);
    }

    //=================================================================================================

    /// <summary>Resets the tool with the tolerance and option.</summary>
    public void Init(double tolerance, bool option)
    {
        _sewing.Init(tolerance, option, false, true);
    }

    //=================================================================================================

    /// <summary>Adds a shape (a face, shell, solid...) to analyse.</summary>
    public void Add(in Shape aShape)
    {
        _sewing.Add(aShape);
    }

    //=================================================================================================

    /// <summary>Finds the contiguous edges.</summary>
    public void Perform()
    {
        _sewing.Perform();
    }

    //=================================================================================================

    /// <summary>The number of contiguous edges.</summary>
    public int ContigousEdgeCount => _sewing.ContigousEdgeCount;

    //=================================================================================================

    /// <summary>The contiguous edge of index <paramref name="index"/>.</summary>
    /// <exception cref="OutOfRangeException">The index is out of range.</exception>
    public Edge ContigousEdge(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > ContigousEdgeCount,
                                     "BRepOffsetAPI_FindContigousEdges::ContigousEdge");
        return _sewing.ContigousEdge(index);
    }

    //=================================================================================================

    /// <summary>The edges of the shapes added that make the contiguous edge of index <paramref name="index"/>.</summary>
    /// <exception cref="OutOfRangeException">The index is out of range.</exception>
    public ISchematiqList<Shape> ContigousEdgeCouple(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > ContigousEdgeCount,
                                     "BRepOffsetAPI_FindContigousEdges::ContigousEdgeCouple");
        return _sewing.ContigousEdgeCouple(index);
    }

    //=================================================================================================

    /// <summary>The boundary edge a section of a contiguous edge comes from.</summary>
    /// <exception cref="NoSuchObjectException">The edge isn't a section.</exception>
    public Edge SectionToBoundary(in Edge section)
    {
        Standard_NoSuchObject_Raise_if(!_sewing.IsSectionBound(section),
                                       "BRepOffsetAPI_FindContigousEdges::SectionToBoundary");
        return _sewing.SectionToBoundary(section);
    }

    //=================================================================================================

    /// <summary>The number of degenerated shapes found.</summary>
    public int DegeneratedShapeCount => _sewing.DegeneratedShapeCount;

    //=================================================================================================

    /// <summary>The degenerated shape of index <paramref name="index"/>.</summary>
    /// <exception cref="OutOfRangeException">The index is out of range.</exception>
    public Shape DegeneratedShape(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > DegeneratedShapeCount,
                                     "BRepOffsetAPI_FindContigousEdges::DegereratedShape");
        return _sewing.DegeneratedShape(index);
    }

    //=================================================================================================

    /// <summary>True if the shape is degenerated.</summary>
    public bool IsDegenerated(in Shape aShape) => _sewing.IsDegenerated(aShape);

    //=================================================================================================

    /// <summary>True if the shape was modified.</summary>
    public bool IsModified(in Shape aShape) => _sewing.IsModified(aShape);

    //=================================================================================================

    /// <summary>The shape a shape added was modified into.</summary>
    /// <exception cref="NoSuchObjectException">The shape wasn't modified.</exception>
    public Shape Modified(in Shape aShape)
    {
        Standard_NoSuchObject_Raise_if(!IsModified(aShape), "BRepOffsetAPI_FindContigousEdges::Modified");
        return _sewing.Modified(aShape);
    }

    //=================================================================================================

    /// <summary>Writes the sewing's information to the console.</summary>
    public void Dump()
    {
        _sewing.Dump();
    }
}
