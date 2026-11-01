// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_FindContigousEdges.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_FindContigousEdges.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;

namespace Occt;

/// <summary>
/// Finds the contiguous edges of the shapes added (edges shared by two faces within the tolerance), with
/// <see cref="BRepBuilderAPI_Sewing"/> in its analysis-only mode (no sewing, cutting by default).
/// </summary>
/// <remarks>
/// A working object over the sewing, which copies the shapes it is given (decision 583). OCCT declares
/// <c>NbEdges</c> without defining it; it is left out.
/// </remarks>
public sealed class BRepOffsetAPI_FindContigousEdges
{
    private readonly BRepBuilderAPI_Sewing mySewing;

    //=================================================================================================

    /// <summary>
    /// The tool with the tolerance <paramref name="tolerance"/>; <paramref name="option"/> true analyses the
    /// degenerated shapes too.
    /// </summary>
    public BRepOffsetAPI_FindContigousEdges(double tolerance = 1.0e-06, bool option = true)
    {
        mySewing = new BRepBuilderAPI_Sewing();
        Init(tolerance, option);
    }

    //=================================================================================================

    /// <summary>Resets the tool with the tolerance and option.</summary>
    public void Init(double tolerance, bool option)
    {
        mySewing.Init(tolerance, option, false, true);
    }

    //=================================================================================================

    /// <summary>Adds a shape (a face, shell, solid...) to analyse.</summary>
    public void Add(in TopoDS_Shape aShape)
    {
        mySewing.Add(aShape);
    }

    //=================================================================================================

    /// <summary>Finds the contiguous edges.</summary>
    public void Perform()
    {
        mySewing.Perform();
    }

    //=================================================================================================

    /// <summary>The number of contiguous edges.</summary>
    public int NbContigousEdges() => mySewing.NbContigousEdges();

    //=================================================================================================

    /// <summary>The contiguous edge of index <paramref name="index"/>.</summary>
    /// <exception cref="Standard_OutOfRange">The index is out of range.</exception>
    public TopoDS_Edge ContigousEdge(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > NbContigousEdges(),
                                     "BRepOffsetAPI_FindContigousEdges::ContigousEdge");
        return mySewing.ContigousEdge(index);
    }

    //=================================================================================================

    /// <summary>The edges of the shapes added that make the contiguous edge of index <paramref name="index"/>.</summary>
    /// <exception cref="Standard_OutOfRange">The index is out of range.</exception>
    public IBentoList<TopoDS_Shape> ContigousEdgeCouple(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > NbContigousEdges(),
                                     "BRepOffsetAPI_FindContigousEdges::ContigousEdgeCouple");
        return mySewing.ContigousEdgeCouple(index);
    }

    //=================================================================================================

    /// <summary>The boundary edge a section of a contiguous edge comes from.</summary>
    /// <exception cref="Standard_NoSuchObject">The edge isn't a section.</exception>
    public TopoDS_Edge SectionToBoundary(in TopoDS_Edge section)
    {
        Standard_NoSuchObject_Raise_if(!mySewing.IsSectionBound(section),
                                       "BRepOffsetAPI_FindContigousEdges::SectionToBoundary");
        return mySewing.SectionToBoundary(section);
    }

    //=================================================================================================

    /// <summary>The number of degenerated shapes found.</summary>
    public int NbDegeneratedShapes() => mySewing.NbDegeneratedShapes();

    //=================================================================================================

    /// <summary>The degenerated shape of index <paramref name="index"/>.</summary>
    /// <exception cref="Standard_OutOfRange">The index is out of range.</exception>
    public TopoDS_Shape DegeneratedShape(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > NbDegeneratedShapes(),
                                     "BRepOffsetAPI_FindContigousEdges::DegereratedShape");
        return mySewing.DegeneratedShape(index);
    }

    //=================================================================================================

    /// <summary>True if the shape is degenerated.</summary>
    public bool IsDegenerated(in TopoDS_Shape aShape) => mySewing.IsDegenerated(aShape);

    //=================================================================================================

    /// <summary>True if the shape was modified.</summary>
    public bool IsModified(in TopoDS_Shape aShape) => mySewing.IsModified(aShape);

    //=================================================================================================

    /// <summary>The shape a shape added was modified into.</summary>
    /// <exception cref="Standard_NoSuchObject">The shape wasn't modified.</exception>
    public TopoDS_Shape Modified(in TopoDS_Shape aShape)
    {
        Standard_NoSuchObject_Raise_if(!IsModified(aShape), "BRepOffsetAPI_FindContigousEdges::Modified");
        return mySewing.Modified(aShape);
    }

    //=================================================================================================

    /// <summary>Writes the sewing's information to the console.</summary>
    public void Dump()
    {
        mySewing.Dump();
    }
}
