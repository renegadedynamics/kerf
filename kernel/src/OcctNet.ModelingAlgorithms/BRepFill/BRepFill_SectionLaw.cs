// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_SectionLaw.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_SectionLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Occt;

/// <summary>A section law built on a vertex or a wire: one GeomFill section law per non-degenerated edge.</summary>
/// <remarks>
/// A working object (decisions 21 and 806). <see cref="Law"/> returns the law's own elementary laws, not copies;
/// <see cref="CurrentEdge"/> advances the law's wire explorer.
/// </remarks>
public abstract class BRepFill_SectionLaw : Transient
{
    protected NCollection_HArray1<GeomFill_SectionLaw>? myLaws;
    protected bool uclosed;
    protected bool vclosed;
    protected bool myDone;
    protected readonly NCollection_DataMap<Shape, int> myIndices = new(ShapeHasher.Instance);
    private readonly WireExplorer myIterator = new();

    //=======================================================================
    // function : NbLaw
    // purpose  : Gives the number of elementary (or Geometric) law
    //=======================================================================

    /// <summary>The number of elementary laws.</summary>
    public int NbLaw() => myLaws!.Length();

    //=================================================================================================

    /// <summary>The elementary law of rank <paramref name="Index"/>: the law's own object.</summary>
    public GeomFill_SectionLaw Law(int Index) => myLaws!.Value(Index);

    //=================================================================================================

    /// <summary>The index of <paramref name="anEdge"/> in the section (negative for a reversed edge of NSections).</summary>
    /// <exception cref="NoSuchObjectException">The edge isn't in the section.</exception>
    public int IndexOfEdge(in Shape anEdge) => myIndices.Find(anEdge);

    /// <summary>True if the law is constant.</summary>
    public abstract bool IsConstant();

    //=================================================================================================

    /// <summary>True if the sections are closed.</summary>
    public bool IsUClosed() => uclosed;

    //=================================================================================================

    /// <summary>True if the law is closed along the path (the first and last sections are the same).</summary>
    public bool IsVClosed() => vclosed;

    //=================================================================================================

    /// <summary>True if the law was built.</summary>
    public bool IsDone() => myDone;

    /// <summary>True if the input shape is a vertex.</summary>
    public abstract bool IsVertex();

    /// <summary>The law built on the concatenated section.</summary>
    public abstract GeomFill_SectionLaw? ConcatenedLaw();

    /// <summary>The continuity of the section between its edges of ranks <paramref name="Index"/> and next.</summary>
    public abstract Continuity Continuity(int Index, double TolAngular);

    /// <summary>The gap between the edges of ranks <paramref name="Index"/> and next at the law's parameter.</summary>
    public abstract double VertexTol(int Index, double Param);

    /// <summary>The section's vertex of rank <paramref name="Index"/> at the law's parameter <paramref name="Param"/>.</summary>
    public abstract Vertex Vertex(int Index, double Param);

    /// <summary>The section at the law's parameter <paramref name="U"/>.</summary>
    public abstract void D0(double U, ref Shape S);

    //=================================================================================================

    /// <summary>Starts parsing <paramref name="W"/> with <see cref="CurrentEdge"/>.</summary>
    public void Init(in Wire W)
    {
        myIterator.Init(W);
    }

    //=======================================================================
    // function :
    // purpose  : Parses the wire omitting the degenerated Edges
    //=======================================================================

    /// <summary>The next non-degenerated edge of the wire given to <see cref="Init"/>.</summary>
    public Edge CurrentEdge()
    {
        Edge E = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool B;
        bool Suivant = false;
        if (myIterator.More())
        {
            E = myIterator.Current();
            //    Next = (B.Degenerated(E));
            Suivant = (Tool.Degenerated(E));
        }

        while (Suivant)
        {
            myIterator.Next();
            E = myIterator.Current();
            //    Next = (B.Degenerated(E) && myIterator.More());
            Suivant = (Tool.Degenerated(E) && myIterator.More());
        }

        if (myIterator.More())
        {
            myIterator.Next();
        }
        return E;
    }
}
