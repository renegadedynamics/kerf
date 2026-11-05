// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_SectionLaw.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_SectionLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Foundation;

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Filling;

/// <summary>A section law built on a vertex or a wire: one GeomFill section law per non-degenerated edge.</summary>
/// <remarks>
/// A working object (decisions 21 and 806). <see cref="Law"/> returns the law's own elementary laws, not copies;
/// <see cref="CurrentEdge"/> advances the law's wire explorer.
/// </remarks>
internal abstract class SectionLaw : Transient
{
    protected NCollection_HArray1<SurfaceSectionLaw>? _laws;
    protected bool _uclosed;
    protected bool _vclosed;
    protected bool _done;
    protected readonly Dictionary<Shape, int> _indices = new(ShapeHasher.Instance);
    private readonly WireExplorer _iterator = new();

    //=======================================================================
    // function : NbLaw
    // purpose  : Gives the number of elementary (or Geometric) law
    //=======================================================================

    /// <summary>The number of elementary laws.</summary>
    public int LawCount => _laws!.Length();

    //=================================================================================================

    /// <summary>The elementary law of rank <paramref name="index"/>: the law's own object.</summary>
    public SurfaceSectionLaw Law(int index) => _laws!.Value(index);

    //=================================================================================================

    /// <summary>The index of <paramref name="anEdge"/> in the section (negative for a reversed edge of NSections).</summary>
    /// <exception cref="NoSuchObjectException">The edge isn't in the section.</exception>
    public int IndexOfEdge(in Shape anEdge) => _indices.GetValue(anEdge);

    /// <summary>True if the law is constant.</summary>
    public abstract bool IsConstant { get; }

    //=================================================================================================

    /// <summary>True if the sections are closed.</summary>
    public bool IsUClosed => _uclosed;

    //=================================================================================================

    /// <summary>True if the law is closed along the path (the first and last sections are the same).</summary>
    public bool IsVClosed => _vclosed;

    //=================================================================================================

    /// <summary>True if the law was built.</summary>
    public bool IsDone => _done;

    /// <summary>True if the input shape is a vertex.</summary>
    public abstract bool IsVertex { get; }

    /// <summary>The law built on the concatenated section.</summary>
    public abstract SurfaceSectionLaw? ConcatenedLaw();

    /// <summary>The continuity of the section between its edges of ranks <paramref name="index"/> and next.</summary>
    public abstract Continuity Continuity(int index, double tolAngular);

    /// <summary>The gap between the edges of ranks <paramref name="index"/> and next at the law's parameter.</summary>
    public abstract double VertexTol(int index, double param);

    /// <summary>The section's vertex of rank <paramref name="index"/> at the law's parameter <paramref name="param"/>.</summary>
    public abstract Vertex Vertex(int index, double param);

    /// <summary>The section at the law's parameter <paramref name="u"/>.</summary>
    public abstract void D0(double u, ref Shape s);

    //=================================================================================================

    /// <summary>Starts parsing <paramref name="w"/> with <see cref="CurrentEdge"/>.</summary>
    public void Init(in Wire w)
    {
        _iterator.Init(w);
    }

    //=======================================================================
    // function :
    // purpose  : Parses the wire omitting the degenerated Edges
    //=======================================================================

    /// <summary>The next non-degenerated edge of the wire given to <see cref="Init"/>.</summary>
    public Edge CurrentEdge()
    {
        Edge e = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool B;
        bool suivant = false;
        if (_iterator.More())
        {
            e = _iterator.Current();
            //    Next = (B.Degenerated(E));
            suivant = (Tool.Degenerated(e));
        }

        while (suivant)
        {
            _iterator.Next();
            e = _iterator.Current();
            //    Next = (B.Degenerated(E) && myIterator.More());
            suivant = (Tool.Degenerated(e) && _iterator.More());
        }

        if (_iterator.More())
        {
            _iterator.Next();
        }
        return e;
    }
}
