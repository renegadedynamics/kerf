// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeWire.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeWire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Construction.Api.WireError;
using static Kerf.Engine.Construction.Library.LibraryWireError;

namespace Kerf.Engine.Construction.Api;

/// <summary>Builds a wire from edges, connected by their vertices (coincident within the vertices' tolerances); an edge is added only if it shares a vertex with the wire.</summary>
/// <remarks>
/// A wrapper of <see cref="LibraryMakeWire"/>. The result is BRepLib's, read without freezing it; <see cref="MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// <para>Kerf: where the maker had to copy frozen sub-shapes of its input to update them (decision 417), the
/// copies are what OCCT's in-place result holds, and <see cref="MakeShape.Modified"/> answers them for
/// the input's sub-shapes (OCCT's Modified, which would answer nothing, has nothing to report since it changes them in
/// place).</para>
/// </remarks>
internal class MakeWire : MakeShape
{
    private readonly LibraryMakeWire _makeWire;

    // OCCT's "if (myMakeWire.IsDone()) { Done(); myShape = myMakeWire.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (_makeWire.IsDone)
        {
            Done();
            _shape = _makeWire.InternalShape();
        }
    }

    public MakeWire()
    {
        _makeWire = new LibraryMakeWire();
    }

    public MakeWire(in Edge e)
    {
        _makeWire = new LibraryMakeWire(e);
        TakeResult();
    }

    public MakeWire(in Edge e1, in Edge e2)
    {
        _makeWire = new LibraryMakeWire(e1, e2);
        TakeResult();
    }

    public MakeWire(in Edge e1, in Edge e2, in Edge e3)
    {
        _makeWire = new LibraryMakeWire(e1, e2, e3);
        TakeResult();
    }

    public MakeWire(in Edge e1, in Edge e2, in Edge e3, in Edge e4)
    {
        _makeWire = new LibraryMakeWire(e1, e2, e3, e4);
        TakeResult();
    }

    public MakeWire(in Wire w)
    {
        _makeWire = new LibraryMakeWire(w);
        TakeResult();
    }

    public MakeWire(in Wire w, in Edge e)
    {
        _makeWire = new LibraryMakeWire(w, e);
        TakeResult();
    }

    public void Add(in Wire w)
    {
        _makeWire.Add(w);
        TakeResult();
    }

    public void Add(in Edge e)
    {
        _makeWire.Add(e);
        TakeResult();
    }

    public void Add(NCollection_List<Shape> l)
    {
        _makeWire.Add(l);
        TakeResult();
    }


    /// <summary>The wire (frozen).</summary>
    public Wire Wire()
    {
        return _makeWire.Wire();
    }

    /// <summary><see cref="Wire"/> without freezing, for ported code.</summary>
    internal Wire InternalWire()
    {
        return _makeWire.InternalWire();
    }

    /// <summary>The last edge added (frozen with the wire).</summary>
    public Edge Edge()
    {
        return _makeWire.Edge();
    }

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal Edge InternalEdge()
    {
        return _makeWire.InternalEdge();
    }

    /// <summary>The last vertex connected (frozen with the wire).</summary>
    public Vertex Vertex()
    {
        return _makeWire.Vertex();
    }

    /// <summary><see cref="Vertex"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex()
    {
        return _makeWire.InternalVertex();
    }

    /// <summary>The wire (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Wire(MakeWire maker) => maker.Wire();

    public override bool IsDone
    {
        get
        {
            return _makeWire.IsDone;
        }
    }

    /// <summary>The construction status.</summary>
    public WireError Error()
    {
        switch (_makeWire.Error())
        {
            case LibraryWireError.WireDone:
                return WireError.WireDone;
            case LibraryWireError.EmptyWire:
                return WireError.EmptyWire;
            case LibraryWireError.DisconnectedWire:
                return WireError.DisconnectedWire;
            case LibraryWireError.NonManifoldWire:
                return WireError.NonManifoldWire;
        }

        // portage WNT
        return WireError.WireDone;
    }

    // Kerf: the replacements of frozen input sub-shapes by the copies the maker updated (decision 417).
    internal override NCollection_List<Shape> InternalModified(in Shape s)
    {
        _generated.Clear();
        History? history = _makeWire.InternalHistory();
        if (history is not null)
        {
            foreach (Shape modified in history.Modified(s))
            {
                _generated.Append(modified);
            }
        }
        return _generated;
    }
}
