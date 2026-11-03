// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeWire.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeWire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Topology;
using static Occt.BRepBuilderAPI_WireError;
using static Occt.BRepLib_WireError;

namespace Occt;

/// <summary>Builds a wire from edges, connected by their vertices (coincident within the vertices' tolerances); an edge is added only if it shares a vertex with the wire.</summary>
/// <remarks>
/// A wrapper of <see cref="BRepLib_MakeWire"/>. The result is BRepLib's, read without freezing it; <see cref="BRepBuilderAPI_MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// <para>OCCT.NET: where the maker had to copy frozen sub-shapes of its input to update them (decision 417), the
/// copies are what OCCT's in-place result holds, and <see cref="BRepBuilderAPI_MakeShape.Modified"/> answers them for
/// the input's sub-shapes (OCCT's Modified, which would answer nothing, has nothing to report since it changes them in
/// place).</para>
/// </remarks>
public class BRepBuilderAPI_MakeWire : BRepBuilderAPI_MakeShape
{
    private readonly BRepLib_MakeWire myMakeWire;

    // OCCT's "if (myMakeWire.IsDone()) { Done(); myShape = myMakeWire.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (myMakeWire.IsDone())
        {
            Done();
            myShape = myMakeWire.InternalShape();
        }
    }

    public BRepBuilderAPI_MakeWire()
    {
        myMakeWire = new BRepLib_MakeWire();
    }

    public BRepBuilderAPI_MakeWire(in Edge E)
    {
        myMakeWire = new BRepLib_MakeWire(E);
        TakeResult();
    }

    public BRepBuilderAPI_MakeWire(in Edge E1, in Edge E2)
    {
        myMakeWire = new BRepLib_MakeWire(E1, E2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeWire(in Edge E1, in Edge E2, in Edge E3)
    {
        myMakeWire = new BRepLib_MakeWire(E1, E2, E3);
        TakeResult();
    }

    public BRepBuilderAPI_MakeWire(in Edge E1, in Edge E2, in Edge E3, in Edge E4)
    {
        myMakeWire = new BRepLib_MakeWire(E1, E2, E3, E4);
        TakeResult();
    }

    public BRepBuilderAPI_MakeWire(in Wire W)
    {
        myMakeWire = new BRepLib_MakeWire(W);
        TakeResult();
    }

    public BRepBuilderAPI_MakeWire(in Wire W, in Edge E)
    {
        myMakeWire = new BRepLib_MakeWire(W, E);
        TakeResult();
    }

    public void Add(in Wire W)
    {
        myMakeWire.Add(W);
        TakeResult();
    }

    public void Add(in Edge E)
    {
        myMakeWire.Add(E);
        TakeResult();
    }

    public void Add(NCollection_List<Shape> L)
    {
        myMakeWire.Add(L);
        TakeResult();
    }


    /// <summary>The wire (frozen).</summary>
    public Wire Wire()
    {
        return myMakeWire.Wire();
    }

    /// <summary><see cref="Wire"/> without freezing, for ported code.</summary>
    internal Wire InternalWire()
    {
        return myMakeWire.InternalWire();
    }

    /// <summary>The last edge added (frozen with the wire).</summary>
    public Edge Edge()
    {
        return myMakeWire.Edge();
    }

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal Edge InternalEdge()
    {
        return myMakeWire.InternalEdge();
    }

    /// <summary>The last vertex connected (frozen with the wire).</summary>
    public Vertex Vertex()
    {
        return myMakeWire.Vertex();
    }

    /// <summary><see cref="Vertex"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex()
    {
        return myMakeWire.InternalVertex();
    }

    /// <summary>The wire (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Wire(BRepBuilderAPI_MakeWire theMaker) => theMaker.Wire();

    public override bool IsDone()
    {
        return myMakeWire.IsDone();
    }

    /// <summary>The construction status.</summary>
    public BRepBuilderAPI_WireError Error()
    {
        switch (myMakeWire.Error())
        {
            case BRepLib_WireDone:
                return BRepBuilderAPI_WireDone;
            case BRepLib_EmptyWire:
                return BRepBuilderAPI_EmptyWire;
            case BRepLib_DisconnectedWire:
                return BRepBuilderAPI_DisconnectedWire;
            case BRepLib_NonManifoldWire:
                return BRepBuilderAPI_NonManifoldWire;
        }

        // portage WNT
        return BRepBuilderAPI_WireDone;
    }

    // OCCT.NET: the replacements of frozen input sub-shapes by the copies the maker updated (decision 417).
    internal override NCollection_List<Shape> InternalModified(in Shape S)
    {
        myGenerated.Clear();
        History? aHistory = myMakeWire.InternalHistory();
        if (aHistory is not null)
        {
            foreach (Shape aModified in aHistory.Modified(S))
            {
                myGenerated.Append(aModified);
            }
        }
        return myGenerated;
    }
}
