// Port of: src/DataExchange/TKXSBase/Interface/Interface_HGraph.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_HGraph.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// This class allows to store a redefinable Graph, via a Handle (useful for an Object which can work on
/// several successive Models, with the same general conditions)
/// </summary>
public class Interface_HGraph : Transient
{

    /// <summary>
    /// Creates an HGraph directly from a Graph. Remark that the starting Graph is duplicated
    /// </summary>
    public Interface_HGraph(Interface_Graph agraph)
    {
        thegraph = new Interface_Graph(agraph);

    }

    /// <summary>
    /// Creates an HGraph with a Graph created from &lt;amodel&gt; and &lt;lib&gt;
    /// </summary>
    public Interface_HGraph(Interface_InterfaceModel? amodel, Interface_GeneralLib lib, bool theModeStat = true)
    {
        thegraph = new Interface_Graph(amodel, lib, theModeStat);

    }

    /// <summary>
    /// Creates an HGraph with a graph itself created from &lt;amodel&gt; and &lt;protocol&gt;
    /// </summary>
    public Interface_HGraph(Interface_InterfaceModel? amodel, Interface_Protocol? protocol, bool theModeStat = true)
    {
        thegraph = new Interface_Graph(amodel, protocol, theModeStat);

    }

    /// <summary>
    /// Creates an HGraph with a graph itself created from &lt;amodel&gt; and &lt;protocol&gt;
    /// </summary>
    public Interface_HGraph(Interface_InterfaceModel? amodel, Interface_GTool? gtool, bool theModeStat = true)
    {
        thegraph = new Interface_Graph(amodel, gtool, theModeStat);

    }

    /// <summary>
    /// Same a above, but works with the GTool in the model
    /// </summary>
    public Interface_HGraph(Interface_InterfaceModel? amodel, bool theModeStat = true)
    {
        thegraph = new Interface_Graph(amodel, theModeStat);

    }

    /// <summary>
    /// Returns the Graph contained in &lt;me&gt;, for Read Only Operations Remark that it is returns as "const
    /// &amp;" Getting it in a new variable instead of a reference would be a pity, because all the graph's
    /// content would be duplicated
    /// </summary>
    public Interface_Graph Graph()
    {
        return thegraph;
    }

    /// <summary>
    /// Same as above, but for Read-Write Operations Then, The Graph will be modified in the HGraph itself
    /// </summary>
    public Interface_Graph CGraph()
    {
        return thegraph;
    }

    private Interface_Graph thegraph;
}
