// Port of: src/DataExchange/TKXSBase/Interface/Interface_HGraph.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_HGraph.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// This class allows to store a redefinable Graph, via a Handle (useful for an Object which can work on
/// several successive Models, with the same general conditions)
/// </summary>
internal class HGraph : Transient
{

    /// <summary>
    /// Creates an HGraph directly from a Graph. Remark that the starting Graph is duplicated
    /// </summary>
    public HGraph(EntityGraph agraph)
    {
        _thegraph = new EntityGraph(agraph);

    }

    /// <summary>
    /// Creates an HGraph with a Graph created from &lt;amodel&gt; and &lt;lib&gt;
    /// </summary>
    public HGraph(InterfaceModel? amodel, GeneralLib lib, bool modeStat = true)
    {
        _thegraph = new EntityGraph(amodel, lib, modeStat);

    }

    /// <summary>
    /// Creates an HGraph with a graph itself created from &lt;amodel&gt; and &lt;protocol&gt;
    /// </summary>
    public HGraph(InterfaceModel? amodel, Protocol? protocol, bool modeStat = true)
    {
        _thegraph = new EntityGraph(amodel, protocol, modeStat);

    }

    /// <summary>
    /// Creates an HGraph with a graph itself created from &lt;amodel&gt; and &lt;protocol&gt;
    /// </summary>
    public HGraph(InterfaceModel? amodel, GTool? gtool, bool modeStat = true)
    {
        _thegraph = new EntityGraph(amodel, gtool, modeStat);

    }

    /// <summary>
    /// Same a above, but works with the GTool in the model
    /// </summary>
    public HGraph(InterfaceModel? amodel, bool modeStat = true)
    {
        _thegraph = new EntityGraph(amodel, modeStat);

    }

    /// <summary>
    /// Returns the Graph contained in &lt;me&gt;, for Read Only Operations Remark that it is returns as "const
    /// &amp;" Getting it in a new variable instead of a reference would be a pity, because all the graph's
    /// content would be duplicated
    /// </summary>
    public EntityGraph Graph()
    {
        return _thegraph;
    }

    /// <summary>
    /// Same as above, but for Read-Write Operations Then, The Graph will be modified in the HGraph itself
    /// </summary>
    public EntityGraph CGraph()
    {
        return _thegraph;
    }

    private EntityGraph _thegraph;
}
