// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_AllShared.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_AllShared.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Graph;

/// <summary>
/// this class determines all Entities shared by some specific ones, at any level (those which will be lead
/// in a Transfer for instance)
/// </summary>
public class AllShared : GraphContent
{

    /// <summary>
    /// creates an AllShared from a graph, empty ready to be filled
    /// </summary>
    public AllShared(EntityGraph agraph)
    {
        _thegraph = new EntityGraph(agraph);

    }

    /// <summary>
    /// creates an AllShared which memrizes Entities shared by a given one, at any level, including itself
    /// </summary>
    public AllShared(EntityGraph agraph, Transient? ent)
    {
        _thegraph = new EntityGraph(agraph);
        if (!agraph.Model().Contains(ent))
        {
            return;
        }
        GetFromEntity(ent);
    }

    /// <summary>
    /// adds an entity and its shared ones to the list (allows to cumulate all Entities shared by some ones)
    /// </summary>
    public void GetFromEntity(Transient? ent)
    {
        _thegraph.GetFromEntity(ent, true);
    }

    /// <summary>
    /// Adds Entities from an EntityIterator and all their shared ones at any level
    /// </summary>
    public void GetFromIter(EntityIterator iter)
    {
        for (iter.Start(); iter.More(); iter.Next())
        {
            _thegraph.GetFromEntity(iter.Value(), true);
        }
    }

    /// <summary>
    /// Allows to restart on a new data set
    /// </summary>
    public void ResetData()
    {
        Reset();
        _thegraph.Reset();
    }

    /// <summary>
    /// does the specific evaluation (shared entities atall levels)
    /// </summary>
    public override void Evaluate()
    {
        Reset();
        GetFromGraph(_thegraph);
    }

    private EntityGraph _thegraph;
}
