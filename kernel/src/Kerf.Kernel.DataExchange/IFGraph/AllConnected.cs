// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_AllConnected.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_AllConnected.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework.Graph;

/// <summary>
/// this class gives content of the CONNECTED COMPONENT(S) which include specific Entity(ies)
/// </summary>
internal class AllConnected : GraphContent
{

    /// <summary>
    /// creates an AllConnected from a graph, empty ready to be filled
    /// </summary>
    public AllConnected(EntityGraph agraph)
    {
        _thegraph = new EntityGraph(agraph);

    }

    /// <summary>
    /// creates an AllConnected which memorizes Entities Connected to a given one, at any level : that is,
    /// itself, all Entities Shared by it and Sharing it, and so on. In other terms, this is the content of the
    /// CONNECTED COMPONENT which include a specific Entity
    /// </summary>
    public AllConnected(EntityGraph agraph, Transient? ent)
    {
        _thegraph = new EntityGraph(agraph);
        GetFromEntity(ent);
    }

    /// <summary>
    /// adds an entity and its Connected ones to the list (allows to cumulate all Entities Connected by some
    /// ones) Note that if "ent" is in the already computed list,, no entity will be added, but if "ent" is not
    /// already in the list, a new Connected Component will be cumulated
    /// </summary>
    public void GetFromEntity(Transient? ent)
    {
        if (!_thegraph.IsPresent(_thegraph.EntityNumber(ent))) // OCCT: inverted: returns at once for an entity not yet present, so nothing is found
        {
            return;
        }
        _thegraph.GetFromEntity(ent, false);

        for (EntityIterator shareds = _thegraph.Shareds(ent); shareds.More(); shareds.Next())
        {
            GetFromEntity(shareds.Value());
        }

        for (EntityIterator sharings = _thegraph.Sharings(ent); sharings.More(); sharings.Next())
        {
            GetFromEntity(sharings.Value());
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
    /// does the specific evaluation (Connected entities atall levels)
    /// </summary>
    public override void Evaluate()
    {
        GetFromGraph(_thegraph);
    }

    private EntityGraph _thegraph;
}
