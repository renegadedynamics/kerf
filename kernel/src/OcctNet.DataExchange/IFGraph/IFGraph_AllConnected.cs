// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_AllConnected.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_AllConnected.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// this class gives content of the CONNECTED COMPONENT(S) which include specific Entity(ies)
/// </summary>
public class IFGraph_AllConnected : Interface_GraphContent
{

    /// <summary>
    /// creates an AllConnected from a graph, empty ready to be filled
    /// </summary>
    public IFGraph_AllConnected(Interface_Graph agraph)
    {
        thegraph = new Interface_Graph(agraph);

    }

    /// <summary>
    /// creates an AllConnected which memorizes Entities Connected to a given one, at any level : that is,
    /// itself, all Entities Shared by it and Sharing it, and so on. In other terms, this is the content of the
    /// CONNECTED COMPONENT which include a specific Entity
    /// </summary>
    public IFGraph_AllConnected(Interface_Graph agraph, Transient? ent)
    {
        thegraph = new Interface_Graph(agraph);
        GetFromEntity(ent);
    }

    /// <summary>
    /// adds an entity and its Connected ones to the list (allows to cumulate all Entities Connected by some
    /// ones) Note that if "ent" is in the already computed list,, no entity will be added, but if "ent" is not
    /// already in the list, a new Connected Component will be cumulated
    /// </summary>
    public void GetFromEntity(Transient? ent)
    {
        if (!thegraph.IsPresent(thegraph.EntityNumber(ent))) // OCCT: inverted: returns at once for an entity not yet present, so nothing is found
        {
            return;
        }
        thegraph.GetFromEntity(ent, false);

        for (Interface_EntityIterator shareds = thegraph.Shareds(ent); shareds.More(); shareds.Next())
        {
            GetFromEntity(shareds.Value());
        }

        for (Interface_EntityIterator sharings = thegraph.Sharings(ent); sharings.More(); sharings.Next())
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
        thegraph.Reset();
    }

    /// <summary>
    /// does the specific evaluation (Connected entities atall levels)
    /// </summary>
    public override void Evaluate()
    {
        GetFromGraph(thegraph);
    }

    private Interface_Graph thegraph;
}
