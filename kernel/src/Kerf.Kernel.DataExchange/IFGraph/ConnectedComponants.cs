// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_ConnectedComponants.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_ConnectedComponants.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework.Graph;

/// <summary>
/// determines Connected Components in a Graph. They define disjoined sets of Entities.
/// </summary>
internal class ConnectedComponants : SubPartsIterator
{

    /// <summary>
    /// creates with a Graph, and will analyse : whole True : all the contents of the Model whole False :
    /// sub-parts which will be given later
    /// </summary>
    public ConnectedComponants(EntityGraph agraph, bool whole) : base(agraph, whole)
    {

    }

    /// <summary>
    /// does the computation
    /// </summary>
    public override void Evaluate()
    {
        //  Start from "loaded"
        //  For each: if it is noted in the graph, we pass
        //  Otherwise, add the AllConnected as sub-part
        EntityIterator loaded = Loaded();
        Reset();
        for (loaded.Start(); loaded.More(); loaded.Next())
        {
            Transient ent = loaded.Value();
            if (IsInPart(ent))
            {
                continue;
            }
            AllConnected connect = new AllConnected(new EntityGraph(Model()), ent);
            AddPart();
            GetFromIter(connect);
        }
    }
}
