// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_SCRoots.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_SCRoots.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Graph;

/// <summary>
/// determines strong components in a graph which are Roots
/// </summary>
public class SCRoots : StrongComponants
{

    /// <summary>
    /// creates with a Graph, and will analyse : whole True : all the contents of the Model whole False :
    /// sub-parts which will be given later
    /// </summary>
    public SCRoots(EntityGraph agraph, bool whole) : base(agraph, whole)
    {

    }

    /// <summary>
    /// creates from a StrongComponants which was already computed
    /// </summary>
    public SCRoots(StrongComponants subparts) : base(subparts)
    {

    }

    /// <summary>
    /// does the computation
    /// </summary>
    public override void Evaluate()
    {
        StrongComponants complist = new StrongComponants(new EntityGraph(Model()), false);
        complist.GetFromIter(Loaded());
        //  Interface_Graph G(Model());
        EntityGraph g = new EntityGraph(_thegraph);
        // OCCT_DEBUG block left out
        g.ResetStatus();
        for (complist.Start(); complist.More(); complist.Next())
        {
            Transient ent = complist.FirstEntity();
            int num = g.EntityNumber(ent);
        // OCCT_DEBUG block left out
            if (!g.IsPresent(num))
            { // register for following
                g.GetFromEntity(ent, true);
                EntityIterator list = complist.Entities();
                AddPart();
                GetFromIter(list);
            }
        }
    }
}
