// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_StrongComponants.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_StrongComponants.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework.Graph;

/// <summary>
/// determines strong components of a graph, that is isolated entities (single components) or loops
/// </summary>
internal class StrongComponants : SubPartsIterator
{

    /// <summary>
    /// creates with a Graph, and will analyse : whole True : all the contents of the Model whole False :
    /// sub-parts which will be given later
    /// </summary>
    /// <summary>A copy of another one (C++'s implicit copy constructor, through SubPartsIterator's).</summary>
    public StrongComponants(StrongComponants other) : base(other)
    {
    }

    public StrongComponants(EntityGraph agraph, bool whole) : base(agraph, whole)
    {

    }

    /// <summary>
    /// does the computation
    /// </summary>
    public override void Evaluate()
    {
        GraphContent iter = Loaded();
        EntityGraph g = new EntityGraph(_thegraph);
        g.GetFromIter(iter, 0);
        int nb = g.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (!g.IsPresent(i))
            {
                continue;
            }
            AddPart(); // OCCT: each entity in a part of its own: no strong components
            GetFromEntity(g.Entity(i), false);
        }
    }
}
