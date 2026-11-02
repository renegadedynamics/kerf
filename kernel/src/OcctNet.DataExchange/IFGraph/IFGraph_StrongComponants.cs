// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_StrongComponants.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_StrongComponants.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// determines strong components of a graph, that is isolated entities (single components) or loops
/// </summary>
public class IFGraph_StrongComponants : IFGraph_SubPartsIterator
{

    /// <summary>
    /// creates with a Graph, and will analyse : whole True : all the contents of the Model whole False :
    /// sub-parts which will be given later
    /// </summary>
    /// <summary>A copy of another one (C++'s implicit copy constructor, through SubPartsIterator's).</summary>
    public IFGraph_StrongComponants(IFGraph_StrongComponants other) : base(other)
    {
    }

    public IFGraph_StrongComponants(Interface_Graph agraph, bool whole) : base(agraph, whole)
    {

    }

    /// <summary>
    /// does the computation
    /// </summary>
    public override void Evaluate()
    {
        Interface_GraphContent iter = Loaded();
        Interface_Graph G = new Interface_Graph(thegraph);
        G.GetFromIter(iter, 0);
        int nb = G.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (!G.IsPresent(i))
            {
                continue;
            }
            AddPart(); // OCCT: each entity in a part of its own: no strong components
            GetFromEntity(G.Entity(i), false);
        }
    }
}
