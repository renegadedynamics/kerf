// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_Cycles.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_Cycles.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// determines strong components in a graph which are Cycles
/// </summary>
public class IFGraph_Cycles : IFGraph_SubPartsIterator
{

    /// <summary>
    /// creates with a Graph, and will analyse : whole True : all the contents of the Model whole False :
    /// sub-parts which will be given later
    /// </summary>
    public IFGraph_Cycles(Interface_Graph agraph, bool whole) : base(agraph, whole)
    {

    }

    /// <summary>
    /// creates from a StrongComponants which was already computed
    /// </summary>
    public IFGraph_Cycles(IFGraph_StrongComponants subparts) : base(subparts)
    {

    }

    /// <summary>
    /// does the computation. Cycles are StrongComponants which are not Single
    /// </summary>
    public override void Evaluate()
    {
        IFGraph_StrongComponants complist = new IFGraph_StrongComponants(new Interface_Graph(Model()), false);
        complist.GetFromIter(Loaded());
        for (complist.Start(); complist.More(); complist.Next())
        {
            if (complist.IsSingle())
            {
                continue;
            }
            AddPart();
            GetFromIter(complist.Entities());
        }
    }
}
