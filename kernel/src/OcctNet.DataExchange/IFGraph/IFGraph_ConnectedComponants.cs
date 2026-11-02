// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_ConnectedComponants.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_ConnectedComponants.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// determines Connected Components in a Graph. They define disjoined sets of Entities.
/// </summary>
public class IFGraph_ConnectedComponants : IFGraph_SubPartsIterator
{

    /// <summary>
    /// creates with a Graph, and will analyse : whole True : all the contents of the Model whole False :
    /// sub-parts which will be given later
    /// </summary>
    public IFGraph_ConnectedComponants(Interface_Graph agraph, bool whole) : base(agraph, whole)
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
        Interface_EntityIterator loaded = Loaded();
        Reset();
        for (loaded.Start(); loaded.More(); loaded.Next())
        {
            Standard_Transient ent = loaded.Value();
            if (IsInPart(ent))
            {
                continue;
            }
            IFGraph_AllConnected connect = new IFGraph_AllConnected(new Interface_Graph(Model()), ent);
            AddPart();
            GetFromIter(connect);
        }
    }
}
