// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_DispPerOne.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_DispPerOne.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// A DispPerOne gathers all the input Entities into as many Packets as there Root Entities from the Final
/// Selection, that is, one Packet per Entity
/// </summary>
public class IFSelect_DispPerOne : IFSelect_Dispatch
{

    /// <summary>
    /// Creates a DispPerOne
    /// </summary>
    public IFSelect_DispPerOne()
    {
    }

    /// <summary>
    /// Returns as Label, "One File per Input Entity"
    /// </summary>
    public override string Label()
    {
        return ("One File per Input Entity");
    }

    /// <summary>
    /// Returns True, maximum limit is given as &lt;nbent&gt;
    /// </summary>
    public override bool LimitedMax(int nbent, out int pcount)
    {
        pcount = nbent;
        return true;
    }

    /// <summary>
    /// Returns the list of produced Packets. It defines one Packet per Entity given by RootResult from the Final
    /// Selection.
    /// </summary>
    public override void Packets(Interface_Graph G, IFGraph_SubPartsIterator packs)
    {
        IFGraph_SCRoots packsc = new IFGraph_SCRoots(G, false); // OK for SubPartsIterator
        packsc.SetLoad();
        packsc.GetFromIter(FinalSelection().UniqueResult(G));
        //   SCRoots initiated the resolution: splitting into StrongComponents + selection
        //   of Roots. Each Root corresponds to a Packet. QED
        packs.GetParts(packsc);
    }
}
