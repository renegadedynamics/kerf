// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_DispPerSignature.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_DispPerSignature.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// A DispPerSignature sorts input Entities according to a Signature : it works with a SignCounter to do
/// this.
/// </summary>
public class IFSelect_DispPerSignature : IFSelect_Dispatch
{

    /// <summary>
    /// Creates a DispPerSignature with no SignCounter (by default, produces only one packet)
    /// </summary>
    public IFSelect_DispPerSignature()
    {
    }

    /// <summary>
    /// Returns the SignCounter used for splitting
    /// </summary>
    public IFSelect_SignCounter? SignCounter()
    {
        return thesign;
    }

    /// <summary>
    /// Sets a SignCounter for sort Remark : it is set to record lists of entities, not only counts
    /// </summary>
    public void SetSignCounter(IFSelect_SignCounter? sign)
    {
        thesign = sign;
        thesign.SetList(true);
    }

    /// <summary>
    /// Returns the name of the SignCounter, which characterises the sorting criterium for this Dispatch
    /// </summary>
    public string SignName()
    {
        return (thesign is null ? "???" : thesign.Name());
    }

    /// <summary>
    /// Returns as Label, "One File per Signature &lt;name&gt;"
    /// </summary>
    public override string Label()
    {
        return Sprintf("One File per Signature %s", SignName());
    }

    /// <summary>
    /// Returns True, maximum count is given as &lt;nbent&gt;
    /// </summary>
    public override bool LimitedMax(int nbent, out int max)
    {
        max = nbent;
        return true;
    }

    /// <summary>
    /// Computes the list of produced Packets. It defines Packets from the SignCounter, which sirts the input
    /// Entities per Signature (specific of the SignCounter).
    /// </summary>
    public override void Packets(Interface_Graph G, IFGraph_SubPartsIterator packs)
    {
        if (thesign is null)
        {
            packs.AddPart();
            packs.GetFromIter(FinalSelection().RootResult(G));
            return;
        }

        thesign.Clear();
        thesign.AddList(FinalSelection().RootResult(G).Content(), G.Model());
        NCollection_HSequence<TCollection_HAsciiString> list = thesign.List();
        int i, nb, nbs = list.Length();
        TCollection_HAsciiString asign = null;
        NCollection_HSequence<Standard_Transient> ents = null;
        for (int @is = 1; @is <= nbs; @is++)
        {
            asign = list.Value(@is);
            ents = thesign.Entities(asign.ToCString());
            if (ents is null)
            {
                continue;
            }
            packs.AddPart();
            nb = ents.Length();
            for (i = 1; i <= nb; i++)
            {
                packs.GetFromEntity(ents.Value(i), false);
            }
        }
    }

    private IFSelect_SignCounter? thesign;
}
