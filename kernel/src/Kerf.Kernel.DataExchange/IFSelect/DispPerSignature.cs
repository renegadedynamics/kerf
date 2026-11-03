// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_DispPerSignature.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_DispPerSignature.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Graph;
using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// A DispPerSignature sorts input Entities according to a Signature : it works with a SignCounter to do
/// this.
/// </summary>
public class DispPerSignature : Dispatch
{

    /// <summary>
    /// Creates a DispPerSignature with no SignCounter (by default, produces only one packet)
    /// </summary>
    public DispPerSignature()
    {
    }

    /// <summary>
    /// Returns the SignCounter used for splitting
    /// </summary>
    public SignCounter? SignCounter()
    {
        return _thesign;
    }

    /// <summary>
    /// Sets a SignCounter for sort Remark : it is set to record lists of entities, not only counts
    /// </summary>
    public void SetSignCounter(SignCounter? sign)
    {
        _thesign = sign;
        _thesign.SetList(true);
    }

    /// <summary>
    /// Returns the name of the SignCounter, which characterises the sorting criterium for this Dispatch
    /// </summary>
    public string SignName()
    {
        return (_thesign is null ? "???" : _thesign.Name());
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
    public override void Packets(EntityGraph g, SubPartsIterator packs)
    {
        if (_thesign is null)
        {
            packs.AddPart();
            packs.GetFromIter(FinalSelection().RootResult(g));
            return;
        }

        _thesign.Clear();
        _thesign.AddList(FinalSelection().RootResult(g).Content(), g.Model());
        NCollection_HSequence<TCollection_HAsciiString> list = _thesign.List();
        int i, nb, nbs = list.Length();
        TCollection_HAsciiString asign = null;
        NCollection_HSequence<Transient> ents = null;
        for (int @is = 1; @is <= nbs; @is++)
        {
            asign = list.Value(@is);
            ents = _thesign.Entities(asign.ToCString());
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

    private SignCounter? _thesign;
}
