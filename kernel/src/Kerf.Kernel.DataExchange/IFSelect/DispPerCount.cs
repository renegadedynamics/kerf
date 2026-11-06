// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_DispPerCount.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_DispPerCount.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Graph;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// A DispPerCount gathers all the input Entities into one or several Packets, each containing a defined
/// count of Entity This count is a Parameter of the DispPerCount, given as an IntParam, thus allowing
/// external control of its Value
/// </summary>
internal class DispPerCount : Dispatch
{

    /// <summary>
    /// Creates a DispPerCount with no Count (default value 1)
    /// </summary>
    public DispPerCount()
    {
    }

    /// <summary>
    /// Returns the Count Parameter used for splitting
    /// </summary>
    public IntParam? Count()
    {
        return _thecount;
    }

    /// <summary>
    /// Sets a new Parameter for Count
    /// </summary>
    public void SetCount(IntParam? pcount)
    {
        _thecount = pcount;
    }

    /// <summary>
    /// Returns the effective value of the count parameter (if Count Parameter not Set or value not positive,
    /// returns 1)
    /// </summary>
    public int CountValue()
    {
        int pcount = 0;
        if (_thecount is not null)
        {
            pcount = _thecount.Value();
        }
        if (pcount <= 0)
        {
            pcount = 1; // option prise par defaut
        }
        return pcount;
    }

    /// <summary>
    /// Returns as Label, "One File per &lt;count&gt; Input Entities"
    /// </summary>
    public override string Label()
    {
        string lab = CountValue().ToString(System.Globalization.CultureInfo.InvariantCulture);
        lab = "One File per " + lab;
        lab += " Entities";
        return lab;
    }

    /// <summary>
    /// Returns True, maximum count is given as &lt;nbent&gt;
    /// </summary>
    public override bool LimitedMax(int nbent, out int pcount)
    {
        pcount = 1 + nbent / CountValue();
        return true;
    }

    /// <summary>
    /// Computes the list of produced Packets. It defines Packets in order to have at most &lt;Count&gt; Entities
    /// per Packet, Entities are given by RootResult from the Final Selection.
    /// </summary>
    public override void Packets(EntityGraph g, SubPartsIterator packs)
    {
        //  Ressemble a DispPerOne, mais fait un AddPart tous les "count" racines
        int pcount = CountValue();

        SCRoots roots = new SCRoots(g, false);
        roots.SetLoad();
        roots.GetFromIter(FinalSelection().UniqueResult(g));
        //   SCRoots initiated the resolution: splitting into StrongComponents + selection
        //   of roots. A packet then corresponds to <count> roots
        //   Therefore, we must iterate on the Parts of roots and take them by <count>

        int i = 0;
        for (roots.Start(); roots.More(); roots.Next())
        {
            if (i == 0)
            {
                packs.AddPart();
            }
            i++;
            if (i >= pcount)
            {
                i = 0; // regroupement selon "count"
            }
            packs.GetFromIter(roots.Entities());
        }
    }

    private IntParam? _thecount;
}
