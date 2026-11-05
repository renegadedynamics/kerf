// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_DispPerFiles.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_DispPerFiles.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Graph;

using Occt;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// A DispPerFiles produces a determined count of Packets from the input Entities. It divides, as equally as
/// possible, the input list into a count of files. This count is the parameter of the DispPerFiles. If the
/// input list has less than this count, of course there will be one packet per input entity. This count is a
/// Parameter of the DispPerFiles, given as an IntParam, thus allowing external control of its Value
/// </summary>
internal class DispPerFiles : Dispatch
{

    /// <summary>
    /// Creates a DispPerFiles with no Count (default value 1 file)
    /// </summary>
    public DispPerFiles()
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
            pcount = 1; // default option taken
        }
        return pcount;
    }

    /// <summary>
    /// Returns as Label, "Maximum &lt;count&gt; Files"
    /// </summary>
    public override string Label()
    {
        string lab = CountValue().ToString(System.Globalization.CultureInfo.InvariantCulture);
        lab = "Maximum " + lab;
        lab += " Files";
        return lab;
    }

    /// <summary>
    /// Returns True, maximum count is given as CountValue
    /// </summary>
    public override bool LimitedMax(int nbent, out int pcount)
    {
        pcount = CountValue();
        return true;
    }

    /// <summary>
    /// Computes the list of produced Packets. It defines Packets in order to have &lt;Count&gt; Packets, except
    /// if the input count of Entities is lower. Entities are given by RootResult from the Final Selection.
    /// </summary>
    public override void Packets(EntityGraph g, SubPartsIterator packs)
    {
        //  Resembles DispPerOne, but does "count" AddPart roots
        int pcount = CountValue();

        SCRoots roots = new SCRoots(g, false);
        roots.SetLoad();
        roots.GetFromIter(FinalSelection().UniqueResult(g));
        //   SCRoots initiated the resolution: breakdown into StrongComponents + selection
        //   of roots. A packet then corresponds to <count> roots
        //   Therefore, we need to iterate over the Parts of roots and take them by <count>
        roots.Start(); // Start performs specific Evaluate
        int nb = roots.PartCount;
        if (pcount > 0)
        {
            pcount = (nb - 1) / pcount + 1; // per packet
        }

        int i = 0;
        for (; roots.More(); roots.Next())
        { // Start already done
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
