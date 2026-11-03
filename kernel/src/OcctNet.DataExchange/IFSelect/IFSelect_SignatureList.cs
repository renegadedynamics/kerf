// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SignatureList.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SignatureList.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using static Occt.IFSelect_PrintCount;

namespace Occt;

/// <summary>
/// A SignatureList is given as result from a Counter (any kind) It gives access to a list of signatures,
/// with counts, and optionally with list of corresponding entities It can also be used only to give a
/// signature, through SignOnly Mode. This can be useful for a specific counter (used in a Selection), while
/// it remains better to use a Signature whenever possible
/// </summary>
public class IFSelect_SignatureList : Transient
{

    /// <summary>
    /// Creates a SignatureList. If &lt;withlist&gt; is True, entities will be not only counted per signature,
    /// but also listed.
    /// </summary>
    public IFSelect_SignatureList(bool withlist = false)
    {
        thesignonly = false;
        thelistat = withlist;
        thenbnuls = 0;
        SetName("...");
    }

    /// <summary>
    /// Changes the record-list status. The list is not cleared but its use changes
    /// </summary>
    public void SetList(bool withlist)
    {
        thelistat = withlist;
    }

    /// <summary>
    /// Returns modifiable the SignOnly Mode If False (D), the counter normally counts If True, the counting work
    /// is turned off, Add only fills the LastValue, which can be used as signature, when a counter works from
    /// data which are not available from a Signature
    /// </summary>
    public ref bool ModeSignOnly()
    {
        return ref thesignonly;
    }

    public virtual void Clear()
    {
        thelastval = "";
        thenbnuls = 0;
        thedicount.Clear();
        thediclist.Clear();
    }

    /// <summary>
    /// Adds an entity with its signature, i.e. : - counts an item more for &lt;sign&gt; - if record-list status
    /// is set, records the entity Accepts a null entity (the signature is then for the global model). But if the
    /// string is empty, counts a Null item. If SignOnly Mode is set, this work is replaced by just setting
    /// LastValue
    /// </summary>
    public void Add(Transient? ent, string sign)
    {
        if (thesignonly)
        {
            thelastval = sign;
            return;
        }

        if (sign.Length == 0)
        {
            thenbnuls++;
            return;
        }

        if (thedicount.Contains(sign))
        {
            thedicount.ChangeFromKey(sign)++;
        }
        else
        {
            thedicount.Add(sign, 1);
        }

        if (thelistat)
        {
            NCollection_HSequence<Transient> alist = null;
            if (thediclist.Contains(sign))
            {
                alist = thediclist.FindFromKey(sign);
            }
            else
            {
                alist = new NCollection_HSequence<Transient>();
                thediclist.Add(sign, alist);
            }
            alist.Append(ent);
        }
    }

    /// <summary>
    /// Returns the last value recorded by Add (only if SignMode set) Cleared by Clear or Init
    /// </summary>
    public string LastValue()
    {
        return thelastval;
    }

    /// <summary>
    /// Acknowledges the list in once. Name identifies the Signature
    /// </summary>
    public void Init(string name, NCollection_IndexedDataMap<string, int> theCount, NCollection_IndexedDataMap<string, NCollection_HSequence<Transient>> list, int nbnuls)
    {
        thelastval = "";
        thename = new TCollection_HAsciiString(name);
        thedicount = theCount;
        thediclist = list;
        thenbnuls = nbnuls;
        if (thediclist.IsEmpty())
        {
            thelistat = false;
        }
    }

    /// <summary>
    /// Returns the list of signatures, as a sequence of strings (but without their respective counts). It is
    /// ordered. By default, for all the signatures. If &lt;root&gt; is given non empty, for the signatures which
    /// begin by &lt;root&gt;
    /// </summary>
    public NCollection_HSequence<TCollection_HAsciiString>? List(string root = "")
    {
        NCollection_HSequence<TCollection_HAsciiString> list =
            new NCollection_HSequence<TCollection_HAsciiString>();
        for (int iter = 1; iter <= thedicount.Extent(); iter++)
        {
            if (!thedicount.FindKey(iter).StartsWith(root, StringComparison.Ordinal))
            {
                continue;
            }

            TCollection_HAsciiString sign = new TCollection_HAsciiString(thedicount.FindKey(iter));
            list.Append(sign);
        }
        return list;
    }

    /// <summary>
    /// Returns True if the list of Entities is acknowledged, else the method Entities will always return a Null
    /// Handle
    /// </summary>
    public bool HasEntities()
    {
        return thelistat;
    }

    /// <summary>
    /// Returns the count of null entities
    /// </summary>
    public int NbNulls()
    {
        return thenbnuls;
    }

    /// <summary>
    /// Returns the number of times a signature was counted, 0 if it has not been recorded at all
    /// </summary>
    public int NbTimes(string sign)
    {
        thedicount.FindFromKey(sign, out int nb);
        return nb;
    }

    /// <summary>
    /// Returns the list of entities attached to a signature It is empty if &lt;sign&gt; has not been recorded It
    /// is a Null Handle if the list of entities is not known
    /// </summary>
    public NCollection_HSequence<Transient>? Entities(string sign)
    {
        NCollection_HSequence<Transient> list = null;
        if (!thelistat)
        {
            return list;
        }
        if (thediclist.FindFromKey(sign, out NCollection_HSequence<Transient> aTList))
        {
            list = aTList;
        }
        else
        {
            list = new NCollection_HSequence<Transient>();
        }
        return list;
    }

    /// <summary>
    /// Defines a name for a SignatureList (used to print it)
    /// </summary>
    public void SetName(string name)
    {
        thename = new TCollection_HAsciiString(name);
    }

    /// <summary>
    /// Returns the recorded Name. Remark : default is "..." (no SetName called)
    /// </summary>
    public virtual string Name()
    {
        return thename.ToCString();
    }

    /// <summary>
    /// Prints the counts of items (not the list)
    /// </summary>
    public virtual void PrintCount(TextWriter S)
    {
        int nbtot = 0, nbsign = 0;
        S.Write(" Count\t");
        S.Write(thename);
        S.Write("\n -----\t-----------");
        S.Write('\n');
        for (int iter = 1; iter <= thedicount.Extent(); iter++)
        {
            int val = thedicount.FindFromIndex(iter);
            S.Write(Interface_MSG.Blanks(val, 6));
            S.Write(val);
            S.Write("\t");
            S.Write(thedicount.FindKey(iter));
            S.Write('\n');
            nbtot += val;
            nbsign++;
        }
        if (thenbnuls > 0)
        {
            S.Write(thename);
            S.Write(" Nul : ");
            S.Write(thenbnuls);
            S.Write('\n');
        }
        S.Write("    Nb Total:");
        S.Write(nbtot);
        S.Write("  for ");
        S.Write(nbsign);
        S.Write(" items");
        S.Write('\n');
    }

    /// <summary>
    /// Prints the lists of items, if they are present (else, prints a message "no list available") Uses
    /// &lt;model&gt; to determine for each entity to be listed, its number, and its specific identifier (by
    /// PrintLabel) &lt;mod&gt; gives a mode for printing : - CountByItem : just count (as PrintCount) -
    /// ShortByItem : minimum i.e. count plus 5 first entity numbers - ShortByItem(D) complete list of entity
    /// numbers (0: "Global") - EntitiesByItem : list of (entity number/PrintLabel from the model) other modes
    /// are ignored
    /// </summary>
    public virtual void PrintList(TextWriter S, Interface_InterfaceModel? model, IFSelect_PrintCount mod = IFSelect_ListByItem)
    {
        if (mod == IFSelect_ItemsByEntity)
        {
            return;
        }
        if (mod == IFSelect_CountByItem)
        {
            PrintCount(S);
            return;
        }
        if (mod == IFSelect_CountSummary)
        {
            PrintSum(S);
            return;
        }
        if (!HasEntities())
        {
            S.Write(" SignatureList ");
            S.Write(Name());
            S.Write(" : PrintList, list not available");
            S.Write('\n');
            PrintCount(S);
            return;
        }
        int nbtot = 0, nbsign = 0;
        for (int iter = 1; iter <= thediclist.Extent(); iter++)
        {
            NCollection_HSequence<Transient> list = thediclist.FindFromIndex(iter);
            S.Write(Name());
            S.Write(" : ");
            S.Write(thediclist.FindKey(iter));
            S.Write('\n');
            if (list is null)
            {
                S.Write("  - (empty list)");
                S.Write('\n');
                continue;
            }
            int nb = list.Length();
            S.Write("  - Nb: ");
            S.Write(nb);
            S.Write(" : ");
            int nc = nb;
            if (nb > 5 && mod == IFSelect_ShortByItem)
            {
                nc = 5;
            }
            for (int i = 1; i <= nc; i++)
            {
                if (list.Value(i) is null)
                {
                    S.Write("  0");
                    if (mod == IFSelect_EntitiesByItem)
                    {
                        S.Write(":(Global)");
                    }
                    continue;
                }
                int num = model.Number(list.Value(i));
                if (num == (int)IFSelect_ShortByItem) // OCCT: compares an entity number with the enumeration's value (2)
                {
                    S.Write("  ??");
                    continue;
                }
                S.Write("  ");
                S.Write(num);
                if (mod == IFSelect_EntitiesByItem)
                {
                    S.Write(":");
                    model.PrintLabel(list.Value(i), S);
                }
            }
            if (nc < nb)
            {
                S.Write("  .. etc");
            }
            S.Write('\n');
            nbtot += nb;
            nbsign++;
        }
        S.Write(" Nb Total:");
        S.Write(nbtot);
        S.Write("  for ");
        S.Write(nbsign);
        S.Write(" items");
        S.Write('\n');
    }

    /// <summary>
    /// Prints a summary Item which has the greatest count of entities For items which are numeric values : their
    /// count, maximum, minimum values, cumul, average
    /// </summary>
    public virtual void PrintSum(TextWriter S)
    {
        S.Write(" Summary ");
        S.Write(thename);
        S.Write("\n -----\t-----------");
        S.Write('\n');
        int nbtot = 0, nbsign = 0, maxent = 0, nbval = 0, nbve = 0, minval = 0, maxval = 0, totval = 0;
        for (int iter = 1; iter <= thedicount.Extent(); iter++)
        {
            int nbent = thedicount.FindFromIndex(iter);
            nbtot += nbent;
            nbsign++;
            if (nbent > maxent)
            {
                maxent = nbent;
            }
            string name = thedicount.FindKey(iter);
            //    if (!name.IsIntegerValue()) continue;  not very reliable
            int ic, nc = name.Length;
            bool iaint = true;
            for (ic = 1; ic <= nc; ic++)
            {
                char unc = name[ic - 1];
                if (ic == 1 && (unc == ' ' || unc == '+' || unc == '-'))
                {
                    continue;
                }
                if (unc >= '0' && unc <= '9')
                {
                    continue;
                }
                iaint = false;
                break;
            }
            if (!iaint)
            {
                continue;
            }
            int val = TCollection_AsciiString.IntegerValue(name);
            if (nbval == 0)
            {
                minval = maxval = val;
            }
            if (minval > val)
            {
                minval = val;
            }
            if (maxval < val)
            {
                maxval = val;
            }
            nbval++;
            nbve += nbent;
            totval += (val * nbent);
        }
        S.Write("    Nb Total:");
        S.Write(nbtot);
        S.Write("  for ");
        S.Write(nbsign);
        S.Write(" items");
        S.Write('\n');
        S.Write("    Highest count of entities : ");
        S.Write(maxent);
        S.Write(" on one item");
        S.Write('\n');
        if (nbval > 0)
        {
            S.Write("    Summary on Integer Values");
            S.Write('\n');
            S.Write("    Nb Integer Items : ");
            S.Write(nbval);
            S.Write('\n');
            S.Write("    For Nb Entities  : ");
            S.Write(nbve);
            S.Write('\n');
            S.Write("    Cumulated Values : ");
            S.Write(totval);
            S.Write('\n');
            S.Write("    Maximum Value    : ");
            S.Write(maxval);
            S.Write('\n');
            int avg1, avg2;
            avg1 = totval / nbve;
            avg2 = ((totval - (avg1 * nbve)) * 10) / nbve;
            S.Write("    Average Value    : ");
            S.Write(avg1);
            S.Write(" ");
            S.Write(avg2);
            S.Write("/10");
            S.Write('\n');
            S.Write("    Minimum Value    : ");
            S.Write(minval);
            S.Write('\n');
        }
    }

    private bool thesignonly;
    private bool thelistat;
    private int thenbnuls;
    private TCollection_HAsciiString? thename;
    private string thelastval = "";
    private NCollection_IndexedDataMap<string, int> thedicount = new NCollection_IndexedDataMap<string, int>();
    private NCollection_IndexedDataMap<string, NCollection_HSequence<Transient>> thediclist = new NCollection_IndexedDataMap<string, NCollection_HSequence<Transient>>();
}
