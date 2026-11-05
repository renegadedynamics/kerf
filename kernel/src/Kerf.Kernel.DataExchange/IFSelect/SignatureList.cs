// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SignatureList.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SignatureList.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Exchange.Framework.Selection.PrintCount;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// A SignatureList is given as result from a Counter (any kind) It gives access to a list of signatures,
/// with counts, and optionally with list of corresponding entities It can also be used only to give a
/// signature, through SignOnly Mode. This can be useful for a specific counter (used in a Selection), while
/// it remains better to use a Signature whenever possible
/// </summary>
internal class SignatureList : Transient
{

    /// <summary>
    /// Creates a SignatureList. If &lt;withlist&gt; is True, entities will be not only counted per signature,
    /// but also listed.
    /// </summary>
    public SignatureList(bool withlist = false)
    {
        _thesignonly = false;
        _thelistat = withlist;
        _thenbnuls = 0;
        SetName("...");
    }

    /// <summary>
    /// Changes the record-list status. The list is not cleared but its use changes
    /// </summary>
    public void SetList(bool withlist)
    {
        _thelistat = withlist;
    }

    /// <summary>
    /// Returns modifiable the SignOnly Mode If False (D), the counter normally counts If True, the counting work
    /// is turned off, Add only fills the LastValue, which can be used as signature, when a counter works from
    /// data which are not available from a Signature
    /// </summary>
    public ref bool ModeSignOnly()
    {
        return ref _thesignonly;
    }

    public virtual void Clear()
    {
        _thelastval = "";
        _thenbnuls = 0;
        _thedicount.Clear();
        _thediclist.Clear();
    }

    /// <summary>
    /// Adds an entity with its signature, i.e. : - counts an item more for &lt;sign&gt; - if record-list status
    /// is set, records the entity Accepts a null entity (the signature is then for the global model). But if the
    /// string is empty, counts a Null item. If SignOnly Mode is set, this work is replaced by just setting
    /// LastValue
    /// </summary>
    public void Add(Transient? ent, string sign)
    {
        if (_thesignonly)
        {
            _thelastval = sign;
            return;
        }

        if (sign.Length == 0)
        {
            _thenbnuls++;
            return;
        }

        if (_thedicount.ContainsKey(sign))
        {
            _thedicount.GetValueRef(sign)++;
        }
        else
        {
            _thedicount.Add(sign, 1);
        }

        if (_thelistat)
        {
            NCollection_HSequence<Transient> alist = null;
            if (_thediclist.ContainsKey(sign))
            {
                alist = _thediclist.GetValue(sign);
            }
            else
            {
                alist = new NCollection_HSequence<Transient>();
                _thediclist.Add(sign, alist);
            }
            alist.Append(ent);
        }
    }

    /// <summary>
    /// Returns the last value recorded by Add (only if SignMode set) Cleared by Clear or Init
    /// </summary>
    public string LastValue()
    {
        return _thelastval;
    }

    /// <summary>
    /// Acknowledges the list in once. Name identifies the Signature
    /// </summary>
    public void Init(string name, IndexedMap<string, int> count, IndexedMap<string, NCollection_HSequence<Transient>> list, int nbnuls)
    {
        _thelastval = "";
        _thename = new TCollection_HAsciiString(name);
        _thedicount = count;
        _thediclist = list;
        _thenbnuls = nbnuls;
        if (_thediclist.Count == 0)
        {
            _thelistat = false;
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
        for (int iter = 0; iter < _thedicount.Count; iter++)
        {
            if (!_thedicount.KeyAt(iter).StartsWith(root, StringComparison.Ordinal))
            {
                continue;
            }

            TCollection_HAsciiString sign = new TCollection_HAsciiString(_thedicount.KeyAt(iter));
            list.Append(sign);
        }
        return list;
    }

    /// <summary>
    /// Returns True if the list of Entities is acknowledged, else the method Entities will always return a Null
    /// Handle
    /// </summary>
    public bool HasEntities
    {
        get
        {
            return _thelistat;
        }
    }

    /// <summary>
    /// Returns the count of null entities
    /// </summary>
    public int NullCount
    {
        get
        {
            return _thenbnuls;
        }
    }

    /// <summary>
    /// Returns the number of times a signature was counted, 0 if it has not been recorded at all
    /// </summary>
    public int NbTimes(string sign)
    {
        _thedicount.TryGetValue(sign, out int nb);
        return nb;
    }

    /// <summary>
    /// Returns the list of entities attached to a signature It is empty if &lt;sign&gt; has not been recorded It
    /// is a Null Handle if the list of entities is not known
    /// </summary>
    public NCollection_HSequence<Transient>? Entities(string sign)
    {
        NCollection_HSequence<Transient> list = null;
        if (!_thelistat)
        {
            return list;
        }
        if (_thediclist.TryGetValue(sign, out NCollection_HSequence<Transient> tList))
        {
            list = tList;
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
        _thename = new TCollection_HAsciiString(name);
    }

    /// <summary>
    /// Returns the recorded Name. Remark : default is "..." (no SetName called)
    /// </summary>
    public virtual string Name()
    {
        return _thename.ToCString();
    }

    /// <summary>
    /// Prints the counts of items (not the list)
    /// </summary>
    public virtual void PrintCount(TextWriter s)
    {
        int nbtot = 0, nbsign = 0;
        s.Write(" Count\t");
        s.Write(_thename);
        s.Write("\n -----\t-----------");
        s.Write('\n');
        for (int iter = 0; iter < _thedicount.Count; iter++)
        {
            int val = _thedicount.ValueAt(iter);
            s.Write(FrameworkMsg.Blanks(val, 6));
            s.Write(val);
            s.Write("\t");
            s.Write(_thedicount.KeyAt(iter));
            s.Write('\n');
            nbtot += val;
            nbsign++;
        }
        if (_thenbnuls > 0)
        {
            s.Write(_thename);
            s.Write(" Nul : ");
            s.Write(_thenbnuls);
            s.Write('\n');
        }
        s.Write("    Nb Total:");
        s.Write(nbtot);
        s.Write("  for ");
        s.Write(nbsign);
        s.Write(" items");
        s.Write('\n');
    }

    /// <summary>
    /// Prints the lists of items, if they are present (else, prints a message "no list available") Uses
    /// &lt;model&gt; to determine for each entity to be listed, its number, and its specific identifier (by
    /// PrintLabel) &lt;mod&gt; gives a mode for printing : - CountByItem : just count (as PrintCount) -
    /// ShortByItem : minimum i.e. count plus 5 first entity numbers - ShortByItem(D) complete list of entity
    /// numbers (0: "Global") - EntitiesByItem : list of (entity number/PrintLabel from the model) other modes
    /// are ignored
    /// </summary>
    public virtual void PrintList(TextWriter s, InterfaceModel? model, PrintCount mod = global::Kerf.Engine.Exchange.Framework.Selection.PrintCount.ListByItem)
    {
        if (mod == global::Kerf.Engine.Exchange.Framework.Selection.PrintCount.ItemsByEntity)
        {
            return;
        }
        if (mod == global::Kerf.Engine.Exchange.Framework.Selection.PrintCount.CountByItem)
        {
            PrintCount(s);
            return;
        }
        if (mod == global::Kerf.Engine.Exchange.Framework.Selection.PrintCount.CountSummary)
        {
            PrintSum(s);
            return;
        }
        if (!HasEntities)
        {
            s.Write(" SignatureList ");
            s.Write(Name());
            s.Write(" : PrintList, list not available");
            s.Write('\n');
            PrintCount(s);
            return;
        }
        int nbtot = 0, nbsign = 0;
        for (int iter = 0; iter < _thediclist.Count; iter++)
        {
            NCollection_HSequence<Transient> list = _thediclist.ValueAt(iter);
            s.Write(Name());
            s.Write(" : ");
            s.Write(_thediclist.KeyAt(iter));
            s.Write('\n');
            if (list is null)
            {
                s.Write("  - (empty list)");
                s.Write('\n');
                continue;
            }
            int nb = list.Length();
            s.Write("  - Nb: ");
            s.Write(nb);
            s.Write(" : ");
            int nc = nb;
            if (nb > 5 && mod == global::Kerf.Engine.Exchange.Framework.Selection.PrintCount.ShortByItem)
            {
                nc = 5;
            }
            for (int i = 1; i <= nc; i++)
            {
                if (list.Value(i) is null)
                {
                    s.Write("  0");
                    if (mod == global::Kerf.Engine.Exchange.Framework.Selection.PrintCount.EntitiesByItem)
                    {
                        s.Write(":(Global)");
                    }
                    continue;
                }
                int num = model.Number(list.Value(i));
                if (num == (int)global::Kerf.Engine.Exchange.Framework.Selection.PrintCount.ShortByItem) // OCCT: compares an entity number with the enumeration's value (2)
                {
                    s.Write("  ??");
                    continue;
                }
                s.Write("  ");
                s.Write(num);
                if (mod == global::Kerf.Engine.Exchange.Framework.Selection.PrintCount.EntitiesByItem)
                {
                    s.Write(":");
                    model.PrintLabel(list.Value(i), s);
                }
            }
            if (nc < nb)
            {
                s.Write("  .. etc");
            }
            s.Write('\n');
            nbtot += nb;
            nbsign++;
        }
        s.Write(" Nb Total:");
        s.Write(nbtot);
        s.Write("  for ");
        s.Write(nbsign);
        s.Write(" items");
        s.Write('\n');
    }

    /// <summary>
    /// Prints a summary Item which has the greatest count of entities For items which are numeric values : their
    /// count, maximum, minimum values, cumul, average
    /// </summary>
    public virtual void PrintSum(TextWriter s)
    {
        s.Write(" Summary ");
        s.Write(_thename);
        s.Write("\n -----\t-----------");
        s.Write('\n');
        int nbtot = 0, nbsign = 0, maxent = 0, nbval = 0, nbve = 0, minval = 0, maxval = 0, totval = 0;
        for (int iter = 0; iter < _thedicount.Count; iter++)
        {
            int nbent = _thedicount.ValueAt(iter);
            nbtot += nbent;
            nbsign++;
            if (nbent > maxent)
            {
                maxent = nbent;
            }
            string name = _thedicount.KeyAt(iter);
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
        s.Write("    Nb Total:");
        s.Write(nbtot);
        s.Write("  for ");
        s.Write(nbsign);
        s.Write(" items");
        s.Write('\n');
        s.Write("    Highest count of entities : ");
        s.Write(maxent);
        s.Write(" on one item");
        s.Write('\n');
        if (nbval > 0)
        {
            s.Write("    Summary on Integer Values");
            s.Write('\n');
            s.Write("    Nb Integer Items : ");
            s.Write(nbval);
            s.Write('\n');
            s.Write("    For Nb Entities  : ");
            s.Write(nbve);
            s.Write('\n');
            s.Write("    Cumulated Values : ");
            s.Write(totval);
            s.Write('\n');
            s.Write("    Maximum Value    : ");
            s.Write(maxval);
            s.Write('\n');
            int avg1, avg2;
            avg1 = totval / nbve;
            avg2 = ((totval - (avg1 * nbve)) * 10) / nbve;
            s.Write("    Average Value    : ");
            s.Write(avg1);
            s.Write(" ");
            s.Write(avg2);
            s.Write("/10");
            s.Write('\n');
            s.Write("    Minimum Value    : ");
            s.Write(minval);
            s.Write('\n');
        }
    }

    private bool _thesignonly;
    private bool _thelistat;
    private int _thenbnuls;
    private TCollection_HAsciiString? _thename;
    private string _thelastval = "";
    private IndexedMap<string, int> _thedicount = new IndexedMap<string, int>();
    private IndexedMap<string, NCollection_HSequence<Transient>> _thediclist = new IndexedMap<string, NCollection_HSequence<Transient>>();
}
