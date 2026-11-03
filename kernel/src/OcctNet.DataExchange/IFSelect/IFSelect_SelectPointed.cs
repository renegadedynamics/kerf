// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectPointed.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectPointed.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// This type of Selection is intended to describe a direct selection without an explicit criterium, for
/// instance the result of picking viewed entities on a graphic screen It can also be used to provide a list
/// as internal alternate input : this use implies to clear the list once queried
/// </summary>
public class IFSelect_SelectPointed : IFSelect_SelectBase
{

    /// <summary>
    /// Creates a SelectPointed
    /// </summary>
    public IFSelect_SelectPointed()
    {
        theset = false;
    }

    /// <summary>
    /// Clears the list of selected items Also says the list is unset All Add* methods and SetList say the list
    /// is set
    /// </summary>
    public void Clear()
    {
        theitems.Clear();
        theset = false;
    }

    /// <summary>
    /// Tells if the list has been set. Even if empty
    /// </summary>
    public bool IsSet()
    {
        return theset;
    }

    /// <summary>
    /// As SetList but with only one entity If &lt;ent&gt; is Null, the list is said as being set but is empty
    /// </summary>
    public void SetEntity(Transient? ent)
    {
        theitems.Clear();
        theset = true;
        if (ent is null)
        {
            return;
        }
        theitems.Append(ent);
    }

    /// <summary>
    /// Sets a given list to define the list of selected items &lt;list&gt; can be empty or null : in this case,
    /// the list is said as being set, but it is empty To use it as an alternate input, one shot : - SetList or
    /// SetEntity to define the input list - RootResult to get it - then Clear to drop it
    /// </summary>
    public void SetList(NCollection_HSequence<Transient>? list)
    {
        theitems.Clear();
        theset = true;
        if (list is null)
        {
            return;
        }
        int i, nb = list.Length();
        for (i = 1; i <= nb; i++)
        {
            theitems.Append(list.Value(i));
        }
    }

    /// <summary>
    /// Adds an item. Returns True if Done, False if &lt;item&gt; is already in the selected list
    /// </summary>
    public bool Add(Transient? item)
    {
        if (item is null)
        {
            return false;
        }
        for (int i = theitems.Length(); i >= 1; i--)
        {
            if (item == theitems.Value(i))
            {
                return false;
            }
        }
        theitems.Append(item);
        theset = true;
        return true;
    }

    /// <summary>
    /// Removes an item. Returns True if Done, False if &lt;item&gt; was not in the selected list
    /// </summary>
    public bool Remove(Transient? item)
    {
        if (item is null)
        {
            return false;
        }
        for (int i = theitems.Length(); i >= 1; i--)
        {
            if (item == theitems.Value(i))
            {
                theitems.Remove(i);
                return true;
            }
        }
        return true;
    }

    /// <summary>
    /// Toggles status of an item : adds it if not pointed or removes it if already pointed. Returns the new
    /// status (Pointed or not)
    /// </summary>
    public bool Toggle(Transient? item)
    {
        if (item is null)
        {
            return false;
        }
        int num = 0;
        for (int i = theitems.Length(); i >= 1; i--)
        {
            if (item == theitems.Value(i))
            {
                num = i;
            }
        }
        if (num == 0)
        {
            theitems.Append(item);
        }
        else
        {
            theitems.Remove(num);
        }
        return (num == 0);
    }

    /// <summary>
    /// Adds all the items defined in a list. Returns True if at least one item has been added, False else
    /// </summary>
    public bool AddList(NCollection_HSequence<Transient>? list)
    {
        //   Optimized with a Map
        bool res = false;
        if (list is null)
        {
            return res;
        }
        int i, nb = theitems.Length(), nl = list.Length();
        NCollection_Map<Transient> deja = new NCollection_Map<Transient>();
        for (i = 1; i <= nb; i++)
        {
            deja.Add(theitems.Value(i));
        }

        for (i = 1; i <= nl; i++)
        {
            if (!deja.Contains(list.Value(i)))
            {
                theitems.Append(list.Value(i));
            }
        }
        theset = true;
        return res;
    }

    /// <summary>
    /// Removes all the items defined in a list. Returns True if at least one item has been removed, False else
    /// </summary>
    public bool RemoveList(NCollection_HSequence<Transient>? list)
    {
        bool res = false;
        if (list is null)
        {
            return res;
        }
        int i, nb = list.Length();
        for (i = 1; i <= nb; i++)
        {
            res |= Remove(list.Value(i));
        }
        return res;
    }

    /// <summary>
    /// Toggles status of all the items defined in a list : adds it if not pointed or removes it if already
    /// pointed.
    /// </summary>
    public bool ToggleList(NCollection_HSequence<Transient>? list)
    {
        bool res = true;
        if (list is null)
        {
            return res;
        }
        int i, nb = list.Length();
        for (i = 1; i <= nb; i++)
        {
            res |= Toggle(list.Value(i));
        }
        return res;
    }

    /// <summary>
    /// Returns the rank of an item in the selected list, or 0.
    /// </summary>
    public int Rank(Transient? item)
    {
        if (item is null)
        {
            return 0;
        }
        for (int i = theitems.Length(); i >= 1; i--)
        {
            if (item == theitems.Value(i))
            {
                return i;
            }
        }
        return 0;
    }

    /// <summary>
    /// Returns the count of selected items
    /// </summary>
    public int NbItems()
    {
        return theitems.Length();
    }

    /// <summary>
    /// Returns an item given its rank, or a Null Handle
    /// </summary>
    public Transient? Item(int num)
    {
        Transient item = null;
        if (num <= 0 || num > theitems.Length())
        {
            return item;
        }
        return theitems.Value(num);
    }

    /// <summary>
    /// Rebuilds the selected list. Any selected entity which has a bound result is replaced by this result, else
    /// it is removed.
    /// </summary>
    public void Update(Interface_CopyControl? control)
    {
        int nb = theitems.Length();
        for (int i = nb; i > 0; i--)
        {
            Transient enfr = theitems.Value(i);
            if (!control.Search(enfr, out Transient ento))
            {
                theitems.Remove(i);
            }
            else
            {
                theitems.SetValue(i, ento);
            }
        }
    }

    // Update(IFSelect_Transformer) is left out with IFSelect_Transformer (IFSelect's cut).

    /// <summary>
    /// Returns the list of selected items. Only the selected entities which are present in the graph are given
    /// (this result assures uniqueness).
    /// </summary>
    public override Interface_EntityIterator RootResult(Interface_Graph G)
    {
        Interface_EntityIterator result = new Interface_EntityIterator();
        int nb = theitems.Length();
        for (int i = 1; i <= nb; i++)
        {
            Transient item = theitems.Value(i);
            if (G.EntityNumber(item) > 0)
            {
                result.GetOneItem(item);
            }
        }
        return result;
    }

    /// <summary>
    /// Returns a text which identifies the type of selection made. It is "Pointed Entities"
    /// </summary>
    public override string Label()
    {
        return ("Pointed Entities");
    }

    private bool theset;
    private NCollection_Sequence<Transient> theitems = new NCollection_Sequence<Transient>();
}
