// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectPointed.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectPointed.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// This type of Selection is intended to describe a direct selection without an explicit criterium, for
/// instance the result of picking viewed entities on a graphic screen It can also be used to provide a list
/// as internal alternate input : this use implies to clear the list once queried
/// </summary>
internal class SelectPointed : SelectBase
{

    /// <summary>
    /// Creates a SelectPointed
    /// </summary>
    public SelectPointed()
    {
        _theset = false;
    }

    /// <summary>
    /// Clears the list of selected items Also says the list is unset All Add* methods and SetList say the list
    /// is set
    /// </summary>
    public void Clear()
    {
        _theitems.Clear();
        _theset = false;
    }

    /// <summary>
    /// Tells if the list has been set. Even if empty
    /// </summary>
    public bool IsSet
    {
        get
        {
            return _theset;
        }
    }

    /// <summary>
    /// As SetList but with only one entity If &lt;ent&gt; is Null, the list is said as being set but is empty
    /// </summary>
    public void SetEntity(Transient? ent)
    {
        _theitems.Clear();
        _theset = true;
        if (ent is null)
        {
            return;
        }
        _theitems.Append(ent);
    }

    /// <summary>
    /// Sets a given list to define the list of selected items &lt;list&gt; can be empty or null : in this case,
    /// the list is said as being set, but it is empty To use it as an alternate input, one shot : - SetList or
    /// SetEntity to define the input list - RootResult to get it - then Clear to drop it
    /// </summary>
    public void SetList(NCollection_HSequence<Transient>? list)
    {
        _theitems.Clear();
        _theset = true;
        if (list is null)
        {
            return;
        }
        int i, nb = list.Length();
        for (i = 1; i <= nb; i++)
        {
            _theitems.Append(list.Value(i));
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
        for (int i = _theitems.Length(); i >= 1; i--)
        {
            if (item == _theitems.Value(i))
            {
                return false;
            }
        }
        _theitems.Append(item);
        _theset = true;
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
        for (int i = _theitems.Length(); i >= 1; i--)
        {
            if (item == _theitems.Value(i))
            {
                _theitems.Remove(i);
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
        for (int i = _theitems.Length(); i >= 1; i--)
        {
            if (item == _theitems.Value(i))
            {
                num = i;
            }
        }
        if (num == 0)
        {
            _theitems.Append(item);
        }
        else
        {
            _theitems.Remove(num);
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
        int i, nb = _theitems.Length(), nl = list.Length();
        HashSet<Transient> deja = new HashSet<Transient>();
        for (i = 1; i <= nb; i++)
        {
            deja.Add(_theitems.Value(i));
        }

        for (i = 1; i <= nl; i++)
        {
            if (!deja.Contains(list.Value(i)))
            {
                _theitems.Append(list.Value(i));
            }
        }
        _theset = true;
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
        for (int i = _theitems.Length(); i >= 1; i--)
        {
            if (item == _theitems.Value(i))
            {
                return i;
            }
        }
        return 0;
    }

    /// <summary>
    /// Returns the count of selected items
    /// </summary>
    public int ItemCount
    {
        get
        {
            return _theitems.Length();
        }
    }

    /// <summary>
    /// Returns an item given its rank, or a Null Handle
    /// </summary>
    public Transient? Item(int num)
    {
        Transient item = null;
        if (num <= 0 || num > _theitems.Length())
        {
            return item;
        }
        return _theitems.Value(num);
    }

    /// <summary>
    /// Rebuilds the selected list. Any selected entity which has a bound result is replaced by this result, else
    /// it is removed.
    /// </summary>
    public void Update(CopyControl? control)
    {
        int nb = _theitems.Length();
        for (int i = nb; i > 0; i--)
        {
            Transient enfr = _theitems.Value(i);
            if (!control.Search(enfr, out Transient ento))
            {
                _theitems.Remove(i);
            }
            else
            {
                _theitems.SetValue(i, ento);
            }
        }
    }

    // Update(IFSelect_Transformer) is left out with IFSelect_Transformer (IFSelect's cut).

    /// <summary>
    /// Returns the list of selected items. Only the selected entities which are present in the graph are given
    /// (this result assures uniqueness).
    /// </summary>
    public override EntityIterator RootResult(EntityGraph g)
    {
        EntityIterator result = new EntityIterator();
        int nb = _theitems.Length();
        for (int i = 1; i <= nb; i++)
        {
            Transient item = _theitems.Value(i);
            if (g.EntityNumber(item) > 0)
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

    private bool _theset;
    private NCollection_Sequence<Transient> _theitems = new NCollection_Sequence<Transient>();
}
