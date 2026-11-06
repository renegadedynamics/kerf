// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectionIterator.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectionIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// Defines an Iterator on a list of Selections
/// </summary>
internal class SelectionIterator
{

    /// <summary>
    /// Creates an empty iterator, ready to be filled
    /// </summary>
    public SelectionIterator()
    {
        _thecurr = 1;
        _thelist = new NCollection_HSequence<AbstractSelection>();
    }

    /// <summary>
    /// Creates an iterator from a Selection : it lists the Selections from which &lt;sel&gt; depends (given by
    /// its method FillIterator)
    /// </summary>
    public SelectionIterator(AbstractSelection? sel)
    {
        _thecurr = 1;
        _thelist = new NCollection_HSequence<AbstractSelection>();
        sel.FillIterator(this);
    }

    /// <summary>
    /// Adds to an iterator the content of another one (each selection is present only once in the result)
    /// </summary>
    public void AddFromIter(SelectionIterator iter)
    {
        for (; iter.More(); iter.Next())
        {
            AddItem(iter.Value());
        }
    }

    /// <summary>
    /// Adds a Selection to an iterator (if not yet noted)
    /// </summary>
    public void AddItem(AbstractSelection? sel)
    {
        if (sel is not null)
        {
            _thelist.Append(sel);
        }
    }

    /// <summary>
    /// Adds a list of Selections to an iterator (this list comes from the description of a Selection or a
    /// Dispatch, etc...)
    /// </summary>
    public void AddList(NCollection_Sequence<AbstractSelection>? list)
    {
        int nb = list.Length(); // <list> Pas thelist Handle
        for (int i = 1; i <= nb; i++)
        {
            _thelist.Append(list.Value(i));
        }
    }

    /// <summary>
    /// Returns True if there are more Selections to get
    /// </summary>
    public bool More()
    {
        return (_thecurr <= _thelist.Length());
    }

    /// <summary>
    /// Sets iterator to the next item
    /// </summary>
    public void Next()
    {
        _thecurr++;
    }

    /// <summary>
    /// Returns the current Selection being iterated Error if count of Selection has been passed
    /// </summary>
    public AbstractSelection? Value()
    {
        return _thelist.Value(_thecurr);
    }

    private int _thecurr;
    private NCollection_HSequence<AbstractSelection>? _thelist;
}
