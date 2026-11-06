// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_IteratorOfProcessForFinder.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_IteratorOfProcessForFinder_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>An iterator on the results of a <see cref="ProcessForFinder"/>, with their starting objects if asked.</summary>
internal class IteratorOfProcessForFinder : TransferIterator
{
    /// <summary>
    /// Creates an empty Iterator. If <paramref name="withstarts"/> is True, each Binder to be iterated will be
    /// associated to its corresponding Starting Object.
    /// </summary>
    public IteratorOfProcessForFinder(bool withstarts)
    {
        if (withstarts)
        {
            _thestarts = new NCollection_HSequence<Finder>();
        }
    }

    /// <summary>Adds a Binder to the iteration list (construction) with no corresponding Starting Object.</summary>
    /// <exception cref="NoSuchObjectException">The iterator was created with starting objects.</exception>
    public void Add(Binder? binder)
    {
        if (_thestarts is not null)
        {
            throw new NoSuchObjectException(
                "Transfer_IteratorOfProcessForFinder : Add, Starting Object required not provided");
        }
        AddItem(binder);
    }

    /// <summary>
    /// Adds a Binder to the iteration list, associated with its corresponding Starting Object "start". Starting
    /// Object is ignored if not required at Creation time.
    /// </summary>
    public void Add(Binder? binder, Finder? start)
    {
        AddItem(binder);
        if (_thestarts is not null)
        {
            _thestarts.Append(start);
        }
    }

    /// <summary>
    /// After having added all items, keeps or rejects items which are attached to starting data given by
    /// <paramref name="list"/>. <paramref name="keep"/> = True (D) : keeps. False : rejects. Does nothing if
    /// <c>withstarts</c> was False.
    /// </summary>
    public void Filter(NCollection_HSequence<Finder>? list, bool keep = true)
    {
        if (list is null || _thestarts is null)
        {
            return;
        }
        int i, j, nb = _thestarts.Length();
        if (nb == 0)
        {
            return;
        }
        Binder? factice = null;
        var amap = new IndexedMap<Finder, Binder?>(FindHasher.Instance);
        for (i = 1; i <= nb; i++)
        {
            j = amap.Add(_thestarts.Value(i), factice) + 1;
            SelectItem(j, !keep);
        }

        //  Comparison
        nb = list.Length();
        for (i = 1; i <= nb; i++)
        {
            j = amap.IndexOf(list.Value(i)) + 1;
            if (j > 0)
            {
                SelectItem(j, keep);
            }
        }
    }

    /// <summary>Returns True if Starting Object is available (defined at Creation Time).</summary>
    public bool HasStarting
    {
        get
        {
            return _thestarts is not null;
        }
    }

    /// <summary>Returns corresponding Starting Object.</summary>
    /// <exception cref="NoSuchObjectException">No starting objects were asked.</exception>
    public Finder? Starting()
    {
        if (_thestarts is null)
        {
            throw new NoSuchObjectException("TransferIterator : No Starting defined at all");
        }
        return _thestarts.Value(_thecurr);
    }

    private readonly NCollection_HSequence<Finder>? _thestarts;
}
