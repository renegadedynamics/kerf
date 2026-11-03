// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_IteratorOfProcessForFinder.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_IteratorOfProcessForFinder_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>An iterator on the results of a <see cref="Transfer_ProcessForFinder"/>, with their starting objects if asked.</summary>
public class Transfer_IteratorOfProcessForFinder : Transfer_TransferIterator
{
    /// <summary>
    /// Creates an empty Iterator. If <paramref name="withstarts"/> is True, each Binder to be iterated will be
    /// associated to its corresponding Starting Object.
    /// </summary>
    public Transfer_IteratorOfProcessForFinder(bool withstarts)
    {
        if (withstarts)
        {
            thestarts = new NCollection_HSequence<Transfer_Finder>();
        }
    }

    /// <summary>Adds a Binder to the iteration list (construction) with no corresponding Starting Object.</summary>
    /// <exception cref="NoSuchObjectException">The iterator was created with starting objects.</exception>
    public void Add(Transfer_Binder? binder)
    {
        if (thestarts is not null)
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
    public void Add(Transfer_Binder? binder, Transfer_Finder? start)
    {
        AddItem(binder);
        if (thestarts is not null)
        {
            thestarts.Append(start);
        }
    }

    /// <summary>
    /// After having added all items, keeps or rejects items which are attached to starting data given by
    /// <paramref name="list"/>. <paramref name="keep"/> = True (D) : keeps. False : rejects. Does nothing if
    /// <c>withstarts</c> was False.
    /// </summary>
    public void Filter(NCollection_HSequence<Transfer_Finder>? list, bool keep = true)
    {
        if (list is null || thestarts is null)
        {
            return;
        }
        int i, j, nb = thestarts.Length();
        if (nb == 0)
        {
            return;
        }
        Transfer_Binder? factice = null;
        var amap = new NCollection_IndexedDataMap<Transfer_Finder, Transfer_Binder?>(Transfer_FindHasher.Instance);
        for (i = 1; i <= nb; i++)
        {
            j = amap.Add(thestarts.Value(i), factice);
            SelectItem(j, !keep);
        }

        //  Comparison
        nb = list.Length();
        for (i = 1; i <= nb; i++)
        {
            j = amap.FindIndex(list.Value(i));
            if (j > 0)
            {
                SelectItem(j, keep);
            }
        }
    }

    /// <summary>Returns True if Starting Object is available (defined at Creation Time).</summary>
    public bool HasStarting()
    {
        return thestarts is not null;
    }

    /// <summary>Returns corresponding Starting Object.</summary>
    /// <exception cref="NoSuchObjectException">No starting objects were asked.</exception>
    public Transfer_Finder? Starting()
    {
        if (thestarts is null)
        {
            throw new NoSuchObjectException("TransferIterator : No Starting defined at all");
        }
        return thestarts.Value(thecurr);
    }

    private readonly NCollection_HSequence<Transfer_Finder>? thestarts;
}
