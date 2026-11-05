// Port of: src/DataExchange/TKXSBase/Interface/Interface_IntList.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_IntList.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// This class detains the data which describe a Graph. A Graph has two lists, one for shared refs, one for
/// sharing refs (the reverses). Each list comprises, for each Entity of the Model of the Graph, a list of
/// Entities (shared or sharing). In fact, entities are identified by their numbers in the Model or Graph :
/// this gives better performances. A simple way to implement this is to instantiate a HArray1 with a
/// HSequenceOfInteger : each Entity Number designates a value, which is a Sequence (if it is null, it is
/// considered as empty : this is a little optimisation). This class gives a more efficient way to implement
/// this. It has two lists (two arrays of integers), one to describe list (empty, one value given
/// immediately, or negated index in the second list), one to store refs (pointed from the first list). This
/// is much more efficient than a list of sequences, in terms of speed (especially for read) and of memory An
/// IntList can also be set to access data for a given entity number, it then acts as a single sequence
/// Remark that if an IntList is created from another one, it can be read, but if it is created without
/// copying, it may not be edited
/// </summary>
internal class IntList
{

    /// <summary>
    /// Creates empty IntList.
    /// </summary>
    public IntList()
    {
        _thenbe = _thenbr = _thenum = _thecount = _therank = 0;
    }

    /// <summary>
    /// Creates an IntList for &lt;nbe&gt; entities
    /// </summary>
    public IntList(int nbe)
    {
        Initialize(nbe);
    }

    /// <summary>
    /// Creates an IntList from another one. if &lt;copied&gt; is True, copies data else, data are not copied,
    /// only the header object is
    /// </summary>
    public IntList(IntList other, bool copied)
    {
        _thenbe = other.EntityCount;
        _thenum = _thecount = _therank = 0; // szv#4:S4163:12Mar99 initialization needed
        _thenbr = other.ReferenceCount;
        _theents = other.Entities();
        _therefs = other.References();
        if (copied)
        {
            int i;
            NCollection_HArray1<int> ents = new NCollection_HArray1<int>(0, _thenbe);
            ents.Init(0);
            for (i = 1; i <= _thenbe; i++)
            {
                ents.SetValue(i, _theents.Value(i));
            }
            NCollection_HArray1<int> refs = new NCollection_HArray1<int>(0, _thenbr);
            refs.Init(0);
            for (i = 1; i <= _thenbr; i++)
            {
                refs.SetValue(i, _therefs.Value(i));
            }
            _theents = ents;
            _therefs = refs;
        }
        SetNumber(other.Number());
    }

    /// <summary>
    /// Initialize IntList by number of entities.
    /// </summary>
    public void Initialize(int nbe)
    {
        _thenbe = nbe;
        _thenbr = _thenum = _thecount = _therank = 0;
        _theents = new NCollection_HArray1<int>(0, nbe);
        _theents.Init(0);
    }

    /// <summary>
    /// Returns count of stored references. @return number of references
    /// </summary>
    public int ReferenceCount
    {
        get
        {
            return _thenbr;
        }
    }

    /// <summary>
    /// Returns entity headers used to describe the lists. @return handle to the array of entity headers
    /// </summary>
    public NCollection_HArray1<int>? Entities()
    {
        return _theents;
    }

    /// <summary>
    /// Returns the packed references storage. @return handle to the array of packed references
    /// </summary>
    public NCollection_HArray1<int>? References()
    {
        return _therefs;
    }

    /// <summary>
    /// Returns internal values, used for copying @deprecated Use NbReferences(), Entities(), and References()
    /// instead.
    /// </summary>
    /// <remarks>Deprecated in OCCT: use NbReferences(), Entities(), and References() instead.</remarks>
    public void Internals(out int nbrefs, out NCollection_HArray1<int>? ents, out NCollection_HArray1<int>? refs)
    {
        nbrefs = _thenbr;
        ents = _theents;
        refs = _therefs;
    }

    /// <summary>
    /// Returns count of entities to be acknowledged
    /// </summary>
    public int EntityCount
    {
        get
        {
            return _thenbe;
        }
    }

    /// <summary>
    /// Changes the count of entities (ignored if decreased)
    /// </summary>
    public void SetNbEntities(int nbe)
    {
        if (nbe <= _theents.Upper())
        {
            return;
        }
        int i;
        NCollection_HArray1<int> ents = new NCollection_HArray1<int>(0, nbe);
        ents.Init(0);
        for (i = 1; i <= _thenbe; i++)
        {
            ents.SetValue(i, _theents.Value(i));
        }
        _theents = ents;
        _thenbe = nbe;
    }

    /// <summary>
    /// Sets an entity number as current (for read and fill)
    /// </summary>
    public void SetNumber(int number)
    {
        //  Pre-reservation usage : to be requested specifically ! . optimization
        //   <preres> verifies that the pre-reservation is valid
        if (number < 0)
        {
            if (_thenum == -number || number < -_thenbe)
            {
                return;
            }
            bool preres = true;
            _thenum = -number;
            int aval = _theents.Value(_thenum); // OCCT: "val", shadowed below in C++
            if (aval == 0)
            {
                _thecount = 0;
                _therank = 0;
            }
            else if (aval > 0)
            {
                _thecount = 1;
                _therank = -1;
            }
            if (aval < -1)
            {
                _therank = -aval;
                _thecount = _therefs.Value(_therank);
                if (_thecount <= 0)
                {
                    preres = false;
                }
            }
            if (preres)
            {
                return;
            }
        }
        //  Current usage. The following in current usage or if no pre-reservation
        else if (number > 0)
        {
            if (_thenum == number || number > _thenbe)
            {
                return;
            }
            _thenum = number;
        }
        else
        {
            return;
        }

        int val = _theents.Value(_thenum);
        if (val == 0)
        {
            _thecount = 0;
            _therank = 0;
        }
        else if (val > 0)
        {
            _thecount = 1;
            _therank = -1;
        }
        else if (val < -1)
        {
            _therank = -val;
            _thecount = 0;
            if (_therefs.Value(_therank + 1) == 0)
            {
                _thecount = -_therefs.Value(_therank);
            }
            else
            {
                for (int j = 1;; j++)
                {
                    val = _therefs.Value(_therank + j);
                    if (val >= 0)
                    {
                        break;
                    }
                    _thecount++;
                }
                if (val > 0)
                {
                    _thecount++;
                }
            }
        }
        else
        {
            _thecount = 0;
            _therank = -1;
        } // val == -1 reste
    }

    /// <summary>
    /// Returns the current entity number
    /// </summary>
    public int Number()
    {
        return _thenum;
    }

    /// <summary>
    /// Returns an IntList, identical to &lt;me&gt; but set to a specified entity Number By default, not copied
    /// (in order to be read) Specified &lt;copied&gt; to produce another list and edit it
    /// </summary>
    public IntList List(int number, bool copied = false)
    {
        IntList alist = new IntList(this, copied);
        alist.SetNumber(number);
        return alist;
    }

    /// <summary>
    /// Sets current entity list to be redefined or not This is used in a Graph for redefinition list : it can be
    /// disable (no redefinition, i.e. list is cleared), or enabled (starts as empty). The original list has not
    /// to be "redefined"
    /// </summary>
    public void SetRedefined(bool mode)
    {
        if (EntityCount == 0 || _thenum == 0)
        {
            return;
        }

        int val = _theents.Value(_thenum);
        if (val < -1)
        {
            return;
        }
        else if (mode)
        {
            if (val == 0)
            {
                _theents.SetValue(_thenum, -1);
            }
            else if (val > 0)
            {
                Reservate(2);
                _theents.SetValue(_thenum, -_thenbr);
                _therefs.SetValue(_thenbr + 1, val);
                _thenbr++;
            }
        }
        else if (!mode)
        {
            if (val == -1)
            {
                _theents.SetValue(_thenum, 0);
            }
            else if (_therefs.Value(_therank + 1) >= 0)
            {
                _theents.SetValue(_thenum, _therefs.Value(_therank + 1));
                if (_thenbr == _therank + 1)
                {
                    _thenbr--;
                }
            }
        }
    }

    /// <summary>
    /// Makes a reservation for &lt;count&gt; references to be later attached to the current entity. If required,
    /// it increases the size of array used to store refs. Remark that if count is less than two, it does nothing
    /// (because immediate storing)
    /// </summary>
    public void Reservate(int count)
    {
        //  Reservate (-count) = Reservate (count) + allocation on current entity + 1
        if (count < 0)
        {
            Reservate(-count - 1);
            if (_thenum == 0)
            {
                return;
            }
            _thenbr++;
            _therefs.SetValue(_thenbr, 0); // will contain the number ...
            _therank = _thenbr;
            _theents.SetValue(_thenum, -_thenbr);
            _thenbr -= count;
            return;
        }
        int up, oldup = 0;
        if (_thenbr == 0)
        { // i.e. not yet allocated ...
            up = _thenbe / 2 + 1;
            if (up < 2)
            {
                up = 2;
            }
            if (up < count)
            {
                up = count * 3 / 2;
            }
            _therefs = new NCollection_HArray1<int>(0, up);
            _therefs.Init(0);
            _thenbr = 2; // we start after (convenience of addressing)
        }
        oldup = _therefs.Upper();
        if (_thenbr + count < oldup)
        {
            return; // OK
        }
        up = oldup * 3 / 2 + count;
        if (up < 2)
        {
            up = 2;
        }
        NCollection_HArray1<int> refs = new NCollection_HArray1<int>(0, up);
        refs.Init(0);
        for (int i = 1; i <= oldup; i++)
        {
            refs.SetValue(i, _therefs.Value(i));
        }
        _therefs = refs;
    }

    /// <summary>
    /// Adds a reference (as an integer value, an entity number) to the current entity number. Zero is ignored
    /// </summary>
    public void Add(int @ref)
    {
        if (_thenum == 0)
        {
            return;
        }
        //   @ref < 0 : pre-reservation
        if (@ref < 0)
        {
            Add(-@ref);
            if (_therank <= 0)
            {
                return;
            }
            if (_therefs.Value(_therank) >= 0)
            {
                _therefs.SetValue(_therank, _thecount);
            }
            return;
        }

        if (_therank == 0)
        {
            _theents.SetValue(_thenum, @ref);
            _thecount = 1;
            _therank = -1;
        }
        else if (_therank < 0)
        {
            Reservate(2);
            _therank = _thenbr;
            int val = _theents.Value(_thenum);
            _theents.SetValue(_thenum, -_thenbr);
            if (_thecount == 1)
            {
                _therefs.SetValue(_thenbr + 1, -val);
                _thenbr++;
            }
            _therefs.SetValue(_thenbr + 1, @ref);
            _thenbr++;
            _thecount++;
        }
        else if (_thenbr == _therank + _thecount)
        { // free space at end
            _therefs.SetValue(_thenbr, -_therefs.Value(_thenbr));
            _therefs.SetValue(_thenbr + 1, @ref);
            _thenbr++;
            _thecount++;
        }
        else if (_therefs.Value(_therank + _thecount + 1) == 0)
        { // free space after
            _therefs.SetValue(_therank + _thecount, -_therefs.Value(_therank + _thecount));
            _therefs.SetValue(_therank + _thecount + 1, @ref);
            _thecount++;
        }
        else
        { // copy further !
            Reservate(_thecount + 2);
            int rank = _therank;
            _therank = _thenbr;
            _theents.SetValue(_thenum, -_therank);
            for (int i = 1; i < _thecount; i++)
            {
                _therefs.SetValue(_therank + i, _therefs.Value(rank + i));
                _therefs.SetValue(rank + i, 0);
            }
            _therefs.SetValue(_therank + _thecount, -_therefs.Value(rank + _thecount));
            _therefs.SetValue(rank + _thecount, 0);
            _therefs.SetValue(_therank + _thecount + 1, @ref);
            _thecount++;
            _thenbr = _therank + _thecount + 1;
        }
    }

    /// <summary>
    /// Returns the count of refs attached to current entity number
    /// </summary>
    public int Length()
    {
        return _thecount;
    }

    /// <summary>
    /// Returns True if the list for a number (default is taken as current) is "redefined" (useful for empty
    /// list)
    /// </summary>
    public bool IsRedefined(int num = 0)
    {
        int n = (num == 0 ? _thenum : num);
        if (EntityCount == 0 || n == 0)
        {
            return false;
        }
        if (_theents.Value(n) < 0)
        {
            return true;
        }
        return false;
    }

    /// <summary>
    /// Returns a reference number in the list for current number, according to its rank
    /// </summary>
    public int Value(int num)
    {
        if (_thenum == 0)
        {
            return 0;
        }
        if (num <= 0 || num > _thecount)
        {
            return 0;
        }
        if (_thecount == 0)
        {
            return 0;
        }
        if (_therank <= 0)
        {
            return _theents.Value(_thenum);
        }
        int val = _therefs.Value(_therank + num);
        if (val < 0)
        {
            return -val;
        }
        return val;
    }

    /// <summary>
    /// Removes an item in the list for current number, given its rank Returns True if done, False else
    /// </summary>
    public bool Remove(int num)
    {
        return false; // not yet implemented
    }

    /// <summary>
    /// Clears all data, hence each entity number has an empty list
    /// </summary>
    public void Clear()
    {
        if (_thenbr == 0)
        {
            return; // already clear
        }
        int i, low, up;
        low = _theents.Lower();
        up = _theents.Upper();
        for (i = low; i <= up; i++)
        {
            _theents.SetValue(i, 0);
        }
        _thenbr = 0;
        if (_therefs is null)
        {
            return;
        }
        low = _therefs.Lower();
        up = _therefs.Upper();
        for (i = low; i <= up; i++)
        {
            _therefs.SetValue(i, 0);
        }
    }

    /// <summary>
    /// Resizes lists to exact sizes. For list of refs, a positive margin can be added.
    /// </summary>
    public void AdjustSize(int margin = 0)
    {
        int i, up = _theents.Upper();
        if (up > _thenbe)
        {
            NCollection_HArray1<int> ents = new NCollection_HArray1<int>(0, _thenbe);
            ents.Init(0);
            for (i = 1; i <= _thenbe; i++)
            {
                ents.SetValue(i, _theents.Value(i));
            }
            _theents = ents;
        }
        if (_thenbr == 0)
        {
            Reservate(margin);
        }
        else
        {
            up = _therefs.Upper();
            if (up >= _thenbr && up <= _thenbr + margin)
            {
                return;
            }
            NCollection_HArray1<int> refs = new NCollection_HArray1<int>(0, _thenbr + margin);
            refs.Init(0);
            for (i = 1; i <= _thenbr; i++)
            {
                refs.SetValue(i, _therefs.Value(i));
            }
            _therefs = refs;
        }
    }


    private int _thenbe;
    private int _thenbr;
    private int _thenum;
    private int _thecount;
    private int _therank;
    private NCollection_HArray1<int>? _theents;
    private NCollection_HArray1<int>? _therefs;
}
