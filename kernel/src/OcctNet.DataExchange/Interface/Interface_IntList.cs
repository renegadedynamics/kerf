// Port of: src/DataExchange/TKXSBase/Interface/Interface_IntList.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_IntList.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

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
public class Interface_IntList
{

    /// <summary>
    /// Creates empty IntList.
    /// </summary>
    public Interface_IntList()
    {
        thenbe = thenbr = thenum = thecount = therank = 0;
    }

    /// <summary>
    /// Creates an IntList for &lt;nbe&gt; entities
    /// </summary>
    public Interface_IntList(int nbe)
    {
        Initialize(nbe);
    }

    /// <summary>
    /// Creates an IntList from another one. if &lt;copied&gt; is True, copies data else, data are not copied,
    /// only the header object is
    /// </summary>
    public Interface_IntList(Interface_IntList other, bool copied)
    {
        thenbe = other.NbEntities();
        thenum = thecount = therank = 0; // szv#4:S4163:12Mar99 initialization needed
        thenbr = other.NbReferences();
        theents = other.Entities();
        therefs = other.References();
        if (copied)
        {
            int i;
            NCollection_HArray1<int> ents = new NCollection_HArray1<int>(0, thenbe);
            ents.Init(0);
            for (i = 1; i <= thenbe; i++)
            {
                ents.SetValue(i, theents.Value(i));
            }
            NCollection_HArray1<int> refs = new NCollection_HArray1<int>(0, thenbr);
            refs.Init(0);
            for (i = 1; i <= thenbr; i++)
            {
                refs.SetValue(i, therefs.Value(i));
            }
            theents = ents;
            therefs = refs;
        }
        SetNumber(other.Number());
    }

    /// <summary>
    /// Initialize IntList by number of entities.
    /// </summary>
    public void Initialize(int nbe)
    {
        thenbe = nbe;
        thenbr = thenum = thecount = therank = 0;
        theents = new NCollection_HArray1<int>(0, nbe);
        theents.Init(0);
    }

    /// <summary>
    /// Returns count of stored references. @return number of references
    /// </summary>
    public int NbReferences()
    {
        return thenbr;
    }

    /// <summary>
    /// Returns entity headers used to describe the lists. @return handle to the array of entity headers
    /// </summary>
    public NCollection_HArray1<int>? Entities()
    {
        return theents;
    }

    /// <summary>
    /// Returns the packed references storage. @return handle to the array of packed references
    /// </summary>
    public NCollection_HArray1<int>? References()
    {
        return therefs;
    }

    /// <summary>
    /// Returns internal values, used for copying @deprecated Use NbReferences(), Entities(), and References()
    /// instead.
    /// </summary>
    /// <remarks>Deprecated in OCCT: use NbReferences(), Entities(), and References() instead.</remarks>
    public void Internals(out int nbrefs, out NCollection_HArray1<int>? ents, out NCollection_HArray1<int>? refs)
    {
        nbrefs = thenbr;
        ents = theents;
        refs = therefs;
    }

    /// <summary>
    /// Returns count of entities to be acknowledged
    /// </summary>
    public int NbEntities()
    {
        return thenbe;
    }

    /// <summary>
    /// Changes the count of entities (ignored if decreased)
    /// </summary>
    public void SetNbEntities(int nbe)
    {
        if (nbe <= theents.Upper())
        {
            return;
        }
        int i;
        NCollection_HArray1<int> ents = new NCollection_HArray1<int>(0, nbe);
        ents.Init(0);
        for (i = 1; i <= thenbe; i++)
        {
            ents.SetValue(i, theents.Value(i));
        }
        theents = ents;
        thenbe = nbe;
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
            if (thenum == -number || number < -thenbe)
            {
                return;
            }
            bool preres = true;
            thenum = -number;
            int aval = theents.Value(thenum); // OCCT: "val", shadowed below in C++
            if (aval == 0)
            {
                thecount = 0;
                therank = 0;
            }
            else if (aval > 0)
            {
                thecount = 1;
                therank = -1;
            }
            if (aval < -1)
            {
                therank = -aval;
                thecount = therefs.Value(therank);
                if (thecount <= 0)
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
            if (thenum == number || number > thenbe)
            {
                return;
            }
            thenum = number;
        }
        else
        {
            return;
        }

        int val = theents.Value(thenum);
        if (val == 0)
        {
            thecount = 0;
            therank = 0;
        }
        else if (val > 0)
        {
            thecount = 1;
            therank = -1;
        }
        else if (val < -1)
        {
            therank = -val;
            thecount = 0;
            if (therefs.Value(therank + 1) == 0)
            {
                thecount = -therefs.Value(therank);
            }
            else
            {
                for (int j = 1;; j++)
                {
                    val = therefs.Value(therank + j);
                    if (val >= 0)
                    {
                        break;
                    }
                    thecount++;
                }
                if (val > 0)
                {
                    thecount++;
                }
            }
        }
        else
        {
            thecount = 0;
            therank = -1;
        } // val == -1 reste
    }

    /// <summary>
    /// Returns the current entity number
    /// </summary>
    public int Number()
    {
        return thenum;
    }

    /// <summary>
    /// Returns an IntList, identical to &lt;me&gt; but set to a specified entity Number By default, not copied
    /// (in order to be read) Specified &lt;copied&gt; to produce another list and edit it
    /// </summary>
    public Interface_IntList List(int number, bool copied = false)
    {
        Interface_IntList alist = new Interface_IntList(this, copied);
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
        if (NbEntities() == 0 || thenum == 0)
        {
            return;
        }

        int val = theents.Value(thenum);
        if (val < -1)
        {
            return;
        }
        else if (mode)
        {
            if (val == 0)
            {
                theents.SetValue(thenum, -1);
            }
            else if (val > 0)
            {
                Reservate(2);
                theents.SetValue(thenum, -thenbr);
                therefs.SetValue(thenbr + 1, val);
                thenbr++;
            }
        }
        else if (!mode)
        {
            if (val == -1)
            {
                theents.SetValue(thenum, 0);
            }
            else if (therefs.Value(therank + 1) >= 0)
            {
                theents.SetValue(thenum, therefs.Value(therank + 1));
                if (thenbr == therank + 1)
                {
                    thenbr--;
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
            if (thenum == 0)
            {
                return;
            }
            thenbr++;
            therefs.SetValue(thenbr, 0); // will contain the number ...
            therank = thenbr;
            theents.SetValue(thenum, -thenbr);
            thenbr -= count;
            return;
        }
        int up, oldup = 0;
        if (thenbr == 0)
        { // i.e. not yet allocated ...
            up = thenbe / 2 + 1;
            if (up < 2)
            {
                up = 2;
            }
            if (up < count)
            {
                up = count * 3 / 2;
            }
            therefs = new NCollection_HArray1<int>(0, up);
            therefs.Init(0);
            thenbr = 2; // we start after (convenience of addressing)
        }
        oldup = therefs.Upper();
        if (thenbr + count < oldup)
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
            refs.SetValue(i, therefs.Value(i));
        }
        therefs = refs;
    }

    /// <summary>
    /// Adds a reference (as an integer value, an entity number) to the current entity number. Zero is ignored
    /// </summary>
    public void Add(int @ref)
    {
        if (thenum == 0)
        {
            return;
        }
        //   @ref < 0 : pre-reservation
        if (@ref < 0)
        {
            Add(-@ref);
            if (therank <= 0)
            {
                return;
            }
            if (therefs.Value(therank) >= 0)
            {
                therefs.SetValue(therank, thecount);
            }
            return;
        }

        if (therank == 0)
        {
            theents.SetValue(thenum, @ref);
            thecount = 1;
            therank = -1;
        }
        else if (therank < 0)
        {
            Reservate(2);
            therank = thenbr;
            int val = theents.Value(thenum);
            theents.SetValue(thenum, -thenbr);
            if (thecount == 1)
            {
                therefs.SetValue(thenbr + 1, -val);
                thenbr++;
            }
            therefs.SetValue(thenbr + 1, @ref);
            thenbr++;
            thecount++;
        }
        else if (thenbr == therank + thecount)
        { // free space at end
            therefs.SetValue(thenbr, -therefs.Value(thenbr));
            therefs.SetValue(thenbr + 1, @ref);
            thenbr++;
            thecount++;
        }
        else if (therefs.Value(therank + thecount + 1) == 0)
        { // free space after
            therefs.SetValue(therank + thecount, -therefs.Value(therank + thecount));
            therefs.SetValue(therank + thecount + 1, @ref);
            thecount++;
        }
        else
        { // copy further !
            Reservate(thecount + 2);
            int rank = therank;
            therank = thenbr;
            theents.SetValue(thenum, -therank);
            for (int i = 1; i < thecount; i++)
            {
                therefs.SetValue(therank + i, therefs.Value(rank + i));
                therefs.SetValue(rank + i, 0);
            }
            therefs.SetValue(therank + thecount, -therefs.Value(rank + thecount));
            therefs.SetValue(rank + thecount, 0);
            therefs.SetValue(therank + thecount + 1, @ref);
            thecount++;
            thenbr = therank + thecount + 1;
        }
    }

    /// <summary>
    /// Returns the count of refs attached to current entity number
    /// </summary>
    public int Length()
    {
        return thecount;
    }

    /// <summary>
    /// Returns True if the list for a number (default is taken as current) is "redefined" (useful for empty
    /// list)
    /// </summary>
    public bool IsRedefined(int num = 0)
    {
        int n = (num == 0 ? thenum : num);
        if (NbEntities() == 0 || n == 0)
        {
            return false;
        }
        if (theents.Value(n) < 0)
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
        if (thenum == 0)
        {
            return 0;
        }
        if (num <= 0 || num > thecount)
        {
            return 0;
        }
        if (thecount == 0)
        {
            return 0;
        }
        if (therank <= 0)
        {
            return theents.Value(thenum);
        }
        int val = therefs.Value(therank + num);
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
        if (thenbr == 0)
        {
            return; // already clear
        }
        int i, low, up;
        low = theents.Lower();
        up = theents.Upper();
        for (i = low; i <= up; i++)
        {
            theents.SetValue(i, 0);
        }
        thenbr = 0;
        if (therefs is null)
        {
            return;
        }
        low = therefs.Lower();
        up = therefs.Upper();
        for (i = low; i <= up; i++)
        {
            therefs.SetValue(i, 0);
        }
    }

    /// <summary>
    /// Resizes lists to exact sizes. For list of refs, a positive margin can be added.
    /// </summary>
    public void AdjustSize(int margin = 0)
    {
        int i, up = theents.Upper();
        if (up > thenbe)
        {
            NCollection_HArray1<int> ents = new NCollection_HArray1<int>(0, thenbe);
            ents.Init(0);
            for (i = 1; i <= thenbe; i++)
            {
                ents.SetValue(i, theents.Value(i));
            }
            theents = ents;
        }
        if (thenbr == 0)
        {
            Reservate(margin);
        }
        else
        {
            up = therefs.Upper();
            if (up >= thenbr && up <= thenbr + margin)
            {
                return;
            }
            NCollection_HArray1<int> refs = new NCollection_HArray1<int>(0, thenbr + margin);
            refs.Init(0);
            for (i = 1; i <= thenbr; i++)
            {
                refs.SetValue(i, therefs.Value(i));
            }
            therefs = refs;
        }
    }


    private int thenbe;
    private int thenbr;
    private int thenum;
    private int thecount;
    private int therank;
    private NCollection_HArray1<int>? theents;
    private NCollection_HArray1<int>? therefs;
}
