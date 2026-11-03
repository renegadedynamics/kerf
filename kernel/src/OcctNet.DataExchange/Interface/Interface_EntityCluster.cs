// Port of: src/DataExchange/TKXSBase/Interface/Interface_EntityCluster.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_EntityCluster.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// Auxiliary class for EntityList. An EntityList designates an EntityCluster, which brings itself an fixed
/// maximum count of Entities. If it is full, it gives access to another cluster ("Next"). This class is
/// intended to give a good compromise between access time (faster than a Sequence, good for little count)
/// and memory use (better than a Sequence in any case, overall for little count, better than an Array for a
/// very little count. It is designed for a light management. Remark that a new Item may not be Null, because
/// this is the criterium used for "End of List"
/// </summary>
public class Interface_EntityCluster : Transient
{

    /// <summary>
    /// Creates an empty, non-chained, EntityCluster
    /// </summary>
    public Interface_EntityCluster()
    {
    }

    /// <summary>
    /// Creates a non-chained EntityCluster, filled with one Entity
    /// </summary>
    public Interface_EntityCluster(Transient? ent)
    {
        theents[0] = ent;
    }

    /// <summary>
    /// Creates an empty EntityCluster, chained with another one (that is, put BEFORE this other one in the list)
    /// </summary>
    public Interface_EntityCluster(Interface_EntityCluster? ec)
    {
        thenext = ec;
    }

    /// <summary>
    /// Creates an EntityCluster, filled with a first Entity, and chained to another EntityCluster (BEFORE it, as
    /// above)
    /// </summary>
    public Interface_EntityCluster(Transient? ent, Interface_EntityCluster? ec)
    {
        theents[0] = ent;
        thenext = ec;
    }

    /// <summary>
    /// Appends an Entity to the Cluster. If it is not full, adds the entity directly inside itself. Else,
    /// transmits to its Next and Creates it if it does not yet exist
    /// </summary>
    public void Append(Transient? ent)
    {
        if (ent is null)
        {
            throw new NullObjectException("Interface_EntityCluster Append");
        }
        if (theents[0] is null)
        {
            theents[0] = ent;
        }
        else if (theents[1] is null)
        {
            theents[1] = ent;
        }
        else if (theents[2] is null)
        {
            theents[2] = ent;
        }
        else if (theents[3] is null)
        {
            theents[3] = ent;
        }
        else
        { // If this one is full ...
            if (thenext is null)
            {
                thenext = new Interface_EntityCluster(ent);
            }
            else
            {
                Interface_EntityCluster aCurEntClust = thenext;
                while (aCurEntClust.HasNext() && aCurEntClust.IsLocalFull())
                {
                    aCurEntClust = aCurEntClust.thenext;
                }
                aCurEntClust.Append(ent);
            }
        }
    }

    /// <summary>
    /// Removes an Entity from the Cluster. If it is not found, calls its Next one to do so. Returns True if it
    /// becomes itself empty, False else (thus, a Cluster which becomes empty is deleted from the list)
    /// </summary>
    public bool Remove(Transient? ent)
    {
        if (ent is null)
        {
            throw new NullObjectException("Interface_EntityCluster Remove");
        }
        int i;
        //  Is <ent> here? if yes, we have its rank
        if (ent == theents[0])
        {
            i = 1;
        }
        else if (ent == theents[1])
        {
            i = 2;
        }
        else if (ent == theents[2])
        {
            i = 3;
        }
        else if (ent == theents[3])
        {
            i = 4;

            //  Otherwise, go to the next one, which can then become empty .
            //  We remove the empty cluster from the list (in principle it's the last one)
        }
        else
        { // Not found in this one ...
            if (thenext is null)
            {
                return false;
            }
            bool res = thenext.Remove(ent); // OCCT: an int in C++
            if (res)
            {
                thenext = thenext.Next();
            }
            return false; // OCCT: false even when <ent> was removed further on
        }
        return Remove(i);
    }

    /// <summary>
    /// Removes an Entity from the Cluster, given its rank. If &lt;num&gt; is greater than NbLocal, calls its
    /// Next with (num - NbLocal), Returns True if it becomes itself empty, False else
    /// </summary>
    public bool Remove(int num)
    {
        if (num < 1)
        {
            throw new OutOfRangeException("EntityCluster : Remove");
        }
        int n = NbLocal();
        if (num > n)
        {
            if (thenext is null)
            {
                throw new OutOfRangeException("EntityCluster : Remove");
            }
            bool res = thenext.Remove(num - n);
            if (res)
            {
                thenext = thenext.Next();
            }
            return false;
        }
        // OCCT: "j--", which runs j down past the start of the array (a write out of bounds in C++, an endless loop
        // in effect) whenever num < n; fixed to "j++", the shift the code means (occt-issues.md, unit 8A1)
        for (int j = num; j < n; j++)
        {
            theents[j - 1] = theents[j];
        }
        theents[3] = null; // We Nullify at the end
        return (n == 1); // Old NbLocal == 1 . becomes null
    }

    /// <summary>
    /// Returns total count of Entities (including Next)
    /// </summary>
    public int NbEntities()
    {
        int nb = NbLocal();
        if (thenext is not null)
        {
            nb += thenext.NbEntities();
        }
        return nb;
    }

    /// <summary>
    /// Returns the Entity identified by its rank in the list (including Next)
    /// </summary>
    public Transient? Value(int num)
    {
        int nb = NbLocal(), aLocalNum = num;
        if (num <= 0)
        {
            throw new OutOfRangeException("Interface EntityCluster : Value");
        }
        if (num > nb)
        {
            Interface_EntityCluster aCurEntClust = thenext;
            aLocalNum -= nb;
            while (aLocalNum > aCurEntClust.NbLocal())
            {
                if (!aCurEntClust.HasNext())
                {
                    throw new OutOfRangeException("Interface EntityCluster : Value");
                }
                aCurEntClust = aCurEntClust.thenext;
                aLocalNum -= nb; // OCCT: the first cluster's count, not the current one's (equal while clusters are full)
            }
            return aCurEntClust.theents[aLocalNum - 1];
        }
        return theents[num - 1]; // numbering from 0
    }

    /// <summary>
    /// Changes an Entity given its rank.
    /// </summary>
    public void SetValue(int num, Transient? ent)
    {
        if (ent is null)
        {
            throw new NullObjectException("Interface_EntityCluster SetValue");
        }
        int nb = NbLocal(), aLocalNum = num;
        if (num <= 0)
        {
            throw new OutOfRangeException("Interface EntityCluster : SetValue");
        }
        if (num > nb)
        {
            Interface_EntityCluster aCurEntClust = thenext;
            aLocalNum -= nb;
            while (aLocalNum > aCurEntClust.NbLocal())
            {
                if (thenext is null) // OCCT: tests this cluster's next, not the current one's
                {
                    throw new OutOfRangeException("Interface EntityCluster : SetValue");
                }
                aCurEntClust = aCurEntClust.thenext;
                aLocalNum -= nb; // OCCT: the first cluster's count, not the current one's (equal while clusters are full)
            }
            aCurEntClust.theents[aLocalNum - 1] = ent;
        }
        else
        {
            theents[num - 1] = ent; // numbering from 0
        }
    }

    /// <summary>
    /// Fills an Iterator with designated Entities (includes Next)
    /// </summary>
    public void FillIterator(Interface_EntityIterator iter)
    {
        if (theents[0] is not null)
        {
            iter.GetOneItem(theents[0]);
        }
        if (theents[1] is not null)
        {
            iter.GetOneItem(theents[1]);
        }
        if (theents[2] is not null)
        {
            iter.GetOneItem(theents[2]);
        }
        if (theents[3] is not null)
        {
            iter.GetOneItem(theents[3]);
        }
        if (thenext is not null)
        {
            thenext.FillIterator(iter);
        }
    }


    /// <summary>
    /// Returns True if all the set of entities local to a Cluster is full. Used by EntityList.
    /// </summary>
    internal bool IsLocalFull()
    {
        return (theents[3] is not null);
    }

    /// <summary>
    /// Returns count of entities in the local set (without Next) Entities can then be read normally by method
    /// Value
    /// </summary>
    internal int NbLocal()
    {
        int nb;
        if (theents[3] is not null)
        {
            nb = 4;
        }
        else if (theents[2] is not null)
        {
            nb = 3;
        }
        else if (theents[1] is not null)
        {
            nb = 2;
        }
        else if (theents[0] is not null)
        {
            nb = 1;
        }
        else
        {
            nb = 0;
        }
        return nb;
    }

    /// <summary>
    /// Returns True if a Cluster has a Next
    /// </summary>
    internal bool HasNext()
    {
        return (thenext is not null);
    }

    /// <summary>
    /// Returns Next Cluster in the chain
    /// </summary>
    internal Interface_EntityCluster? Next()
    {
        return thenext;
    }


    private readonly Transient?[] theents = new Transient?[4];
    private Interface_EntityCluster? thenext;
}
