// Port of: src/DataExchange/TKXSBase/Interface/Interface_EntityCluster.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_EntityCluster.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// Auxiliary class for EntityList. An EntityList designates an EntityCluster, which brings itself an fixed
/// maximum count of Entities. If it is full, it gives access to another cluster ("Next"). This class is
/// intended to give a good compromise between access time (faster than a Sequence, good for little count)
/// and memory use (better than a Sequence in any case, overall for little count, better than an Array for a
/// very little count. It is designed for a light management. Remark that a new Item may not be Null, because
/// this is the criterium used for "End of List"
/// </summary>
internal class EntityCluster : Transient
{

    /// <summary>
    /// Creates an empty, non-chained, EntityCluster
    /// </summary>
    public EntityCluster()
    {
    }

    /// <summary>
    /// Creates a non-chained EntityCluster, filled with one Entity
    /// </summary>
    public EntityCluster(Transient? ent)
    {
        _theents[0] = ent;
    }

    /// <summary>
    /// Creates an empty EntityCluster, chained with another one (that is, put BEFORE this other one in the list)
    /// </summary>
    public EntityCluster(EntityCluster? ec)
    {
        _thenext = ec;
    }

    /// <summary>
    /// Creates an EntityCluster, filled with a first Entity, and chained to another EntityCluster (BEFORE it, as
    /// above)
    /// </summary>
    public EntityCluster(Transient? ent, EntityCluster? ec)
    {
        _theents[0] = ent;
        _thenext = ec;
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
        if (_theents[0] is null)
        {
            _theents[0] = ent;
        }
        else if (_theents[1] is null)
        {
            _theents[1] = ent;
        }
        else if (_theents[2] is null)
        {
            _theents[2] = ent;
        }
        else if (_theents[3] is null)
        {
            _theents[3] = ent;
        }
        else
        { // If this one is full ...
            if (_thenext is null)
            {
                _thenext = new EntityCluster(ent);
            }
            else
            {
                EntityCluster curEntClust = _thenext;
                while (curEntClust.HasNext && curEntClust.IsLocalFull)
                {
                    curEntClust = curEntClust._thenext;
                }
                curEntClust.Append(ent);
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
        if (ent == _theents[0])
        {
            i = 1;
        }
        else if (ent == _theents[1])
        {
            i = 2;
        }
        else if (ent == _theents[2])
        {
            i = 3;
        }
        else if (ent == _theents[3])
        {
            i = 4;

            //  Otherwise, go to the next one, which can then become empty .
            //  We remove the empty cluster from the list (in principle it's the last one)
        }
        else
        { // Not found in this one ...
            if (_thenext is null)
            {
                return false;
            }
            bool res = _thenext.Remove(ent); // OCCT: an int in C++
            if (res)
            {
                _thenext = _thenext.Next();
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
            if (_thenext is null)
            {
                throw new OutOfRangeException("EntityCluster : Remove");
            }
            bool res = _thenext.Remove(num - n);
            if (res)
            {
                _thenext = _thenext.Next();
            }
            return false;
        }
        // OCCT: "j--", which runs j down past the start of the array (a write out of bounds in C++, an endless loop
        // in effect) whenever num < n; fixed to "j++", the shift the code means (occt-issues.md, unit 8A1)
        for (int j = num; j < n; j++)
        {
            _theents[j - 1] = _theents[j];
        }
        _theents[3] = null; // We Nullify at the end
        return (n == 1); // Old NbLocal == 1 . becomes null
    }

    /// <summary>
    /// Returns total count of Entities (including Next)
    /// </summary>
    public int NbEntities()
    {
        int nb = NbLocal();
        if (_thenext is not null)
        {
            nb += _thenext.NbEntities();
        }
        return nb;
    }

    /// <summary>
    /// Returns the Entity identified by its rank in the list (including Next)
    /// </summary>
    public Transient? Value(int num)
    {
        int nb = NbLocal(), localNum = num;
        if (num <= 0)
        {
            throw new OutOfRangeException("Interface EntityCluster : Value");
        }
        if (num > nb)
        {
            EntityCluster curEntClust = _thenext;
            localNum -= nb;
            while (localNum > curEntClust.NbLocal())
            {
                if (!curEntClust.HasNext)
                {
                    throw new OutOfRangeException("Interface EntityCluster : Value");
                }
                curEntClust = curEntClust._thenext;
                localNum -= nb; // OCCT: the first cluster's count, not the current one's (equal while clusters are full)
            }
            return curEntClust._theents[localNum - 1];
        }
        return _theents[num - 1]; // numbering from 0
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
        int nb = NbLocal(), localNum = num;
        if (num <= 0)
        {
            throw new OutOfRangeException("Interface EntityCluster : SetValue");
        }
        if (num > nb)
        {
            EntityCluster curEntClust = _thenext;
            localNum -= nb;
            while (localNum > curEntClust.NbLocal())
            {
                if (_thenext is null) // OCCT: tests this cluster's next, not the current one's
                {
                    throw new OutOfRangeException("Interface EntityCluster : SetValue");
                }
                curEntClust = curEntClust._thenext;
                localNum -= nb; // OCCT: the first cluster's count, not the current one's (equal while clusters are full)
            }
            curEntClust._theents[localNum - 1] = ent;
        }
        else
        {
            _theents[num - 1] = ent; // numbering from 0
        }
    }

    /// <summary>
    /// Fills an Iterator with designated Entities (includes Next)
    /// </summary>
    public void FillIterator(EntityIterator iter)
    {
        if (_theents[0] is not null)
        {
            iter.GetOneItem(_theents[0]);
        }
        if (_theents[1] is not null)
        {
            iter.GetOneItem(_theents[1]);
        }
        if (_theents[2] is not null)
        {
            iter.GetOneItem(_theents[2]);
        }
        if (_theents[3] is not null)
        {
            iter.GetOneItem(_theents[3]);
        }
        if (_thenext is not null)
        {
            _thenext.FillIterator(iter);
        }
    }


    /// <summary>
    /// Returns True if all the set of entities local to a Cluster is full. Used by EntityList.
    /// </summary>
    internal bool IsLocalFull
    {
        get
        {
            return (_theents[3] is not null);
        }
    }

    /// <summary>
    /// Returns count of entities in the local set (without Next) Entities can then be read normally by method
    /// Value
    /// </summary>
    internal int NbLocal()
    {
        int nb;
        if (_theents[3] is not null)
        {
            nb = 4;
        }
        else if (_theents[2] is not null)
        {
            nb = 3;
        }
        else if (_theents[1] is not null)
        {
            nb = 2;
        }
        else if (_theents[0] is not null)
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
    internal bool HasNext
    {
        get
        {
            return (_thenext is not null);
        }
    }

    /// <summary>
    /// Returns Next Cluster in the chain
    /// </summary>
    internal EntityCluster? Next()
    {
        return _thenext;
    }


    private readonly Transient?[] _theents = new Transient?[4];
    private EntityCluster? _thenext;
}
