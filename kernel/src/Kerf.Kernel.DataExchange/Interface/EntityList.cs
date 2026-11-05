// Port of: src/DataExchange/TKXSBase/Interface/Interface_EntityList.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_EntityList.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// This class defines a list of Entities (Transient Objects), it can be used as a field of other Transient
/// classes, with these features : - oriented to define a little list, that is, slower than an Array or a Map
/// of Entities for a big count (about 100 and over), but faster than a Sequence - allows to work as a
/// Sequence, limited to Clear, Append, Remove, Access to an Item identified by its rank in the list - space
/// saving, compared to a Sequence, especially for little amounts; better than an Array for a very little
/// amount (less than 10) but less good for a greater amount Works in conjunction with EntityCluster An
/// EntityList gives access to a list of Entity Clusters, which are chained (in one sense : Single List)
/// Remark : a new Item may not be Null, because this is the criterium used for "End of List"
/// </summary>
internal class EntityList
{

    /// <summary>
    /// Creates a List as being empty
    /// </summary>
    public EntityList()
    {
    }

    /// <summary>
    /// Clears the List
    /// </summary>
    public void Clear()
    {
        _theval = null;
    }

    /// <summary>
    /// Appends an Entity, that is to the END of the list (keeps order, but works slowerly than Add, see below)
    /// </summary>
    public void Append(Transient? ent)
    {
        if (ent is null)
        {
            throw new NullObjectException("Interface_EntityList Append");
        }
        if (_theval is null)
        {
            _theval = ent;
            return;
        }
        EntityCluster valEC = (_theval as EntityCluster);
        if (valEC is not null)
        {
            valEC.Append(ent); // EntityCluster
        }
        else
        { // reste InterfaceEntity ...
            EntityCluster ec = new EntityCluster(_theval);
            ec.Append(ent);
            _theval = ec;
        }
    }

    /// <summary>
    /// Adds an Entity to the list, that is, with NO REGARD about the order (faster than Append if count becomes
    /// greater than 10)
    /// </summary>
    public void Add(Transient? ent)
    {
        if (ent is null)
        {
            throw new NullObjectException("Interface_EntityList Add");
        }
        if (_theval is null)
        {
            _theval = ent;
            return;
        }
        EntityCluster valEC = (_theval as EntityCluster);
        if (valEC is not null)
        { // EntityCluster
            if (valEC.IsLocalFull)
            {
                _theval = new EntityCluster(ent, valEC);
            }
            else
            {
                valEC.Append(ent);
            }
        }
        else
        { // reste InterfaceEntity ...
            EntityCluster ec = new EntityCluster(_theval);
            ec.Append(ent);
            _theval = ec;
        }
    }

    /// <summary>
    /// Removes an Entity from the list, if it is there
    /// </summary>
    public void Remove(Transient? ent)
    {
        if (ent is null)
        {
            throw new NullObjectException("Interface_EntityList Remove");
        }
        if (_theval is null)
        {
            return;
        }
        if (_theval == ent)
        {
            _theval = null;
            return;
        }
        EntityCluster ec = (_theval as EntityCluster);
        if (ec is null)
        {
            return; // A single Entity and not the right one
        }
        bool res = ec.Remove(ent);
        if (res)
        {
            _theval = null;
        }
    }

    /// <summary>
    /// Removes an Entity from the list, given its rank
    /// </summary>
    public void Remove(int num)
    {
        if (_theval is null)
        {
            throw new OutOfRangeException("EntityList : Remove");
        }
        EntityCluster ec = (_theval as EntityCluster);
        if (ec is null)
        {
            if (num != 1)
            {
                throw new OutOfRangeException("EntityList : Remove");
            }
            _theval = null;
            return;
        }
        bool res = ec.Remove(num);
        if (res)
        {
            _theval = null;
        }
    }

    /// <summary>
    /// Returns True if the list is empty
    /// </summary>
    public bool IsEmpty
    {
        get
        {
            return (_theval is null);
        }
    }

    /// <summary>
    /// Returns count of recorded Entities
    /// </summary>
    public int NbEntities()
    {
        if (_theval is null)
        {
            return 0;
        }
        EntityCluster ec = (_theval as EntityCluster);
        if (ec is null)
        {
            return 1; // A single Entity
        }
        return ec.NbEntities();
    }

    /// <summary>
    /// Returns an Item given its number. Beware about the way the list was filled (see above, Add and Append)
    /// </summary>
    public Transient? Value(int num)
    {
        if (_theval is null)
        {
            throw new OutOfRangeException("Interface EntityList : Value");
        }
        EntityCluster ec = (_theval as EntityCluster);
        if (ec is not null)
        {
            return ec.Value(num); // EntityCluster
        }
        else if (num != 1)
        {
            throw new OutOfRangeException("Interface EntityList : Value");
        }
        return _theval;
    }

    /// <summary>
    /// Returns an Item given its number. Beware about the way the list was filled (see above, Add and Append)
    /// </summary>
    public void SetValue(int num, Transient? ent)
    {
        if (ent is null)
        {
            throw new NullObjectException("Interface_EntityList SetValue");
        }
        if (_theval is null)
        {
            throw new OutOfRangeException("Interface EntityList : SetValue");
        }
        EntityCluster ec = (_theval as EntityCluster);
        if (ec is not null)
        {
            ec.SetValue(num, ent); // EntityCluster
        }
        else if (num != 1)
        {
            throw new OutOfRangeException("Interface EntityList : SetValue");
        }
        else
        {
            _theval = ent;
        }
    }

    /// <summary>
    /// fills an Iterator with the content of the list (normal way to consult a list which has been filled with
    /// Add)
    /// </summary>
    public void FillIterator(EntityIterator iter)
    {
        if (_theval is null)
        {
            return;
        }
        EntityCluster ec = (_theval as EntityCluster);
        if (ec is not null)
        {
            ec.FillIterator(iter); // EntityCluster;
        }
        else
        {
            iter.GetOneItem(_theval);
        }
    }

    /// <summary>
    /// Returns count of Entities of a given Type (0 : none)
    /// </summary>
    public int NbTypedEntities(Type? atype)
    {
        int res = 0;
        if (_theval is null)
        {
            return 0;
        }
        EntityCluster ec = (_theval as EntityCluster);
        if (ec is not null)
        { // EntityCluster
            while (ec is not null)
            {
                for (int i = ec.NbLocal(); i > 0; i--)
                {
                    if (ec.Value(i).IsKind(atype))
                    {
                        res++;
                    }
                }
                if (!ec.HasNext)
                {
                    break;
                }
                ec = ec.Next();
            }
        }
        else
        { // Une seule Entite
            if (_theval.IsKind(atype))
            {
                res = 1;
            }
        }
        return res;
    }

    /// <summary>
    /// Returns the Entity which is of a given type. If num = 0 (D), there must be ONE AND ONLY ONE If num &gt;
    /// 0, returns the num-th entity of this type
    /// </summary>
    public Transient? TypedEntity(Type? atype, int num = 0)
    {
        int res = 0;
        Transient entres = null;
        if (_theval is null)
        {
            throw new InterfaceErrorException("Interface EntityList : TypedEntity , none found");
        }
        EntityCluster ec = (_theval as EntityCluster);
        if (ec is not null)
        { // EntityCluster
            while (ec is not null)
            {
                for (int i = ec.NbLocal(); i > 0; i--)
                {
                    if (ec.Value(i).IsKind(atype))
                    {
                        res++;
                        if (num == 0 && res > 1)
                        {
                            throw new InterfaceErrorException("Interface EntityList : TypedEntity , several found");
                        }
                        entres = ec.Value(i);
                        if (res == num)
                        {
                            return entres;
                        }
                    }
                }
                if (!ec.HasNext)
                {
                    break;
                }
                ec = ec.Next();
            }
        }
        else if (num > 1)
        {
            throw new InterfaceErrorException("Interface EntityList : TypedEntity ,out of range");
        }
        else
        { // InterfaceEntity
            if (!_theval.IsKind(atype))
            {
                throw new InterfaceErrorException("Interface EntityList : TypedEntity , none found");
            }
            entres = _theval;
        }
        return entres;
    }

    private Transient? _theval;
}
