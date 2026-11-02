// Port of: src/DataExchange/TKXSBase/Interface/Interface_EntityList.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_EntityList.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

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
public class Interface_EntityList
{

    /// <summary>
    /// Creates a List as being empty
    /// </summary>
    public Interface_EntityList()
    {
    }

    /// <summary>
    /// Clears the List
    /// </summary>
    public void Clear()
    {
        theval = null;
    }

    /// <summary>
    /// Appends an Entity, that is to the END of the list (keeps order, but works slowerly than Add, see below)
    /// </summary>
    public void Append(Standard_Transient? ent)
    {
        if (ent is null)
        {
            throw new Standard_NullObject("Interface_EntityList Append");
        }
        if (theval is null)
        {
            theval = ent;
            return;
        }
        Interface_EntityCluster aValEC = (theval as Interface_EntityCluster);
        if (aValEC is not null)
        {
            aValEC.Append(ent); // EntityCluster
        }
        else
        { // reste InterfaceEntity ...
            Interface_EntityCluster ec = new Interface_EntityCluster(theval);
            ec.Append(ent);
            theval = ec;
        }
    }

    /// <summary>
    /// Adds an Entity to the list, that is, with NO REGARD about the order (faster than Append if count becomes
    /// greater than 10)
    /// </summary>
    public void Add(Standard_Transient? ent)
    {
        if (ent is null)
        {
            throw new Standard_NullObject("Interface_EntityList Add");
        }
        if (theval is null)
        {
            theval = ent;
            return;
        }
        Interface_EntityCluster aValEC = (theval as Interface_EntityCluster);
        if (aValEC is not null)
        { // EntityCluster
            if (aValEC.IsLocalFull())
            {
                theval = new Interface_EntityCluster(ent, aValEC);
            }
            else
            {
                aValEC.Append(ent);
            }
        }
        else
        { // reste InterfaceEntity ...
            Interface_EntityCluster ec = new Interface_EntityCluster(theval);
            ec.Append(ent);
            theval = ec;
        }
    }

    /// <summary>
    /// Removes an Entity from the list, if it is there
    /// </summary>
    public void Remove(Standard_Transient? ent)
    {
        if (ent is null)
        {
            throw new Standard_NullObject("Interface_EntityList Remove");
        }
        if (theval is null)
        {
            return;
        }
        if (theval == ent)
        {
            theval = null;
            return;
        }
        Interface_EntityCluster ec = (theval as Interface_EntityCluster);
        if (ec is null)
        {
            return; // A single Entity and not the right one
        }
        bool res = ec.Remove(ent);
        if (res)
        {
            theval = null;
        }
    }

    /// <summary>
    /// Removes an Entity from the list, given its rank
    /// </summary>
    public void Remove(int num)
    {
        if (theval is null)
        {
            throw new Standard_OutOfRange("EntityList : Remove");
        }
        Interface_EntityCluster ec = (theval as Interface_EntityCluster);
        if (ec is null)
        {
            if (num != 1)
            {
                throw new Standard_OutOfRange("EntityList : Remove");
            }
            theval = null;
            return;
        }
        bool res = ec.Remove(num);
        if (res)
        {
            theval = null;
        }
    }

    /// <summary>
    /// Returns True if the list is empty
    /// </summary>
    public bool IsEmpty()
    {
        return (theval is null);
    }

    /// <summary>
    /// Returns count of recorded Entities
    /// </summary>
    public int NbEntities()
    {
        if (theval is null)
        {
            return 0;
        }
        Interface_EntityCluster ec = (theval as Interface_EntityCluster);
        if (ec is null)
        {
            return 1; // A single Entity
        }
        return ec.NbEntities();
    }

    /// <summary>
    /// Returns an Item given its number. Beware about the way the list was filled (see above, Add and Append)
    /// </summary>
    public Standard_Transient? Value(int num)
    {
        if (theval is null)
        {
            throw new Standard_OutOfRange("Interface EntityList : Value");
        }
        Interface_EntityCluster ec = (theval as Interface_EntityCluster);
        if (ec is not null)
        {
            return ec.Value(num); // EntityCluster
        }
        else if (num != 1)
        {
            throw new Standard_OutOfRange("Interface EntityList : Value");
        }
        return theval;
    }

    /// <summary>
    /// Returns an Item given its number. Beware about the way the list was filled (see above, Add and Append)
    /// </summary>
    public void SetValue(int num, Standard_Transient? ent)
    {
        if (ent is null)
        {
            throw new Standard_NullObject("Interface_EntityList SetValue");
        }
        if (theval is null)
        {
            throw new Standard_OutOfRange("Interface EntityList : SetValue");
        }
        Interface_EntityCluster ec = (theval as Interface_EntityCluster);
        if (ec is not null)
        {
            ec.SetValue(num, ent); // EntityCluster
        }
        else if (num != 1)
        {
            throw new Standard_OutOfRange("Interface EntityList : SetValue");
        }
        else
        {
            theval = ent;
        }
    }

    /// <summary>
    /// fills an Iterator with the content of the list (normal way to consult a list which has been filled with
    /// Add)
    /// </summary>
    public void FillIterator(Interface_EntityIterator iter)
    {
        if (theval is null)
        {
            return;
        }
        Interface_EntityCluster ec = (theval as Interface_EntityCluster);
        if (ec is not null)
        {
            ec.FillIterator(iter); // EntityCluster;
        }
        else
        {
            iter.GetOneItem(theval);
        }
    }

    /// <summary>
    /// Returns count of Entities of a given Type (0 : none)
    /// </summary>
    public int NbTypedEntities(Type? atype)
    {
        int res = 0;
        if (theval is null)
        {
            return 0;
        }
        Interface_EntityCluster ec = (theval as Interface_EntityCluster);
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
                if (!ec.HasNext())
                {
                    break;
                }
                ec = ec.Next();
            }
        }
        else
        { // Une seule Entite
            if (theval.IsKind(atype))
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
    public Standard_Transient? TypedEntity(Type? atype, int num = 0)
    {
        int res = 0;
        Standard_Transient entres = null;
        if (theval is null)
        {
            throw new Interface_InterfaceError("Interface EntityList : TypedEntity , none found");
        }
        Interface_EntityCluster ec = (theval as Interface_EntityCluster);
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
                            throw new Interface_InterfaceError("Interface EntityList : TypedEntity , several found");
                        }
                        entres = ec.Value(i);
                        if (res == num)
                        {
                            return entres;
                        }
                    }
                }
                if (!ec.HasNext())
                {
                    break;
                }
                ec = ec.Next();
            }
        }
        else if (num > 1)
        {
            throw new Interface_InterfaceError("Interface EntityList : TypedEntity ,out of range");
        }
        else
        { // InterfaceEntity
            if (!theval.IsKind(atype))
            {
                throw new Interface_InterfaceError("Interface EntityList : TypedEntity , none found");
            }
            entres = theval;
        }
        return entres;
    }

    private Standard_Transient? theval;
}
