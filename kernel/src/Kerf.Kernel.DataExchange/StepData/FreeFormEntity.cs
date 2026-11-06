// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FreeFormEntity.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FreeFormEntity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// A Free Form Entity allows to record any kind of STEP parameters, in any way of typing. It is implemented with an
/// array of fields. A Complex entity can be defined, as a chain of FreeFormEntity (see Next and As).
/// </summary>
internal class FreeFormEntity : Transient
{
    private string _thetype = "";
    private NCollection_HArray1<Field> _thefields;
    private FreeFormEntity _thenext;

    /// <summary>Creates a completely empty FreeFormEntity.</summary>
    public FreeFormEntity()
    {
    }

    /// <summary>Sets the type of an entity. For a complex one, the type of this member.</summary>
    public void SetStepType(string typenam)
    {
        // Set the STEP entity type name for this free-form entity
        _thetype = typenam ?? "";
    }

    /// <summary>Returns the recorded StepType. For a complex one, the type of this member.</summary>
    public string StepType()
    {
        return _thetype;
    }

    /// <summary>
    /// Sets a next member, in order to define or complete a Complex entity. If last is True (D), this next will be set
    /// as last of given chain. Else, it will be inserted between this and its formerly recorded next.
    /// </summary>
    public void SetNext(FreeFormEntity? next, bool last = true)
    {
        if (next is null)
        {
            _thenext = null;
        }
        else if (_thenext is null)
        {
            _thenext = next;
        }
        else if (last)
        {
            _thenext.SetNext(next);
        }
        else
        {
            next.SetNext(_thenext, last);
            _thenext = next;
        }
    }

    /// <summary>Returns the next member of a Complex entity (remark: the last member has none).</summary>
    public FreeFormEntity? Next()
    {
        return _thenext;
    }

    /// <summary>Returns True if a FreeFormEntity is Complex (i.e. has Next).</summary>
    public bool IsComplex
    {
        get
        {
            // A complex entity is one that has additional entity parts linked via 'next'
            return (_thenext is not null);
        }
    }

    /// <summary>Returns the member of a FreeFormEntity of which the type name is given (exact match, no sub-type).</summary>
    public FreeFormEntity? Typed(string typenam)
    {
        FreeFormEntity res = null;
        if (string.Equals(_thetype, typenam, StringComparison.Ordinal))
        {
            return this;
        }
        if (_thenext is null)
        {
            return res;
        }
        return _thenext.Typed(typenam);
    }

    /// <summary>Returns the list of types (one type for a simple entity), as is (non-alphabetic sort).</summary>
    public NCollection_HSequence<string> TypeList()
    {
        NCollection_HSequence<string> li = new NCollection_HSequence<string>();
        li.Append(_thetype);
        FreeFormEntity next = _thenext;
        while (next is not null)
        {
            li.Append(next.StepType());
            next = next.Next();
        }
        return li;
    }

    /// <summary>
    /// Reorders a Complex entity if required, i.e. if member types are not in alphabetic order. Returns False if
    /// nothing done (order was OK), True if reordered.
    /// </summary>
    /// <remarks>
    /// OCCT rebuilds the chain by iterating a map of the members keyed by type name, in hash order; the port iterates
    /// it in alphabetic order, as OCCT's comment intends (decision 1194). No OCCT code calls it.
    /// </remarks>
    public static bool Reorder(ref FreeFormEntity? ent)
    {
        // Reorder complex entities to ensure alphabetical sorting of entity types
        if (ent is null)
        {
            return false;
        }
        if (!ent.IsComplex)
        {
            return false;
        }
        bool afr = false; // flag: any reordering needed
        FreeFormEntity e1 = ent;
        FreeFormEntity e2 = ent.Next();
        // Check if entities are already in alphabetical order
        while (e2 is not null)
        {
            if (string.CompareOrdinal(e1.StepType(), e2.StepType()) > 0)
            {
                afr = true; // Found out-of-order pair
                break;
            }
            e1 = e2;
            e2 = e1.Next();
        }
        if (!afr)
        {
            return afr;
        }
        //  Reordering using a dictionary (map) to sort entity types alphabetically
        e1 = ent;
        e2 = null;
        SortedDictionary<string, Transient> dic = new(StringComparer.Ordinal);
        while (e1 is not null)
        {
            dic[e1.StepType()] = e1;
            e1 = e1.Next();
        }
        //  First clear the current 'next' links to break the chain...
        foreach (KeyValuePair<string, Transient> iter in dic)
        {
            e1 = iter.Value as FreeFormEntity;
            if (e1 is not null)
            {
                e1.SetNext(e2);
            }
        }
        //  ... then rebuild the chain in alphabetical order
        e1 = null;
        foreach (KeyValuePair<string, Transient> iter in dic)
        {
            e2 = iter.Value as FreeFormEntity;
            if (e1 is not null)
            {
                e1.SetNext(e2);
            }
            e1 = e2;
        }

        ent = e1; // OCCT: the last member of the new chain, not its first
        return afr;
    }

    /// <summary>Sets a count of Fields, from scratch.</summary>
    public void SetNbFields(int nb)
    {
        // Initialize the array of fields for this entity
        if (nb <= 0)
        {
            _thefields = null;
        }
        else
        {
            _thefields = FieldListN.NewFields(1, nb);
        }
    }

    /// <summary>Returns the count of fields.</summary>
    public int FieldCount
    {
        get
        {
            return (_thefields is null ? 0 : _thefields.Length());
        }
    }

    /// <summary>Returns a field from its rank, for read-only use.</summary>
    public Field Field(int num)
    {
        return _thefields.Value(num);
    }

    /// <summary>Returns a field from its rank, in order to modify it.</summary>
    public Field CField(int num)
    {
        return _thefields.ChangeValue(num);
    }
}
