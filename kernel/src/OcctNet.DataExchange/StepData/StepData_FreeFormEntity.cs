// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FreeFormEntity.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FreeFormEntity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// A Free Form Entity allows to record any kind of STEP parameters, in any way of typing. It is implemented with an
/// array of fields. A Complex entity can be defined, as a chain of FreeFormEntity (see Next and As).
/// </summary>
public class StepData_FreeFormEntity : Standard_Transient
{
    private string thetype = "";
    private NCollection_HArray1<StepData_Field> thefields;
    private StepData_FreeFormEntity thenext;

    /// <summary>Creates a completely empty FreeFormEntity.</summary>
    public StepData_FreeFormEntity()
    {
    }

    /// <summary>Sets the type of an entity. For a complex one, the type of this member.</summary>
    public void SetStepType(string typenam)
    {
        // Set the STEP entity type name for this free-form entity
        thetype = typenam ?? "";
    }

    /// <summary>Returns the recorded StepType. For a complex one, the type of this member.</summary>
    public string StepType()
    {
        return thetype;
    }

    /// <summary>
    /// Sets a next member, in order to define or complete a Complex entity. If last is True (D), this next will be set
    /// as last of given chain. Else, it will be inserted between this and its formerly recorded next.
    /// </summary>
    public void SetNext(StepData_FreeFormEntity? next, bool last = true)
    {
        if (next is null)
        {
            thenext = null;
        }
        else if (thenext is null)
        {
            thenext = next;
        }
        else if (last)
        {
            thenext.SetNext(next);
        }
        else
        {
            next.SetNext(thenext, last);
            thenext = next;
        }
    }

    /// <summary>Returns the next member of a Complex entity (remark: the last member has none).</summary>
    public StepData_FreeFormEntity? Next()
    {
        return thenext;
    }

    /// <summary>Returns True if a FreeFormEntity is Complex (i.e. has Next).</summary>
    public bool IsComplex()
    {
        // A complex entity is one that has additional entity parts linked via 'next'
        return (thenext is not null);
    }

    /// <summary>Returns the member of a FreeFormEntity of which the type name is given (exact match, no sub-type).</summary>
    public StepData_FreeFormEntity? Typed(string typenam)
    {
        StepData_FreeFormEntity res = null;
        if (string.Equals(thetype, typenam, StringComparison.Ordinal))
        {
            return this;
        }
        if (thenext is null)
        {
            return res;
        }
        return thenext.Typed(typenam);
    }

    /// <summary>Returns the list of types (one type for a simple entity), as is (non-alphabetic sort).</summary>
    public NCollection_HSequence<string> TypeList()
    {
        NCollection_HSequence<string> li = new NCollection_HSequence<string>();
        li.Append(thetype);
        StepData_FreeFormEntity next = thenext;
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
    /// it in alphabetic order, as OCCT's comment intends (decision 12504). No OCCT code calls it.
    /// </remarks>
    public static bool Reorder(ref StepData_FreeFormEntity? ent)
    {
        // Reorder complex entities to ensure alphabetical sorting of entity types
        if (ent is null)
        {
            return false;
        }
        if (!ent.IsComplex())
        {
            return false;
        }
        bool afr = false; // flag: any reordering needed
        StepData_FreeFormEntity e1 = ent;
        StepData_FreeFormEntity e2 = ent.Next();
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
        SortedDictionary<string, Standard_Transient> dic = new(StringComparer.Ordinal);
        while (e1 is not null)
        {
            dic[e1.StepType()] = e1;
            e1 = e1.Next();
        }
        //  First clear the current 'next' links to break the chain...
        foreach (KeyValuePair<string, Standard_Transient> iter in dic)
        {
            e1 = iter.Value as StepData_FreeFormEntity;
            if (e1 is not null)
            {
                e1.SetNext(e2);
            }
        }
        //  ... then rebuild the chain in alphabetical order
        e1 = null;
        foreach (KeyValuePair<string, Standard_Transient> iter in dic)
        {
            e2 = iter.Value as StepData_FreeFormEntity;
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
            thefields = null;
        }
        else
        {
            thefields = StepData_FieldListN.NewFields(1, nb);
        }
    }

    /// <summary>Returns the count of fields.</summary>
    public int NbFields()
    {
        return (thefields is null ? 0 : thefields.Length());
    }

    /// <summary>Returns a field from its rank, for read-only use.</summary>
    public StepData_Field Field(int num)
    {
        return thefields.Value(num);
    }

    /// <summary>Returns a field from its rank, in order to modify it.</summary>
    public StepData_Field CField(int num)
    {
        return thefields.ChangeValue(num);
    }
}
