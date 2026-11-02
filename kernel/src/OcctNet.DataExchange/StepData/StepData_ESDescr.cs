// Port of: src/DataExchange/TKDESTEP/StepData/StepData_ESDescr.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_ESDescr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>This class is intended to describe the authorized form for a Simple (not Plex) Entity, as a list of fields.</summary>
public class StepData_ESDescr : StepData_EDescr
{
    private readonly string thenom;
    private NCollection_HArray1<Standard_Transient> thedescr;
    private readonly NCollection_DataMap<string, int> thenames = new();
    private StepData_ESDescr thebase;
    private StepData_ESDescr thesuper;

    /// <summary>Creates an ESDescr with a type name.</summary>
    public StepData_ESDescr(string name)
    {
        thenom = name ?? "";
        // Constructor for Simple Entity Descriptor with the given type name
    }

    /// <summary>Sets a new count of fields. Each one is described by a PDescr.</summary>
    public void SetNbFields(int nb)
    {
        // Set the number of fields for this entity descriptor, preserving existing field data
        int minb, i, oldnb = NbFields();
        thenames.Clear(); // Clear name-to-index mapping
        if (nb == 0)
        {
            thedescr = null;
            return;
        }
        NCollection_HArray1<Standard_Transient> li = new NCollection_HArray1<Standard_Transient>(1, nb);
        if (oldnb == 0)
        {
            thedescr = li;
            return;
        }
        // Copy existing field descriptors up to the minimum of old and new sizes
        minb = (oldnb > nb ? nb : oldnb);
        for (i = 1; i <= minb; i++)
        {
            StepData_PDescr pde = thedescr.Value(i) as StepData_PDescr;
            if (pde is not null)
            {
                thenames.Bind(pde.Name(), i); // Rebuild name-to-index mapping
            }
            li.SetValue(i, pde);
        }
        thedescr = li;
    }

    /// <summary>Sets a PDescr to describe a field. A Field is designated by its rank and name.</summary>
    public void SetField(int num, string name, StepData_PDescr? descr)
    {
        // Set field descriptor at specified position with given name and parameter descriptor
        if (num < 1 || num > NbFields())
        {
            return;
        }
        StepData_PDescr pde = new StepData_PDescr();
        pde.SetFrom(descr); // Copy descriptor properties
        pde.SetName(name);  // Set field name
        thedescr.SetValue(num, pde);
        thenames.Bind(name, num); // Update name-to-index mapping
    }

    /// <summary>Sets an ESDescr as based on another one, hence it gets its fields (but they are not copied).</summary>
    public void SetBase(StepData_ESDescr? base_)
    {
        thebase = base_;
        //  Need to ACCUMULATE the fields from the base and its superclasses
    }

    /// <summary>Sets an ESDescr as "super-type". Applies an a base (non derived) ESDescr.</summary>
    public void SetSuper(StepData_ESDescr super_)
    {
        // Set the superclass descriptor, handling inheritance hierarchy
        StepData_ESDescr sup = super_.Base();
        if (sup is null)
        {
            sup = super_;
        }
        if (thebase is not null)
        {
            thebase.SetSuper(sup); // Delegate to base if exists
        }
        else
        {
            thesuper = sup; // Otherwise set directly
        }
    }

    /// <summary>Returns the type name given at creation time.</summary>
    public string TypeName()
    {
        return thenom;
    }

    /// <summary>Returns the type name as an AsciiString.</summary>
    public string StepType()
    {
        return thenom;
    }

    /// <summary>Returns the basic ESDescr, null if none.</summary>
    public StepData_ESDescr? Base()
    {
        return thebase;
    }

    /// <summary>Returns the super-type ESDescr, null if none.</summary>
    public StepData_ESDescr? Super()
    {
        return thesuper;
    }

    /// <summary>Tells if this ESDescr is sub-type of another one.</summary>
    public bool IsSub(StepData_ESDescr other)
    {
        // Check if this descriptor is a subclass of the given descriptor
        StepData_ESDescr oth = other.Base();
        if (oth is null)
        {
            oth = other;
        }
        if (thebase is not null)
        {
            return thebase.IsSub(oth); // Delegate to base if exists
        }
        Standard_Transient t1 = this;
        if (oth == t1)
        {
            return true; // Same descriptor
        }
        if (oth == thesuper)
        {
            return true; // Direct superclass
        }
        else if (thesuper is null)
        {
            return false; // No superclass
        }
        return thesuper.IsSub(oth); // Check recursively up the hierarchy
    }

    /// <summary>Returns the count of fields.</summary>
    public int NbFields()
    {
        return (thedescr is null ? 0 : thedescr.Length());
    }

    /// <summary>Returns the rank of a field from its name. 0 if unknown.</summary>
    public int Rank(string name)
    {
        if (!thenames.Find(name, out int rank))
        {
            return 0;
        }
        return rank;
    }

    /// <summary>Returns the name of a field from its rank. Empty if out of range.</summary>
    public string Name(int num)
    {
        if (num < 1)
        {
            return "";
        }
        if (num > NbFields())
        {
            return "";
        }
        StepData_PDescr pde = thedescr.Value(num) as StepData_PDescr;
        return pde.Name();
    }

    /// <summary>Returns the PDescr for the field num (or null).</summary>
    public StepData_PDescr? Field(int num)
    {
        return thedescr.Value(num) as StepData_PDescr;
    }

    /// <summary>Returns the PDescr for the field named name (or null).</summary>
    public StepData_PDescr? NamedField(string name)
    {
        StepData_PDescr pde = null;
        int rank = Rank(name);
        if (rank > 0)
        {
            pde = thedescr.Value(rank) as StepData_PDescr;
        }
        return pde;
    }

    /// <summary>Tells if a ESDescr matches a step type: exact or super type.</summary>
    public override bool Matches(string name)
    {
        // Check if this descriptor matches the given type name (including inheritance)
        if (string.Equals(thenom, name, StringComparison.Ordinal))
        {
            return true; // Direct match
        }
        if (thesuper is null)
        {
            return false; // No superclass to check
        }
        return thesuper.Matches(name); // Check superclass hierarchy
    }

    /// <summary>Returns False.</summary>
    public override bool IsComplex()
    {
        return false;
    }

    /// <summary>Creates a described entity (i.e. a simple one).</summary>
    public override StepData_Described NewEntity()
    {
        // Create a new simple entity instance based on this descriptor
        StepData_Simple ent = new StepData_Simple(this);
        return ent;
    }
}
