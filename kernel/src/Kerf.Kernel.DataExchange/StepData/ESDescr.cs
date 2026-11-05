// Port of: src/DataExchange/TKDESTEP/StepData/StepData_ESDescr.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_ESDescr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>This class is intended to describe the authorized form for a Simple (not Plex) Entity, as a list of fields.</summary>
internal class ESDescr : EDescr
{
    private readonly string _thenom;
    private NCollection_HArray1<Transient> _thedescr;
    private readonly Dictionary<string, int> _thenames = new();
    private ESDescr _thebase;
    private ESDescr _thesuper;

    /// <summary>Creates an ESDescr with a type name.</summary>
    public ESDescr(string name)
    {
        _thenom = name ?? "";
        // Constructor for Simple Entity Descriptor with the given type name
    }

    /// <summary>Sets a new count of fields. Each one is described by a PDescr.</summary>
    public void SetNbFields(int nb)
    {
        // Set the number of fields for this entity descriptor, preserving existing field data
        int minb, i, oldnb = FieldCount;
        _thenames.Clear(); // Clear name-to-index mapping
        if (nb == 0)
        {
            _thedescr = null;
            return;
        }
        NCollection_HArray1<Transient> li = new NCollection_HArray1<Transient>(1, nb);
        if (oldnb == 0)
        {
            _thedescr = li;
            return;
        }
        // Copy existing field descriptors up to the minimum of old and new sizes
        minb = (oldnb > nb ? nb : oldnb);
        for (i = 1; i <= minb; i++)
        {
            PDescr pde = _thedescr.Value(i) as PDescr;
            if (pde is not null)
            {
                _thenames[pde.Name()] = i; // Rebuild name-to-index mapping
            }
            li.SetValue(i, pde);
        }
        _thedescr = li;
    }

    /// <summary>Sets a PDescr to describe a field. A Field is designated by its rank and name.</summary>
    public void SetField(int num, string name, PDescr? descr)
    {
        // Set field descriptor at specified position with given name and parameter descriptor
        if (num < 1 || num > FieldCount)
        {
            return;
        }
        PDescr pde = new PDescr();
        pde.SetFrom(descr); // Copy descriptor properties
        pde.SetName(name);  // Set field name
        _thedescr.SetValue(num, pde);
        _thenames[name] = num; // Update name-to-index mapping
    }

    /// <summary>Sets an ESDescr as based on another one, hence it gets its fields (but they are not copied).</summary>
    public void SetBase(ESDescr? base_)
    {
        _thebase = base_;
        //  Need to ACCUMULATE the fields from the base and its superclasses
    }

    /// <summary>Sets an ESDescr as "super-type". Applies an a base (non derived) ESDescr.</summary>
    public void SetSuper(ESDescr super_)
    {
        // Set the superclass descriptor, handling inheritance hierarchy
        ESDescr sup = super_.Base();
        if (sup is null)
        {
            sup = super_;
        }
        if (_thebase is not null)
        {
            _thebase.SetSuper(sup); // Delegate to base if exists
        }
        else
        {
            _thesuper = sup; // Otherwise set directly
        }
    }

    /// <summary>Returns the type name given at creation time.</summary>
    public string TypeName()
    {
        return _thenom;
    }

    /// <summary>Returns the type name as an AsciiString.</summary>
    public string StepType()
    {
        return _thenom;
    }

    /// <summary>Returns the basic ESDescr, null if none.</summary>
    public ESDescr? Base()
    {
        return _thebase;
    }

    /// <summary>Returns the super-type ESDescr, null if none.</summary>
    public ESDescr? Super()
    {
        return _thesuper;
    }

    /// <summary>Tells if this ESDescr is sub-type of another one.</summary>
    public bool IsSub(ESDescr other)
    {
        // Check if this descriptor is a subclass of the given descriptor
        ESDescr oth = other.Base();
        if (oth is null)
        {
            oth = other;
        }
        if (_thebase is not null)
        {
            return _thebase.IsSub(oth); // Delegate to base if exists
        }
        Transient t1 = this;
        if (oth == t1)
        {
            return true; // Same descriptor
        }
        if (oth == _thesuper)
        {
            return true; // Direct superclass
        }
        else if (_thesuper is null)
        {
            return false; // No superclass
        }
        return _thesuper.IsSub(oth); // Check recursively up the hierarchy
    }

    /// <summary>Returns the count of fields.</summary>
    public int FieldCount
    {
        get
        {
            return (_thedescr is null ? 0 : _thedescr.Length());
        }
    }

    /// <summary>Returns the rank of a field from its name. 0 if unknown.</summary>
    public int Rank(string name)
    {
        if (!_thenames.TryGetValue(name, out int rank))
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
        if (num > FieldCount)
        {
            return "";
        }
        PDescr pde = _thedescr.Value(num) as PDescr;
        return pde.Name();
    }

    /// <summary>Returns the PDescr for the field num (or null).</summary>
    public PDescr? Field(int num)
    {
        return _thedescr.Value(num) as PDescr;
    }

    /// <summary>Returns the PDescr for the field named name (or null).</summary>
    public PDescr? NamedField(string name)
    {
        PDescr pde = null;
        int rank = Rank(name);
        if (rank > 0)
        {
            pde = _thedescr.Value(rank) as PDescr;
        }
        return pde;
    }

    /// <summary>Tells if a ESDescr matches a step type: exact or super type.</summary>
    public override bool Matches(string name)
    {
        // Check if this descriptor matches the given type name (including inheritance)
        if (string.Equals(_thenom, name, StringComparison.Ordinal))
        {
            return true; // Direct match
        }
        if (_thesuper is null)
        {
            return false; // No superclass to check
        }
        return _thesuper.Matches(name); // Check superclass hierarchy
    }

    /// <summary>Returns False.</summary>
    public override bool IsComplex
    {
        get
        {
            return false;
        }
    }

    /// <summary>Creates a described entity (i.e. a simple one).</summary>
    public override Described NewEntity()
    {
        // Create a new simple entity instance based on this descriptor
        Simple ent = new Simple(this);
        return ent;
    }
}
