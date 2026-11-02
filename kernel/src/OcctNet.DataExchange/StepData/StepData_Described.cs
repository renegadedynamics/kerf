// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Described.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Described.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// General frame to describe entities with Description (Simple or Complex).
/// </summary>
public abstract class StepData_Described : Standard_Transient
{
    private readonly StepData_EDescr thedescr;

    /// <summary>Initializes a Described Entity from a Description (i.e. puts it in a field ...).</summary>
    protected StepData_Described(StepData_EDescr? descr)
    {
        thedescr = descr;
    }

    /// <summary>Returns the Description used to define this entity.</summary>
    public StepData_EDescr? Description()
    {
        return thedescr;
    }

    /// <summary>Tells if a described entity is complex.</summary>
    public abstract bool IsComplex();

    /// <summary>Tells if a step type is matched by this entity (by inheritance or by complex composition).</summary>
    public abstract bool Matches(string steptype);

    /// <summary>Returns a Simple Entity which matches with a Type in this entity: for a Simple Entity, this one if it matches; for a Complex Entity, the member which matches; else null.</summary>
    public abstract StepData_Simple? As(string steptype);

    /// <summary>Tells if a Field brings a given name.</summary>
    public abstract bool HasField(string name);

    /// <summary>Returns a Field from its name; read-only.</summary>
    public abstract StepData_Field Field(string name);

    /// <summary>Returns a Field from its name; read or write.</summary>
    public abstract StepData_Field CField(string name);

    /// <summary>Fills a Check by using its Description.</summary>
    public abstract void Check(Interface_Check ach);

    /// <summary>Fills an EntityIterator with entities shared by this entity.</summary>
    public abstract void Shared(Interface_EntityIterator list);
}
