// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Simple.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Simple.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>A Simple Entity is defined by a type (which can heve super types) and a list of parameters.</summary>
public class StepData_Simple : StepData_Described
{
    private readonly StepData_FieldListN thefields;

    /// <summary>Creates a Simple Entity.</summary>
    public StepData_Simple(StepData_ESDescr descr) : base(descr)
    {
        thefields = new StepData_FieldListN(descr.NbFields());
    }

    /// <summary>Returns description, as for simple.</summary>
    public StepData_ESDescr ESDescr()
    {
        return Description() as StepData_ESDescr;
    }

    /// <summary>Returns the recorded StepType (TypeName of its ESDescr).</summary>
    public string StepType()
    {
        return ESDescr().TypeName();
    }

    /// <summary>Returns False.</summary>
    public override bool IsComplex()
    {
        return false;
    }

    /// <summary>Tells if a step type is matched by this entity (by inheritance or by complex composition).</summary>
    public override bool Matches(string steptype)
    {
        return ESDescr().Matches(steptype);
    }

    /// <summary>Returns this entity if it matches the step type, else null.</summary>
    public override StepData_Simple? As(string steptype)
    {
        StepData_Simple nulent = null;
        if (Matches(steptype))
        {
            return this;
        }
        return nulent;
    }

    /// <summary>Tells if a Field brings a given name.</summary>
    public override bool HasField(string name)
    {
        int num = ESDescr().Rank(name);
        return (num > 0);
    }

    /// <summary>Returns a Field from its name; read-only.</summary>
    public override StepData_Field Field(string name)
    {
        int num = ESDescr().Rank(name);
        if (num == 0)
        {
            throw new Interface_InterfaceMismatch("StepData_Simple : Field");
        }
        return FieldNum(num);
    }

    /// <summary>Returns a Field from its name; read or write.</summary>
    public override StepData_Field CField(string name)
    {
        int num = ESDescr().Rank(name);
        if (num == 0)
        {
            throw new Interface_InterfaceMismatch("StepData_Simple : Field");
        }
        return CFieldNum(num);
    }

    /// <summary>Returns the count of fields.</summary>
    public int NbFields()
    {
        return thefields.NbFields();
    }

    /// <summary>Returns a field from its rank, for read-only use.</summary>
    public StepData_Field FieldNum(int num)
    {
        return thefields.Field(num);
    }

    /// <summary>Returns a field from its rank, in order to modify it.</summary>
    public StepData_Field CFieldNum(int num)
    {
        return thefields.CField(num);
    }

    /// <summary>Returns the entire field list, read-only.</summary>
    public StepData_FieldListN Fields()
    {
        return thefields;
    }

    /// <summary>Returns the entire field list, read or write.</summary>
    public StepData_FieldListN CFields()
    {
        return thefields;
    }

    /// <summary>Fills a Check by using its Description.</summary>
    public override void Check(Interface_Check ach)
    {
    } // something? see the description

    /// <summary>Fills an EntityIterator with entities shared by this entity.</summary>
    public override void Shared(Interface_EntityIterator list)
    {
        int i, nb = thefields.NbFields();
        for (i = 1; i <= nb; i++)
        {
            StepData_Field fi = thefields.Field(i);
            int j1, j2, l1, l2;
            l1 = l2 = 1;
            if (fi.Arity() >= 1)
            {
                l1 = fi.Length(1);
            }
            if (fi.Arity() > 1)
            {
                l2 = fi.Length(2);
            }
            for (j1 = 1; j1 <= l1; j1++)
            {
                for (j2 = 1; j2 <= l2; j2++)
                {
                    Transient ent = fi.Entity(j1, j2);
                    if (ent is not null)
                    {
                        list.AddItem(ent);
                    }
                }
            }
        }
    }
}
