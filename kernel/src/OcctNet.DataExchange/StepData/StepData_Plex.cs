// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Plex.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Plex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>A Plex (for Complex) Entity is defined as a list of Simple Members ("external mapping"). The types of these members must be in alphabetic order.</summary>
public class StepData_Plex : StepData_Described
{
    private readonly NCollection_Sequence<Transient> themembers = new();

    /// <summary>Creates a Plex (empty). The complete creation is made by the ECDescr itself (see method NewEntity).</summary>
    public StepData_Plex(StepData_ECDescr descr) : base(descr)
    {
    }

    /// <summary>Adds a member to a Plex.</summary>
    public void Add(StepData_Simple member)
    {
        themembers.Append(member);
    }

    /// <summary>Returns the Description as for a Plex.</summary>
    public StepData_ECDescr ECDescr()
    {
        return Description() as StepData_ECDescr;
    }

    /// <summary>Returns True.</summary>
    public override bool IsComplex()
    {
        return true;
    }

    /// <summary>Tells if a step type is matched by this entity, i.e. by one of its members.</summary>
    public override bool Matches(string steptype)
    {
        int i, nb = NbMembers();
        for (i = 1; i <= nb; i++)
        {
            if (Member(i).Matches(steptype))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>Returns the member which matches the step type, else null.</summary>
    public override StepData_Simple? As(string steptype)
    {
        StepData_Simple ent;
        int i, nb = NbMembers();
        for (i = 1; i <= nb; i++)
        {
            ent = Member(i);
            if (ent.Matches(steptype))
            {
                return ent;
            }
        }
        ent = null;
        return ent;
    }

    /// <summary>Tells if a Field brings a given name.</summary>
    public override bool HasField(string name)
    {
        int i, nb = NbMembers();
        for (i = 1; i <= nb; i++)
        {
            if (Member(i).HasField(name))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>Returns a Field from its name; read-only.</summary>
    public override StepData_Field Field(string name)
    {
        StepData_Simple ent;
        int i, nb = NbMembers();
        for (i = 1; i <= nb; i++)
        {
            ent = Member(i);
            if (ent.HasField(name))
            {
                return ent.Field(name);
            }
        }
        throw new Interface_InterfaceMismatch("StepData_Plex : Field");
    }

    /// <summary>Returns a Field from its name; read or write.</summary>
    public override StepData_Field CField(string name)
    {
        StepData_Simple ent;
        int i, nb = NbMembers();
        for (i = 1; i <= nb; i++)
        {
            ent = Member(i);
            if (ent.HasField(name))
            {
                return ent.CField(name);
            }
        }
        throw new Interface_InterfaceMismatch("StepData_Plex : Field");
    }

    /// <summary>Returns the count of simple members.</summary>
    public int NbMembers()
    {
        return themembers.Length();
    }

    /// <summary>Returns a simple member from its rank.</summary>
    public StepData_Simple Member(int num)
    {
        return themembers.Value(num) as StepData_Simple;
    }

    /// <summary>Returns the actual list of members types.</summary>
    public NCollection_HSequence<string> TypeList()
    {
        NCollection_HSequence<string> tl = new NCollection_HSequence<string>();
        int i, nb = NbMembers();
        for (i = 1; i <= nb; i++)
        {
            string nam = Member(i).StepType();
            tl.Append(nam);
        }
        return tl;
    }

    /// <summary>Fills a Check by using its Description.</summary>
    public override void Check(Interface_Check ach)
    {
        int i, nb = NbMembers();
        for (i = 1; i <= nb; i++)
        {
            StepData_Simple si = Member(i);
            si.Check(ach);
        }
    }

    /// <summary>Fills an EntityIterator with entities shared by this entity.</summary>
    public override void Shared(Interface_EntityIterator list)
    {
        int i, nb = NbMembers();
        for (i = 1; i <= nb; i++)
        {
            StepData_Simple si = Member(i);
            si.Shared(list);
        }
    }
}
