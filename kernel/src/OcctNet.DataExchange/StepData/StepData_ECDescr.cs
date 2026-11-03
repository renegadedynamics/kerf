// Port of: src/DataExchange/TKDESTEP/StepData/StepData_ECDescr.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_ECDescr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>Describes a Complex Entity (Plex) as a list of Simple ones.</summary>
public class StepData_ECDescr : StepData_EDescr
{
    private readonly NCollection_Sequence<Transient> thelist = new();

    public StepData_ECDescr()
    {
    }

    /// <summary>Adds a member. Warning: members are added in alphabetic order.</summary>
    public void Add(StepData_ESDescr? member)
    {
        if (member is null)
        {
            return;
        }
        string name = member.TypeName();
        string nam = name;
        for (int i = NbMembers(); i > 0; i--)
        {
            StepData_ESDescr mem = Member(i);
            if (string.CompareOrdinal(nam, mem.TypeName()) < 0)
            {
                thelist.InsertBefore(i, member);
                return;
            }
        }
        thelist.Append(member);
    }

    /// <summary>Returns the count of members.</summary>
    public int NbMembers()
    {
        return thelist.Length();
    }

    /// <summary>Returns a Member from its rank.</summary>
    public StepData_ESDescr Member(int num)
    {
        return thelist.Value(num) as StepData_ESDescr;
    }

    /// <summary>Returns the ordered list of types.</summary>
    public NCollection_HSequence<string> TypeList()
    {
        NCollection_HSequence<string> tl = new NCollection_HSequence<string>();
        int i, nb = NbMembers();
        for (i = 1; i <= nb; i++)
        {
            string nam = Member(i).TypeName();
            tl.Append(nam);
        }
        return tl;
    }

    /// <summary>Tells if a ESDescr matches a step type: exact or super type.</summary>
    public override bool Matches(string name)
    {
        int i, nb = NbMembers();
        for (i = 1; i <= nb; i++)
        {
            StepData_ESDescr member = Member(i);
            if (member.Matches(name))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>Returns True.</summary>
    public override bool IsComplex()
    {
        return true;
    }

    /// <summary>Creates a described entity (i.e. a complex one, made of one simple entity per member).</summary>
    public override StepData_Described NewEntity()
    {
        StepData_Plex ent = new StepData_Plex(this);
        int i, nb = NbMembers();
        for (i = 1; i <= nb; i++)
        {
            StepData_ESDescr member = Member(i);
            StepData_Simple mem = member.NewEntity() as StepData_Simple;
            if (mem is not null)
            {
                ent.Add(mem);
            }
        }
        return ent;
    }
}
