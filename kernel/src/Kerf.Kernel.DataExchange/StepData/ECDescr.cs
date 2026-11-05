// Port of: src/DataExchange/TKDESTEP/StepData/StepData_ECDescr.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_ECDescr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>Describes a Complex Entity (Plex) as a list of Simple ones.</summary>
internal class ECDescr : EDescr
{
    private readonly NCollection_Sequence<Transient> _thelist = new();

    public ECDescr()
    {
    }

    /// <summary>Adds a member. Warning: members are added in alphabetic order.</summary>
    public void Add(ESDescr? member)
    {
        if (member is null)
        {
            return;
        }
        string name = member.TypeName();
        string nam = name;
        for (int i = MemberCount; i > 0; i--)
        {
            ESDescr mem = Member(i);
            if (string.CompareOrdinal(nam, mem.TypeName()) < 0)
            {
                _thelist.InsertBefore(i, member);
                return;
            }
        }
        _thelist.Append(member);
    }

    /// <summary>Returns the count of members.</summary>
    public int MemberCount
    {
        get
        {
            return _thelist.Length();
        }
    }

    /// <summary>Returns a Member from its rank.</summary>
    public ESDescr Member(int num)
    {
        return _thelist.Value(num) as ESDescr;
    }

    /// <summary>Returns the ordered list of types.</summary>
    public NCollection_HSequence<string> TypeList()
    {
        NCollection_HSequence<string> tl = new NCollection_HSequence<string>();
        int i, nb = MemberCount;
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
        int i, nb = MemberCount;
        for (i = 1; i <= nb; i++)
        {
            ESDescr member = Member(i);
            if (member.Matches(name))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>Returns True.</summary>
    public override bool IsComplex
    {
        get
        {
            return true;
        }
    }

    /// <summary>Creates a described entity (i.e. a complex one, made of one simple entity per member).</summary>
    public override Described NewEntity()
    {
        Plex ent = new Plex(this);
        int i, nb = MemberCount;
        for (i = 1; i <= nb; i++)
        {
            ESDescr member = Member(i);
            Simple mem = member.NewEntity() as Simple;
            if (mem is not null)
            {
                ent.Add(mem);
            }
        }
        return ent;
    }
}
