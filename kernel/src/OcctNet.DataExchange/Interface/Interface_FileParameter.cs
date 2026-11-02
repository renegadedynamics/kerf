// Port of: src/DataExchange/TKXSBase/Interface/Interface_FileParameter.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_FileParameter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Interface_ParamType;

namespace Occt;

/// <summary>
/// Stores a literal parameter of a file, in a file's intermediate directory or in an undefined content: its text,
/// its type, and for a reference, the number of the record it designates.
/// </summary>
/// <remarks>
/// A value in C++, stored in <see cref="Interface_ParamList"/>s and <see cref="Interface_ParamSet"/>s, which own the
/// characters (the parameter keeps a pointer into their buffer). Here the text is a string, and the lists copy a
/// parameter where C++ copies a value (<see cref="Interface_ParamList.SetValue"/>), so a parameter returned by
/// <c>ChangeValue</c>/<c>ChangeParam</c> is the stored one, changed in place as in C++.
/// </remarks>
public sealed class Interface_FileParameter
{
    /// <summary>An empty parameter (type Misc, no text, entity number 0).</summary>
    public Interface_FileParameter()
    {
        thetype = Interface_ParamMisc;
        thenum = 0;
    }

    /// <summary>A copy of another parameter (C++'s copy of the value).</summary>
    public Interface_FileParameter(Interface_FileParameter theOther)
    {
        thetype = theOther.thetype;
        theval = theOther.theval;
        thenum = theOther.thenum;
    }

    /// <summary>Fills the fields (with the entity number set to zero).</summary>
    /// <remarks>
    /// OCCT has two overloads, one copying a <c>TCollection_AsciiString</c>, one keeping a pointer to characters
    /// managed by a ParamSet; both are this one with strings.
    /// </remarks>
    public void Init(string? val, Interface_ParamType typ)
    {
        theval = val;
        thetype = typ;
        thenum = 0;
    }

    /// <summary>The value, as a string (null once cleared, or never set).</summary>
    public string? CValue() => theval;

    /// <summary>The type of the parameter.</summary>
    public Interface_ParamType ParamType() => thetype;

    /// <summary>Sets a reference to an entity in a numbered list.</summary>
    public void SetEntityNumber(int num)
    {
        thenum = num;
    }

    /// <summary>The value set by <see cref="SetEntityNumber"/>.</summary>
    public int EntityNumber() => thenum;

    /// <summary>Clears the stored text.</summary>
    public void Clear()
    {
        theval = null;
    } // delete theval; not if managed by ParamSet

    /// <summary>Does nothing (the memory is managed by the ParamSet).</summary>
    public void Destroy()
    {
    }

    private Interface_ParamType thetype;
    private string? theval;
    private int thenum;
}
