// Port of: src/DataExchange/TKXSBase/Interface/Interface_FileParameter.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_FileParameter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Exchange.Framework.ParamType;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// Stores a literal parameter of a file, in a file's intermediate directory or in an undefined content: its text,
/// its type, and for a reference, the number of the record it designates.
/// </summary>
/// <remarks>
/// A value in C++, stored in <see cref="ParamList"/>s and <see cref="ParamSet"/>s, which own the
/// characters (the parameter keeps a pointer into their buffer). Here the text is a string, and the lists copy a
/// parameter where C++ copies a value (<see cref="ParamList.SetValue"/>), so a parameter returned by
/// <c>ChangeValue</c>/<c>ChangeParam</c> is the stored one, changed in place as in C++.
/// </remarks>
public sealed class FileParameter
{
    /// <summary>An empty parameter (type Misc, no text, entity number 0).</summary>
    public FileParameter()
    {
        _thetype = global::Kerf.Engine.Exchange.Framework.ParamType.ParamMisc;
        _thenum = 0;
    }

    /// <summary>A copy of another parameter (C++'s copy of the value).</summary>
    public FileParameter(FileParameter other)
    {
        _thetype = other._thetype;
        _theval = other._theval;
        _thenum = other._thenum;
    }

    /// <summary>Fills the fields (with the entity number set to zero).</summary>
    /// <remarks>
    /// OCCT has two overloads, one copying a <c>TCollection_AsciiString</c>, one keeping a pointer to characters
    /// managed by a ParamSet; both are this one with strings.
    /// </remarks>
    public void Init(string? val, ParamType typ)
    {
        _theval = val;
        _thetype = typ;
        _thenum = 0;
    }

    /// <summary>The value, as a string (null once cleared, or never set).</summary>
    public string? CValue() => _theval;

    /// <summary>The type of the parameter.</summary>
    public ParamType ParamType() => _thetype;

    /// <summary>Sets a reference to an entity in a numbered list.</summary>
    public void SetEntityNumber(int num)
    {
        _thenum = num;
    }

    /// <summary>The value set by <see cref="SetEntityNumber"/>.</summary>
    public int EntityNumber() => _thenum;

    /// <summary>Clears the stored text.</summary>
    public void Clear()
    {
        _theval = null;
    } // delete theval; not if managed by ParamSet

    /// <summary>Does nothing (the memory is managed by the ParamSet).</summary>
    public void Destroy()
    {
    }

    private ParamType _thetype;
    private string? _theval;
    private int _thenum;
}
