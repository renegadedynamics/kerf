// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_FileDescription.hxx
// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_FileDescription.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Step.Header;

/// <summary>The FILE_DESCRIPTION entity of a STEP file's header.</summary>
internal class FileDescription : Transient
{
    private NCollection_HArray1<TCollection_HAsciiString> _description;
    private TCollection_HAsciiString _implementationLevel;

    /// <summary>Returns a FileDescription.</summary>
    public FileDescription()
    {
    }

    public void Init(NCollection_HArray1<TCollection_HAsciiString>? aDescription, TCollection_HAsciiString? aImplementationLevel)
    {
        // --- class own fields ---
        _description = aDescription;
        _implementationLevel = aImplementationLevel;
    }

    public void SetDescription(NCollection_HArray1<TCollection_HAsciiString>? aDescription)
    {
        _description = aDescription;
    }

    public NCollection_HArray1<TCollection_HAsciiString>? Description()
    {
        return _description;
    }

    public TCollection_HAsciiString? DescriptionValue(int num)
    {
        return _description.Value(num);
    }

    public int NbDescription()
    {
        if (_description is null)
        {
            return 0;
        }
        return _description.Length();
    }

    public void SetImplementationLevel(TCollection_HAsciiString? aImplementationLevel)
    {
        _implementationLevel = aImplementationLevel;
    }

    public TCollection_HAsciiString? ImplementationLevel()
    {
        return _implementationLevel;
    }
}
