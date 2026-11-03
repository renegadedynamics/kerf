// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_FileDescription.hxx
// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_FileDescription.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>The FILE_DESCRIPTION entity of a STEP file's header.</summary>
public class HeaderSection_FileDescription : Transient
{
    private NCollection_HArray1<TCollection_HAsciiString> description;
    private TCollection_HAsciiString implementationLevel;

    /// <summary>Returns a FileDescription.</summary>
    public HeaderSection_FileDescription()
    {
    }

    public void Init(NCollection_HArray1<TCollection_HAsciiString>? aDescription, TCollection_HAsciiString? aImplementationLevel)
    {
        // --- class own fields ---
        description = aDescription;
        implementationLevel = aImplementationLevel;
    }

    public void SetDescription(NCollection_HArray1<TCollection_HAsciiString>? aDescription)
    {
        description = aDescription;
    }

    public NCollection_HArray1<TCollection_HAsciiString>? Description()
    {
        return description;
    }

    public TCollection_HAsciiString? DescriptionValue(int num)
    {
        return description.Value(num);
    }

    public int NbDescription()
    {
        if (description is null)
        {
            return 0;
        }
        return description.Length();
    }

    public void SetImplementationLevel(TCollection_HAsciiString? aImplementationLevel)
    {
        implementationLevel = aImplementationLevel;
    }

    public TCollection_HAsciiString? ImplementationLevel()
    {
        return implementationLevel;
    }
}
