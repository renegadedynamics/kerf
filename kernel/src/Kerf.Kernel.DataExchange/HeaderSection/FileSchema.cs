// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_FileSchema.hxx
// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_FileSchema.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Step.Header;

/// <summary>The FILE_SCHEMA entity of a STEP file's header.</summary>
internal class FileSchema : Transient
{
    private NCollection_HArray1<TCollection_HAsciiString> _schemaIdentifiers;

    /// <summary>Returns a FileSchema.</summary>
    public FileSchema()
    {
    }

    public void Init(NCollection_HArray1<TCollection_HAsciiString>? aSchemaIdentifiers)
    {
        // --- class own fields ---
        _schemaIdentifiers = aSchemaIdentifiers;
    }

    public void SetSchemaIdentifiers(NCollection_HArray1<TCollection_HAsciiString>? aSchemaIdentifiers)
    {
        _schemaIdentifiers = aSchemaIdentifiers;
    }

    public NCollection_HArray1<TCollection_HAsciiString>? SchemaIdentifiers()
    {
        return _schemaIdentifiers;
    }

    public TCollection_HAsciiString? SchemaIdentifiersValue(int num)
    {
        return _schemaIdentifiers.Value(num);
    }

    public int NbSchemaIdentifiers()
    {
        if (_schemaIdentifiers is null)
        {
            return 0;
        }
        return _schemaIdentifiers.Length();
    }
}
