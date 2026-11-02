// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_FileSchema.hxx
// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_FileSchema.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>The FILE_SCHEMA entity of a STEP file's header.</summary>
public class HeaderSection_FileSchema : Standard_Transient
{
    private NCollection_HArray1<TCollection_HAsciiString> schemaIdentifiers;

    /// <summary>Returns a FileSchema.</summary>
    public HeaderSection_FileSchema()
    {
    }

    public void Init(NCollection_HArray1<TCollection_HAsciiString>? aSchemaIdentifiers)
    {
        // --- class own fields ---
        schemaIdentifiers = aSchemaIdentifiers;
    }

    public void SetSchemaIdentifiers(NCollection_HArray1<TCollection_HAsciiString>? aSchemaIdentifiers)
    {
        schemaIdentifiers = aSchemaIdentifiers;
    }

    public NCollection_HArray1<TCollection_HAsciiString>? SchemaIdentifiers()
    {
        return schemaIdentifiers;
    }

    public TCollection_HAsciiString? SchemaIdentifiersValue(int num)
    {
        return schemaIdentifiers.Value(num);
    }

    public int NbSchemaIdentifiers()
    {
        if (schemaIdentifiers is null)
        {
            return 0;
        }
        return schemaIdentifiers.Length();
    }
}
