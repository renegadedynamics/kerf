// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_RWFileSchema.hxx
// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_RWFileSchema.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;

using Kerf.Engine.Exchange.Step.Data;

namespace Kerf.Engine.Exchange.Step.Header;

/// <summary>Read and Write Module for FileSchema.</summary>
internal class RWFileSchema
{
    public RWFileSchema()
    {
    }

    public void ReadStep(StepReaderData data, int num, FrameworkCheck ach, FileSchema ent)
    {
        // --- Number of Parameter Control ---
        if (!data.CheckNbParams(num, 1, ach, "file_schema has not 1 parameter(s)"))
        {
            return;
        }

        // --- own field : schemaIdentifiers ---
        NCollection_HArray1<TCollection_HAsciiString> schemaIdentifiers = null;
        TCollection_HAsciiString schemaIdentifiersItem = null;
        int nsub1;
        nsub1 = data.SubListNumber(num, 1, false);
        if (nsub1 != 0)
        {
            int nb1 = data.NbParams(nsub1);
            schemaIdentifiers = new NCollection_HArray1<TCollection_HAsciiString>(1, nb1);
            for (int i1 = 1; i1 <= nb1; i1++)
            {
                bool stat1 = data.ReadString(nsub1, i1, "schema_identifiers", ach, ref schemaIdentifiersItem);
                if (stat1)
                {
                    schemaIdentifiers.SetValue(i1, schemaIdentifiersItem);
                }
            }
        }
        else
        {
            ach.AddFail("Parameter #1 (schema_identifiers) is not a LIST");
        }

        //--- Initialisation of the read entity ---
        if (!ach.HasFailed)
        {
            ent.Init(schemaIdentifiers);
        }
    }

    public void WriteStep(StepWriter sw, FileSchema ent)
    {
        // --- own field : schemaIdentifiers ---
        sw.OpenSub();
        for (int i1 = 1; i1 <= ent.NbSchemaIdentifiers(); i1++)
        {
            sw.Send(ent.SchemaIdentifiersValue(i1));
        }
        sw.CloseSub();
    }
}
