// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_RWFileDescription.hxx
// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_RWFileDescription.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;

using Kerf.Engine.Exchange.Step.Data;

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Step.Header;

/// <summary>Read and Write Module for FileDescription.</summary>
internal class RWFileDescription
{
    public RWFileDescription()
    {
    }

    public void ReadStep(StepReaderData data, int num, FrameworkCheck ach, FileDescription ent)
    {
        // --- Number of Parameter Control ---
        if (!data.CheckNbParams(num, 2, ach, "file_description has not 2 parameter(s)"))
        {
            return;
        }

        // --- own field : description ---
        NCollection_HArray1<TCollection_HAsciiString> description = null;
        TCollection_HAsciiString descriptionItem = null;
        int nsub1 = data.SubListNumber(num, 1, false);
        if (nsub1 != 0)
        {
            int nb1 = data.NbParams(nsub1);
            if (nb1 > 0)
            {
                description = new NCollection_HArray1<TCollection_HAsciiString>(1, nb1);
                for (int i1 = 1; i1 <= nb1; i1++)
                {
                    bool stat1 = data.ReadString(nsub1, i1, "description", ach, ref descriptionItem);
                    if (stat1)
                    {
                        description.SetValue(i1, descriptionItem);
                    }
                }
            }
        }
        else
        {
            ach.AddFail("Parameter #1 (description) is not a LIST");
        }

        // --- own field : implementationLevel ---
        TCollection_HAsciiString implementationLevel = null;
        data.ReadString(num, 2, "implementation_level", ach, ref implementationLevel);

        //--- Initialisation of the read entity ---
        if (!ach.HasFailed)
        {
            ent.Init(description, implementationLevel);
        }
    }

    public void WriteStep(StepWriter sw, FileDescription ent)
    {
        // --- own field : description ---
        sw.OpenSub();
        for (int i1 = 1; i1 <= ent.NbDescription(); i1++)
        {
            sw.Send(ent.DescriptionValue(i1));
        }
        sw.CloseSub();

        // --- own field : implementationLevel ---
        sw.Send(ent.ImplementationLevel());
    }
}
