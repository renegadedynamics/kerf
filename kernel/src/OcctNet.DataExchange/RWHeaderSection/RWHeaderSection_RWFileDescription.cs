// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_RWFileDescription.hxx
// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_RWFileDescription.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>Read and Write Module for FileDescription.</summary>
public class RWHeaderSection_RWFileDescription
{
    public RWHeaderSection_RWFileDescription()
    {
    }

    public void ReadStep(StepData_StepReaderData data, int num, Interface_Check ach, HeaderSection_FileDescription ent)
    {
        // --- Number of Parameter Control ---
        if (!data.CheckNbParams(num, 2, ach, "file_description has not 2 parameter(s)"))
        {
            return;
        }

        // --- own field : description ---
        NCollection_HArray1<TCollection_HAsciiString> aDescription = null;
        TCollection_HAsciiString aDescriptionItem = null;
        int nsub1 = data.SubListNumber(num, 1, false);
        if (nsub1 != 0)
        {
            int nb1 = data.NbParams(nsub1);
            if (nb1 > 0)
            {
                aDescription = new NCollection_HArray1<TCollection_HAsciiString>(1, nb1);
                for (int i1 = 1; i1 <= nb1; i1++)
                {
                    bool stat1 = data.ReadString(nsub1, i1, "description", ach, ref aDescriptionItem);
                    if (stat1)
                    {
                        aDescription.SetValue(i1, aDescriptionItem);
                    }
                }
            }
        }
        else
        {
            ach.AddFail("Parameter #1 (description) is not a LIST");
        }

        // --- own field : implementationLevel ---
        TCollection_HAsciiString aImplementationLevel = null;
        data.ReadString(num, 2, "implementation_level", ach, ref aImplementationLevel);

        //--- Initialisation of the read entity ---
        if (!ach.HasFailed())
        {
            ent.Init(aDescription, aImplementationLevel);
        }
    }

    public void WriteStep(StepData_StepWriter SW, HeaderSection_FileDescription ent)
    {
        // --- own field : description ---
        SW.OpenSub();
        for (int i1 = 1; i1 <= ent.NbDescription(); i1++)
        {
            SW.Send(ent.DescriptionValue(i1));
        }
        SW.CloseSub();

        // --- own field : implementationLevel ---
        SW.Send(ent.ImplementationLevel());
    }
}
