// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_RWFileName.hxx
// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_RWFileName.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>Read and Write Module for FileName.</summary>
public class RWHeaderSection_RWFileName
{
    public RWHeaderSection_RWFileName()
    {
    }

    public void ReadStep(StepData_StepReaderData data, int num, Interface_Check ach, HeaderSection_FileName ent)
    {
        // --- Number of Parameter Control ---
        if (!data.CheckNbParams(num, 7, ach, "file_name has not 7 parameter(s)"))
        {
            return;
        }

        // --- own field : name ---
        TCollection_HAsciiString aName = null;
        data.ReadString(num, 1, "name", ach, ref aName);

        // --- own field : timeStamp ---
        TCollection_HAsciiString aTimeStamp = null;
        data.ReadString(num, 2, "time_stamp", ach, ref aTimeStamp);

        // --- own field : author ---
        NCollection_HArray1<TCollection_HAsciiString> aAuthor = null;
        TCollection_HAsciiString aAuthorItem = null;
        int nsub3;
        nsub3 = data.SubListNumber(num, 3, false);
        if (nsub3 != 0)
        {
            int nb3 = data.NbParams(nsub3);
            aAuthor = new NCollection_HArray1<TCollection_HAsciiString>(1, nb3);
            for (int i3 = 1; i3 <= nb3; i3++)
            {
                bool stat3 = data.ReadString(nsub3, i3, "author", ach, ref aAuthorItem);
                if (stat3)
                {
                    aAuthor.SetValue(i3, aAuthorItem);
                }
            }
        }
        else
        {
            ach.AddFail("Parameter #3 (author) is not a LIST");
        }

        // --- own field : organization ---
        NCollection_HArray1<TCollection_HAsciiString> aOrganization = null;
        TCollection_HAsciiString aOrganizationItem = null;
        int nsub4;
        nsub4 = data.SubListNumber(num, 4, false);
        if (nsub4 != 0)
        {
            int nb4 = data.NbParams(nsub4);
            aOrganization = new NCollection_HArray1<TCollection_HAsciiString>(1, nb4);
            for (int i4 = 1; i4 <= nb4; i4++)
            {
                bool stat4 = data.ReadString(nsub4, i4, "organization", ach, ref aOrganizationItem);
                if (stat4)
                {
                    aOrganization.SetValue(i4, aOrganizationItem);
                }
            }
        }
        else
        {
            ach.AddFail("Parameter #4 (organization) is not a LIST");
        }

        // --- own field : preprocessorVersion ---
        TCollection_HAsciiString aPreprocessorVersion = null;
        data.ReadString(num, 5, "preprocessor_version", ach, ref aPreprocessorVersion);

        // --- own field : originatingSystem ---
        TCollection_HAsciiString aOriginatingSystem = null;
        data.ReadString(num, 6, "originating_system", ach, ref aOriginatingSystem);

        // --- own field : authorisation ---
        TCollection_HAsciiString aAuthorisation = null;
        data.ReadString(num, 7, "authorisation", ach, ref aAuthorisation);

        //--- Initialisation of the read entity ---
        if (!ach.HasFailed())
        {
            ent.Init(aName, aTimeStamp, aAuthor, aOrganization, aPreprocessorVersion, aOriginatingSystem, aAuthorisation);
        }
    }

    public void WriteStep(StepData_StepWriter SW, HeaderSection_FileName ent)
    {
        // --- own field : name ---
        SW.Send(ent.Name());

        // --- own field : timeStamp ---
        SW.Send(ent.TimeStamp());

        // --- own field : author ---
        SW.OpenSub();
        for (int i3 = 1; i3 <= ent.NbAuthor(); i3++)
        {
            SW.Send(ent.AuthorValue(i3));
        }
        SW.CloseSub();

        // --- own field : organization ---
        SW.OpenSub();
        for (int i4 = 1; i4 <= ent.NbOrganization(); i4++)
        {
            SW.Send(ent.OrganizationValue(i4));
        }
        SW.CloseSub();

        // --- own field : preprocessorVersion ---
        SW.Send(ent.PreprocessorVersion());

        // --- own field : originatingSystem ---
        SW.Send(ent.OriginatingSystem());

        // --- own field : authorisation ---
        SW.Send(ent.Authorisation());
    }
}
