// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_RWFileName.hxx
// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_RWFileName.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;

using Kerf.Engine.Exchange.Step.Data;

using Occt;

namespace Kerf.Engine.Exchange.Step.Header;

/// <summary>Read and Write Module for FileName.</summary>
internal class RWFileName
{
    public RWFileName()
    {
    }

    public void ReadStep(StepReaderData data, int num, FrameworkCheck ach, FileName ent)
    {
        // --- Number of Parameter Control ---
        if (!data.CheckNbParams(num, 7, ach, "file_name has not 7 parameter(s)"))
        {
            return;
        }

        // --- own field : name ---
        TCollection_HAsciiString name = null;
        data.ReadString(num, 1, "name", ach, ref name);

        // --- own field : timeStamp ---
        TCollection_HAsciiString timeStamp = null;
        data.ReadString(num, 2, "time_stamp", ach, ref timeStamp);

        // --- own field : author ---
        NCollection_HArray1<TCollection_HAsciiString> author = null;
        TCollection_HAsciiString authorItem = null;
        int nsub3;
        nsub3 = data.SubListNumber(num, 3, false);
        if (nsub3 != 0)
        {
            int nb3 = data.NbParams(nsub3);
            author = new NCollection_HArray1<TCollection_HAsciiString>(1, nb3);
            for (int i3 = 1; i3 <= nb3; i3++)
            {
                bool stat3 = data.ReadString(nsub3, i3, "author", ach, ref authorItem);
                if (stat3)
                {
                    author.SetValue(i3, authorItem);
                }
            }
        }
        else
        {
            ach.AddFail("Parameter #3 (author) is not a LIST");
        }

        // --- own field : organization ---
        NCollection_HArray1<TCollection_HAsciiString> organization = null;
        TCollection_HAsciiString organizationItem = null;
        int nsub4;
        nsub4 = data.SubListNumber(num, 4, false);
        if (nsub4 != 0)
        {
            int nb4 = data.NbParams(nsub4);
            organization = new NCollection_HArray1<TCollection_HAsciiString>(1, nb4);
            for (int i4 = 1; i4 <= nb4; i4++)
            {
                bool stat4 = data.ReadString(nsub4, i4, "organization", ach, ref organizationItem);
                if (stat4)
                {
                    organization.SetValue(i4, organizationItem);
                }
            }
        }
        else
        {
            ach.AddFail("Parameter #4 (organization) is not a LIST");
        }

        // --- own field : preprocessorVersion ---
        TCollection_HAsciiString preprocessorVersion = null;
        data.ReadString(num, 5, "preprocessor_version", ach, ref preprocessorVersion);

        // --- own field : originatingSystem ---
        TCollection_HAsciiString originatingSystem = null;
        data.ReadString(num, 6, "originating_system", ach, ref originatingSystem);

        // --- own field : authorisation ---
        TCollection_HAsciiString authorisation = null;
        data.ReadString(num, 7, "authorisation", ach, ref authorisation);

        //--- Initialisation of the read entity ---
        if (!ach.HasFailed)
        {
            ent.Init(name, timeStamp, author, organization, preprocessorVersion, originatingSystem, authorisation);
        }
    }

    public void WriteStep(StepWriter sw, FileName ent)
    {
        // --- own field : name ---
        sw.Send(ent.Name());

        // --- own field : timeStamp ---
        sw.Send(ent.TimeStamp());

        // --- own field : author ---
        sw.OpenSub();
        for (int i3 = 1; i3 <= ent.NbAuthor(); i3++)
        {
            sw.Send(ent.AuthorValue(i3));
        }
        sw.CloseSub();

        // --- own field : organization ---
        sw.OpenSub();
        for (int i4 = 1; i4 <= ent.NbOrganization(); i4++)
        {
            sw.Send(ent.OrganizationValue(i4));
        }
        sw.CloseSub();

        // --- own field : preprocessorVersion ---
        sw.Send(ent.PreprocessorVersion());

        // --- own field : originatingSystem ---
        sw.Send(ent.OriginatingSystem());

        // --- own field : authorisation ---
        sw.Send(ent.Authorisation());
    }
}
