// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_GeneralModule.hxx
// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_GeneralModule.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Step.Header;

/// <summary>Defines General Services for HeaderSection Entities (Share, Check, Copy; Trace already inherited). Depends (for case numbers) of Protocol from HeaderSection.</summary>
internal class HeaderGeneralModule : StepGeneralModule
{
    /// <summary>Creates a GeneralModule, registered with HeaderSection's protocol (process-wide, once: RWHeaderSection.Init).</summary>
    public HeaderGeneralModule()
    {
        GeneralLib.SetGlobal(this, global::Kerf.Engine.Exchange.Step.Header.HeaderSection.Protocol());
    }

    /// <summary>Specific filling of the list of Entities shared by an Entity ent, according to a Case Number CN (provided by HeaderSection Protocol).</summary>
    public override void FillSharedCase(int cn, Transient? ent, EntityIterator iter)
    {
        if (cn != 4)
        {
            return;
        }

        UndefinedEntity undf = ent as UndefinedEntity;
        undf.FillShared(iter);
    }

    /// <summary>Specific Checking of an Entity ent.</summary>
    public override void CheckCase(int cn, Transient? ent, ShareTool shares, ref FrameworkCheck? ach)
    {
    }

    /// <summary>Specific Copy ("Deep") from entfrom to entto (same type) by using a CopyTool which provides its working Map. Use method Transferred from CopyTool to work.</summary>
    public override void CopyCase(int cn, Transient? entfrom, Transient? entto, CopyTool tc)
    {
        //   ajout manuel
        switch (cn)
        {
            case 1:
            {
                int i;
                FileName enfr = entfrom as FileName;
                FileName ento = entto as FileName;
                TCollection_HAsciiString name = new TCollection_HAsciiString(enfr.Name());
                TCollection_HAsciiString time = new TCollection_HAsciiString(enfr.TimeStamp());
                int nba = enfr.NbAuthor();
                NCollection_HArray1<TCollection_HAsciiString> auth = new NCollection_HArray1<TCollection_HAsciiString>(1, nba);
                for (i = 1; i <= nba; i++)
                {
                    auth.SetValue(i, new TCollection_HAsciiString(enfr.AuthorValue(i)));
                }
                int nbo = enfr.NbOrganization();
                NCollection_HArray1<TCollection_HAsciiString> orga = new NCollection_HArray1<TCollection_HAsciiString>(1, nbo);
                for (i = 1; i <= nbo; i++)
                {
                    orga.SetValue(i, new TCollection_HAsciiString(enfr.OrganizationValue(i)));
                }
                TCollection_HAsciiString prep = new TCollection_HAsciiString(enfr.PreprocessorVersion());
                TCollection_HAsciiString orig = new TCollection_HAsciiString(enfr.OriginatingSystem());
                TCollection_HAsciiString autr = new TCollection_HAsciiString(enfr.Authorisation());
                ento.Init(name, time, auth, orga, prep, orig, autr);
            }
            break;

            case 2:
            {
                int i;
                FileDescription enfr = entfrom as FileDescription;
                FileDescription ento = entto as FileDescription;
                int nbd = enfr.NbDescription();
                NCollection_HArray1<TCollection_HAsciiString> desc = new NCollection_HArray1<TCollection_HAsciiString>(1, nbd);
                for (i = 1; i <= nbd; i++)
                {
                    desc.SetValue(i, new TCollection_HAsciiString(enfr.DescriptionValue(i)));
                }
                TCollection_HAsciiString impl = new TCollection_HAsciiString(enfr.ImplementationLevel());
                ento.Init(desc, impl);
            }
            break;

            case 3:
            {
                int i;
                FileSchema enfr = entfrom as FileSchema;
                FileSchema ento = entto as FileSchema;
                int nbs = enfr.NbSchemaIdentifiers();
                NCollection_HArray1<TCollection_HAsciiString> sche = new NCollection_HArray1<TCollection_HAsciiString>(1, nbs);
                for (i = 1; i <= nbs; i++)
                {
                    sche.SetValue(i, new TCollection_HAsciiString(enfr.SchemaIdentifiersValue(i)));
                }
                ento.Init(sche);
            }
            break;

            case 4:
            {
                UndefinedEntity undfrom = entfrom as UndefinedEntity;
                UndefinedEntity undto = entto as UndefinedEntity;
                undto.GetFromAnother(undfrom, tc); //  On pourrait rapatrier cela
            }
            break;

            default:
                break;
        }
    }

    // --- Construction of empty class ---

    public override bool NewVoid(int cn, out Transient? ent)
    {
        ent = null;
        if (cn == 0)
        {
            return false;
        }
        switch (cn)
        {
            case 1:
                ent = new FileName();
                break;
            case 2:
                ent = new FileDescription();
                break;
            case 3:
                ent = new FileSchema();
                break;
            case 4:
                ent = new UndefinedEntity();
                break;
            default:
                return false;
        }
        return true;
    }
}
