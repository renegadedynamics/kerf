// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_GeneralModule.hxx
// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_GeneralModule.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>Defines General Services for HeaderSection Entities (Share, Check, Copy; Trace already inherited). Depends (for case numbers) of Protocol from HeaderSection.</summary>
public class RWHeaderSection_GeneralModule : StepData_GeneralModule
{
    /// <summary>Creates a GeneralModule, registered with HeaderSection's protocol (process-wide, once: RWHeaderSection.Init).</summary>
    public RWHeaderSection_GeneralModule()
    {
        Interface_GeneralLib.SetGlobal(this, HeaderSection.Protocol());
    }

    /// <summary>Specific filling of the list of Entities shared by an Entity ent, according to a Case Number CN (provided by HeaderSection Protocol).</summary>
    public override void FillSharedCase(int CN, Standard_Transient? ent, Interface_EntityIterator iter)
    {
        if (CN != 4)
        {
            return;
        }

        StepData_UndefinedEntity undf = ent as StepData_UndefinedEntity;
        undf.FillShared(iter);
    }

    /// <summary>Specific Checking of an Entity ent.</summary>
    public override void CheckCase(int CN, Standard_Transient? ent, Interface_ShareTool shares, ref Interface_Check? ach)
    {
    }

    /// <summary>Specific Copy ("Deep") from entfrom to entto (same type) by using a CopyTool which provides its working Map. Use method Transferred from CopyTool to work.</summary>
    public override void CopyCase(int CN, Standard_Transient? entfrom, Standard_Transient? entto, Interface_CopyTool TC)
    {
        //   ajout manuel
        switch (CN)
        {
            case 1:
            {
                int i;
                HeaderSection_FileName enfr = entfrom as HeaderSection_FileName;
                HeaderSection_FileName ento = entto as HeaderSection_FileName;
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
                HeaderSection_FileDescription enfr = entfrom as HeaderSection_FileDescription;
                HeaderSection_FileDescription ento = entto as HeaderSection_FileDescription;
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
                HeaderSection_FileSchema enfr = entfrom as HeaderSection_FileSchema;
                HeaderSection_FileSchema ento = entto as HeaderSection_FileSchema;
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
                StepData_UndefinedEntity undfrom = entfrom as StepData_UndefinedEntity;
                StepData_UndefinedEntity undto = entto as StepData_UndefinedEntity;
                undto.GetFromAnother(undfrom, TC); //  On pourrait rapatrier cela
            }
            break;

            default:
                break;
        }
    }

    // --- Construction of empty class ---

    public override bool NewVoid(int CN, out Standard_Transient? ent)
    {
        ent = null;
        if (CN == 0)
        {
            return false;
        }
        switch (CN)
        {
            case 1:
                ent = new HeaderSection_FileName();
                break;
            case 2:
                ent = new HeaderSection_FileDescription();
                break;
            case 3:
                ent = new HeaderSection_FileSchema();
                break;
            case 4:
                ent = new StepData_UndefinedEntity();
                break;
            default:
                return false;
        }
        return true;
    }
}
