// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_ReadWriteModule.hxx
// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_ReadWriteModule.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>General module to read and write HeaderSection entities.</summary>
public class RWHeaderSection_ReadWriteModule : StepData_ReadWriteModule
{
    // -- General Declarations (Recognize, StepType) ---

    private const string PasReco = ""; // neutralise StartEntity de SW
    private const string Reco_FileName = "FILE_NAME";
    private const string Reco_FileDescription = "FILE_DESCRIPTION";
    private const string Reco_FileSchema = "FILE_SCHEMA";

    // -- Definition of the libraries --

    /// <summary>Creates the module, registered (with a new HeaderSection_Protocol) in the writer and reader libraries: process-wide, once (RWHeaderSection.Init).</summary>
    public RWHeaderSection_ReadWriteModule()
    {
        HeaderSection_Protocol protocol = new HeaderSection_Protocol();
        StepData_WriterLib.SetGlobal(this, protocol);
        Interface_ReaderLib.SetGlobal(this, protocol);
    }

    // --- Case Recognition ---

    /// <summary>Associates a positive Case Number to each type of HeaderSection entity, given as a String defined in the EXPRESS form.</summary>
    public override int CaseStep(string key)
    {
        if (key == Reco_FileName)
        {
            return 1;
        }
        if (key == Reco_FileDescription)
        {
            return 2;
        }
        if (key == Reco_FileSchema)
        {
            return 3;
        }
        return 0;
    }

    // --- External Mapping Case Recognition ---

    /// <summary>Associates a positive Case Number to each type of HeaderSection Complex entity, given as a String defined in the EXPRESS form.</summary>
    public override int CaseStep(NCollection_Sequence<string> types)
    {
        return 0; // OCCT_DEBUG: "Plex Instance illegal" left out
    }

    // --- External Mapping Recognition ---

    /// <summary>Returns True if the Case Number corresponds to a Complex Type.</summary>
    public override bool IsComplex(int CN)
    {
        return false;
    }

    /// <summary>Returns a StepType (defined in EXPRESS form which belongs to a Type of Entity, identified by its CaseNumber determined by Protocol.</summary>
    public override string StepType(int CN)
    {
        switch (CN)
        {
            case 1:
                return Reco_FileName;
            case 2:
                return Reco_FileDescription;
            case 3:
                return Reco_FileSchema;
            default:
                return PasReco;
        }
    }

    // -- Reading of a file --

    public override void ReadStep(int CN, StepData_StepReaderData data, int num, Interface_Check ach, Standard_Transient? ent)
    {
        if (CN == 0)
        {
            return;
        }

        switch (CN)
        {
            case 1:
            {
                HeaderSection_FileName anent = ent as HeaderSection_FileName;
                RWHeaderSection_RWFileName tool = new RWHeaderSection_RWFileName();
                if (anent is null)
                {
                    ach.AddFail("Type Mismatch on FileName");
                }
                else
                {
                    tool.ReadStep(data, num, ach, anent);
                }
            }
            break;

            case 2:
            {
                HeaderSection_FileDescription anent = ent as HeaderSection_FileDescription;
                RWHeaderSection_RWFileDescription tool = new RWHeaderSection_RWFileDescription();
                if (anent is null)
                {
                    ach.AddFail("Type Mismatch on FileDescription");
                }
                else
                {
                    tool.ReadStep(data, num, ach, anent);
                }
            }
            break;

            case 3:
            {
                HeaderSection_FileSchema anent = ent as HeaderSection_FileSchema;
                RWHeaderSection_RWFileSchema tool = new RWHeaderSection_RWFileSchema();
                if (anent is null)
                {
                    ach.AddFail("Type Mismatch on FileSchema");
                }
                else
                {
                    tool.ReadStep(data, num, ach, anent);
                }
            }
            break;

            case 4:
            {
                StepData_UndefinedEntity und = ent as StepData_UndefinedEntity;
                if (und is null)
                {
                    ach.AddFail("# Entity neither Recognized nor set as UndefinedEntity from StepData #");
                }
                else
                {
                    und.ReadRecord(data, num, ach);
                }
            }
            break;

            default:
                ach.AddFail("Type Mismatch when reading - Entity");
                break;
        }
    }

    // -- Writing of a file --

    public override void WriteStep(int CN, StepData_StepWriter SW, Standard_Transient? ent)
    {
        if (CN == 0)
        {
            return;
        }
        switch (CN)
        {
            case 1:
            {
                HeaderSection_FileName anent = ent as HeaderSection_FileName;
                RWHeaderSection_RWFileName tool = new RWHeaderSection_RWFileName();
                if (anent is null)
                {
                    return;
                }
                else
                {
                    tool.WriteStep(SW, anent);
                }
            }
            break;

            case 2:
            {
                HeaderSection_FileDescription anent = ent as HeaderSection_FileDescription;
                RWHeaderSection_RWFileDescription tool = new RWHeaderSection_RWFileDescription();
                if (anent is null)
                {
                    return;
                }
                else
                {
                    tool.WriteStep(SW, anent);
                }
            }
            break;

            case 3:
            {
                HeaderSection_FileSchema anent = ent as HeaderSection_FileSchema;
                RWHeaderSection_RWFileSchema tool = new RWHeaderSection_RWFileSchema();
                if (anent is null)
                {
                    return;
                }
                else
                {
                    tool.WriteStep(SW, anent);
                }
            }
            break;

            case 4:
            {
                if (ent is not StepData_UndefinedEntity und)
                {
                    break;
                }
                //  Parentheses gerees par StepWriter lui-meme
                //   car en recevant le type PasReco, StepWriter ne l'affiche pas,
                //   mais il gere les parentheses comme si de rien n etait
                if (und.IsComplex())
                {
                    SW.StartComplex();
                }
                und.WriteParams(SW);
                if (und.IsComplex())
                {
                    SW.EndComplex();
                }
            }
            break;

            default:
                return;
        }
    }
}
