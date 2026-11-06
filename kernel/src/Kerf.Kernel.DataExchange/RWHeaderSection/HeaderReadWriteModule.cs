// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_ReadWriteModule.hxx
// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection_ReadWriteModule.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Step.Header;

/// <summary>General module to read and write HeaderSection entities.</summary>
internal class HeaderReadWriteModule : ReadWriteModule
{
    // -- General Declarations (Recognize, StepType) ---

    private const string PasReco = ""; // neutralise StartEntity de SW
    private const string Reco_FileName = "FILE_NAME";
    private const string Reco_FileDescription = "FILE_DESCRIPTION";
    private const string Reco_FileSchema = "FILE_SCHEMA";

    // -- Definition of the libraries --

    /// <summary>Creates the module, registered (with a new HeaderSection_Protocol) in the writer and reader libraries: process-wide, once (RWHeaderSection.Init).</summary>
    public HeaderReadWriteModule()
    {
        HeaderProtocol protocol = new HeaderProtocol();
        WriterLib.SetGlobal(this, protocol);
        ReaderLib.SetGlobal(this, protocol);
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
    public override bool IsComplex(int cn)
    {
        return false;
    }

    /// <summary>Returns a StepType (defined in EXPRESS form which belongs to a Type of Entity, identified by its CaseNumber determined by Protocol.</summary>
    public override string StepType(int cn)
    {
        switch (cn)
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

    public override void ReadStep(int cn, StepReaderData data, int num, FrameworkCheck ach, Transient? ent)
    {
        if (cn == 0)
        {
            return;
        }

        switch (cn)
        {
            case 1:
            {
                FileName anent = ent as FileName;
                RWFileName tool = new RWFileName();
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
                FileDescription anent = ent as FileDescription;
                RWFileDescription tool = new RWFileDescription();
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
                FileSchema anent = ent as FileSchema;
                RWFileSchema tool = new RWFileSchema();
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
                UndefinedEntity und = ent as UndefinedEntity;
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

    public override void WriteStep(int cn, StepWriter sw, Transient? ent)
    {
        if (cn == 0)
        {
            return;
        }
        switch (cn)
        {
            case 1:
            {
                FileName anent = ent as FileName;
                RWFileName tool = new RWFileName();
                if (anent is null)
                {
                    return;
                }
                else
                {
                    tool.WriteStep(sw, anent);
                }
            }
            break;

            case 2:
            {
                FileDescription anent = ent as FileDescription;
                RWFileDescription tool = new RWFileDescription();
                if (anent is null)
                {
                    return;
                }
                else
                {
                    tool.WriteStep(sw, anent);
                }
            }
            break;

            case 3:
            {
                FileSchema anent = ent as FileSchema;
                RWFileSchema tool = new RWFileSchema();
                if (anent is null)
                {
                    return;
                }
                else
                {
                    tool.WriteStep(sw, anent);
                }
            }
            break;

            case 4:
            {
                if (ent is not UndefinedEntity und)
                {
                    break;
                }
                //  Parentheses gerees par StepWriter lui-meme
                //   car en recevant le type PasReco, StepWriter ne l'affiche pas,
                //   mais il gere les parentheses comme si de rien n etait
                if (und.IsComplex)
                {
                    sw.StartComplex();
                }
                und.WriteParams(sw);
                if (und.IsComplex)
                {
                    sw.EndComplex();
                }
            }
            break;

            default:
                return;
        }
    }
}
