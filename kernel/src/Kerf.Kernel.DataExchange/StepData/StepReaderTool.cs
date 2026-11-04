// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepReaderTool.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepReaderTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// Specific FileReaderTool for Step; works with FileReaderData provides references evaluation, plus access to
/// literal data and specific methods defined by FileReaderTool. Remarks: works with a ReaderLib to load Entities.
/// </summary>
internal class StepReaderTool : FileReaderTool
{
    private FileRecognizer _thereco;
    private readonly GeneralLib _theglib;
    private readonly ReaderLib _therlib;

    // Constructor: Initialize the STEP reader tool with reader data and protocol

    /// <summary>Creates StepReaderTool to work with a StepReaderData according to a Step Protocol. Defines the ReaderLib at this time.</summary>
    public StepReaderTool(StepReaderData reader, StepProtocol protocol)
    {
        _theglib = new GeneralLib(protocol); // General library from protocol
        _therlib = new ReaderLib(protocol);  // Reader library from protocol
        SetData(reader, protocol);
    }

    // Recognize and create an entity from a STEP record number
    // Returns true if recognition was successful

    /// <summary>
    /// Bounds empty entities to records, works with the Protocol (in fact its GeneralLib) recorded in the Library.
    /// Called by Prepare (which may not be called if Prepare with a Recognizer is called).
    /// </summary>
    public override bool Recognize(int num, ref FrameworkCheck? ach, ref Transient? ent)
    {
        // Recognizer: It ensures the Recognition (-> Restrictive list of types)
        // If a specific recognizer is available, use it for type recognition
        if (_thereco is not null)
        {
            StepReaderData stepdat = Data() as StepReaderData;
            return _thereco.Evaluate(stepdat.RecordType(num), ref ent);
        }

        // No Recognizer: Recognition by the library
        // Fall back to standard library recognition when no specific recognizer is set
        return RecognizeByLib(num, _theglib, _therlib, ref ach, ref ent);
    }

    // ....         Preparation methods specific to StepReaderTool         ....

    // Prepare the reader tool with a file recognizer and optimization flag

    /// <summary>Bounds empty entities to records, uses a Recognizer, then calls Prepare(optimize).</summary>
    public void Prepare(FileRecognizer? reco, bool optim = true)
    {
        _thereco = reco; // Store the recognizer for later use
        Prepare(optim); // Continue with standard preparation
    }

    /// <summary>
    /// Bounds empty entities to records, uses the Protocol (in fact its GeneralLib), resolves references in the
    /// data (SetEntityNumbers), then binds the entities (SetEntities). optimize True (D) uses the map of the idents.
    /// </summary>
    public void Prepare(bool optim = true)
    {
        // SetEntityNumbers has been moved to ReaderData side, because of many accesses
        // Check if error handling is enabled to decide on exception handling strategy
        bool erh = ErrorHandle();
        StepReaderData stepdat = Data() as StepReaderData;
        if (erh)
        {
            try
            {
                stepdat.SetEntityNumbers(optim);
                SetEntities();
            }
            catch (FailureException exception)
            {
                using global::Kerf.Engine.Foundation.Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
                sout.Append(" Exception Raised during Preparation :\n");
                sout.Append(exception.Message);
                sout.Append("\n Now, trying to continue, but with presomption of failure\n");
            }
        }
        else
        {
            stepdat.SetEntityNumbers(optim);
            SetEntities();
        }
    }

    // ....            Header Management: Preparation, reading            .... //

    // Prepare header entities by recognizing their types and binding them

    /// <summary>
    /// Prepares the Header: recognizes its entities (with a Recognizer if given, else by the library of the
    /// Protocol) and binds them to their records; an unrecognized one is bound to an Unknown Entity.
    /// </summary>
    public void PrepareHeader(FileRecognizer? reco)
    {
        int i = 0; // Index for iterating through header records

        // Type recognition
        StepReaderData stepdat = Data() as StepReaderData;
        while ((i = stepdat.FindNextHeaderRecord(i)) != 0)
        {
            Transient ent = null;
            // A Recognizer was provided: it sets a restrictive list of recognized types
            if (reco is not null)
            {
                // Try to recognize the entity type using the provided recognizer
                if (!reco.Evaluate(stepdat.RecordType(i), ref ent))
                {
                    // If recognition fails, create an unknown entity
                    ent = Protocol().UnknownEntity();
                }
            }
            else
            {
                // No Recognizer: Recognition by the library
                FrameworkCheck ach = new FrameworkCheck(); // should this be listed...?
                RecognizeByLib(i, _theglib, _therlib, ref ach, ref ent);
            }
            if (ent is null)
            {
                ent = Protocol().UnknownEntity();
            }
            stepdat.BindEntity(i, ent);
        }

        // Remaining reference resolution: only concerns sub-lists
        // Handled by ReaderData
        stepdat.PrepareHeader();
    }

    // ....   Methods for reading the Model (after preparation)   .... //

    // Begin reading process: clear model header and process header entities

    /// <summary>Fills model's header; that is, gives to it Header entities and commands their loading. Also fills StepModel's Global Check from StepReaderData's GlobalCheck.</summary>
    public override void BeginRead(InterfaceModel? amodel)
    {
        using global::Kerf.Engine.Foundation.Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendTrace();

        StepModel model = amodel as StepModel;
        StepReaderData stepdat = Data() as StepReaderData;

        model.ClearHeader();
        model.SetGlobalCheck(stepdat.GlobalCheck());
        int i = 0;
        while ((i = stepdat.FindNextHeaderRecord(i)) != 0)
        {
            Transient ent = stepdat.BoundEntity(i);
            FrameworkCheck ach = new FrameworkCheck(ent);
            AnalyseRecord(i, ent, ref ach);
            if (ent is UndefinedEntity)
            {
                string mess = "Header Entity not Recognized, StepType: " + stepdat.RecordType(i);
                ach.AddWarning(mess);
            }
            if (ach.HasFailed || ach.HasWarnings)
            {
                FrameworkCheck mch = model.GlobalCheck();
                mch.GetMessages(ach);
                model.SetGlobalCheck(mch);
            }
            model.AddHeaderEntity(ent);
            if (ach.HasWarnings)
            {
                int nbmess = ach.WarningCount;
                sout.Append(nbmess).Append(" Warnings on Reading Header Entity N0.").Append(i).Append(":");
                if (ent is not null)
                {
                    sout.Append(ent.DynamicTypeName()).Append('\n');
                }
                for (int nf = 1; nf <= nbmess; nf++)
                {
                    sout.Append(ach.CWarning(nf)).Append("\n");
                }
            }
            if (ach.HasFailed)
            {
                int nbmess = ach.FailCount;
                sout.Append(" Fails on Reading Header Entity N0.").Append(i).Append(":");
                if (ent is not null)
                {
                    sout.Append(ent.DynamicTypeName()).Append('\n');
                }
                for (int nf = 1; nf <= nbmess; nf++)
                {
                    sout.Append(ach.CFail(nf)).Append("\n");
                }
            }
        }
    }

    // Analyze a STEP record and populate the corresponding entity
    // Returns true if analysis was successful (no failures)

    /// <summary>Fills an entity, given record no; works by using a ReaderLib to load each entity, which must be a Transient. Actually, returned value is True if no fail, False else.</summary>
    public override bool AnalyseRecord(int num, Transient? anent, ref FrameworkCheck? acheck)
    {
        StepReaderData stepdat = Data() as StepReaderData;
        // Try to find appropriate reader module for this entity type
        if (_therlib.Select(anent, out ReaderModule imodule, out int cn))
        {
            // Cast to STEP-specific module and read the entity data
            ReadWriteModule module = imodule as ReadWriteModule;
            module.ReadStep(cn, stepdat, num, acheck, anent);
        }
        else
        {
            // Not found: try UndefinedEntity from StepData
            if (anent is not UndefinedEntity und)
            {
                acheck.AddFail("# Entity neither Recognized nor set as UndefinedEntity from StepData #");
            }
            else
            {
                und.ReadRecord(stepdat, num, acheck);
            }
        }
        return (!acheck.HasFailed);
    }

    // Finish reading process: set entity labels from record identifiers

    /// <summary>Ends file reading after reading all the entities; here, it binds the idents of the records as the labels of the entities.</summary>
    public override void EndRead(InterfaceModel? amodel)
    {
        StepReaderData stepdat = Data() as StepReaderData;
        if (amodel is not StepModel stepmodel)
        {
            return;
        }
        int i = 0;
        while ((i = stepdat.FindNextRecord(i)) != 0)
        {
            stepmodel.SetIdentLabel(stepdat.BoundEntity(i), stepdat.RecordIdent(i));
        }
    }
}
