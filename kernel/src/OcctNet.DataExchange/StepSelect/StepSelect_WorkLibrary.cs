// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_WorkLibrary.hxx
// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_WorkLibrary.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// Performs Read and Write a STEP File with a STEP Model Following the protocols, Copy may be implemented or not
/// </summary>
/// <remarks>
/// OCCT.NET: reading takes a .NET stream, a progress range and the parser's limits for hostile input
/// (<see cref="StepFile_ReadLimits"/>, through the overloads that take them); writing writes the context's stream when
/// it has one (<see cref="IFSelect_ContextWrite.OStream"/>, decision 1225), else the file. Like OCCT 8's, the library
/// reads into the model it is given (decision 1227): STEPControl_Reader makes it.
/// </remarks>
public class StepSelect_WorkLibrary : IFSelect_WorkLibrary
{
    /// <summary>
    /// Creates a STEP WorkLibrary &lt;copymode&gt; precises whether Copy is implemented or not
    /// </summary>
    public StepSelect_WorkLibrary(bool copymode = true)
    {
        thecopymode = copymode;
        thelabmode = 0;
        SetDumpLevels(1, 2);
        SetDumpHelp(0, "#id + Step Type");
        SetDumpHelp(1, "Entity as in file");
        SetDumpHelp(2, "Entity + shareds (level 1) as in file");
    }

    // rq : les init sont faits par ailleurs, pas de souci a se faire

    /// <summary>
    /// Selects a mode to dump entities 0 (D) : prints numbers, then displays table number/label 1 : prints labels,
    /// then displays table label/number 2 : prints labels onky
    /// </summary>
    public void SetDumpLabel(int mode)
    {
        thelabmode = mode;
    }

    /// <summary>
    /// Reads a STEP File and returns a STEP Model (into &lt;mod&gt;), or lets &lt;mod&gt; "Null" in case of Error
    /// Returns 0 if OK, 1 if Read Error, -1 if File not opened
    /// </summary>
    public override int ReadFile(string name, ref Interface_InterfaceModel model, Interface_Protocol protocol,
                                 ProgressRange theProgress = default)
    {
        return ReadFile(name, model, protocol, theProgress, null);
    }

    /// <summary>As <see cref="ReadFile(string, ref Interface_InterfaceModel, Interface_Protocol, ProgressRange)"/>, with the parser's limits (OCCT.NET).</summary>
    public int ReadFile(string name, Interface_InterfaceModel model, Interface_Protocol protocol,
                        ProgressRange theProgress, StepFile_ReadLimits theLimits)
    {
        StepData_Protocol stepro = protocol as StepData_Protocol;
        if (stepro is null)
        {
            return 1;
        }
        int aStatus = StepFile_Read.Read(name, null, model as StepData_StepModel, stepro, theProgress, theLimits);
        return aStatus;
    }

    /// <summary>
    /// Reads a STEP file from stream and returns a STEP Model (into &lt;mod&gt;), or lets &lt;mod&gt; "Null" in
    /// case of Error Returns 0 if OK, 1 if Read Error, -1 if File not opened
    /// </summary>
    public override int ReadStream(string theName, Stream theIStream, ref Interface_InterfaceModel model, Interface_Protocol protocol,
                                   ProgressRange theProgress = default)
    {
        return ReadStream(theName, theIStream, model, protocol, theProgress, null);
    }

    /// <summary>As <see cref="ReadStream(string, Stream, ref Interface_InterfaceModel, Interface_Protocol, ProgressRange)"/>, with the parser's limits (OCCT.NET).</summary>
    public int ReadStream(string theName, Stream theIStream, Interface_InterfaceModel model, Interface_Protocol protocol,
                          ProgressRange theProgress, StepFile_ReadLimits theLimits)
    {
        StepData_Protocol stepro = protocol as StepData_Protocol;
        if (stepro is null)
        {
            return 1;
        }
        int aStatus = StepFile_Read.Read(theName, theIStream, model as StepData_StepModel, stepro, theProgress, theLimits);
        return aStatus;
    }

    /// <summary>
    /// Writes a File from a STEP Model Returns False (and writes no file) if &lt;ctx&gt; does not bring a STEP
    /// Model
    /// </summary>
    public override bool WriteFile(IFSelect_ContextWrite ctx)
    {
        //  Preparation
        using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
        StepData_StepModel stepmodel = ctx.Model() as StepData_StepModel;
        StepData_Protocol stepro = ctx.Protocol() as StepData_Protocol;
        if (stepmodel is null || stepro is null)
        {
            return false;
        }

        // OCCT.NET: the context's stream, when it has one (decision 1225), else the file
        Stream aStream = ctx.OStream();
        FileStream aFile = null;
        if (aStream is null)
        {
            try
            {
                aFile = new FileStream(ctx.FileName(), FileMode.Create, FileAccess.Write, FileShare.Read);
                aStream = aFile;
            }
            catch (Exception anException) when (anException is IOException or UnauthorizedAccessException or ArgumentException
                                                    or NotSupportedException)
            {
                aStream = null;
            }
        }

        if (aStream is null)
        {
            ctx.CCheck(0).AddFail("Step File could not be created");
            sout.Append(" Step File could not be created : ").Append(ctx.FileName()).Append('\n');
            return false;
        }
        try
        {
            sout.Append(" Step File Name : ").Append(ctx.FileName());
            StepData_StepWriter SW = new StepData_StepWriter(stepmodel);
            sout.Append("(").Append(stepmodel.NbEntities()).Append(" ents) ");

            //  File Modifiers
            int nbmod = ctx.NbModifiers();
            for (int numod = 1; numod <= nbmod; numod++)
            {
                ctx.SetModifier(numod);
                StepSelect_FileModifier filemod = ctx.FileModifier() as StepSelect_FileModifier;
                if (filemod is not null)
                {
                    filemod.Perform(ctx, SW);
                }
                //   (impressions de mise au point)
                sout.Append(" .. FileMod.").Append(numod).Append(filemod.Label()); // OCCT: a modifier of another kind is dereferenced null
                if (ctx.IsForAll())
                {
                    sout.Append(" (all model)");
                }
                else
                {
                    sout.Append(" (").Append(ctx.NbEntities()).Append(" entities)");
                }
            }

            //  Envoi
            SW.SendModel(stepro);
            Interface_CheckIterator chl = SW.CheckList();
            for (chl.Start(); chl.More(); chl.Next())
            {
                ctx.CCheck(chl.Number()).GetMessages(chl.Value());
            }
            sout.Append(" Write ");
            bool isGood = SW.Print(aStream);
            sout.Append(" Done").Append('\n');

            try
            {
                aStream.Flush();
            }
            catch (IOException anException)
            {
                sout.Append(anException.Message).Append('\n');
                isGood = false;
            }
            return isGood;
        }
        finally
        {
            aFile?.Dispose();
        }
    }

    /// <summary>
    /// Performs the copy of entities from an original model to a new one. Works according &lt;copymode&gt; : if True,
    /// standard copy is run else nothing is done and returned value is False
    /// </summary>
    public override bool CopyModel(Interface_InterfaceModel original, Interface_InterfaceModel newmodel, Interface_EntityIterator list,
                                   Interface_CopyTool TC)
    {
        if (thecopymode)
        {
            return base.CopyModel(original, newmodel, list, TC);
        }
        return thecopymode;
    }

    /// <summary>
    /// Dumps an entity under STEP form, i.e. as a part of a Step file Works with a StepDumper. Level 0 just displays
    /// type; level 1 displays the entity itself, and level 2 displays the entity plus its shared ones (one sub-level:
    /// immediately shared entities)
    /// </summary>
    public override void DumpEntity(Interface_InterfaceModel model, Interface_Protocol protocol, Transient entity, TextWriter S,
                                    int level)
    {
        int nument = model.Number(entity);
        if (nument <= 0 || nument > model.NbEntities())
        {
            return;
        }
        bool iserr = model.IsRedefinedContent(nument);
        Transient ent, con;
        ent = entity;
        S.Write(" --- (STEP) Entity ");
        model.Print(entity, S);
        if (iserr)
        {
            con = model.ReportEntity(nument).Content();
        }
        if (entity is null)
        {
            S.Write(" Null\n");
            return;
        }

        //  On attaque le dump : d abord cas de l Erreur
        S.Write(" Type cdl : " + entity.DynamicTypeName() + "\n");
        if (iserr)
        {
            S.Write(" ***  NOT WELL LOADED : CONTENT FROM FILE  ***\n");
        }
        else if (model.IsUnknownEntity(nument))
        {
            S.Write(" ***  UNKNOWN TYPE  ***\n");
        }

        StepData_StepDumper dump = new StepData_StepDumper(model as StepData_StepModel, protocol as StepData_Protocol, thelabmode);
        dump.Dump(S, ent, level);
    }

    private bool thecopymode;
    private int thelabmode;
}
