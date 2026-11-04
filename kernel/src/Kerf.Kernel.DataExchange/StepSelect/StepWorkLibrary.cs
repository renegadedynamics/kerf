// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_WorkLibrary.hxx
// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_WorkLibrary.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Parser;
using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Step.Selections;

/// <summary>
/// Performs Read and Write a STEP File with a STEP Model Following the protocols, Copy may be implemented or not
/// </summary>
/// <remarks>
/// OCCT.NET: reading takes a .NET stream, a progress range and the parser's limits for hostile input
/// (<see cref="ParserReadLimits"/>, through the overloads that take them); writing writes the context's stream when
/// it has one (<see cref="ContextWrite.OStream"/>, decision 1225), else the file. Like OCCT 8's, the library
/// reads into the model it is given (decision 1227): STEPControl_Reader makes it.
/// </remarks>
internal class StepWorkLibrary : WorkLibrary
{
    /// <summary>
    /// Creates a STEP WorkLibrary &lt;copymode&gt; precises whether Copy is implemented or not
    /// </summary>
    public StepWorkLibrary(bool copymode = true)
    {
        _thecopymode = copymode;
        _thelabmode = 0;
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
        _thelabmode = mode;
    }

    /// <summary>
    /// Reads a STEP File and returns a STEP Model (into &lt;mod&gt;), or lets &lt;mod&gt; "Null" in case of Error
    /// Returns 0 if OK, 1 if Read Error, -1 if File not opened
    /// </summary>
    public override int ReadFile(string name, ref InterfaceModel model, Protocol protocol,
                                 ProgressRange progress = default)
    {
        StepModel aModel = ModelFor(model, protocol);
        int status = ReadFile(name, aModel, protocol, progress, null);
        if (status == 0)
        {
            model = aModel;
        }
        return status;
    }

    // The model to read into: the one given, or a new one for the protocol when there is none, as IFSelect_WorkSession
    // gives (OCCT 8's library read into the null model, so XSControl_Reader::ReadFile failed on STEP; Kerf decision
    // 16822)
    private static StepModel ModelFor(InterfaceModel model, Protocol protocol)
    {
        if (model is StepModel || protocol is not StepProtocol aProtocol)
        {
            return model as StepModel;
        }
        var aModel = new StepModel();
        aModel.SetProtocol(aProtocol);
        return aModel;
    }

    /// <summary>As <see cref="ReadFile(string, ref InterfaceModel, Protocol, ProgressRange)"/>, with the parser's limits (OCCT.NET).</summary>
    public int ReadFile(string name, InterfaceModel model, Protocol protocol,
                        ProgressRange progress, ParserReadLimits limits)
    {
        StepProtocol stepro = protocol as StepProtocol;
        if (stepro is null)
        {
            return 1;
        }
        int status = StepFileReader.Read(name, null, model as StepModel, stepro, progress, limits);
        return status;
    }

    /// <summary>
    /// Reads a STEP file from stream and returns a STEP Model (into &lt;mod&gt;), or lets &lt;mod&gt; "Null" in
    /// case of Error Returns 0 if OK, 1 if Read Error, -1 if File not opened
    /// </summary>
    public override int ReadStream(string name, Stream iStream, ref InterfaceModel model, Protocol protocol,
                                   ProgressRange progress = default)
    {
        StepModel aModel = ModelFor(model, protocol);
        int status = ReadStream(name, iStream, aModel, protocol, progress, null);
        if (status == 0)
        {
            model = aModel;
        }
        return status;
    }

    /// <summary>As <see cref="ReadStream(string, Stream, ref InterfaceModel, Protocol, ProgressRange)"/>, with the parser's limits (OCCT.NET).</summary>
    public int ReadStream(string name, Stream iStream, InterfaceModel model, Protocol protocol,
                          ProgressRange progress, ParserReadLimits limits)
    {
        StepProtocol stepro = protocol as StepProtocol;
        if (stepro is null)
        {
            return 1;
        }
        int status = StepFileReader.Read(name, iStream, model as StepModel, stepro, progress, limits);
        return status;
    }

    /// <summary>
    /// Writes a File from a STEP Model Returns False (and writes no file) if &lt;ctx&gt; does not bring a STEP
    /// Model
    /// </summary>
    public override bool WriteFile(ContextWrite ctx)
    {
        //  Preparation
        using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
        StepModel stepmodel = ctx.Model() as StepModel;
        StepProtocol stepro = ctx.Protocol() as StepProtocol;
        if (stepmodel is null || stepro is null)
        {
            return false;
        }

        // OCCT.NET: the context's stream, when it has one (decision 1225), else the file
        Stream stream = ctx.OStream();
        FileStream file = null;
        if (stream is null)
        {
            try
            {
                file = new FileStream(ctx.FileName(), FileMode.Create, FileAccess.Write, FileShare.Read);
                stream = file;
            }
            catch (Exception exception) when (exception is IOException or UnauthorizedAccessException or ArgumentException
                                                    or NotSupportedException)
            {
                stream = null;
            }
        }

        if (stream is null)
        {
            ctx.CCheck(0).AddFail("Step File could not be created");
            sout.Append(" Step File could not be created : ").Append(ctx.FileName()).Append('\n');
            return false;
        }
        try
        {
            sout.Append(" Step File Name : ").Append(ctx.FileName());
            StepWriter sw = new StepWriter(stepmodel);
            sout.Append("(").Append(stepmodel.EntityCount).Append(" ents) ");

            //  File Modifiers
            int nbmod = ctx.ModifierCount;
            for (int numod = 1; numod <= nbmod; numod++)
            {
                ctx.SetModifier(numod);
                FileModifier filemod = ctx.FileModifier() as FileModifier;
                if (filemod is not null)
                {
                    filemod.Perform(ctx, sw);
                }
                //   (impressions de mise au point)
                sout.Append(" .. FileMod.").Append(numod).Append(filemod.Label()); // OCCT: a modifier of another kind is dereferenced null
                if (ctx.IsForAll)
                {
                    sout.Append(" (all model)");
                }
                else
                {
                    sout.Append(" (").Append(ctx.EntityCount).Append(" entities)");
                }
            }

            //  Envoi
            sw.SendModel(stepro);
            CheckIterator chl = sw.CheckList();
            for (chl.Start(); chl.More(); chl.Next())
            {
                ctx.CCheck(chl.Number()).GetMessages(chl.Value());
            }
            sout.Append(" Write ");
            bool isGood = sw.Print(stream);
            sout.Append(" Done").Append('\n');

            try
            {
                stream.Flush();
            }
            catch (IOException exception)
            {
                sout.Append(exception.Message).Append('\n');
                isGood = false;
            }
            return isGood;
        }
        finally
        {
            file?.Dispose();
        }
    }

    /// <summary>
    /// Performs the copy of entities from an original model to a new one. Works according &lt;copymode&gt; : if True,
    /// standard copy is run else nothing is done and returned value is False
    /// </summary>
    public override bool CopyModel(InterfaceModel original, InterfaceModel newmodel, EntityIterator list,
                                   CopyTool tc)
    {
        if (_thecopymode)
        {
            return base.CopyModel(original, newmodel, list, tc);
        }
        return _thecopymode;
    }

    /// <summary>
    /// Dumps an entity under STEP form, i.e. as a part of a Step file Works with a StepDumper. Level 0 just displays
    /// type; level 1 displays the entity itself, and level 2 displays the entity plus its shared ones (one sub-level:
    /// immediately shared entities)
    /// </summary>
    public override void DumpEntity(InterfaceModel model, Protocol protocol, Transient entity, TextWriter s,
                                    int level)
    {
        int nument = model.Number(entity);
        if (nument <= 0 || nument > model.EntityCount)
        {
            return;
        }
        bool iserr = model.IsRedefinedContent(nument);
        Transient ent, con;
        ent = entity;
        s.Write(" --- (STEP) Entity ");
        model.Print(entity, s);
        if (iserr)
        {
            con = model.ReportEntity(nument).Content();
        }
        if (entity is null)
        {
            s.Write(" Null\n");
            return;
        }

        //  On attaque le dump : d abord cas de l Erreur
        s.Write(" Type cdl : " + entity.DynamicTypeName() + "\n");
        if (iserr)
        {
            s.Write(" ***  NOT WELL LOADED : CONTENT FROM FILE  ***\n");
        }
        else if (model.IsUnknownEntity(nument))
        {
            s.Write(" ***  UNKNOWN TYPE  ***\n");
        }

        StepDumper dump = new StepDumper(model as StepModel, protocol as StepProtocol, _thelabmode);
        dump.Dump(s, ent, level);
    }

    private bool _thecopymode;
    private int _thelabmode;
}
