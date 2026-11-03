// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Text;
using OcctNet.DataExchange.Tests.Interface;
using OcctNet.DataExchange.Tests.Transfer;

namespace OcctNet.DataExchange.Tests.IFSelect;

/// <summary>
/// The toy norm's work library: a file is a line per entity, "name ref ref ..." (refs by name, defined anywhere in the
/// file; a name starting with '!' fails its check, with '?' warns); writing writes the entities' names, a line each,
/// after a line per applied file modifier ("mod: label (all|n entities)").
/// </summary>
internal sealed class ToyLibrary : IFSelect_WorkLibrary
{
    public int NbReads;

    public static ToyModel Parse(TextReader theReader)
    {
        var aLines = new List<string[]>();
        var aByName = new Dictionary<string, ToyEntity>(StringComparer.Ordinal);
        for (string? aLine = theReader.ReadLine(); aLine is not null; aLine = theReader.ReadLine())
        {
            string[] aWords = aLine.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (aWords.Length == 0)
            {
                continue;
            }
            aLines.Add(aWords);
            string aName = aWords[0].TrimStart('!', '?');
            aByName[aName] = new ToyEntity(aName) { Fails = aWords[0][0] == '!', Warns = aWords[0][0] == '?' };
        }
        var aModel = new ToyModel();
        aModel.SetProtocol(ToyNorm.Protocol);
        foreach (string[] aWords in aLines)
        {
            ToyEntity anEnt = aByName[aWords[0].TrimStart('!', '?')];
            foreach (string aRef in aWords.Skip(1))
            {
                anEnt.Ref(aByName[aRef]);
            }
            aModel.AddEntity(anEnt);
        }
        return aModel;
    }

    public override int ReadFile(string name, ref Interface_InterfaceModel? model, Interface_Protocol? protocol,
                                 ProgressRange theProgress = default)
    {
        if (!File.Exists(name))
        {
            return -1;
        }
        using var aStream = File.OpenRead(name);
        return ReadStream(name, aStream, ref model, protocol, theProgress);
    }

    public override int ReadStream(string theName, Stream theIStream, ref Interface_InterfaceModel? model,
                                   Interface_Protocol? protocol, ProgressRange theProgress = default)
    {
        NbReads++;
        if (protocol is not ToyProtocol)
        {
            return 1;
        }
        model = Parse(new StreamReader(theIStream, Encoding.UTF8));
        return 0;
    }

    public override bool WriteFile(IFSelect_ContextWrite ctx)
    {
        var aText = new StringBuilder();
        for (int numod = 1; numod <= ctx.NbModifiers(); numod++)
        {
            ctx.SetModifier(numod);
            if (ctx.FileModifier() is { } aMod)
            {
                aText.Append("mod: ").Append(aMod.Label())
                     .Append(ctx.IsForAll() ? " (all)" : " (" + ctx.NbEntities() + " entities)").Append('\n');
            }
        }
        Interface_InterfaceModel aModel = ctx.Model()!;
        for (int i = 1; i <= aModel.NbEntities(); i++)
        {
            aText.Append(aModel.Value(i)).Append('\n');
        }
        byte[] aBytes = Encoding.UTF8.GetBytes(aText.ToString());
        if (ctx.OStream() is { } aStream)
        {
            aStream.Write(aBytes);
            return true;
        }
        if (string.IsNullOrEmpty(ctx.FileName()))
        {
            ctx.CCheck(0)!.AddFail("no file");
            return false;
        }
        File.WriteAllBytes(ctx.FileName(), aBytes);
        return true;
    }

    public override void DumpEntity(Interface_InterfaceModel? model, Interface_Protocol? protocol, Transient? entity,
                                    TextWriter S, int level)
    {
        S.Write("toy:" + entity + " level " + level);
    }
}

/// <summary>A model modifier: appends "*" to the names of the entities it is applied to (the result's entities).</summary>
internal sealed class ToyRename() : IFSelect_Modifier(false)
{
    public override void Perform(IFSelect_ContextModif ctx, Interface_InterfaceModel? target, Interface_Protocol? protocol,
                                 Interface_CopyTool TC)
    {
        for (ctx.Start(); ctx.More(); ctx.Next())
        {
            ((ToyEntity)ctx.ValueResult()!).Name += "*";
        }
    }

    public override string Label() => "Rename";
}

/// <summary>A file modifier (applied by the toy library when it writes).</summary>
internal sealed class ToyFileMod() : IFSelect_GeneralModifier(false)
{
    public override string Label() => "FileMod";
}

/// <summary>The toy norm's writing actor: a text becomes a ToyEntity named by it in upper case.</summary>
internal sealed class ToyWriteActor : Transfer_ActorOfFinderProcess
{
    public override bool Recognize(Transfer_Finder? start) =>
        start is Transfer_TransientMapper tm && tm.Value() is TCollection_HAsciiString;

    public override Transient? TransferTransient(Transient? ent, Transfer_FinderProcess? FP,
                                                          ProgressRange theProgress = default) =>
        ent is TCollection_HAsciiString aText ? new ToyEntity(aText.ToCString().ToUpperInvariant()) : null;
}

/// <summary>The toy norm's controller, recorded once under "ToyNorm" and "toy".</summary>
internal sealed class ToyController : XSControl_Controller
{
    public static readonly ToyController Instance = Create();

    private static ToyController Create()
    {
        var aCtl = new ToyController();
        aCtl.AutoRecord();
        return aCtl;
    }

    private ToyController() : base("ToyNorm", "toy")
    {
        myAdaptorLibrary = new ToyLibrary();
        myAdaptorProtocol = ToyNorm.Protocol;
        myAdaptorRead = new ToyActor();
        myAdaptorWrite = new ToyWriteActor();
        TraceStatic("toy.param", 2);
        SetModeWrite(0, 1);
        SetModeWriteHelp(1, "upper case");
    }

    public override Interface_InterfaceModel? NewModel()
    {
        var aModel = new ToyModel();
        aModel.SetProtocol(ToyNorm.Protocol);
        return aModel;
    }

    public override void InitStatics(Interface_StaticSet theStatics)
    {
        base.InitStatics(theStatics);
        if (theStatics.StartInit("toy"))
        {
            theStatics.Init("toy", "toy.param", 'i', "3");
        }
    }
}
