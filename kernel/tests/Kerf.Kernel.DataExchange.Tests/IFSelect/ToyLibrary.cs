// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Foundation;
using System.Text;
using Kerf.Kernel.DataExchange.Tests.Interface;
using Kerf.Kernel.DataExchange.Tests.Transfer;

namespace Kerf.Kernel.DataExchange.Tests.IFSelect;

/// <summary>
/// The toy norm's work library: a file is a line per entity, "name ref ref ..." (refs by name, defined anywhere in the
/// file; a name starting with '!' fails its check, with '?' warns); writing writes the entities' names, a line each,
/// after a line per applied file modifier ("mod: label (all|n entities)").
/// </summary>
internal sealed class ToyLibrary : WorkLibrary
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
        aModel.SetProtocol(ToyNorm.ProtocolInstance);
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

    public override int ReadFile(string name, ref InterfaceModel? model, Protocol? protocol,
                                 ProgressRange theProgress = default)
    {
        if (!File.Exists(name))
        {
            return -1;
        }
        using var aStream = File.OpenRead(name);
        return ReadStream(name, aStream, ref model, protocol, theProgress);
    }

    public override int ReadStream(string theName, Stream theIStream, ref InterfaceModel? model,
                                   Protocol? protocol, ProgressRange theProgress = default)
    {
        NbReads++;
        if (protocol is not ToyProtocol)
        {
            return 1;
        }
        model = Parse(new StreamReader(theIStream, Encoding.UTF8));
        return 0;
    }

    public override bool WriteFile(ContextWrite ctx)
    {
        var aText = new StringBuilder();
        for (int numod = 1; numod <= ctx.ModifierCount; numod++)
        {
            ctx.SetModifier(numod);
            if (ctx.FileModifier() is { } aMod)
            {
                aText.Append("mod: ").Append(aMod.Label())
                     .Append(ctx.IsForAll ? " (all)" : " (" + ctx.EntityCount + " entities)").Append('\n');
            }
        }
        InterfaceModel aModel = ctx.Model()!;
        for (int i = 1; i <= aModel.EntityCount; i++)
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

    public override void DumpEntity(InterfaceModel? model, Protocol? protocol, Transient? entity,
                                    TextWriter S, int level)
    {
        S.Write("toy:" + entity + " level " + level);
    }
}

/// <summary>A model modifier: appends "*" to the names of the entities it is applied to (the result's entities).</summary>
internal sealed class ToyRename() : SelectionModifier(false)
{
    public override void Perform(ContextModif ctx, InterfaceModel? target, Protocol? protocol,
                                 CopyTool TC)
    {
        for (ctx.Start(); ctx.More(); ctx.Next())
        {
            ((ToyEntity)ctx.ValueResult()!).Name += "*";
        }
    }

    public override string Label() => "Rename";
}

/// <summary>A file modifier (applied by the toy library when it writes).</summary>
internal sealed class ToyFileMod() : GeneralModifier(false)
{
    public override string Label() => "FileMod";
}

/// <summary>The toy norm's writing actor: a text becomes a ToyEntity named by it in upper case.</summary>
internal sealed class ToyWriteActor : ActorOfFinderProcess
{
    public override bool Recognize(Finder? start) =>
        start is TransientMapper tm && tm.Value() is TCollection_HAsciiString;

    public override Transient? TransferTransient(Transient? ent, FinderProcess? FP,
                                                          ProgressRange theProgress = default) =>
        ent is TCollection_HAsciiString aText ? new ToyEntity(aText.ToCString().ToUpperInvariant()) : null;
}

/// <summary>The toy norm's controller, recorded once under "ToyNorm" and "toy".</summary>
internal sealed class ToyController : Controller
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
        _adaptorLibrary = new ToyLibrary();
        _adaptorProtocol = ToyNorm.ProtocolInstance;
        _adaptorRead = new ToyActor();
        _adaptorWrite = new ToyWriteActor();
        TraceStatic("toy.param", 2);
        SetModeWrite(0, 1);
        SetModeWriteHelp(1, "upper case");
    }

    public override InterfaceModel? NewModel()
    {
        var aModel = new ToyModel();
        aModel.SetProtocol(ToyNorm.ProtocolInstance);
        return aModel;
    }

    public override void InitStatics(StaticSet theStatics)
    {
        base.InitStatics(theStatics);
        if (theStatics.StartInit("toy"))
        {
            theStatics.Init("toy", "toy.param", 'i', "3");
        }
    }
}
