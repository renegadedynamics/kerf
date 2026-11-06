// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Step.Caf;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Parser;
using Kerf.Engine.Foundation;

namespace Kerf.Kernel.Benchmarks;

// What a STEP import holds at each stage (unit 11Q, sixth slice): `-- step-phases file`. The peak of an import is what's live during
// the transfer, so this reads what the reader and its document hold, by type and by the field that reached it, after parsing and after the transfer.
internal static class StepPhases
{
    public static void Run(string file)
    {
        Parse(file);
        var reader = new CafReader();
        reader.SetColorMode(true);
        reader.SetNameMode(true);
        reader.SetLayerMode(true);
        long start = Live();
        using (var stream = File.OpenRead(file))
            reader.ReadStream(file, stream, new ProgressRange(), new ParserReadLimits());
        Console.WriteLine($"after parsing: live {(Live() - start) / 1048576.0:F1} MB");
        Report(reader, null);
        var document = new ExchangeDocument();
        reader.Transfer(document);
        Console.WriteLine($"after the transfer: live {(Live() - start) / 1048576.0:F1} MB");
        Report(reader, document);
        GC.KeepAlive(reader);
        GC.KeepAlive(document);
    }

    // Before the entities are made: the parser's records, then the reader's parameters, which last until the entities are done
    private static void Parse(string file)
    {
        long start = Live();
        var data = new ReadData();
        using (var stream = File.OpenRead(file))
        {
            var parser = new StepParser(new Scanner(data, stream, new ParserReadLimits()), new ProgressRange());
            parser.Parse();
        }
        Console.WriteLine($"the parser's records: live {(Live() - start) / 1048576.0:F1} MB");
        Report(data, null);
        data.GetFileNbR(out int nbhead, out int nbrec, out int nbpar);
        data.WalkOnce();
        var undirec = new StepReaderData(nbhead, nbrec, nbpar, default);
        for (int nr = 1; nr <= nbrec; nr++)
        {
            data.GetRecordDescription(out string? ident, out string? typrec, out int nbarg);
            undirec.SetRecord(nr, ident!, typrec!, nbarg);
            if (nbarg > 0)
            {
                while (data.GetArgDescription(out Kerf.Engine.Exchange.Framework.ParamType typa, out string? val))
                    undirec.AddStepParam(nr, val!, typa);
            }
            undirec.InitParams(nr);
            data.NextRecord();
        }
        data.ClearRecorder(1);
        Console.WriteLine($"the reader's {nbpar} parameters: live {(Live() - start) / 1048576.0:F1} MB");
        Report(undirec, null);
        GC.KeepAlive(data);
        GC.KeepAlive(undirec);
    }

    private static long Live()
    {
        for (int i = 0; i < 2; i++)
        {
            GC.Collect(GC.MaxGeneration, GCCollectionMode.Forced, true, true);
            GC.WaitForPendingFinalizers();
        }
        return GC.GetTotalMemory(true);
    }

    private static void Report(object reader, object? document)
    {
        var walk = new HeapWalk();
        walk.Add(reader, "reader");
        walk.Add(document, "document");
        Console.WriteLine($"  walked {walk.Bytes / 1048576.0:F1} MB in {walk.Objects} objects");
        foreach (var t in walk.ByType.OrderByDescending(t => t.Value.Bytes).Take(12))
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"    {t.Value.Bytes / 1048576.0,8:F2} MB {100.0 * t.Value.Bytes / walk.Bytes,5:F1}% {t.Value.Count,9}  {t.Key}"));
        Console.WriteLine("  by the field that reached it first");
        foreach (var t in walk.ByHolder.OrderByDescending(t => t.Value.Bytes).Take(12))
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"    {t.Value.Bytes / 1048576.0,8:F2} MB {100.0 * t.Value.Bytes / walk.Bytes,5:F1}% {t.Value.Count,9}  {t.Key}"));
    }
}
