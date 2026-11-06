// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Step.Caf;
using Kerf.Engine.Exchange.Step.Parser;
using Kerf.Engine.Foundation;

namespace Kerf.Kernel.Benchmarks;

// What a STEP import holds at each stage (unit 11Q, sixth slice): `-- step-phases file`. The peak of an import is what's live during
// the transfer, so this reads what the reader and its document hold, by type and by the field that reached it, after parsing and after the transfer.
internal static class StepPhases
{
    public static void Run(string file)
    {
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
        foreach (var t in walk.ByType.OrderByDescending(t => t.Value.Bytes).Take(25))
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"    {t.Value.Bytes / 1048576.0,8:F2} MB {100.0 * t.Value.Bytes / walk.Bytes,5:F1}% {t.Value.Count,9}  {t.Key}"));
        Console.WriteLine("  by the field that reached it first");
        foreach (var t in walk.ByHolder.OrderByDescending(t => t.Value.Bytes).Take(25))
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"    {t.Value.Bytes / 1048576.0,8:F2} MB {100.0 * t.Value.Bytes / walk.Bytes,5:F1}% {t.Value.Count,9}  {t.Key}"));
    }
}
