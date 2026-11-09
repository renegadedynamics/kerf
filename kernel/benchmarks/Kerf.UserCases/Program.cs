// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Users' cases (unit 13C, README.md): built against this tree's Kerf.Modeling, or a released one's packages with
// -p:KerfPackageVersion=. From the kernel folder:
//   dotnet run -c Release --project benchmarks/Kerf.UserCases -- check      each case twice, against its fixed result; exit 1 if any differs
//   dotnet run -c Release --project benchmarks/Kerf.UserCases -- rig [runs] the rig's rows (tools/bench.ps1), median of [runs], 9 by default
//   dotnet run -c Release --project benchmarks/Kerf.UserCases -- serve      for tools/release-check.ps1: "run <case>" on each line in, a line out
//   dotnet run -c Release --project benchmarks/Kerf.UserCases -- expected   Expected.cs as this build gives it, to stdout
//   dotnet run -c Release --project benchmarks/Kerf.UserCases -- list
// Every mode takes the data folder as --data <folder>; by default the one copied beside the program.

using System.Diagnostics;
using System.Globalization;
using System.Reflection;
using System.Text;
using Kerf.Modeling;
using Kerf.UserCases;

CultureInfo.DefaultThreadCurrentCulture = CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;
var arguments = args.ToList();
var data = Path.Combine(AppContext.BaseDirectory, "data");
var dataAt = arguments.IndexOf("--data");
if (dataAt >= 0 && dataAt + 1 < arguments.Count)
{
    data = Path.GetFullPath(arguments[dataAt + 1]);
    arguments.RemoveRange(dataAt, 2);
}
var mode = arguments.Count > 0 ? arguments[0] : "check";
var kerfVersion = typeof(Part).Assembly.GetCustomAttribute<AssemblyInformationalVersionAttribute>()?.InformationalVersion ?? "unknown";

switch (mode)
{
    case "list":
        foreach (var c in UserCases.All)
        {
            Console.WriteLine($"{c.Name}\t{c.Description}");
        }
        return 0;
    case "check":
        {
            // Each case twice from the same operands, as the timed modes run them: the second run has to build its
            // shapes again (the enclosure keeps its parts in static fields) and give the same result
            var differ = 0;
            foreach (var c in UserCases.All)
            {
                var operation = c.Prepare(data);
                if (operation is null)
                {
                    differ++;
                    Console.WriteLine($"DIFFERS {c.Name}: not in this version");
                    continue;
                }
                var first = operation();
                var second = operation();
                var result = c.Read(second);
                var rebuilt = !ReferenceEquals(first, second) &&
                    (first is not List<(string, Part)> a || second is not List<(string, Part)> b || !a.Zip(b).Any(p => ReferenceEquals(p.First.Item2, p.Second.Item2)));
                var matches = c.Read(first).Matches(c.Expected) && result.Matches(c.Expected) && rebuilt;
                differ += matches ? 0 : 1;
                Console.WriteLine($"{(matches ? "ok     " : "DIFFERS")} {c.Name}: {result.Summary()}{(rebuilt ? "" : " (not built again)")}");
            }
            Console.WriteLine($"Kerf.Modeling {kerfVersion}: {UserCases.All.Count - differ} of {UserCases.All.Count} cases give their fixed result");
            return differ == 0 ? 0 : 1;
        }
    case "expected":
        Console.Write(ExpectedSource(data));
        return 0;
    case "rig":
        Rig(data, arguments.Count > 1 ? int.Parse(arguments[1], CultureInfo.InvariantCulture) : 9);
        return 0;
    case "serve":
        Serve(data, kerfVersion);
        return 0;
    default:
        Console.Error.WriteLine("usage: Kerf.UserCases [check | rig [runs] | serve | expected | list] [--data <folder>]");
        return 2;
}

// One timed run: a collection first, so one run's garbage isn't another's; the time and the bytes allocated on every
// thread, then the result, read untimed
static (double Milliseconds, long Bytes, CaseResult Result) Time(UserCase userCase, Func<object> operation)
{
    GC.Collect();
    GC.WaitForPendingFinalizers();
    GC.Collect();
    var bytes = GC.GetTotalAllocatedBytes(precise: true);
    var clock = Stopwatch.StartNew();
    object outcome;
    try
    {
        outcome = operation();
    }
    catch (Exception e)
    {
        outcome = $"{e.GetType().Name}: {e.Message}";
    }
    var milliseconds = clock.Elapsed.TotalMilliseconds;
    bytes = GC.GetTotalAllocatedBytes(precise: true) - bytes;
    var result = outcome is string failure ? CaseResult.Failed(failure) : userCase.Read(outcome);
    return (milliseconds, bytes, result);
}

// The rig's rows (tools/bench.ps1): each case warmed twice, then the median of its runs, in group "user cases"
static void Rig(string data, int runs)
{
    var rowFile = Environment.GetEnvironmentVariable("KERF_BENCH_ROWS");
    void Row(string name, string metric, string value)
    {
        if (rowFile is not null)
        {
            File.AppendAllText(rowFile, $"user cases\t{name}\t{metric}\t{value}\n");
        }
    }
    foreach (var c in UserCases.All)
    {
        var operation = c.Prepare(data);
        if (operation is null)
        {
            continue;
        }
        Time(c, operation);
        Time(c, operation);
        var measured = Enumerable.Range(0, runs).Select(_ => Time(c, operation)).ToList();
        var milliseconds = measured.Select(m => m.Milliseconds).Order().ToList();
        var megabytes = measured.Select(m => m.Bytes / 1e6).Order().ToList();
        // The lower of two middles, as the rig takes medians
        var ms = milliseconds[(runs - 1) / 2];
        var mb = megabytes[(runs - 1) / 2];
        var result = measured[^1].Result;
        Row(c.Name, "ms", ms.ToString("F3", CultureInfo.InvariantCulture));
        Row(c.Name, "alloc MB", mb.ToString("F3", CultureInfo.InvariantCulture));
        Row(c.Name, "result", result.Summary() + (result.Matches(c.Expected) ? "" : " (not its fixed result)"));
        Console.WriteLine($"{c.Name,-20} {ms,9:F1} ms {mb,9:F1} MB  {result.Summary()}");
    }
}

// The release check's side of a conversation (tools/release-check.ps1): "run <case>" on a line, and one line back,
// tab-separated: the case, milliseconds, bytes allocated, whether it gave its fixed result, and the result exactly.
// Each case's operands are read on its first run, untimed
static void Serve(string data, string kerfVersion)
{
    var prepared = new Dictionary<string, Func<object>?>();
    var output = new StreamWriter(Console.OpenStandardOutput(), new UTF8Encoding(false)) { AutoFlush = true, NewLine = "\n" };
    output.WriteLine($"ready\t{kerfVersion}\t{Environment.Version}");
    string? line;
    while ((line = Console.ReadLine()) is not null)
    {
        // Windows PowerShell's pipe to us starts with a byte order mark
        line = line.Trim().TrimStart('﻿');
        if (line == "quit")
        {
            return;
        }
        if (!line.StartsWith("run ", StringComparison.Ordinal))
        {
            output.WriteLine($"error\tunknown command: {line}");
            continue;
        }
        var name = line[4..].Trim();
        var c = UserCases.All.FirstOrDefault(u => u.Name == name);
        if (c is null)
        {
            output.WriteLine($"{name}\t0\t0\tno\tabsent");
            continue;
        }
        if (!prepared.TryGetValue(name, out var operation))
        {
            operation = c.Prepare(data);
            prepared[name] = operation;
        }
        if (operation is null)
        {
            output.WriteLine($"{name}\t0\t0\tno\tabsent");
            continue;
        }
        var (milliseconds, bytes, result) = Time(c, operation);
        output.WriteLine(string.Join('\t', name, milliseconds.ToString("R", CultureInfo.InvariantCulture),
            bytes.ToString(CultureInfo.InvariantCulture), result.Matches(c.Expected) ? "yes" : "no", result.Exact()));
    }
}

// Expected.cs as this build gives it: what a deliberate change of result rewrites, with its decision
static string ExpectedSource(string data)
{
    static string Text(double value) => value.ToString("R", CultureInfo.InvariantCulture);
    var text = new StringBuilder();
    text.Append("// Written by `Kerf.UserCases expected`; change it only with a decision that says why the result changed.\r\n\r\n");
    text.Append("namespace Kerf.UserCases;\r\n\r\n");
    text.Append("internal static class Expected\r\n{\r\n");
    text.Append("    public static CaseResult Of(string name) => name switch\r\n    {\r\n");
    foreach (var c in UserCases.All)
    {
        var result = UserCases.RunOnce(c, data);
        if (result.IsAbsent || result.Failure is not null)
        {
            throw new InvalidOperationException($"{c.Name} gave no shapes: {result.Exact()}");
        }
        text.Append($"        \"{c.Name}\" => new(\r\n        [\r\n");
        foreach (var m in result.Measures)
        {
            text.Append($"            new(\"{m.Label}\", {Text(m.Volume)}, {(m.IsValid ? "true" : "false")}, {m.Faces}),\r\n");
        }
        text.Append("        ], null),\r\n");
    }
    text.Append("        _ => CaseResult.Absent,\r\n    };\r\n}\r\n");
    return text.ToString();
}
