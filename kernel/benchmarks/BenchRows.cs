// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Kerf.Kernel.Benchmarks;

/// <summary>
/// The figures tools/bench.ps1 keeps: tab-separated rows of group, name, metric and value, appended to the file the
/// environment variable <c>KERF_BENCH_ROWS</c> names, and nothing without it. The metric's last word says how the rig
/// compares it: ms and us are times, MB and KB allocation, x higher is better, count is for information, and result has
/// to stay the same.
/// </summary>
internal static class BenchRows
{
    private static readonly string? TheFile = Environment.GetEnvironmentVariable("KERF_BENCH_ROWS");

    public static void Add(string group, string name, string metric, double value)
        => Add(group, name, metric, value.ToString("F3", CultureInfo.InvariantCulture));

    public static void Add(string group, string name, string metric, string value)
    {
        if (TheFile is not null)
        {
            File.AppendAllText(TheFile, $"{group}\t{name}\t{metric}\t{value}\n");
        }
    }
}
