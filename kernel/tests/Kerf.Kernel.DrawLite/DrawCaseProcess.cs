// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using System.Globalization;
using System.Text.RegularExpressions;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// Runs a corpus case in a process of its own, as OCCT's harness runs each case in a new DRAWEXE. The cases that read
/// the process's own figures need that here (decision 1699): the managed heap and the processor time belong to the
/// process, and in the tests' process they count everything else that's running. See docs/design-rules.md, section 13.
/// </summary>
internal static class DrawCaseProcess
{
    /// <summary>What a case reads from its process, so what the process it runs in has to do for it.</summary>
    internal enum Reading
    {
        /// <summary>Nothing: it can run beside others.</summary>
        None,

        /// <summary>
        /// Only that it's a new process: <c>meminfo</c> of anything but the heap, such as the peak working set, or a section
        /// timed against a fixed figure (bug83), which passes or fails by whether the code is compiled yet, and so has to start
        /// from the same place each time, as DRAWEXE does.
        /// </summary>
        Process,

        /// <summary><c>meminfo h</c>: the live heap, which has to hold only what the case keeps.</summary>
        Heap,

        /// <summary><c>bbuild -t</c>, against a fixed figure: the time has to be the case's own work.</summary>
        BuildTime,
    }

    private const string RunCaseArgument = "--run-case";

    // the exit code of a run that wrote its result
    private const int Written = 0;

    // on top of the case's own limit, so that a process that stops itself at the limit can report it
    private static readonly TimeSpan s_killMargin = TimeSpan.FromSeconds(60);

    private static readonly Regex s_heapReading = new(@"\bmeminfo\s+h(eap)?\b", RegexOptions.CultureInvariant);
    private static readonly Regex s_anyReading = new(@"\bmeminfo\b|is performed too slow", RegexOptions.CultureInvariant);
    private static readonly Regex s_buildTime = new(@"\bbbuild\b[^\n]*-t\b(?s:.*)Error: low performance", RegexOptions.CultureInvariant);

    /// <summary>What this process is running for, set only by <see cref="Main"/>.</summary>
    internal static Reading Own { get; private set; } = Reading.None;

    /// <summary>
    /// Whether parsed scripts and expressions stay in the interpreter's process-wide caches. A script's text repeated
    /// with new numbers in it, as <c>checktrend</c>'s are, fills them by kilobytes a cycle, which the heap reading would
    /// call the case's own.
    /// </summary>
    internal static bool KeepsParsedText => Own != Reading.Heap;

    /// <summary>A moment on both of <c>bbuild -t</c>'s clocks.</summary>
    internal readonly record struct BuildClock(long Elapsed, TimeSpan Processor)
    {
        public static BuildClock Now => new(Stopwatch.GetTimestamp(), Environment.CpuUsage.TotalTime);
    }

    /// <summary>
    /// The time <c>bbuild -t</c> reports: elapsed time, as OCCT's, except where a case compares it with a fixed figure,
    /// where it's this process's processor time, which the machine's other work doesn't add to.
    /// </summary>
    internal static double SecondsSince(in BuildClock theStart)
        => Own == Reading.BuildTime ? (Environment.CpuUsage.TotalTime - theStart.Processor).TotalSeconds
                                    : Stopwatch.GetElapsedTime(theStart.Elapsed).TotalSeconds;

    /// <summary>What the script of a case reads from its process.</summary>
    internal static Reading ReadingOf(string theScript)
        => s_heapReading.IsMatch(theScript) ? Reading.Heap
           : s_anyReading.IsMatch(theScript) ? Reading.Process
           : s_buildTime.IsMatch(theScript) ? Reading.BuildTime
           : Reading.None;

    /// <summary>
    /// Runs the case in a new process, which runs and judges it as <see cref="DrawCorpus.RunAndCheck"/> does and leaves
    /// the status and the log in a file. It takes the corpus's time scale, and leaves out a coverage profiler's
    /// variables: its code isn't counted, and these are a handful of cases.
    /// </summary>
    internal static DrawCaseRun Run(DrawCorpus theCorpus, DrawTestCase theCase, Reading theReading, string theResourcesDirectory, string? theDataPath, TimeSpan theLimit)
    {
        string aResult = Path.Combine(Path.GetTempPath(), $"drawlite-case-{Guid.NewGuid():N}.txt");
        var aStart = new ProcessStartInfo(Environment.GetEnvironmentVariable("DOTNET_HOST_PATH") ?? "dotnet")
        {
            UseShellExecute = false,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
        };
        foreach (string aVariable in new[] { "CORECLR_ENABLE_PROFILING", "COR_ENABLE_PROFILING", "CORECLR_PROFILER", "COR_PROFILER" })
        {
            aStart.Environment.Remove(aVariable);
        }

        aStart.Environment[DrawCorpus.TimeScaleVariable] = theCorpus.TimeScale.ToString(CultureInfo.InvariantCulture);

        aStart.ArgumentList.Add(typeof(DrawCaseProcess).Assembly.Location);
        foreach (string anArgument in new[]
                 {
                     RunCaseArgument, theReading.ToString(), theCorpus.TestsDirectory, theCase.Group, theCase.Grid, theCase.Name, theCase.Path,
                     theResourcesDirectory, theDataPath ?? "", theLimit.TotalSeconds.ToString(CultureInfo.InvariantCulture), aResult,
                 })
        {
            aStart.ArgumentList.Add(anArgument);
        }

        try
        {
            using Process aProcess = Process.Start(aStart)!;
            Task<string> anOutput = aProcess.StandardOutput.ReadToEndAsync();
            Task<string> anErrors = aProcess.StandardError.ReadToEndAsync();
            if (!aProcess.WaitForExit(theLimit * theCorpus.TimeScale + s_killMargin))
            {
                aProcess.Kill(entireProcessTree: true);
                return new DrawCaseRun(theCase, "FAILED (the case's process outlived its limit)", "Process killed by elapsed limit\n");
            }

            if (aProcess.ExitCode != Written || !File.Exists(aResult))
            {
                return new DrawCaseRun(theCase, $"FAILED (the case's process exited with {aProcess.ExitCode})", anOutput.Result + anErrors.Result);
            }

            string aText = File.ReadAllText(aResult);
            int aLineEnd = aText.IndexOf('\n', StringComparison.Ordinal);
            return new DrawCaseRun(theCase, aText[..aLineEnd], aText[(aLineEnd + 1)..]);
        }
        finally
        {
            File.Delete(aResult);
        }
    }

    /// <summary>
    /// The process's entry: <c>--run-case reading tests group grid name path resources data limit result</c>. A case that
    /// reads the heap runs twice and is judged on the second: what a first run fills once (static tables, the runtime's
    /// own caches) is what OCCT's bug31546 warns of, and a repeat doesn't keep adding to it.
    /// </summary>
    internal static int Main(string[] theArguments)
    {
        if (theArguments.Length != 11 || theArguments[0] != RunCaseArgument)
        {
            Console.Error.WriteLine("Kerf.Kernel.DrawLite runs one corpus case for DrawCorpus: it isn't meant to be started by hand.");
            return 2;
        }

        Own = Enum.Parse<Reading>(theArguments[1]);
        var aCorpus = new DrawCorpus(theArguments[2]);
        var aCase = new DrawTestCase(theArguments[3], theArguments[4], theArguments[5], theArguments[6]);
        string aResources = theArguments[7];
        string? aData = theArguments[8].Length > 0 ? theArguments[8] : null;
        TimeSpan aLimit = TimeSpan.FromSeconds(double.Parse(theArguments[9], CultureInfo.InvariantCulture));
        if (Own == Reading.Heap)
        {
            aCorpus.RunAndCheck(aCase, aResources, aData, aLimit, null);
        }

        DrawCaseRun aRun = aCorpus.RunAndCheck(aCase, aResources, aData, aLimit, null);
        File.WriteAllText(theArguments[10], aRun.Status.ReplaceLineEndings(" ") + "\n" + aRun.Log);
        return Written;
    }
}
