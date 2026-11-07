// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;

namespace Kerf.Kernel.Reliability.Tests;

/// <summary>
/// Runs a scenario of <see cref="Scenarios"/> in a process of its own, with a time limit and a limit on its heap: it can
/// overflow its stack, run out of memory or spin, and only that process goes.
/// </summary>
internal static class ChildProcess
{
    private const int StackBytes = 1 << 20;
    private const string HeapLimit = "0x80000000"; // 2 GB

    /// <summary>What the scenario returned; the test fails if the process died or passed its limit.</summary>
    public static string Run(string scenario, int limitSeconds = 120)
    {
        var start = new ProcessStartInfo(Environment.GetEnvironmentVariable("DOTNET_HOST_PATH") ?? "dotnet")
        {
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
        };
        start.ArgumentList.Add("exec");
        start.ArgumentList.Add(typeof(ChildProcess).Assembly.Location);
        start.ArgumentList.Add(scenario);
        start.Environment["DOTNET_GCHeapHardLimit"] = HeapLimit;
        using Process process = Process.Start(start)!;
        Task<string> output = process.StandardOutput.ReadToEndAsync();
        Task<string> errors = process.StandardError.ReadToEndAsync();
        if (!process.WaitForExit(limitSeconds * 1000))
        {
            process.Kill(entireProcessTree: true);
            Assert.Fail($"{scenario} didn't return within {limitSeconds} s");
        }

        string error = errors.Result;
        Assert.True(process.ExitCode == 0, $"{scenario}: the process ended with 0x{process.ExitCode:X8}: {error[..Math.Min(error.Length, 500)]}");
        return output.Result.Trim();
    }

    /// <summary>The child process: the scenario, on a thread with a stack of the size a thread commonly has.</summary>
    internal static int Main(string[] arguments)
    {
        string result = "";
        var thread = new Thread(() => result = Scenarios.All[arguments[0]](), StackBytes);
        thread.Start();
        thread.Join();
        Console.WriteLine(result);
        return 0;
    }
}
