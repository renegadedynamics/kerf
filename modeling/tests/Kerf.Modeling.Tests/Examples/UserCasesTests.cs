using System.Diagnostics;
using Xunit.Abstractions;

namespace Kerf.Modeling.Tests.Examples;

/// <summary>
/// Users' cases (kernel/benchmarks/Kerf.UserCases, unit 13C): each gives its fixed result on this tree, twice from the
/// same operands as the rig and the release check run them, so a change that moves one fails here before they time it.
/// </summary>
/// <remarks>
/// The program runs in a process of its own, as the rig and the release check run it, so its thousands of operations
/// don't warm this one's runtime for other tests (decision 25303).
/// </remarks>
public class UserCasesTests(ITestOutputHelper output)
{
    [Fact]
    public async Task Each_gives_its_fixed_result()
    {
        var program = Path.Combine(AppContext.BaseDirectory, "Kerf.UserCases.dll");
        var start = new ProcessStartInfo("dotnet")
        {
            ArgumentList = { program, "check", "--data", Path.Combine(AppContext.BaseDirectory, "data") },
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
        };
        using var process = Process.Start(start)!;
        using var limit = new CancellationTokenSource(TimeSpan.FromMinutes(10));
        var errors = process.StandardError.ReadToEndAsync(limit.Token);
        string text;
        try
        {
            text = await process.StandardOutput.ReadToEndAsync(limit.Token);
            await process.WaitForExitAsync(limit.Token);
        }
        catch (OperationCanceledException)
        {
            // This test started the process: stopping it touches nothing else
            process.Kill();
            throw new TimeoutException("the cases took over 10 minutes");
        }
        output.WriteLine(text);
        var errorText = await errors;
        output.WriteLine(errorText);
        Assert.True(process.ExitCode == 0, text + errorText);
        Assert.Matches(@"(\d+) of \1 cases give their fixed result", text);
    }
}
