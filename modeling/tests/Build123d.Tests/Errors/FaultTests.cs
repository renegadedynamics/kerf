using Build123d.Occt;

namespace Build123d.Tests.Errors;

/// <summary>
/// A fault in native code (an access violation, say) comes back as an <see cref="OcctException"/> on whichever thread
/// it happens, rather than ending the process, and leaves the thread as it was.
/// </summary>
public class FaultTests
{
    [NativeFact]
    public void AnAccessViolationIsAnOcctException()
    {
        var e = Assert.Throws<OcctException>(() => Occt.Occt.FaultForTesting(0x10));
        Assert.True(e.IsFault);
        if (OperatingSystem.IsWindows())
        {
            // Typed by OCCT's per-thread translator
            Assert.Equal("OSD_Exception_ACCESS_VIOLATION", e.OcctType);
            Assert.Contains("0x0000000000000010", e.Message);
            Assert.Contains("READ", e.Message);
        }
    }

    [NativeFact]
    public void FaultsAreTypedOnEveryThread()
    {
        // Pool threads and new threads alike: the translator is installed on each the first time it calls in
        var types = new string[32];
        Parallel.For(0, types.Length, i => types[i] = Assert.Throws<OcctException>(() => Occt.Occt.FaultForTesting(0x10 + i)).OcctType);
        string? onNewThread = null;
        var thread = new Thread(() => onNewThread = Assert.Throws<OcctException>(() => Occt.Occt.FaultForTesting(0x20)).OcctType);
        thread.Start();
        thread.Join();
        var expected = OperatingSystem.IsWindows() ? "OSD_Exception_ACCESS_VIOLATION" : types[0];
        Assert.All(types, t => Assert.Equal(expected, t));
        Assert.Equal(expected, onNewThread);
    }

    [NativeFact]
    public void AThreadIsUnchangedByAFault()
    {
        var thread = new Thread(() =>
        {
            Assert.Throws<OcctException>(() => Occt.Occt.FaultForTesting(0x10));
            // .NET's own faults and floating point are as they were: no floating-point traps
            object? nothing = null;
            Assert.Throws<NullReferenceException>(() => nothing!.ToString());
            double zero = 0;
            Assert.Equal(double.PositiveInfinity, 1 / zero);
            Assert.True(double.IsNaN(zero / zero));
            // And OCCT works
            Assert.Equal(6, new Box(1, 2, 3).Volume, 9);
        });
        thread.Start();
        thread.Join();
    }

    [Fact]
    public void FloatingPointStaysUntrappedAfterOcctCalls()
    {
        _ = (new Box(1, 1, 1) - new Sphere(0.7)).Volume;
        double zero = 0, huge = double.MaxValue;
        Assert.Equal(double.PositiveInfinity, huge * 10);
        Assert.Equal(double.NegativeInfinity, -1 / zero);
    }
}
