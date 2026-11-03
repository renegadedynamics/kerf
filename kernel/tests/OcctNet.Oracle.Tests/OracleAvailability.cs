// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

[assembly: OcctNative.NativeTape]

namespace OcctNet.Oracle.Tests;

public class OracleAvailability
{
    /// <summary>
    /// Fails loudly rather than letting a missing oracle pass as "no differences": native OCCT's version, as recorded
    /// (or live, when recording).
    /// </summary>
    [Fact]
    public void Native_OCCT_is_available()
    {
        Assert.StartsWith("8.0.1", N.Occt.Version);
#if !ORACLE_DIRECT
        if (!Tape.IsRecording)
        {
            Assert.Contains(typeof(OracleAvailability).Assembly.GetManifestResourceNames(), n => n.EndsWith(".tape.gz", StringComparison.Ordinal));
        }
#endif
    }
}
