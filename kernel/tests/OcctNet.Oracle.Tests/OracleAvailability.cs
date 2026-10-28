// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Oracle.Tests;

public class OracleAvailability
{
    /// <summary>Fails loudly rather than letting a missing oracle pass as "no differences".</summary>
    [Fact]
    public void Native_OCCT_is_available()
    {
#if HAS_ORACLE
        Assert.StartsWith("8.0.1", Build123d.Occt.Occt.Version);
#else
        Assert.Fail("No oracle: the oracle tests need a sibling build123d.NET checkout with its native build "
                    + "(see docs/oracle-testing.md). Set Build123dNetRoot to point at another checkout.");
#endif
    }
}
