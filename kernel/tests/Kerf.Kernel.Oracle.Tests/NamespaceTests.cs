// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Kernel.Oracle.Tests;

public class NamespaceTests
{
    /// <summary>
    /// A namespace named after an OCCT package (Kerf.Kernel.Oracle.Tests.Geom, ...BSplCLib) hides the OCCT class of the same
    /// name from every other file, so all oracle tests live in the root namespace (docs/oracle-testing.md).
    /// </summary>
    [Fact]
    public void Every_type_is_in_the_root_namespace()
    {
        var aStrays = typeof(NamespaceTests).Assembly.GetTypes()
                                            .Where(t => t.Namespace?.StartsWith("Kerf.Kernel.Oracle.Tests.", StringComparison.Ordinal) == true)
                                            .Select(t => t.FullName)
                                            .ToList();

        Assert.True(aStrays.Count == 0, "Move these into namespace Kerf.Kernel.Oracle.Tests: " + string.Join(", ", aStrays));
    }
}
