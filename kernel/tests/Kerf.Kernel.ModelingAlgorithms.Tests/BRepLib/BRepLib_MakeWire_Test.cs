// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepLib_MakeWire_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

public class BRepLib_MakeWire_Test
{
    [Fact]
    public void OCC30708_2_InitializeWithNullWire()
    {
        Wire empty = new();

        // Should not throw exception when initializing with null wire
        var aWBuilder = new LibraryMakeWire(empty);
        Assert.NotNull(aWBuilder);
    }
}
