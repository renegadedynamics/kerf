// Port of: src/FoundationClasses/TKMath/GTests/math_ComputeKronrodPointsAndWeights_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class math_ComputeKronrodPointsAndWeightsTest
{
    [Fact]
    public void OCC33048_ComputeWithOrder125()
    {
        // This method uses raw pointers for memory manipulations in OCCT
        // Test that it completes successfully without crashes
        math_ComputeKronrodPointsAndWeights aCalc = new(125);
        Assert.True(aCalc.IsDone());
    }
}
