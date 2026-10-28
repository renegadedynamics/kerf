// Port of: src/FoundationClasses/TKMath/GTests/gp_Mat_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class gp_MatTest
{
    [Fact]
    public void OCC22595_DefaultConstructor()
    {
        gp_Mat aM0 = new();

        // Bug OCC22595: gp_Mat's constructors incompletely initialize memory
        // Test that default constructor properly initializes all matrix elements to zero
        Assert.Equal(0.0, aM0[1, 1]);
        Assert.Equal(0.0, aM0[1, 2]);
        Assert.Equal(0.0, aM0[1, 3]);

        Assert.Equal(0.0, aM0[2, 1]);
        Assert.Equal(0.0, aM0[2, 2]);
        Assert.Equal(0.0, aM0[2, 3]);

        Assert.Equal(0.0, aM0[3, 1]);
        Assert.Equal(0.0, aM0[3, 2]);
        Assert.Equal(0.0, aM0[3, 3]);
    }
}
