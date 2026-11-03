// Port of: src/ModelingAlgorithms/TKPrim/GTests/BRepPrimAPI_MakePrism_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

public class BRepPrimAPI_MakePrism_Test
{
    [Fact]
    public void OCC31294_GeneratedListForNonBaseShape()
    {
        // Bug OCC31294: Modeling Algorithms - Regression relatively 7.3.0.
        // Crash in method BRepPrimAPI_MakePrism::Generated(...) if input sub-shape
        // does not belong to the base shape
        //
        // This test verifies that calling Generated() with a shape that doesn't belong
        // to the base shape doesn't crash and returns an empty list

        var aMkVert = new MakeVertex(new Point3D(0.0, 0.0, 0.0));
        var aMkDummy = new MakeVertex(new Point3D(0.0, 0.0, 0.0));
        var aMkPrism = new ApiMakePrism(aMkVert.Shape(), new Vector3D(0.0, 0.0, 1.0));

        // Check that Generated() returns 1 shape for the vertex used to create the prism
        int aNbGen = aMkPrism.Generated(aMkVert.Shape()).Count;
        Assert.Equal(1, aNbGen);

        // Check that Generated() returns 0 shapes for a vertex not used to create the prism
        // (this should not crash)
        int aNbDummy = aMkPrism.Generated(aMkDummy.Shape()).Count;
        Assert.Equal(0, aNbDummy);
    }
}
