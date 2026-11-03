// Port of: src/ModelingAlgorithms/TKShHealing/GTests/ShapeUpgrade_FaceDivide_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTests of ShapeUpgrade_FaceDivide (issue #1179: a null context, and concurrent use).</summary>
public class ShapeUpgrade_FaceDivideTest
{
    // Test for issue #1179: ShapeUpgrade_FaceDivide crashes with null context.
    [Fact]
    public void Perform_WithoutContext_DoesNotCrash()
    {
        for (int i = 0; i < 10; ++i)
        {
            double aSize = 10.0 + i * 0.5;
            TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(aSize, aSize + 5, aSize + 10).Shape();
            var anExp = new TopExp_Explorer(aBox, TopAbs_FACE);
            Assert.True(anExp.More(), "No faces found in box");

            var aDivider = new ShapeUpgrade_FaceDivide(TopoDS.Face(anExp.Current()));
            Exception? anException = Record.Exception(() => aDivider.Perform());
            Assert.Null(anException);
        }
    }

    // Test for issue #1179: ShapeUpgrade_FaceDivide is safe to call concurrently on independent faces.
    [Fact]
    public void Perform_Parallel_DoesNotCrash()
    {
        bool anError = false;
        OsdParallel.For(0, 100, theIndex =>
        {
            double aSize = 10.0 + theIndex * 0.1;
            TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(aSize, aSize + 5, aSize + 10).Shape();
            var anExp = new TopExp_Explorer(aBox, TopAbs_FACE);
            if (!anExp.More())
            {
                Volatile.Write(ref anError, true);
                return;
            }
            try
            {
                var aDivider = new ShapeUpgrade_FaceDivide(TopoDS.Face(anExp.Current()));
                aDivider.Perform();
            }
            catch
            {
                Volatile.Write(ref anError, true);
            }
        });

        Assert.False(anError, "FaceDivide failed in parallel execution");
    }
}
