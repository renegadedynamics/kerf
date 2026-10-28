// Port of: src/FoundationClasses/TKMath/GTests/gp_Ax3_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class gp_Ax3_Test
{
    [Fact]
    public void OCC29406_SetDirectionPreservesOrientation()
    {
        // Test Main (Z) direction
        {
            // gp_Ax3::SetDirection() test
            gp_Ax3 anAx1 = new(), anAx2 = new(), anAx3 = new(), anAx4 = new();
            anAx3.ZReverse();
            anAx4.ZReverse();

            bool bDirect1 = anAx1.Direct();
            anAx1.SetDirection(gp.DX());
            // Coordinate system orientation should be preserved
            Assert.Equal(bDirect1, anAx1.Direct());
            Assert.True(gp.DX().IsEqual(anAx1.Direction(), Precision.Angular()));

            bool bDirect2 = anAx2.Direct();
            anAx2.SetDirection(-gp.DX());
            Assert.Equal(bDirect2, anAx2.Direct());
            Assert.True((-gp.DX()).IsEqual(anAx2.Direction(), Precision.Angular()));

            bool bDirect3 = anAx3.Direct();
            anAx3.SetDirection(gp.DX());
            Assert.Equal(bDirect3, anAx3.Direct());
            Assert.True(gp.DX().IsEqual(anAx3.Direction(), Precision.Angular()));

            bool bDirect4 = anAx4.Direct();
            anAx4.SetDirection(-gp.DX());
            Assert.Equal(bDirect4, anAx4.Direct());
            Assert.True((-gp.DX()).IsEqual(anAx4.Direction(), Precision.Angular()));

            // gp_Ax3::SetAxis() test
            gp_Ax3 anAx5 = new(), anAx6 = new();
            gp_Ax1 anAx0_1 = new(gp.Origin(), gp.DX());
            gp_Ax1 anAx0_2 = new(gp.Origin(), -gp.DX());

            bool bDirect5 = anAx5.Direct();
            anAx5.SetAxis(anAx0_1);
            Assert.Equal(bDirect5, anAx5.Direct());
            Assert.True(anAx0_1.Direction().IsEqual(anAx5.Direction(), Precision.Angular()));

            bool bDirect6 = anAx6.Direct();
            anAx6.SetAxis(anAx0_2);
            Assert.Equal(bDirect6, anAx6.Direct());
            Assert.True(anAx0_2.Direction().IsEqual(anAx6.Direction(), Precision.Angular()));
        }

        // Test X direction
        {
            gp_Ax3 anAx1 = new(), anAx2 = new(), anAx3 = new(), anAx4 = new();
            anAx3.XReverse();
            anAx4.XReverse();

            bool bDirect1 = anAx1.Direct();
            anAx1.SetXDirection(gp.DZ());
            Assert.Equal(bDirect1, anAx1.Direct());
            gp_Dir aGoodY1 = anAx1.Direction().Crossed(gp.DZ());
            if (anAx1.Direct())
            {
                Assert.True(aGoodY1.IsEqual(anAx1.YDirection(), Precision.Angular()));
            }
            else
            {
                Assert.True(aGoodY1.IsOpposite(anAx1.YDirection(), Precision.Angular()));
            }

            bool bDirect2 = anAx2.Direct();
            anAx2.SetXDirection(-gp.DZ());
            Assert.Equal(bDirect2, anAx2.Direct());

            bool bDirect3 = anAx3.Direct();
            anAx3.SetXDirection(gp.DZ());
            Assert.Equal(bDirect3, anAx3.Direct());

            bool bDirect4 = anAx4.Direct();
            anAx4.SetXDirection(-gp.DZ());
            Assert.Equal(bDirect4, anAx4.Direct());
        }

        // Test Y direction
        {
            gp_Ax3 anAx1 = new(), anAx2 = new(), anAx3 = new(), anAx4 = new();
            anAx3.YReverse();
            anAx4.YReverse();

            bool bDirect1 = anAx1.Direct();
            anAx1.SetYDirection(gp.DZ());
            Assert.Equal(bDirect1, anAx1.Direct());
            gp_Dir aGoodX1 = anAx1.Direction().Crossed(gp.DZ());
            if (anAx1.Direct())
            {
                Assert.True(aGoodX1.IsOpposite(anAx1.XDirection(), Precision.Angular()));
            }
            else
            {
                Assert.True(aGoodX1.IsEqual(anAx1.XDirection(), Precision.Angular()));
            }

            bool bDirect2 = anAx2.Direct();
            anAx2.SetYDirection(-gp.DZ());
            Assert.Equal(bDirect2, anAx2.Direct());

            bool bDirect3 = anAx3.Direct();
            anAx3.SetYDirection(gp.DZ());
            Assert.Equal(bDirect3, anAx3.Direct());

            bool bDirect4 = anAx4.Direct();
            anAx4.SetYDirection(-gp.DZ());
            Assert.Equal(bDirect4, anAx4.Direct());
        }
    }
}
