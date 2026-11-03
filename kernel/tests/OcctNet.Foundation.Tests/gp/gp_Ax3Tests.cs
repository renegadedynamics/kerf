// Port of: src/FoundationClasses/TKMath/GTests/gp_Ax3_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace OcctNet.Foundation.Tests;

public class gp_Ax3_Test
{
    [Fact]
    public void OCC29406_SetDirectionPreservesOrientation()
    {
        // Test Main (Z) direction
        {
            // gp_Ax3::SetDirection() test
            CoordinateSystem anAx1 = new(), anAx2 = new(), anAx3 = new(), anAx4 = new();
            anAx3.ZReverse();
            anAx4.ZReverse();

            bool bDirect1 = anAx1.Direct();
            anAx1.SetDirection(Constants.DX());
            // Coordinate system orientation should be preserved
            Assert.Equal(bDirect1, anAx1.Direct());
            Assert.True(Constants.DX().IsEqual(anAx1.Direction(), global::Kerf.Engine.Foundation.Precision.Angular()));

            bool bDirect2 = anAx2.Direct();
            anAx2.SetDirection(-Constants.DX());
            Assert.Equal(bDirect2, anAx2.Direct());
            Assert.True((-Constants.DX()).IsEqual(anAx2.Direction(), global::Kerf.Engine.Foundation.Precision.Angular()));

            bool bDirect3 = anAx3.Direct();
            anAx3.SetDirection(Constants.DX());
            Assert.Equal(bDirect3, anAx3.Direct());
            Assert.True(Constants.DX().IsEqual(anAx3.Direction(), global::Kerf.Engine.Foundation.Precision.Angular()));

            bool bDirect4 = anAx4.Direct();
            anAx4.SetDirection(-Constants.DX());
            Assert.Equal(bDirect4, anAx4.Direct());
            Assert.True((-Constants.DX()).IsEqual(anAx4.Direction(), global::Kerf.Engine.Foundation.Precision.Angular()));

            // gp_Ax3::SetAxis() test
            CoordinateSystem anAx5 = new(), anAx6 = new();
            Axis3D anAx0_1 = new(Constants.Origin(), Constants.DX());
            Axis3D anAx0_2 = new(Constants.Origin(), -Constants.DX());

            bool bDirect5 = anAx5.Direct();
            anAx5.SetAxis(anAx0_1);
            Assert.Equal(bDirect5, anAx5.Direct());
            Assert.True(anAx0_1.Direction().IsEqual(anAx5.Direction(), global::Kerf.Engine.Foundation.Precision.Angular()));

            bool bDirect6 = anAx6.Direct();
            anAx6.SetAxis(anAx0_2);
            Assert.Equal(bDirect6, anAx6.Direct());
            Assert.True(anAx0_2.Direction().IsEqual(anAx6.Direction(), global::Kerf.Engine.Foundation.Precision.Angular()));
        }

        // Test X direction
        {
            CoordinateSystem anAx1 = new(), anAx2 = new(), anAx3 = new(), anAx4 = new();
            anAx3.XReverse();
            anAx4.XReverse();

            bool bDirect1 = anAx1.Direct();
            anAx1.SetXDirection(Constants.DZ());
            Assert.Equal(bDirect1, anAx1.Direct());
            Direction3D aGoodY1 = anAx1.Direction().Crossed(Constants.DZ());
            if (anAx1.Direct())
            {
                Assert.True(aGoodY1.IsEqual(anAx1.YDirection(), global::Kerf.Engine.Foundation.Precision.Angular()));
            }
            else
            {
                Assert.True(aGoodY1.IsOpposite(anAx1.YDirection(), global::Kerf.Engine.Foundation.Precision.Angular()));
            }

            bool bDirect2 = anAx2.Direct();
            anAx2.SetXDirection(-Constants.DZ());
            Assert.Equal(bDirect2, anAx2.Direct());

            bool bDirect3 = anAx3.Direct();
            anAx3.SetXDirection(Constants.DZ());
            Assert.Equal(bDirect3, anAx3.Direct());

            bool bDirect4 = anAx4.Direct();
            anAx4.SetXDirection(-Constants.DZ());
            Assert.Equal(bDirect4, anAx4.Direct());
        }

        // Test Y direction
        {
            CoordinateSystem anAx1 = new(), anAx2 = new(), anAx3 = new(), anAx4 = new();
            anAx3.YReverse();
            anAx4.YReverse();

            bool bDirect1 = anAx1.Direct();
            anAx1.SetYDirection(Constants.DZ());
            Assert.Equal(bDirect1, anAx1.Direct());
            Direction3D aGoodX1 = anAx1.Direction().Crossed(Constants.DZ());
            if (anAx1.Direct())
            {
                Assert.True(aGoodX1.IsOpposite(anAx1.XDirection(), global::Kerf.Engine.Foundation.Precision.Angular()));
            }
            else
            {
                Assert.True(aGoodX1.IsEqual(anAx1.XDirection(), global::Kerf.Engine.Foundation.Precision.Angular()));
            }

            bool bDirect2 = anAx2.Direct();
            anAx2.SetYDirection(-Constants.DZ());
            Assert.Equal(bDirect2, anAx2.Direct());

            bool bDirect3 = anAx3.Direct();
            anAx3.SetYDirection(Constants.DZ());
            Assert.Equal(bDirect3, anAx3.Direct());

            bool bDirect4 = anAx4.Direct();
            anAx4.SetYDirection(-Constants.DZ());
            Assert.Equal(bDirect4, anAx4.Direct());
        }
    }
}
