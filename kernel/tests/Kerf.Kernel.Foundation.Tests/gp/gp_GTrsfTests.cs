// Port of: src/FoundationClasses/TKMath/GTests/gp_GTrsf_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class gp_GTrsfTest
{
    // OCC13963: SetAffinity with ratio=1 is identity transform for any point.
    // The plane is defined by gp_Ax2 at origin with main dir (1,-1,0)/sqrt(2).
    [Fact]
    public void OCC13963_SetAffinity_Ratio1_IsIdentity()
    {
        Frame3D aPln = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, -1.0, 0.0));
        GeneralTransform3D aTrf = new();
        aTrf.SetAffinity(aPln, 1.0);

        Xyz aOrigin = new(1.0, 0.0, 0.0);
        Xyz aResult = aOrigin;
        aTrf.Transforms(ref aResult);

        Assert.Equal(1.0, aResult.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aResult.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aResult.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // OCC13963: SetAffinity with ratio=2 scales the point component along the plane normal.
    // Point (1,0,0) through plane with normal (1,-1,0)/sqrt(2) with ratio 2 gives (1.5,-0.5,0).
    [Fact]
    public void OCC13963_SetAffinity_Ratio2_ScalesAlongNormal()
    {
        Frame3D aPln = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, -1.0, 0.0));
        GeneralTransform3D aTrf = new();
        aTrf.SetAffinity(aPln, 2.0);

        Xyz aOrigin = new(1.0, 0.0, 0.0);
        Xyz aResult = aOrigin;
        aTrf.Transforms(ref aResult);

        Assert.Equal(1.5, aResult.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-0.5, aResult.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aResult.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
