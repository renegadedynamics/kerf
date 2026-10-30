// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/GeomFill_GuideTrihedronAC_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GeomFill_GuideTrihedronAC GTest (suite GeomFill_GuideTrihedronAC).</summary>
public class GeomFill_GuideTrihedronAC_Test
{
    // Helper: create a cubic BSpline curve suitable for C2 evaluation.
    private static GeomAdaptor_Curve makeBSplineCurve(gp_Pnt theP1, gp_Pnt theP2, gp_Pnt theP3, gp_Pnt theP4)
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 4);
        aPoles[1] = theP1;
        aPoles[2] = theP2;
        aPoles[3] = theP3;
        aPoles[4] = theP4;

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        Geom_BSplineCurve aCurve = new(aPoles, aKnots, aMults, 3);
        return new GeomAdaptor_Curve(aCurve);
    }

    [Fact]
    public void D0_ReturnsTrue()
    {
        Adaptor3d_Curve aGuide = makeBSplineCurve(new gp_Pnt(0, 0, 1), new gp_Pnt(1, 0, 1), new gp_Pnt(1, 1, 1), new gp_Pnt(0, 1, 1));
        Adaptor3d_Curve aPath = makeBSplineCurve(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(0, 1, 0));

        GeomFill_GuideTrihedronAC aTrihedron = new(aGuide);
        aTrihedron.SetCurve(aPath);

        gp_Vec aTangent = new(), aNormal = new(), aBiNormal = new();
        bool isOk = aTrihedron.D0(0.5, ref aTangent, ref aNormal, ref aBiNormal);
        Assert.True(isOk);
        Assert.True(aTangent.Magnitude() > Precision.Confusion());
        Assert.True(aNormal.Magnitude() > Precision.Confusion());
        Assert.True(aBiNormal.Magnitude() > Precision.Confusion());
    }

    [Fact]
    public void D1_ReturnsTrue()
    {
        Adaptor3d_Curve aGuide = makeBSplineCurve(new gp_Pnt(0, 0, 1), new gp_Pnt(1, 0, 1), new gp_Pnt(1, 1, 1), new gp_Pnt(0, 1, 1));
        Adaptor3d_Curve aPath = makeBSplineCurve(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(0, 1, 0));

        GeomFill_GuideTrihedronAC aTrihedron = new(aGuide);
        aTrihedron.SetCurve(aPath);

        gp_Vec aTangent = new(), aDTangent = new(), aNormal = new(), aDNormal = new(), aBiNormal = new(), aDBiNormal = new();
        bool isOk = aTrihedron.D1(0.5, ref aTangent, ref aDTangent, ref aNormal, ref aDNormal, ref aBiNormal, ref aDBiNormal);
        Assert.True(isOk);
        Assert.True(aTangent.Magnitude() > Precision.Confusion());
        Assert.True(aNormal.Magnitude() > Precision.Confusion());
        Assert.True(aBiNormal.Magnitude() > Precision.Confusion());
    }

    // Regression test for bug #7: D2 was returning false despite completing computation.
    [Fact]
    public void D2_ReturnsTrue()
    {
        Adaptor3d_Curve aGuide = makeBSplineCurve(new gp_Pnt(0, 0, 1), new gp_Pnt(1, 0, 1), new gp_Pnt(1, 1, 1), new gp_Pnt(0, 1, 1));
        Adaptor3d_Curve aPath = makeBSplineCurve(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(0, 1, 0));

        GeomFill_GuideTrihedronAC aTrihedron = new(aGuide);
        aTrihedron.SetCurve(aPath);

        gp_Vec aTangent = new(), aDTangent = new(), aD2Tangent = new();
        gp_Vec aNormal = new(), aDNormal = new(), aD2Normal = new();
        gp_Vec aBiNormal = new(), aDBiNormal = new(), aD2BiNormal = new();

        bool isOk = aTrihedron.D2(0.5, ref aTangent, ref aDTangent, ref aD2Tangent, ref aNormal, ref aDNormal,
                                  ref aD2Normal, ref aBiNormal, ref aDBiNormal, ref aD2BiNormal);
        Assert.True(isOk);

        // Verify frame vectors are non-degenerate.
        Assert.True(aTangent.Magnitude() > Precision.Confusion());
        Assert.True(aNormal.Magnitude() > Precision.Confusion());
        Assert.True(aBiNormal.Magnitude() > Precision.Confusion());
    }

    // Verify D2 produces consistent results at multiple parameters.
    [Fact]
    public void D2_ConsistentAtMultipleParams()
    {
        Adaptor3d_Curve aGuide = makeBSplineCurve(new gp_Pnt(0, 0, 2), new gp_Pnt(2, 0, 2), new gp_Pnt(2, 2, 2), new gp_Pnt(0, 2, 2));
        Adaptor3d_Curve aPath = makeBSplineCurve(new gp_Pnt(0, 0, 0), new gp_Pnt(2, 0, 0), new gp_Pnt(2, 2, 0), new gp_Pnt(0, 2, 0));

        GeomFill_GuideTrihedronAC aTrihedron = new(aGuide);
        aTrihedron.SetCurve(aPath);

        double[] aParams = [0.2, 0.4, 0.6, 0.8];
        foreach (double aParam in aParams)
        {
            gp_Vec aTangent = new(), aDTangent = new(), aD2Tangent = new();
            gp_Vec aNormal = new(), aDNormal = new(), aD2Normal = new();
            gp_Vec aBiNormal = new(), aDBiNormal = new(), aD2BiNormal = new();

            bool isOk = aTrihedron.D2(aParam, ref aTangent, ref aDTangent, ref aD2Tangent, ref aNormal, ref aDNormal,
                                      ref aD2Normal, ref aBiNormal, ref aDBiNormal, ref aD2BiNormal);
            Assert.True(isOk, $"D2 failed at param={aParam}");
        }
    }
}
