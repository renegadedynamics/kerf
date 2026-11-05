// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/GeomFill_CorrectedFrenet_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>OCCT's GeomFill_CorrectedFrenet GTest (suite GeomFill_CorrectedFrenet).</summary>
public class GeomFill_CorrectedFrenet_Test
{
    [Fact]
    public void EndlessLoopPrevention()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 0.0, 0.0);
        aPoles[3] = new Point3D(1.0, 1.0, 0.0);
        aPoles[4] = new Point3D(0.0, 1.0, 0.0);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        BSplineCurve3D aCurve = new(aPoles, aKnots, aMults, 3);
        CurveAdaptor anAdaptor = new(aCurve);

        CorrectedFrenet aCorrectedFrenet = new(false);
        aCorrectedFrenet.SetCurve(anAdaptor);

        double aParam1 = 0.0;
        double aParam2 = 1.0;

        Vector3D aTangent1 = new(), aNormal1 = new(), aBinormal1 = new();
        Vector3D aTangent2 = new(), aNormal2 = new(), aBinormal2 = new();

        aCorrectedFrenet.D0(aParam1, ref aTangent1, ref aNormal1, ref aBinormal1);
        aCorrectedFrenet.D0(aParam2, ref aTangent2, ref aNormal2, ref aBinormal2);

        Assert.True(aTangent1.Magnitude > 1e-10);
        Assert.True(aNormal1.Magnitude > 1e-10);
        Assert.True(aBinormal1.Magnitude > 1e-10);

        Assert.True(aTangent2.Magnitude > 1e-10);
        Assert.True(aNormal2.Magnitude > 1e-10);
        Assert.True(aBinormal2.Magnitude > 1e-10);
    }

    [Fact]
    public void SmallStepHandling()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 2);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1e-10, 0.0, 0.0);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 2;
        aMults[2] = 2;

        BSplineCurve3D aCurve = new(aPoles, aKnots, aMults, 1);
        CurveAdaptor anAdaptor = new(aCurve);

        CorrectedFrenet aCorrectedFrenet = new(false);
        aCorrectedFrenet.SetCurve(anAdaptor);

        Vector3D aTangent = new(), aNormal = new(), aBinormal = new();

        aCorrectedFrenet.D0(0.5, ref aTangent, ref aNormal, ref aBinormal);
    }

    [Fact]
    public void ParameterProgressionGuarantee()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(0.5, 0.5, 0.0);
        aPoles[3] = new Point3D(1.0, 0.0, 0.0);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 3;
        aMults[2] = 3;

        BSplineCurve3D aCurve = new(aPoles, aKnots, aMults, 2);
        CurveAdaptor anAdaptor = new(aCurve);

        CorrectedFrenet aCorrectedFrenet = new(false);
        aCorrectedFrenet.SetCurve(anAdaptor);

        for (double aParam = 0.1; aParam <= 0.9; aParam += 0.1)
        {
            Vector3D aTangent = new(), aNormal = new(), aBinormal = new();

            aCorrectedFrenet.D0(aParam, ref aTangent, ref aNormal, ref aBinormal);

            Assert.True(aTangent.Magnitude > 1e-12);
            Assert.True(aNormal.Magnitude > 1e-12);
            Assert.True(aBinormal.Magnitude > 1e-12);
        }
    }

    [Fact]
    public void ActualReproducerCase()
    {
        NCollection_Array1<Point3D> aPoints = new(1, 4);
        aPoints[1] = new Point3D(-1, -1, 0);
        aPoints[2] = new Point3D(0, -2, 0);
        aPoints[3] = new Point3D(0, -2, -1);
        aPoints[4] = new Point3D(0, -1, -1);

        ExtendWireData anExtend = new();
        for (int i = 2; i <= aPoints.Length(); i++)
        {
            TrimmedCurve3D aCurve = new MakeSegment(aPoints[i - 1], aPoints[i]).Value();
            Edge anEdge = new MakeEdge(aCurve).Edge();
            anExtend.Add(anEdge);
        }

        WireCurveAdaptor anAdaptor = new(anExtend.WireAPIMake());

        CorrectedFrenet aCorrectedFrenet = new(false);

        // This SetCurve call should not hang (was causing infinite loops)
        aCorrectedFrenet.SetCurve(anAdaptor.ShallowCopy());

        // Verify we can evaluate the trihedron at various parameters
        Vector3D aTangent = new(), aNormal = new(), aBinormal = new();

        aCorrectedFrenet.D0(0.0, ref aTangent, ref aNormal, ref aBinormal);

        aCorrectedFrenet.D0(0.5, ref aTangent, ref aNormal, ref aBinormal);

        aCorrectedFrenet.D0(1.0, ref aTangent, ref aNormal, ref aBinormal);
    }
}
