// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests.BRepAlgoAPI;

/// <summary>
/// OCCT's TKBool GTests of the Boolean API (src/ModelingAlgorithms/TKBool/GTests/BRepAlgoAPI_Cut_Test.cxx,
/// BRepAlgoAPI_Fuse_Test.cxx and BRepAlgoAPI_Section_Test.cxx). OCCT's suites are nested classes, so the tests keep
/// their names (the outer class keeps them apart from TKBO's suites of the same names).
/// </summary>
public static class TKBool_BRepAlgoAPITests
{
    private static double runHollowBoxMeshTest(double theMeshDelta)
    {
        double aDelt = 5.0 * global::Kerf.Engine.Foundation.Precision.Confusion();

        // Create outer box solid
        Solid aFullSolid = new BRepPrimAPI_MakeBox(new Point3D(0, 0, 0), 30.0, 30.0, 30.0).Solid();

        // Create inner box solid
        Solid anInnerSolid = new BRepPrimAPI_MakeBox(new Point3D(10, 10, 10), 10.0, 10.0, 10.0).Solid();

        // Cut inner from outer
        BRepAlgoAPI_Cut aCut = new(aFullSolid, anInnerSolid);
        if (!aCut.IsDone())
        {
            return -1.0;
        }

        // Extract the single solid from the cut result
        Solid aCutSolid = new();
        int aNbSolids = 0;
        Explorer anExp = new();
        for (anExp.Init(aCut.Shape(), ShapeType.Solid); anExp.More(); anExp.Next())
        {
            Solid aSol = ShapeCasts.Solid(anExp.Current());
            if (!aSol.IsNull)
            {
                aCutSolid = aSol;
                aNbSolids++;
            }
        }
        if (aNbSolids != 1)
        {
            return -1.0;
        }

        // Calculate original volume
        Properties aVProps = new();
        BRepGProp.VolumeProperties(aCutSolid, aVProps);
        double anOriginalVolume = aVProps.Mass();
        if (anOriginalVolume <= 0.0)
        {
            return -1.0;
        }

        // Build bounding box and extend by small delta
        Box aBndBox = new();
        BRepBndLib.Add(aCutSolid, ref aBndBox);
        aBndBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);
        aXmin -= aDelt;
        aYmin -= aDelt;
        aZmin -= aDelt;
        aXmax += aDelt;
        aYmax += aDelt;
        aZmax += aDelt;

        // Grid the bounding box
        int aNx = (int)((aXmax - aXmin) / theMeshDelta);
        if (aNx <= 0)
        {
            aNx = 1;
        }
        int aNy = (int)((aYmax - aYmin) / theMeshDelta);
        if (aNy <= 0)
        {
            aNy = 1;
        }
        int aNz = (int)((aZmax - aZmin) / theMeshDelta);
        if (aNz <= 0)
        {
            aNz = 1;
        }

        double aStepX = (aXmax - aXmin) / aNx;
        double aStepY = (aYmax - aYmin) / aNy;
        double aStepZ = (aZmax - aZmin) / aNz;
        int aNbSubvols = aNx * aNy * aNz;

        NCollection_Array1<Shape> aSubvols = new(0, aNbSubvols - 1);
        NCollection_Array1<double> aSubvolVols = new(0, aNbSubvols - 1);

        // Build grid cells
        int l = 0;
        for (int i = 0; i < aNx; i++)
        {
            for (int j = 0; j < aNy; j++)
            {
                for (int k = 0; k < aNz; k++)
                {
                    Point3D aPnt = new(aXmin + i * aStepX, aYmin + j * aStepY, aZmin + k * aStepZ);
                    aSubvols.SetValue(l, new BRepPrimAPI_MakeBox(aPnt, aStepX, aStepY, aStepZ).Solid());
                    BRepGProp.VolumeProperties(aSubvols[l], aVProps);
                    aSubvolVols.SetValue(l, aVProps.Mass());
                    l++;
                }
            }
        }

        // Compute common of solid with each grid cell, accumulate volumes
        double anAccumulated = 0.0;
        for (l = 0; l < aNbSubvols; l++)
        {
            Shape aCopySolid = new BRepBuilderAPI_Copy(aCutSolid).Shape();
            BRepAlgoAPI_Common aCommon = new(aCopySolid, aSubvols[l]);
            if (!aCommon.IsDone())
            {
                continue;
            }

            int aNbCommon = 0;
            Shape aFoundSolid = new();
            for (anExp.Init(aCommon.Shape(), ShapeType.Solid); anExp.More(); anExp.Next())
            {
                Solid aSol = ShapeCasts.Solid(anExp.Current());
                if (!aSol.IsNull)
                {
                    aFoundSolid = aSol;
                    aNbCommon++;
                }
            }
            if (aNbCommon == 1)
            {
                BRepGProp.VolumeProperties(aFoundSolid, aVProps);
                double aVol = aVProps.Mass();
                if (aVol > 0.0 && aVol <= aSubvolVols[l])
                {
                    anAccumulated += aVol;
                }
            }
        }

        return anAccumulated / anOriginalVolume;
    }

    /// <summary>TKBool's BRepAlgoAPI_Cut_Test.cxx.</summary>
    public class BRepAlgoAPI_CutTest
    {
        [Fact]
        public void HollowBox_VolumeAccuracy_Delta10()
        {
            double aRatio = runHollowBoxMeshTest(10.0);
            Assert.True(aRatio >= 0.0, "Cut or Common operation failed");
            Assert.Equal(1.0, aRatio, 0.001); // Accumulated meshed volume differs from original by > 0.1%
        }

        [Fact]
        public void HollowBox_VolumeAccuracy_Delta15()
        {
            double aRatio = runHollowBoxMeshTest(15.0);
            Assert.True(aRatio >= 0.0, "Cut or Common operation failed");
            Assert.Equal(1.0, aRatio, 0.001);
        }

        [Fact]
        public void HollowBox_VolumeAccuracy_Delta30()
        {
            double aRatio = runHollowBoxMeshTest(30.0);
            Assert.True(aRatio >= 0.0, "Cut or Common operation failed");
            Assert.Equal(1.0, aRatio, 0.001);
        }

        [Fact]
        public void HollowBox_SurfaceAreaAndValidity()
        {
            Solid aFullSolid = new BRepPrimAPI_MakeBox(new Point3D(0, 0, 0), 30.0, 30.0, 30.0).Solid();
            Solid anInnerSolid = new BRepPrimAPI_MakeBox(new Point3D(10, 10, 10), 10.0, 10.0, 10.0).Solid();

            BRepAlgoAPI_Cut aCut = new(aFullSolid, anInnerSolid);
            Assert.True(aCut.IsDone());

            // Extract solid
            Solid aCutSolid = new();
            Explorer anExp = new();
            for (anExp.Init(aCut.Shape(), ShapeType.Solid); anExp.More(); anExp.Next())
            {
                aCutSolid = ShapeCasts.Solid(anExp.Current());
            }
            Assert.False(aCutSolid.IsNull);

            // Surface area: outer box 6*900=5400, inner box adds 6*100=600 => total=6000
            Properties aSProps = new();
            BRepGProp.SurfaceProperties(aCutSolid, aSProps);
            Assert.Equal(6000.0, aSProps.Mass(), 6.0); // 0.1% tolerance

            BRepCheck_Analyzer aChecker = new(aCutSolid);
            Assert.True(aChecker.IsValid());
        }
    }

    private static void CheckResult(in Shape aResult, double theArea, double theTol)
    {
        Assert.False(aResult.IsNull);

        Properties aSProps = new();
        BRepGProp.SurfaceProperties(aResult, aSProps);
        Assert.Equal(theArea, aSProps.Mass(), theTol); // 0.1% tolerance

        BRepCheck_Analyzer aChecker = new(aResult);
        Assert.True(aChecker.IsValid());
    }

    /// <summary>TKBool's BRepAlgoAPI_Fuse_Test.cxx.</summary>
    public class BRepAlgoAPI_FuseTest
    {
        [Fact]
        public void CylinderAndCone_FuseThenCut()
        {
            Frame3D anAxis1 = new(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z));
            Shape aCylIn = new BRepPrimAPI_MakeCylinder(anAxis1, 40, 110).Shape();
            Shape aCylOut = new BRepPrimAPI_MakeCylinder(anAxis1, 50, 100).Shape();

            Frame3D anAxis2 = new(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.NZ));
            Shape aConIn = new BRepPrimAPI_MakeCone(anAxis2, 40, 60, 110).Shape();
            Shape aConOut = new BRepPrimAPI_MakeCone(anAxis2, 50, 70, 100).Shape();

            BRepAlgoAPI_Fuse aFuseIn = new(aCylIn, aConIn);
            Assert.True(aFuseIn.IsDone());

            BRepAlgoAPI_Fuse aFuseOut = new(aCylOut, aConOut);
            Assert.True(aFuseOut.IsDone());

            BRepAlgoAPI_Cut aCut = new(aFuseOut.Shape(), aFuseIn.Shape());
            Assert.True(aCut.IsDone());

            CheckResult(aCut.Shape(), 133931.0, 133.931);
        }

        [Fact]
        public void BoxAndSphere()
        {
            Shape aBox = new BRepPrimAPI_MakeBox(new Point3D(0, 0, 0), new Point3D(100, 100, 100)).Shape();
            Shape aSphere = new BRepPrimAPI_MakeSphere(new Point3D(100, 50, 50), 25.0).Shape();

            BRepAlgoAPI_Fuse aFuse = new(aBox, aSphere);
            Assert.True(aFuse.IsDone());

            CheckResult(aFuse.Shape(), 61963.5, 61.9635);
        }

        [Fact]
        public void TwoCylinders()
        {
            Frame3D anAxis1 = new(new Point3D(40, 50, 0), new Direction3D(100, 0, 0));
            Shape aCyl1 = new BRepPrimAPI_MakeCylinder(anAxis1, 20, 100).Shape();

            const double aSize = 0.001;
            Frame3D anAxis2 = new(new Point3D(100, 50, aSize), new Direction3D(0, aSize, 80));
            Shape aCyl2 = new BRepPrimAPI_MakeCylinder(anAxis2, 20, 80).Shape();

            BRepAlgoAPI_Fuse aFuse = new(aCyl2, aCyl1);
            Assert.True(aFuse.IsDone());

            CheckResult(aFuse.Shape(), 23189.5, 23.1895);
        }

        [Fact]
        public void CylinderAndSphere()
        {
            Point3D aCenter = new(100, 0, 0);
            Frame3D anAxis = new(aCenter, new Direction3D(Direction3D.D.NX));
            Shape aCyl = new BRepPrimAPI_MakeCylinder(anAxis, 20, 100).Shape();
            Shape aSphere = new BRepPrimAPI_MakeSphere(aCenter, 20.0).Shape();

            BRepAlgoAPI_Fuse aFuse = new(aCyl, aSphere);
            Assert.True(aFuse.IsDone());

            CheckResult(aFuse.Shape(), 16336.3, 16.3363);
        }

        [Fact]
        public void RevolvedFaceAndSphere()
        {
            BRepBuilderAPI_MakePolygon aWire = new();
            const double aX1 = 181.82808, aX2 = 202.39390;
            const double aY1 = 31.011970, aY2 = 123.06856;
            aWire.Add(new Point3D(aX1, aY1, 0));
            aWire.Add(new Point3D(aX2, aY1, 0));
            aWire.Add(new Point3D(aX2, aY2, 0));
            aWire.Add(new Point3D(aX1, aY2, 0));
            aWire.Add(new Point3D(aX1, aY1, 0));

            Face aFace = new BRepBuilderAPI_MakeFace(aWire.Wire(), false).Face();
            Axis3D anAxis = new(new Point3D(0, 0, 0), new Direction3D(0, 30, 0));
            Shape aRevol = new BRepPrimAPI_MakeRevol(aFace, anAxis, 2.0 * M_PI).Shape();
            Shape aSphere = new BRepPrimAPI_MakeSphere(new Point3D(166.373, 77.0402, 96.0555), 23.218586).Shape();

            BRepAlgoAPI_Fuse aFuse = new(aRevol, aSphere);
            Assert.True(aFuse.IsDone());

            CheckResult(aFuse.Shape(), 272935.0, 272.935);
        }

        [Fact]
        public void RevolvedSolidAndTwoTori()
        {
            BRepBuilderAPI_MakePolygon aWire = new();
            aWire.Add(new Point3D(10, 0, 0));
            aWire.Add(new Point3D(20, 0, 0));
            aWire.Add(new Point3D(20, 0, 50));
            aWire.Add(new Point3D(10, 0, 50));
            aWire.Add(new Point3D(10, 0, 0));

            Face aFace = new BRepBuilderAPI_MakeFace(aWire.Wire(), false).Face();
            Axis3D anAxis = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 30));
            Shape aRevol = new BRepPrimAPI_MakeRevol(aFace, anAxis, 2.0 * M_PI).Shape();

            const double aMajRad = 15.0;
            const double aMinRad = 5.0;

            Shape aTor1 = new BRepPrimAPI_MakeTorus(new Frame3D(new Point3D(0, 0, 50), new Direction3D(0, 0, 30)), aMajRad, aMinRad).Shape();
            Shape aTor2 = new BRepPrimAPI_MakeTorus(new Frame3D(new Point3D(0, 0, 10), new Direction3D(0, 0, 30)), aMajRad, aMinRad).Shape();

            BRepAlgoAPI_Fuse aFuse1 = new(aTor1, aRevol);
            Assert.True(aFuse1.IsDone());

            BRepAlgoAPI_Fuse aFuse2 = new(aTor2, aFuse1.Shape());
            Assert.True(aFuse2.IsDone());

            CheckResult(aFuse2.Shape(), 11847.7, 11.8477);
        }
    }

    /// <summary>TKBool's BRepAlgoAPI_Section_Test.cxx.</summary>
    public class BRepAlgoAPI_SectionTest
    {
        [Fact]
        public void OCCN2_CylinderSphereSectionIsDone()
        {
            BRepPrimAPI_MakeCylinder aCylMaker = new(50.0, 200.0);
            Shape aCylinder = aCylMaker.Shape();

            BRepPrimAPI_MakeSphere aSphereMaker = new(new Point3D(60.0, 0.0, 100.0), 50.0);
            Shape aSphere = aSphereMaker.Shape();

            BRepAlgoAPI_Section aSection = new(aCylinder, aSphere);
            Assert.True(aSection.IsDone());
            Assert.False(aSection.Shape().IsNull);
        }
    }
}
