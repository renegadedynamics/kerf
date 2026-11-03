// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepBndLib against native OCCT on all the random scenes of <see cref="ClassificationScenes"/> (with meshed boxes
/// and mesh-only tetrahedra): Add with and without triangulations, AddClose, AddOptimal with its two flags, and AddOBB
/// with its three flags (the point path, DiTO, on planar, linear and meshed shapes; the inertia path, through
/// BRepGProp, on the others), each into a void box and into a box already holding something. The boxes are compared
/// value by value: void and open flags, bounds and gap; the oriented boxes' centre, axes, half sizes and flags.
/// </summary>
public class BRepBndLibOracleTests(ITestOutputHelper output)
{
    private const int Scenes = 120;

    [Fact]
    public void Boxes_match_native_OCCT()
    {
        var anOracle = new Oracle(output, 20261102);
        int aCalls = 0;
        for (int aScene = 0; aScene < Scenes; aScene++)
        {
            Twin<N.TopoDS_Shape, TopoDS_Shape> aShape = ClassificationScenes.Scene(anOracle, aScene);
            string aWhat = $"scene {aScene} ({ClassificationScenes.Name(aScene)})";
            bool aStartFull = anOracle.Bool();
            Point3D aMin = new(anOracle.Real(-30, 0), anOracle.Real(-30, 0), anOracle.Real(-30, 0));
            Point3D aMax = new(aMin.X() + anOracle.Real(0, 5), aMin.Y() + anOracle.Real(0, 5), aMin.Z() + anOracle.Real(0, 5));

            (N.Bnd_Box, Box) Start()
            {
                if (!aStartFull)
                {
                    return (new N.Bnd_Box(), new Box());
                }

                using var aNMin = Oracle.ToNative(aMin);
                using var aNMax = Oracle.ToNative(aMax);
                return (new N.Bnd_Box(aNMin, aNMax), new Box(aMin, aMax));
            }

            foreach (bool aUseTri in new[] { true, false })
            {
                (N.Bnd_Box aNB, Box aPB) = Start();
                using (aNB)
                {
                    N.BRepBndLib.Add(aShape.N, aNB, aUseTri);
                    BRepBndLib.Add(aShape.P, ref aPB, aUseTri);
                    SameBox(anOracle, aNB, aPB, $"{aWhat} Add({aUseTri})");
                    aCalls++;
                }
            }

            {
                (N.Bnd_Box aNB, Box aPB) = Start();
                using (aNB)
                {
                    N.BRepBndLib.AddClose(aShape.N, aNB);
                    BRepBndLib.AddClose(aShape.P, ref aPB);
                    SameBox(anOracle, aNB, aPB, $"{aWhat} AddClose");
                    aCalls++;
                }
            }

            foreach (bool aUseTri in new[] { true, false })
            {
                foreach (bool aUseTol in new[] { true, false })
                {
                    (N.Bnd_Box aNB, Box aPB) = Start();
                    using (aNB)
                    {
                        N.BRepBndLib.AddOptimal(aShape.N, aNB, aUseTri, aUseTol);
                        BRepBndLib.AddOptimal(aShape.P, ref aPB, aUseTri, aUseTol);
                        SameBox(anOracle, aNB, aPB, $"{aWhat} AddOptimal({aUseTri}, {aUseTol})");
                        aCalls++;
                    }
                }
            }

            foreach (bool aUseTri in new[] { true, false })
            {
                foreach (bool anOptimal in new[] { true, false })
                {
                    foreach (bool aUseTol in new[] { true, false })
                    {
                        N.Bnd_OBB aNO;
                        OrientedBox aPO;
                        if (aStartFull)
                        {
                            using var aNC = Oracle.ToNative(aMin);
                            using var aNX = new N.gp_Dir(1, 0, 0);
                            using var aNY = new N.gp_Dir(0, 1, 0);
                            using var aNZ = new N.gp_Dir(0, 0, 1);
                            aNO = new N.Bnd_OBB(aNC, aNX, aNY, aNZ, 1.0, 2.0, 0.5);
                            aPO = new OrientedBox(aMin, new Direction3D(1, 0, 0), new Direction3D(0, 1, 0), new Direction3D(0, 0, 1), 1.0, 2.0, 0.5);
                        }
                        else
                        {
                            aNO = new N.Bnd_OBB();
                            aPO = new OrientedBox();
                        }

                        using (aNO)
                        {
                            N.BRepBndLib.AddOBB(aShape.N, aNO, aUseTri, anOptimal, aUseTol);
                            BRepBndLib.AddOBB(aShape.P, ref aPO, aUseTri, anOptimal, aUseTol);
                            SameOBB(anOracle, aNO, aPO, $"{aWhat} AddOBB({aUseTri}, {anOptimal}, {aUseTol})");
                            aCalls++;
                        }
                    }
                }
            }

            // the functional forms
            {
                using var aNB = new N.Bnd_Box();
                N.BRepBndLib.Add(aShape.N, aNB);
                SameBox(anOracle, aNB, BRepBndLib.Box(aShape.P), $"{aWhat} Box");
                using var aNO = new N.Bnd_OBB();
                N.BRepBndLib.AddOBB(aShape.N, aNO);
                SameOBB(anOracle, aNO, BRepBndLib.OBB(aShape.P), $"{aWhat} OBB");
            }
        }

        output.WriteLine($"{Scenes} scenes: {aCalls} BRepBndLib calls");
        anOracle.Report("BRepBndLib (Add, AddClose, AddOptimal, AddOBB)");
    }

    private static void SameBox(Oracle theO, N.Bnd_Box theN, in Box theP, string theWhat)
    {
        theO.Same(theN.IsVoid(), theP.IsVoid, theWhat + " IsVoid");
        if (theN.IsVoid())
        {
            return;
        }

        theO.Same(theN.IsOpenXmin(), theP.IsOpenXmin, theWhat + " IsOpenXmin");
        theO.Same(theN.IsOpenXmax(), theP.IsOpenXmax, theWhat + " IsOpenXmax");
        theO.Same(theN.IsOpenYmin(), theP.IsOpenYmin, theWhat + " IsOpenYmin");
        theO.Same(theN.IsOpenYmax(), theP.IsOpenYmax, theWhat + " IsOpenYmax");
        theO.Same(theN.IsOpenZmin(), theP.IsOpenZmin, theWhat + " IsOpenZmin");
        theO.Same(theN.IsOpenZmax(), theP.IsOpenZmax, theWhat + " IsOpenZmax");
        theO.Same(theN.GetXMin(), theP.GetXMin(), theWhat + " Xmin");
        theO.Same(theN.GetXMax(), theP.GetXMax(), theWhat + " Xmax");
        theO.Same(theN.GetYMin(), theP.GetYMin(), theWhat + " Ymin");
        theO.Same(theN.GetYMax(), theP.GetYMax(), theWhat + " Ymax");
        theO.Same(theN.GetZMin(), theP.GetZMin(), theWhat + " Zmin");
        theO.Same(theN.GetZMax(), theP.GetZMax(), theWhat + " Zmax");
        theO.Same(theN.GetGap(), theP.GetGap(), theWhat + " Gap");
    }

    private static void SameOBB(Oracle theO, N.Bnd_OBB theN, in OrientedBox theP, string theWhat)
    {
        theO.Same(theN.IsVoid(), theP.IsVoid, theWhat + " IsVoid");
        if (theN.IsVoid())
        {
            return;
        }

        theO.Same(theN.IsAABox(), theP.IsAABox, theWhat + " IsAABox");
        using (N.gp_XYZ aC = theN.Center())
        {
            theO.Same(new Xyz(aC.X(), aC.Y(), aC.Z()), theP.Center(), theWhat + " Center");
        }

        using (N.gp_XYZ aX = theN.XDirection())
        {
            theO.Same(new Xyz(aX.X(), aX.Y(), aX.Z()), theP.XDirection(), theWhat + " XDirection");
        }

        using (N.gp_XYZ aY = theN.YDirection())
        {
            theO.Same(new Xyz(aY.X(), aY.Y(), aY.Z()), theP.YDirection(), theWhat + " YDirection");
        }

        using (N.gp_XYZ aZ = theN.ZDirection())
        {
            theO.Same(new Xyz(aZ.X(), aZ.Y(), aZ.Z()), theP.ZDirection(), theWhat + " ZDirection");
        }

        theO.Same(theN.XHSize(), theP.XHSize(), theWhat + " XHSize");
        theO.Same(theN.YHSize(), theP.YHSize(), theWhat + " YHSize");
        theO.Same(theN.ZHSize(), theP.ZHSize(), theWhat + " ZHSize");
    }
}
