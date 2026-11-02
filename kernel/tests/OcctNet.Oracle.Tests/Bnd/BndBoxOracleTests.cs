// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>Bnd_Box against native OCCT, on random inputs.</summary>
public class BndBoxOracleTests(ITestOutputHelper output)
{
    private const int Cases = 500;

    [Fact]
    public void Box_points_gap_and_queries()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            using var aNative = new N.Bnd_Box();
            var aBox = new Bnd_Box();
            int aNbPoints = anOracle.Int(1, 12);
            for (int k = 0; k < aNbPoints; k++)
            {
                gp_Pnt aP = anOracle.Pnt();
                using var aNP = Oracle.ToNative(aP);
                aNative.Add(aNP);
                aBox.Add(aP);
            }

            if (anOracle.Int(0, 1) == 0)
            {
                double aX = anOracle.Real(), aY = anOracle.Real(), aZ = anOracle.Real();
                aNative.Update(aX, aY, aZ);
                aBox.Update(aX, aY, aZ);
            }

            double aTol = anOracle.Real(-2, 2);
            aNative.Enlarge(aTol);
            aBox.Enlarge(aTol);
            if (anOracle.Int(0, 3) == 0)
            {
                double aGap = anOracle.Real(-1, 1);
                aNative.SetGap(aGap);
                aBox.SetGap(aGap);
            }

            SameBox(anOracle, aNative, aBox, "points");
            using (var aNMin = aNative.CornerMin())
            using (var aNMax = aNative.CornerMax())
            {
                anOracle.Same(Oracle.FromNative(aNMin).XYZ(), aBox.CornerMin().XYZ(), "CornerMin");
                anOracle.Same(Oracle.FromNative(aNMax).XYZ(), aBox.CornerMax().XYZ(), "CornerMax");
            }

            double aThin = anOracle.Real(0, 150);
            anOracle.Same(aNative.IsXThin(aThin), aBox.IsXThin(aThin), "IsXThin");
            anOracle.Same(aNative.IsThin(aThin), aBox.IsThin(aThin), "IsThin");

            for (int k = 0; k < 4; k++)
            {
                gp_Pnt aP = anOracle.Pnt();
                using var aNP = Oracle.ToNative(aP);
                anOracle.Same(aNative.IsOut(aNP), aBox.IsOut(aP), "IsOut(point)");
                anOracle.Same(aNative.Contains(aNP), aBox.Contains(aP), "Contains");
            }
        }

        anOracle.Report("Bnd_Box points, gap and queries");
    }

    [Fact]
    public void Box_add_boxes_directions_and_distance()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.Bnd_Box aN1, Bnd_Box aB1) = RandomBox(anOracle, 0.3);
            (N.Bnd_Box aN2, Bnd_Box aB2) = RandomBox(anOracle, 0.3);
            using (aN1)
            using (aN2)
            {
                anOracle.Same(aN1.IsOut(aN2), aB1.IsOut(aB2), "IsOut(box)");
                anOracle.Same(aN1.Intersects(aN2), aB1.Intersects(aB2), "Intersects");
                anOracle.Same(aN1.HasFinitePart(), aB1.HasFinitePart(), "HasFinitePart");
                if (!aB1.IsOpen() && !aB2.IsOpen())
                {
                    anOracle.Same(aN1.Distance(aN2), aB1.Distance(aB2), "Distance");
                }

                using (var aNFinite = aN1.FinitePart())
                {
                    SameBox(anOracle, aNFinite, aB1.FinitePart(), "FinitePart");
                }

                using var aNSum = new N.Bnd_Box(aN1);
                aNSum.Add(aN2);
                SameBox(anOracle, aNSum, aB1.Added(aB2), "Add(box)");
            }
        }

        anOracle.Report("Bnd_Box Add(box), open directions and Distance");
    }

    [Fact]
    public void Box_transformed()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.Bnd_Box aN1, Bnd_Box aB1) = RandomBox(anOracle, 0.1);
            (N.Bnd_Box aN2, Bnd_Box aB2) = RandomBox(anOracle, 0.1);
            (N.gp_Trsf aNT1, gp_Trsf aT1) = RandomTrsf(anOracle);
            (N.gp_Trsf aNT2, gp_Trsf aT2) = RandomTrsf(anOracle);
            using (aN1)
            using (aN2)
            using (aNT1)
            using (aNT2)
            {
                using (var aNTransformed = aN1.Transformed(aNT1))
                {
                    SameBox(anOracle, aNTransformed, aB1.Transformed(aT1), "Transformed");
                }

                anOracle.Same(aN1.IsOut(aN2, aNT1), aB1.IsOut(aB2, aT1), "IsOut(box, T)");
                anOracle.Same(aN1.IsOut(aNT1, aN2, aNT2), aB1.IsOut(aT1, aB2, aT2), "IsOut(T1, box, T2)");
            }
        }

        anOracle.Report("Bnd_Box Transformed");
    }

    [Fact]
    public void Box_IsOut_line_plane_and_band()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.Bnd_Box aNative, Bnd_Box aBox) = RandomBox(anOracle, 0.1);
            using (aNative)
            {
                gp_Pnt aP = Pnt(anOracle, -30, 30);
                gp_Dir aD = AxisBiasedDir(anOracle);
                using var aNP = Oracle.ToNative(aP);
                using var aND = Oracle.ToNative(aD);

                using var aNLin = new N.gp_Lin(aNP, aND);
                anOracle.Same(aNative.IsOut(aNLin), aBox.IsOut(new gp_Lin(aP, aD)), "IsOut(line)");

                using var aNPln = new N.gp_Pln(aNP, aND);
                anOracle.Same(aNative.IsOut(aNPln), aBox.IsOut(new gp_Pln(aP, aD)), "IsOut(plane)");

                gp_Pnt aP2 = Pnt(anOracle, -30, 30);
                using var aNP2 = Oracle.ToNative(aP2);
                anOracle.Same(aNative.IsOut(aNP, aNP2, aND), aBox.IsOut(aP, aP2, aD), "IsOut(P1, P2, D)");
            }
        }

        anOracle.Report("Bnd_Box IsOut line, plane and band");
    }

    private static gp_Pnt Pnt(Oracle theOracle, double theMin, double theMax)
        => new(theOracle.Real(theMin, theMax), theOracle.Real(theMin, theMax), theOracle.Real(theMin, theMax));

    // Compares everything a box exposes.
    private static void SameBox(Oracle theOracle, N.Bnd_Box theNative, in Bnd_Box thePort, string theWhat)
    {
        theOracle.Same(theNative.IsVoid(), thePort.IsVoid(), $"{theWhat} IsVoid");
        theOracle.Same(theNative.IsWhole(), thePort.IsWhole(), $"{theWhat} IsWhole");
        theOracle.Same(theNative.IsOpenXmin(), thePort.IsOpenXmin(), $"{theWhat} IsOpenXmin");
        theOracle.Same(theNative.IsOpenXmax(), thePort.IsOpenXmax(), $"{theWhat} IsOpenXmax");
        theOracle.Same(theNative.IsOpenYmin(), thePort.IsOpenYmin(), $"{theWhat} IsOpenYmin");
        theOracle.Same(theNative.IsOpenYmax(), thePort.IsOpenYmax(), $"{theWhat} IsOpenYmax");
        theOracle.Same(theNative.IsOpenZmin(), thePort.IsOpenZmin(), $"{theWhat} IsOpenZmin");
        theOracle.Same(theNative.IsOpenZmax(), thePort.IsOpenZmax(), $"{theWhat} IsOpenZmax");
        theOracle.Same(theNative.GetGap(), thePort.GetGap(), $"{theWhat} GetGap");
        theOracle.Same(theNative.SquareExtent(), thePort.SquareExtent(), $"{theWhat} SquareExtent");
        if (thePort.IsVoid())
        {
            return;
        }

        double aXmin = 0, aYmin = 0, aZmin = 0, aXmax = 0, aYmax = 0, aZmax = 0;
        theNative.Get(ref aXmin, ref aYmin, ref aZmin, ref aXmax, ref aYmax, ref aZmax);
        thePort.Get(out double aPXmin, out double aPYmin, out double aPZmin, out double aPXmax, out double aPYmax, out double aPZmax);
        theOracle.Same(aXmin, aPXmin, $"{theWhat} Xmin");
        theOracle.Same(aYmin, aPYmin, $"{theWhat} Ymin");
        theOracle.Same(aZmin, aPZmin, $"{theWhat} Zmin");
        theOracle.Same(aXmax, aPXmax, $"{theWhat} Xmax");
        theOracle.Same(aYmax, aPYmax, $"{theWhat} Ymax");
        theOracle.Same(aZmax, aPZmax, $"{theWhat} Zmax");
    }

    // A box made the same way natively and in the port: points, a gap, and with the given probability open
    // directions (half-lines) or a void or whole box.
    private static (N.Bnd_Box Native, Bnd_Box Port) RandomBox(Oracle theOracle, double theOpenProbability)
    {
        var aNative = new N.Bnd_Box();
        var aBox = new Bnd_Box();
        int aKind = theOracle.Int(0, 49);
        if (aKind == 0)
        {
            return (aNative, aBox); // void
        }

        int aNbPoints = theOracle.Int(1, 6);
        for (int k = 0; k < aNbPoints; k++)
        {
            gp_Pnt aP = Pnt(theOracle, -20, 20);
            using var aNP = Oracle.ToNative(aP);
            aNative.Add(aNP);
            aBox.Add(aP);
        }

        double aGap = theOracle.Real(0, 0.5);
        aNative.SetGap(aGap);
        aBox.SetGap(aGap);

        if (aKind == 1)
        {
            aNative.SetWhole();
            aBox.SetWhole();
        }
        else if (theOracle.Real(0, 1) < theOpenProbability)
        {
            gp_Dir aD = AxisBiasedDir(theOracle);
            using var aND = Oracle.ToNative(aD);
            aNative.Add(aND);
            aBox.Add(aD);
        }

        return (aNative, aBox);
    }

    // A random direction, half the time along an axis or in a coordinate plane, to reach OCCT's special cases.
    private static gp_Dir AxisBiasedDir(Oracle theOracle)
    {
        gp_Dir aD = theOracle.Dir();
        switch (theOracle.Int(0, 5))
        {
            case 0:
                return new gp_Dir(theOracle.Int(0, 1) == 0 ? gp_Dir.D.X : gp_Dir.D.NY);
            case 1:
                return new gp_Dir(0.0, aD.Y(), aD.Z() + 0.5);
            case 2:
                return new gp_Dir(aD.X() + 0.5, 0.0, aD.Z());
            default:
                return aD;
        }
    }

    // One of gp_Trsf's kinds, made the same way natively and in the port.
    private static (N.gp_Trsf Native, gp_Trsf Port) RandomTrsf(Oracle theOracle)
    {
        var aNative = new N.gp_Trsf();
        switch (theOracle.Int(0, 4))
        {
            case 0:
                return (aNative, new gp_Trsf()); // identity
            case 1:
            {
                gp_Vec aV = theOracle.Vec();
                using var aNV = Oracle.ToNative(aV);
                aNative.SetTranslation(aNV);
                return (aNative, gp_Trsf.Translation(aV));
            }
            case 2:
            {
                gp_Pnt aP = theOracle.Pnt();
                double aS = theOracle.Real(-3, 3);
                using var aNP = Oracle.ToNative(aP);
                aNative.SetScale(aNP, aS);
                return (aNative, gp_Trsf.Scale(aP, aS));
            }
            default:
            {
                gp_Ax1 anAxis = theOracle.Ax1();
                double anAngle = theOracle.Angle();
                using var aNAxis = Oracle.ToNative(anAxis);
                aNative.SetRotation(aNAxis, anAngle);
                gp_Vec aV = theOracle.Vec();
                using var aNV = Oracle.ToNative(aV);
                aNative.SetTranslationPart(aNV);
                return (aNative, gp_Trsf.Rotation(anAxis, anAngle).WithTranslationPart(aV));
            }
        }
    }
}
