// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text;
using System.Text.RegularExpressions;
using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The port's BRepPrimAPI_MakeBox, MakeWedge, MakeCylinder, MakeCone, MakeSphere, MakeTorus and MakeHalfSpace against
/// native OCCT's on random parameters (every constructor, negative box sizes, partial angles, cut spheres and tori with
/// ordinary, inverted and equal heights, apexes, flat and invalid inputs) and placements, compared through BRep text:
/// the solid, and the box's shell and six faces or the one-axis primitives' lateral face and shell. Placements are drawn
/// so that native gp_Ax2 holds the port's directions to the bit (as BRepLibMakersOracleTests). A half-space holds the
/// caller's frozen face or shell, which keeps Free = 1 where native OCCT clears it (design 4.4), so files that differ
/// are compared again with the Free column masked. Constructions that throw must throw on both sides.
/// </summary>
public sealed class BRepPrimAPIPrimitivesOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-prim-").FullName;
    private int myFileNumber;

    public void Dispose()
    {
        try
        {
            Directory.Delete(myDirectory, true);
        }
        catch (IOException)
        {
        }
    }

    private string NewFile() => Path.Combine(myDirectory, $"p{myFileNumber++}.brep");

    private string NativeText(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    private string PortText(in Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BrepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    // The Free flag is the first of the seven flag digits on their own line.
    private static string MaskFree(string theText) => Regex.Replace(theText, "(?m)^[01]([01]{6})$", "x$1");

    private sealed class Tally
    {
        public int Files;
        public int Identical;
        public int MaskedEqual;
        public int Failed;
        public int BothThrew;
        public readonly Dictionary<string, int> Kinds = [];
        public readonly Dictionary<string, int> MaskedKinds = [];
        public readonly List<string> Differences = [];
    }

    private readonly Tally myTally = new();

    private void Compare(string theWhat, N.TopoDS_Shape theNative, in Shape thePort)
    {
        string aN = NativeText(theNative);
        string aP = PortText(thePort);
        myTally.Files++;
        if (aN == aP)
        {
            myTally.Identical++;
        }
        else if (MaskFree(aN) == MaskFree(aP))
        {
            myTally.MaskedEqual++;
            string aKind = theWhat;
            myTally.MaskedKinds[aKind] = myTally.MaskedKinds.GetValueOrDefault(aKind) + 1;
        }
        else
        {
            myTally.Failed++;
            if (myTally.Differences.Count < 8)
            {
                string[] aNL = MaskFree(aN).Split('\n'), aPL = MaskFree(aP).Split('\n');
                var aDiff = new StringBuilder();
                for (int i = 0, n = 0; i < Math.Max(aNL.Length, aPL.Length) && n < 6; i++)
                {
                    string a = i < aNL.Length ? aNL[i] : "", b = i < aPL.Length ? aPL[i] : "";
                    if (a != b)
                    {
                        aDiff.Append($"  line {i + 1}: native [{a.TrimEnd()}] port [{b.TrimEnd()}]\n");
                        n++;
                    }
                }
                myTally.Differences.Add($"{theWhat} ({aNL.Length} and {aPL.Length} lines):\n{aDiff}");
            }
        }
    }

    // ------------------------------------------------------------ settled placements

    private static bool Same(N.gp_Dir theN, in Direction3D theP) => theN.X() == theP.X() && theN.Y() == theP.Y() && theN.Z() == theP.Z();

    private static Direction3D FromNative(N.gp_Dir theD) => new(theD.X(), theD.Y(), theD.Z());

    // A random placement that native gp_Ax2 holds with the port's directions to the bit.
    private static Frame3D Ax2(Oracle theO)
    {
        while (true)
        {
            Point3D aP = new(theO.Real(-20, 20), theO.Real(-20, 20), theO.Real(-20, 20));
            using var aNP = Oracle.ToNative(aP);
            using var aN0 = Oracle.ToNative(theO.Dir());
            using var aX0 = Oracle.ToNative(theO.Dir());
            var aNAx = new N.gp_Ax2(aNP, aN0, aX0);
            for (int i = 0; i < 4; i++)
            {
                using var aD = aNAx.Direction();
                using var aX = aNAx.XDirection();
                aNAx = new N.gp_Ax2(aNP, aD, aX);
            }
            using var aND = aNAx.Direction();
            using var aNX = aNAx.XDirection();
            var anAx = new Frame3D(aP, FromNative(aND), FromNative(aNX));
            using var aN2 = Oracle.ToNative(anAx);
            using var aD2 = aN2.Direction();
            using var aX2 = aN2.XDirection();
            using var aY2 = aN2.YDirection();
            if (Same(aD2, anAx.Direction()) && Same(aX2, anAx.XDirection()) && Same(aY2, anAx.YDirection()))
            {
                return anAx;
            }
        }
    }

    // ------------------------------------------------------------ one case

    // Builds both makers; compares a throw on both sides, or the shapes the compare step reads.
    private void Case<TN, TP>(string theKind, Func<TN> theNative, Func<TP> thePort, Action<TN, TP> theCompare)
        where TN : class, IDisposable
    {
        myTally.Kinds[theKind] = myTally.Kinds.GetValueOrDefault(theKind) + 1;
        TN? aN = null;
        string? aNErr = null, aPErr = null;
        TP? aP = default;
        try
        {
            aN = theNative();
        }
        catch (N.OcctException e)
        {
            aNErr = e.Message;
        }
        try
        {
            aP = thePort();
        }
        catch (FailureException e)
        {
            aPErr = global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType());
        }
        Assert.True((aNErr is null) == (aPErr is null), $"{theKind}: native {aNErr ?? "built"}, port {aPErr ?? "built"}");
        if (aN is null)
        {
            myTally.BothThrew++;
            return;
        }
        using (aN)
        {
            theCompare(aN, aP!);
        }
    }

    // A shape read from both makers, or a throw on both sides.
    private void Shapes(string theWhat, Func<N.TopoDS_Shape> theNative, Func<Shape> thePort)
    {
        N.TopoDS_Shape? aN = null;
        bool aNThrew = false, aPThrew = false;
        string aPMessage = "";
        Shape aP = new();
        try
        {
            aN = theNative();
        }
        catch (N.OcctException)
        {
            aNThrew = true;
        }
        try
        {
            aP = thePort();
        }
        catch (FailureException e)
        {
            aPThrew = true;
            aPMessage = e.ToString();
        }
        Assert.True(aNThrew == aPThrew, $"{theWhat}: native {(aNThrew ? "threw" : "built")}, port {(aPThrew ? "threw " + aPMessage : "built")}");
        if (aN is null)
        {
            myTally.BothThrew++;
            return;
        }
        using (aN)
        {
            Assert.True(aP.TShape()!.IsFrozen, $"{theWhat}: the published shape isn't frozen");
            Compare(theWhat, aN, aP);
        }
    }

    private void OneAxisShapes(string theKind, N.BRepPrimAPI_MakeOneAxis theN, BRepPrimAPI_MakeOneAxis theP)
    {
        Shapes(theKind + " Shape", theN.Shape, () => theP.Shape());
        Shapes(theKind + " Face", () => theN.Face(), () => theP.Face());
        Shapes(theKind + " Shell", () => theN.Shell(), () => theP.Shell());
        Shapes(theKind + " Solid", () => theN.Solid(), () => theP.Solid());
    }

    // A size that is sometimes negative, rarely flat.
    private static double Size(Oracle theO, bool theSigned)
    {
        int k = theO.Int(0, 40);
        if (k == 0)
        {
            return 0.0;
        }
        double s = theO.Real(0.2, 20);
        return theSigned && theO.Bool() ? -s : s;
    }

    private static double PartialAngle(Oracle theO) => theO.Int(0, 9) == 0 ? 2 * Math.PI : theO.Real(0.1, 2 * Math.PI - 0.1);

    [Fact]
    public void The_primitive_makers_build_the_shapes_native_OCCT_builds()
    {
        var anOracle = new Oracle(output, 20261029);
        foreach (int iCase in Tape.Budget(700))
        {
            switch (iCase % 7)
            {
                case 0:
                    BoxCase(anOracle);
                    break;
                case 1:
                    WedgeCase(anOracle);
                    break;
                case 2:
                    CylinderCase(anOracle);
                    break;
                case 3:
                    ConeCase(anOracle);
                    break;
                case 4:
                    SphereCase(anOracle);
                    break;
                case 5:
                    TorusCase(anOracle);
                    break;
                default:
                    HalfSpaceCase(anOracle);
                    break;
            }
        }

        output.WriteLine($"{myTally.Files} files, {myTally.Identical} byte-identical, {myTally.MaskedEqual} equal with the Free "
                         + $"column masked, {myTally.Failed} different; {myTally.BothThrew} throws on both sides; by kind: "
                         + string.Join(", ", myTally.Kinds.OrderBy(k => k.Key).Select(k => $"{k.Key} {k.Value}"))
                         + "; masked files by kind: "
                         + string.Join(", ", myTally.MaskedKinds.OrderBy(k => k.Key).Select(k => $"{k.Key} {k.Value}")));
        foreach (string aD in myTally.Differences)
        {
            output.WriteLine(aD);
        }
        Assert.Equal(0, myTally.Failed);
    }

    private void BoxCase(Oracle theO)
    {
        int aForm = theO.Int(0, 3);
        double dx = Size(theO, aForm != 3), dy = Size(theO, aForm != 3), dz = Size(theO, aForm != 3);
        Point3D aP = new(theO.Real(-20, 20), theO.Real(-20, 20), theO.Real(-20, 20));
        Point3D aP2 = new(aP.X() + dx, aP.Y() + dy, aP.Z() + dz);
        Frame3D anAx = aForm == 3 ? Ax2(theO) : new Frame3D();
        Case("box" + aForm,
             () =>
             {
                 using var aNP = Oracle.ToNative(aP);
                 using var aNP2 = Oracle.ToNative(aP2);
                 using var aNAx = Oracle.ToNative(anAx);
                 return aForm switch
                 {
                     0 => new N.BRepPrimAPI_MakeBox(dx, dy, dz),
                     1 => new N.BRepPrimAPI_MakeBox(aNP, dx, dy, dz),
                     2 => new N.BRepPrimAPI_MakeBox(aNP, aNP2),
                     _ => new N.BRepPrimAPI_MakeBox(aNAx, dx, dy, dz),
                 };
             },
             () => aForm switch
             {
                 0 => new BRepPrimAPI_MakeBox(dx, dy, dz),
                 1 => new BRepPrimAPI_MakeBox(aP, dx, dy, dz),
                 2 => new BRepPrimAPI_MakeBox(aP, aP2),
                 _ => new BRepPrimAPI_MakeBox(anAx, dx, dy, dz),
             },
             (aN, aPM) =>
             {
                 // The port publishes a face once every face is built (decision 467); native OCCT hands out the face
                 // and completes its edges in place as the other faces are built. So native's faces are all built
                 // before they are compared, in the port's order (XMin, XMax, YMin, YMax, ZMin, ZMax), which orders the pcurves of
                 // a degenerated box's edges.
                 Shapes("box BottomFace", () =>
                 {
                     foreach (Func<N.TopoDS_Face> aF in (Func<N.TopoDS_Face>[])[aN.BackFace, aN.FrontFace, aN.LeftFace, aN.RightFace, aN.BottomFace, aN.TopFace])
                     {
                         aF().Dispose();
                     }
                     return aN.BottomFace();
                 }, () => aPM.BottomFace());
                 Shapes("box TopFace", () => aN.TopFace(), () => aPM.TopFace());
                 Shapes("box Shape", aN.Shape, () => aPM.Shape());
                 Shapes("box BackFace", () => aN.BackFace(), () => aPM.BackFace());
                 Shapes("box FrontFace", () => aN.FrontFace(), () => aPM.FrontFace());
                 Shapes("box LeftFace", () => aN.LeftFace(), () => aPM.LeftFace());
                 Shapes("box RightFace", () => aN.RightFace(), () => aPM.RightFace());
                 Shapes("box Solid", () => aN.Solid(), () => aPM.Solid());
                 Shapes("box Shell", () => aN.Shell(), () => aPM.Shell());
                 Shapes("box Shape after Shell", aN.Shape, () => aPM.Shape());
             });
    }

    private void WedgeCase(Oracle theO)
    {
        int aForm = theO.Int(0, 3);
        double dx = theO.Real(0.5, 20), dy = theO.Real(0.5, 20), dz = theO.Real(0.5, 20);
        // ltx in [0, dx] (0 makes a degenerated top edge), sometimes beyond or negative
        int aKind = theO.Int(0, 9);
        double ltx = aKind == 0 ? 0.0 : aKind == 1 ? -theO.Real(0.1, 2) : aKind == 2 ? dx * theO.Real(1, 2) : theO.Real(0, dx);
        double xmin = theO.Real(-5, dx), xmax = theO.Int(0, 6) == 0 ? xmin : xmin + theO.Real(0, 10);
        double zmin = theO.Real(-5, dz), zmax = theO.Int(0, 6) == 0 ? zmin : zmin + theO.Real(0, 10);
        Frame3D anAx = aForm % 2 == 1 ? Ax2(theO) : new Frame3D();
        Case("wedge" + aForm,
             () =>
             {
                 using var aNAx = Oracle.ToNative(anAx);
                 return aForm switch
                 {
                     0 => new N.BRepPrimAPI_MakeWedge(dx, dy, dz, ltx),
                     1 => new N.BRepPrimAPI_MakeWedge(aNAx, dx, dy, dz, ltx),
                     2 => new N.BRepPrimAPI_MakeWedge(dx, dy, dz, xmin, zmin, xmax, zmax),
                     _ => new N.BRepPrimAPI_MakeWedge(aNAx, dx, dy, dz, xmin, zmin, xmax, zmax),
                 };
             },
             () => aForm switch
             {
                 0 => new BRepPrimAPI_MakeWedge(dx, dy, dz, ltx),
                 1 => new BRepPrimAPI_MakeWedge(anAx, dx, dy, dz, ltx),
                 2 => new BRepPrimAPI_MakeWedge(dx, dy, dz, xmin, zmin, xmax, zmax),
                 _ => new BRepPrimAPI_MakeWedge(anAx, dx, dy, dz, xmin, zmin, xmax, zmax),
             },
             (aN, aPM) =>
             {
                 Shapes("wedge Shape", aN.Shape, () => aPM.Shape());
                 Shapes("wedge Shell", () => aN.Shell(), () => aPM.Shell());
                 Shapes("wedge Solid", () => aN.Solid(), () => aPM.Solid());
             });
    }

    private void CylinderCase(Oracle theO)
    {
        int aForm = theO.Int(0, 3);
        double R = theO.Real(0.2, 10), H = theO.Real(0.2, 20), anAngle = PartialAngle(theO);
        Frame3D anAx = aForm >= 2 ? Ax2(theO) : new Frame3D();
        Case("cylinder" + aForm,
             () =>
             {
                 using var aNAx = Oracle.ToNative(anAx);
                 return aForm switch
                 {
                     0 => new N.BRepPrimAPI_MakeCylinder(R, H),
                     1 => new N.BRepPrimAPI_MakeCylinder(R, H, anAngle),
                     2 => new N.BRepPrimAPI_MakeCylinder(aNAx, R, H),
                     _ => new N.BRepPrimAPI_MakeCylinder(aNAx, R, H, anAngle),
                 };
             },
             () => aForm switch
             {
                 0 => new BRepPrimAPI_MakeCylinder(R, H),
                 1 => new BRepPrimAPI_MakeCylinder(R, H, anAngle),
                 2 => new BRepPrimAPI_MakeCylinder(anAx, R, H),
                 _ => new BRepPrimAPI_MakeCylinder(anAx, R, H, anAngle),
             },
             (aN, aPM) => OneAxisShapes("cylinder", aN, aPM));
    }

    private void ConeCase(Oracle theO)
    {
        int aForm = theO.Int(0, 3);
        int aKind = theO.Int(0, 9);
        double R1 = aKind == 0 ? 0.0 : theO.Real(0.2, 10);
        double R2 = aKind == 1 ? 0.0 : aKind == 2 ? R1 : theO.Real(0.2, 10);
        double H = aKind == 3 ? 0.0 : theO.Real(0.2, 20), anAngle = PartialAngle(theO);
        Frame3D anAx = aForm >= 2 ? Ax2(theO) : new Frame3D();
        Case("cone" + aForm,
             () =>
             {
                 using var aNAx = Oracle.ToNative(anAx);
                 return aForm switch
                 {
                     0 => new N.BRepPrimAPI_MakeCone(R1, R2, H),
                     1 => new N.BRepPrimAPI_MakeCone(R1, R2, H, anAngle),
                     2 => new N.BRepPrimAPI_MakeCone(aNAx, R1, R2, H),
                     _ => new N.BRepPrimAPI_MakeCone(aNAx, R1, R2, H, anAngle),
                 };
             },
             () => aForm switch
             {
                 0 => new BRepPrimAPI_MakeCone(R1, R2, H),
                 1 => new BRepPrimAPI_MakeCone(R1, R2, H, anAngle),
                 2 => new BRepPrimAPI_MakeCone(anAx, R1, R2, H),
                 _ => new BRepPrimAPI_MakeCone(anAx, R1, R2, H, anAngle),
             },
             (aN, aPM) => OneAxisShapes("cone", aN, aPM));
    }

    private void SphereCase(Oracle theO)
    {
        int aForm = theO.Int(0, 11);
        double R = theO.Real(0.2, 10);
        // a wedge angle (sometimes negative for the (R, angle) form), and a segment between two latitudes
        double anAngle = theO.Int(0, 3) == 0 && aForm == 1 ? -PartialAngle(theO) : PartialAngle(theO);
        double a1 = theO.Int(0, 4) == 0 ? -Math.PI / 2 : theO.Real(-Math.PI / 2, Math.PI / 2 - 0.2);
        double a2 = theO.Int(0, 4) == 0 ? Math.PI / 2 : theO.Real(a1 + 0.1, Math.PI / 2);
        Point3D aC = new(theO.Real(-20, 20), theO.Real(-20, 20), theO.Real(-20, 20));
        Frame3D anAx = aForm >= 8 ? Ax2(theO) : new Frame3D();
        Case("sphere" + aForm,
             () =>
             {
                 using var aNC = Oracle.ToNative(aC);
                 using var aNAx = Oracle.ToNative(anAx);
                 return aForm switch
                 {
                     0 => new N.BRepPrimAPI_MakeSphere(R),
                     1 => new N.BRepPrimAPI_MakeSphere(R, anAngle),
                     2 => new N.BRepPrimAPI_MakeSphere(R, a1, a2),
                     3 => new N.BRepPrimAPI_MakeSphere(R, a1, a2, anAngle),
                     4 => new N.BRepPrimAPI_MakeSphere(aNC, R),
                     5 => new N.BRepPrimAPI_MakeSphere(aNC, R, anAngle),
                     6 => new N.BRepPrimAPI_MakeSphere(aNC, R, a1, a2),
                     7 => new N.BRepPrimAPI_MakeSphere(aNC, R, a1, a2, anAngle),
                     8 => new N.BRepPrimAPI_MakeSphere(aNAx, R),
                     9 => new N.BRepPrimAPI_MakeSphere(aNAx, R, anAngle),
                     10 => new N.BRepPrimAPI_MakeSphere(aNAx, R, a1, a2),
                     _ => new N.BRepPrimAPI_MakeSphere(aNAx, R, a1, a2, anAngle),
                 };
             },
             () => aForm switch
             {
                 0 => new BRepPrimAPI_MakeSphere(R),
                 1 => new BRepPrimAPI_MakeSphere(R, anAngle),
                 2 => new BRepPrimAPI_MakeSphere(R, a1, a2),
                 3 => new BRepPrimAPI_MakeSphere(R, a1, a2, anAngle),
                 4 => new BRepPrimAPI_MakeSphere(aC, R),
                 5 => new BRepPrimAPI_MakeSphere(aC, R, anAngle),
                 6 => new BRepPrimAPI_MakeSphere(aC, R, a1, a2),
                 7 => new BRepPrimAPI_MakeSphere(aC, R, a1, a2, anAngle),
                 8 => new BRepPrimAPI_MakeSphere(anAx, R),
                 9 => new BRepPrimAPI_MakeSphere(anAx, R, anAngle),
                 10 => new BRepPrimAPI_MakeSphere(anAx, R, a1, a2),
                 _ => new BRepPrimAPI_MakeSphere(anAx, R, a1, a2, anAngle),
             },
             (aN, aPM) => OneAxisShapes("sphere", aN, aPM));
    }

    private void TorusCase(Oracle theO)
    {
        int aForm = theO.Int(0, 7);
        double R2 = theO.Real(0.2, 5), R1 = R2 + theO.Real(0.2, 10), anAngle = PartialAngle(theO);
        // V ranges: crossing 0, inverted heights (90..270 degrees), equal heights, the whole circle
        double a1, a2;
        switch (theO.Int(0, 3))
        {
            case 0:
                a1 = theO.Real(-Math.PI / 2, 0);
                a2 = theO.Real(0.1, Math.PI / 2);
                break;
            case 1:
                a1 = theO.Real(Math.PI / 2, Math.PI);
                a2 = theO.Real(Math.PI + 0.1, 3 * Math.PI / 2);
                break;
            case 2:
                a1 = theO.Real(0, Math.PI / 2 - 0.1);
                a2 = Math.PI - a1;
                break;
            default:
                a1 = theO.Real(-Math.PI, Math.PI);
                a2 = a1 + theO.Real(0.2, 2 * Math.PI - 0.2);
                break;
        }
        Frame3D anAx = aForm >= 4 ? Ax2(theO) : new Frame3D();
        Case("torus" + aForm,
             () =>
             {
                 using var aNAx = Oracle.ToNative(anAx);
                 return aForm switch
                 {
                     0 => new N.BRepPrimAPI_MakeTorus(R1, R2),
                     1 => new N.BRepPrimAPI_MakeTorus(R1, R2, anAngle),
                     2 => new N.BRepPrimAPI_MakeTorus(R1, R2, a1, a2),
                     3 => new N.BRepPrimAPI_MakeTorus(R1, R2, a1, a2, anAngle),
                     4 => new N.BRepPrimAPI_MakeTorus(aNAx, R1, R2),
                     5 => new N.BRepPrimAPI_MakeTorus(aNAx, R1, R2, anAngle),
                     6 => new N.BRepPrimAPI_MakeTorus(aNAx, R1, R2, a1, a2),
                     _ => new N.BRepPrimAPI_MakeTorus(aNAx, R1, R2, a1, a2, anAngle),
                 };
             },
             () => aForm switch
             {
                 0 => new BRepPrimAPI_MakeTorus(R1, R2),
                 1 => new BRepPrimAPI_MakeTorus(R1, R2, anAngle),
                 2 => new BRepPrimAPI_MakeTorus(R1, R2, a1, a2),
                 3 => new BRepPrimAPI_MakeTorus(R1, R2, a1, a2, anAngle),
                 4 => new BRepPrimAPI_MakeTorus(anAx, R1, R2),
                 5 => new BRepPrimAPI_MakeTorus(anAx, R1, R2, anAngle),
                 6 => new BRepPrimAPI_MakeTorus(anAx, R1, R2, a1, a2),
                 _ => new BRepPrimAPI_MakeTorus(anAx, R1, R2, a1, a2, anAngle),
             },
             (aN, aPM) => OneAxisShapes("torus", aN, aPM));
    }

    // A half-space on a face or the shell of a box, a cylinder or a sphere (made by both sides' makers), with the
    // reference point anywhere around it.
    private void HalfSpaceCase(Oracle theO)
    {
        Frame3D anAx = Ax2(theO);
        int aBase = theO.Int(0, 2);
        double a = theO.Real(1, 10), b = theO.Real(1, 10), c = theO.Real(1, 10);
        bool isShell = theO.Bool();
        int aFaceIndex = theO.Int(1, 6);
        Point3D aRef = new(anAx.Location().X() + theO.Real(-15, 15), anAx.Location().Y() + theO.Real(-15, 15),
                          anAx.Location().Z() + theO.Real(-15, 15));

        using var aNAx = Oracle.ToNative(anAx);
        using N.BRepBuilderAPI_MakeShape aNBase = aBase switch
        {
            0 => new N.BRepPrimAPI_MakeBox(aNAx, a, b, c),
            1 => new N.BRepPrimAPI_MakeCylinder(aNAx, a, b),
            _ => new N.BRepPrimAPI_MakeSphere(aNAx, a),
        };
        BRepBuilderAPI_MakeShape aPBase = aBase switch
        {
            0 => new BRepPrimAPI_MakeBox(anAx, a, b, c),
            1 => new BRepPrimAPI_MakeCylinder(anAx, a, b),
            _ => new BRepPrimAPI_MakeSphere(anAx, a),
        };
        using N.TopoDS_Shape aNSolid = aNBase.Shape();
        Shape aPSolid = aPBase.Shape();

        // the shell, or the face of that index in the solid (both sides explore the same structure)
        N.TopoDS_Shape aNBound;
        Shape aPBound;
        {
            var aType = isShell ? N.TopAbs_ShapeEnum.TopAbs_SHELL : N.TopAbs_ShapeEnum.TopAbs_FACE;
            int aCount = 0;
            for (var aPExp0 = new Explorer(aPSolid, isShell ? ShapeType.Shell : ShapeType.Face);
                 aPExp0.More(); aPExp0.Next())
            {
                aCount++;
            }
            int aK = Math.Min(aFaceIndex, aCount);
            using var aNExp = new N.TopExp_Explorer(aNSolid, aType);
            var aPExp = new Explorer(aPSolid, isShell ? ShapeType.Shell : ShapeType.Face);
            for (int i = 1; i < aK; i++)
            {
                aNExp.Next();
                aPExp.Next();
            }
            aNBound = aNExp.Current();
            aPBound = aPExp.Current();
        }

        using (aNBound)
        {
            using var aNRef = Oracle.ToNative(aRef);
            string aKind = (isShell ? "halfspace shell " : "halfspace face ") + aBase;
            Case(aKind,
                 () => isShell
                     ? new N.BRepPrimAPI_MakeHalfSpace(N.TopoDS.Shell(aNBound), aNRef)
                     : new N.BRepPrimAPI_MakeHalfSpace(N.TopoDS.Face(aNBound), aNRef),
                 () => isShell
                     ? new BRepPrimAPI_MakeHalfSpace(ShapeCasts.Shell(aPBound), aRef)
                     : new BRepPrimAPI_MakeHalfSpace(ShapeCasts.Face(aPBound), aRef),
                 (aN, aPM) =>
                 {
                     Assert.Equal(aN.IsDone(), aPM.IsDone());
                     if (aN.IsDone())
                     {
                         Shapes(aKind + " Solid", () => aN.Solid(), () => aPM.Solid());
                         Shapes(aKind + " Shape", aN.Shape, () => aPM.Shape());
                     }
                 });
        }
    }
}
