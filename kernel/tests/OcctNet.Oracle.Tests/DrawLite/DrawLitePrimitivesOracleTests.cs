// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using System.Globalization;
using System.Text;
using System.Text.RegularExpressions;
using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Unit 3M's Draw-lite commands (box in every form, wedge, pcylinder, pcone, psphere, ptorus with and without a plane,
/// halfspace, prism, revol, bounding -shape) on random parameters against the native BRepPrimAPI makers Draw calls with
/// the same numbers, compared through BRep text. The commands' words are the numbers written round-trip, and Draw's
/// degree conversion is done the same way on the native side. Placements whose native gp_Ax2 doesn't come out with the
/// port's directions to the bit are drawn again. A shape the command builds on a frozen input (halfspace, prism, revol)
/// keeps the input's Free flag where native OCCT clears it (design 4.4): files that differ are compared again with the
/// Free column masked.
/// </summary>
public sealed class DrawLitePrimitivesOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-drawprim-").FullName;
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

    private string NewFile() => Path.Combine(myDirectory, $"s{myFileNumber++}.brep");

    private string NativeText(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    private string PortText(in TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BRepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    // The Free flag is the first of the seven flag digits on their own line.
    private static string MaskFree(string theText) => Regex.Replace(theText, "(?m)^[01]([01]{6})$", "x$1");

    private static string R(double theValue) => theValue.ToString("R", CultureInfo.InvariantCulture);

    private static string R(in Point3D theP) => $"{R(theP.X())} {R(theP.Y())} {R(theP.Z())}";

    private static string R(in Direction3D theD) => $"{R(theD.X())} {R(theD.Y())} {R(theD.Z())}";

    private const double DegToRad = Math.PI / 180.0;

    private static bool Same(N.gp_Dir theN, in Direction3D theP) => theN.X() == theP.X() && theN.Y() == theP.Y() && theN.Z() == theP.Z();

    private static bool Same(N.gp_Ax2 theN, in Frame3D theP)
    {
        using N.gp_Pnt aL = theN.Location();
        using N.gp_Dir aD = theN.Direction();
        using N.gp_Dir aX = theN.XDirection();
        using N.gp_Dir aY = theN.YDirection();
        return aL.X() == theP.Location().X() && aL.Y() == theP.Location().Y() && aL.Z() == theP.Location().Z()
               && Same(aD, theP.Direction()) && Same(aX, theP.XDirection()) && Same(aY, theP.YDirection());
    }

    // A direction that native OCCT and the port normalise to the same numbers.
    private static Direction3D Dir(Oracle theO)
    {
        while (true)
        {
            Direction3D aD = Oracle.Settled(theO.Dir());
            using var aND = Oracle.ToNative(aD);
            if (Same(aND, aD))
            {
                return aD;
            }
        }
    }

    private static string myLastBox = "";
    private int myFiles;
    private int myIdentical;
    private int myMasked;
    private int myErrorsEqual;
    private int myNullBoth;
    private int mySkipped;
    private readonly List<string> myDifferences = [];
    private readonly Dictionary<string, int> myByCommand = [];

    private void Compare(string theScript, N.TopoDS_Shape theNative, in TopoDS_Shape thePort)
    {
        string aN = NativeText(theNative);
        string aP = PortText(thePort);
        myFiles++;
        string aCommand = theScript.Split(' ')[0];
        myByCommand[aCommand] = myByCommand.GetValueOrDefault(aCommand) + 1;
        if (aN == aP)
        {
            myIdentical++;
        }
        else if (MaskFree(aN) == MaskFree(aP))
        {
            myMasked++;
        }
        else if (myDifferences.Count < 5)
        {
            myDifferences.Add($"{theScript}:\n--- native\n{aN}\n--- port\n{aP}");
        }
        else
        {
            myDifferences.Add(theScript);
        }
    }

    /// <summary>Runs the command; compares its result with the native shape, or both failures.</summary>
    private void Run(DrawInterpreter theDI, string theScript, Func<N.TopoDS_Shape> theNative)
    {
        N.TopoDS_Shape? aNative = null;
        string aNativeError = "not done";
        try
        {
            aNative = theNative();
        }
        catch (Exception anException)
        {
            aNative = null;
            aNativeError = global::Kerf.Engine.Foundation.OcctTypeNames.Of(anException.GetType()) + ": " + anException.Message;
        }

        bool isPortDone;
        try
        {
            theDI.Eval("unset -nocomplain r");
            string aText = theDI.Eval(theScript);
            isPortDone = aText.Length == 0 && theDI.GetDrawable("r") is not null;
        }
        catch (DrawException)
        {
            isPortDone = false;
        }

        if (aNative is not null && aNative.IsNull() && isPortDone && theDI.GetShape("r").IsNull())
        {
            // a null result on both sides (an edge on the axis of a revolution: done, and null)
            myNullBoth++;
            aNative.Dispose();
            return;
        }

        if (aNative is null || aNative.IsNull())
        {
            if (isPortDone)
            {
                myDifferences.Add($"{myLastBox}; {theScript}: native fails ({aNativeError}), the port builds");
            }
            else
            {
                myErrorsEqual++;
            }

            aNative?.Dispose();
            return;
        }

        if (!isPortDone)
        {
            myDifferences.Add($"{theScript}: the port fails, native builds");
            aNative.Dispose();
            return;
        }

        Compare(theScript, aNative, theDI.GetShape("r"));
        aNative.Dispose();
    }

    private static N.gp_Pnt P(in Point3D theP) => Oracle.ToNative(theP);

    private static N.gp_Pnt P(double x, double y, double z) => new(x, y, z);

    private static N.gp_Ax2 Ax2Native(in Point3D theP, in Direction3D theN, in Direction3D theVx)
    {
        using N.gp_Pnt aP = P(theP);
        using N.gp_Dir aN = Oracle.ToNative(theN);
        using N.gp_Dir aX = Oracle.ToNative(theVx);
        return new N.gp_Ax2(aP, aN, aX);
    }

    [Fact]
    public void Primitive_and_sweep_commands_bind_what_native_makers_build()
    {
        var anO = new Oracle(output, 3260);
        var aDI = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        foreach (int aCase in Tape.Budget(1200))
        {
            int aKind = aCase % 24;
            double a = anO.Real(0.5, 10), b = anO.Real(0.5, 10), c = anO.Real(0.5, 10);
            double sa = anO.Bool() ? a : -a, sb = anO.Bool() ? b : -b, sc = anO.Bool() ? c : -c;
            Point3D anOrigin = anO.Pnt();
            switch (aKind)
            {
                case 0:
                    Run(aDI, $"box r {R(sa)} {R(sb)} {R(sc)}",
                        () => { using var m = new N.BRepPrimAPI_MakeBox(P(0, 0, 0), sa, sb, sc); return m.Solid(); });
                    break;
                case 1:
                    Run(aDI, $"box r {R(anOrigin)} {R(a)} {R(b)} {R(c)}",
                        () => { using var m = new N.BRepPrimAPI_MakeBox(P(anOrigin), a, b, c); return m.Solid(); });
                    break;
                case 2:
                {
                    Point3D aMax = anO.Pnt();
                    Run(aDI, $"box r -min {R(anOrigin)} -max {R(aMax)}",
                        () => { using var m = new N.BRepPrimAPI_MakeBox(P(anOrigin), P(aMax)); return m.Solid(); });
                    break;
                }
                case 3:
                {
                    Run(aDI, $"box r -size {R(sa)} {R(sb)} {R(sc)} -MIN {R(anOrigin)}",
                        () => { using var m = new N.BRepPrimAPI_MakeBox(P(anOrigin), sa, sb, sc); return m.Solid(); });
                    break;
                }
                case 4:
                {
                    Direction3D aD = Dir(anO), aX = Dir(anO);
                    Frame3D anAx;
                    try
                    {
                        anAx = new Frame3D(anOrigin, aD, aX);
                    }
                    catch (FailureException)
                    {
                        break;
                    }

                    using N.gp_Ax2 aNAx = Ax2Native(anOrigin, aD, aX);
                    if (!Same(aNAx, anAx))
                    {
                        mySkipped++;
                        break;
                    }

                    Run(aDI, $"box r {R(anOrigin)} {R(a)} {R(b)} {R(c)} -dir {R(aD)} -xdir {R(aX)}",
                        () => { using var m = new N.BRepPrimAPI_MakeBox(aNAx, a, b, c); return m.Solid(); });
                    break;
                }
                case 5:
                {
                    // a preview: zero, one, two or three thin dimensions
                    double dx = anO.Bool() ? 0 : sa, dy = anO.Bool() ? 0 : sb, dz = anO.Bool() ? 0 : sc;
                    Run(aDI, $"box r {R(anOrigin)} {R(dx)} {R(dy)} {R(dz)} -preview",
                        () => NativePreview(anOrigin, dx, dy, dz));
                    break;
                }
                case 6:
                {
                    double ltx = anO.Bool() ? 0 : anO.Real(0, 2 * a);
                    Run(aDI, $"wedge r {R(a)} {R(b)} {R(c)} {R(ltx)}",
                        () => { using var m = new N.BRepPrimAPI_MakeWedge(a, b, c, ltx); return m.Solid(); });
                    break;
                }
                case 7:
                {
                    double xmin = anO.Real(-a, a), xmax = anO.Real(xmin, 2 * a), zmin = anO.Real(-c, c), zmax = anO.Real(zmin, 2 * c);
                    Run(aDI, $"wedge r {R(a)} {R(b)} {R(c)} {R(xmin)} {R(zmin)} {R(xmax)} {R(zmax)}",
                        () => { using var m = new N.BRepPrimAPI_MakeWedge(a, b, c, xmin, zmin, xmax, zmax); return m.Solid(); });
                    break;
                }
                case 8:
                {
                    Direction3D aD = Dir(anO), aX = Dir(anO);
                    Frame3D anAx;
                    try
                    {
                        anAx = new Frame3D(anOrigin, aD, aX);
                    }
                    catch (FailureException)
                    {
                        break;
                    }

                    using N.gp_Ax2 aNAx = Ax2Native(anOrigin, aD, aX);
                    if (!Same(aNAx, anAx))
                    {
                        mySkipped++;
                        break;
                    }

                    if (anO.Bool())
                    {
                        double ltx = anO.Real(0, 2 * a);
                        Run(aDI, $"wedge r {R(anOrigin)} {R(aD)} {R(aX)} {R(a)} {R(b)} {R(c)} {R(ltx)}",
                            () => { using var m = new N.BRepPrimAPI_MakeWedge(aNAx, a, b, c, ltx); return m.Solid(); });
                    }
                    else
                    {
                        double xmin = anO.Real(-a, a), xmax = anO.Real(xmin, 2 * a), zmin = anO.Real(-c, c), zmax = anO.Real(zmin, 2 * c);
                        Run(aDI, $"wedge r {R(anOrigin)} {R(aD)} {R(aX)} {R(a)} {R(b)} {R(c)} {R(xmin)} {R(zmin)} {R(xmax)} {R(zmax)}",
                            () => { using var m = new N.BRepPrimAPI_MakeWedge(aNAx, a, b, c, xmin, zmin, xmax, zmax); return m.Solid(); });
                    }
                    break;
                }
                case 9:
                {
                    double anAngle = anO.Real(1, 359);
                    if (anO.Bool())
                    {
                        Run(aDI, $"pcylinder r {R(a)} {R(b)}",
                            () => { using var m = new N.BRepPrimAPI_MakeCylinder(a, b); return m.Solid(); });
                    }
                    else
                    {
                        Run(aDI, $"pcylinder r {R(a)} {R(b)} {R(anAngle)}",
                            () => { using var m = new N.BRepPrimAPI_MakeCylinder(a, b, anAngle * DegToRad); return m.Solid(); });
                    }
                    break;
                }
                case 10:
                {
                    if (!Plane(aDI, anO, out N.gp_Ax2? aNAx))
                    {
                        break;
                    }

                    double anAngle = anO.Real(1, 359);
                    if (anO.Bool())
                    {
                        Run(aDI, $"pcylinder r pl {R(a)} {R(b)}",
                            () => { using var m = new N.BRepPrimAPI_MakeCylinder(aNAx!, a, b); return m.Solid(); });
                    }
                    else
                    {
                        Run(aDI, $"pcylinder r pl {R(a)} {R(b)} {R(anAngle)}",
                            () => { using var m = new N.BRepPrimAPI_MakeCylinder(aNAx!, a, b, anAngle * DegToRad); return m.Solid(); });
                    }
                    aNAx!.Dispose();
                    break;
                }
                case 11:
                {
                    double r2 = anO.Int(0, 3) == 0 ? 0 : b;
                    double anAngle = anO.Real(1, 359);
                    if (anO.Bool())
                    {
                        Run(aDI, $"pcone r {R(a)} {R(r2)} {R(c)}",
                            () => { using var m = new N.BRepPrimAPI_MakeCone(a, r2, c); return m.Solid(); });
                    }
                    else
                    {
                        Run(aDI, $"pcone r {R(a)} {R(r2)} {R(c)} {R(anAngle)}",
                            () => { using var m = new N.BRepPrimAPI_MakeCone(a, r2, c, anAngle * DegToRad); return m.Solid(); });
                    }
                    break;
                }
                case 12:
                {
                    if (!Plane(aDI, anO, out N.gp_Ax2? aNAx))
                    {
                        break;
                    }

                    double r1 = anO.Int(0, 3) == 0 ? 0 : a;
                    double anAngle = anO.Real(1, 359);
                    if (anO.Bool())
                    {
                        Run(aDI, $"pcone r pl {R(r1)} {R(b)} {R(c)}",
                            () => { using var m = new N.BRepPrimAPI_MakeCone(aNAx!, r1, b, c); return m.Solid(); });
                    }
                    else
                    {
                        Run(aDI, $"pcone r pl {R(r1)} {R(b)} {R(c)} {R(anAngle)}",
                            () => { using var m = new N.BRepPrimAPI_MakeCone(aNAx!, r1, b, c, anAngle * DegToRad); return m.Solid(); });
                    }
                    aNAx!.Dispose();
                    break;
                }
                case 13:
                {
                    double a1 = anO.Real(-90, 89), a2 = anO.Real(a1 + 0.5, 90), anAngle = anO.Real(1, 359);
                    switch (anO.Int(0, 3))
                    {
                        case 0:
                            Run(aDI, $"psphere r {R(a)}", () => { using var m = new N.BRepPrimAPI_MakeSphere(a); return m.Solid(); });
                            break;
                        case 1:
                            Run(aDI, $"psphere r {R(a)} {R(anAngle)}",
                                () => { using var m = new N.BRepPrimAPI_MakeSphere(a, anAngle * DegToRad); return m.Solid(); });
                            break;
                        case 2:
                            Run(aDI, $"psphere r {R(a)} {R(a1)} {R(a2)}",
                                () => { using var m = new N.BRepPrimAPI_MakeSphere(a, a1 * DegToRad, a2 * DegToRad); return m.Solid(); });
                            break;
                        default:
                            Run(aDI, $"psphere r {R(a)} {R(a1)} {R(a2)} {R(anAngle)}",
                                () => { using var m = new N.BRepPrimAPI_MakeSphere(a, a1 * DegToRad, a2 * DegToRad, anAngle * DegToRad); return m.Solid(); });
                            break;
                    }
                    break;
                }
                case 14:
                {
                    if (!Plane(aDI, anO, out N.gp_Ax2? aNAx))
                    {
                        break;
                    }

                    double a1 = anO.Real(-90, 89), a2 = anO.Real(a1 + 0.5, 90), anAngle = anO.Real(1, 359);
                    switch (anO.Int(0, 3))
                    {
                        case 0:
                            Run(aDI, $"psphere r pl {R(a)}", () => { using var m = new N.BRepPrimAPI_MakeSphere(aNAx!, a); return m.Solid(); });
                            break;
                        case 1:
                            Run(aDI, $"psphere r pl {R(a)} {R(anAngle)}",
                                () => { using var m = new N.BRepPrimAPI_MakeSphere(aNAx!, a, anAngle * DegToRad); return m.Solid(); });
                            break;
                        case 2:
                            Run(aDI, $"psphere r pl {R(a)} {R(a1)} {R(a2)}",
                                () => { using var m = new N.BRepPrimAPI_MakeSphere(aNAx!, a, a1 * DegToRad, a2 * DegToRad); return m.Solid(); });
                            break;
                        default:
                            Run(aDI, $"psphere r pl {R(a)} {R(a1)} {R(a2)} {R(anAngle)}",
                                () => { using var m = new N.BRepPrimAPI_MakeSphere(aNAx!, a, a1 * DegToRad, a2 * DegToRad, anAngle * DegToRad); return m.Solid(); });
                            break;
                    }
                    aNAx!.Dispose();
                    break;
                }
                case 15:
                {
                    double r1 = a + b, r2 = Math.Min(a, b) * 0.9;
                    double a1 = anO.Real(-80, 0), a2 = anO.Real(10, 80), anAngle = anO.Real(1, 359);
                    switch (anO.Int(0, 3))
                    {
                        case 0:
                            Run(aDI, $"ptorus r {R(r1)} {R(r2)}", () => { using var m = new N.BRepPrimAPI_MakeTorus(r1, r2); return m.Solid(); });
                            break;
                        case 1:
                            Run(aDI, $"ptorus r {R(r1)} {R(r2)} {R(anAngle)}",
                                () => { using var m = new N.BRepPrimAPI_MakeTorus(r1, r2, anAngle * DegToRad); return m.Solid(); });
                            break;
                        case 2:
                            Run(aDI, $"ptorus r {R(r1)} {R(r2)} {R(a1)} {R(a2)}",
                                () => { using var m = new N.BRepPrimAPI_MakeTorus(r1, r2, a1 * DegToRad, a2 * DegToRad); return m.Solid(); });
                            break;
                        default:
                            Run(aDI, $"ptorus r {R(r1)} {R(r2)} {R(a1)} {R(a2)} {R(anAngle)}",
                                () => { using var m = new N.BRepPrimAPI_MakeTorus(r1, r2, a1 * DegToRad, a2 * DegToRad, anAngle * DegToRad); return m.Solid(); });
                            break;
                    }
                    break;
                }
                case 16:
                {
                    if (!Plane(aDI, anO, out N.gp_Ax2? aNAx))
                    {
                        break;
                    }

                    double r1 = a + b, r2 = Math.Min(a, b) * 0.9;
                    double a1 = anO.Real(-80, 0), a2 = anO.Real(10, 80), anAngle = anO.Real(1, 359);
                    switch (anO.Int(0, 3))
                    {
                        case 0:
                            Run(aDI, $"ptorus r pl {R(r1)} {R(r2)}", () => { using var m = new N.BRepPrimAPI_MakeTorus(aNAx!, r1, r2); return m.Solid(); });
                            break;
                        case 1:
                            Run(aDI, $"ptorus r pl {R(r1)} {R(r2)} {R(anAngle)}",
                                () => { using var m = new N.BRepPrimAPI_MakeTorus(aNAx!, r1, r2, anAngle * DegToRad); return m.Solid(); });
                            break;
                        case 2:
                            Run(aDI, $"ptorus r pl {R(r1)} {R(r2)} {R(a1)} {R(a2)}",
                                () => { using var m = new N.BRepPrimAPI_MakeTorus(aNAx!, r1, r2, a1 * DegToRad, a2 * DegToRad); return m.Solid(); });
                            break;
                        default:
                            Run(aDI, $"ptorus r pl {R(r1)} {R(r2)} {R(a1)} {R(a2)} {R(anAngle)}",
                                () => { using var m = new N.BRepPrimAPI_MakeTorus(aNAx!, r1, r2, a1 * DegToRad, a2 * DegToRad, anAngle * DegToRad); return m.Solid(); });
                            break;
                    }
                    aNAx!.Dispose();
                    break;
                }
                case 17:
                case 18:
                {
                    // halfspace of a face or of the shell of a box
                    using N.TopoDS_Shape aNBox = BindBox(aDI, anOrigin, a, b, c);
                    Point3D aRef = anO.Pnt();
                    if (aKind == 17)
                    {
                        int k = anO.Int(1, 6);
                        Run(aDI, $"halfspace r b_{k} {R(aRef)}",
                            () =>
                            {
                                using N.TopoDS_Shape aF = Nth(aNBox, N.TopAbs_ShapeEnum.TopAbs_FACE, k);
                                using N.TopoDS_Face aFace = N.TopoDS.Face(aF);
                                using var m = new N.BRepPrimAPI_MakeHalfSpace(aFace, P(aRef));
                                return m.IsDone() ? m.Solid() : new N.TopoDS_Shape();
                            });
                    }
                    else
                    {
                        aDI.Eval("explode b sh");
                        Run(aDI, $"halfspace r b_1 {R(aRef)}",
                            () =>
                            {
                                using N.TopoDS_Shape aS = Nth(aNBox, N.TopAbs_ShapeEnum.TopAbs_SHELL, 1);
                                using N.TopoDS_Shell aShell = N.TopoDS.Shell(aS);
                                using var m = new N.BRepPrimAPI_MakeHalfSpace(aShell, P(aRef));
                                return m.IsDone() ? m.Solid() : new N.TopoDS_Shape();
                            });
                    }
                    break;
                }
                case 19:
                case 20:
                {
                    // prism of a face, an edge or a vertex of a box
                    using N.TopoDS_Shape aNBox = BindBox(aDI, anOrigin, a, b, c);
                    (string aSub, N.TopAbs_ShapeEnum aType, int aCount) = anO.Int(0, 2) switch
                    {
                        0 => ("f", N.TopAbs_ShapeEnum.TopAbs_FACE, 6),
                        1 => ("e", N.TopAbs_ShapeEnum.TopAbs_EDGE, 12),
                        _ => ("v", N.TopAbs_ShapeEnum.TopAbs_VERTEX, 8),
                    };
                    aDI.Eval($"explode b {aSub}");
                    int k = anO.Int(1, aCount);
                    double vx = anO.Int(-5, 5), vy = anO.Int(-5, 5), vz = anO.Int(1, 5);
                    int anOption = aKind == 19 ? anO.Int(0, 1) : anO.Int(2, 3);
                    string aWord = anOption switch { 0 => "", 1 => " Copy", 2 => " Inf", _ => " SemiInf" };
                    Run(aDI, $"prism r b_{k} {R(vx)} {R(vy)} {R(vz)}{aWord}",
                        () =>
                        {
                            using N.TopoDS_Shape aS = Nth(aNBox, aType, k);
                            using var aV = new N.gp_Vec(vx, vy, vz);
                            if (anOption >= 2)
                            {
                                using var aD = new N.gp_Dir(aV);
                                using var m = new N.BRepPrimAPI_MakePrism(aS, aD, anOption == 2);
                                return m.Shape();
                            }

                            using var m2 = new N.BRepPrimAPI_MakePrism(aS, aV, anOption == 1);
                            return m2.Shape();
                        });
                    break;
                }
                default:
                {
                    if (aKind == 23)
                    {
                        // bounding -shape of a box: the AABB as a solid
                        using N.TopoDS_Shape aNBox = BindBox(aDI, anOrigin, a, b, c);
                        Run(aDI, "bounding b -shape r", () => NativeBoundingShape(aNBox));
                        break;
                    }

                    // revol of a face or an edge of a box around a random axis (the axis on the box's corner half the time)
                    using N.TopoDS_Shape aNBoxR = BindBox(aDI, anOrigin, a, b, c);
                    bool isFace = anO.Bool();
                    aDI.Eval($"explode b {(isFace ? "f" : "e")}");
                    int kr = anO.Int(1, isFace ? 6 : 12);
                    Point3D anAxisPnt = anO.Bool() ? anOrigin : anO.Pnt();
                    Direction3D anAxisDir = anO.Int(0, 2) switch { 0 => new Direction3D(0, 0, 1), 1 => new Direction3D(1, 0, 0), _ => Dir(anO) };
                    double anAngle = anO.Real(1, 360);
                    bool isCopy = anO.Bool();
                    Run(aDI, $"revol r b_{kr} {R(anAxisPnt)} {R(anAxisDir)} {R(anAngle)}{(isCopy ? " copy" : "")}",
                        () =>
                        {
                            using N.TopoDS_Shape aS = Nth(aNBoxR, isFace ? N.TopAbs_ShapeEnum.TopAbs_FACE : N.TopAbs_ShapeEnum.TopAbs_EDGE, kr);
                            using N.gp_Pnt aP = P(anAxisPnt);
                            using N.gp_Dir aD = Oracle.ToNative(anAxisDir);
                            using var anAx = new N.gp_Ax1(aP, aD);
                            using var m = new N.BRepPrimAPI_MakeRevol(aS, anAx, anAngle * DegToRad, isCopy);
                            return m.IsDone() ? m.Shape() : new N.TopoDS_Shape();
                        });
                    break;
                }
            }
        }

        output.WriteLine($"{myFiles} shapes, {myIdentical} byte-identical, {myMasked} equal with the Free column masked; "
                         + $"{myErrorsEqual} failures and {myNullBoth} null results on both sides; {mySkipped} placements drawn again");
        output.WriteLine("by command: " + string.Join(", ", myByCommand.OrderBy(p => p.Key, StringComparer.Ordinal).Select(p => $"{p.Key} {p.Value}")));
        foreach (string aD in myDifferences)
        {
            output.WriteLine(aD);
        }
        Assert.Empty(myDifferences);
        Assert.True(myFiles > 0); // as recorded, a budgeted sample (decision 1337)
    }

    // A plane for the [plane(ax2)] forms, bound to pl; its native gp_Ax2 (Pln().Position().Ax2(), as Draw reads it),
    // or false when the two sides' placements differ.
    private bool Plane(DrawInterpreter theDI, Oracle theO, out N.gp_Ax2? theNative)
    {
        Direction3D aD = Dir(theO), aX = Dir(theO);
        CoordinateSystem anAx3;
        try
        {
            anAx3 = new CoordinateSystem(theO.Pnt(), aD, aX);
        }
        catch (FailureException)
        {
            theNative = null;
            mySkipped++;
            return false;
        }

        if (theO.Bool())
        {
            anAx3 = anAx3.ZReversed();
        }

        var aPlane = new Geom_Plane(anAx3);
        theDI.SetDrawable("pl", DrawGeometry.Of(aPlane));
        using N.gp_Ax3 aNAx3 = Oracle.ToNative(anAx3);
        using var aNPlane = new N.Geom_Plane(aNAx3);
        using N.gp_Pln aNPln = aNPlane.Pln();
        using N.gp_Ax3 aNPos = aNPln.Position();
        theNative = aNPos.Ax2();
        if (!Same(theNative, aPlane.Pln().Position().Ax2()))
        {
            theNative.Dispose();
            theNative = null;
            mySkipped++;
            return false;
        }

        return true;
    }

    // box b at the origin with the sizes, its faces exploded as b_1 ... b_6; the native box
    private static N.TopoDS_Shape BindBox(DrawInterpreter theDI, in Point3D theOrigin, double a, double b, double c)
    {
        myLastBox = $"box b {R(theOrigin)} {R(a)} {R(b)} {R(c)}";
        theDI.Eval(myLastBox);
        theDI.Eval("explode b f");
        using N.gp_Pnt aP = P(theOrigin);
        using var m = new N.BRepPrimAPI_MakeBox(aP, a, b, c);
        return m.Solid();
    }

    // The k-th distinct sub-shape of the type, in TopExp_Explorer's order: explode's (it skips repeated ones)
    private static N.TopoDS_Shape Nth(N.TopoDS_Shape theShape, N.TopAbs_ShapeEnum theType, int k)
    {
        var aFound = new List<N.TopoDS_Shape>();
        using var anExp = new N.TopExp_Explorer(theShape, theType);
        for (; anExp.More(); anExp.Next())
        {
            N.TopoDS_Shape aS = anExp.Current();
            if (aFound.Any(f => f.IsSame(aS)))
            {
                aS.Dispose();
                continue;
            }

            aFound.Add(aS);
            if (aFound.Count == k)
            {
                break;
            }
        }

        for (int i = 0; i < aFound.Count - 1; i++)
        {
            aFound[i].Dispose();
        }

        return aFound[k - 1];
    }

    private static N.TopoDS_Shape NativePreview(in Point3D theOrigin, double dx, double dy, double dz)
    {
        // BRepPreviewAPI_MakeBox isn't bound: its Build for each thin case, with the makers it calls
        Point3D aMin = new(theOrigin.X() + Math.Min(dx, 0), theOrigin.Y() + Math.Min(dy, 0), theOrigin.Z() + Math.Min(dz, 0));
        double ax = Math.Abs(dx), ay = Math.Abs(dy), az = Math.Abs(dz);
        var aF = new Point3D(aMin.X(), aMin.Y(), aMin.Z());
        var aS = new Point3D(aMin.X() + ax, aMin.Y() + ay, aMin.Z() + az);
        bool tx = Math.Abs(aF.X() - aS.X()) < 1e-7, ty = Math.Abs(aF.Y() - aS.Y()) < 1e-7, tz = Math.Abs(aF.Z() - aS.Z()) < 1e-7;
        int aType = (tx ? 1 : 0) + (ty ? 1 : 0) + (tz ? 1 : 0);
        if (aType == 3)
        {
            using var m = new N.BRepBuilderAPI_MakeVertex(P(aF));
            return m.Shape();
        }

        if (aType == 2)
        {
            using var m = new N.BRepBuilderAPI_MakeEdge(P(aF), P(aS));
            return m.Shape();
        }

        if (aType == 1)
        {
            Point3D p1, p2, p3, p4;
            if (tx)
            {
                p1 = new(aF.X(), aF.Y(), aF.Z()); p2 = new(aF.X(), aS.Y(), aF.Z()); p3 = new(aF.X(), aS.Y(), aS.Z()); p4 = new(aF.X(), aF.Y(), aS.Z());
            }
            else if (ty)
            {
                p1 = new(aF.X(), aF.Y(), aF.Z()); p2 = new(aS.X(), aF.Y(), aF.Z()); p3 = new(aS.X(), aF.Y(), aS.Z()); p4 = new(aF.X(), aF.Y(), aS.Z());
            }
            else
            {
                p1 = new(aF.X(), aF.Y(), aF.Z()); p2 = new(aS.X(), aF.Y(), aF.Z()); p3 = new(aS.X(), aS.Y(), aF.Z()); p4 = new(aF.X(), aS.Y(), aF.Z());
            }

            using var e1 = new N.BRepBuilderAPI_MakeEdge(P(p1), P(p2));
            using var e2 = new N.BRepBuilderAPI_MakeEdge(P(p2), P(p3));
            using var e3 = new N.BRepBuilderAPI_MakeEdge(P(p3), P(p4));
            using var e4 = new N.BRepBuilderAPI_MakeEdge(P(p4), P(p1));
            using var w = new N.BRepBuilderAPI_MakeWire(e1.Edge(), e2.Edge(), e3.Edge(), e4.Edge());
            using var f = new N.BRepBuilderAPI_MakeFace(w.Wire());
            return f.Shape();
        }

        using var mb = new N.BRepPrimAPI_MakeBox(P(theOrigin), dx, dy, dz);
        return mb.Solid();
    }

    private static N.TopoDS_Shape NativeBoundingShape(N.TopoDS_Shape theShape)
    {
        // bounding's default: BRepBndLib::Add with triangulation, then ConvertBndToShape of the OBB of the AABB
        using var aBox = new N.Bnd_Box();
        N.BRepBndLib.Add(theShape, aBox, true);
        using var anOBB = new N.Bnd_OBB(aBox);
        using N.gp_XYZ aC = anOBB.Center();
        using N.gp_XYZ aX = anOBB.XDirection();
        using N.gp_XYZ aY = anOBB.YDirection();
        using N.gp_XYZ aZ = anOBB.ZDirection();
        double hx = anOBB.XHSize(), hy = anOBB.YHSize(), hz = anOBB.ZHSize();
        using var aCP = new N.gp_Pnt(aC.X(), aC.Y(), aC.Z());
        using var aZD = new N.gp_Dir(aZ.X(), aZ.Y(), aZ.Z());
        using var aXD = new N.gp_Dir(aX.X(), aX.Y(), aX.Z());
        using var anAxes = new N.gp_Ax2(aCP, aZD, aXD);
        using var aLoc = new N.gp_Pnt(aC.X() - hx * aX.X() - hy * aY.X() - hz * aZ.X(),
                                      aC.Y() - hx * aX.Y() - hy * aY.Y() - hz * aZ.Y(),
                                      aC.Z() - hx * aX.Z() - hy * aY.Z() - hz * aZ.Z());
        anAxes.SetLocation(aLoc);
        using var m = new N.BRepPrimAPI_MakeBox(anAxes, 2.0 * hx, 2.0 * hy, 2.0 * hz);
        return m.Solid();
    }
}
