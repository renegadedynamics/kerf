// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using System.Globalization;
using System.Reflection;
using System.Text;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The walking step by step against native OCCT (decision 687): every return of TestDeflection (its status, the point
/// and its parameters) and the result, every double bit for bit, as tools/walkrec records them from OCCT's own
/// IntWalk_IWalking.gxx, IntPatch_ImpPrmIntersection.cxx and IntWalk_PWalking.cxx built with MSVC against
/// build123d.NET's OCCT V8_0_1. Through IntPatch_ImpPrmIntersection (IntWalk_IWalking): a cylinder against a B-spline
/// patch (closed and open lines), a plane against a torus, a sphere, a cone and a tilted plane against patches. Through
/// IntWalk_PWalking from a start point: a cylinder, a torus and another patch against a patch. Each gives the same
/// line serially on every run and from parallel threads (decision 490).
/// </summary>
public class IntWalkNativeStepsTests
{
    // tools/walkrec's main.cpp, line for line.
    private static BSplineSurface Patch(double theX0, double theX1, double theY0, double theY1, int theTable = 0)
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 4, 1, 4);
        double[,] aZ =
        {
            { 0.10, -0.20, 0.30, 0.00 },
            { -0.30, 0.40, -0.10, 0.20 },
            { 0.20, -0.10, 0.50, -0.30 },
            { 0.00, 0.30, -0.20, 0.10 },
        };
        double[,] aZB =
        {
            { 0.40, 0.10, -0.20, 0.30 },
            { -0.10, -0.30, 0.20, 0.00 },
            { 0.30, 0.20, -0.40, 0.10 },
            { -0.20, 0.00, 0.30, -0.10 },
        };
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aPoles[i, j] = new Point3D(theX0 + (theX1 - theX0) * (i - 1) / 3.0, theY0 + (theY1 - theY0) * (j - 1) / 3.0,
                                          theTable == 0 ? aZ[i - 1, j - 1] : aZB[i - 1, j - 1]);
            }
        }
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        return new BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
    }

    private static CylindricalSurface Cylinder() =>
        new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0);

    private static void Hex(StringBuilder theOut, double theValue) =>
        theOut.Append(' ').Append(BitConverter.DoubleToUInt64Bits(theValue).ToString("x16", CultureInfo.InvariantCulture));

    private static void RecPnt(StringBuilder theOut, string theTag, int theStatus, in PntOn2S theP)
    {
        theOut.Append(theTag).Append(' ').Append(theStatus.ToString(CultureInfo.InvariantCulture));
        Hex(theOut, theP.Value().X());
        Hex(theOut, theP.Value().Y());
        Hex(theOut, theP.Value().Z());
        theP.Parameters(out double u1, out double v1, out double u2, out double v2);
        Hex(theOut, u1);
        Hex(theOut, v1);
        Hex(theOut, u2);
        Hex(theOut, v2);
        theOut.Append('\n');
    }

    private static void DumpVertex(StringBuilder theOut, in PatchPoint theV)
    {
        theOut.Append("V ").Append(theV.IsOnDomS1 ? 1 : 0).Append(' ').Append(theV.IsOnDomS2 ? 1 : 0);
        Hex(theOut, theV.ParameterOnLine());
        RecPnt(theOut, "", 0, theV.PntOn2S());
    }

    private static void DumpResult(StringBuilder theOut, ImpPrmIntersection theInt)
    {
        theOut.Append("DONE ").Append(theInt.IsDone ? 1 : 0).Append('\n');
        if (!theInt.IsDone)
        {
            return;
        }
        theOut.Append(CultureInfo.InvariantCulture,
                      $"EMPTY {(theInt.IsEmpty ? 1 : 0)} NBPNTS {theInt.PntCount} NBLINES {theInt.LineCount}\n");
        for (int i = 1; i <= theInt.LineCount; i++)
        {
            PatchLine aLine = theInt.Line(i);
            theOut.Append(CultureInfo.InvariantCulture,
                          $"LINE {i} TYPE {(int)aLine.ArcType()} TRANS {(int)aLine.TransitionOnS1()} {(int)aLine.TransitionOnS2()}\n");
            if (aLine is WLine aWL)
            {
                theOut.Append(CultureInfo.InvariantCulture, $"NBP {aWL.NbPnts()} NBV {aWL.VertexCount}\n");
                for (int k = 1; k <= aWL.NbPnts(); k++)
                {
                    RecPnt(theOut, "P", 0, aWL.Point(k));
                }
                for (int k = 1; k <= aWL.VertexCount; k++)
                {
                    DumpVertex(theOut, aWL.Vertex(k));
                }
            }
            else if (aLine is RLine aRL)
            {
                theOut.Append(CultureInfo.InvariantCulture,
                              $"RLINE NBP {(aRL.HasPolygon ? aRL.NbPnts() : 0)} NBV {aRL.VertexCount}\n");
                if (aRL.HasPolygon)
                {
                    for (int k = 1; k <= aRL.NbPnts(); k++)
                    {
                        RecPnt(theOut, "P", 0, aRL.Point(k));
                    }
                }
                for (int k = 1; k <= aRL.VertexCount; k++)
                {
                    DumpVertex(theOut, aRL.Vertex(k));
                }
            }
        }
    }

    private static string ImpPrm(Surface theS1, Surface theS2, double theFleche, double thePas)
    {
        var aOut = new StringBuilder();
        var aS1 = new SurfaceAdaptor(theS1);
        var aS2 = new SurfaceAdaptor(theS2);
        var aD1 = new TopolTool(aS1);
        var aD2 = new TopolTool(aS2);
        PatchTheIWalking.s_threadStepTrace = (theStatus, theP) => RecPnt(aOut, "S", (int)theStatus, theP);
        try
        {
            var anInt = new ImpPrmIntersection(aS1, aD1, aS2, aD2, 1.0e-7, 1.0e-7, theFleche, thePas);
            DumpResult(aOut, anInt);
        }
        finally
        {
            PatchTheIWalking.s_threadStepTrace = null;
        }
        return aOut.ToString();
    }

    private static string PWalk(Surface theS1, Surface theS2, double theU1, double theV1, double theU2,
                                double theV2)
    {
        var aOut = new StringBuilder();
        var aS1 = new SurfaceAdaptor(theS1);
        var aS2 = new SurfaceAdaptor(theS2);
        var aPW = new PWalking(aS1, aS2, 1.0e-8, 1.0e-8, 0.01, 0.05);
        aPW._stepTrace = (theStatus, theP) => RecPnt(aOut, "S", (int)theStatus, theP);
        var aParDep = new NCollection_Array1<double>(1, 4);
        aParDep[1] = theU1;
        aParDep[2] = theV1;
        aParDep[3] = theU2;
        aParDep[4] = theV2;
        aPW.Perform(aParDep);
        aOut.Append("DONE ").Append(aPW.IsDone ? 1 : 0).Append('\n');
        if (aPW.IsDone)
        {
            aOut.Append(CultureInfo.InvariantCulture, $"NBP {aPW.PointCount} CLOSED {(aPW.IsClosed ? 1 : 0)}\n");
            for (int k = 1; k <= aPW.PointCount; k++)
            {
                RecPnt(aOut, "P", 0, aPW.Value(k));
            }
        }
        return aOut.ToString();
    }

    private static string Run(string theCase) => theCase switch
    {
        "cyl-bspline-closed" => ImpPrm(Cylinder(), Patch(-2.0, 2.0, -2.0, 2.0), 0.01, 0.05),
        "cyl-bspline-open" => ImpPrm(Cylinder(), Patch(0.0, 3.0, -2.0, 2.0), 0.01, 0.05),
        "plane-torus" => ImpPrm(new PlaneSurface(new Point3D(0, 0, 0.2), new Direction3D(0.1, 0.05, 1.0)),
                                new ToroidalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0, 0.5),
                                0.01, 0.05),
        "pwalk-cyl-bspline" => PWalk(new TrimmedSurface(Cylinder(), 0.0, 2.0 * M_PI, -2.0, 2.0),
                                     Patch(0.0, 3.0, -2.0, 2.0), 0.3, 0.1, 0.4, 0.55),
        "sphere-bspline" => ImpPrm(new SphericalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.2),
                                   Patch(-2.0, 2.0, -2.0, 2.0), 0.01, 0.05),
        "cone-bspline" => ImpPrm(new ConicalSurface(new CoordinateSystem(new Point3D(0.1, -0.2, -1.0), new Direction3D(0, 0, 1)),
                                                         M_PI / 6, 0.5),
                                 Patch(-2.0, 2.0, -2.0, 2.0), 0.01, 0.05),
        "bspline-plane" => ImpPrm(Patch(-2.0, 2.0, -2.0, 2.0, 1),
                                  new PlaneSurface(new Point3D(0, 0, 0.05), new Direction3D(0.2, -0.1, 1.0)), 0.01, 0.05),
        "pwalk-torus-bspline" => PWalk(new ToroidalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0, 0.5),
                                       Patch(-2.0, 2.0, -2.0, 2.0), 0.0, M_PI, 0.875, 0.5),
        "pwalk-bspline-bspline" => PWalk(Patch(-2.0, 2.0, -2.0, 2.0), Patch(-2.0, 2.0, -2.0, 2.0, 1), 0.5, 0.5, 0.5, 0.5),
        _ => throw new ArgumentOutOfRangeException(nameof(theCase)),
    };

    private static string Recording(string theCase)
    {
        using Stream aStream = Assembly.GetExecutingAssembly().GetManifestResourceStream($"IntWalk.Recordings.{theCase}.txt")
                               ?? throw new InvalidOperationException($"no recording {theCase}");
        using var aReader = new StreamReader(aStream);
        return aReader.ReadToEnd().Replace("\r\n", "\n", StringComparison.Ordinal);
    }

    /// <summary>The first line that differs, with its number, for a readable failure.</summary>
    private static void AssertSameSteps(string theExpected, string theActual)
    {
        string[] anExp = theExpected.Split('\n');
        string[] anAct = theActual.Split('\n');
        for (int i = 0; i < Math.Min(anExp.Length, anAct.Length); i++)
        {
            Assert.True(anExp[i] == anAct[i], $"line {i + 1} differs:\n native: {anExp[i]}\n port:   {anAct[i]}");
        }
        Assert.Equal(anExp.Length, anAct.Length);
    }

    [Theory]
    [InlineData("cyl-bspline-closed")]
    [InlineData("cyl-bspline-open")]
    [InlineData("plane-torus")]
    [InlineData("pwalk-cyl-bspline")]
    [InlineData("sphere-bspline")]
    [InlineData("cone-bspline")]
    [InlineData("bspline-plane")]
    [InlineData("pwalk-torus-bspline")]
    [InlineData("pwalk-bspline-bspline")]
    public void Steps_AsNative(string theCase)
    {
        string anExpected = Recording(theCase);
        Assert.Contains("\nS ", "\n" + anExpected, StringComparison.Ordinal);
        AssertSameSteps(anExpected, Run(theCase));
    }

    [Theory]
    [InlineData("cyl-bspline-closed")]
    [InlineData("plane-torus")]
    [InlineData("pwalk-cyl-bspline")]
    [InlineData("pwalk-bspline-bspline")]
    public void Steps_SameEveryRun_SerialAndParallel(string theCase)
    {
        string aRef = Run(theCase);
        for (int i = 0; i < 3; i++)
        {
            Assert.Equal(aRef, Run(theCase));
        }
        string[] aParallel = new string[8];
        Parallel.For(0, aParallel.Length, i => aParallel[i] = Run(theCase));
        Assert.All(aParallel, s => Assert.Equal(aRef, s));
    }
}
