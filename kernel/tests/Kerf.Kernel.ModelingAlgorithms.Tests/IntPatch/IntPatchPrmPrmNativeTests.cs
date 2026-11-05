// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Polygons;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using System.Globalization;
using System.Reflection;
using System.Text;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// IntPatch_PrmPrmIntersection step by step against native OCCT: tools/walkrec compiles OCCT's own
/// IntPatch_PrmPrmIntersection.cxx with the instrumented IntWalk_PWalking.cxx (decision 687) against Kerf.Modeling's
/// OCCT V8_0_1 and records every walking step (status, point, parameters), the polyhedra, their interference (section
/// lines and tangent zones, point by point), the voxel search's start points and every line (transitions, points,
/// vertices), every double as its 64 bits. The cases cover each Perform: on the domains' samplings through
/// IntPolyh (two B-spline patches, an offset surface and a patch), on polyhedra (IntPatch_InterferencePolyhedron),
/// from a start point, from a list of points on a closed surface then the sampled Perform keeping its lines, the voxel
/// search (two B-splines of 30 x 30 poles, over 2,500 samples each), and the self-intersection of a folded surface.
/// </summary>
public class IntPatchPrmPrmNativeTests
{
    private const double THE_TOL = 1.0e-7, THE_FLECHE = 0.01, THE_PAS = 0.05;

    // tools/walkrec's main.cpp, line for line.
    internal static BSplineSurface Patch(double theX0, double theX1, double theY0, double theY1, int theTable = 0)
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

    internal static BSplineSurface Bumpy(int theSeed, double theZ0)
    {
        const int aN = 30;
        var aPoles = new NCollection_Array2<Point3D>(1, aN, 1, aN);
        for (int i = 1; i <= aN; i++)
        {
            for (int j = 1; j <= aN; j++)
            {
                int aR = (i * 7919 + j * 104729 + theSeed * 31) % 97;
                aPoles[i, j] = new Point3D(-2.0 + 4.0 * (i - 1) / (aN - 1), -2.0 + 4.0 * (j - 1) / (aN - 1),
                                          theZ0 + 0.4 * aR / 97.0 - 0.2);
            }
        }
        var aKnots = new NCollection_Array1<double>(1, aN - 2);
        var aMults = new NCollection_Array1<int>(1, aN - 2);
        for (int k = 1; k <= aN - 2; k++)
        {
            aKnots[k] = (k - 1) / (double)(aN - 3);
            aMults[k] = (k == 1 || k == aN - 2) ? 4 : 1;
        }
        return new BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
    }

    internal static BSplineSurface Folded()
    {
        double[] aX = [-1.0, 2.0, 2.0, 0.0, 0.0, 1.5];
        double[] aZ = [0.0, 0.0, 1.2, 1.2, -0.6, -0.6];
        var aPoles = new NCollection_Array2<Point3D>(1, 6, 1, 4);
        for (int i = 1; i <= 6; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aPoles[i, j] = new Point3D(aX[i - 1], -1.0 + 2.0 * (j - 1) / 3.0, aZ[i - 1] + 0.05 * (j - 1));
            }
        }
        var aUKnots = new NCollection_Array1<double>(1, 3);
        var aVKnots = new NCollection_Array1<double>(1, 2);
        var aUMults = new NCollection_Array1<int>(1, 3);
        var aVMults = new NCollection_Array1<int>(1, 2);
        aUKnots[1] = 0.0;
        aUKnots[2] = 0.5;
        aUKnots[3] = 1.0;
        aUMults[1] = 4;
        aUMults[2] = 2;
        aUMults[3] = 4;
        aVKnots[1] = 0.0;
        aVKnots[2] = 1.0;
        aVMults[1] = 4;
        aVMults[2] = 4;
        return new BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 3, 3);
    }

    internal static BSplineSurface PatchA() => Patch(-2.0, 2.0, -2.0, 2.0);

    internal static BSplineSurface PatchB() => Patch(-2.0, 2.0, -2.0, 2.0, 1);

    internal static CylindricalSurface Cylinder() =>
        new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0);

    private static void Hex(StringBuilder theOut, double theValue) =>
        theOut.Append(' ').Append(BitConverter.DoubleToUInt64Bits(theValue).ToString("x16", CultureInfo.InvariantCulture));

    private static void RecPnt(StringBuilder theOut, string theTag, int theStatus, in PntOn2S theP)
    {
        theOut.Append(theTag).Append(' ').Append(theStatus.ToString(CultureInfo.InvariantCulture));
        Hex(theOut, theP.Value().X);
        Hex(theOut, theP.Value().Y);
        Hex(theOut, theP.Value().Z);
        theP.Parameters(out double u1, out double v1, out double u2, out double v2);
        Hex(theOut, u1);
        Hex(theOut, v1);
        Hex(theOut, u2);
        Hex(theOut, v2);
        theOut.Append('\n');
    }

    private static void DumpPrmPrm(StringBuilder theOut, PrmPrmIntersection theInt)
    {
        theOut.Append("DONE ").Append(theInt.IsDone ? 1 : 0).Append('\n');
        if (!theInt.IsDone)
        {
            return;
        }
        theOut.Append(CultureInfo.InvariantCulture, $"EMPTY {(theInt.IsEmpty ? 1 : 0)} NBLINES {theInt.LineCount}\n");
        for (int i = 1; i <= theInt.LineCount; i++)
        {
            PatchLine aLine = theInt.Line(i);
            theOut.Append(CultureInfo.InvariantCulture,
                          $"LINE {i} TYPE {(int)aLine.ArcType()} TRANS {(int)aLine.TransitionOnS1()} {(int)aLine.TransitionOnS2()}\n");
            var aWL = (WLine)aLine;
            theOut.Append(CultureInfo.InvariantCulture, $"NBP {aWL.NbPnts()} NBV {aWL.VertexCount}\n");
            for (int k = 1; k <= aWL.NbPnts(); k++)
            {
                RecPnt(theOut, "P", 0, aWL.Point(k));
            }
            for (int k = 1; k <= aWL.VertexCount; k++)
            {
                PatchPoint aV = aWL.Vertex(k);
                theOut.Append("V ").Append(aV.IsOnDomS1 ? 1 : 0).Append(' ').Append(aV.IsOnDomS2 ? 1 : 0);
                Hex(theOut, aV.ParameterOnLine());
                RecPnt(theOut, "", 0, aV.PntOn2S());
            }
        }
    }

    private static void DumpSectionPoint(StringBuilder theOut, string theTag, in SectionPoint theP)
    {
        theP.InfoFirst(out PIType aT1, out int a1, out int a2, out double p1);
        theP.InfoSecond(out PIType aT2, out int a3, out int a4, out double p2);
        theOut.Append(CultureInfo.InvariantCulture, $"{theTag} {(int)aT1} {a1} {a2} {(int)aT2} {a3} {a4}");
        Hex(theOut, p1);
        Hex(theOut, p2);
        Hex(theOut, theP.Incidence());
        Hex(theOut, theP.Pnt().X);
        Hex(theOut, theP.Pnt().Y);
        Hex(theOut, theP.Pnt().Z);
        theOut.Append('\n');
    }

    private static void DumpPolyhedron(StringBuilder theOut, Polyhedron thePoly)
    {
        thePoly.Size(out int aNbU, out int aNbV);
        theOut.Append(CultureInfo.InvariantCulture, $"POLY {aNbU} {aNbV} {thePoly.TriangleCount}");
        Hex(theOut, thePoly.DeflectionOverEstimation());
        theOut.Append('\n');
    }

    private static void DumpInterference(StringBuilder theOut, InterferencePolyhedron theIntf)
    {
        theOut.Append(CultureInfo.InvariantCulture,
                      $"INTF {theIntf.SectionPointCount} {theIntf.SectionLineCount} {theIntf.TangentZoneCount}");
        Hex(theOut, theIntf.GetTolerance());
        theOut.Append('\n');
        for (int i = 1; i <= theIntf.SectionLineCount; i++)
        {
            SectionLine aL = theIntf.LineValue(i);
            theOut.Append(CultureInfo.InvariantCulture, $"SL {i} {aL.NumberOfPoints()}\n");
            for (int k = 1; k <= aL.NumberOfPoints(); k++)
            {
                DumpSectionPoint(theOut, "SP", aL.GetPoint(k));
            }
        }
        for (int i = 1; i <= theIntf.TangentZoneCount; i++)
        {
            TangentZone aZ = theIntf.ZoneValue(i);
            theOut.Append(CultureInfo.InvariantCulture, $"TZ {i} {aZ.NumberOfPoints()}\n");
            for (int k = 1; k <= aZ.NumberOfPoints(); k++)
            {
                DumpSectionPoint(theOut, "TP", aZ.GetPoint(k));
            }
        }
    }

    private static (SurfaceAdaptor, TopolTool) Domain(Surface theS)
    {
        var aS = new SurfaceAdaptor(theS);
        return (aS, new TopolTool(aS));
    }

    private static void PrmPrm(StringBuilder theOut, Surface theS1, Surface theS2)
    {
        var (aS1, aD1) = Domain(theS1);
        var (aS2, aD2) = Domain(theS2);
        var anInt = new PrmPrmIntersection();
        anInt.Perform(aS1, aD1, aS2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
        DumpPrmPrm(theOut, anInt);
    }

    private static void PrmPrmPoly(StringBuilder theOut, Surface theS1, Surface theS2)
    {
        var (aS1, aD1) = Domain(theS1);
        var (aS2, aD2) = Domain(theS2);
        var aPoly1 = new Polyhedron(aS1);
        var aPoly2 = new Polyhedron(aS2);
        DumpPolyhedron(theOut, aPoly1);
        DumpPolyhedron(theOut, aPoly2);
        DumpInterference(theOut, new InterferencePolyhedron(aPoly1, aPoly2));
        var anInt = new PrmPrmIntersection();
        anInt.Perform(aS1, aPoly1, aD1, aS2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
        DumpPrmPrm(theOut, anInt);
    }

    private static void PrmPrmSelf(StringBuilder theOut, Surface theS)
    {
        var (aS, aD) = Domain(theS);
        var aPoly = new Polyhedron(aS, aD.NbSamplesU(), aD.NbSamplesV());
        DumpPolyhedron(theOut, aPoly);
        DumpInterference(theOut, new InterferencePolyhedron(aPoly));
        var anInt = new PrmPrmIntersection();
        anInt.Perform(aS, aD, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
        DumpPrmPrm(theOut, anInt);
    }

    private static void PrmPrmVoxels(StringBuilder theOut, Surface theS1, Surface theS2)
    {
        var (aS1, aD1) = Domain(theS1);
        var (aS2, aD2) = Domain(theS2);
        aD1.SamplePnts(THE_FLECHE, 10, 10);
        aD2.SamplePnts(THE_FLECHE, 10, 10);
        theOut.Append(CultureInfo.InvariantCulture,
                      $"SAMPLES {aD1.NbSamplesU()} {aD1.NbSamplesV()} {aD2.NbSamplesU()} {aD2.NbSamplesV()}\n");
        var anInt = new PrmPrmIntersection();
        var aLOn2S = new LineOn2S();
        anInt.PointDepart(aLOn2S, aS1, aD1.NbSamplesU(), aD1.NbSamplesV(), aS2, aD2.NbSamplesU(), aD2.NbSamplesV());
        theOut.Append(CultureInfo.InvariantCulture, $"DEPART {aLOn2S.PointCount}\n");
        for (int k = 1; k <= aLOn2S.PointCount; k++)
        {
            RecPnt(theOut, "D", 0, aLOn2S.Value(k));
        }
        anInt.Perform(aS1, aD1, aS2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
        DumpPrmPrm(theOut, anInt);
    }

    private static void PrmPrmStart(StringBuilder theOut, Surface theS1, Surface theS2, double theU1,
                                    double theV1, double theU2, double theV2)
    {
        var (aS1, aD1) = Domain(theS1);
        var (aS2, aD2) = Domain(theS2);
        var anInt = new PrmPrmIntersection();
        anInt.Perform(aS1, aD1, aS2, aD2, theU1, theV1, theU2, theV2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS);
        DumpPrmPrm(theOut, anInt);
    }

    private static void PrmPrmList(StringBuilder theOut, Surface theS1, Surface theS2)
    {
        var (aS1, aD1) = Domain(theS1);
        var (aS2, aD2) = Domain(theS2);
        var aList = new NCollection_List<PntOn2S>();
        PntOn2S aP = new();
        aP.SetValue(0.0, 0.0, 1.0 / 3.0, 0.5);
        aList.Append(aP);
        aP.SetValue(0.3, 0.1, 0.4, 0.55);
        aList.Append(aP);
        var anInt = new PrmPrmIntersection();
        anInt.Perform(aS1, aD1, aS2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS, aList);
        theOut.Append(CultureInfo.InvariantCulture, $"LIST {aList.Size()}\n");
        for (var anIt = new NCollection_List<PntOn2S>.Iterator(aList); anIt.More(); anIt.Next())
        {
            RecPnt(theOut, "L", 0, anIt.Value());
        }
        DumpPrmPrm(theOut, anInt);
        // Then the sampled Perform keeping these lines, as IntPatch_Intersection does with a list of points.
        anInt.Perform(aS1, aD1, aS2, aD2, THE_TOL, THE_TOL, THE_FLECHE, THE_PAS, false);
        DumpPrmPrm(theOut, anInt);
    }

    internal static string Run(string theCase)
    {
        var aOut = new StringBuilder();
        PWalking.s_threadStepTrace = (theStatus, theP) => RecPnt(aOut, "S", (int)theStatus, theP);
        try
        {
            switch (theCase)
            {
                case "prmprm-bspline-bspline":
                    PrmPrm(aOut, PatchA(), PatchB());
                    break;
                case "prmprm-offset-bspline":
                    PrmPrm(aOut, new OffsetSurface(PatchA(), 0.15), PatchB());
                    break;
                case "prmprm-poly":
                    PrmPrmPoly(aOut, PatchA(), PatchB());
                    break;
                case "prmprm-start":
                    PrmPrmStart(aOut, PatchA(), PatchB(), 0.5, 0.5, 0.5, 0.5);
                    break;
                case "prmprm-voxels":
                    PrmPrmVoxels(aOut, Bumpy(1, 0.0), Bumpy(2, 0.12));
                    break;
                case "prmprm-listpnts":
                    PrmPrmList(aOut, new TrimmedSurface(Cylinder(), 0.0, 2.0 * M_PI, -2.0, 2.0),
                               Patch(0.0, 3.0, -2.0, 2.0));
                    break;
                case "prmprm-self":
                    PrmPrmSelf(aOut, Folded());
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(theCase));
            }
        }
        finally
        {
            PWalking.s_threadStepTrace = null;
        }
        return aOut.ToString();
    }

    private static string Recording(string theCase)
    {
        using Stream aStream = Assembly.GetExecutingAssembly().GetManifestResourceStream($"IntPatch.Recordings.{theCase}.txt")
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
    [InlineData("prmprm-bspline-bspline")]
    [InlineData("prmprm-offset-bspline")]
    [InlineData("prmprm-poly")]
    [InlineData("prmprm-start")]
    [InlineData("prmprm-voxels")]
    [InlineData("prmprm-listpnts")]
    [InlineData("prmprm-self")]
    public void Steps_AsNative(string theCase)
    {
        string anExpected = Recording(theCase);
        Assert.Contains("\nS ", "\n" + anExpected, StringComparison.Ordinal);
        AssertSameSteps(anExpected, Run(theCase));
    }

    [Theory]
    [InlineData("prmprm-bspline-bspline")]
    [InlineData("prmprm-poly")]
    [InlineData("prmprm-voxels")]
    [InlineData("prmprm-self")]
    public void Steps_SameEveryRun_SerialAndParallel(string theCase)
    {
        string aRef = Run(theCase);
        for (int i = 0; i < 2; i++)
        {
            Assert.Equal(aRef, Run(theCase));
        }
        string[] aParallel = new string[6];
        Parallel.For(0, aParallel.Length, i => aParallel[i] = Run(theCase));
        Assert.All(aParallel, s => Assert.Equal(aRef, s));
    }
}
