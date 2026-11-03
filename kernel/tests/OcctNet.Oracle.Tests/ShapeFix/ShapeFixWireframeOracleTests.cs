// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Diagnostics;
using Xunit.Abstractions;
using static Occt.ShapeExtend_Status;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// ShapeFix_Wireframe against native OCCT: FixWireGaps and FixSmallEdges as build123d.NET's
/// <c>Wire.FixDegenerateEdges</c> calls them (precision, maximal tolerance 10^-6, FixSmallEdges then FixWireGaps) on
/// random sketch wires, and as the heal group's <c>fixwgaps</c> (10^-3) and <c>fixsmall</c> (1) commands run them on its
/// public inputs and data/occ (with 10^-2 and a limit angle besides).
/// </summary>
/// <remarks>
/// The statuses compared are the fixes' results and StatusWireGaps (DONE1, DONE2, FAIL1, FAIL2) and StatusSmallEdges
/// (DONE1, DONE2, DONE3, FAIL1). The port's input is frozen, as a caller's; see <see cref="ShapeFixOracleRun"/>.
/// </remarks>
public class ShapeFixWireframeOracleTests(ITestOutputHelper output)
{
    private enum Fix
    {
        Gaps,
        Small,
        SmallThenGaps,
    }

    private static N.ShapeExtend_Status NS(ShapeExtend_Status theStatus) => (N.ShapeExtend_Status)(int)theStatus;

    private static string Status(Func<ShapeExtend_Status, bool> theGaps, Func<ShapeExtend_Status, bool> theSmall)
        => $"gaps {theGaps(ShapeExtend_DONE1)}/{theGaps(ShapeExtend_DONE2)}/{theGaps(ShapeExtend_FAIL1)}/{theGaps(ShapeExtend_FAIL2)}, "
           + $"small {theSmall(ShapeExtend_DONE1)}/{theSmall(ShapeExtend_DONE2)}/{theSmall(ShapeExtend_DONE3)}/{theSmall(ShapeExtend_FAIL1)}";

    private static void Compare(ShapeFixOracleRun theRun, N.TopoDS_Shape theShape, string theWhat, Fix theFix, double thePrecision,
                                double theMaxTol = -1.0, double theLimitAngle = -1.0)
    {
        if (theRun.Read(theShape) is not (N.TopoDS_Shape aNS, TopoDS_Shape aPS))
        {
            return;
        }
        string anInputText = theRun.PortText(aPS);

        N.TopoDS_Shape? aNRes = null;
        string? aNErr = null;
        string aNStatus = "";
        try
        {
            using var aFix = new N.ShapeFix_Wireframe(aNS);
            aFix.SetPrecision(thePrecision);
            if (theMaxTol > 0.0)
            {
                aFix.SetMaxTolerance(theMaxTol);
            }
            if (theLimitAngle > 0.0)
            {
                aFix.SetLimitAngle(theLimitAngle);
            }
            bool aSmall = theFix != Fix.Gaps && aFix.FixSmallEdges();
            bool aGaps = theFix != Fix.Small && aFix.FixWireGaps();
            aNRes = aFix.Shape();
            aNStatus = $"{aSmall}/{aGaps}, " + Status(s => aFix.StatusWireGaps(NS(s)), s => aFix.StatusSmallEdges(NS(s)));
        }
        catch (N.OcctException e)
        {
            aNErr = ShapeFixOracleRun.ErrorOf(e);
        }

        TopoDS_Shape? aPRes = null;
        string? aPErr = null;
        string aPStatus = "";
        try
        {
            var aFix = new ShapeFix_Wireframe(aPS);
            aFix.SetPrecision(thePrecision);
            if (theMaxTol > 0.0)
            {
                aFix.SetMaxTolerance(theMaxTol);
            }
            if (theLimitAngle > 0.0)
            {
                aFix.SetLimitAngle(theLimitAngle);
            }
            bool aSmall = theFix != Fix.Gaps && aFix.FixSmallEdges();
            bool aGaps = theFix != Fix.Small && aFix.FixWireGaps();
            aPRes = aFix.Shape();
            aPStatus = $"{aSmall}/{aGaps}, " + Status(aFix.StatusWireGaps, aFix.StatusSmallEdges);
        }
        catch (FailureException e)
        {
            aPErr = ShapeFixOracleRun.ErrorOf(e);
        }
        theRun.Check(theWhat, aNRes, aNErr, aNStatus, aPRes, aPErr, aPStatus, aNS, aPS, anInputText);
    }

    [Fact]
    public void ShapeFix_Wireframe_on_random_wires_as_build123d_matches_native_OCCT()
    {
        var aWatch = Stopwatch.StartNew();
        using var aRun = new ShapeFixOracleRun(output, "occtnet-sfwf-");
        var anOracle = new Oracle(output, 63401);
        double[] aPrecisions = [1.0e-5, 1.0e-3, 1.0e-2, 0.1];
        foreach (int i in Tape.Budget(400))
        {
            N.TopoDS_Shape aWire = ShapeFixWireOracleTests.RandomWire(anOracle, i, out string aWhat);
            double aPrec = aPrecisions[i % aPrecisions.Length];
            Compare(aRun, aWire, $"random wire {i} ({aWhat}), FixDegenerateEdges({aPrec})", Fix.SmallThenGaps, aPrec, 1.0e-6);
            if (i % 3 == 0)
            {
                Compare(aRun, aWire, $"random wire {i} ({aWhat}), FixWireGaps({aPrec})", Fix.Gaps, aPrec);
            }
        }
        aRun.Report("ShapeFix_Wireframe on random sketch wires", aWatch.Elapsed);
        Assert.Empty(aRun.Differences);
    }

    [Fact]
    public void ShapeFix_Wireframe_on_the_heal_group_and_data_occ_matches_native_OCCT()
    {
        var aWatch = Stopwatch.StartNew();
        using var aRun = new ShapeFixOracleRun(output, "occtnet-sfwh-");
        int aShapes = 0;
        foreach ((string aFile, string aGrids) in Tape.Budget(aRun.Inputs(1_000_000), isSpread: true))
        {
            if (ShapeFixOracleRun.NativeRead(aFile) is not N.TopoDS_Shape aShape)
            {
                continue;
            }
            aShapes++;
            string aName = Path.GetFileName(aFile);
            Compare(aRun, aShape, $"{aName} ({aGrids}), fixwgaps 1e-3", Fix.Gaps, 1.0e-3);
            Compare(aRun, aShape, $"{aName} ({aGrids}), fixsmall 1", Fix.Small, 1.0);
            if (aShapes % 3 == 0)
            {
                Compare(aRun, aShape, $"{aName} ({aGrids}), fixsmall 1e-2, limit angle 0.2", Fix.Small, 1.0e-2, -1.0, 0.2);
            }
        }
        output.WriteLine($"{aShapes} shapes");
        aRun.Report("ShapeFix_Wireframe on the heal group's and data/occ's shapes", aWatch.Elapsed);
        Assert.Empty(aRun.Differences);
    }
}
