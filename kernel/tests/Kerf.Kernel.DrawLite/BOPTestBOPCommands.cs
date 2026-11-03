// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// The commands of BOPTest_BOPCommands.cxx: <c>mkvolume</c> (unit 6G), <c>bop</c>, <c>bopcommon</c>, <c>bopfuse</c>,
/// <c>bopcut</c>, <c>boptuc</c>, <c>bopsection</c>, <c>bcommon</c>, <c>bfuse</c>, <c>bcut</c>, <c>btuc</c>,
/// <c>bsection</c> and <c>bopcurves</c> (unit 6L).
/// </summary>
/// <remarks>
/// OCCT's file-static pave filler of <c>bop</c> is the session's (<see cref="BOPTestObjects.BopPaveFiller"/>).
/// Draw_ProgressIndicator goes: Draw-lite has no progress display. The Booleans of Draw's variables, which hold frozen
/// shapes, are non-destructive whatever <c>bnondestructive</c> says (decision 577); DRAWEXE's default is destructive.
/// </remarks>
internal static class BOPTestBOPCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "bop", Bop);
        DrawCxx.Add(theInterpreter, "bopcommon", (i, di, a) => BopSmt(i, di, a, Operation.Common));
        DrawCxx.Add(theInterpreter, "bopfuse", (i, di, a) => BopSmt(i, di, a, Operation.Fuse));
        DrawCxx.Add(theInterpreter, "bopcut", (i, di, a) => BopSmt(i, di, a, Operation.Cut));
        DrawCxx.Add(theInterpreter, "boptuc", (i, di, a) => BopSmt(i, di, a, Operation.CUT21));
        DrawCxx.Add(theInterpreter, "bopsection", BopSection);
        DrawCxx.Add(theInterpreter, "bcommon", (i, di, a) => BSmt(i, di, a, Operation.Common));
        DrawCxx.Add(theInterpreter, "bfuse", (i, di, a) => BSmt(i, di, a, Operation.Fuse));
        DrawCxx.Add(theInterpreter, "bcut", (i, di, a) => BSmt(i, di, a, Operation.Cut));
        DrawCxx.Add(theInterpreter, "btuc", (i, di, a) => BSmt(i, di, a, Operation.CUT21));
        DrawCxx.Add(theInterpreter, "bsection", BSection);
        DrawCxx.Add(theInterpreter, "bopcurves", BopCurves);
        DrawCxx.Add(theInterpreter, "mkvolume", MkVolume);
    }

    /// <summary><c>bop s1 s2</c>: intersects two shapes with the session's own pave filler (<c>pPF</c>).</summary>
    private static int Bop(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        NCollection_List<Shape> aLC = new();
        if (n != 3)
        {
            di.Append(" use bop s1 s2 \n");
            return 0;
        }
        Shape aS1 = theDI.GetShape(a[1]);
        Shape aS2 = theDI.GetShape(a[2]);
        if (aS1.IsNull || aS2.IsNull)
        {
            di.Append(" null shapes are not allowed \n");
            return 0;
        }
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        double aTol = aSession.FuzzyValue;
        bool bRunParallel = aSession.RunParallel;
        bool bNonDestructive = aSession.NonDestructive;
        GlueMode aGlue = aSession.Glue;
        aLC.Append(aS1);
        aLC.Append(aS2);
        aSession.BopPaveFiller = null;
        PaveFiller pPF = new();
        aSession.BopPaveFiller = pPF;
        pPF.SetArguments(aLC);
        pPF.SetFuzzyValue(aTol);
        pPF.SetRunParallel(bRunParallel);
        pPF.SetNonDestructive(bNonDestructive);
        pPF.SetGlue(aGlue);
        pPF.SetUseOBB(aSession.UseOBB);
        pPF.Perform();
        BOPTestObjects.ReportAlerts(theDI, di, pPF.GetReport());
        return 0;
    }

    /// <summary><c>bopcommon r</c> and the others: a Boolean of <c>bop</c>'s two shapes on its pave filler (bopsmt).</summary>
    private static int BopSmt(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a, Operation aOp)
    {
        int n = a.Count;
        if (n < 2)
        {
            di.Append(" use bopsmt r\n");
            return 0;
        }
        PaveFiller? pPF = BOPTestObjects.Of(theDI).BopPaveFiller;
        if (pPF is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 0;
        }
        if (pPF.HasErrors)
        {
            di.Append(" PaveFiller has not been done\n");
            return 0;
        }
        Bop aBOP = new();
        NCollection_List<Shape> aLC = pPF.Arguments();
        int aNb = aLC.Extent();
        if (aNb != 2)
        {
            di.Append(" wrong number of arguments ").Append(aNb).Append("\n");
            return 0;
        }
        bool bRunParallel = BOPTestObjects.Of(theDI).RunParallel;
        Shape aS1 = aLC.First();
        Shape aS2 = aLC.Last();
        aBOP.AddArgument(aS1);
        aBOP.AddTool(aS2);
        aBOP.SetOperation(aOp);
        aBOP.SetRunParallel(bRunParallel);
        aBOP.SetCheckInverted(BOPTestObjects.Of(theDI).CheckInverted);
        aBOP.SetToFillHistory(BRepTestHistoryCommands.IsHistoryNeeded(theDI));
        aBOP.PerformWithFiller(pPF);
        BOPTestObjects.ReportAlerts(theDI, di, aBOP.GetReport());

        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            BRepTestHistoryCommands.SetHistory(theDI, aBOP.History());
        }

        if (aBOP.HasErrors)
        {
            return 0;
        }
        Shape aR = aBOP.Shape();
        if (aR.IsNull)
        {
            di.Append(" null shape\n");
            return 0;
        }
        theDI.SetShape(a[1], aR);
        return 0;
    }

    /// <summary><c>bopsection r</c>: the section of <c>bop</c>'s two shapes on its pave filler.</summary>
    private static int BopSection(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            di.Append(" use bopsection r\n");
            return 0;
        }
        PaveFiller? pPF = BOPTestObjects.Of(theDI).BopPaveFiller;
        if (pPF is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 0;
        }
        if (pPF.HasErrors)
        {
            di.Append(" PaveFiller has not been done\n");
            return 0;
        }
        Section aBOP = new();
        NCollection_List<Shape> aLC = pPF.Arguments();
        int aNb = aLC.Extent();
        if (aNb != 2)
        {
            di.Append(" wrong number of arguments ").Append(aNb).Append("\n");
            return 0;
        }
        bool bRunParallel = BOPTestObjects.Of(theDI).RunParallel;
        Shape aS1 = aLC.First();
        Shape aS2 = aLC.Last();
        aBOP.AddArgument(aS1);
        aBOP.AddArgument(aS2);
        aBOP.SetRunParallel(bRunParallel);
        aBOP.SetCheckInverted(BOPTestObjects.Of(theDI).CheckInverted);
        aBOP.SetToFillHistory(BRepTestHistoryCommands.IsHistoryNeeded(theDI));
        aBOP.PerformWithFiller(pPF);
        BOPTestObjects.ReportAlerts(theDI, di, aBOP.GetReport());

        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            BRepTestHistoryCommands.SetHistory(theDI, aBOP.History());
        }

        if (aBOP.HasErrors)
        {
            return 0;
        }
        Shape aR = aBOP.Shape();
        if (aR.IsNull)
        {
            di.Append(" null shape\n");
            return 0;
        }
        theDI.SetShape(a[1], aR);
        return 0;
    }

    /// <summary><c>bsection r s1 s2 [-n2d/-n2d1/-n2d2] [-na]</c>: BRepAlgoAPI_Section of two shapes.</summary>
    private static int BSection(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            di.Append("use bsection r s1 s2 [-n2d/-n2d1/-n2d2] [-na] [tol]\n");
            return 0;
        }
        Shape aS1 = theDI.GetShape(a[2]);
        Shape aS2 = theDI.GetShape(a[3]);
        if (aS1.IsNull || aS2.IsNull)
        {
            di.Append(" Null shapes are not allowed \n");
            return 0;
        }
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        bool bApp = true;
        bool bPC1 = true;
        bool bPC2 = true;
        double aTol = aSession.FuzzyValue;
        bool bRunParallel = aSession.RunParallel;
        bool bNonDestructive = aSession.NonDestructive;
        GlueMode aGlue = aSession.Glue;
        for (int i = 4; i < n; ++i)
        {
            if (a[i] == "-n2d")
            {
                bPC1 = false;
                bPC2 = false;
            }
            else if (a[i] == "-n2d1")
            {
                bPC1 = false;
            }
            else if (a[i] == "-n2d2")
            {
                bPC2 = false;
            }
            else if (a[i] == "-na")
            {
                bApp = false;
            }
        }
        SectionOperation aSec = new(aS1, aS2, false);
        aSec.Approximation(bApp);
        aSec.ComputePCurveOn1(bPC1);
        aSec.ComputePCurveOn2(bPC2);
        aSec.SetFuzzyValue(aTol);
        aSec.SetRunParallel(bRunParallel);
        aSec.SetNonDestructive(bNonDestructive);
        aSec.SetGlue(aGlue);
        aSec.SetUseOBB(aSession.UseOBB);
        aSec.Build();
        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            BRepTestHistoryCommands.SetHistory(theDI, aSec.History());
        }

        if (aSec.HasWarnings)
        {
            StringWriter aSStream = new();
            aSec.DumpWarnings(aSStream);
            di.Append(aSStream.ToString());
        }
        if (!aSec.IsDone)
        {
            StringWriter aSStream = new();
            aSec.DumpErrors(aSStream);
            di.Append(aSStream.ToString());
            return 0;
        }
        Shape aR = aSec.Shape();
        if (aR.IsNull)
        {
            di.Append(" null shape\n");
            return 0;
        }
        theDI.SetShape(a[1], aR);
        return 0;
    }

    /// <summary><c>bcommon r s1 s2</c> and the others: a Boolean of two shapes (bsmt, BOPAlgo_BOP).</summary>
    private static int BSmt(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a, Operation aOp)
    {
        int n = a.Count;
        NCollection_List<Shape> aLC = new();
        if (n != 4)
        {
            di.Append(" use bx r s1 s2\n");
            return 0;
        }
        Shape aS1 = theDI.GetShape(a[2]);
        Shape aS2 = theDI.GetShape(a[3]);
        if (aS1.IsNull || aS2.IsNull)
        {
            di.Append(" null shapes are not allowed \n");
            return 0;
        }
        aLC.Append(aS1);
        aLC.Append(aS2);
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        Bop aBOP = new();
        aBOP.AddArgument(aS1);
        aBOP.AddTool(aS2);
        aBOP.SetOperation(aOp);
        aBOP.SetGlue(aSession.Glue);
        aBOP.SetFuzzyValue(aSession.FuzzyValue);
        aBOP.SetNonDestructive(aSession.NonDestructive);
        aBOP.SetRunParallel(aSession.RunParallel);
        aBOP.SetUseOBB(aSession.UseOBB);
        aBOP.SetCheckInverted(aSession.CheckInverted);
        aBOP.SetToFillHistory(BRepTestHistoryCommands.IsHistoryNeeded(theDI));
        aBOP.Perform();
        BOPTestObjects.ReportAlerts(theDI, di, aBOP.GetReport());

        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            BRepTestHistoryCommands.SetHistory(theDI, aBOP.PDS()!.Arguments(), new BOPTestBuilderHistory(aBOP));
        }

        if (aBOP.HasErrors)
        {
            return 0;
        }
        Shape aR = aBOP.Shape();
        if (aR.IsNull)
        {
            di.Append(" null shape\n");
            return 0;
        }
        theDI.SetShape(a[1], aR);
        return 0;
    }

    /// <summary>
    /// <c>bopcurves F1 F2 [-2d/-2d1/-2d2] [-p u1 v1 u2 v2] [-v]</c>: the intersection curves and points of two faces
    /// (IntTools_FaceFace), saved as <c>c_i</c>, <c>c2d1_i</c>, <c>c2d2_i</c> and <c>p_i</c>.
    /// </summary>
    private static int BopCurves(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            di.Append("Usage: bopcurves F1 F2 [-2d/-2d1/-2d2] "
                      + "[-p u1 v1 u2 v2 (to add start points] [-v (for extended output)]\n");
            return 1;
        }
        Shape S1 = theDI.GetShape(a[1]);
        Shape S2 = theDI.GetShape(a[2]);
        if (S1.IsNull || S2.IsNull)
        {
            di.Append(" Null shapes are not allowed \n");
            return 1;
        }
        ShapeType aType = S1.Type;
        if (aType != ShapeType.Face)
        {
            di.Append(" Type mismatch F1\n");
            return 1;
        }
        aType = S2.Type;
        if (aType != ShapeType.Face)
        {
            di.Append(" Type mismatch F2\n");
            return 1;
        }
        Face aF1 = ShapeCasts.Face(S1);
        Face aF2 = ShapeCasts.Face(S2);
        NCollection_List<PntOn2S> aListOfPnts = new();
        string aNm = "c_", aNp = "p_";
        double anAppTol = 0.0000001;
        bool aToApproxC3d = true;
        bool aToApproxC2dOnS1 = false;
        bool aToApproxC2dOnS2 = false;

        bool bExtOut = false;
        for (int i = 3; i < n; i++)
        {
            if (string.Equals(a[i], "-2d", StringComparison.OrdinalIgnoreCase))
            {
                aToApproxC2dOnS1 = true;
                aToApproxC2dOnS2 = true;
            }
            else if (string.Equals(a[i], "-2d1", StringComparison.OrdinalIgnoreCase))
            {
                aToApproxC2dOnS1 = true;
            }
            else if (string.Equals(a[i], "-2d2", StringComparison.OrdinalIgnoreCase))
            {
                aToApproxC2dOnS2 = true;
            }
            else if (string.Equals(a[i], "-p", StringComparison.OrdinalIgnoreCase))
            {
                PntOn2S aPt = new();
                double aU1 = theDI.Atof(a[++i]);
                double aV1 = theDI.Atof(a[++i]);
                double aU2 = theDI.Atof(a[++i]);
                double aV2 = theDI.Atof(a[++i]);

                aPt.SetValue(aU1, aV1, aU2, aV2);
                aListOfPnts.Append(aPt);
            }
            else if (string.Equals(a[i], "-v", StringComparison.OrdinalIgnoreCase))
            {
                bExtOut = true;
            }
            else
            {
                di.Append("Wrong key.\n");
                di.Append("To build 2d curves use one of the following keys: -2d/-2d1/-2d2\n");
                di.Append("To add start points use the following key: -p u1 v1 u2 v2\n");
                di.Append("For extended output use the following key: -v\n");
                return 1;
            }
        }
        FaceFace aFF = new();
        aFF.SetParameters(aToApproxC3d, aToApproxC2dOnS1, aToApproxC2dOnS2, anAppTol);
        aFF.SetList(aListOfPnts);
        aFF.SetFuzzyValue(BOPTestObjects.Of(theDI).FuzzyValue);
        aFF.Perform(aF1, aF2, BOPTestObjects.Of(theDI).RunParallel);
        bool anIsDone = aFF.IsDone;
        if (!anIsDone)
        {
            di.Append("Error: Intersection failed\n");
            return 0;
        }
        aFF.PrepareLines3D(false);
        NCollection_Sequence<IntersectionCurve> aSCs = aFF.Lines();
        NCollection_Sequence<PntOn2Faces> aSPs = aFF.Points();
        int aNbCurves = aSCs.Length();
        int aNbPoints = aSPs.Length();
        if (aNbCurves == 0 && aNbPoints == 0)
        {
            di.Append(" has no 3d curves\n");
            di.Append(" has no 3d points\n");
            return 0;
        }
        if (aNbCurves != 0)
        {
            double aTolR = 0.0;
            if (!bExtOut)
            {
                for (int i = 1; i <= aNbCurves; i++)
                {
                    IntersectionCurve anIC = aSCs[i];
                    if (aTolR < anIC.Tolerance())
                    {
                        aTolR = anIC.Tolerance();
                    }
                }
                di.Append("Tolerance Reached=").Append(aTolR).Append("\n");
            }
            di.Append(aNbCurves).Append(" curve(s) found.\n");
            for (int i = 1; i <= aNbCurves; i++)
            {
                IntersectionCurve anIC = aSCs[i];

                Curve3D? aC3D = anIC.Curve();

                if (aC3D is null)
                {
                    di.Append(" has Null 3d curve# ").Append(i).Append("\n");
                    continue;
                }

                string anIndx = i.ToString(System.Globalization.CultureInfo.InvariantCulture);
                string nameC = aNm + anIndx;

                DrawTrSurf.Set(theDI, nameC, aC3D);
                di.Append(nameC).Append(" ");
                Curve2D? aPC1 = anIC.FirstCurve2d();
                Curve2D? aPC2 = anIC.SecondCurve2d();
                if (aPC1 is not null || aPC2 is not null)
                {
                    di.Append("(");
                    if (aPC1 is not null)
                    {
                        string nameC2d1 = "c2d1_" + anIndx;
                        DrawTrSurf.Set(theDI, nameC2d1, aPC1);
                        di.Append(nameC2d1);
                    }
                    if (aPC2 is not null)
                    {
                        string nameC2d2 = "c2d2_" + anIndx;
                        DrawTrSurf.Set(theDI, nameC2d2, aPC2);
                        if (aPC1 is not null)
                        {
                            di.Append(", ");
                        }
                        di.Append(nameC2d2);
                    }
                    di.Append(") ");
                }
                if (bExtOut)
                {
                    di.Append("\nTolerance: ").Append(anIC.Tolerance()).Append("\n");
                    di.Append("Tangential tolerance: ").Append(anIC.TangentialTolerance()).Append("\n");
                    di.Append("\n");
                }
            }
            if (!bExtOut)
            {
                di.Append("\n");
            }
        }
        if (aNbPoints != 0)
        {
            di.Append(aNbPoints).Append(" point(s) found.\n");
            for (int i = 1; i <= aNbPoints; i++)
            {
                PntOn2Faces aPi = aSPs[i];
                Point3D aP = aPi.P1().Pnt();
                string nameP = aNp + i.ToString(System.Globalization.CultureInfo.InvariantCulture);
                DrawTrSurf.Set(theDI, nameP, aP);
                di.Append(nameP).Append(" ");
            }
            di.Append("\n");
        }
        return 0;
    }

    /// <summary><c>mkvolume r b1 b2 ... [-c] [-ni] [-ai]</c>: solids from a set of shapes (BOPAlgo_MakerVolume).</summary>
    private static int MkVolume(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            di.Append("Usage: mkvolume r b1 b2 ... [-c] [-ni] [-ai]\n");
            di.Append("Options:\n");
            di.Append(" -c  - use this option if the arguments are compounds\n");
            di.Append("       containing shapes that should be interfered;\n");
            di.Append(" -ni - use this option if the arguments should not be interfered;\n");
            di.Append(" -ai - use this option to avoid internal for solids shapes in the result.\n");
            return 1;
        }
        //
        const string usage = "Type mkvolume without arguments for the usage of the command.\n";
        //
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        double aTol = aSession.FuzzyValue;
        bool bRunParallel = aSession.RunParallel;
        bool bNonDestructive = aSession.NonDestructive;
        GlueMode aGlue = aSession.Glue;
        //
        bool bToIntersect = true;
        bool bCompounds = false;
        bool bAvoidInternal = false;
        NCollection_List<Shape> aLS = new();
        //
        for (int i = 2; i < n; ++i)
        {
            Shape aS = theDI.GetShape(a[i]);
            if (!aS.IsNull)
            {
                aLS.Append(aS);
            }
            else
            {
                if (a[i] == "-c")
                {
                    bCompounds = true;
                }
                else if (a[i] == "-ni")
                {
                    bToIntersect = false;
                }
                else if (a[i] == "-ai")
                {
                    bAvoidInternal = true;
                }
            }
        }
        //
        if (aLS.IsEmpty())
        {
            di.Append("No shapes to process.\n");
            di.Append(usage);
            return 1;
        }
        //
        // treat list of arguments for the case of compounds
        if (bToIntersect && bCompounds)
        {
            NCollection_List<Shape> aLSx = new();
            foreach (Shape aSx in aLS)
            {
                for (Iterator aItS = new(aSx); aItS.More(); aItS.Next())
                {
                    aLSx.Append(aItS.Value());
                }
            }
            //
            aLS.Clear();
            aLS.Assign(aLSx);
        }
        //
        MakerVolume aMV = new();
        aMV.SetArguments(aLS);
        aMV.SetIntersect(bToIntersect);
        aMV.SetRunParallel(bRunParallel);
        aMV.SetFuzzyValue(aTol);
        aMV.SetNonDestructive(bNonDestructive);
        aMV.SetAvoidInternalShapes(bAvoidInternal);
        aMV.SetGlue(aGlue);
        aMV.SetUseOBB(aSession.UseOBB);
        aMV.SetToFillHistory(BRepTestHistoryCommands.IsHistoryNeeded(theDI));
        //
        aMV.Perform();
        BOPTestObjects.ReportAlerts(theDI, di, aMV.GetReport());

        // Store the history of Volume Maker into the session
        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            BRepTestHistoryCommands.SetHistory(theDI, aLS, new BOPTestBuilderHistory(aMV));
        }

        if (aMV.HasErrors)
        {
            return 0;
        }
        //
        theDI.SetShape(a[1], aMV.Shape());
        return 0;
    }
}

/// <summary>
/// The standard history methods of a BOPAlgo builder, for BRepTest_Objects::SetHistory(arguments, algorithm) (OCCT's
/// template over any algorithm with IsDeleted, Modified and Generated).
/// </summary>
internal sealed class BOPTestBuilderHistory(BuilderShape theAlgo) : History.IAlgorithm
{
    public bool IsDeleted(in Shape theS) => theAlgo.IsDeleted(theS);

    public IEnumerable<Shape> Modified(in Shape theS) => theAlgo.Modified(theS);

    public IEnumerable<Shape> Generated(in Shape theS) => theAlgo.Generated(theS);
}
