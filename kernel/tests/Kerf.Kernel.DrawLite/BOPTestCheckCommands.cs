// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using System.Diagnostics;
using System.Globalization;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// The commands of BOPTest_CheckCommands.cxx (unit 6G): <c>bopcheck</c> (BOPAlgo_CheckerSI), <c>bopargcheck</c>
/// (BOPAlgo_ArgumentAnalyzer), <c>bopapicheck</c> (BRepAlgoAPI_Check), <c>xdistef</c> and <c>checkcurveonsurf</c>
/// (BOPTools_AlgoTools::ComputeTolerance).
/// </summary>
/// <remarks>
/// They read the session's fuzzy value and parallel flag (<see cref="BOPTestObjects"/>). Draw's progress indicator
/// prints nothing by default and isn't kept; PrintHelp's usage errors return 1 with an empty result. The checks only
/// read their shapes; <c>bopargcheck</c> checks a copy of the second shape (BRepBuilderAPI_Copy), as OCCT's does.
/// </remarks>
internal static class BOPTestCheckCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "bopcheck", BopCheck);
        DrawCxx.Add(theInterpreter, "bopargcheck", BopArgCheck);
        DrawCxx.Add(theInterpreter, "xdistef", XDistEF);
        DrawCxx.Add(theInterpreter, "checkcurveonsurf", CheckCurveOnSurf);
        DrawCxx.Add(theInterpreter, "bopapicheck", BopApiCheck);
    }

    // BOPTest_Interf: an interference's indices and type, ordered by type, then the first index, then the second
    private readonly record struct BOPTestInterf(int Index1, int Index2, int Type) : IComparable<BOPTestInterf>
    {
        public int CompareTo(BOPTestInterf aOther)
        {
            if (Type != aOther.Type)
            {
                return Type.CompareTo(aOther.Type);
            }
            if (Index1 != aOther.Index1)
            {
                return Index1.CompareTo(aOther.Index1);
            }
            return Index2.CompareTo(aOther.Index2);
        }
    }

    private static int BopCheck(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            di.Append(" use bopcheck Shape [level of check: 0 - 9] [-t]\n");
            di.Append(" The level of check defines ");
            di.Append(" which interferences will be checked:\n");
            di.Append(" 0 - V/V only\n");
            di.Append(" 1 - V/V, V/E\n");
            di.Append(" 2 - V/V, V/E, E/E\n");
            di.Append(" 3 - V/V, V/E, E/E , V/F\n");
            di.Append(" 4 - V/V, V/E, E/E, V/F , E/F\n");
            di.Append(" 5 - V/V, V/E, E/E, V/F, E/F, F/F;\n");
            di.Append(" 6 - V/V, V/E, E/E, V/F, E/F, F/F, V/Z\n");
            di.Append(" 7 - V/V, V/E, E/E, V/F, E/F, F/F, E/Z\n");
            di.Append(" 8 - V/V, V/E, E/E, V/F, E/F, F/F, E/Z, F/Z\n");
            di.Append(" 9 - V/V, V/E, E/E, V/F, E/F, F/F, E/Z, F/Z, Z/Z\n");
            di.Append(" Default level is 9\n");
            return 1;
        }
        //
        Shape aS = theDI.GetShape(a[1]);
        if (aS.IsNull)
        {
            di.Append("null shapes are not allowed here!");
            return 1;
        }
        //
        int aNbInterfTypes = DS.InterfTypeCount;
        int aLevel = aNbInterfTypes - 1;
        //
        if (n > 2)
        {
            if (a[2].Length == 0 || a[2][0] != '-')
            {
                aLevel = theDI.Atoi(a[2]);
            }
        }
        //
        if (aLevel < 0 || aLevel > aNbInterfTypes - 1)
        {
            di.Append("Invalid level");
            return 1;
        }
        //
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        bool bShowTime = false;
        double aTol = aSession.FuzzyValue;
        bool bRunParallel = aSession.RunParallel;
        //
        for (int i = 2; i < n; ++i)
        {
            if (a[i] == "-t")
            {
                bShowTime = true;
            }
        }
        //
        string[] aInterfTypes = ["V/V", "V/E", "E/E", "V/F", "E/F", "F/F", "V/Z", "E/Z", "F/Z", "Z/Z"];
        //
        CheckerSI aChecker = new();
        NCollection_List<Shape> aLS = new();
        //
        if (aLevel < (aNbInterfTypes - 1))
        {
            di.Append("Info:\nThe level of check is set to ").Append(aInterfTypes[aLevel]).Append(", i.e. intersection(s)\n");

            for (int i = aLevel + 1; i < aNbInterfTypes; ++i)
            {
                di.Append(aInterfTypes[i]);
                if (i < aNbInterfTypes - 1)
                {
                    di.Append(", ");
                }
            }
            di.Append(" will not be checked.\n\n");
        }
        //
        aLS.Append(aS);
        aChecker.SetArguments(aLS);
        aChecker.SetLevelOfCheck(aLevel);
        aChecker.SetRunParallel(bRunParallel);
        aChecker.SetFuzzyValue(aTol);
        //
        Stopwatch aTimer = Stopwatch.StartNew();
        aChecker.Perform();
        aTimer.Stop();
        //
        BOPTestObjects.ReportAlerts(theDI, di, aChecker.GetReport());
        //
        bool iErr = aChecker.HasErrors;
        //
        DS aDS = aChecker.PDS()!;
        //
        List<BOPTestInterf> aVec = [];
        //
        for (MapOfPair.Iterator aItMPK = new(aDS.Interferences()); aItMPK.More(); aItMPK.Next())
        {
            aItMPK.Value().Indices(out int n1, out int n2);
            if (aDS.IsNewShape(n1) || aDS.IsNewShape(n2))
            {
                continue;
            }
            //
            ShapeType aType1 = aDS.ShapeInfo(n1).ShapeType();
            ShapeType aType2 = aDS.ShapeInfo(n2).ShapeType();
            //
            int iT = DataTools.TypeToInteger(aType1, aType2);
            aVec.Add(new BOPTestInterf(n1, n2, iT));
        }
        //
        // std::sort: the order is total (the pairs are distinct)
        aVec.Sort();
        //
        int iCnt = 0;
        foreach (BOPTestInterf aBI in aVec)
        {
            int n1 = aBI.Index1, n2 = aBI.Index2;
            if (aDS.IsNewShape(n1) || aDS.IsNewShape(n2))
            {
                continue;
            }
            //
            // the shape's own sub-shapes (the checker works on a copy of a frozen shape)
            Shape aS1 = aChecker.Original(aDS.Shape(n1));
            Shape aS2 = aChecker.Original(aDS.Shape(n2));
            //
            di.Append(aInterfTypes[aBI.Type]).Append(": ");
            //
            string aName1 = "x" + n1.ToString(CultureInfo.InvariantCulture);
            theDI.SetShape(aName1, aS1);
            //
            ++iCnt;
            string aName2 = "x" + n2.ToString(CultureInfo.InvariantCulture);
            theDI.SetShape(aName2, aS2);
            ++iCnt;
            //
            di.Append($"{aName1} {aName2} \n");
        }
        //
        if (iErr)
        {
            di.Append("There were errors during the operation, ");
            di.Append("so the list may be incomplete.\n");
        }
        //
        if (iCnt == 0)
        {
            di.Append(" This shape seems to be OK.\n");
        }
        if (bShowTime)
        {
            di.Append($"  Tps: {aTimer.Elapsed.TotalSeconds.ToString("F2", CultureInfo.InvariantCulture),7}\n");
        }
        return 0;
    }

    private static int BopArgCheck(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            di.Append("\n");
            di.Append(" Use >bopargcheck Shape1 [[Shape2] ");
            di.Append("[-F/O/C/T/S/U] [/R|F|T|V|E|I|P|C|S]] [#BF]\n\n");
            di.Append(" -<Boolean Operation>\n");
            di.Append(" F (fuse)\n");
            di.Append(" O (common)\n");
            di.Append(" C (cut)\n");
            di.Append(" T (cut21)\n");
            di.Append(" S (section)\n");
            di.Append(" U (unknown)\n");
            di.Append(" For example: \"bopargcheck s1 s2 -F\" enables");
            di.Append(" checking for Fuse operation\n");
            di.Append(" default - section\n\n");
            di.Append(" /<Test Options>\n");
            di.Append(" R (disable small edges (shrank range) test)\n");
            di.Append(" F (disable faces verification test)\n");
            di.Append(" T (disable tangent faces searching test)\n");
            di.Append(" V (disable test possibility to merge vertices)\n");
            di.Append(" E (disable test possibility to merge edges)\n");
            di.Append(" I (disable self-interference test)\n");
            di.Append(" P (disable shape type test)\n");
            di.Append(" C (disable test for shape continuity)\n");
            di.Append(" S (disable curve on surface check)\n");
            di.Append(" For example: \"bopargcheck s1 s2 /RI\" disables ");
            di.Append("small edge detection and self-intersection detection\n");
            di.Append(" default - all options are enabled\n\n");
            di.Append(" #<Additional Test Options>\n");
            di.Append(" B (stop test on first faulty found); default OFF\n");
            di.Append(" F (full output for faulty shapes); default - output ");
            di.Append("in a short format\n\n");
            di.Append(" NOTE: <Boolean Operation> and <Test Options> are ");
            di.Append("used only for couple\n");
            di.Append("       of argument shapes, except I and P options ");
            di.Append("that are always used for\n");
            di.Append("       couple of shapes as well as for ");
            di.Append("single shape test.\n");
            return 1;
        }

        Shape aS1 = theDI.GetShape(a[1]);

        if (aS1.IsNull)
        {
            di.Append("Error: null shape not allowed!\n");
            di.Append("Type bopargcheck without arguments for more ");
            di.Append("information\n");
            return 1;
        }

        bool isBO = false;
        int indxBO = 0;
        bool isOP = false;
        int indxOP = 0;
        bool isAD = false;
        int indxAD = 0;
        bool isS2 = false;
        int indxS2 = 0;
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        bool bRunParallel = aSession.RunParallel;
        double aTolerance = aSession.FuzzyValue;

        if (n >= 3)
        {
            for (int iIndex = 2; iIndex < n; iIndex++)
            {
                char aFirst = a[iIndex].Length > 0 ? a[iIndex][0] : '\0';
                if (aFirst == '-')
                {
                    isBO = true;
                    indxBO = iIndex;
                }
                else if (aFirst == '/')
                {
                    isOP = true;
                    indxOP = iIndex;
                }
                else if (aFirst == '#')
                {
                    isAD = true;
                    indxAD = iIndex;
                }
                else
                {
                    isS2 = true;
                    indxS2 = iIndex;
                }
            }
        }

        // set & test second shape
        Shape aS22 = new(), aS2 = new();
        if (isS2)
        {
            if (indxS2 != 2)
            {
                di.Append("Error: second shape should follow the first one!\n");
                di.Append("Type bopargcheck without arguments for more information\n");
                return 1;
            }
            else
            {
                aS22 = theDI.GetShape(a[2]);
                if (aS22.IsNull)
                {
                    di.Append("Error: second shape is null!\n");
                    di.Append("Type bopargcheck without arguments for more information\n");
                    return 1;
                }
            }
        }

        // a C string's characters, with the terminating '\0' past the end
        static char At(string theWord, int theIndex) => theIndex < theWord.Length ? theWord[theIndex] : '\0';

        // init checker
        ArgumentAnalyzer aChecker = new();
        aChecker.SetRunParallel(bRunParallel);
        aChecker.SetFuzzyValue(aTolerance);
        aChecker.SetShape1(aS1);

        // set default options (always tested!) for single and couple shapes
        aChecker.ArgumentTypeMode() = true;
        aChecker.SelfInterMode() = true;
        aChecker.SmallEdgeMode() = true;
        aChecker.RebuildFaceMode() = true;
        aChecker.ContinuityMode() = true;
        aChecker.CurveOnSurfaceMode() = true;

        // test & set options and operation for two shapes
        if (!aS22.IsNull)
        {
            aS2 = new Copy(aS22).Shape();
            aChecker.SetShape2(aS2);
            // set operation (default - Section)
            if (isBO)
            {
                char aC = At(a[indxBO], 1);
                if (aC == 'F' || aC == 'f')
                {
                    aChecker.OperationType() = Operation.Fuse;
                }
                else if (aC == 'O' || aC == 'o')
                {
                    aChecker.OperationType() = Operation.Common;
                }
                else if (aC == 'C' || aC == 'c')
                {
                    aChecker.OperationType() = Operation.Cut;
                }
                else if (aC == 'T' || aC == 't')
                {
                    aChecker.OperationType() = Operation.CUT21;
                }
                else if (aC == 'S' || aC == 's')
                {
                    aChecker.OperationType() = Operation.Section;
                }
                else if (aC == 'U' || aC == 'u')
                {
                    aChecker.OperationType() = Operation.Unknown;
                }
                else
                {
                    di.Append("Error: invalid boolean operation type!\n");
                    di.Append("Type bopargcheck without arguments for more information\n");
                    return 1;
                }
            }
            else
            {
                aChecker.OperationType() = Operation.Section;
            }

            aChecker.TangentMode() = true;
            aChecker.MergeVertexMode() = true;
            aChecker.MergeEdgeMode() = true;
        }

        // set options (default - all ON)
        if (isOP)
        {
            for (int ind = 1; At(a[indxOP], ind) != '\0'; ind++)
            {
                char aC = At(a[indxOP], ind);
                if (aC == 'R' || aC == 'r')
                {
                    aChecker.SmallEdgeMode() = false;
                }
                else if (aC == 'F' || aC == 'f')
                {
                    aChecker.RebuildFaceMode() = false;
                }
                else if (aC == 'T' || aC == 't')
                {
                    aChecker.TangentMode() = false;
                }
                else if (aC == 'V' || aC == 'v')
                {
                    aChecker.MergeVertexMode() = false;
                }
                else if (aC == 'E' || aC == 'e')
                {
                    aChecker.MergeEdgeMode() = false;
                }
                else if (aC == 'I' || aC == 'i')
                {
                    aChecker.SelfInterMode() = false;
                }
                else if (aC == 'P' || aC == 'p')
                {
                    aChecker.ArgumentTypeMode() = false;
                }
                else if (aC == 'C' || aC == 'c')
                {
                    aChecker.ContinuityMode() = false;
                }
                else if (aC == 'S' || aC == 's')
                {
                    aChecker.CurveOnSurfaceMode() = false;
                }
                else
                {
                    di.Append("Error: invalid test option(s)!\n");
                    di.Append("Type bopargcheck without arguments for more information\n");
                    return 1;
                }
            }
        }

        // set additional options
        bool fullOutput = false;
        if (isAD)
        {
            for (int ind = 1; At(a[indxAD], ind) != '\0'; ind++)
            {
                char aC = At(a[indxAD], ind);
                if (aC == 'B' || aC == 'b')
                {
                    aChecker.StopOnFirstFaulty() = true;
                }
                else if (aC == 'F' || aC == 'f')
                {
                    fullOutput = true;
                }
                else
                {
                    di.Append("Error: invalid additional test option(s)!\n");
                    di.Append("Type bopargcheck without arguments for more information\n");
                    return 1;
                }
            }
        }

        // run checker
        aChecker.Perform();
        // process result of checking
        if (!aChecker.HasFaulty)
        {
            di.Append("Shape(s) seem(s) to be valid for BOP.").Append("\n");
        }
        else
        {
            if (!fullOutput)
            {
                di.Append("Faulties, that can not be treated by BOP, are detected.\n");
            }
            else
            {
                FullOutput(theDI, di, aChecker);
            }
        }

        return 0;
    }

    // bopargcheck's full output (#F)
    private static void FullOutput(DrawInterpreter theDI, DrawResult di, ArgumentAnalyzer aChecker)
    {
        int S1_BadType = 0, S1_SelfInt = 0, S1_SmalE = 0, S1_BadF = 0, S1_BadV = 0, S1_BadE = 0;
        int S1_SelfIntAll = 0, S1_SmalEAll = 0, S1_BadFAll = 0, S1_BadVAll = 0, S1_BadEAll = 0;
        int S2_BadType = 0, S2_SelfInt = 0, S2_SmalE = 0, S2_BadF = 0, S2_BadV = 0, S2_BadE = 0;
        int S2_SelfIntAll = 0, S2_SmalEAll = 0, S2_BadFAll = 0, S2_BadVAll = 0, S2_BadEAll = 0;
        int S1_OpAb = 0, S2_OpAb = 0;
        int S1_C0 = 0, S2_C0 = 0, S1_C0All = 0, S2_C0All = 0;
        int S1_COnS = 0, S2_COnS = 0, S1_COnSAll = 0, S2_COnSAll = 0;
        bool hasUnknown = false;

        foreach (CheckResult aResult in aChecker.InternalGetCheckResult())
        {
            Shape aSS1 = aResult.InternalGetShape1();
            Shape aSS2 = aResult.InternalGetShape2();
            NCollection_List<Shape> aLS1 = aResult.InternalGetFaultyShapes1();
            NCollection_List<Shape> aLS2 = aResult.InternalGetFaultyShapes2();
            bool isL1 = !aLS1.IsEmpty();
            bool isL2 = !aLS2.IsEmpty();

            void Count(ref int theS1, ref int theS1All, ref int theS2, ref int theS2All, string theBase1, string theBase2,
                       bool theCurveOnSurf = false)
            {
                if (!aSS1.IsNull)
                {
                    theS1++;
                    if (isL1)
                    {
                        MakeShapeForFullOutput(theDI, theBase1, theS1, aLS1, ref theS1All, di, theCurveOnSurf,
                                               aResult.GetMaxDistance1(), aResult.GetMaxParameter1());
                    }
                }
                if (!aSS2.IsNull)
                {
                    theS2++;
                    if (isL2)
                    {
                        MakeShapeForFullOutput(theDI, theBase2, theS2, aLS2, ref theS2All, di, theCurveOnSurf,
                                               aResult.GetMaxDistance2(), aResult.GetMaxParameter2());
                    }
                }
            }

            switch (aResult.GetCheckStatus())
            {
                case CheckStatus.BadType:
                {
                    if (!aSS1.IsNull)
                    {
                        S1_BadType++;
                    }
                    if (!aSS2.IsNull)
                    {
                        S2_BadType++;
                    }
                }
                break;
                case CheckStatus.SelfIntersect:
                    Count(ref S1_SelfInt, ref S1_SelfIntAll, ref S2_SelfInt, ref S2_SelfIntAll, "s1si_", "s2si_");
                    break;
                case CheckStatus.TooSmallEdge:
                    Count(ref S1_SmalE, ref S1_SmalEAll, ref S2_SmalE, ref S2_SmalEAll, "s1se_", "s2se_");
                    break;
                case CheckStatus.NonRecoverableFace:
                    Count(ref S1_BadF, ref S1_BadFAll, ref S2_BadF, ref S2_BadFAll, "s1bf_", "s2bf_");
                    break;
                case CheckStatus.IncompatibilityOfVertex:
                    Count(ref S1_BadV, ref S1_BadVAll, ref S2_BadV, ref S2_BadVAll, "s1bv_", "s2bv_");
                    break;
                case CheckStatus.IncompatibilityOfEdge:
                    Count(ref S1_BadE, ref S1_BadEAll, ref S2_BadE, ref S2_BadEAll, "s1be_", "s2be_");
                    break;
                case CheckStatus.IncompatibilityOfFace:
                {
                    // not yet implemented
                }
                break;
                case CheckStatus.GeomAbs_C0:
                    Count(ref S1_C0, ref S1_C0All, ref S2_C0, ref S2_C0All, "s1C0_", "s2C0_");
                    break;
                case CheckStatus.InvalidCurveOnSurface:
                    Count(ref S1_COnS, ref S1_COnSAll, ref S2_COnS, ref S2_COnSAll, "s1COnS_", "s2COnS_", true);
                    break;
                case CheckStatus.OperationAborted:
                {
                    if (!aSS1.IsNull)
                    {
                        S1_OpAb++;
                    }
                    if (!aSS2.IsNull)
                    {
                        S2_OpAb++;
                    }
                }
                break;
                default:
                {
                    hasUnknown = true;
                }
                break;
            } // switch
        } // faulties

        int FS1 = S1_SelfInt + S1_SmalE + S1_BadF + S1_BadV + S1_BadE + S1_OpAb + S1_C0 + S1_COnS;
        FS1 += (S1_BadType != 0) ? 1 : 0;
        int FS2 = S2_SelfInt + S2_SmalE + S2_BadF + S2_BadV + S2_BadE + S2_OpAb + S2_C0 + S2_COnS;
        FS2 += (S2_BadType != 0) ? 1 : 0;

        void Line(string theLabel, int theCount, int theAll, bool theMode)
        {
            di.Append(theLabel).Append(theCount != 0 ? "YES" : theMode ? "NO" : "DISABLED");
            if (theCount != 0)
            {
                di.Append("  Cases(").Append(theCount).Append(")  Total shapes(").Append(theAll).Append(")\n");
            }
            else
            {
                di.Append("\n");
            }
        }

        // output for first shape
        di.Append("Faulties for FIRST  shape found : ").Append(FS1).Append("\n");
        if (FS1 != 0)
        {
            di.Append("---------------------------------\n");
            di.Append("Shapes are not suppotrted by BOP: ")
              .Append(S1_BadType != 0 ? "YES" : aChecker.ArgumentTypeMode() ? "NO" : "DISABLED").Append("\n");
            Line("Self-Intersections              : ", S1_SelfInt, S1_SelfIntAll, aChecker.SelfInterMode());
            di.Append("Check for SI has been aborted   : ")
              .Append(S1_OpAb != 0 ? "YES" : aChecker.SelfInterMode() ? "NO" : "DISABLED").Append("\n");
            Line("Too small edges                 : ", S1_SmalE, S1_SmalEAll, aChecker.SmallEdgeMode());
            Line("Bad faces                       : ", S1_BadF, S1_BadFAll, aChecker.RebuildFaceMode());
            Line("Too close vertices              : ", S1_BadV, S1_BadVAll, aChecker.MergeVertexMode());
            Line("Too close edges                 : ", S1_BadE, S1_BadEAll, aChecker.MergeEdgeMode());
            Line("Shapes with Continuity C0       : ", S1_C0, S1_C0All, aChecker.ContinuityMode());
            Line("Invalid Curve on Surface        : ", S1_COnS, S1_COnSAll, aChecker.CurveOnSurfaceMode());
        }

        // output for second shape
        di.Append("\n");
        di.Append("Faulties for SECOND  shape found : ").Append(FS2).Append("\n");
        if (FS2 != 0)
        {
            di.Append("---------------------------------\n");
            di.Append("Shapes are not suppotrted by BOP: ")
              .Append(S2_BadType != 0 ? "YES" : aChecker.ArgumentTypeMode() ? "NO" : "DISABLED").Append("\n");
            Line("Self-Intersections              : ", S2_SelfInt, S2_SelfIntAll, aChecker.SelfInterMode());
            di.Append("Check for SI has been aborted   : ")
              .Append(S2_OpAb != 0 ? "YES" : aChecker.SelfInterMode() ? "NO" : "DISABLED").Append("\n");
            Line("Too small edges                 : ", S2_SmalE, S2_SmalEAll, aChecker.SmallEdgeMode());
            Line("Bad faces                       : ", S2_BadF, S2_BadFAll, aChecker.RebuildFaceMode());
            Line("Too close vertices              : ", S2_BadV, S2_BadVAll, aChecker.MergeVertexMode());
            Line("Too close edges                 : ", S2_BadE, S2_BadEAll, aChecker.MergeEdgeMode());
            Line("Shapes with Continuity C0       : ", S2_C0, S2_C0All, aChecker.ContinuityMode());
            Line("Invalid Curve on Surface        : ", S2_COnS, S2_COnSAll, aChecker.CurveOnSurfaceMode());
        }
        // warning
        if (hasUnknown)
        {
            di.Append("\n");
            di.Append("WARNING: The unexpected test break occurs!\n");
        }
    }

    // MakeShapeForFullOutput: the faulty shapes as the compound <base><index>, counted
    private static void MakeShapeForFullOutput(DrawInterpreter theDI, string aBaseName, int aIndex,
                                               NCollection_List<Shape> aList, ref int aCount, DrawResult di,
                                               bool bCurveOnSurf, double aMaxDist, double aMaxParameter)
    {
        string name = aBaseName + aIndex.ToString(CultureInfo.InvariantCulture);

        Compound cmp = new();
        BrepBuilder BB = new();
        BB.MakeCompound(ref cmp);

        foreach (Shape aS in aList)
        {
            BB.Add(cmp, aS);
            aCount++;
        }
        di.Append("Made faulty shape: ").Append(name);
        //
        if (bCurveOnSurf)
        {
            di.Append(" (MaxDist = ").Append(aMaxDist).Append(", MaxPar = ").Append(aMaxParameter).Append(")");
        }
        //
        di.Append("\n");
        //
        theDI.SetShape(name, cmp);
    }

    private static int BopApiCheck(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            return 1;
        }

        Shape aS1 = theDI.GetShape(a[1]);
        Shape aS2 = new();
        if (n > 2)
        {
            // Try to get the second shape
            aS2 = theDI.GetShape(a[2]);
        }

        Operation anOp = Operation.Unknown;
        bool bTestSE = true;
        bool bTestSI = true;

        for (int i = aS2.IsNull ? 2 : 3; i < n; ++i)
        {
            if (a[i] == "-op")
            {
                // Get the operation type
                ++i;
                // OCCT reads a[n] (the terminating null pointer) when -op ends the line; strcmp on it crashes DRAWEXE
                string anOpName = i < n ? a[i] : "";
                if (anOpName == "common")
                {
                    anOp = Operation.Common;
                }
                else if (anOpName == "fuse")
                {
                    anOp = Operation.Fuse;
                }
                else if (anOpName == "cut")
                {
                    anOp = Operation.Cut;
                }
                else if (anOpName == "tuc")
                {
                    anOp = Operation.CUT21;
                }
                else if (anOpName == "section")
                {
                    anOp = Operation.Section;
                }
            }
            else if (a[i] == "-se")
            {
                bTestSE = false;
            }
            else if (a[i] == "-si")
            {
                bTestSI = false;
            }
            else
            {
                di.Append("Invalid key: ").Append(a[i]).Append(". Skipped.\n");
            }
        }

        Check aChecker = new(aS1, aS2, anOp, bTestSE, bTestSI);
        if (aChecker.IsValid)
        {
            if (aS2.IsNull)
            {
                di.Append("The shape seems to be valid\n");
            }
            else
            {
                di.Append("The shapes seem to be valid\n");
            }
            return 0;
        }

        // Shapes seem to be invalid.
        // Analyze the invalidity.

        bool isInv1 = false, isInv2 = false;
        bool isBadOp = false;
        foreach (CheckResult aFaulty in aChecker.InternalResult())
        {
            if (aFaulty.GetCheckStatus() == CheckStatus.BadType)
            {
                isBadOp = true;
            }
            else
            {
                if (!isInv1)
                {
                    isInv1 = !aFaulty.InternalGetShape1().IsNull;
                }
                if (!isInv2)
                {
                    isInv2 = !aFaulty.InternalGetShape2().IsNull;
                }
            }

            if (isInv1 && isInv2 && isBadOp)
            {
                break;
            }
        }

        if (isInv1)
        {
            if (aS2.IsNull)
            {
                di.Append("The shape is invalid\n");
            }
            else
            {
                di.Append("The first shape is invalid\n");
            }
        }
        if (isInv2)
        {
            di.Append("The second shape is invalid\n");
        }
        if (isBadOp)
        {
            if (aS2.IsNull)
            {
                di.Append("The shape is empty\n");
            }
            else
            {
                di.Append("The shapes are not valid for Boolean operation\n");
            }
        }
        return 0;
    }

    private static int XDistEF(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            di.Append("use xdistef edge face\n");
            return 1;
        }
        //
        Shape aS1 = theDI.GetShape(a[1]);
        Shape aS2 = theDI.GetShape(a[2]);
        //
        if (aS1.IsNull || aS2.IsNull)
        {
            di.Append("null shapes\n");
            return 1;
        }
        //
        if (aS1.ShapeType() != ShapeType.Edge || aS2.ShapeType() != ShapeType.Face)
        {
            di.Append("type mismatch\n");
            return 1;
        }
        //
        Edge anEdge = global::Kerf.Engine.Topology.ShapeCasts.Edge(aS1);
        Face aFace = global::Kerf.Engine.Topology.ShapeCasts.Face(aS2);
        //
        if (!AlgoTools.ComputeTolerance(aFace, anEdge, out double aMaxDist, out double aMaxPar))
        {
            di.Append("Tolerance cannot be computed\n");
            return 1;
        }
        //
        di.Append("Max Distance = ").Append(aMaxDist).Append("; Parameter on curve = ").Append(aMaxPar).Append("\n");
        //
        return 0;
    }

    // C's %3.16f
    private static string F16(double theValue) => theValue.ToString("F16", CultureInfo.InvariantCulture);

    private static int CheckCurveOnSurf(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            di.Append("use checkcurveonsurf shape\n");
            return 1;
        }
        //
        Shape aS = theDI.GetShape(a[1]);
        if (aS.IsNull)
        {
            di.Append("null shape\n");
            return 1;
        }
        //
        int nE, nF, anECounter, aFCounter;
        double aT, aTolE, aDMax;
        global::Kerf.Engine.Topology.Explorer aExpF = new(), aExpE = new();
        NCollection_DataMap<Shape, double> aDMETol = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        NCollection_DataMap<Shape, int> aMSI = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        //
        anECounter = 0;
        aFCounter = 0;
        //
        aExpF.Init(aS, ShapeType.Face);
        for (; aExpF.More(); aExpF.Next())
        {
            Face aF = global::Kerf.Engine.Topology.ShapeCasts.Face(aExpF.Current());
            //
            aExpE.Init(aF, ShapeType.Edge);
            for (; aExpE.More(); aExpE.Next())
            {
                Edge aE = global::Kerf.Engine.Topology.ShapeCasts.Edge(aExpE.Current());
                //
                if (!AlgoTools.ComputeTolerance(aF, aE, out aDMax, out aT))
                {
                    continue;
                }
                //
                aTolE = Tool.Tolerance(aE);
                if (!(aDMax > aTolE))
                {
                    continue;
                }
                //
                if (aDMETol.IsBound(aE))
                {
                    ref double aD = ref aDMETol.ChangeFind(aE);
                    if (aDMax > aD)
                    {
                        aD = aDMax;
                    }
                }
                else
                {
                    aDMETol.Bind(aE, aDMax);
                }
                //
                if (anECounter == 0)
                {
                    di.Append("Invalid curves on surface:\n");
                }
                //
                if (aMSI.IsBound(aE))
                {
                    nE = aMSI.Find(aE);
                }
                else
                {
                    nE = anECounter;
                    aMSI.Bind(aE, nE);
                    ++anECounter;
                }
                //
                if (aMSI.IsBound(aF))
                {
                    nF = aMSI.Find(aF);
                }
                else
                {
                    nF = aFCounter;
                    aMSI.Bind(aF, nF);
                    ++aFCounter;
                }
                //
                string anEName = "e_" + nE.ToString(CultureInfo.InvariantCulture);
                string aFName = "f_" + nF.ToString(CultureInfo.InvariantCulture);
                di.Append($"edge {anEName} on face {aFName} (max dist: {F16(aDMax)}, parameter on curve: {F16(aT)})\n");
                //
                theDI.SetShape(anEName, aE);
                theDI.SetShape(aFName, aF);
            }
        }
        //
        if (anECounter > 0)
        {
            di.Append("\n\nSugestions to fix the shape:\n");
            di.Append("explode ").Append(a[1]).Append(" e;\n");
            //
            NCollection_Map<Shape> M = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
            aExpE.Init(aS, ShapeType.Edge);
            for (anECounter = 0; aExpE.More(); aExpE.Next())
            {
                Shape aE = aExpE.Current();
                if (!M.Add(aE))
                {
                    continue;
                }
                ++anECounter;
                //
                if (!aDMETol.IsBound(aE))
                {
                    continue;
                }
                //
                aTolE = aDMETol.Find(aE);
                aTolE *= 1.001;
                di.Append($"settolerance {a[1]}_{anECounter.ToString(CultureInfo.InvariantCulture)} {F16(aTolE)};\n");
            }
        }
        else
        {
            di.Append("This shape seems to be OK.\n");
        }
        //
        return 0;
    }
}
