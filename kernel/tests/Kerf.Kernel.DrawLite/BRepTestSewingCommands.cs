// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing;
using Kerf.Topology;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// BRepTest_SurfaceCommands.cxx's <c>sewing</c> (BRepBuilderAPI_Sewing) and <c>fastsewing</c>
/// (BRepBuilderAPI_FastSewing), unit 6L.
/// </summary>
/// <remarks>
/// Sewing doesn't write into the shapes it is given (unit 6S copies them at the entry), so the variables keep their
/// shapes as in DRAWEXE. What OCCT prints on <c>std::cout</c> (the sewing's Dump, FastSewing's statuses) goes to the
/// interpreter's output, as DRAWEXE's log has it.
/// </remarks>
internal static class BRepTestSewingCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "sewing", Sewing);
        DrawCxx.Add(theInterpreter, "fastsewing", FastSewing);
    }

    /// <summary><c>sewing result [tolerance] shape1 shape2 ... [min tolerance] [max tolerance] [switches]</c>.</summary>
    private static int Sewing(DrawInterpreter theDI, DrawResult theDi, IReadOnlyList<string> theArgv)
    {
        int theArgc = theArgv.Count;
        var aSewing = new Sewing();
        int aPar = 1;
        var aSeq = new NCollection_Sequence<Shape>();

        double aTol = 1.0e-06;
        bool aSewingMode = true;
        bool anAnalysisMode = true;
        bool aCuttingMode = true;
        bool aNonManifoldMode = false;
        bool aSameParameterMode = true;
        bool aFloatingEdgesMode = false;
        bool aFaceMode = true;
        bool aSetMinTol = false;
        double aMinTol = 0.0;
        double aMaxTol = global::Kerf.Engine.Foundation.Precision.Infinite();

        static char At(string theWord, int theIndex) => theIndex < theWord.Length ? theWord[theIndex] : '\0';

        for (int i = 2; i < theArgc; i++)
        {
            if (At(theArgv[i], 0) == '-' || At(theArgv[i], 0) == '+')
            {
                bool aVal = At(theArgv[i], 0) == '+';
                switch (char.ToLowerInvariant(At(theArgv[i], 1)))
                {
                    case 'm':
                    {
                        if (char.ToLowerInvariant(At(theArgv[i], 2)) == 'i' && i + 1 < theArgc)
                        {
                            if (theDI.Atof(theArgv[i + 1]) != 0)
                            {
                                aMinTol = theDI.Atof(theArgv[++i]);
                                aSetMinTol = true;
                            }
                            else
                            {
                                theDi.Append("Error! min tolerance can't possess the null value\n");
                                return 1;
                            }
                        }
                        if (char.ToLowerInvariant(At(theArgv[i], 2)) == 'a' && i + 1 < theArgc)
                        {
                            if (theDI.Atof(theArgv[i + 1]) != 0)
                            {
                                aMaxTol = theDI.Atof(theArgv[++i]);
                            }
                            else
                            {
                                theDi.Append("Error! max tolerance can't possess the null value\n");
                                return 1;
                            }
                        }
                    }
                    break;
                    case 's':
                        aSewingMode = aVal;
                        break;
                    case 'a':
                        anAnalysisMode = aVal;
                        break;
                    case 'c':
                        aCuttingMode = aVal;
                        break;
                    case 'n':
                        aNonManifoldMode = aVal;
                        break;
                    case 'p':
                        aSameParameterMode = aVal;
                        break;
                    case 'e':
                        aFloatingEdgesMode = aVal;
                        break;
                    case 'f':
                        aFaceMode = aVal;
                        break;
                }
            }
            else
            {
                Shape aShape = theDI.GetShape(theArgv[i]);
                if (!aShape.IsNull)
                {
                    aSeq.Append(aShape);
                    aPar++;
                }
                else
                {
                    if (theDI.Atof(theArgv[i]) != 0)
                    {
                        aTol = theDI.Atof(theArgv[i]);
                    }
                }
            }
        }

        if (aPar < 2)
        {
            theDi.Append("Use: ").Append(theArgv[0])
                 .Append(" result [tolerance] shape1 shape2 ... [min tolerance] [max tolerance] [switches]\n");
            theDi.Append("To set user's value of min/max tolerances the following syntax is used: +<parameter> <value>\n");
            theDi.Append("- parameters are identified by letters:\n");
            theDi.Append("  mint - min tolerance\n");
            theDi.Append("  maxt - max tolerance\n");
            theDi.Append("Switches allow to tune other parameters of Sewing\n");
            theDi.Append("The following syntax is used: <symbol><parameter>\n");
            theDi.Append("- symbol may be - to set parameter off, + to set on\n");
            theDi.Append("- parameters are identified by letters:\n");
            theDi.Append("  s - mode for creating sewed shape\n");
            theDi.Append("  a - mode for analysis of input shapes\n");
            theDi.Append("  c - mode for cutting of free edges\n");
            theDi.Append("  n - mode for non manifold processing\n");
            theDi.Append("  p - mode for same parameter processing for edges\n");
            theDi.Append("  e - mode for sewing floating edges\n");
            theDi.Append("  f - mode for sewing faces\n");
            return 1;
        }

        if (!aSetMinTol)
        {
            aMinTol = aTol * 1e-4;
        }
        if (aTol < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            aTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        }
        if (aMinTol < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            aMinTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        }
        if (aMinTol > aTol)
        {
            theDi.Append("Error! min tolerance can't exceed working tolerance\n");
            return 1;
        }
        if (aMaxTol < aTol)
        {
            theDi.Append("Error! max tolerance can't be less than working tolerance\n");
            return 1;
        }

        aSewing.Init(aTol, aSewingMode, anAnalysisMode, aCuttingMode, aNonManifoldMode);
        aSewing.SetSameParameterMode(aSameParameterMode);
        aSewing.SetFloatingEdgesMode(aFloatingEdgesMode);
        aSewing.SetFaceMode(aFaceMode);
        aSewing.SetMinTolerance(aMinTol);
        aSewing.SetMaxTolerance(aMaxTol);

        for (int i = 1; i <= aSeq.Length(); i++)
        {
            aSewing.Add(aSeq.Value(i));
        }

        aSewing.Perform();
        // OCCT dumps on std::cout
        aSewing.Dump(theDI.Output);

        Shape aRes = aSewing.SewedShape();
        if (!aRes.IsNull)
        {
            theDI.SetShape(theArgv[1], aRes);
        }
        return 0;
    }

    /// <summary><c>fastsewing result [-tol &lt;value&gt;] &lt;list_of_faces&gt;</c>.</summary>
    private static int FastSewing(DrawInterpreter theDI, DrawResult theDi, IReadOnlyList<string> theArgVal)
    {
        int theNArg = theArgVal.Count;
        if (theNArg < 3)
        {
            //                0         1       2     3         4
            theDi.Append("Use: fastsewing result [-tol <value>] <list_of_faces>\n");
            return 1;
        }

        var aFS = new FastSewing();

        int aStartIndex = 2;

        if (theArgVal[aStartIndex] == "-tol")
        {
            // OCCT reads argv[3] even when it isn't given (a null pointer); Draw-lite reads an empty word
            aFS.SetTolerance(theDI.Atof(aStartIndex + 1 < theNArg ? theArgVal[aStartIndex + 1] : ""));
            aStartIndex = 4;
        }

        for (int i = aStartIndex; i < theNArg; i++)
        {
            Shape aS = theDI.GetShape(theArgVal[i]);

            if (!aFS.Add(aS))
            {
                theDi.Append("Face is not added. See statuses.\n");
            }
        }

        uint aStatus = aFS.GetStatuses();

        if (aStatus != 0)
        {
            theDi.Append("Error: There are some problems while adding (").Append((int)aStatus).Append(")\n");
            aFS.GetStatuses(theDI.Output);
        }

        aFS.Perform();

        aStatus = aFS.GetStatuses();

        if (aStatus != 0)
        {
            theDi.Append("Error: There are some problems while performing (").Append((int)aStatus).Append(")\n");
            aFS.GetStatuses(theDI.Output);
        }

        theDI.SetShape(theArgVal[1], aFS.GetResult());

        return 0;
    }
}
