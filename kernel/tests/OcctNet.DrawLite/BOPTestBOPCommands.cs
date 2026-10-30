// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>The commands of BOPTest_BOPCommands.cxx: <c>mkvolume</c> (unit 6G); unit 6L adds the rest.</summary>
internal static class BOPTestBOPCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "mkvolume", MkVolume);
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
        BOPAlgo_GlueEnum aGlue = aSession.Glue;
        //
        bool bToIntersect = true;
        bool bCompounds = false;
        bool bAvoidInternal = false;
        NCollection_List<TopoDS_Shape> aLS = new();
        //
        for (int i = 2; i < n; ++i)
        {
            TopoDS_Shape aS = theDI.GetShape(a[i]);
            if (!aS.IsNull())
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
            NCollection_List<TopoDS_Shape> aLSx = new();
            foreach (TopoDS_Shape aSx in aLS)
            {
                for (TopoDS_Iterator aItS = new(aSx); aItS.More(); aItS.Next())
                {
                    aLSx.Append(aItS.Value());
                }
            }
            //
            aLS.Clear();
            aLS.Assign(aLSx);
        }
        //
        BOPAlgo_MakerVolume aMV = new();
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

        if (aMV.HasErrors())
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
internal sealed class BOPTestBuilderHistory(BOPAlgo_BuilderShape theAlgo) : BRepTools_History.IAlgorithm
{
    public bool IsDeleted(in TopoDS_Shape theS) => theAlgo.IsDeleted(theS);

    public IEnumerable<TopoDS_Shape> Modified(in TopoDS_Shape theS) => theAlgo.Modified(theS);

    public IEnumerable<TopoDS_Shape> Generated(in TopoDS_Shape theS) => theAlgo.Generated(theS);
}
