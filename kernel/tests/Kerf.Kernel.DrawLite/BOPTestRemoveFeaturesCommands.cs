// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Topology;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>The commands of BOPTest_RemoveFeaturesCommands.cxx (unit 6G): <c>removefeatures</c>.</summary>
/// <remarks>
/// <c>removefeatures result shape f1 f2 ... [-parallel]</c> runs BRepAlgoAPI_Defeaturing on the shape and records its
/// history in the session (BRepTest_Objects), for <c>savehistory</c>. The input is never written, so no variable is
/// rebound.
/// </remarks>
internal static class BOPTestRemoveFeaturesCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "removefeatures", RemoveFeatures);
    }

    private static int RemoveFeatures(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int theArgc = a.Count;
        if (theArgc < 4)
        {
            return 1;
        }

        // Get the shape to remove the features from
        Shape aShape = theDI.GetShape(a[2]);
        if (aShape.IsNull)
        {
            di.Append("Error: ").Append(a[2]).Append(" is a null shape.\n");
            return 1;
        }

        Defeaturing aRF = new();
        aRF.SetShape(aShape);

        // Add faces to remove
        for (int i = 3; i < theArgc; ++i)
        {
            Shape aF = theDI.GetShape(a[i]);
            if (aF.IsNull)
            {
                if (a[i] == "-parallel")
                {
                    // enable the parallel processing mode
                    aRF.SetRunParallel(true);
                }
                else
                {
                    di.Append("Warning: ").Append(a[i]).Append(" is a null shape. Skip it.\n");
                }

                continue;
            }

            aRF.AddFaceToRemove(aF);
        }

        aRF.SetToFillHistory(BRepTestHistoryCommands.IsHistoryNeeded(theDI));
        // Perform the removal
        aRF.Build();

        // Check for the errors/warnings
        BOPTestObjects.ReportAlerts(theDI, di, aRF.GetReport());

        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            BRepTestHistoryCommands.SetHistory(theDI, aRF.History());
        }

        if (aRF.HasErrors)
        {
            return 0;
        }

        Shape aResult = aRF.Shape();
        theDI.SetShape(a[1], aResult);

        return 0;
    }
}
