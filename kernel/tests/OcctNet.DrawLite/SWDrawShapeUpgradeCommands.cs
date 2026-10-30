// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// The commands of SWDRAW_ShapeUpgrade.cxx that are ported: <c>unifysamedom</c> (ShapeUpgrade_UnifySameDomain).
/// </summary>
/// <remarks>
/// OCCT's UnifySameDomain writes into the shape it is given, even in its safe input mode (it adds the pcurves of the
/// unified faces' reference surface to the input's edges), so in DRAWEXE every variable holding the input's sub-shapes
/// sees the change. Draw-lite runs it as decision 495's in-place commands do: on an unfrozen copy of the input, which
/// the algorithm then works on in place as OCCT's does, and every variable sharing a changed TShape is rebound
/// (DrawShapeRebind); the copy's sub-shapes left unchanged are the input's again (BRepTools_CopyAtEntry's Finish), so
/// only the variables holding a changed one are rebound. The shapes to keep are taken on the copy. The history is
/// recorded in the session (BRepTest_Objects), composed with the copy's.
/// </remarks>
internal static class SWDrawShapeUpgradeCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "unifysamedom", UnifySameDom);
    }

    private static int UnifySameDom(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            di.Append("Use unifysamedom result shape [s1 s2 ...] [-f] [-e] [-nosafe] [+b] [+i] [-t val] [-a val]\n");
            di.Append("options:\n");
            di.Append("s1 s2 ... to keep the given edges during unification of faces\n");
            di.Append("-f to switch off 'unify-faces' mode \n");
            di.Append("-e to switch off 'unify-edges' mode\n");
            di.Append("-nosafe to switch off 'safe input shape' mode\n");
            di.Append("+b to switch on 'concat bspline' mode\n");
            di.Append("+i to switch on 'allow internal edges' mode\n");
            di.Append("-t val to set linear tolerance\n");
            di.Append("-a val to set angular tolerance (in degrees)\n");
            di.Append("'unify-faces' and 'unify-edges' modes are switched on by default");
            return 1;
        }

        TopoDS_Shape aShape = theDI.GetShape(a[2]);
        if (aShape.IsNull())
        {
            return 1;
        }

        // default values
        bool   anUFaces        = true;
        bool   anUEdges        = true;
        bool   anConBS         = false;
        bool   isAllowInternal = false;
        bool   isSafeInputMode = true;
        double aLinTol         = Precision.Confusion();
        double aAngTol         = Precision.Angular();
        var    aKeepShapes     = new List<TopoDS_Shape>();

        if (n > 3)
        {
            for (int i = 3; i < n; i++)
            {
                TopoDS_Shape aKeepShape = theDI.GetShape(a[i]);
                if (!aKeepShape.IsNull())
                {
                    aKeepShapes.Add(aKeepShape);
                }
                else
                {
                    if (a[i] == "-f")
                    {
                        anUFaces = false;
                    }
                    else if (a[i] == "-e")
                    {
                        anUEdges = false;
                    }
                    else if (a[i] == "-nosafe")
                    {
                        isSafeInputMode = false;
                    }
                    else if (a[i] == "+b")
                    {
                        anConBS = true;
                    }
                    else if (a[i] == "+i")
                    {
                        isAllowInternal = true;
                    }
                    else if (a[i] == "-t" || a[i] == "-a")
                    {
                        if (++i < n)
                        {
                            if (a[i - 1][1] == 't')
                            {
                                aLinTol = theDI.Atof(a[i]);
                            }
                            else
                            {
                                aAngTol = theDI.Atof(a[i]) * (Math.PI / 180.0);
                            }
                        }
                        else
                        {
                            di.Append("value expected after ").Append(a[i - 1]);
                            return 1;
                        }
                    }
                }
            }
        }

        // OCCT works on the input itself: Draw-lite on an unfrozen copy, rebound afterwards (see the class)
        var anEntry = new BRepTools_CopyAtEntry(aShape, null, true);
        var aMapOfShapes = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        foreach (TopoDS_Shape aKeepShape in aKeepShapes)
        {
            aMapOfShapes.Add(anEntry.Copied(aKeepShape));
        }

        var aUnifier = new ShapeUpgrade_UnifySameDomain();
        aUnifier.Initialize(anEntry.Shape(), anUEdges, anUFaces, anConBS);
        aUnifier.KeepShapes(aMapOfShapes);
        aUnifier.SetSafeInputMode(isSafeInputMode);
        aUnifier.AllowInternalEdges(isAllowInternal);
        aUnifier.SetLinearTolerance(aLinTol);
        aUnifier.SetAngularTolerance(aAngTol);
        aUnifier.Build();

        // the copy's sub-shapes left unchanged are the input's again, in the result and in the copy, so only the
        // variables sharing a changed sub-shape are rebound
        TopoDS_Shape aWork = anEntry.Shape();
        TopoDS_Shape Result = anEntry.Finish(aUnifier.InternalShape(), [aWork]);
        DrawShapeRebind.Rebind(theDI, aShape, anEntry.Final(aWork));
        theDI.SetShape(a[1], Result);
        // BRepTest_Objects::SetHistory(aUnifier.History()), composed with the copy so it answers for the input's shapes
        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            BRepTestHistoryCommands.SetHistory(theDI, anEntry.Composed(aUnifier.InternalHistory()));
        }
        return 0;
    }
}
