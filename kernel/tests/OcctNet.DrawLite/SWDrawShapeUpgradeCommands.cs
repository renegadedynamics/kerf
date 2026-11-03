// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// The commands of SWDRAW_ShapeUpgrade.cxx that are ported: <c>unifysamedom</c> (ShapeUpgrade_UnifySameDomain); unit
/// 6L's <c>removeloc</c>, <c>copytranslate</c> and <c>reshape</c>.
/// </summary>
/// <remarks>
/// OCCT's UnifySameDomain writes into the shape it is given, even in its safe input mode (it adds the pcurves of the
/// unified faces' reference surface to the input's edges), so in DRAWEXE every variable holding the input's sub-shapes
/// sees the change. Draw-lite runs it as decision 495's in-place commands do: on an unfrozen copy of the input, which
/// the algorithm then works on in place as OCCT's does, and every variable sharing a changed TShape is rebound
/// (DrawShapeRebind); the copy's sub-shapes left unchanged are the input's again (BRepTools_CopyAtEntry's Finish), so
/// only the variables holding a changed one are rebound. The shapes to keep are taken on the copy. The history is
/// recorded in the session (BRepTest_Objects), composed with the copy's.
/// <para>
/// Unit 7C's divide and convert commands (<c>DT_*</c>, <c>splitface</c>, <c>RemoveIntWires</c>) run the same way: OCCT's
/// tools (ShapeFix_ComposeShell's edge splits, ShapeConvertToBezier's last step) and the ShapeFix::SameParameter that
/// follows them write into the input and into the result, which shares the input's sub-shapes, so Draw-lite runs the
/// tool on an unfrozen copy of the input (<see cref="SWDRAWShapeFixCommands.InPlace"/>, reading the tools' internal,
/// unfrozen results) and SameParameter on a copy of the result (<see cref="SameParameterInPlace"/>), rebinding every
/// variable that shares a changed sub-shape after each. <c>offsetcurve</c> and <c>offset2dcurve</c> make offset curves;
/// the split commands bind the pieces of a curve or surface. Nothing of the file is left out (the commented-out OCCT
/// commands aside).
/// </para>
/// </remarks>
internal static partial class SWDrawShapeUpgradeCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "unifysamedom", UnifySameDom);
        DrawCxx.Add(theInterpreter, "removeloc", RemoveLoc);
        DrawCxx.Add(theInterpreter, "copytranslate", CopyTranslate);
        DrawCxx.Add(theInterpreter, "reshape", ReShape);
        // unit 7C: the divide and convert tools
        DrawCxx.Add(theInterpreter, "DT_ShapeDivide", DT_ShapeDivide);
        DrawCxx.Add(theInterpreter, "DT_SplitAngle", DT_SplitAngle);
        DrawCxx.Add(theInterpreter, "DT_ShapeConvert", DT_ShapeConvert);
        DrawCxx.Add(theInterpreter, "DT_ShapeConvertRev", DT_ShapeConvertRev);
        DrawCxx.Add(theInterpreter, "DT_SplitCurve2d", DT_SplitCurve2d);
        DrawCxx.Add(theInterpreter, "DT_SplitCurve", DT_SplitCurve);
        DrawCxx.Add(theInterpreter, "DT_SplitSurface", DT_SplitSurface);
        DrawCxx.Add(theInterpreter, "offset2dcurve", offset2dcurve);
        DrawCxx.Add(theInterpreter, "offsetcurve", offsetcurve);
        DrawCxx.Add(theInterpreter, "splitface", splitface);
        DrawCxx.Add(theInterpreter, "DT_ToBspl", converttobspline);
        DrawCxx.Add(theInterpreter, "DT_ClosedSplit", splitclosed);
        DrawCxx.Add(theInterpreter, "DT_SplitByArea", splitarea);
        DrawCxx.Add(theInterpreter, "DT_SplitByNumber", splitbynumber);
        DrawCxx.Add(theInterpreter, "RemoveIntWires", removeinternalwires);
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
        double aLinTol         = global::Kerf.Engine.Foundation.Precision.Confusion();
        double aAngTol         = global::Kerf.Engine.Foundation.Precision.Angular();
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
            BRepTestHistoryCommands.SetHistory(theDI, ReboundHistory(aShape, anEntry, aUnifier.InternalHistory()));
        }
        return 0;
    }

    /// <summary>
    /// The history of an algorithm run on <paramref name="theEntry"/>'s copy of <paramref name="theInput"/>, answering
    /// both for the input's sub-shapes (<see cref="BRepTools_CopyAtEntry.Composed"/>) and for the changed copies that
    /// the rebound variables now hold in their place (unit 6L). In DRAWEXE a sub-shape changed in place is still the
    /// same TShape, so a variable exploded after the command (the unify_same_domain grid's end script explodes the
    /// input again) asks the history about it; here it holds the changed copy, whose entries are its copy's own: removed,
    /// modified and generated as the algorithm says, and not modified when the algorithm says nothing (natively the
    /// shape itself is in the result).
    /// </summary>
    private static BRepTools_History ReboundHistory(in TopoDS_Shape theInput, BRepTools_CopyAtEntry theEntry, BRepTools_History? theHistory)
    {
        BRepTools_History aComposed = theEntry.Composed(theHistory);
        if (theHistory is null || !theEntry.IsCopied())
        {
            return aComposed;
        }
        var aSubShapes = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theInput, aSubShapes);
        foreach (TopoDS_Shape aSub in aSubShapes)
        {
            if (!BRepTools_History.IsSupportedType(aSub))
            {
                continue;
            }
            TopoDS_Shape aCopy = theEntry.Copied(aSub);
            TopoDS_Shape aNow = theEntry.Final(aCopy);
            if (ReferenceEquals(aNow.TShape(), aSub.TShape()))
            {
                continue;
            }
            if (theHistory.IsRemoved(aCopy))
            {
                aComposed.Remove(aNow);
                continue;
            }
            foreach (TopoDS_Shape aModified in theHistory.Modified(aCopy))
            {
                TopoDS_Shape aFinal = theEntry.Final(aModified);
                if (!aFinal.IsSame(aNow))
                {
                    aComposed.AddModified(aNow, aFinal);
                }
            }
            foreach (TopoDS_Shape aGenerated in theHistory.Generated(aCopy))
            {
                aComposed.AddGenerated(aNow, theEntry.Final(aGenerated));
            }
        }
        return aComposed;
    }

    /// <summary><c>removeloc res shape [remove_level]</c>: ShapeUpgrade_RemoveLocations (unit 6L).</summary>
    private static int RemoveLoc(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 3)
        {
            di.Append("bad number of arguments. Should be:  removeloc res shape [remove_level(see ShapeEnum)]\n");
            return 1;
        }

        TopoDS_Shape aShape = theDI.GetShape(argv[2]);
        if (aShape.IsNull())
        {
            return 1;
        }
        var aRemLoc = new ShapeUpgrade_RemoveLocations();
        if (argc > 3)
        {
            aRemLoc.SetRemoveLevel((TopAbs_ShapeEnum)theDI.Atoi(argv[3]));
        }
        aRemLoc.Remove(aShape);
        TopoDS_Shape aNewShape = aRemLoc.GetResult();

        theDI.SetShape(argv[1], aNewShape);
        return 0;
    }

    /// <summary><c>copytranslate res shape dx dy dz</c>: a translated copy (BRepBuilderAPI_Transform), unit 6L.</summary>
    private static int CopyTranslate(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 6)
        {
            di.Append("bad number of arguments. Should be:  copytranslate res shape dx dy dz\n");
            return 1;
        }
        TopoDS_Shape aShape = theDI.GetShape(argv[2]);
        if (aShape.IsNull())
        {
            return 1;
        }
        double aDx = theDI.Atof(argv[3]);
        double aDy = theDI.Atof(argv[4]);
        double aDz = theDI.Atof(argv[5]);
        var aTrsf = new Transform3D();
        aTrsf.SetTranslation(new Vector3D(aDx, aDy, aDz));
        var builderTransform = new BRepBuilderAPI_Transform(aTrsf);
        builderTransform.Perform(aShape, true);
        TopoDS_Shape aNewShape = builderTransform.Shape();
        theDI.SetShape(argv[1], aNewShape);
        return 0;
    }

    /// <summary>
    /// <c>reshape result shape [-replace what with] [-remove what] [-until level]</c>: BRepTools_ReShape (unit 6L).
    /// OCCT reports its errors with Message::SendFail, which DRAWEXE's printer writes into the command's result.
    /// </summary>
    private static int ReShape(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgv)
    {
        int theArgc = theArgv.Count;
        if (theArgc < 4)
        {
            theResult.Append("Error: wrong number of arguments. Type 'help ").Append(theArgv[0]).Append("'\n");
            return 1;
        }

        TopoDS_Shape aSource = theDI.GetShape(theArgv[2]);
        if (aSource.IsNull())
        {
            theResult.Append("Error: source shape ('").Append(theArgv[2]).Append("') is null\n");
            return 1;
        }

        var aReShaper = new BRepTools_ReShape();

        TopAbs_ShapeEnum aShapeLevel = TopAbs_ShapeEnum.TopAbs_SHAPE;

        // Record the requested modifications
        for (int i = 3; i < theArgc; ++i)
        {
            string anOpt = theArgv[i].ToLowerInvariant();

            if (anOpt == "-replace")
            {
                if (theArgc - i < 3)
                {
                    theResult.Append("Error: not enough arguments for replacement\n");
                    return 1;
                }

                TopoDS_Shape aWhat = theDI.GetShape(theArgv[++i]);
                if (aWhat.IsNull())
                {
                    theResult.Append("Error: argument shape ('").Append(theArgv[i]).Append("') is null\n");
                    return 1;
                }

                TopoDS_Shape aWith = theDI.GetShape(theArgv[++i]);
                if (aWith.IsNull())
                {
                    theResult.Append("Error: replacement shape ('").Append(theArgv[i]).Append("') is null\n");
                    return 1;
                }

                aReShaper.Replace(aWhat, aWith);
            }
            else if (anOpt == "-remove")
            {
                if (theArgc - i < 2)
                {
                    theResult.Append("Error: not enough arguments for removal\n");
                    return 1;
                }

                TopoDS_Shape aWhat = theDI.GetShape(theArgv[++i]);
                if (aWhat.IsNull())
                {
                    theResult.Append("Error: shape to remove ('").Append(theArgv[i]).Append("') is null\n");
                    return 1;
                }

                aReShaper.Remove(aWhat);
            }
            else if (anOpt == "-until")
            {
                if (theArgc - i < 2)
                {
                    theResult.Append("Error: not enough arguments for level specification\n");
                    return 1;
                }

                string aLevelStr = theArgv[++i].ToLowerInvariant();
                if (aLevelStr == "compound" || aLevelStr == "cd")
                {
                    aShapeLevel = TopAbs_ShapeEnum.TopAbs_COMPOUND;
                }
                else if (aLevelStr == "compsolid" || aLevelStr == "c")
                {
                    aShapeLevel = TopAbs_ShapeEnum.TopAbs_COMPSOLID;
                }
                else if (aLevelStr == "solid" || aLevelStr == "so")
                {
                    aShapeLevel = TopAbs_ShapeEnum.TopAbs_SOLID;
                }
                else if (aLevelStr == "shell" || aLevelStr == "sh")
                {
                    aShapeLevel = TopAbs_ShapeEnum.TopAbs_SHELL;
                }
                else if (aLevelStr == "face" || aLevelStr == "f")
                {
                    aShapeLevel = TopAbs_ShapeEnum.TopAbs_FACE;
                }
                else if (aLevelStr == "wire" || aLevelStr == "w")
                {
                    aShapeLevel = TopAbs_ShapeEnum.TopAbs_WIRE;
                }
                else if (aLevelStr == "edge" || aLevelStr == "e")
                {
                    aShapeLevel = TopAbs_ShapeEnum.TopAbs_EDGE;
                }
                else if (aLevelStr == "vertex" || aLevelStr == "v")
                {
                    aShapeLevel = TopAbs_ShapeEnum.TopAbs_VERTEX;
                }
                else if (aLevelStr == "shape" || aLevelStr == "s")
                {
                    aShapeLevel = TopAbs_ShapeEnum.TopAbs_SHAPE;
                }
                else
                {
                    theResult.Append("Error: unknown shape type '").Append(theArgv[i]).Append("'\n");
                    return 1;
                }
            }
            else
            {
                theResult.Append("Error: invalid syntax at ").Append(anOpt).Append("\n");
                return 1;
            }
        }

        // Apply all the recorded modifications
        TopoDS_Shape aResult = aReShaper.Apply(aSource, aShapeLevel);
        if (aResult.IsNull())
        {
            theResult.Append("Error: result shape is null\n");
            return 1;
        }

        theDI.SetShape(theArgv[1], aResult);
        return 0;
    }
}
