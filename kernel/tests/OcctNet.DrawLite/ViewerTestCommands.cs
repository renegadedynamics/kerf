// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// ViewerTest's display commands, for what they do to shapes: AIS meshes the shapes it displays (decision 495).
/// <c>vdisplay</c> computes the shape's presentation in its display mode and its selection; the shaded mode (1)
/// meshes it as StdPrs_ShadedShape does (StdPrs_ToolTriangulatedShape::Tessellate with the object's drawer, linked to
/// the context's default drawer), and the selection meshes a shape without a complete mesh as
/// StdSelect_BRepSelectionTool::Load does (BRepMesh_IncrementalMesh with the same deflection). <c>vsetdispmode</c>
/// recomputes the displayed objects in the new mode, <c>vdefaults</c> sets the default drawer's deflections and
/// automatic triangulation, and <c>vinit</c>, <c>vclose</c>, <c>verase</c>, <c>vremove</c>, <c>vclear</c> and
/// <c>vdisplayall</c> keep the context and its objects as AIS does.
/// </summary>
/// <remarks>
/// <para>
/// Nothing is drawn: the presentations and selections are only the meshing they do. The meshes are written into the
/// shapes as in DRAWEXE, on a copy that every variable sharing the shape's sub-shapes is rebound to
/// (<see cref="DrawShapeRebind"/>).
/// </para>
/// <para>
/// The context holds only shapes. A name that doesn't hold a shape (an AIS object other commands would make, which
/// Draw-lite ignores) is skipped without OCCT's "object ... does not exist" error. vinit and vclose, which return a
/// view's name in DRAWEXE, return nothing and aren't logged, as before.
/// </para>
/// </remarks>
internal static class ViewerTestCommands
{
    private const int AIS_WireFrame = 0;
    private const int AIS_Shaded = 1;

    /// <summary>An AIS_Shape: the shape, its drawer and what AIS has computed of it.</summary>
    private sealed class AisShape(TopoDS_Shape theShape, Prs3d_Drawer theDrawer)
    {
        public TopoDS_Shape Shape = theShape;
        public readonly Prs3d_Drawer Drawer = theDrawer;
        public int OwnDisplayMode = -1;
        public bool IsDisplayed;
        public bool IsErased;
        public int DisplayMode;
        public readonly HashSet<int> ComputedModes = [];
        public bool HasSelection;

        public bool HasDisplayMode => OwnDisplayMode != -1;

        // AIS_Shape::AcceptDisplayMode
        public static bool AcceptDisplayMode(int theMode) => theMode is >= 0 and <= 2;
    }

    /// <summary>The AIS_InteractiveContext of the views, and ViewerTest's map of displayed objects by name.</summary>
    private sealed class AisContext
    {
        public readonly Prs3d_Drawer DefaultDrawer = new();
        public int DisplayMode = AIS_WireFrame;
        public readonly Dictionary<string, AisShape> Objects = new(StringComparer.Ordinal);
        public int NbViews;

        public AisContext() => DefaultDrawer.SetupOwnDefaults();
    }

    private static readonly ConditionalWeakTable<DrawInterpreter, StrongBox<AisContext?>> s_contexts = new();

    private static AisContext? Context(DrawInterpreter theDI)
        => s_contexts.TryGetValue(theDI, out StrongBox<AisContext?>? aBox) ? aBox.Value : null;

    private static AisContext ViewerInit(DrawInterpreter theDI)
    {
        StrongBox<AisContext?> aBox = s_contexts.GetOrCreateValue(theDI);
        aBox.Value ??= new AisContext();
        return aBox.Value;
    }

    public static void Register(DrawInterpreter theInterpreter)
    {
        theInterpreter.Register("vinit", static (i, _) =>
        {
            ViewerInit(i).NbViews++;
            return "";
        });
        theInterpreter.Register("vclose", static (i, a) =>
        {
            if (Context(i) is { } aContext)
            {
                bool isAll = a.Skip(1).Any(w => w is "ALL" or "*");
                aContext.NbViews = isAll ? 0 : aContext.NbViews - 1;
                if (aContext.NbViews <= 0)
                {
                    // the last view closed removes the context and ViewerTest's objects
                    s_contexts.GetOrCreateValue(i).Value = null;
                }
            }

            return "";
        });
        DrawCxx.Add(theInterpreter, "vdisplay", VDisplay);
        DrawCxx.Add(theInterpreter, "vdisplayall", VDisplayAll);
        DrawCxx.Add(theInterpreter, "vsetdispmode", VDispMode);
        DrawCxx.Add(theInterpreter, "vunsetdispmode", VDispMode);
        DrawCxx.Add(theInterpreter, "vdefaults", VDefaults);
        DrawCxx.Add(theInterpreter, "verase", VErase);
        DrawCxx.Add(theInterpreter, "vremove", VRemove);
        DrawCxx.Add(theInterpreter, "vclear", VClear);
    }

    /// <summary>Gives the shapes the context holds their rebound form (DrawShapeRebind).</summary>
    internal static void RebindDisplayed(DrawInterpreter theDI, Func<TopoDS_Shape, TopoDS_Shape> theRebind)
    {
        if (Context(theDI) is { } aContext)
        {
            foreach (AisShape anObject in aContext.Objects.Values)
            {
                anObject.Shape = theRebind(anObject.Shape);
            }
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // AIS

    private static bool IsEmpty(in TopoDS_Shape theShape)
        => theShape.IsNull() || (theShape.ShapeType() == TopAbs_ShapeEnum.TopAbs_COMPOUND && theShape.NbChildren() == 0);

    /// <summary>AIS_Shape::Compute, for its meshing: the shaded mode tessellates the shape (StdPrs_ShadedShape::Add).</summary>
    private static void Compute(DrawInterpreter theDI, AisShape theObject, int theMode)
    {
        TopoDS_Shape aShape = theObject.Shape;
        if (IsEmpty(aShape) || theMode != AIS_Shaded || (int)aShape.ShapeType() > 4 || !theObject.Drawer.IsAutoTriangulation())
        {
            // the wireframe and bounding box modes, and wires, edges and vertices, draw without meshing
            return;
        }

        try
        {
            if (StdPrs_ToolTriangulatedShape.Tessellate(aShape, theObject.Drawer, out TopoDS_Shape aMeshed, out _))
            {
                DrawShapeRebind.Rebind(theDI, aShape, aMeshed);
            }
        }
        catch (Standard_Failure anException)
        {
            theDI.Output.WriteLine("Error: AIS_Shape::Compute() shaded presentation builder has failed ("
                                   + anException.GetMessageString() + ")");
        }
    }

    /// <summary>
    /// AIS_Shape::ComputeSelection, for its meshing: StdSelect_BRepSelectionTool::Load meshes a shape that misses a
    /// triangulation or a polygon (BRepMesh_IncrementalMesh, serial, with the presentation's deflection).
    /// </summary>
    private static void ComputeSelection(DrawInterpreter theDI, AisShape theObject)
    {
        TopoDS_Shape aShape = theObject.Shape;
        if (IsEmpty(aShape))
        {
            return;
        }

        try
        {
            double aDeflection = StdPrs_ToolTriangulatedShape.GetDeflection(aShape, theObject.Drawer);
            if (theObject.Drawer.IsAutoTriangulation() && !BRepTools.Triangulation(aShape, Precision.Infinite(), true))
            {
                var aMesher = new BRepMesh_IncrementalMesh(aShape, aDeflection, false, theObject.Drawer.DeviationAngle());
                DrawShapeRebind.Rebind(theDI, aShape, aMesher.Shape());
            }
        }
        catch (Standard_Failure anException)
        {
            theDI.Output.WriteLine("Error: AIS_Shape::ComputeSelection(0) has failed (" + anException.GetMessageString() + ")");
        }
    }

    /// <summary>AIS_InteractiveContext::Display(object, mode, selection mode, status).</summary>
    private static void Display(DrawInterpreter theDI, AisShape theObject, int theDispMode, int theSelMode, bool theIsErased)
    {
        if (theIsErased)
        {
            // loaded hidden: nothing computed until displayed
            theObject.IsDisplayed = false;
            theObject.IsErased = true;
            theObject.DisplayMode = theDispMode;
            return;
        }

        theObject.IsDisplayed = true;
        theObject.IsErased = false;
        theObject.DisplayMode = theDispMode;
        if (theObject.ComputedModes.Add(theDispMode))
        {
            Compute(theDI, theObject, theDispMode);
        }

        if (theSelMode != -1 && !theObject.HasSelection)
        {
            theObject.HasSelection = true;
            ComputeSelection(theDI, theObject);
        }
    }

    /// <summary>AIS_InteractiveContext::Redisplay: the presentations and selection are computed again.</summary>
    private static void Redisplay(DrawInterpreter theDI, AisShape theObject)
    {
        bool hadSelection = theObject.HasSelection;
        theObject.ComputedModes.Clear();
        theObject.HasSelection = false;
        if (theObject.IsDisplayed)
        {
            theObject.ComputedModes.Add(theObject.DisplayMode);
            Compute(theDI, theObject, theObject.DisplayMode);
        }

        if (hadSelection)
        {
            theObject.HasSelection = true;
            ComputeSelection(theDI, theObject);
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // ViewerTest.cxx

    /// <summary>
    /// vdisplay name... [options]: displays the shapes (VDisplay2). A name already displayed is updated to its
    /// variable's shape (redisplayed if it changed) and echoed ("Display name").
    /// </summary>
    private static int VDisplay(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgVec)
    {
        int theArgNb = theArgVec.Count;
        if (theArgNb < 2)
        {
            theDI.Output.WriteLine("Syntax error: wrong number of arguments.");
            return 1;
        }

        if (theArgNb == 2 && theArgVec[1] == "*")
        {
            // alias
            return VDisplayAll(theDI, di, [theArgVec[0]]);
        }

        AisContext aCtx = Context(theDI) ?? ViewerInit(theDI);
        if (aCtx.NbViews == 0)
        {
            aCtx.NbViews = 1;
        }

        bool toReDisplay = false;
        int isSelectable = -1;
        int anObjDispMode = -2;
        bool toEcho = true;
        int isAutoTriang = -1;
        bool isErased = false;
        var aNamesOfDisplayIO = new List<string>();
        for (int anArgIter = 1; anArgIter < theArgNb; ++anArgIter)
        {
            string aName = theArgVec[anArgIter];
            string aNameCase = aName.ToLowerInvariant();
            switch (aNameCase)
            {
                case "-update" or "-redraw" or "-noupdate" or "-noredraw":
                case "-mutable" or "-neutral" or "-immediate" or "-top" or "-topmost" or "-osd" or "-toposd" or "-overlay"
                    or "-botosd" or "-underlay" or "-3d" or "-view" or "-inview":
                    break;
                case "-select" or "-selectable":
                    isSelectable = 1;
                    break;
                case "-noselect" or "-noselection":
                    isSelectable = 0;
                    break;
                case "-dispmode" or "-displaymode":
                    if (++anArgIter >= theArgNb)
                    {
                        theDI.Output.WriteLine("Error: wrong syntax at " + aName + ".");
                        return 1;
                    }

                    anObjDispMode = theDI.Atoi(theArgVec[anArgIter]);
                    break;
                case "-himode" or "-highmode" or "-highlightmode" or "-trsfpers" or "-pers" or "-layer" or "-zlayer":
                    if (++anArgIter >= theArgNb)
                    {
                        theDI.Output.WriteLine("Error: wrong syntax at " + aName + ".");
                        return 1;
                    }

                    break;
                case "-2d" or "-trihedron" or "-triedron":
                    // an optional corner, and an optional offset of two integers after it
                    if (anArgIter + 1 < theArgNb && IsCorner(theArgVec[anArgIter + 1]))
                    {
                        ++anArgIter;
                        if (anArgIter + 2 < theArgNb && int.TryParse(theArgVec[anArgIter + 1], out _) && int.TryParse(theArgVec[anArgIter + 2], out _))
                        {
                            anArgIter += 2;
                        }
                    }

                    break;
                case "-trsfperspos" or "-perspos":
                    if (anArgIter + 2 >= theArgNb)
                    {
                        theDI.Output.WriteLine("Error: wrong syntax at " + aName + ".");
                        return 1;
                    }

                    anArgIter += 2;
                    if (anArgIter + 1 < theArgNb && theDI.ParseReal(theArgVec[anArgIter + 1], out _))
                    {
                        ++anArgIter;
                    }

                    break;
                case "-trsfpersortho":
                    break;
                case "-redisplay":
                    toReDisplay = true;
                    break;
                case "-autotr" or "-autotrian" or "-autotriang" or "-autotriangulation" or "-noautotr" or "-noautotrian"
                    or "-noautotriang" or "-noautotriangulation":
                    isAutoTriang = DBRepCommands.ParseOnOffNoIterator(theArgVec, ref anArgIter) ? 1 : 0;
                    break;
                case "-erased" or "-load":
                    isErased = true;
                    break;
                case "-noecho":
                    toEcho = false;
                    break;
                default:
                    aNamesOfDisplayIO.Add(aName);
                    break;
            }
        }

        if (aNamesOfDisplayIO.Count == 0)
        {
            theDI.Output.WriteLine("Syntax error: wrong number of arguments.");
            return 1;
        }

        // Display interactive objects
        foreach (string aName in aNamesOfDisplayIO)
        {
            if (!aCtx.Objects.TryGetValue(aName, out AisShape? aShape))
            {
                // create the AIS_Shape from a name
                TopoDS_Shape aDrawShape = theDI.GetShape(aName);
                if (aDrawShape.IsNull())
                {
                    // OCCT: "Error: object with name ... does not exist!", or another AIS object (decision 560)
                    continue;
                }

                var aDrawer = new Prs3d_Drawer();
                aDrawer.SetLink(aCtx.DefaultDrawer);
                aShape = new AisShape(aDrawShape, aDrawer);
                if (isAutoTriang != -1)
                {
                    aShape.Drawer.SetAutoTriangulation(isAutoTriang == 1);
                }

                if (anObjDispMode != -2)
                {
                    if (anObjDispMode == -1)
                    {
                        aShape.OwnDisplayMode = -1;
                    }

                    if (!AisShape.AcceptDisplayMode(anObjDispMode))
                    {
                        theDI.Output.WriteLine("Syntax error: AIS_Shape rejects " + anObjDispMode + " display mode");
                        return 1;
                    }

                    aShape.OwnDisplayMode = anObjDispMode;
                }

                aCtx.Objects[aName] = aShape;
                int aDispMode = aShape.HasDisplayMode ? aShape.OwnDisplayMode
                              : AisShape.AcceptDisplayMode(aCtx.DisplayMode) ? aCtx.DisplayMode : 0;
                int aSelMode = isSelectable == 1 || isSelectable == -1 ? 0 : -1;
                Display(theDI, aShape, aDispMode, aSelMode, isErased);
                continue;
            }

            if (isAutoTriang != -1)
            {
                aShape.Drawer.SetAutoTriangulation(isAutoTriang == 1);
            }

            if (anObjDispMode != -2)
            {
                aShape.OwnDisplayMode = anObjDispMode;
            }

            int aDispMode2 = aShape.HasDisplayMode ? aShape.OwnDisplayMode
                           : AisShape.AcceptDisplayMode(aCtx.DisplayMode) ? aCtx.DisplayMode : 0;
            int aSelMode2 = isSelectable == 1 || isSelectable == -1 ? 0 : -1;
            if (toEcho)
            {
                di.Append("Display ").Append(aName).Append("\n");
            }

            // update the Shape in the AIS_Shape
            TopoDS_Shape aNewShape = theDI.GetShape(aName);
            bool toReDisplayThis = toReDisplay;
            if (!aShape.Shape.IsEqual(aNewShape))
            {
                toReDisplayThis = true;
            }

            aShape.Shape = aNewShape;
            if (toReDisplayThis)
            {
                Redisplay(theDI, aShape);
            }

            if (aSelMode2 == -1)
            {
                aShape.IsDisplayed = false;
            }

            Display(theDI, aShape, aDispMode2, aSelMode2, isErased);
        }

        return 0;
    }

    /// <summary>ViewerTest::ParseCorner's keywords.</summary>
    private static bool IsCorner(string theArg)
    {
        string aCase = theArg.ToLowerInvariant();
        return aCase.Contains("top") || aCase.Contains("bottom") || aCase.Contains("upper") || aCase.Contains("lower")
               || aCase.Contains("left") || aCase.Contains("right") || aCase.Contains("center");
    }

    /// <summary>vdisplayall: displays the erased objects.</summary>
    private static int VDisplayAll(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgVec)
    {
        if (Context(theDI) is not { } aCtx)
        {
            theDI.Output.WriteLine("Error: no active viewer");
            return 1;
        }

        foreach (AisShape aShape in aCtx.Objects.Values)
        {
            if (aShape.IsErased)
            {
                int aDispMode = aShape.HasDisplayMode ? aShape.OwnDisplayMode
                              : AisShape.AcceptDisplayMode(aCtx.DisplayMode) ? aCtx.DisplayMode : 0;
                Display(theDI, aShape, aDispMode, 0, false);
            }
        }

        return 0;
    }

    /// <summary>
    /// vsetdispmode [name] mode, vunsetdispmode [name]: the display mode of the context (the objects without their
    /// own) or of one object; displayed objects are computed in the new mode.
    /// </summary>
    private static int VDispMode(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 1 || argc > 3)
        {
            theDI.Output.WriteLine("Syntax error: wrong number of arguments");
            return 1;
        }

        bool isUnset = argv[0] == "vunsetdispmode";
        if (Context(theDI) is not { } aCtx)
        {
            theDI.Output.WriteLine("Error: no active viewer");
            return 1;
        }

        if (isUnset)
        {
            IEnumerable<AisShape> anObjects = argc == 1 ? aCtx.Objects.Values
                                            : aCtx.Objects.TryGetValue(argv[1], out AisShape? aPrs) ? [aPrs] : [];
            foreach (AisShape anObject in anObjects)
            {
                // AIS_InteractiveContext::UnsetDisplayMode
                anObject.OwnDisplayMode = -1;
                if (anObject.IsDisplayed)
                {
                    SetDisplayMode(theDI, anObject, aCtx.DisplayMode);
                }
            }
        }
        else if (argc == 2)
        {
            int aDispMode = theDI.Atoi(argv[1]);
            // AIS_InteractiveContext::SetDisplayMode(mode)
            if (aDispMode != aCtx.DisplayMode)
            {
                foreach (AisShape anObject in aCtx.Objects.Values)
                {
                    if (anObject.HasDisplayMode || !AisShape.AcceptDisplayMode(aDispMode))
                    {
                        continue;
                    }

                    if (anObject.IsDisplayed)
                    {
                        SetDisplayMode(theDI, anObject, aDispMode);
                    }
                }

                aCtx.DisplayMode = aDispMode;
            }
        }
        else if (aCtx.Objects.TryGetValue(argv[1], out AisShape? aPrs))
        {
            // AIS_InteractiveContext::SetDisplayMode(object, mode)
            int aDispMode = theDI.Atoi(argv[2]);
            if (AisShape.AcceptDisplayMode(aDispMode))
            {
                if (aPrs.IsDisplayed)
                {
                    SetDisplayMode(theDI, aPrs, aDispMode);
                }

                aPrs.OwnDisplayMode = aDispMode;
            }
        }

        return 0;
    }

    private static void SetDisplayMode(DrawInterpreter theDI, AisShape theObject, int theMode)
    {
        theObject.DisplayMode = theMode;
        if (theObject.ComputedModes.Add(theMode))
        {
            Compute(theDI, theObject, theMode);
        }
    }

    /// <summary>verase [name...]: hides the objects (all displayed ones without names).</summary>
    private static int VErase(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgVec)
    {
        if (Context(theDI) is not { } aCtx)
        {
            theDI.Output.WriteLine("Error: no active viewer");
            return 1;
        }

        List<string> aNames = theArgVec.Skip(1).Where(w => !w.StartsWith('-')).ToList();
        foreach ((string aName, AisShape anObject) in aCtx.Objects)
        {
            if ((aNames.Count == 0 && anObject.IsDisplayed) || aNames.Contains(aName))
            {
                anObject.IsDisplayed = false;
                anObject.IsErased = true;
            }
        }

        return 0;
    }

    /// <summary>vremove name... | -all: removes the objects from the context.</summary>
    private static int VRemove(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgVec)
    {
        if (Context(theDI) is not { } aCtx)
        {
            theDI.Output.WriteLine("Error: no active viewer");
            return 1;
        }

        if (theArgVec.Skip(1).Any(w => w.Equals("-all", StringComparison.OrdinalIgnoreCase)))
        {
            aCtx.Objects.Clear();
            return 0;
        }

        foreach (string aName in theArgVec.Skip(1))
        {
            aCtx.Objects.Remove(aName);
        }

        return 0;
    }

    /// <summary>vclear: removes every object from the context.</summary>
    private static int VClear(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgVec)
    {
        Context(theDI)?.Objects.Clear();
        return 0;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // ViewerTest_ViewerCommands.cxx

    /// <summary>
    /// vdefaults [-absDefl value] [-devCoeff value] [-angDefl degrees] [-autoTriang {on|off}]: the context's default
    /// drawer; without arguments, prints it.
    /// </summary>
    private static int VDefaults(DrawInterpreter theDI, DrawResult theDi, IReadOnlyList<string> theArgVec)
    {
        if (Context(theDI) is not { } aCtx)
        {
            theDI.Output.WriteLine("Error: no active viewer");
            return 1;
        }

        Prs3d_Drawer aDefParams = aCtx.DefaultDrawer;
        int theArgsNb = theArgVec.Count;
        if (theArgsNb < 2)
        {
            if (aDefParams.TypeOfDeflection() == Aspect_TypeOfDeflection.Aspect_TOD_RELATIVE)
            {
                theDi.Append("DeflType:           relative\n")
                     .Append("DeviationCoeff:     ").Append(aDefParams.DeviationCoefficient()).Append("\n");
            }
            else
            {
                theDi.Append("DeflType:           absolute\n")
                     .Append("AbsoluteDeflection: ").Append(aDefParams.MaximalChordialDeviation()).Append("\n");
            }

            theDi.Append("AngularDeflection:  ").Append(180.0 * aDefParams.DeviationAngle() / Math.PI).Append("\n");
            theDi.Append("AutoTriangulation:  ").Append(aDefParams.IsAutoTriangulation() ? "on" : "off").Append("\n");
            return 0;
        }

        for (int anArgIter = 1; anArgIter < theArgsNb; ++anArgIter)
        {
            string anArg = theArgVec[anArgIter].ToUpperInvariant();
            if (anArg is "-ABSDEFL" or "-ABSOLUTEDEFLECTION" or "-DEFL" or "-DEFLECTION")
            {
                if (++anArgIter >= theArgsNb)
                {
                    theDI.Output.WriteLine("Syntax error at " + anArg);
                    return 1;
                }

                aDefParams.SetTypeOfDeflection(Aspect_TypeOfDeflection.Aspect_TOD_ABSOLUTE);
                aDefParams.SetMaximalChordialDeviation(theDI.Atof(theArgVec[anArgIter]));
            }
            else if (anArg is "-RELDEFL" or "-RELATIVEDEFLECTION" or "-DEVCOEFF" or "-DEVIATIONCOEFF" or "-DEVIATIONCOEFFICIENT")
            {
                if (++anArgIter >= theArgsNb)
                {
                    theDI.Output.WriteLine("Syntax error at " + anArg);
                    return 1;
                }

                aDefParams.SetTypeOfDeflection(Aspect_TypeOfDeflection.Aspect_TOD_RELATIVE);
                aDefParams.SetDeviationCoefficient(theDI.Atof(theArgVec[anArgIter]));
            }
            else if (anArg is "-ANGDEFL" or "-ANGULARDEFL" or "-ANGULARDEFLECTION")
            {
                if (++anArgIter >= theArgsNb)
                {
                    theDI.Output.WriteLine("Syntax error at " + anArg);
                    return 1;
                }

                aDefParams.SetDeviationAngle(Math.PI * theDI.Atof(theArgVec[anArgIter]) / 180.0);
            }
            else if (anArg is "-AUTOTR" or "-AUTOTRIANG" or "-AUTOTRIANGULATION")
            {
                ++anArgIter;
                bool toTurnOn = true;
                if (anArgIter >= theArgsNb || !DBRepCommands.ParseOnOff(theArgVec[anArgIter], ref toTurnOn))
                {
                    theDI.Output.WriteLine("Syntax error at '" + anArg + "'");
                    return 1;
                }

                aDefParams.SetAutoTriangulation(toTurnOn);
            }
            else
            {
                theDI.Output.WriteLine("Syntax error: unknown argument '" + anArg + "'");
                return 1;
            }
        }

        return 0;
    }
}
