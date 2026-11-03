// Port of: src/Draw/TKXDEDRAW/XDEDRAW/XDEDRAW_Colors.cxx (colour commands), XDEDRAW_Layers.cxx,
//          XDEDRAW_Props.cxx (validation property commands)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// XDEDRAW's colour, layer and validation property commands (unit 8G1) over Draw-lite's documents. Left out
/// (decision 1259): the visualisation material commands, the instance (SHUO) colour and visibility commands
/// (<c>XGetInstanceColor</c>, <c>XSetInstanceColor</c>, <c>XGetInstanceVisible</c>), <c>XGetLayerRefs</c>,
/// <c>XSetMaterial</c>, <c>XShapeMassProps</c>, <c>XGetValProps</c>, <c>XCheckProps</c>.
/// <c>Message::SendFail</c> lines go to the interpreter's output (decision 405).
/// </summary>
internal static class XDEDRAWColorsLayersCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "XSetColor", setColor);
        DrawCxx.Add(theInterpreter, "XGetColor", getColor);
        DrawCxx.Add(theInterpreter, "XGetShapeColor", getShapeColor);
        DrawCxx.Add(theInterpreter, "XGetAllColors", getAllColors);
        DrawCxx.Add(theInterpreter, "XAddColor", addColor);
        DrawCxx.Add(theInterpreter, "XRemoveColor", removeColor);
        DrawCxx.Add(theInterpreter, "XFindColor", findColor);
        DrawCxx.Add(theInterpreter, "XUnsetColor", unsetColor);
        DrawCxx.Add(theInterpreter, "XSetObjVisibility", setObjVisibility);
        DrawCxx.Add(theInterpreter, "XGetObjVisibility", getObjVisibility);

        DrawCxx.Add(theInterpreter, "XSetLayer", setLayer);
        DrawCxx.Add(theInterpreter, "XGetLayers", getLayers);
        DrawCxx.Add(theInterpreter, "XGetOneLayer", getOneLayer);
        DrawCxx.Add(theInterpreter, "XAddLayer", addLayer);
        DrawCxx.Add(theInterpreter, "XSetLinkLayer", setLinkLayer);
        DrawCxx.Add(theInterpreter, "XGetAllLayers", getAllLayers);
        DrawCxx.Add(theInterpreter, "XUnSetLayer", unSetLayer);
        DrawCxx.Add(theInterpreter, "XUnSetAllLayers", unSetAllLayers);
        DrawCxx.Add(theInterpreter, "XRemoveLayer", removeLayer);
        DrawCxx.Add(theInterpreter, "XRemoveAllLayers", removeAllLayers);
        DrawCxx.Add(theInterpreter, "XFindLayer", findLayer);
        DrawCxx.Add(theInterpreter, "XGetLayerLabels", getLayerLabels);
        DrawCxx.Add(theInterpreter, "XSetVisibility", setLayerVisibility);
        DrawCxx.Add(theInterpreter, "XIsVisible", isLayerVisible);

        DrawCxx.Add(theInterpreter, "XSetProps", setProps);
        DrawCxx.Add(theInterpreter, "XSetVolume", setVolume);
        DrawCxx.Add(theInterpreter, "XGetVolume", getVolume);
        DrawCxx.Add(theInterpreter, "XSetArea", setArea);
        DrawCxx.Add(theInterpreter, "XGetArea", getArea);
        DrawCxx.Add(theInterpreter, "XSetCentroid", setCentroid);
        DrawCxx.Add(theInterpreter, "XGetCentroid", getCentroid);
    }

    // Message::SendFail (decision 405)
    private static int fail(DrawInterpreter theDI, string theMessage)
    {
        theDI.Output.WriteLine(theMessage);
        return 1;
    }

    private static XdeDoc_Document? failDoc(DrawInterpreter theDI, string theName)
    {
        XdeDoc_Document? aDoc = DDocStdCommands.GetDocument(theDI, theName);
        if (aDoc is null)
        {
            fail(theDI, "Syntax error: " + theName + " is not a document");
        }
        return aDoc;
    }

    private static bool parseXDocColorType(string theArg, out XdeDoc_ColorType theType)
    {
        switch (theArg.ToLowerInvariant())
        {
            case "surf" or "surface" or "s":
                theType = XdeDoc_ColorType.XdeDoc_ColorSurf;
                return true;
            case "curve" or "c":
                theType = XdeDoc_ColorType.XdeDoc_ColorCurv;
                return true;
            case "gen" or "generic":
                theType = XdeDoc_ColorType.XdeDoc_ColorGen;
                return true;
        }
        theType = XdeDoc_ColorType.XdeDoc_ColorGen;
        return false;
    }

    // the colour's name, with its alpha when not opaque
    private static void appendColor(DrawResult di, in global::Kerf.Engine.Foundation.ColorWithAlpha theColor)
    {
        di.Append(global::Kerf.Engine.Foundation.Color.StringName(theColor.GetRGB().Name()));
        if ((1.0 - theColor.Alpha()) >= global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            di.Append(" (").Append((double)theColor.Alpha()).Append(")");
        }
    }

    // ============================================================================================== colours

    private static int setColor(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 4)
        {
            return fail(theDI, "Syntax error: wrong number of arguments");
        }
        XdeDoc_Document? aDoc = failDoc(theDI, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        Shape aShape = new();
        if (aLabel.IsNull())
        {
            aShape = theDI.GetShape(argv[2]);
            if (aShape.IsNull)
            {
                return fail(theDI, "Syntax error: " + argv[2] + " is not a label nor shape");
            }
        }
        global::Kerf.Engine.Foundation.ColorWithAlpha aColor = new();
        bool isColorDefined = false;
        XdeDoc_ColorType aColType = XdeDoc_ColorType.XdeDoc_ColorGen;
        for (int anArgIter = 3; anArgIter < argc; ++anArgIter)
        {
            if (parseXDocColorType(argv[anArgIter], out XdeDoc_ColorType aType))
            {
                aColType = aType;
            }
            else if (!isColorDefined)
            {
                isColorDefined = true;
                int aNbParsed = DDocStdCommands.ParseColor(theDI, argv, anArgIter, out aColor);
                if (aNbParsed == 0)
                {
                    return fail(theDI, "Syntax error at '" + argv[anArgIter] + "'");
                }
                anArgIter += aNbParsed - 1;
            }
            else
            {
                return fail(theDI, "Syntax error at '" + argv[anArgIter] + "'");
            }
        }
        if (!isColorDefined)
        {
            return fail(theDI, "Syntax error: wrong number of arguments");
        }
        if (!aLabel.IsNull())
        {
            aDoc.SetColor(aLabel, aColor, aColType);
        }
        else if (!aDoc.SetColor(aShape, aColor, aColType))
        {
            return fail(theDI, "Syntax error: " + argv[2] + " is not a label nor shape");
        }
        return 0;
    }

    private static int getColor(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            return fail(theDI, "Syntax error: wrong number of arguments");
        }
        XdeDoc_Document? aDoc = failDoc(theDI, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        if (aDoc.GetColor(aLabel, out global::Kerf.Engine.Foundation.ColorWithAlpha aColor))
        {
            appendColor(di, aColor);
        }
        return 0;
    }

    private static int getShapeColor(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3 && argv.Count != 4)
        {
            return fail(theDI, "Syntax error: wrong number of arguments");
        }
        XdeDoc_Document? aDoc = failDoc(theDI, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        if (aLabel.IsNull())
        {
            return fail(theDI, "Syntax error: '" + argv[2] + "' label is not found in the document");
        }
        XdeDoc_ColorType aColType = XdeDoc_ColorType.XdeDoc_ColorGen;
        if (argv.Count > 3 && !parseXDocColorType(argv[3], out aColType))
        {
            return fail(theDI, "Syntax error: unknown color type '" + argv[3] + "'");
        }
        if (aDoc.GetColor(aLabel, aColType, out global::Kerf.Engine.Foundation.ColorWithAlpha aColor))
        {
            appendColor(di, aColor);
        }
        return 0;
    }

    private static int getAllColors(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 2)
        {
            return fail(theDI, "Syntax error: wrong number of arguments");
        }
        XdeDoc_Document? aDoc = failDoc(theDI, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        var aLabels = new NCollection_Sequence<XdeDoc_Label>();
        aDoc.GetColors(aLabels);
        foreach (XdeDoc_Label aLabel in aLabels)
        {
            if (!aDoc.GetColor(aLabel, out global::Kerf.Engine.Foundation.ColorWithAlpha aColor))
            {
                continue;
            }
            appendColor(di, aColor);
            di.Append(" ");
        }
        return 0;
    }

    private static int addColor(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 3)
        {
            return fail(theDI, "Syntax error: wrong number of arguments");
        }
        XdeDoc_Document? aDoc = failDoc(theDI, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        int aNbParsed = DDocStdCommands.ParseColor(theDI, argv, 2, out global::Kerf.Engine.Foundation.ColorWithAlpha aColor);
        if (aNbParsed != argv.Count - 2)
        {
            return fail(theDI, "Syntax error at '" + argv[2] + "'");
        }
        di.Append(aDoc.AddColor(aColor).Entry());
        return 0;
    }

    private static int removeColor(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            return fail(theDI, "Syntax error: wrong number of arguments");
        }
        XdeDoc_Document? aDoc = failDoc(theDI, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        if (aLabel.IsNull())
        {
            return fail(theDI, "Syntax error: " + argv[2] + " label is not found in the document");
        }
        aDoc.RemoveColor(aLabel);
        return 0;
    }

    private static int findColor(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 3)
        {
            return fail(theDI, "Syntax error: wrong number of arguments");
        }
        XdeDoc_Document? aDoc = failDoc(theDI, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        int aNbParsed = DDocStdCommands.ParseColor(theDI, argv, 2, out global::Kerf.Engine.Foundation.ColorWithAlpha aColor);
        if (aNbParsed != argv.Count - 2)
        {
            return fail(theDI, "Syntax error at '" + argv[2] + "'");
        }
        di.Append(aDoc.FindColor(aColor).Entry());
        return 0;
    }

    private static int unsetColor(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 4)
        {
            return fail(theDI, "Syntax error: wrong number of arguments");
        }
        XdeDoc_Document? aDoc = failDoc(theDI, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        if (!parseXDocColorType(argv[3], out XdeDoc_ColorType aColType))
        {
            return fail(theDI, "Syntax error: unknown color type '" + argv[3] + "'");
        }
        XdeDoc_Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        if (!aLabel.IsNull())
        {
            aDoc.UnSetColor(aLabel, aColType);
            return 0;
        }
        Shape aShape = theDI.GetShape(argv[2]);
        if (aShape.IsNull)
        {
            return fail(theDI, "Syntax error: " + argv[2] + " is not a label nor shape");
        }
        // XCAFDoc_ColorTool::UnSetColor (shape): the label found by Search
        if (aDoc.Search(aShape, out XdeDoc_Label aFound))
        {
            aDoc.UnSetColor(aFound, aColType);
        }
        return 0;
    }

    // the label of the argument, or of the shape it names (XCAFDoc_ShapeTool::FindShape with the instance)
    private static XdeDoc_Label labelOrInstance(DrawInterpreter theDI, XdeDoc_Document theDoc, string theArg)
    {
        XdeDoc_Label aLabel = DDocStdCommands.Label(theDoc, theArg);
        if (aLabel.IsNull())
        {
            Shape aShape = theDI.GetShape(theArg);
            if (!aShape.IsNull)
            {
                aLabel = theDoc.FindShape(aShape, true);
            }
        }
        return aLabel;
    }

    private static int setObjVisibility(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3 && argv.Count != 4)
        {
            return fail(theDI, "Syntax error: wrong number of arguments");
        }
        XdeDoc_Document? aDoc = failDoc(theDI, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = labelOrInstance(theDI, aDoc, argv[2]);
        if (aLabel.IsNull())
        {
            return fail(theDI, "Syntax error: " + argv[2] + " is not a label not shape");
        }
        bool isVisible = false;
        if (argv.Count == 4)
        {
            if (argv[3] == "1")
            {
                isVisible = true;
            }
            else if (argv[3] != "0")
            {
                return fail(theDI, "Syntax error: unknown argument '" + argv[3] + "'");
            }
        }
        // XCAFDoc_ColorTool::SetVisibility hides only shapes
        if (isVisible || aDoc.IsShape(aLabel))
        {
            aDoc.SetVisibility(aLabel, isVisible);
        }
        return 0;
    }

    private static int getObjVisibility(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            return fail(theDI, "Syntax error: wrong number of arguments");
        }
        XdeDoc_Document? aDoc = failDoc(theDI, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = labelOrInstance(theDI, aDoc, argv[2]);
        if (aLabel.IsNull())
        {
            return fail(theDI, "Syntax error: " + argv[2] + " is not a label not shape");
        }
        di.Append(aDoc.IsVisible(aLabel) ? 1 : 0);
        return 0;
    }

    // ============================================================================================== layers

    // the shape's label as XCAFDoc_LayerTool's shape methods find it (XCAFDoc_ShapeTool::Search)
    private static XdeDoc_Label shapeLabel(DrawInterpreter theDI, XdeDoc_Document theDoc, string theArg, out bool theIsLabel)
    {
        XdeDoc_Label aLabel = DDocStdCommands.Label(theDoc, theArg);
        theIsLabel = !aLabel.IsNull();
        if (theIsLabel)
        {
            return aLabel;
        }
        Shape aShape = theDI.GetShape(theArg);
        return !aShape.IsNull && theDoc.Search(aShape, out XdeDoc_Label aFound) ? aFound : default;
    }

    private static int addLayer(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName StringLayer \n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        di.Append(aDoc.AddLayer(argv[2]).Entry());
        return 0;
    }

    private static int findLayer(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName StringLayer \n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        di.Append(aDoc.FindLayer(argv[2]).Entry());
        return 0;
    }

    private static int removeLayer(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName {Label|string}\n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        if (aLabel.IsNull())
        {
            aLabel = aDoc.FindLayer(argv[2]);
        }
        if (aLabel.IsNull())
        {
            return 1;
        }
        aDoc.RemoveLayer(aLabel);
        return 0;
    }

    private static int setLayer(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 4)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName {Shape|Label} StringLayer [shapeInOneLayer(0/1)]\n");
            return 1;
        }
        bool shapeInOneLayer = argv.Count == 5 && theDI.Atoi(argv[4]) == 1;
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = shapeLabel(theDI, aDoc, argv[2], out bool isLabel);
        if (!aLabel.IsNull())
        {
            aDoc.SetLayer(aLabel, argv[3], shapeInOneLayer);
        }
        else if (!isLabel && !theDI.GetShape(argv[2]).IsNull)
        {
            // XCAFDoc_LayerTool::SetLayer (shape, name) adds the layer before searching the shape
            aDoc.AddLayer(argv[3]);
        }
        return 0;
    }

    private static int getLayers(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName {Shape|Label} \n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = shapeLabel(theDI, aDoc, argv[2], out _);
        var aLayerS = new NCollection_Sequence<string>();
        if (!aLabel.IsNull() && aDoc.GetLayers(aLabel, aLayerS))
        {
            foreach (string aLayer in aLayerS)
            {
                di.Append("\"").Append(aLayer).Append("\" ");
            }
        }
        return 0;
    }

    private static int getLayerLabels(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 2)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName\n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        var aLabs = new NCollection_Sequence<XdeDoc_Label>();
        aDoc.GetLayerLabels(aLabs);
        foreach (XdeDoc_Label aLabel in aLabs)
        {
            di.Append(aLabel.Entry()).Append(" ");
        }
        return 0;
    }

    private static int getOneLayer(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName LayerLabel\n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        if (!aLabel.IsNull())
        {
            aDoc.GetLayer(aLabel, out string? aLayerName);
            di.Append("\"").Append(aLayerName ?? "").Append("\"");
        }
        return 0;
    }

    private static int setLinkLayer(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 4)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName {Shape|Label} LayerL [shapeInOneLayer(0/1)]\n");
            return 1;
        }
        bool shapeInOneLayer = argv.Count == 5 && theDI.Atoi(argv[4]) == 1;
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLayerLabel = DDocStdCommands.Label(aDoc, argv[3]);
        if (!aLayerLabel.IsNull())
        {
            XdeDoc_Label aLabel = shapeLabel(theDI, aDoc, argv[2], out _);
            if (!aLabel.IsNull())
            {
                aDoc.SetLayer(aLabel, aLayerLabel, shapeInOneLayer);
            }
        }
        return 0;
    }

    private static int getAllLayers(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 2)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName\n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        var aLabs = new NCollection_Sequence<XdeDoc_Label>();
        aDoc.GetLayerLabels(aLabs);
        foreach (XdeDoc_Label aLabel in aLabs)
        {
            aDoc.GetLayer(aLabel, out string? aLayerName);
            di.Append("\"").Append(aLayerName ?? "").Append("\"").Append(" ");
        }
        return 0;
    }

    private static int unSetLayer(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 4)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName {Shape|Label} stringL\n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = shapeLabel(theDI, aDoc, argv[2], out _);
        if (!aLabel.IsNull())
        {
            aDoc.UnSetOneLayer(aLabel, argv[3]);
        }
        return 0;
    }

    private static int unSetAllLayers(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName {Shape|Label}\n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = shapeLabel(theDI, aDoc, argv[2], out _);
        if (!aLabel.IsNull())
        {
            aDoc.UnSetLayers(aLabel);
        }
        return 0;
    }

    private static int removeAllLayers(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 2)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName\n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        var aLabs = new NCollection_Sequence<XdeDoc_Label>();
        aDoc.GetLayerLabels(aLabs);
        foreach (XdeDoc_Label aLabel in aLabs)
        {
            aDoc.RemoveLayer(aLabel);
        }
        return 0;
    }

    private static XdeDoc_Label layerOf(XdeDoc_Document theDoc, string theArg)
    {
        XdeDoc_Label aLabel = DDocStdCommands.Label(theDoc, theArg);
        return aLabel.IsNull() ? theDoc.FindLayer(theArg) : aLabel;
    }

    private static int setLayerVisibility(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 3)
        {
            di.Append("Use: ").Append(argv[0]).Append("DocName {layerLable|StringLayer} [isvisible(1/0)]\n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        bool isvisible = argv.Count == 4 && theDI.Atoi(argv[3]) == 1;
        XdeDoc_Label aLabel = layerOf(aDoc, argv[2]);
        if (aLabel.IsNull())
        {
            return 1;
        }
        aDoc.SetVisibility(aLabel, isvisible);
        return 0;
    }

    private static int isLayerVisible(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName {layerLable|StringLayer}\n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = layerOf(aDoc, argv[2]);
        if (aLabel.IsNull())
        {
            return 1;
        }
        di.Append(aDoc.IsVisible(aLabel) ? 1 : 0);
        return 0;
    }

    // ============================================================================================== validation properties

    // the label of the argument, or the top-level label of the shape it names (XCAFDoc_ShapeTool::FindShape)
    private static XdeDoc_Label labelOrShape(DrawInterpreter theDI, XdeDoc_Document theDoc, string theArg)
    {
        XdeDoc_Label aLabel = DDocStdCommands.Label(theDoc, theArg);
        if (aLabel.IsNull())
        {
            Shape aShape = theDI.GetShape(theArg);
            if (!aShape.IsNull)
            {
                aLabel = theDoc.FindShape(aShape);
            }
        }
        return aLabel;
    }

    private static int setProps(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName {Shape|Label} [epsilon = 0.001]\n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        Shape aShape;
        if (aLabel.IsNull())
        {
            aShape = theDI.GetShape(argv[2]);
            if (!aShape.IsNull)
            {
                aLabel = aDoc.FindShape(aShape);
            }
        }
        else
        {
            aShape = aDoc.GetShape(aLabel);
        }
        if (!aLabel.IsNull())
        {
            double anEps = argv.Count > 3 ? theDI.Atof(argv[3]) : 0.001;
            var G = new Properties();
            MassPropertiesBuilder.VolumeProperties(aShape, G, anEps, true);
            double Vres = G.Mass();
            Point3D aPoint = G.CentreOfMass();
            aDoc.SetVolume(aLabel, Vres);
            aDoc.SetCentroid(aLabel, aPoint);
            MassPropertiesBuilder.SurfaceProperties(aShape, G, anEps);
            double Ares = G.Mass();
            aDoc.SetArea(aLabel, Ares);
            di.Append(argv[2]).Append(": Volume = ").Append(Vres).Append(", Area = ").Append(Ares)
              .Append(", Centroid is (").Append(aPoint.X()).Append(", ").Append(aPoint.Y()).Append(", ").Append(aPoint.Z()).Append(")");
        }
        return 0;
    }

    private static int setVolume(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 4)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName {Label|Shape} volume\n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        double res = 0.0;
        XdeDoc_Label aLabel = labelOrShape(theDI, aDoc, argv[2]);
        if (!aLabel.IsNull())
        {
            res = theDI.Atof(argv[3]);
            aDoc.SetVolume(aLabel, res);
        }
        di.Append(res);
        return 0;
    }

    private static int setArea(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 4)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName {Label|Shape} area\n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        double res = 0.0;
        XdeDoc_Label aLabel = labelOrShape(theDI, aDoc, argv[2]);
        if (!aLabel.IsNull())
        {
            res = theDI.Atof(argv[3]);
            aDoc.SetArea(aLabel, res);
        }
        di.Append(res);
        return 0;
    }

    private static int setCentroid(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 6)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName {Label|Shape} x y z\n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        var aPnt = new Point3D();
        XdeDoc_Label aLabel = labelOrShape(theDI, aDoc, argv[2]);
        if (!aLabel.IsNull())
        {
            aPnt.SetX(theDI.Atof(argv[3]));
            aPnt.SetY(theDI.Atof(argv[4]));
            aPnt.SetZ(theDI.Atof(argv[5]));
            aDoc.SetCentroid(aLabel, aPnt);
        }
        di.Append(theDI.Atof(argv[3])).Append(" ").Append(theDI.Atof(argv[4])).Append(" ").Append(theDI.Atof(argv[5]));
        return 0;
    }

    private static int getVolume(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName {Shape|Label}\n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = labelOrShape(theDI, aDoc, argv[2]);
        if (!aLabel.IsNull() && aDoc.GetValidationProps(aLabel)?.Volume is double aVolume)
        {
            di.Append(aVolume);
        }
        return 0;
    }

    private static int getArea(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName {Shape|Label}\n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = labelOrShape(theDI, aDoc, argv[2]);
        if (!aLabel.IsNull() && aDoc.GetValidationProps(aLabel)?.Area is double anArea)
        {
            di.Append(anArea);
        }
        return 0;
    }

    private static int getCentroid(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName {Shape|Label} \n");
            return 1;
        }
        XdeDoc_Document? aDoc = XDEDRAWCommands.Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        XdeDoc_Label aLabel = labelOrShape(theDI, aDoc, argv[2]);
        if (!aLabel.IsNull() && aDoc.GetValidationProps(aLabel)?.Centroid is Point3D aPoint)
        {
            di.Append(aPoint.X()).Append(" ").Append(aPoint.Y()).Append(" ").Append(aPoint.Z());
        }
        return 0;
    }
}
