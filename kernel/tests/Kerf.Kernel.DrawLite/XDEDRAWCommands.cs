// Port of: src/Draw/TKXDEDRAW/XDEDRAW/XDEDRAW.cxx (general commands), XDEDRAW_Shapes.cxx (shape commands)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.Text;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// XDEDRAW's general and shape commands (unit 8G1) over Draw-lite's documents (<see cref="DrawDocument"/>, an
/// <see cref="ExchangeDocument"/> under a name): XCAF's tools are the document's methods of the same names
/// (decision 1231). Left out (decision 1259): <c>XSave</c>, <c>XOpen</c> (OCAF storage), <c>XShow</c>,
/// <c>XWdump</c>, <c>XSetPrs</c>, <c>XShowFaceBoundary</c>, <c>XSetTransparency</c>, <c>XAttributeValue</c>,
/// <c>XTestDoc</c>, <c>XRescaleGeometry</c> (XCAFDoc_Editor), the SHUO commands, <c>XSetLocation</c>,
/// <c>XGetProperties</c>; <c>XDisplay</c> is a stub, as Draw-lite has no viewer.
/// </summary>
internal static class XDEDRAWCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "XNewDoc", newDoc);
        DrawCxx.Add(theInterpreter, "Xdump", dump);
        DrawCxx.Add(theInterpreter, "XStat", statdoc);
        DrawCxx.Add(theInterpreter, "XSetLengthUnit", setLengthUnit);
        DrawCxx.Add(theInterpreter, "XGetLengthUnit", dumpLengthUnit);
        DrawCxx.Add(theInterpreter, "XDumpAssemblyTree", XDEDRAWAssemblyCommands.XDumpAssemblyTree);
        DrawCxx.Add(theInterpreter, "XDumpAssemblyGraph", XDEDRAWAssemblyCommands.XDumpAssemblyGraph);
        DrawCxx.Add(theInterpreter, "XDumpNomenclature", XDEDRAWAssemblyCommands.XDumpNomenclature);
        // XDisplay displays the document's shapes in a viewer, which Draw-lite hasn't: it only checks the document
        DrawCxx.Add(theInterpreter, "XDisplay", static (_, _, _) => 0);

        DrawCxx.Add(theInterpreter, "XAddShape", addShape);
        DrawCxx.Add(theInterpreter, "XNewShape", newShape);
        DrawCxx.Add(theInterpreter, "XSetShape", setShape);
        DrawCxx.Add(theInterpreter, "XGetShape", getShape);
        DrawCxx.Add(theInterpreter, "XRemoveShape", removeShape);
        DrawCxx.Add(theInterpreter, "XFindShape", findShape);
        DrawCxx.Add(theInterpreter, "XFindSubShape", findSubShape);
        DrawCxx.Add(theInterpreter, "XFindMainShape", findMainShape);
        DrawCxx.Add(theInterpreter, "XAddSubShape", addSubShape);
        DrawCxx.Add(theInterpreter, "XLabelInfo", labelInfo);
        DrawCxx.Add(theInterpreter, "XGetUsers", getUsers);
        DrawCxx.Add(theInterpreter, "XNbComponents", nbComponents);
        DrawCxx.Add(theInterpreter, "XAddComponent", addComponent);
        DrawCxx.Add(theInterpreter, "XRemoveComponent", removeComponent);
        DrawCxx.Add(theInterpreter, "XGetReferredShape", getReferredShape);
        DrawCxx.Add(theInterpreter, "XGetTopLevelShapes", getTopLevelShapes);
        DrawCxx.Add(theInterpreter, "XGetFreeShapes", getFreeShapes);
        DrawCxx.Add(theInterpreter, "XGetOneShape", getOneShape);
        DrawCxx.Add(theInterpreter, "XDumpLocation", XDumpLocation);
        DrawCxx.Add(theInterpreter, "XUpdateAssemblies", updateAssemblies);
        DrawCxx.Add(theInterpreter, "XAutoNaming", XAutoNaming);
    }

    // DDocStd::GetDocument (complaining) and the command's own message
    internal static ExchangeDocument? Doc(DrawInterpreter theDI, DrawResult di, string theName)
    {
        ExchangeDocument? aDoc = DDocStdCommands.GetDocument(theDI, theName);
        if (aDoc is null)
        {
            di.Append(theName).Append(" is not a document\n");
        }
        return aDoc;
    }

    // ============================================================================================== general commands

    private static int newDoc(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 2)
        {
            di.Append("Give document name\n");
            return 1;
        }
        if (DDocStdCommands.GetDocument(theDI, argv[1], false) is null)
        {
            DDocStdCommands.NewDocument(theDI, argv[1]);
            di.Append("document ").Append(argv[1]).Append(" created\n");
        }
        else
        {
            di.Append(argv[1]).Append(" is already a document\n");
        }
        return 0;
    }

    private static int dump(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 2)
        {
            di.Append("Use: ").Append(argv[0]).Append(" Doc [int deep (0/1)]\n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        bool deep = argv.Count == 3 && theDI.Atoi(argv[2]) == 1;
        di.Append(Dump(aDoc, deep));
        return 0;
    }

    /// <summary>XCAFDoc_ShapeTool::Dump (deep: with the shapes' TShape and location, by identity here).</summary>
    internal static string Dump(ExchangeDocument theDoc, bool deep)
    {
        var aLog = new StringBuilder();
        var aLabels = new NCollection_Sequence<Label>();
        theDoc.GetShapes(aLabels);
        if (aLabels.Length() > 0)
        {
            aLog.Append('\n');
        }
        foreach (Label aLabel in aLabels)
        {
            dumpAssembly(theDoc, aLog, aLabel, 0, deep);
        }
        theDoc.GetFreeShapes(aLabels);
        aLog.Append('\n').Append("Free Shapes: ").Append(aLabels.Length().ToString(CultureInfo.InvariantCulture)).Append('\n');
        foreach (Label aLabel in aLabels)
        {
            dumpShape(theDoc, aLog, aLabel, 0, deep);
            aLog.Append('\n');
        }
        return aLog.ToString();
    }

    // L.Father().Father().Father().IsRoot(): a top-level label 0:1:1:n
    private static bool isTopLevelEntry(in Label theLabel) => theLabel.Entry().Count(c => c == ':') == 3;

    private static void dumpKind(ExchangeDocument theDoc, StringBuilder theLog, in Label theLabel, in Shape theShape)
    {
        if (theDoc.IsAssembly(theLabel))
        {
            theLog.Append("ASSEMBLY ");
        }
        else if (theDoc.IsSimpleShape(theLabel))
        {
            if (isTopLevelEntry(theLabel))
            {
                theLog.Append("PART ");
            }
        }
        else
        {
            theLog.Append("INSTANCE ");
        }
        theLog.Append(global::Kerf.Engine.Topology.TopAbs.ShapeTypeToString(theShape.Type));
    }

    private static void dumpRefNameDeep(ExchangeDocument theDoc, StringBuilder theLog, in Label theLabel, in Shape theShape, bool deep)
    {
        if (theDoc.GetReferredShape(theLabel, out Label aReferred))
        {
            theLog.Append(" (refers to ").Append(aReferred.Entry()).Append(')');
        }
        if (theDoc.GetName(theLabel, out string? aName))
        {
            theLog.Append(" \"").Append(aName).Append("\" ");
        }
        if (deep)
        {
            // the TShape's and the location's addresses: their identities here
            theLog.Append('(').Append(System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(theShape.TShape()!).ToString("x", CultureInfo.InvariantCulture));
            if (!theShape.Placement.IsIdentity)
            {
                theLog.Append(", ").Append(theShape.Placement.HashCode().ToString("x", CultureInfo.InvariantCulture));
            }
            theLog.Append(") ");
        }
    }

    private static void dumpAssembly(ExchangeDocument theDoc, StringBuilder theLog, in Label theLabel, int level, bool deep)
    {
        Shape aShape = theDoc.GetShape(theLabel);
        if (aShape.IsNull)
        {
            return;
        }
        theLog.Append('\t', level);
        dumpKind(theDoc, theLog, theLabel, aShape);
        theLog.Append(' ').Append(theLabel.Entry());
        dumpRefNameDeep(theDoc, theLog, theLabel, aShape, deep);
        theLog.Append('\n');
        foreach (Label aChild in theDoc.Children(theLabel))
        {
            dumpAssembly(theDoc, theLog, aChild, level + 1, deep);
        }
        if (level == 0)
        {
            theLog.Append('\n');
        }
    }

    private static void dumpShape(ExchangeDocument theDoc, StringBuilder theLog, in Label theLabel, int level, bool deep)
    {
        if (!theDoc.GetShape(theLabel, out Shape aShape))
        {
            return;
        }
        theLog.Append('\t', level);
        dumpKind(theDoc, theLog, theLabel, aShape);
        theLog.Append("  ").Append(theLabel.Entry());
        dumpRefNameDeep(theDoc, theLog, theLabel, aShape, deep);
    }

    private sealed class Stat
    {
        public readonly int[] HAI = new int[21];
        public int NbCentroidProp, NbVolumeProp, NbAreaProp, NbShapesWithName, NbShapesWithColor, NbShapesWithLayer;
    }

    private static void statAssembly(ExchangeDocument theDoc, in Label L, int level, Stat theStat, bool PrintStructMode, DrawResult di)
    {
        if (PrintStructMode)
        {
            for (int j = 0; j <= level; j++)
            {
                di.Append("  ");
            }
            di.Append(L.Entry());
        }
        if (theDoc.GetName(L, out string? aName))
        {
            theStat.NbShapesWithName++;
            if (PrintStructMode)
            {
                di.Append(" ").Append(aName ?? "").Append("  has attributes: ");
            }
        }
        else if (PrintStructMode)
        {
            di.Append(" NoName  has attributes: ");
        }

        DocumentValidationProps? aProps = theDoc.GetValidationProps(L);
        if (aProps?.Centroid is not null)
        {
            if (PrintStructMode)
            {
                di.Append("Centroid ");
            }
            theStat.NbCentroidProp++;
        }
        if (aProps?.Volume is double aVolume)
        {
            if (PrintStructMode)
            {
                di.Append("Volume(").Append(aVolume).Append(") ");
            }
            theStat.NbVolumeProp++;
        }
        if (aProps?.Area is double anArea)
        {
            if (PrintStructMode)
            {
                di.Append("Area(").Append(anArea).Append(") ");
            }
            theStat.NbAreaProp++;
        }
        // colour by layer (XCAFDoc_ColorTool::IsColorByLayer) isn't kept by Draw-lite's documents
        if (theDoc.GetColor(L, ColorType.ColorGen, out ColorWithAlpha col)
            || theDoc.GetColor(L, ColorType.ColorSurf, out col)
            || theDoc.GetColor(L, ColorType.ColorCurv, out col))
        {
            if (PrintStructMode)
            {
                di.Append("Color(").Append(LinearColor.StringName(col.GetRGB().Name())).Append(" ").Append((double)col.Alpha()).Append(") ");
            }
            theStat.NbShapesWithColor++;
        }
        var aLayerS = new NCollection_Sequence<string>();
        if (theDoc.GetLayers(L, aLayerS))
        {
            if (PrintStructMode)
            {
                di.Append("Layer(");
                for (int i = 1; i <= aLayerS.Length(); i++)
                {
                    di.Append(i == 1 ? "\"" : " \"").Append(aLayerS.Value(i)).Append("\"");
                }
                di.Append(") ");
            }
            theStat.NbShapesWithLayer++;
        }
        // visualisation materials aren't kept by Draw-lite's documents
        if (PrintStructMode)
        {
            di.Append("\n");
        }

        theStat.HAI[level]++;
        foreach (Label aChild in theDoc.Children(L))
        {
            statAssembly(theDoc, aChild, level + 1, theStat, PrintStructMode, di);
        }

        // unit 8G2: XCAF keeps a label's external references (XCAFDoc_ShapeTool::SetExternRefs) in a child label of its
        // own, unnamed, whose children are named by the files; the document keeps them as an attribute: counted as
        // XCAF's labels (decision 1281)
        var aRefs = new NCollection_Sequence<string>();
        theDoc.GetExternRefs(L, aRefs);
        if (aRefs.Length() > 0 && level + 2 <= 20)
        {
            string aRefEntry = L.Entry() + ":" + (theDoc.Children(L).Count + 1).ToString(System.Globalization.CultureInfo.InvariantCulture);
            if (PrintStructMode)
            {
                for (int j = 0; j <= level + 1; j++)
                {
                    di.Append("  ");
                }
                di.Append(aRefEntry).Append(" NoName  has attributes: \n");
            }
            theStat.HAI[level + 1]++;
            for (int i = 1; i <= aRefs.Length(); i++)
            {
                if (PrintStructMode)
                {
                    for (int j = 0; j <= level + 2; j++)
                    {
                        di.Append("  ");
                    }
                    di.Append(aRefEntry).Append(':').Append(i).Append(' ').Append(aRefs.Value(i)).Append("  has attributes: \n");
                }
                theStat.NbShapesWithName++;
                theStat.HAI[level + 2]++;
            }
        }
    }

    private static int statdoc(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 2)
        {
            di.Append("Use: ").Append(argv[0]).Append(" Doc \n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }

        bool PrintStructMode = argv.Count == 3;
        var SeqLabels = new NCollection_Sequence<Label>();
        aDoc.GetShapes(SeqLabels);
        if (SeqLabels.Length() <= 0)
        {
            return 0;
        }
        if (PrintStructMode)
        {
            di.Append("\nStructure of shapes in the document:\n");
        }
        var aStat = new Stat();
        foreach (Label aLabel in SeqLabels)
        {
            statAssembly(aDoc, aLabel, 0, aStat, PrintStructMode, di);
        }
        int NbLabelsShape = 0;
        di.Append("\nStatistis of shapes in the document:\n");
        for (int i = 0; i <= 20; i++)
        {
            if (aStat.HAI[i] == 0)
            {
                break;
            }
            di.Append("level N ").Append(i).Append(" : ").Append(aStat.HAI[i]).Append("\n");
            NbLabelsShape += aStat.HAI[i];
        }
        di.Append("Total number of labels for shapes in the document = ").Append(NbLabelsShape).Append("\n");
        di.Append("Number of labels with name = ").Append(aStat.NbShapesWithName).Append("\n");
        di.Append("Number of labels with color link = ").Append(aStat.NbShapesWithColor).Append("\n");
        di.Append("Number of labels with layer link = ").Append(aStat.NbShapesWithLayer).Append("\n");
        di.Append("Number of labels with vis material link = ").Append(0).Append("\n");

        di.Append("\nStatistis of Props in the document:\n");
        di.Append("Number of Centroid Props = ").Append(aStat.NbCentroidProp).Append("\n");
        di.Append("Number of Volume Props = ").Append(aStat.NbVolumeProp).Append("\n");
        di.Append("Number of Area Props = ").Append(aStat.NbAreaProp).Append("\n");

        var CLabels = new NCollection_Sequence<Label>();
        aDoc.GetColors(CLabels);
        di.Append("\nNumber of colors = ").Append(CLabels.Length()).Append("\n");
        if (CLabels.Length() > 0)
        {
            foreach (Label aLabel in CLabels)
            {
                aDoc.GetColor(aLabel, out ColorWithAlpha aColor);
                di.Append(LinearColor.StringName(aColor.GetRGB().Name())).Append(" ");
            }
            di.Append("\n");
        }

        var LLabels = new NCollection_Sequence<Label>();
        aDoc.GetLayerLabels(LLabels);
        di.Append("\nNumber of layers = ").Append(LLabels.Length()).Append("\n");
        if (LLabels.Length() > 0)
        {
            foreach (Label aLabel in LLabels)
            {
                aDoc.GetLayer(aLabel, out string? layerName);
                di.Append("\"").Append(layerName ?? "").Append("\" ");
            }
            di.Append("\n");
        }

        di.Append("\nNumber of vis materials = ").Append(0).Append("\n");
        di.Append("\n");
        return 0;
    }

    private static int setLengthUnit(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" Doc {unitName|scaleFactor} \n");
            return 1;
        }
        ExchangeDocument? aDoc = DDocStdCommands.GetDocument(theDI, argv[1]);
        if (aDoc is null)
        {
            di.Append("Error: ").Append(argv[1]).Append(" is not a document\n");
            return 1;
        }
        string anUnit = argv[2];
        double anUnitValue;
        string anUnitName;
        if (!double.TryParse(anUnit, NumberStyles.Float, CultureInfo.InvariantCulture, out double aValue))
        {
            global::Kerf.Engine.Exchange.Framework.FrameworkLengthUnit aTypeUnit = global::Kerf.Engine.Exchange.Framework.UnitsMethods.LengthUnitFromString(anUnit, false);
            if (aTypeUnit == global::Kerf.Engine.Exchange.Framework.FrameworkLengthUnit.LengthUnit_Undefined)
            {
                di.Append("Error: ").Append(anUnit).Append(" is incorrect unit, use m, mm, km, cm, micron, mille, in, min, nin, ft, stat.mile\n");
                return 1;
            }
            anUnitName = anUnit;
            anUnitValue = global::Kerf.Engine.Exchange.Framework.UnitsMethods.GetLengthFactorValue((int)aTypeUnit) * 0.001;
        }
        else
        {
            anUnitValue = aValue;
            anUnitName = global::Kerf.Engine.Exchange.Framework.UnitsMethods.DumpLengthUnit(anUnitValue, global::Kerf.Engine.Exchange.Framework.FrameworkLengthUnit.LengthUnit_Meter);
        }
        aDoc.SetLengthUnit(anUnitValue, anUnitName);
        return 0;
    }

    private static int dumpLengthUnit(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 2 && argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" Doc [-scale]\n");
            return 1;
        }
        ExchangeDocument? aDoc = DDocStdCommands.GetDocument(theDI, argv[1]);
        if (aDoc is null)
        {
            di.Append("Error: ").Append(argv[1]).Append(" is not a document\n");
            return 1;
        }
        if (aDoc.LengthUnit is not DocumentLengthUnit aUnits)
        {
            di.Append("Error: Document doesn't contain a Length Unit\n");
            return 1;
        }
        if (argv.Count == 3)
        {
            if (argv[2].ToLowerInvariant() == "-scale")
            {
                di.Append(aUnits.UnitValue);
                return 0;
            }
            di.Append("Error: Incorrect option, use -scale\n");
            return 1;
        }
        di.Append(aUnits.Name);
        return 0;
    }

    // ============================================================================================== shape commands

    /// <summary>
    /// The document's length unit, set as XAddShape sets it when there is none: the session's unit
    /// (<c>xstep.cascade.unit</c>, millimetres until unit 8F gives Draw-lite its parameters) in metres.
    /// </summary>
    internal static void EnsureLengthUnit(ExchangeDocument theDoc)
    {
        if (!theDoc.GetLengthUnit(out _))
        {
            double aLengthUnit = global::Kerf.Engine.Exchange.Framework.UnitsMethods.GetCasCadeLengthUnit(null, global::Kerf.Engine.Exchange.Framework.FrameworkLengthUnit.LengthUnit_Meter);
            // XCAFDoc_DocumentTool::SetLengthUnit (doc, value): named by DumpLengthUnit
            theDoc.SetLengthUnit(aLengthUnit, global::Kerf.Engine.Exchange.Framework.UnitsMethods.DumpLengthUnit(aLengthUnit, global::Kerf.Engine.Exchange.Framework.FrameworkLengthUnit.LengthUnit_Meter));
        }
    }

    private static int addShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName Shape [int makeAssembly (1/0)]\n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        Shape aShape = theDI.GetShape(argv[2]);
        if (aShape.IsNull)
        {
            di.Append("Syntax error: shape '").Append(argv[2]).Append("' is undefined\n");
            return 1;
        }
        EnsureLengthUnit(aDoc);
        bool makeAssembly = !(argv.Count == 4 && theDI.Atoi(argv[3]) == 0);
        Label aLabel = aDoc.AddShape(aShape, makeAssembly);
        if (aLabel.IsNull)
        {
            di.Append("Null Label\n");
        }
        di.Append(aLabel.Entry());
        return 0;
    }

    private static int newShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 2)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName \n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        di.Append(aDoc.NewShape().Entry());
        return 0;
    }

    private static int setShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 4)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName Label Shape \n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        Shape aShape = theDI.GetShape(argv[3]);
        if (aLabel.IsNull)
        {
            // TNaming_Builder on a null label: OCAF raises
            throw new NullObjectException("TDF_Label::Data");
        }
        aDoc.SetShape(aLabel, aShape);
        return 0;
    }

    private static int getShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 4)
        {
            di.Append("Use: ").Append(argv[0]).Append(" Result DocName Label\n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[2]);
        if (aDoc is null)
        {
            return 1;
        }
        Label aLabel = DDocStdCommands.Label(aDoc, argv[3]);
        if (aLabel.IsNull)
        {
            di.Append("No such Label\n");
            return 1;
        }
        theDI.SetShape(argv[1], aDoc.GetShape(aLabel));
        return 0;
    }

    private static int removeShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3 && argv.Count != 4)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName Label [int removeCompletely (1/0)]\n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        if (aLabel.IsNull)
        {
            di.Append("No such Label\n");
            return 1;
        }
        bool removeCompletely = !(argv.Count == 4 && theDI.Atoi(argv[3]) == 0);
        aDoc.RemoveShape(aLabel, removeCompletely);
        return 0;
    }

    private static int findShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName Shape [0/1]\n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        Shape aShape = theDI.GetShape(argv[2]);
        bool findInstance = argv.Count == 4 && argv[3].StartsWith('1');
        di.Append(aDoc.FindShape(aShape, findInstance).Entry());
        return 0;
    }

    private static int findSubShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 4)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName Shape ParentLabel\n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        Shape aShape = theDI.GetShape(argv[2]);
        Label aParent = DDocStdCommands.Label(aDoc, argv[3]);
        Label aLabel = default;
        if (!aParent.IsNull)
        {
            aDoc.FindSubShape(aParent, aShape, out aLabel);
        }
        di.Append(aLabel.Entry());
        return 0;
    }

    private static int findMainShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName SubShape\n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        di.Append(aDoc.FindMainShape(theDI.GetShape(argv[2])).Entry());
        return 0;
    }

    private static int addSubShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 4)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName Shape ParentLabel\n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        Shape aShape = theDI.GetShape(argv[2]);
        Label aParent = DDocStdCommands.Label(aDoc, argv[3]);
        Label aLabel = aParent.IsNull ? default : aDoc.AddSubShape(aParent, aShape);
        di.Append(aLabel.Entry());
        return 0;
    }

    private static int labelInfo(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName Label \n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        if (aDoc.IsShape(aLabel))
        {
            di.Append("There are a TopLevelShape");
        }
        if (aDoc.IsComponent(aLabel))
        {
            di.Append("There are a Component");
        }
        if (aDoc.IsAssembly(aLabel))
        {
            di.Append("There are an Assembly");
        }
        if (aDoc.IsFree(aLabel))
        {
            di.Append("This Shape don't used");
        }
        return 0;
    }

    private static int getUsers(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" Doc Label [withSubChilds(int)]\n");
            return 1;
        }
        bool getsubchilds = argv.Count == 4 && theDI.Atoi(argv[3]) == 1;
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        int aNb = aLabel.IsNull ? 0 : aDoc.GetUsers(aLabel, new NCollection_Sequence<Label>(), getsubchilds);
        di.Append(aNb);
        return 0;
    }

    private static int nbComponents(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" Doc Label [withSubChilds(int)]\n");
            return 1;
        }
        bool getsubchilds = argv.Count == 4 && theDI.Atoi(argv[3]) == 1;
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        di.Append(aLabel.IsNull ? 0 : aDoc.NbComponents(aLabel, getsubchilds));
        return 0;
    }

    private static int addComponent(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 4)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName Label Shape \n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        Shape aShape = theDI.GetShape(argv[3]);
        if (!aLabel.IsNull)
        {
            aDoc.AddComponent(aLabel, aShape);
        }
        di.Append(aLabel.Entry());
        return 0;
    }

    private static int removeComponent(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName Label \n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        if (!aLabel.IsNull)
        {
            aDoc.RemoveComponent(aLabel);
        }
        return 0;
    }

    private static int getReferredShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName Label \n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        aDoc.GetReferredShape(aLabel, out Label aRoot);
        di.Append(aRoot.Entry());
        return 0;
    }

    private static int getTopLevelShapes(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 2)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName \n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        var aLabels = new NCollection_Sequence<Label>();
        aDoc.GetShapes(aLabels);
        foreach (Label aLabel in aLabels)
        {
            di.Append(aLabel.Entry()).Append(" ");
        }
        return 0;
    }

    private static int getFreeShapes(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 2)
        {
            di.Append("Use: ").Append(argv[0]).Append(" DocName [shape_prefix]\n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        var aLabels = new NCollection_Sequence<Label>();
        aDoc.GetFreeShapes(aLabels);
        if (aLabels.Length() <= 0)
        {
            di.Append("Document ").Append(argv[1]).Append(" contain no shapes\n");
            return 0;
        }
        if (argv.Count == 2)
        {
            foreach (Label aLabel in aLabels)
            {
                di.Append(aLabel.Entry()).Append(" ");
            }
        }
        else if (aLabels.Length() == 1)
        {
            theDI.SetShape(argv[2], aDoc.GetShape(aLabels.Value(1)));
            di.Append(argv[2]);
        }
        else
        {
            for (int i = 1; i <= aLabels.Length(); i++)
            {
                string aName = argv[2] + "_" + i.ToString(CultureInfo.InvariantCulture);
                theDI.SetShape(aName, aDoc.GetShape(aLabels.Value(i)));
                di.Append(aName).Append(" ");
            }
        }
        return 0;
    }

    private static int getOneShape(DrawInterpreter theDI, DrawResult theDI2, IReadOnlyList<string> theArgVec)
    {
        if (theArgVec.Count != 3)
        {
            theDI2.Append("Use: ").Append(theArgVec[0]).Append(" shape DocName \n");
            return 1;
        }
        ExchangeDocument? aDoc = DDocStdCommands.GetDocument(theDI, theArgVec[2]);
        if (aDoc is null)
        {
            theDI2.Append("Error: ").Append(theArgVec[2]).Append(" is not a document\n");
            return 1;
        }
        Shape aShape = GetOneShape(aDoc);
        if (aShape.IsNull)
        {
            theDI2.Append("Error: Document ").Append(theArgVec[2]).Append(" contain no shapes\n");
            return 1;
        }
        theDI.SetShape(theArgVec[1], aShape);
        theDI2.Append(theArgVec[1]);
        return 0;
    }

    /// <summary>
    /// XCAFDoc_ShapeTool::GetOneShape: the only free shape, or the compound of the free shapes, or a null shape when
    /// there is none.
    /// </summary>
    internal static Shape GetOneShape(ExchangeDocument theDoc)
    {
        var aFree = new NCollection_Sequence<Label>();
        theDoc.GetFreeShapes(aFree);
        return aFree.Length() == 0 ? new Shape() : theDoc.GetOneShape();
    }

    private static int XDumpLocation(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" Doc Label \n");
            return 1;
        }
        ExchangeDocument? aDoc = Doc(theDI, di, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        Label aLabel = DDocStdCommands.Label(aDoc, argv[2]);
        // XCAFDoc_Location: a reference's location
        if (!aDoc.IsReference(aLabel))
        {
            di.Append("Label ").Append(argv[2]).Append(" doesn't contain XCAFDoc_Location attribute\n");
            return 1;
        }
        Transform3D aTrsf = aDoc.GetLocation(aLabel).Transformation;
        di.Append("Transformation (3 rows * 4 columns matrix):");
        for (int i = 1; i <= 3; i++)
        {
            di.Append(" (");
            for (int j = 1; j <= 4; j++)
            {
                if (j > 1)
                {
                    di.Append(",");
                }
                di.Append(DrawFormat.AsciiString(aTrsf.Value(i, j)));
            }
            di.Append(")");
        }
        return 0;
    }

    private static int updateAssemblies(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count != 2)
        {
            di.Append("Use: ").Append(argv[0]).Append(" Doc\n");
            return 1;
        }
        ExchangeDocument? aDoc = DDocStdCommands.GetDocument(theDI, argv[1]);
        if (aDoc is null)
        {
            return 1;
        }
        aDoc.UpdateAssemblies();
        return 0;
    }

    private static int XAutoNaming(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVec)
    {
        if (theArgVec.Count != 2 && theArgVec.Count != 3)
        {
            theResult.Append("Syntax error: wrong number of arguments");
            return 1;
        }
        if (DDocStdCommands.GetDocument(theDI, theArgVec[1]) is null)
        {
            theResult.Append("Syntax error: '").Append(theArgVec[1]).Append("' is not a document");
            return 1;
        }
        if (theArgVec.Count == 2)
        {
            theResult.Append(DDocStdCommands.AutoNaming(theDI) ? "1" : "0");
            return 0;
        }
        bool toEnable = true;
        if (!DBRepCommands.ParseOnOff(theArgVec[2], ref toEnable))
        {
            theResult.Append("Syntax error at '").Append(theArgVec[2]).Append("'");
            return 1;
        }
        DDocStdCommands.SetAutoNaming(theDI, toEnable);
        return 0;
    }
}
