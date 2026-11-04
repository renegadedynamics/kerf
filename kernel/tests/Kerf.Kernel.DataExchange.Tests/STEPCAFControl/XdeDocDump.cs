// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.Text;

namespace Kerf.Kernel.DataExchange.Tests.STEPCAFControl;

/// <summary>
/// The dump of a document as tools/stepcafrec dumps an XCAF document (unit 8G2), and the documents its write mode
/// builds: the comparisons with native read these.
/// </summary>
internal static class XdeDocDump
{
    private static string R17(double theValue) => theValue.ToString("G17", CultureInfo.InvariantCulture).Replace('E', 'e');

    private static string F9(float theValue) => ((double)theValue).ToString("G9", CultureInfo.InvariantCulture).Replace('E', 'e');

    private static string Escape(string theText)
    {
        var aBuilder = new StringBuilder();
        foreach (char c in theText)
        {
            switch (c)
            {
                case '\\':
                    aBuilder.Append("\\\\");
                    break;
                case '\n':
                    aBuilder.Append("\\n");
                    break;
                case '\r':
                    aBuilder.Append("\\r");
                    break;
                default:
                    aBuilder.Append(c);
                    break;
            }
        }
        return aBuilder.ToString();
    }

    private static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        global::Kerf.Engine.Topology.Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    private static void dumpLabel(ExchangeDocument theDoc, Label theLabel, List<string> theOut)
    {
        string aKind = theDoc.IsAssembly(theLabel) ? "A"
                       : theDoc.IsReference(theLabel) ? "R"
                       : theDoc.IsSubShape(theLabel) ? "U"
                       : theDoc.IsSimpleShape(theLabel) ? "S"
                       : "E";
        theOut.Add($"L {theLabel.Entry()} {aKind}");
        if (theDoc.GetShape(theLabel, out Shape aShape))
        {
            theOut.Add($"T {Kerf.Engine.Topology.TopAbs.ShapeTypeToString(aShape.Type)} {Count(aShape, ShapeType.Solid)} "
                       + $"{Count(aShape, ShapeType.Shell)} {Count(aShape, ShapeType.Face)} "
                       + $"{Count(aShape, ShapeType.Edge)} {Count(aShape, ShapeType.Vertex)}");
        }
        if (theDoc.GetReferredShape(theLabel, out Label aReferred))
        {
            theOut.Add("P " + aReferred.Entry());
            Transform3D aTrsf = theDoc.GetLocation(theLabel).Transformation;
            var aBuilder = new StringBuilder("O");
            for (int r = 1; r <= 3; r++)
            {
                for (int c = 1; c <= 4; c++)
                {
                    aBuilder.Append(' ').Append(R17(aTrsf.Value(r, c)));
                }
            }
            theOut.Add(aBuilder.ToString());
        }
        if (theDoc.GetName(theLabel, out string? aName))
        {
            theOut.Add("N " + Escape(aName ?? ""));
        }
        foreach ((ColorType aType, string aTag) in new[]
                 {
                     (ColorType.ColorGen, "g"), (ColorType.ColorSurf, "s"),
                     (ColorType.ColorCurv, "c"),
                 })
        {
            if (theDoc.GetColor(theLabel, aType, out ColorWithAlpha aColor))
            {
                NCollection_Vec3<float> aRgb = aColor.GetRGB().Rgb();
                theOut.Add($"C {aTag} {F9(aRgb.R())} {F9(aRgb.G())} {F9(aRgb.B())} {F9(aColor.Alpha())}");
            }
        }
        if (!theDoc.IsVisible(theLabel))
        {
            theOut.Add("H");
        }
        var aLayers = new NCollection_Sequence<string>();
        if (theDoc.GetLayers(theLabel, aLayers))
        {
            foreach (string aLayer in aLayers)
            {
                theOut.Add("Y " + Escape(aLayer));
            }
        }
        DocumentValidationProps? aProps = theDoc.GetValidationProps(theLabel);
        if (aProps?.Volume is double aVolume)
        {
            theOut.Add("V " + R17(aVolume));
        }
        if (aProps?.Area is double anArea)
        {
            theOut.Add("A " + R17(anArea));
        }
        if (aProps?.Centroid is Point3D aPnt)
        {
            theOut.Add($"G {R17(aPnt.X)} {R17(aPnt.Y)} {R17(aPnt.Z)}");
        }
        foreach (Label aChild in theDoc.Children(theLabel))
        {
            dumpLabel(theDoc, aChild, theOut);
        }
        var aRefs = new NCollection_Sequence<string>();
        theDoc.GetExternRefs(theLabel, aRefs);
        foreach (string aRef in aRefs)
        {
            theOut.Add("X " + Escape(aRef));
        }
    }

    /// <summary>The document's dump, line by line.</summary>
    public static List<string> Dump(ExchangeDocument theDoc)
    {
        var aLines = new List<string>();
        aLines.Add(theDoc.GetLengthUnit(out double aUnit) ? "U " + R17(aUnit) : "U -");
        foreach (Label aChild in theDoc.Children(theDoc.ShapesLabel()))
        {
            dumpLabel(theDoc, aChild, aLines);
        }
        var aLayers = new NCollection_Sequence<Label>();
        theDoc.GetLayerLabels(aLayers);
        foreach (Label aLayer in aLayers)
        {
            theDoc.GetLayer(aLayer, out string? aName);
            aLines.Add($"Z {aLayer.Entry()} {(theDoc.IsVisible(aLayer) ? 1 : 0)} {Escape(aName ?? "")}");
        }
        return aLines;
    }

    // stepcafrec's palette
    public static ColorWithAlpha PaletteColor(int theIndex) => (theIndex % 4) switch
    {
        0 => new ColorWithAlpha(new LinearColor(NameOfColor.NocRed)),
        1 => new ColorWithAlpha(new LinearColor(0.25, 0.5, 0.75, ColorSpace.TocRgb)),
        2 => new ColorWithAlpha(new LinearColor(0.125, 0.625, 0.25, ColorSpace.TocRgb), 0.5f),
        _ => new ColorWithAlpha(new LinearColor(NameOfColor.NocGreen)),
    };

    /// <summary>stepcafrec's MakePartDoc, with XCAF's auto-naming.</summary>
    public static ExchangeDocument MakePartDoc(in Shape theShape, string theName, int theIndex)
    {
        var aDoc = new ExchangeDocument { AutoNaming = true };
        Label aLabel = aDoc.AddShape(theShape, false);
        aDoc.SetName(aLabel, theName);
        aDoc.SetColor(aLabel, PaletteColor(theIndex), theIndex % 3 == 1 ? ColorType.ColorGen : ColorType.ColorSurf);
        var anExp = new global::Kerf.Engine.Topology.Explorer(theShape, ShapeType.Face);
        if (anExp.More())
        {
            Label aSub = aDoc.AddSubShape(aLabel, anExp.Current());
            if (!aSub.IsNull)
            {
                aDoc.SetName(aSub, "first face");
                aDoc.SetColor(aSub, PaletteColor(theIndex + 1).GetRGB(), ColorType.ColorCurv);
                aDoc.SetLayer(aSub, "faces");
            }
        }
        aDoc.SetLayer(aLabel, "parts");
        if (theIndex % 5 == 0)
        {
            aDoc.SetVisibility(aDoc.FindLayer("parts"), false);
        }
        if (theIndex % 7 == 3)
        {
            aDoc.SetVisibility(aLabel, false);
        }
        aDoc.SetVolume(aLabel, 100.0 + theIndex);
        aDoc.SetArea(aLabel, 50.5 + theIndex);
        aDoc.SetCentroid(aLabel, new Point3D(theIndex, 2.0 * theIndex, 0.5));
        return aDoc;
    }

    /// <summary>stepcafrec's MakeAssemblyDoc, with XCAF's auto-naming.</summary>
    public static ExchangeDocument MakeAssemblyDoc(in Shape theShape)
    {
        var aDoc = new ExchangeDocument { AutoNaming = true };
        Label aPart = aDoc.AddShape(theShape, false);
        aDoc.SetName(aPart, "part");
        aDoc.SetColor(aPart, new LinearColor(NameOfColor.NocRed), ColorType.ColorSurf);
        aDoc.SetLayer(aPart, "parts");
        aDoc.SetVolume(aPart, 12.5);
        Label aSub = aDoc.NewShape();
        aDoc.SetName(aSub, "sub");
        var aTrsf = new Transform3D();
        aTrsf = aTrsf.WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0.5);
        aTrsf = aTrsf.WithTranslationPart(new Vector3D(100.0, 50.0, 0.0));
        aDoc.AddComponent(aSub, aPart, new Placement());
        Label aSecond = aDoc.AddComponent(aSub, aPart, new Placement(aTrsf));
        aDoc.SetName(aSecond, "second");
        aDoc.SetColor(aSecond, new LinearColor(0.25, 0.5, 0.75, ColorSpace.TocRgb), ColorType.ColorSurf);
        Label aTop = aDoc.NewShape();
        aDoc.SetName(aTop, "top");
        var aTrsf2 = new Transform3D();
        aTrsf2 = aTrsf2.WithTranslation(new Vector3D(0.0, 0.0, 300.0));
        aDoc.AddComponent(aTop, aSub, new Placement());
        Label anUpper = aDoc.AddComponent(aTop, aSub, new Placement(aTrsf2));
        aDoc.SetName(anUpper, "upper");
        aDoc.UpdateAssemblies();
        return aDoc;
    }
}
