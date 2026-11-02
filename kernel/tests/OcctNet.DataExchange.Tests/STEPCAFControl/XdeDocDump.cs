// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;

namespace OcctNet.DataExchange.Tests.STEPCAFControl;

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

    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static void dumpLabel(XdeDoc_Document theDoc, XdeDoc_Label theLabel, List<string> theOut)
    {
        string aKind = theDoc.IsAssembly(theLabel) ? "A"
                       : theDoc.IsReference(theLabel) ? "R"
                       : theDoc.IsSubShape(theLabel) ? "U"
                       : theDoc.IsSimpleShape(theLabel) ? "S"
                       : "E";
        theOut.Add($"L {theLabel.Entry()} {aKind}");
        if (theDoc.GetShape(theLabel, out TopoDS_Shape aShape))
        {
            theOut.Add($"T {TopAbs.ShapeTypeToString(aShape.ShapeType())} {Count(aShape, TopAbs_ShapeEnum.TopAbs_SOLID)} "
                       + $"{Count(aShape, TopAbs_ShapeEnum.TopAbs_SHELL)} {Count(aShape, TopAbs_ShapeEnum.TopAbs_FACE)} "
                       + $"{Count(aShape, TopAbs_ShapeEnum.TopAbs_EDGE)} {Count(aShape, TopAbs_ShapeEnum.TopAbs_VERTEX)}");
        }
        if (theDoc.GetReferredShape(theLabel, out XdeDoc_Label aReferred))
        {
            theOut.Add("P " + aReferred.Entry());
            gp_Trsf aTrsf = theDoc.GetLocation(theLabel).Transformation();
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
        foreach ((XdeDoc_ColorType aType, string aTag) in new[]
                 {
                     (XdeDoc_ColorType.XdeDoc_ColorGen, "g"), (XdeDoc_ColorType.XdeDoc_ColorSurf, "s"),
                     (XdeDoc_ColorType.XdeDoc_ColorCurv, "c"),
                 })
        {
            if (theDoc.GetColor(theLabel, aType, out Quantity_ColorRGBA aColor))
            {
                NCollection_Vec3<float> aRgb = aColor.GetRGB().Rgb();
                theOut.Add($"C {aTag} {F9(aRgb.r())} {F9(aRgb.g())} {F9(aRgb.b())} {F9(aColor.Alpha())}");
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
        XdeDoc_ValidationProps? aProps = theDoc.GetValidationProps(theLabel);
        if (aProps?.Volume is double aVolume)
        {
            theOut.Add("V " + R17(aVolume));
        }
        if (aProps?.Area is double anArea)
        {
            theOut.Add("A " + R17(anArea));
        }
        if (aProps?.Centroid is gp_Pnt aPnt)
        {
            theOut.Add($"G {R17(aPnt.X())} {R17(aPnt.Y())} {R17(aPnt.Z())}");
        }
        foreach (XdeDoc_Label aChild in theDoc.Children(theLabel))
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
    public static List<string> Dump(XdeDoc_Document theDoc)
    {
        var aLines = new List<string>();
        aLines.Add(theDoc.GetLengthUnit(out double aUnit) ? "U " + R17(aUnit) : "U -");
        foreach (XdeDoc_Label aChild in theDoc.Children(theDoc.ShapesLabel()))
        {
            dumpLabel(theDoc, aChild, aLines);
        }
        var aLayers = new NCollection_Sequence<XdeDoc_Label>();
        theDoc.GetLayerLabels(aLayers);
        foreach (XdeDoc_Label aLayer in aLayers)
        {
            theDoc.GetLayer(aLayer, out string? aName);
            aLines.Add($"Z {aLayer.Entry()} {(theDoc.IsVisible(aLayer) ? 1 : 0)} {Escape(aName ?? "")}");
        }
        return aLines;
    }

    // stepcafrec's palette
    public static Quantity_ColorRGBA PaletteColor(int theIndex) => (theIndex % 4) switch
    {
        0 => new Quantity_ColorRGBA(new Quantity_Color(Quantity_NameOfColor.Quantity_NOC_RED)),
        1 => new Quantity_ColorRGBA(new Quantity_Color(0.25, 0.5, 0.75, Quantity_TypeOfColor.Quantity_TOC_RGB)),
        2 => new Quantity_ColorRGBA(new Quantity_Color(0.125, 0.625, 0.25, Quantity_TypeOfColor.Quantity_TOC_RGB), 0.5f),
        _ => new Quantity_ColorRGBA(new Quantity_Color(Quantity_NameOfColor.Quantity_NOC_GREEN)),
    };

    /// <summary>stepcafrec's MakePartDoc, with XCAF's auto-naming.</summary>
    public static XdeDoc_Document MakePartDoc(in TopoDS_Shape theShape, string theName, int theIndex)
    {
        var aDoc = new XdeDoc_Document { AutoNaming = true };
        XdeDoc_Label aLabel = aDoc.AddShape(theShape, false);
        aDoc.SetName(aLabel, theName);
        aDoc.SetColor(aLabel, PaletteColor(theIndex), theIndex % 3 == 1 ? XdeDoc_ColorType.XdeDoc_ColorGen : XdeDoc_ColorType.XdeDoc_ColorSurf);
        var anExp = new TopExp_Explorer(theShape, TopAbs_ShapeEnum.TopAbs_FACE);
        if (anExp.More())
        {
            XdeDoc_Label aSub = aDoc.AddSubShape(aLabel, anExp.Current());
            if (!aSub.IsNull())
            {
                aDoc.SetName(aSub, "first face");
                aDoc.SetColor(aSub, PaletteColor(theIndex + 1).GetRGB(), XdeDoc_ColorType.XdeDoc_ColorCurv);
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
        aDoc.SetCentroid(aLabel, new gp_Pnt(theIndex, 2.0 * theIndex, 0.5));
        return aDoc;
    }

    /// <summary>stepcafrec's MakeAssemblyDoc, with XCAF's auto-naming.</summary>
    public static XdeDoc_Document MakeAssemblyDoc(in TopoDS_Shape theShape)
    {
        var aDoc = new XdeDoc_Document { AutoNaming = true };
        XdeDoc_Label aPart = aDoc.AddShape(theShape, false);
        aDoc.SetName(aPart, "part");
        aDoc.SetColor(aPart, new Quantity_Color(Quantity_NameOfColor.Quantity_NOC_RED), XdeDoc_ColorType.XdeDoc_ColorSurf);
        aDoc.SetLayer(aPart, "parts");
        aDoc.SetVolume(aPart, 12.5);
        XdeDoc_Label aSub = aDoc.NewShape();
        aDoc.SetName(aSub, "sub");
        var aTrsf = new gp_Trsf();
        aTrsf.SetRotation(new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 0.5);
        aTrsf.SetTranslationPart(new gp_Vec(100.0, 50.0, 0.0));
        aDoc.AddComponent(aSub, aPart, new TopLoc_Location());
        XdeDoc_Label aSecond = aDoc.AddComponent(aSub, aPart, new TopLoc_Location(aTrsf));
        aDoc.SetName(aSecond, "second");
        aDoc.SetColor(aSecond, new Quantity_Color(0.25, 0.5, 0.75, Quantity_TypeOfColor.Quantity_TOC_RGB), XdeDoc_ColorType.XdeDoc_ColorSurf);
        XdeDoc_Label aTop = aDoc.NewShape();
        aDoc.SetName(aTop, "top");
        var aTrsf2 = new gp_Trsf();
        aTrsf2.SetTranslation(new gp_Vec(0.0, 0.0, 300.0));
        aDoc.AddComponent(aTop, aSub, new TopLoc_Location());
        XdeDoc_Label anUpper = aDoc.AddComponent(aTop, aSub, new TopLoc_Location(aTrsf2));
        aDoc.SetName(anUpper, "upper");
        aDoc.UpdateAssemblies();
        return aDoc;
    }
}
