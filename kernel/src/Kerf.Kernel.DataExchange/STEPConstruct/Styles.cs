// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Styles.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Styles.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Collections;
using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Header;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Foundation.NameOfColor;
using static Kerf.Engine.Foundation.ColorSpace;
using static Kerf.Engine.Exchange.Step.Visual.SurfaceSide;

namespace Kerf.Engine.Exchange.Step.Construct;

/// <summary>
/// Provides a mechanism for reading and writing shape styles (such as color) to and from the STEP file This tool
/// maintains a list of styles, either taking them from STEP model (reading), or filling it by calls to AddStyle or
/// directly (writing). Some methods deal with general structures of styles and presentations in STEP, but there are
/// methods which deal with particular implementation of colors (as described in RP)
/// </summary>
/// <remarks>A C++ value class, a C# class here (decision 1185).</remarks>
public class Styles : ConstructTool
{
    //=======================================================================
    // function : ProcessAsSurfaceStyleRendering
    // purpose  : Process StepVisual_SurfaceStyleElementSelect to extract a
    //           render color and render trnasparency from it. Returns true,
    //           if theSSES was of type StepVisual_SurfaceStyleRendering
    //           (even if color and transparency data couldn't be extracted
    //           for some reason), otherwise returns false.
    //=======================================================================
    private static bool ProcessAsSurfaceStyleRendering(SurfaceStyleElementSelect sses,
                                                       RenderingProperties renderingProps)
    {
        SurfaceStyleRendering ssr = sses.SurfaceStyleRendering();
        if (ssr is null)
        {
            return false;
        }
        SurfaceStyleRenderingWithProperties ssrwp = ssr as SurfaceStyleRenderingWithProperties;
        if (ssrwp is null)
        {
            return true;
        }
        renderingProps.Init(ssrwp);
        return renderingProps.IsDefined;
    }

    //=======================================================================
    // function : ProcessAsSurfaceStyleBoundary
    // purpose  : Process StepVisual_SurfaceStyleElementSelect to extract a
    //           boundary color from it. Returns true,
    //           if theSSES was of type StepVisual_SurfaceStyleBoundary
    //           (even if boundary color data couldn't be extracted
    //           for some reason), otherwise returns false.
    //=======================================================================
    private static bool ProcessAsSurfaceStyleBoundary(SurfaceStyleElementSelect sses,
                                                      ref Colour boundaryColour)
    {
        SurfaceStyleBoundary ssb = sses.SurfaceStyleBoundary();
        if (ssb is null)
        {
            return false;
        }
        CurveStyle cs = ssb.StyleOfBoundary();
        if (cs is null)
        {
            return true;
        }
        boundaryColour = cs.CurveColour();
        return true;
    }

    //=======================================================================
    // function : ProcessAsSurfaceStyleFillArea
    // purpose  : Process StepVisual_SurfaceStyleElementSelect to extract a
    //           surface color from it. Doesn't return color for negative
    //           side. Returns true, if theSSES was of type
    //           StepVisual_SurfaceStyleFillArea (even if surface color data
    //           couldn't be extracted or some reason), otherwise returns
    //           false.
    //=======================================================================
    private static bool ProcessAsSurfaceStyleFillArea(SurfaceStyleElementSelect sses,
                                                      SurfaceSide side,
                                                      ref Colour surfaceColour)
    {
        SurfaceStyleFillArea ssfa = sses.SurfaceStyleFillArea();
        if (ssfa is null)
        {
            return false;
        }
        FillAreaStyle fas = ssfa.FillArea();
        if (fas is null)
        {
            return true;
        }

        for (int fssIndex = 1; fssIndex <= fas.FillStyleCount; fssIndex++)
        {
            FillStyleSelect fss = fas.FillStylesValue(fssIndex);
            FillAreaStyleColour fasc = fss?.FillAreaStyleColour();
            if (fasc is not null
                // If current surface color is null, we will use negative side color.
                // Otherwise negative side color is ignored.
                && (surfaceColour is null || side != SurfaceSide.SsNegative)) // abv 30 Mar 00: trj3_s1-pe.stp
            {
                surfaceColour = fasc.FillColour();
            }
        }
        return true;
    }

    //=======================================================================
    // function : ProcessAsSurfaceStyleUsage
    // purpose  : Process StepVisual_PresentationStyleSelect to extract
    //           following data from it: surface color, boundary color,
    //           render color, render transparency. Returns true,
    //           if thePSS was of type StepVisual_SurfaceStyleUsage
    //           (even if no data at all could be extracted for some reason),
    //           otherwise returns false.
    //=======================================================================
    private static bool ProcessAsSurfaceStyleUsage(PresentationStyleSelect pss,
                                                   ref Colour surfaceColour,
                                                   ref Colour boundaryColour,
                                                   RenderingProperties renderingProps)
    {
        SurfaceStyleUsage ssu = pss.SurfaceStyleUsage();
        if (ssu is null)
        {
            return false;
        }

        SurfaceSideStyle sss = ssu.Style();
        for (int ssesIndex = 1; ssesIndex <= sss.StyleCount; ++ssesIndex)
        {
            SurfaceStyleElementSelect sses = sss.StylesValue(ssesIndex);
            // SurfaceStyleElementSelect can be of only one of the following types:
            // SurfaceStyleFillArea, SurfaceStyleBoundary, SurfaceStyleRendering.
            // So we're using && operator to stop as soon as this type is processed.
            _ = ProcessAsSurfaceStyleFillArea(sses, ssu.Side(), ref surfaceColour)
                || ProcessAsSurfaceStyleBoundary(sses, ref boundaryColour)
                || ProcessAsSurfaceStyleRendering(sses, renderingProps);
        }
        return true;
    }

    //=======================================================================
    // function : ProcessAsCurveStyle
    // purpose  : Process StepVisual_PresentationStyleSelect to extract a
    //           curve color from it. Returns true,
    //           if thePSS was of type StepVisual_SurfaceStyleRendering
    //           (even if curve color data couldn't be extracted
    //           for some reason), otherwise returns false.
    //=======================================================================
    private static bool ProcessAsCurveStyle(PresentationStyleSelect pss, ref Colour curveColour)
    {
        CurveStyle cs = pss.CurveStyle();
        if (cs is null)
        {
            return false;
        }

        curveColour = cs.CurveColour();
        return true;
    }

    /// <summary>Creates an empty tool</summary>
    public Styles()
    {
    }

    /// <summary>Creates a tool and initializes it</summary>
    public Styles(ExchangeWorkSession ws) : base(ws)
    {
    }

    /// <summary>Initializes tool; returns True if succeeded</summary>
    public bool Init(ExchangeWorkSession ws)
    {
        _mapOfStyles.Clear();
        _styles.Clear();
        _psa.Clear();
        return SetWS(ws);
    }

    /// <summary>Returns number of defined styles</summary>
    public int StyleCount
    {
        get
        {
            return _styles.Extent();
        }
    }

    /// <summary>Returns style with given index</summary>
    public StyledItem Style(int i)
    {
        return _styles.FindKey(i) as StyledItem;
    }

    /// <summary>Returns number of override styles</summary>
    public int RootStyleCount
    {
        get
        {
            return _rootStyles.Extent();
        }
    }

    /// <summary>Returns override style with given index</summary>
    public StyledItem RootStyle(int i)
    {
        return _rootStyles.FindKey(i) as StyledItem;
    }

    /// <summary>Clears all defined styles and PSA sequence</summary>
    public void ClearStyles()
    {
        _styles.Clear();
        _psa.Clear();
        _rootStyles.Clear();
    }

    /// <summary>Adds a style to a sequence</summary>
    public void AddStyle(StyledItem style)
    {
        _styles.Add(style);
    }

    /// <summary>
    /// Create a style linking giving PSA to the item, and add it to the sequence of stored styles. If Override is not
    /// Null, then the resulting style will be of the subtype OverridingStyledItem.
    /// </summary>
    public StyledItem AddStyle(RepresentationItem item, PresentationStyleAssignment psa,
                                          StyledItem Override)
    {
        StyledItem style;

        NCollection_HArray1<PresentationStyleAssignment> styles =
            new NCollection_HArray1<PresentationStyleAssignment>(1, 1);
        styles.SetValue(1, psa);

        if (Override is null)
        {
            TCollection_HAsciiString styName = new TCollection_HAsciiString("color");
            style = new StyledItem();
            style.Init(styName, styles, item);
        }
        else
        {
            TCollection_HAsciiString styName = new TCollection_HAsciiString("overriding color");
            OverRidingStyledItem oStyle = new OverRidingStyledItem();
            oStyle.Init(styName, styles, item, Override);
            style = oStyle;
        }

        _styles.Add(style);
        // for future using
        _psa.Append(psa);

        return style;
    }

    /// <summary>
    /// Create a style linking giving PSA to the Shape, and add it to the sequence of stored styles. If Override is not
    /// Null, then the resulting style will be of the subtype OverridingStyledItem. The Sape is used to find
    /// corresponding STEP entity by call to STEPConstruct::FindEntity(), then previous method is called.
    /// </summary>
    public StyledItem AddStyle(Shape shape, PresentationStyleAssignment psa,
                                          StyledItem Override)
    {
        RepresentationItem item = StepConstruct.FindEntity(FinderProcess(), shape);
        StyledItem style = null;
        if (item is not null)
        {
            style = AddStyle(item, psa, Override);
        }
        return style;
    }

    /// <summary>Create MDGPR, fill it with all the styles previously defined, and add it to the model</summary>
    public bool CreateMDGPR(RepresentationContext context,
                            out MechanicalDesignGeometricPresentationRepresentation repr,
                            StepModel stepModel)
    {
        repr = null;
        if (_styles.Extent() < 1)
        {
            return false;
        }

        // create MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION
        NCollection_HArray1<RepresentationItem> elems =
            new NCollection_HArray1<RepresentationItem>(1, _styles.Extent());
        for (int i = 1; i <= _styles.Extent(); i++)
        {
            elems.SetValue(i, _styles.FindKey(i) as RepresentationItem);
        }
        // create new MDGPR
        repr = new MechanicalDesignGeometricPresentationRepresentation();
        TCollection_HAsciiString reprName = new TCollection_HAsciiString("");
        repr.Init(reprName, elems, context);

        // record Repr in order to have it written to the file
        //   Model()->AddWithRefs ( Repr ); add into the model upper

        // for AP203, add subschema name
        if (stepModel.InternalParameters.WriteSchema == Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP203)
        {
            MakeHeader mkHdr = new MakeHeader(Model() as StepModel);
            TCollection_HAsciiString subSchema = new TCollection_HAsciiString("SHAPE_APPEARANCE_LAYER_MIM");
            mkHdr.AddSchemaIdentifier(subSchema);
        }

        return true;
    }

    /// <summary>
    /// Create MDGPR, fill it with all the styles previously defined, and add it to the model IMPORTANT: &lt;initPDS&gt;
    /// must be null when use for NAUO colors &lt;initPDS&gt; initialised only for SHUO case.
    /// </summary>
    public bool CreateNAUOSRD(RepresentationContext context, StepContextDependentShapeRepresentation cdsr,
                              ProductDefinitionShape initPDS)
    {
        StepShapeDefinitionRepresentation sdr = new StepShapeDefinitionRepresentation();
        StepShapeRepresentation sr = new StepShapeRepresentation();
        TCollection_HAsciiString reprName = new TCollection_HAsciiString("");
        // element for axis 2 placement
        NCollection_HArray1<RepresentationItem> elems = new NCollection_HArray1<RepresentationItem>(1, 1);
        // get PDS
        ProductDefinitionShape pds;
        if (initPDS is null)
        {
            pds = cdsr.RepresentedProductRelation();
        }
        else
        {
            pds = initPDS; // for SHUO
        }
        ShapeRepresentationRelationship repRelationShip = cdsr.RepresentationRelation();
        RepresentationRelationshipWithTransformation rRwTRSF =
            repRelationShip as RepresentationRelationshipWithTransformation;
        Transformation setReprTRSF;
        if (rRwTRSF is not null)
        {
            setReprTRSF = rRwTRSF.TransformationOperator();
        }
        else
        {
            return false;
        }
        // take Item defined transformation
        ItemDefinedTransformation itDT = setReprTRSF.ItemDefinedTransformation();
        elems.SetValue(1, itDT.TransformItem2());
        // init Shape representation.
        sr.Init(reprName, elems, context);
        // register reference between PresentationStyleByContext and ShapeRepresentation
        for (int psbci = 1; psbci <= _psa.Length(); psbci++)
        {
            PresentationStyleByContext psa = _psa.Value(psbci) as PresentationStyleByContext;
            if (psa is null)
            {
                continue;
            }
            // register the reference
            StyleContextSelect styleCntxSlct = new StyleContextSelect();
            styleCntxSlct.SetValue(sr);
            psa.SetStyleContext(styleCntxSlct);
        }

        RepresentedDefinition pdSselect = new RepresentedDefinition();
        pdSselect.SetValue(pds);
        sdr.Init(pdSselect, sr);
        Model().AddWithRefs(sdr);

        return true;
    }

    /// <summary>
    /// Searches the STEP model for the RepresentationContext in which given shape is defined. This context (if found)
    /// can be used then in call to CreateMDGPR()
    /// </summary>
    public RepresentationContext FindContext(Shape shape)
    {
        // find context of items
        RepresentationContext context = null;
        ShapeMapper mapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(FinderProcess(), shape);
        StepShapeRepresentation sr = null;
        if (FinderProcess().FindTypedTransient(mapper, typeof(StepShapeRepresentation), ref sr))
        {
            context = sr.ContextOfItems();
        }
        else
        {
            StepGeometricRepresentationItem item = null;
            if (FinderProcess().FindTypedTransient(mapper, typeof(StepGeometricRepresentationItem), ref item))
            {
                EntityIterator subs = Graph().Sharings(item);
                for (subs.Start(); context is null && subs.More(); subs.Next())
                {
                    if (!subs.Value().IsKind(typeof(StepShapeRepresentation)))
                    {
                        continue;
                    }
                    sr = subs.Value() as StepShapeRepresentation;
                    context = sr.ContextOfItems();
                }
            }
        }
        // OCCT_DEBUG output left out

        return context;
    }

    /// <summary>Searches the STEP model for the MDGPR or DM entities (which bring styles) and fills sequence of styles</summary>
    public bool LoadStyles()
    {
        _styles.Clear();
        _psa.Clear();
        _rootStyles.Clear();

        // find all MDGPRs and DMs and collect all defined styles in myStyles
        InterfaceModel model = Model();
        int nb = model.EntityCount;
        Type tMDGPR = typeof(MechanicalDesignGeometricPresentationRepresentation);
        Type tDM = typeof(DraughtingModel);
        Type tSR = typeof(StepShapeRepresentation);
        for (int i = 1; i <= nb; i++)
        {
            Transient enti = model.Value(i);
            if (enti.DynamicType() == tMDGPR || enti.DynamicType() == tDM)
            {
                StepRepresentation container = enti as StepRepresentation;

                int nbi = container.NbItems();
                for (int j = 1; j <= nbi; j++)
                {
                    StyledItem style = container.ItemsValue(j) as StyledItem;
                    if (style is null)
                    {
                        continue;
                    }
                    Transient item = style.ItemAP242()?.Value();
                    if (item is not null && item.IsKind(tSR))
                    {
                        _rootStyles.Add(style);
                    }
                    else
                    {
                        _styles.Add(style);
                    }
                }
            }
            else if (enti.IsKind(typeof(StyledItem)))
            {
                StyledItem styledItem = enti as StyledItem;
                Transient item = styledItem.ItemAP242()?.Value();
                if (item is not null && item.IsKind(tSR) && !_rootStyles.Contains(styledItem))
                {
                    _rootStyles.Add(styledItem);
                }
                else if (!_styles.Contains(styledItem))
                {
                    _styles.Add(styledItem);
                }
            }
        }
        return _styles.Extent() > 0;
    }

    /// <summary>Searches the STEP model for the INISIBILITY entities (which bring styles) and fills out sequence of styles</summary>
    public bool LoadInvisStyles(ref NCollection_HSequence<Transient> invStyles)
    {
        InterfaceModel model = Model();
        int nb = model.EntityCount;
        Type tInvisibility = typeof(Invisibility);
        // search for invisibility
        for (int i = 1; i <= nb; i++)
        {
            Transient enti = model.Value(i);
            if (enti.DynamicType() != tInvisibility)
            {
                continue;
            }
            // search for styled items
            Invisibility container = enti as Invisibility;
            int nbi = container.InvisibleItemCount;
            for (int j = 1; j <= nbi; j++)
            {
                InvisibleItem invItem = container.InvisibleItemsValue(j);
                StyledItem style = invItem?.StyledItem();
                if (style is null)
                {
                    continue;
                }
                // collect the invisible styled items
                invStyles ??= new NCollection_HSequence<Transient>();
                invStyles.Append(style);
            }
        }
        return invStyles is not null && invStyles.Length() > 0;
    }

    /// <summary>
    /// Create a PresentationStyleAssignment entity which defines two colors (for filling surfaces and curves) if
    /// isForNAUO true then returns PresentationStyleByContext
    /// </summary>
    public PresentationStyleAssignment MakeColorPSA(RepresentationItem item, Colour surfCol,
                                                               Colour curveCol,
                                                               RenderingProperties renderingProps,
                                                               bool isForNAUO = false)
    {
        _ = item;
        PresentationStyleAssignment psa = null;
        NCollection_Sequence<Transient> items = new NCollection_Sequence<Transient>();

        // surface color
        if (surfCol is not null)
        {
            TCollection_HAsciiString fascName = new TCollection_HAsciiString("");
            FillAreaStyleColour fasc = new FillAreaStyleColour();
            fasc.Init(fascName, surfCol);

            FillStyleSelect fss = new FillStyleSelect();
            fss.SetValue(fasc);

            NCollection_HArray1<FillStyleSelect> fasSs = new NCollection_HArray1<FillStyleSelect>(1, 1);
            fasSs.SetValue(1, fss);

            TCollection_HAsciiString fasName = new TCollection_HAsciiString("");
            FillAreaStyle fas = new FillAreaStyle();
            fas.Init(fasName, fasSs);

            SurfaceStyleFillArea ssfa = new SurfaceStyleFillArea();
            ssfa.Init(fas);

            SurfaceStyleElementSelect ses = new SurfaceStyleElementSelect();
            ses.SetValue(ssfa);

            SurfaceStyleRenderingWithProperties ssrwp = renderingProps.CreateRenderingProperties(surfCol);

            NCollection_HArray1<SurfaceStyleElementSelect> sseSs;
            if (ssrwp is null)
            {
                sseSs = new NCollection_HArray1<SurfaceStyleElementSelect>(1, 1);
            }
            else
            {
                SurfaceStyleElementSelect sesr = new SurfaceStyleElementSelect();
                sesr.SetValue(ssrwp);
                sseSs = new NCollection_HArray1<SurfaceStyleElementSelect>(1, 2);
                sseSs.SetValue(2, sesr);
            }
            sseSs.SetValue(1, ses);

            TCollection_HAsciiString sssName = new TCollection_HAsciiString("");
            SurfaceSideStyle sss = new SurfaceSideStyle();
            sss.Init(sssName, sseSs);

            SurfaceStyleUsage ssu = new SurfaceStyleUsage();
            ssu.Init(SurfaceSide.SsBoth, sss);

            items.Append(ssu);
        }

        // curve color
        if (curveCol is not null)
        {
            TCollection_HAsciiString fontName = new TCollection_HAsciiString("continuous");
            DraughtingPreDefinedCurveFont sdpdcf = new DraughtingPreDefinedCurveFont();
            sdpdcf.Init(fontName);

            MeasureValueMember val = new MeasureValueMember();
            val.SetReal(0.1);
            val.SetName("POSITIVE_LENGTH_MEASURE");

            CurveStyleFontSelect scsfs = new CurveStyleFontSelect();
            scsfs.SetValue(sdpdcf);

            SizeSelect ssSelect = new SizeSelect();
            ssSelect.SetValue(val);

            TCollection_HAsciiString scsName = new TCollection_HAsciiString("");
            CurveStyle scs = new CurveStyle();
            scs.Init(scsName, scsfs, ssSelect, curveCol);

            items.Append(scs);
        }

        if (items.Length() < 1)
        {
            // OCCT_DEBUG output left out
            return psa;
        }

        // general part
        NCollection_HArray1<PresentationStyleSelect> psSs =
            new NCollection_HArray1<PresentationStyleSelect>(1, items.Length());
        for (int i = 1; i <= items.Length(); i++)
        {
            PresentationStyleSelect pss = new PresentationStyleSelect();
            pss.SetValue(items.Value(i));
            psSs.SetValue(i, pss);
        }
        if (!isForNAUO)
        {
            psa = new PresentationStyleAssignment();
        }
        else
        {
            psa = new PresentationStyleByContext();
        }
        psa.Init(psSs);

        return psa;
    }

    /// <summary>
    /// Returns a PresentationStyleAssignment entity which defines surface and curve colors as Col. This PSA is either
    /// created or taken from internal map where all PSAs created by this method are remembered.
    /// </summary>
    public PresentationStyleAssignment GetColorPSA(RepresentationItem item, Colour col)
    {
        // if this color already was processed, just use the same PSA, else create new and add it to map
        PresentationStyleAssignment psa;
        if (_mapOfStyles.Contains(col))
        {
            psa = _mapOfStyles.FindFromKey(col) as PresentationStyleAssignment;
        }
        else
        {
            psa = MakeColorPSA(item, col, col, new RenderingProperties());
            _mapOfStyles.Add(col, psa);
        }
        return psa;
    }

    /// <summary>
    /// Extract color definitions from the style entity For each type of color supported, result can be either NULL if
    /// it is not defined by that style, or last definition (if they are 1 or more)
    /// </summary>
    public bool GetColors(StyledItem style, out Colour surfaceColour,
                          out Colour boundaryColour, out Colour curveColour,
                          out RenderingProperties renderingProps, ref bool isComponent)
    {
        surfaceColour = null;
        boundaryColour = null;
        curveColour = null;
        renderingProps = new RenderingProperties();

        // parse on styles
        for (int psaIndex = 1; psaIndex <= style.StyleCount; ++psaIndex)
        {
            PresentationStyleAssignment psa = style.StylesValue(psaIndex);
            if (psa is null || psa.Styles() is null)
            {
                continue;
            }
            isComponent = true;

            for (int pssIndex = 1; pssIndex <= psa.StyleCount; ++pssIndex)
            {
                PresentationStyleSelect pss = psa.StylesValue(pssIndex);
                if (pss is null)
                {
                    continue;
                }
                // PresentationStyleSelect can be of only one of the following types:
                // SurfaceStyleUsage, CurveStyle.
                // So we're using && operator to stop as soon as this type is processed.
                _ = ProcessAsSurfaceStyleUsage(pss, ref surfaceColour, ref boundaryColour, renderingProps)
                    || ProcessAsCurveStyle(pss, ref curveColour);
            }
        }
        return surfaceColour is not null || boundaryColour is not null || curveColour is not null
               || renderingProps.IsDefined;
    }

    // the predefined colours of STEP (OCCT's C++ compares with Quantity_Color's operator==, IsEqual)
    private static string PredefinedName(in Color c)
    {
        if (c.IsEqual(new Color(NocGreen)))
        {
            return "green";
        }
        if (c.IsEqual(new Color(NocRed)))
        {
            return "red";
        }
        if (c.IsEqual(new Color(NocBlue1)))
        {
            return "blue";
        }
        if (c.IsEqual(new Color(NocYellow)))
        {
            return "yellow";
        }
        if (c.IsEqual(new Color(NocMagenta1)))
        {
            return "magenta";
        }
        if (c.IsEqual(new Color(NocCyan1)))
        {
            return "cyan";
        }
        if (c.IsEqual(new Color(NocBlack)))
        {
            return "black";
        }
        if (c.IsEqual(new Color(NocWhite)))
        {
            return "white";
        }
        return null;
    }

    /// <summary>
    /// Create STEP color entity by given Quantity_Color The analysis is performed for whether the color corresponds to
    /// one of standard colors predefined in STEP. In that case, PredefinedColour entity is created instead of RGBColour
    /// </summary>
    public static Colour EncodeColor(in Color c)
    {
        // detect if color corresponds to one of pre-defined colors
        string cName = PredefinedName(c);

        if (cName is not null)
        {
            DraughtingPreDefinedColour colPr = new DraughtingPreDefinedColour();
            PreDefinedItem preDef = new PreDefinedItem();
            preDef.Init(new TCollection_HAsciiString(cName));
            colPr.SetPreDefinedItem(preDef);
            return colPr;
        }
        else
        {
            TCollection_HAsciiString colName = new TCollection_HAsciiString("");
            ColourRgb colRGB = new ColourRgb();
            c.Values(out double r, out double g, out double b, TOC_sRGB);
            colRGB.Init(colName, r, g, b);
            return colRGB;
        }
    }

    /// <summary>
    /// Create STEP color entity by given Quantity_Color The analysis is performed for whether the color corresponds to
    /// one of standard colors predefined in STEP. In that case, PredefinedColour entity is created instead of RGBColour
    /// </summary>
    /// <remarks>OCCT.NET: the RGB colours are keyed by their sRGB values (OCCT: a gp_Pnt of them).</remarks>
    public static Colour EncodeColor(in Color c, Dictionary<string, Transient> dpdCs,
                                                Dictionary<Point3D, Transient> colRGBs)
    {
        // detect if color corresponds to one of pre-defined colors
        string cName = PredefinedName(c);

        if (cName is not null)
        {
            DraughtingPreDefinedColour colPr;
            string name = cName;
            if (dpdCs.ContainsKey(name))
            {
                colPr = dpdCs.GetValue(name) as DraughtingPreDefinedColour;
                if (colPr is not null)
                {
                    return colPr;
                }
            }
            colPr = new DraughtingPreDefinedColour();
            PreDefinedItem preDef = new PreDefinedItem();
            preDef.Init(new TCollection_HAsciiString(cName));
            colPr.SetPreDefinedItem(preDef);
            dpdCs[name] = colPr;
            return colPr;
        }
        else
        {
            ColourRgb colRGB;
            c.Values(out double r, out double g, out double b, TOC_sRGB);
            Point3D p = new Point3D(r, g, b);
            if (colRGBs.ContainsKey(p))
            {
                colRGB = colRGBs.GetValue(p) as ColourRgb;
                if (colRGB is not null)
                {
                    return colRGB;
                }
            }
            TCollection_HAsciiString colName = new TCollection_HAsciiString("");
            colRGB = new ColourRgb();
            colRGB.Init(colName, p.Coord(1), p.Coord(2), p.Coord(3));
            colRGBs[p] = colRGB;
            return colRGB;
        }
    }

    /// <summary>
    /// Decodes STEP color and fills the Quantity_Color. Returns True if OK or False if color is not recognized
    /// </summary>
    /// <remarks>OCCT.NET: <paramref name="col"/> is left unchanged when the colour isn't recognised, as in OCCT.</remarks>
    public static bool DecodeColor(Colour colour, ref Color col)
    {
        if (colour.IsKind(typeof(ColourRgb)))
        {
            ColourRgb rgb = colour as ColourRgb;
            if (rgb.Red() > 1.0 || rgb.Green() > 1.0 || rgb.Blue() > 1.0)
            {
                double norm = rgb.Red();
                if (norm < rgb.Green())
                {
                    norm = rgb.Green();
                }
                if (norm < rgb.Blue())
                {
                    norm = rgb.Blue();
                }
                col = new Color(rgb.Red() / norm, rgb.Green() / norm, rgb.Blue() / norm, TOC_sRGB);
            }
            else
            {
                col = new Color(rgb.Red(), rgb.Green(), rgb.Blue(), TOC_sRGB);
            }
            return true;
        }
        else if (colour.IsKind(typeof(PreDefinedColour)))
        {
            PreDefinedColour pdc = colour as PreDefinedColour;
            PreDefinedItem pdi = pdc.GetPreDefinedItem();
            string name = pdi.Name().ToCString();
            switch (name)
            {
                case "red":
                    col = new Color(NocRed);
                    break;
                case "green":
                    col = new Color(NocGreen);
                    break;
                case "blue":
                    col = new Color(NocBlue1);
                    break;
                case "yellow":
                    col = new Color(NocYellow);
                    break;
                case "magenta":
                    col = new Color(NocMagenta1);
                    break;
                case "cyan":
                    col = new Color(NocCyan1);
                    break;
                case "black":
                    col = new Color(NocBlack);
                    break;
                case "white":
                    col = new Color(NocWhite);
                    break;
                default:
                    // OCCT_DEBUG output left out
                    return false;
            }
            return true;
        }
        return false;
    }

    private readonly NCollection_IndexedDataMap<Transient, Transient> _mapOfStyles =
        new NCollection_IndexedDataMap<Transient, Transient>(ReferenceEqualityComparer.Instance);
    private readonly NCollection_IndexedMap<Transient> _styles =
        new NCollection_IndexedMap<Transient>(ReferenceEqualityComparer.Instance);
    private readonly NCollection_IndexedMap<Transient> _rootStyles =
        new NCollection_IndexedMap<Transient>(ReferenceEqualityComparer.Instance);
    private readonly NCollection_Sequence<Transient> _psa = new NCollection_Sequence<Transient>();
}
