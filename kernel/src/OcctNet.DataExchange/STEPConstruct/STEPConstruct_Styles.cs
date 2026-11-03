// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Styles.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Styles.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Foundation.NameOfColor;
using static Kerf.Engine.Foundation.ColorSpace;
using static Occt.StepVisual_SurfaceSide;

namespace Occt;

/// <summary>
/// Provides a mechanism for reading and writing shape styles (such as color) to and from the STEP file This tool
/// maintains a list of styles, either taking them from STEP model (reading), or filling it by calls to AddStyle or
/// directly (writing). Some methods deal with general structures of styles and presentations in STEP, but there are
/// methods which deal with particular implementation of colors (as described in RP)
/// </summary>
/// <remarks>A C++ value class, a C# class here (decision 1185).</remarks>
public class STEPConstruct_Styles : STEPConstruct_Tool
{
    //=======================================================================
    // function : ProcessAsSurfaceStyleRendering
    // purpose  : Process StepVisual_SurfaceStyleElementSelect to extract a
    //           render color and render trnasparency from it. Returns true,
    //           if theSSES was of type StepVisual_SurfaceStyleRendering
    //           (even if color and transparency data couldn't be extracted
    //           for some reason), otherwise returns false.
    //=======================================================================
    private static bool ProcessAsSurfaceStyleRendering(StepVisual_SurfaceStyleElementSelect theSSES,
                                                       STEPConstruct_RenderingProperties theRenderingProps)
    {
        StepVisual_SurfaceStyleRendering aSSR = theSSES.SurfaceStyleRendering();
        if (aSSR is null)
        {
            return false;
        }
        StepVisual_SurfaceStyleRenderingWithProperties aSSRWP = aSSR as StepVisual_SurfaceStyleRenderingWithProperties;
        if (aSSRWP is null)
        {
            return true;
        }
        theRenderingProps.Init(aSSRWP);
        return theRenderingProps.IsDefined();
    }

    //=======================================================================
    // function : ProcessAsSurfaceStyleBoundary
    // purpose  : Process StepVisual_SurfaceStyleElementSelect to extract a
    //           boundary color from it. Returns true,
    //           if theSSES was of type StepVisual_SurfaceStyleBoundary
    //           (even if boundary color data couldn't be extracted
    //           for some reason), otherwise returns false.
    //=======================================================================
    private static bool ProcessAsSurfaceStyleBoundary(StepVisual_SurfaceStyleElementSelect theSSES,
                                                      ref StepVisual_Colour theBoundaryColour)
    {
        StepVisual_SurfaceStyleBoundary aSSB = theSSES.SurfaceStyleBoundary();
        if (aSSB is null)
        {
            return false;
        }
        StepVisual_CurveStyle aCS = aSSB.StyleOfBoundary();
        if (aCS is null)
        {
            return true;
        }
        theBoundaryColour = aCS.CurveColour();
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
    private static bool ProcessAsSurfaceStyleFillArea(StepVisual_SurfaceStyleElementSelect theSSES,
                                                      StepVisual_SurfaceSide theSide,
                                                      ref StepVisual_Colour theSurfaceColour)
    {
        StepVisual_SurfaceStyleFillArea aSSFA = theSSES.SurfaceStyleFillArea();
        if (aSSFA is null)
        {
            return false;
        }
        StepVisual_FillAreaStyle aFAS = aSSFA.FillArea();
        if (aFAS is null)
        {
            return true;
        }

        for (int aFSSIndex = 1; aFSSIndex <= aFAS.NbFillStyles(); aFSSIndex++)
        {
            StepVisual_FillStyleSelect aFSS = aFAS.FillStylesValue(aFSSIndex);
            StepVisual_FillAreaStyleColour aFASC = aFSS?.FillAreaStyleColour();
            if (aFASC is not null
                // If current surface color is null, we will use negative side color.
                // Otherwise negative side color is ignored.
                && (theSurfaceColour is null || theSide != StepVisual_ssNegative)) // abv 30 Mar 00: trj3_s1-pe.stp
            {
                theSurfaceColour = aFASC.FillColour();
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
    private static bool ProcessAsSurfaceStyleUsage(StepVisual_PresentationStyleSelect thePSS,
                                                   ref StepVisual_Colour theSurfaceColour,
                                                   ref StepVisual_Colour theBoundaryColour,
                                                   STEPConstruct_RenderingProperties theRenderingProps)
    {
        StepVisual_SurfaceStyleUsage aSSU = thePSS.SurfaceStyleUsage();
        if (aSSU is null)
        {
            return false;
        }

        StepVisual_SurfaceSideStyle aSSS = aSSU.Style();
        for (int aSSESIndex = 1; aSSESIndex <= aSSS.NbStyles(); ++aSSESIndex)
        {
            StepVisual_SurfaceStyleElementSelect aSSES = aSSS.StylesValue(aSSESIndex);
            // SurfaceStyleElementSelect can be of only one of the following types:
            // SurfaceStyleFillArea, SurfaceStyleBoundary, SurfaceStyleRendering.
            // So we're using && operator to stop as soon as this type is processed.
            _ = ProcessAsSurfaceStyleFillArea(aSSES, aSSU.Side(), ref theSurfaceColour)
                || ProcessAsSurfaceStyleBoundary(aSSES, ref theBoundaryColour)
                || ProcessAsSurfaceStyleRendering(aSSES, theRenderingProps);
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
    private static bool ProcessAsCurveStyle(StepVisual_PresentationStyleSelect thePSS, ref StepVisual_Colour theCurveColour)
    {
        StepVisual_CurveStyle aCS = thePSS.CurveStyle();
        if (aCS is null)
        {
            return false;
        }

        theCurveColour = aCS.CurveColour();
        return true;
    }

    /// <summary>Creates an empty tool</summary>
    public STEPConstruct_Styles()
    {
    }

    /// <summary>Creates a tool and initializes it</summary>
    public STEPConstruct_Styles(XSControl_WorkSession WS) : base(WS)
    {
    }

    /// <summary>Initializes tool; returns True if succeeded</summary>
    public bool Init(XSControl_WorkSession WS)
    {
        myMapOfStyles.Clear();
        myStyles.Clear();
        myPSA.Clear();
        return SetWS(WS);
    }

    /// <summary>Returns number of defined styles</summary>
    public int NbStyles()
    {
        return myStyles.Extent();
    }

    /// <summary>Returns style with given index</summary>
    public StepVisual_StyledItem Style(int i)
    {
        return myStyles.FindKey(i) as StepVisual_StyledItem;
    }

    /// <summary>Returns number of override styles</summary>
    public int NbRootStyles()
    {
        return myRootStyles.Extent();
    }

    /// <summary>Returns override style with given index</summary>
    public StepVisual_StyledItem RootStyle(int i)
    {
        return myRootStyles.FindKey(i) as StepVisual_StyledItem;
    }

    /// <summary>Clears all defined styles and PSA sequence</summary>
    public void ClearStyles()
    {
        myStyles.Clear();
        myPSA.Clear();
        myRootStyles.Clear();
    }

    /// <summary>Adds a style to a sequence</summary>
    public void AddStyle(StepVisual_StyledItem style)
    {
        myStyles.Add(style);
    }

    /// <summary>
    /// Create a style linking giving PSA to the item, and add it to the sequence of stored styles. If Override is not
    /// Null, then the resulting style will be of the subtype OverridingStyledItem.
    /// </summary>
    public StepVisual_StyledItem AddStyle(StepRepr_RepresentationItem item, StepVisual_PresentationStyleAssignment PSA,
                                          StepVisual_StyledItem Override)
    {
        StepVisual_StyledItem Style;

        NCollection_HArray1<StepVisual_PresentationStyleAssignment> Styles =
            new NCollection_HArray1<StepVisual_PresentationStyleAssignment>(1, 1);
        Styles.SetValue(1, PSA);

        if (Override is null)
        {
            TCollection_HAsciiString StyName = new TCollection_HAsciiString("color");
            Style = new StepVisual_StyledItem();
            Style.Init(StyName, Styles, item);
        }
        else
        {
            TCollection_HAsciiString StyName = new TCollection_HAsciiString("overriding color");
            StepVisual_OverRidingStyledItem OStyle = new StepVisual_OverRidingStyledItem();
            OStyle.Init(StyName, Styles, item, Override);
            Style = OStyle;
        }

        myStyles.Add(Style);
        // for future using
        myPSA.Append(PSA);

        return Style;
    }

    /// <summary>
    /// Create a style linking giving PSA to the Shape, and add it to the sequence of stored styles. If Override is not
    /// Null, then the resulting style will be of the subtype OverridingStyledItem. The Sape is used to find
    /// corresponding STEP entity by call to STEPConstruct::FindEntity(), then previous method is called.
    /// </summary>
    public StepVisual_StyledItem AddStyle(TopoDS_Shape Shape, StepVisual_PresentationStyleAssignment PSA,
                                          StepVisual_StyledItem Override)
    {
        StepRepr_RepresentationItem item = STEPConstruct.FindEntity(FinderProcess(), Shape);
        StepVisual_StyledItem Style = null;
        if (item is not null)
        {
            Style = AddStyle(item, PSA, Override);
        }
        return Style;
    }

    /// <summary>Create MDGPR, fill it with all the styles previously defined, and add it to the model</summary>
    public bool CreateMDGPR(StepRepr_RepresentationContext Context,
                            out StepVisual_MechanicalDesignGeometricPresentationRepresentation Repr,
                            StepData_StepModel theStepModel)
    {
        Repr = null;
        if (myStyles.Extent() < 1)
        {
            return false;
        }

        // create MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION
        NCollection_HArray1<StepRepr_RepresentationItem> elems =
            new NCollection_HArray1<StepRepr_RepresentationItem>(1, myStyles.Extent());
        for (int i = 1; i <= myStyles.Extent(); i++)
        {
            elems.SetValue(i, myStyles.FindKey(i) as StepRepr_RepresentationItem);
        }
        // create new MDGPR
        Repr = new StepVisual_MechanicalDesignGeometricPresentationRepresentation();
        TCollection_HAsciiString ReprName = new TCollection_HAsciiString("");
        Repr.Init(ReprName, elems, Context);

        // record Repr in order to have it written to the file
        //   Model()->AddWithRefs ( Repr ); add into the model upper

        // for AP203, add subschema name
        if (theStepModel.InternalParameters.WriteSchema == DESTEP_Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP203)
        {
            APIHeaderSection_MakeHeader mkHdr = new APIHeaderSection_MakeHeader(Model() as StepData_StepModel);
            TCollection_HAsciiString subSchema = new TCollection_HAsciiString("SHAPE_APPEARANCE_LAYER_MIM");
            mkHdr.AddSchemaIdentifier(subSchema);
        }

        return true;
    }

    /// <summary>
    /// Create MDGPR, fill it with all the styles previously defined, and add it to the model IMPORTANT: &lt;initPDS&gt;
    /// must be null when use for NAUO colors &lt;initPDS&gt; initialised only for SHUO case.
    /// </summary>
    public bool CreateNAUOSRD(StepRepr_RepresentationContext Context, StepShape_ContextDependentShapeRepresentation CDSR,
                              StepRepr_ProductDefinitionShape initPDS)
    {
        StepShape_ShapeDefinitionRepresentation aSDR = new StepShape_ShapeDefinitionRepresentation();
        StepShape_ShapeRepresentation aSR = new StepShape_ShapeRepresentation();
        TCollection_HAsciiString ReprName = new TCollection_HAsciiString("");
        // element for axis 2 placement
        NCollection_HArray1<StepRepr_RepresentationItem> elems = new NCollection_HArray1<StepRepr_RepresentationItem>(1, 1);
        // get PDS
        StepRepr_ProductDefinitionShape aPDS;
        if (initPDS is null)
        {
            aPDS = CDSR.RepresentedProductRelation();
        }
        else
        {
            aPDS = initPDS; // for SHUO
        }
        StepRepr_ShapeRepresentationRelationship aRepRelationShip = CDSR.RepresentationRelation();
        StepRepr_RepresentationRelationshipWithTransformation aRRwTRSF =
            aRepRelationShip as StepRepr_RepresentationRelationshipWithTransformation;
        StepRepr_Transformation SetReprTRSF;
        if (aRRwTRSF is not null)
        {
            SetReprTRSF = aRRwTRSF.TransformationOperator();
        }
        else
        {
            return false;
        }
        // take Item defined transformation
        StepRepr_ItemDefinedTransformation anItDT = SetReprTRSF.ItemDefinedTransformation();
        elems.SetValue(1, anItDT.TransformItem2());
        // init Shape representation.
        aSR.Init(ReprName, elems, Context);
        // register reference between PresentationStyleByContext and ShapeRepresentation
        for (int psbci = 1; psbci <= myPSA.Length(); psbci++)
        {
            StepVisual_PresentationStyleByContext PSA = myPSA.Value(psbci) as StepVisual_PresentationStyleByContext;
            if (PSA is null)
            {
                continue;
            }
            // register the reference
            StepVisual_StyleContextSelect aStyleCntxSlct = new StepVisual_StyleContextSelect();
            aStyleCntxSlct.SetValue(aSR);
            PSA.SetStyleContext(aStyleCntxSlct);
        }

        StepRepr_RepresentedDefinition aPDSselect = new StepRepr_RepresentedDefinition();
        aPDSselect.SetValue(aPDS);
        aSDR.Init(aPDSselect, aSR);
        Model().AddWithRefs(aSDR);

        return true;
    }

    /// <summary>
    /// Searches the STEP model for the RepresentationContext in which given shape is defined. This context (if found)
    /// can be used then in call to CreateMDGPR()
    /// </summary>
    public StepRepr_RepresentationContext FindContext(TopoDS_Shape Shape)
    {
        // find context of items
        StepRepr_RepresentationContext Context = null;
        TransferBRep_ShapeMapper mapper = TransferBRep.ShapeMapper(FinderProcess(), Shape);
        StepShape_ShapeRepresentation sr = null;
        if (FinderProcess().FindTypedTransient(mapper, typeof(StepShape_ShapeRepresentation), ref sr))
        {
            Context = sr.ContextOfItems();
        }
        else
        {
            StepGeom_GeometricRepresentationItem item = null;
            if (FinderProcess().FindTypedTransient(mapper, typeof(StepGeom_GeometricRepresentationItem), ref item))
            {
                Interface_EntityIterator subs = Graph().Sharings(item);
                for (subs.Start(); Context is null && subs.More(); subs.Next())
                {
                    if (!subs.Value().IsKind(typeof(StepShape_ShapeRepresentation)))
                    {
                        continue;
                    }
                    sr = subs.Value() as StepShape_ShapeRepresentation;
                    Context = sr.ContextOfItems();
                }
            }
        }
        // OCCT_DEBUG output left out

        return Context;
    }

    /// <summary>Searches the STEP model for the MDGPR or DM entities (which bring styles) and fills sequence of styles</summary>
    public bool LoadStyles()
    {
        myStyles.Clear();
        myPSA.Clear();
        myRootStyles.Clear();

        // find all MDGPRs and DMs and collect all defined styles in myStyles
        Interface_InterfaceModel model = Model();
        int nb = model.NbEntities();
        Type tMDGPR = typeof(StepVisual_MechanicalDesignGeometricPresentationRepresentation);
        Type tDM = typeof(StepVisual_DraughtingModel);
        Type tSR = typeof(StepShape_ShapeRepresentation);
        for (int i = 1; i <= nb; i++)
        {
            Transient enti = model.Value(i);
            if (enti.DynamicType() == tMDGPR || enti.DynamicType() == tDM)
            {
                StepRepr_Representation container = enti as StepRepr_Representation;

                int nbi = container.NbItems();
                for (int j = 1; j <= nbi; j++)
                {
                    StepVisual_StyledItem style = container.ItemsValue(j) as StepVisual_StyledItem;
                    if (style is null)
                    {
                        continue;
                    }
                    Transient anItem = style.ItemAP242()?.Value();
                    if (anItem is not null && anItem.IsKind(tSR))
                    {
                        myRootStyles.Add(style);
                    }
                    else
                    {
                        myStyles.Add(style);
                    }
                }
            }
            else if (enti.IsKind(typeof(StepVisual_StyledItem)))
            {
                StepVisual_StyledItem aStyledItem = enti as StepVisual_StyledItem;
                Transient anItem = aStyledItem.ItemAP242()?.Value();
                if (anItem is not null && anItem.IsKind(tSR) && !myRootStyles.Contains(aStyledItem))
                {
                    myRootStyles.Add(aStyledItem);
                }
                else if (!myStyles.Contains(aStyledItem))
                {
                    myStyles.Add(aStyledItem);
                }
            }
        }
        return myStyles.Extent() > 0;
    }

    /// <summary>Searches the STEP model for the INISIBILITY entities (which bring styles) and fills out sequence of styles</summary>
    public bool LoadInvisStyles(ref NCollection_HSequence<Transient> theInvStyles)
    {
        Interface_InterfaceModel model = Model();
        int nb = model.NbEntities();
        Type tInvisibility = typeof(StepVisual_Invisibility);
        // search for invisibility
        for (int i = 1; i <= nb; i++)
        {
            Transient enti = model.Value(i);
            if (enti.DynamicType() != tInvisibility)
            {
                continue;
            }
            // search for styled items
            StepVisual_Invisibility container = enti as StepVisual_Invisibility;
            int nbi = container.NbInvisibleItems();
            for (int j = 1; j <= nbi; j++)
            {
                StepVisual_InvisibleItem anInvItem = container.InvisibleItemsValue(j);
                StepVisual_StyledItem style = anInvItem?.StyledItem();
                if (style is null)
                {
                    continue;
                }
                // collect the invisible styled items
                theInvStyles ??= new NCollection_HSequence<Transient>();
                theInvStyles.Append(style);
            }
        }
        return theInvStyles is not null && theInvStyles.Length() > 0;
    }

    /// <summary>
    /// Create a PresentationStyleAssignment entity which defines two colors (for filling surfaces and curves) if
    /// isForNAUO true then returns PresentationStyleByContext
    /// </summary>
    public StepVisual_PresentationStyleAssignment MakeColorPSA(StepRepr_RepresentationItem item, StepVisual_Colour SurfCol,
                                                               StepVisual_Colour CurveCol,
                                                               STEPConstruct_RenderingProperties theRenderingProps,
                                                               bool isForNAUO = false)
    {
        _ = item;
        StepVisual_PresentationStyleAssignment PSA = null;
        NCollection_Sequence<Transient> items = new NCollection_Sequence<Transient>();

        // surface color
        if (SurfCol is not null)
        {
            TCollection_HAsciiString FASCName = new TCollection_HAsciiString("");
            StepVisual_FillAreaStyleColour FASC = new StepVisual_FillAreaStyleColour();
            FASC.Init(FASCName, SurfCol);

            StepVisual_FillStyleSelect FSS = new StepVisual_FillStyleSelect();
            FSS.SetValue(FASC);

            NCollection_HArray1<StepVisual_FillStyleSelect> FASSs = new NCollection_HArray1<StepVisual_FillStyleSelect>(1, 1);
            FASSs.SetValue(1, FSS);

            TCollection_HAsciiString FASName = new TCollection_HAsciiString("");
            StepVisual_FillAreaStyle FAS = new StepVisual_FillAreaStyle();
            FAS.Init(FASName, FASSs);

            StepVisual_SurfaceStyleFillArea SSFA = new StepVisual_SurfaceStyleFillArea();
            SSFA.Init(FAS);

            StepVisual_SurfaceStyleElementSelect SES = new StepVisual_SurfaceStyleElementSelect();
            SES.SetValue(SSFA);

            StepVisual_SurfaceStyleRenderingWithProperties aSSRWP = theRenderingProps.CreateRenderingProperties(SurfCol);

            NCollection_HArray1<StepVisual_SurfaceStyleElementSelect> SSESs;
            if (aSSRWP is null)
            {
                SSESs = new NCollection_HArray1<StepVisual_SurfaceStyleElementSelect>(1, 1);
            }
            else
            {
                StepVisual_SurfaceStyleElementSelect aSESR = new StepVisual_SurfaceStyleElementSelect();
                aSESR.SetValue(aSSRWP);
                SSESs = new NCollection_HArray1<StepVisual_SurfaceStyleElementSelect>(1, 2);
                SSESs.SetValue(2, aSESR);
            }
            SSESs.SetValue(1, SES);

            TCollection_HAsciiString SSSName = new TCollection_HAsciiString("");
            StepVisual_SurfaceSideStyle SSS = new StepVisual_SurfaceSideStyle();
            SSS.Init(SSSName, SSESs);

            StepVisual_SurfaceStyleUsage SSU = new StepVisual_SurfaceStyleUsage();
            SSU.Init(StepVisual_ssBoth, SSS);

            items.Append(SSU);
        }

        // curve color
        if (CurveCol is not null)
        {
            TCollection_HAsciiString fontName = new TCollection_HAsciiString("continuous");
            StepVisual_DraughtingPreDefinedCurveFont SDPDCF = new StepVisual_DraughtingPreDefinedCurveFont();
            SDPDCF.Init(fontName);

            StepBasic_MeasureValueMember Val = new StepBasic_MeasureValueMember();
            Val.SetReal(0.1);
            Val.SetName("POSITIVE_LENGTH_MEASURE");

            StepVisual_CurveStyleFontSelect SCSFS = new StepVisual_CurveStyleFontSelect();
            SCSFS.SetValue(SDPDCF);

            StepBasic_SizeSelect SSSelect = new StepBasic_SizeSelect();
            SSSelect.SetValue(Val);

            TCollection_HAsciiString SCSName = new TCollection_HAsciiString("");
            StepVisual_CurveStyle SCS = new StepVisual_CurveStyle();
            SCS.Init(SCSName, SCSFS, SSSelect, CurveCol);

            items.Append(SCS);
        }

        if (items.Length() < 1)
        {
            // OCCT_DEBUG output left out
            return PSA;
        }

        // general part
        NCollection_HArray1<StepVisual_PresentationStyleSelect> PSSs =
            new NCollection_HArray1<StepVisual_PresentationStyleSelect>(1, items.Length());
        for (int i = 1; i <= items.Length(); i++)
        {
            StepVisual_PresentationStyleSelect PSS = new StepVisual_PresentationStyleSelect();
            PSS.SetValue(items.Value(i));
            PSSs.SetValue(i, PSS);
        }
        if (!isForNAUO)
        {
            PSA = new StepVisual_PresentationStyleAssignment();
        }
        else
        {
            PSA = new StepVisual_PresentationStyleByContext();
        }
        PSA.Init(PSSs);

        return PSA;
    }

    /// <summary>
    /// Returns a PresentationStyleAssignment entity which defines surface and curve colors as Col. This PSA is either
    /// created or taken from internal map where all PSAs created by this method are remembered.
    /// </summary>
    public StepVisual_PresentationStyleAssignment GetColorPSA(StepRepr_RepresentationItem item, StepVisual_Colour Col)
    {
        // if this color already was processed, just use the same PSA, else create new and add it to map
        StepVisual_PresentationStyleAssignment PSA;
        if (myMapOfStyles.Contains(Col))
        {
            PSA = myMapOfStyles.FindFromKey(Col) as StepVisual_PresentationStyleAssignment;
        }
        else
        {
            PSA = MakeColorPSA(item, Col, Col, new STEPConstruct_RenderingProperties());
            myMapOfStyles.Add(Col, PSA);
        }
        return PSA;
    }

    /// <summary>
    /// Extract color definitions from the style entity For each type of color supported, result can be either NULL if
    /// it is not defined by that style, or last definition (if they are 1 or more)
    /// </summary>
    public bool GetColors(StepVisual_StyledItem theStyle, out StepVisual_Colour theSurfaceColour,
                          out StepVisual_Colour theBoundaryColour, out StepVisual_Colour theCurveColour,
                          out STEPConstruct_RenderingProperties theRenderingProps, ref bool theIsComponent)
    {
        theSurfaceColour = null;
        theBoundaryColour = null;
        theCurveColour = null;
        theRenderingProps = new STEPConstruct_RenderingProperties();

        // parse on styles
        for (int aPSAIndex = 1; aPSAIndex <= theStyle.NbStyles(); ++aPSAIndex)
        {
            StepVisual_PresentationStyleAssignment aPSA = theStyle.StylesValue(aPSAIndex);
            if (aPSA is null || aPSA.Styles() is null)
            {
                continue;
            }
            theIsComponent = true;

            for (int aPSSIndex = 1; aPSSIndex <= aPSA.NbStyles(); ++aPSSIndex)
            {
                StepVisual_PresentationStyleSelect aPSS = aPSA.StylesValue(aPSSIndex);
                if (aPSS is null)
                {
                    continue;
                }
                // PresentationStyleSelect can be of only one of the following types:
                // SurfaceStyleUsage, CurveStyle.
                // So we're using && operator to stop as soon as this type is processed.
                _ = ProcessAsSurfaceStyleUsage(aPSS, ref theSurfaceColour, ref theBoundaryColour, theRenderingProps)
                    || ProcessAsCurveStyle(aPSS, ref theCurveColour);
            }
        }
        return theSurfaceColour is not null || theBoundaryColour is not null || theCurveColour is not null
               || theRenderingProps.IsDefined();
    }

    // the predefined colours of STEP (OCCT's C++ compares with Quantity_Color's operator==, IsEqual)
    private static string predefinedName(in Color C)
    {
        if (C.IsEqual(new Color(NocGreen)))
        {
            return "green";
        }
        if (C.IsEqual(new Color(NocRed)))
        {
            return "red";
        }
        if (C.IsEqual(new Color(NocBlue1)))
        {
            return "blue";
        }
        if (C.IsEqual(new Color(NocYellow)))
        {
            return "yellow";
        }
        if (C.IsEqual(new Color(NocMagenta1)))
        {
            return "magenta";
        }
        if (C.IsEqual(new Color(NocCyan1)))
        {
            return "cyan";
        }
        if (C.IsEqual(new Color(NocBlack)))
        {
            return "black";
        }
        if (C.IsEqual(new Color(NocWhite)))
        {
            return "white";
        }
        return null;
    }

    /// <summary>
    /// Create STEP color entity by given Quantity_Color The analysis is performed for whether the color corresponds to
    /// one of standard colors predefined in STEP. In that case, PredefinedColour entity is created instead of RGBColour
    /// </summary>
    public static StepVisual_Colour EncodeColor(in Color C)
    {
        // detect if color corresponds to one of pre-defined colors
        string cName = predefinedName(C);

        if (cName is not null)
        {
            StepVisual_DraughtingPreDefinedColour ColPr = new StepVisual_DraughtingPreDefinedColour();
            StepVisual_PreDefinedItem preDef = new StepVisual_PreDefinedItem();
            preDef.Init(new TCollection_HAsciiString(cName));
            ColPr.SetPreDefinedItem(preDef);
            return ColPr;
        }
        else
        {
            TCollection_HAsciiString ColName = new TCollection_HAsciiString("");
            StepVisual_ColourRgb ColRGB = new StepVisual_ColourRgb();
            C.Values(out double aR, out double aG, out double aB, TOC_sRGB);
            ColRGB.Init(ColName, aR, aG, aB);
            return ColRGB;
        }
    }

    /// <summary>
    /// Create STEP color entity by given Quantity_Color The analysis is performed for whether the color corresponds to
    /// one of standard colors predefined in STEP. In that case, PredefinedColour entity is created instead of RGBColour
    /// </summary>
    /// <remarks>OCCT.NET: the RGB colours are keyed by their sRGB values (OCCT: a gp_Pnt of them).</remarks>
    public static StepVisual_Colour EncodeColor(in Color C, NCollection_DataMap<string, Transient> DPDCs,
                                                NCollection_DataMap<Point3D, Transient> ColRGBs)
    {
        // detect if color corresponds to one of pre-defined colors
        string cName = predefinedName(C);

        if (cName is not null)
        {
            StepVisual_DraughtingPreDefinedColour ColPr;
            string aName = cName;
            if (DPDCs.IsBound(aName))
            {
                ColPr = DPDCs.Find(aName) as StepVisual_DraughtingPreDefinedColour;
                if (ColPr is not null)
                {
                    return ColPr;
                }
            }
            ColPr = new StepVisual_DraughtingPreDefinedColour();
            StepVisual_PreDefinedItem preDef = new StepVisual_PreDefinedItem();
            preDef.Init(new TCollection_HAsciiString(cName));
            ColPr.SetPreDefinedItem(preDef);
            DPDCs.Bind(aName, ColPr);
            return ColPr;
        }
        else
        {
            StepVisual_ColourRgb ColRGB;
            C.Values(out double aR, out double aG, out double aB, TOC_sRGB);
            Point3D P = new Point3D(aR, aG, aB);
            if (ColRGBs.IsBound(P))
            {
                ColRGB = ColRGBs.Find(P) as StepVisual_ColourRgb;
                if (ColRGB is not null)
                {
                    return ColRGB;
                }
            }
            TCollection_HAsciiString ColName = new TCollection_HAsciiString("");
            ColRGB = new StepVisual_ColourRgb();
            ColRGB.Init(ColName, P.Coord(1), P.Coord(2), P.Coord(3));
            ColRGBs.Bind(P, ColRGB);
            return ColRGB;
        }
    }

    /// <summary>
    /// Decodes STEP color and fills the Quantity_Color. Returns True if OK or False if color is not recognized
    /// </summary>
    /// <remarks>OCCT.NET: <paramref name="Col"/> is left unchanged when the colour isn't recognised, as in OCCT.</remarks>
    public static bool DecodeColor(StepVisual_Colour Colour, ref Color Col)
    {
        if (Colour.IsKind(typeof(StepVisual_ColourRgb)))
        {
            StepVisual_ColourRgb rgb = Colour as StepVisual_ColourRgb;
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
                Col = new Color(rgb.Red() / norm, rgb.Green() / norm, rgb.Blue() / norm, TOC_sRGB);
            }
            else
            {
                Col = new Color(rgb.Red(), rgb.Green(), rgb.Blue(), TOC_sRGB);
            }
            return true;
        }
        else if (Colour.IsKind(typeof(StepVisual_PreDefinedColour)))
        {
            StepVisual_PreDefinedColour pdc = Colour as StepVisual_PreDefinedColour;
            StepVisual_PreDefinedItem pdi = pdc.GetPreDefinedItem();
            string name = pdi.Name().ToCString();
            switch (name)
            {
                case "red":
                    Col = new Color(NocRed);
                    break;
                case "green":
                    Col = new Color(NocGreen);
                    break;
                case "blue":
                    Col = new Color(NocBlue1);
                    break;
                case "yellow":
                    Col = new Color(NocYellow);
                    break;
                case "magenta":
                    Col = new Color(NocMagenta1);
                    break;
                case "cyan":
                    Col = new Color(NocCyan1);
                    break;
                case "black":
                    Col = new Color(NocBlack);
                    break;
                case "white":
                    Col = new Color(NocWhite);
                    break;
                default:
                    // OCCT_DEBUG output left out
                    return false;
            }
            return true;
        }
        return false;
    }

    private readonly NCollection_IndexedDataMap<Transient, Transient> myMapOfStyles =
        new NCollection_IndexedDataMap<Transient, Transient>(ReferenceEqualityComparer.Instance);
    private readonly NCollection_IndexedMap<Transient> myStyles =
        new NCollection_IndexedMap<Transient>(ReferenceEqualityComparer.Instance);
    private readonly NCollection_IndexedMap<Transient> myRootStyles =
        new NCollection_IndexedMap<Transient>(ReferenceEqualityComparer.Instance);
    private readonly NCollection_Sequence<Transient> myPSA = new NCollection_Sequence<Transient>();
}
