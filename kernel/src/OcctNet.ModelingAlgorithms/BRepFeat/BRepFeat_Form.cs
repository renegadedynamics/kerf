// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Form.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Form.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Form.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using static Occt.BRepFeat_PerfSelection;
using static Occt.BRepFeat_StatusError;
using static Occt.LocOpe_Operation;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The root of the form features (prisms, draft prisms, revolved features, pipes): the topological reconstruction of
/// the result, by gluing the feature on the basis shape where it can, by a local Boolean operation otherwise; and the
/// history of the basis shape's faces.
/// </summary>
/// <remarks>
/// <para>Each feature built is a working object (decision 21). <see cref="BRepBuilderAPI_MakeShape.Shape"/>,
/// <see cref="BRepBuilderAPI_MakeShape.Modified"/>, <see cref="BRepBuilderAPI_MakeShape.Generated"/>,
/// <see cref="FirstShape"/>, <see cref="LastShape"/>, <see cref="NewEdges"/> and <see cref="TgtEdges"/> publish what
/// they return (frozen); ported code reads the <c>Internal</c> forms. The history answers for the caller's shapes.</para>
/// <para>The basis shape, the sketch face, the from and until shapes are read, never written: the gluing copies them
/// (LocOpe_Gluer's working shapes, decision 1013), the Booleans are non-destructive (decision 274).</para>
/// </remarks>
public abstract class BRepFeat_Form : BRepBuilderAPI_MakeShape
{
    protected bool myFuse;
    protected bool myModify;
    protected NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMap = new(TopTools_ShapeMapHasher.Instance);
    protected TopoDS_Shape myFShape = new();
    protected TopoDS_Shape myLShape = new();
    protected NCollection_List<TopoDS_Shape> myNewEdges = new();
    protected NCollection_List<TopoDS_Shape> myTgtEdges = new();
    protected BRepFeat_PerfSelection myPerfSelection;
    protected bool myJustGluer;
    protected bool myJustFeat;
    protected TopoDS_Shape mySbase = new();
    protected TopoDS_Face mySkface = new();
    protected TopoDS_Shape myGShape = new();
    protected TopoDS_Shape mySFrom = new();
    protected TopoDS_Shape mySUntil = new();
    protected NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myGluedF = new(TopTools_ShapeMapHasher.Instance);

    private bool mySbOK;
    private bool mySkOK;
    private bool myGSOK;
    private bool mySFOK;
    private bool mySUOK;
    private bool myGFOK;
    private bool myPSOK;
    private BRepFeat_StatusError myStatusError;

    // OCCT.NET: the working copies of the frozen shapes the feature is given (decision 9501)
    private protected LocOpe_WorkingShapes? myWork;

    /// <summary>Redefines the empty constructor.</summary>
    protected BRepFeat_Form()
    {
        myFuse = false;
        myModify = false;
        myPerfSelection = BRepFeat_NoSelection;
        myJustGluer = false;
        myJustFeat = false;
        mySbOK = false;
        mySkOK = false;
        myGSOK = false;
        mySFOK = false;
        mySUOK = false;
        myGFOK = false;
        myPSOK = false;
        myStatusError = BRepFeat_NotInitialized;
    }

    /// <summary>Initializes the topological construction if the basis shape is present.</summary>
    public void BasisShapeValid() => mySbOK = true;

    /// <summary>Initializes the topological construction if the generated shape is present.</summary>
    public void GeneratedShapeValid() => myGSOK = true;

    /// <summary>Initializes the topological construction if the shape is present from the specified integer on.</summary>
    public void ShapeFromValid() => mySFOK = true;

    /// <summary>Initializes the topological construction if the shape is present until the specified integer.</summary>
    public void ShapeUntilValid() => mySUOK = true;

    /// <summary>Initializes the topological construction if the glued face is present.</summary>
    public void GluedFacesValid() => myGFOK = true;

    /// <summary>
    /// Initializes the topological construction if the sketch face is present. If the sketch face is inside the basis
    /// shape, local operations such as glueing can be performed.
    /// </summary>
    public void SketchFaceValid() => mySkOK = true;

    /// <summary>Initializes the topological construction if the selected face is present.</summary>
    public void PerfSelectionValid() => myPSOK = true;

    /// <summary>The curves the feature is intersected along with the from and until shapes.</summary>
    public abstract void Curves(NCollection_Sequence<Geom_Curve> S);

    /// <summary>The curve through the barycentre of the feature, along its direction.</summary>
    public abstract Geom_Curve? BarycCurve();

    /// <summary>The status of the last construction.</summary>
    public BRepFeat_StatusError CurrentStatusError() => myStatusError;

    /// <summary>
    /// OCCT.NET: starts the feature's working shapes (decision 9501). OCCT's feature writes into the shapes it is
    /// given: the Booleans it runs are destructive (they raise tolerances and add pcurves in place) and the sweeps
    /// write into the profile. Every shape entering the feature goes through <see cref="Working(in TopoDS_Shape)"/>, which gives the
    /// exact copy for write of a frozen one, all copies sharing one map (a face of the basis shape given as the sketch
    /// face or the until shape has its one copy); the result, the history and the lists are given back on the caller's
    /// shapes where the feature left them unchanged, at the first publication (<see cref="FinishWorkingShapes"/>).
    /// </summary>
    private protected void StartWorkingShapes() => myWork = new LocOpe_WorkingShapes();

    /// <summary>The working shape standing for <paramref name="S"/> (decision 9501).</summary>
    private protected TopoDS_Shape Working(in TopoDS_Shape S)
    {
        myWork ??= new LocOpe_WorkingShapes();
        return myWork.Working(S);
    }

    /// <summary>The working face standing for <paramref name="F"/> (decision 9501).</summary>
    private protected TopoDS_Face Working(in TopoDS_Face F) => F.IsNull() ? F : TopoDS.Face(Working((TopoDS_Shape)F));

    /// <summary>
    /// OCCT.NET: ends the working shapes (decision 9501): every copy the feature left unchanged is replaced by the
    /// caller's shape in the result and the lists, and the history is keyed by the caller's shapes. Once per
    /// construction; called by every accessor of the result and the history.
    /// </summary>
    private protected virtual void FinishWorkingShapes()
    {
        if (myWork is null || myWork.IsFinished())
        {
            return;
        }
        LocOpe_WorkingShapes aWork = myWork;
        var aShapes = new List<TopoDS_Shape> { myShape };
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anItem in myMap)
        {
            aShapes.AddRange(anItem.Value);
        }
        aShapes.AddRange(myNewEdges);
        aShapes.AddRange(myTgtEdges);
        aWork.Finish(aShapes);
        if (!aWork.IsCopied())
        {
            return;
        }
        myShape = aWork.Final(myShape);
        var aMap = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anItem in myMap)
        {
            TopoDS_Shape aKey = aWork.Original(anItem.Key);
            if (!aMap.IsBound(aKey))
            {
                aMap.Bind(aKey, aWork.Final(anItem.Value));
            }
        }
        myMap = aMap;
        myNewEdges = aWork.Final(myNewEdges);
        myTgtEdges = aWork.Final(myTgtEdges);
        mySbase = aWork.Original(mySbase);
        mySkface = TopoDS.Face(aWork.Original(mySkface));
        mySFrom = aWork.Original(mySFrom);
        mySUntil = aWork.Original(mySUntil);
        myFShape = aWork.Original(myFShape);
        myLShape = aWork.Original(myLShape);
    }

    /// <summary>The result, unfrozen, on the caller's shapes where they are unchanged (OCCT's virtual).</summary>
    internal override TopoDS_Shape InternalShape()
    {
        base.InternalShape();
        FinishWorkingShapes();
        return myShape;
    }

    /// <summary>General perform method: the topological reconstruction of the result.</summary>
    protected void GlobalPerform()
    {
        if (!mySbOK || !myGSOK || !mySFOK || !mySUOK || !myGFOK || !mySkOK || !myPSOK)
        {
            myStatusError = BRepFeat_NotInitialized;
            NotDone();
            return;
        }

        //--- Initialisation
        TopExp_Explorer exp = new(), exp2 = new();
        int theOpe = 2;

        if (myJustFeat && !myFuse)
        {
            myStatusError = BRepFeat_InvOption;
            NotDone();
            return;
        }
        else if (myJustFeat)
        {
            theOpe = 2;
        }
        else if (!myGluedF.IsEmpty())
        {
            theOpe = 1;
        }
        bool ChangeOpe = false;

        bool FromInShape = false;
        bool UntilInShape = false;

        if (!mySFrom.IsNull())
        {
            FromInShape = true;
            for (exp2.Init(mySFrom, TopAbs_FACE); exp2.More(); exp2.Next())
            {
                TopoDS_Shape ffrom = exp2.Current();
                for (exp.Init(mySbase, TopAbs_FACE); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(ffrom))
                    {
                        break;
                    }
                }
                if (!exp.More())
                {
                    FromInShape = false;
                    break;
                }
            }
        }

        if (!mySUntil.IsNull())
        {
            UntilInShape = true;
            for (exp2.Init(mySUntil, TopAbs_FACE); exp2.More(); exp2.Next())
            {
                TopoDS_Shape funtil = exp2.Current();
                for (exp.Init(mySbase, TopAbs_FACE); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(funtil))
                    {
                        break;
                    }
                }
                if (!exp.More())
                {
                    UntilInShape = false;
                    break;
                }
            }
        }

        NCollection_List<TopoDS_Shape>.Iterator it = new();
        int sens = 0;

        NCollection_Sequence<Geom_Curve> scur = new();
        Curves(scur);

        double mf, Mf, mu, Mu;

        TopAbs_Orientation Orifuntil = TopAbs_INTERNAL;
        TopAbs_Orientation Oriffrom = TopAbs_INTERNAL;
        TopoDS_Face FFrom = new(), FUntil = new();

        LocOpe_CSIntersector ASI1 = new();
        LocOpe_CSIntersector ASI2 = new();

        //--- 1) by intersection

        // Intersection Tool Shape From
        if (!mySFrom.IsNull())
        {
            ASI1.Init(mySFrom);
            ASI1.Perform(scur!);
        }

        // Intersection Tool Shape Until
        if (!mySUntil.IsNull())
        {
            ASI2.Init(mySUntil);
            ASI2.Perform(scur!);
        }

        {
            //  Find sens, FFrom, FUntil
            for (int jj = 1; jj <= scur.Length(); jj++)
            {
                if (ASI1.IsDone() && ASI2.IsDone())
                {
                    if (ASI1.NbPoints(jj) <= 0)
                    {
                        continue;
                    }
                    mf = ASI1.Point(jj, 1).Parameter();
                    Mf = ASI1.Point(jj, ASI1.NbPoints(jj)).Parameter();
                    if (ASI2.NbPoints(jj) <= 0)
                    {
                        continue;
                    }
                    mu = ASI2.Point(jj, 1).Parameter();
                    Mu = ASI2.Point(jj, ASI2.NbPoints(jj)).Parameter();
                    if (!scur[jj].IsPeriodic())
                    {
                        int ku, kf;
                        if (mu <= Mf && mf <= Mu)
                        { // overlapping intervals
                            sens = 1;
                            kf = 1;
                            ku = ASI2.NbPoints(jj);
                        }
                        else if (mu > Mf)
                        {
                            if (sens == -1)
                            {
                                myStatusError = BRepFeat_IntervalOverlap;
                                NotDone();
                                return;
                            }
                            sens = 1;
                            kf = 1;
                            ku = ASI2.NbPoints(jj);
                        }
                        else
                        {
                            if (sens == 1)
                            {
                                myStatusError = BRepFeat_IntervalOverlap;
                                NotDone();
                                return;
                            }
                            sens = -1;
                            kf = ASI1.NbPoints(jj);
                            ku = 1;
                        }
                        if (Oriffrom == TopAbs_INTERNAL)
                        {
                            TopAbs_Orientation Oript = ASI1.Point(jj, kf).Orientation();
                            if (Oript == TopAbs_FORWARD || Oript == TopAbs_REVERSED)
                            {
                                if (sens == -1)
                                {
                                    Oript = TopAbs.Reverse(Oript);
                                }
                                Oriffrom = TopAbs.Reverse(Oript);
                                FFrom = ASI1.Point(jj, kf).Face();
                            }
                        }
                        if (Orifuntil == TopAbs_INTERNAL)
                        {
                            TopAbs_Orientation Oript = ASI2.Point(jj, ku).Orientation();
                            if (Oript == TopAbs_FORWARD || Oript == TopAbs_REVERSED)
                            {
                                if (sens == -1)
                                {
                                    Oript = TopAbs.Reverse(Oript);
                                }
                                Orifuntil = Oript;
                                FUntil = ASI2.Point(jj, ku).Face();
                            }
                        }
                    }
                }
                else if (ASI2.IsDone())
                {
                    if (ASI2.NbPoints(jj) <= 0)
                    {
                        continue;
                    }

                    // for base case prism on mySUntil -> ambivalent direction
                    //      ->  preferable direction = 1
                    if (sens != 1)
                    {
                        if (ASI2.Point(jj, 1).Parameter() * ASI2.Point(jj, ASI2.NbPoints(jj)).Parameter() <= 0)
                        {
                            sens = 1;
                        }
                        else if (ASI2.Point(jj, 1).Parameter() < 0.0)
                        {
                            sens = -1;
                        }
                        else
                        {
                            sens = 1;
                        }
                    }

                    int ku;
                    if (sens == -1)
                    {
                        ku = 1;
                    }
                    else
                    {
                        ku = ASI2.NbPoints(jj);
                    }
                    if (Orifuntil == TopAbs_INTERNAL && sens != 0)
                    {
                        TopAbs_Orientation Oript = ASI2.Point(jj, ku).Orientation();
                        if (Oript == TopAbs_FORWARD || Oript == TopAbs_REVERSED)
                        {
                            if (sens == -1)
                            {
                                Oript = TopAbs.Reverse(Oript);
                            }
                            Orifuntil = Oript;
                            FUntil = ASI2.Point(jj, ku).Face();
                        }
                    }
                }
                else
                {
                    sens = 1;
                    break;
                }
            }
        }

        LocOpe_Gluer theGlue = new();

        //--- case of gluing

        if (theOpe == 1)
        {
            bool Collage = true;
            // cut by FFrom && FUntil
            TopoDS_Compound Comp = new();
            BRep_Builder B = new();
            B.MakeCompound(ref Comp);
            if (!mySFrom.IsNull())
            {
                TopoDS_Solid S = BRepFeat.Tool(mySFrom, FFrom, Oriffrom);
                if (!S.IsNull())
                {
                    B.Add(Comp, S);
                }
            }
            if (!mySUntil.IsNull())
            {
                TopoDS_Solid S = BRepFeat.Tool(mySUntil, FUntil, Orifuntil);
                if (!S.IsNull())
                {
                    B.Add(Comp, S);
                }
            }

            LocOpe_FindEdges theFE = new();
            NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> locmap = new(TopTools_ShapeMapHasher.Instance);
            TopExp_Explorer expp = new(Comp, TopAbs_SOLID);
            if (expp.More() && !Comp.IsNull() && !myGShape.IsNull())
            {
                BRepAlgoAPI_Cut trP = new(myGShape, Comp);
                exp.Init(trP.InternalShape(), TopAbs_SOLID);
                if (exp.Current().IsNull())
                {
                    theOpe = 2;
                    ChangeOpe = true;
                    Collage = false;
                }
                else
                { // else X0
                    // Only solids are preserved
                    TopoDS_Compound theGShape = new();
                    B.MakeCompound(ref theGShape);
                    for (; exp.More(); exp.Next())
                    {
                        B.Add(theGShape, exp.Current());
                    }
                    if (!BRepAlgo.IsValid(theGShape))
                    {
                        theOpe = 2;
                        ChangeOpe = true;
                        Collage = false;
                    }
                    else
                    { // else X1
                        if (!mySFrom.IsNull())
                        {
                            TopExp_Explorer ex = new();
                            ex.Init(mySFrom, TopAbs_FACE);
                            for (; ex.More(); ex.Next())
                            {
                                TopoDS_Face fac = TopoDS.Face(ex.Current());
                                if (!FromInShape)
                                {
                                    NCollection_List<TopoDS_Shape> thelist = new();
                                    myMap.Bind(fac, thelist);
                                }
                                else
                                {
                                    NCollection_List<TopoDS_Shape> thelist1 = new();
                                    locmap.Bind(fac, thelist1);
                                }
                                if (trP.IsDeleted(fac))
                                {
                                }
                                else if (!FromInShape)
                                {
                                    myMap[fac] = new NCollection_List<TopoDS_Shape>(trP.InternalModified(fac));
                                    if (myMap[fac].IsEmpty())
                                    {
                                        myMap[fac].Append(fac);
                                    }
                                }
                                else
                                {
                                    locmap[fac] = new NCollection_List<TopoDS_Shape>(trP.InternalModified(fac));
                                    if (locmap[fac].IsEmpty())
                                    {
                                        locmap[fac].Append(fac);
                                    }
                                }
                            }
                        } // if(!mySFrom.IsNull())
                        //
                        if (!mySUntil.IsNull())
                        {
                            TopExp_Explorer ex = new();
                            ex.Init(mySUntil, TopAbs_FACE);
                            for (; ex.More(); ex.Next())
                            {
                                TopoDS_Face fac = TopoDS.Face(ex.Current());
                                if (!UntilInShape)
                                {
                                    NCollection_List<TopoDS_Shape> thelist2 = new();
                                    myMap.Bind(fac, thelist2);
                                }
                                else
                                {
                                    NCollection_List<TopoDS_Shape> thelist3 = new();
                                    locmap.Bind(fac, thelist3);
                                }
                                if (trP.IsDeleted(fac))
                                {
                                }
                                else if (!UntilInShape)
                                {
                                    myMap[fac] = new NCollection_List<TopoDS_Shape>(trP.InternalModified(fac));
                                    if (myMap[fac].IsEmpty())
                                    {
                                        myMap[fac].Append(fac);
                                    }
                                }
                                else
                                {
                                    locmap[fac] = new NCollection_List<TopoDS_Shape>(trP.InternalModified(fac));
                                    if (locmap[fac].IsEmpty())
                                    {
                                        locmap[fac].Append(fac);
                                    }
                                }
                            }
                        } // if(!mySUntil.IsNull())
                        //
                        UpdateDescendants(trP, theGShape, true); // skip faces

                        theGlue.Init(mySbase, theGShape);
                        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> itm in myGluedF)
                        {
                            TopoDS_Face gl = TopoDS.Face(itm.Key);
                            NCollection_List<TopoDS_Shape> ldsc = new();
                            if (trP.IsDeleted(gl))
                            {
                            }
                            else
                            {
                                ldsc = new NCollection_List<TopoDS_Shape>(trP.InternalModified(gl));
                                if (ldsc.IsEmpty())
                                {
                                    ldsc.Append(gl);
                                }
                            }
                            TopoDS_Face glface = TopoDS.Face(itm.Value);
                            for (it.Initialize(ldsc); it.More(); it.Next())
                            {
                                TopoDS_Face fac = TopoDS.Face(it.Value());
                                Collage = BRepFeat.IsInside(fac, glface);
                                if (!Collage)
                                {
                                    theOpe = 2;
                                    ChangeOpe = true;
                                    break;
                                }
                                else
                                {
                                    theGlue.Bind(fac, glface);
                                    theFE.Set(fac, glface);
                                    for (theFE.InitIterator(); theFE.More(); theFE.Next())
                                    {
                                        theGlue.Bind(theFE.EdgeFrom(), theFE.EdgeTo());
                                    }
                                }
                            }
                        }
                    } // else X1
                } // else X0
            } // if(expp.More() && !Comp.IsNull() && !myGShape.IsNull())
            else
            {
                theGlue.Init(mySbase, myGShape);
                foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> itm in myGluedF)
                {
                    TopoDS_Face glface = TopoDS.Face(itm.Key);
                    TopoDS_Face fac = TopoDS.Face(myGluedF[glface]);
                    for (exp.Init(myGShape, TopAbs_FACE); exp.More(); exp.Next())
                    {
                        if (exp.Current().IsSame(glface))
                        {
                            break;
                        }
                    }
                    if (exp.More())
                    {
                        Collage = BRepFeat.IsInside(glface, fac);
                        if (!Collage)
                        {
                            theOpe = 2;
                            ChangeOpe = true;
                            break;
                        }
                        else
                        {
                            theGlue.Bind(glface, fac);
                            theFE.Set(glface, fac);
                            for (theFE.InitIterator(); theFE.More(); theFE.Next())
                            {
                                theGlue.Bind(theFE.EdgeFrom(), theFE.EdgeTo());
                            }
                        }
                    }
                }
            }

            // Add gluing on start and end face if necessary !!!
            if (FromInShape && Collage)
            {
                TopExp_Explorer ex = new(mySFrom, TopAbs_FACE);
                for (; ex.More(); ex.Next())
                {
                    TopoDS_Face fac2 = TopoDS.Face(ex.Current());
                    for (it.Initialize(locmap.Find(fac2)); it.More(); it.Next())
                    {
                        TopoDS_Face fac1 = TopoDS.Face(it.Value());
                        theFE.Set(fac1, fac2);
                        theGlue.Bind(fac1, fac2);
                        for (theFE.InitIterator(); theFE.More(); theFE.Next())
                        {
                            theGlue.Bind(theFE.EdgeFrom(), theFE.EdgeTo());
                        }
                    }
                }
            }

            if (UntilInShape && Collage)
            {
                TopExp_Explorer ex = new(mySUntil, TopAbs_FACE);
                for (; ex.More(); ex.Next())
                {
                    TopoDS_Face fac2 = TopoDS.Face(ex.Current());
                    for (it.Initialize(locmap.Find(fac2)); it.More(); it.Next())
                    {
                        TopoDS_Face fac1 = TopoDS.Face(it.Value());
                        theGlue.Bind(fac1, fac2);
                        theFE.Set(fac1, fac2);
                        for (theFE.InitIterator(); theFE.More(); theFE.Next())
                        {
                            theGlue.Bind(theFE.EdgeFrom(), theFE.EdgeTo());
                        }
                    }
                }
            }

            LocOpe_Operation ope = theGlue.OpeType();
            if (ope == LocOpe_INVALID || (myFuse && ope != LocOpe_FUSE) || (!myFuse && ope != LocOpe_CUT) || (!Collage))
            {
                theOpe = 2;
                ChangeOpe = true;
            }
        }

        //--- if the gluing is always applicable

        if (theOpe == 1)
        {
            theGlue.Perform();
            if (theGlue.IsDone())
            {
                TopoDS_Shape shshs = theGlue.InternalResultingShape();
                if (BRepAlgo.IsValid(shshs))
                {
                    UpdateDescendants(theGlue);
                    myNewEdges = new NCollection_List<TopoDS_Shape>(theGlue.InternalEdges());
                    myTgtEdges = new NCollection_List<TopoDS_Shape>(theGlue.InternalTgtEdges());
                    Done();
                    myShape = theGlue.InternalResultingShape();
                }
                else
                {
                    theOpe = 2;
                    ChangeOpe = true;
                }
            }
            else
            {
                theOpe = 2;
                ChangeOpe = true;
            }
        }

        //--- case without gluing + Tool with proper dimensions

        if (theOpe == 2 && ChangeOpe && myJustGluer)
        {
            myJustGluer = false;
            theOpe = 0;
        }

        //--- case without gluing

        if (theOpe == 2)
        {
            TopoDS_Shape theGShape = myGShape;

            TopoDS_Compound Comp = new();
            BRep_Builder B = new();
            B.MakeCompound(ref Comp);
            if (!mySFrom.IsNull() || !mySUntil.IsNull())
            {
                if (!mySFrom.IsNull() && !FromInShape)
                {
                    TopoDS_Solid S = BRepFeat.Tool(mySFrom, FFrom, Oriffrom);
                    if (!S.IsNull())
                    {
                        B.Add(Comp, S);
                    }
                }
                if (!mySUntil.IsNull() && !UntilInShape)
                {
                    if (!mySFrom.IsNull())
                    {
                        if (!mySFrom.IsSame(mySUntil))
                        {
                            TopoDS_Solid S = BRepFeat.Tool(mySUntil, FUntil, Orifuntil);
                            if (!S.IsNull())
                            {
                                B.Add(Comp, S);
                            }
                        }
                    }
                    else
                    {
                        TopoDS_Solid S = BRepFeat.Tool(mySUntil, FUntil, Orifuntil);
                        if (!S.IsNull())
                        {
                            B.Add(Comp, S);
                        }
                    }
                }
            }

            // update type of selection
            if (myPerfSelection == BRepFeat_SelectionU && !UntilInShape)
            {
                myPerfSelection = BRepFeat_NoSelection;
            }
            else if (myPerfSelection == BRepFeat_SelectionFU && !FromInShape && !UntilInShape)
            {
                myPerfSelection = BRepFeat_NoSelection;
            }
            else if (myPerfSelection == BRepFeat_SelectionShU && !UntilInShape)
            {
                myPerfSelection = BRepFeat_NoSelection;
            }

            TopExp_Explorer expp = new(Comp, TopAbs_SOLID);
            if (expp.More() && !Comp.IsNull() && !myGShape.IsNull())
            {
                BRepAlgoAPI_Cut trP = new(myGShape, Comp);
                // the result is necessarily a compound.
                exp.Init(trP.InternalShape(), TopAbs_SOLID);
                if (!exp.More())
                {
                    myStatusError = BRepFeat_EmptyCutResult;
                    NotDone();
                    return;
                }
                // Only solids are preserved
                TopoDS_Compound aGShape = new();
                B.MakeCompound(ref aGShape);
                for (; exp.More(); exp.Next())
                {
                    B.Add(aGShape, exp.Current());
                }
                theGShape = aGShape;
                if (!BRepAlgo.IsValid(theGShape))
                {
                    myStatusError = BRepFeat_InvShape;
                    NotDone();
                    return;
                }
                if (!mySFrom.IsNull())
                {
                    if (!FromInShape)
                    {
                        TopExp_Explorer ex = new(mySFrom, TopAbs_FACE);
                        for (; ex.More(); ex.Next())
                        {
                            TopoDS_Face fac = TopoDS.Face(ex.Current());
                            NCollection_List<TopoDS_Shape> thelist4 = new();
                            myMap.Bind(fac, thelist4);
                            if (trP.IsDeleted(fac))
                            {
                            }
                            else
                            {
                                myMap[fac] = new NCollection_List<TopoDS_Shape>(trP.InternalModified(fac));
                                if (myMap[fac].IsEmpty())
                                {
                                    myMap[fac].Append(fac);
                                }
                            }
                        }
                    }
                }
                if (!mySUntil.IsNull())
                {
                    if (!UntilInShape)
                    {
                        TopExp_Explorer ex = new(mySUntil, TopAbs_FACE);
                        for (; ex.More(); ex.Next())
                        {
                            TopoDS_Face fac = TopoDS.Face(ex.Current());
                            NCollection_List<TopoDS_Shape> thelist5 = new();
                            myMap.Bind(fac, thelist5);
                            if (trP.IsDeleted(fac))
                            {
                            }
                            else
                            {
                                myMap[fac] = new NCollection_List<TopoDS_Shape>(trP.InternalModified(fac));
                                if (myMap.IsEmpty()) // OCCT: tests the map, not the face's list
                                {
                                    myMap[fac].Append(fac);
                                }
                            }
                        }
                    }
                }
                UpdateDescendants(trP, theGShape, true);
            } // if(expp.More() && !Comp.IsNull() && !myGShape.IsNull())  {
            //

            //--- generation of "just feature" for assembly = Parts of tool
            bool bFlag = myPerfSelection != BRepFeat_NoSelection;
            BRepFeat_Builder theBuilder = new();
            theBuilder.Init(mySbase, theGShape);
            theBuilder.SetOperation(myFuse ? 1 : 0, bFlag);
            theBuilder.Perform();
            //
            NCollection_List<TopoDS_Shape> lshape = new();
            theBuilder.PartsOfTool(lshape);
            //
            double pbmin = RealLast(), pbmax = RealFirst();
            double prmin = RealLast() - 2 * Precision.Confusion();
            double prmax = RealFirst() + 2 * Precision.Confusion();
            bool flag1 = false;
            Geom_Curve? C = null;

            //--- Selection of pieces of tool to be preserved
            if (!lshape.IsEmpty() && myPerfSelection != BRepFeat_NoSelection)
            {
                //      Find ParametricMinMax depending on the constraints of Shape From and Until
                //   -> prmin, prmax, pbmin and pbmax
                C = BarycCurve();
                if (C is null)
                {
                    myStatusError = BRepFeat_EmptyBaryCurve;
                    NotDone();
                    return;
                }

                if (myPerfSelection == BRepFeat_SelectionSh)
                {
                    BRepFeat.ParametricMinMax(mySbase, C, out prmin, out prmax, out pbmin, out pbmax, out flag1);
                }
                else if (myPerfSelection == BRepFeat_SelectionFU)
                {
                    BRepFeat.ParametricMinMax(mySFrom, C, out double prmin1, out double prmax1, out double prbmin1, out double prbmax1, out flag1);
                    BRepFeat.ParametricMinMax(mySUntil, C, out double prmin2, out double prmax2, out double prbmin2, out double prbmax2, out flag1);

                    // case of revolutions
                    if (C.IsPeriodic())
                    {
                        double period = C.Period();
                        prmax = prmax2;
                        if (flag1)
                        {
                            prmin = ElCLib.InPeriod(prmin1, prmax - period, prmax);
                        }
                        else
                        {
                            prmin = Min(prmin1, prmin2);
                        }
                        pbmax = prbmax2;
                        pbmin = ElCLib.InPeriod(prbmin1, pbmax - period, pbmax);
                    }
                    else
                    {
                        prmin = Min(prmin1, prmin2);
                        prmax = Max(prmax1, prmax2);
                        pbmin = Min(prbmin1, prbmin2);
                        pbmax = Max(prbmax1, prbmax2);
                    }
                }
                else if (myPerfSelection == BRepFeat_SelectionShU)
                {
                    if (!myJustFeat && sens == 0)
                    {
                        sens = 1;
                    }
                    if (sens == 0)
                    {
                        myStatusError = BRepFeat_IncDirection;
                        NotDone();
                        return;
                    }

                    BRepFeat.ParametricMinMax(mySUntil, C, out double prmin1, out double prmax1, out double prbmin1, out double prbmax1, out flag1);

                    BRepFeat.ParametricMinMax(mySbase, C, out double prmin2, out double prmax2, out double prbmin2, out double prbmax2, out flag1);
                    if (sens == 1)
                    {
                        prmin = prmin2;
                        prmax = prmax1;
                        pbmin = prbmin2;
                        pbmax = prbmax1;
                    }
                    else if (sens == -1)
                    {
                        prmin = prmin1;
                        prmax = prmax2;
                        pbmin = prbmin1;
                        pbmax = prbmax2;
                    }
                }
                else if (myPerfSelection == BRepFeat_SelectionU)
                {
                    if (sens == 0)
                    {
                        myStatusError = BRepFeat_IncDirection;
                        NotDone();
                        return;
                    }

                    // Find parts of the tool containing descendants of Shape Until
                    BRepFeat.ParametricMinMax(mySUntil, C, out double prmin1, out double prmax1, out double prbmin1, out double prbmax1, out flag1);
                    if (sens == 1)
                    {
                        prmin = RealFirst();
                        prmax = prmax1;
                        pbmin = RealFirst();
                        pbmax = prbmax1;
                    }
                    else if (sens == -1)
                    {
                        prmin = prmin1;
                        prmax = RealLast();
                        pbmin = prbmin1;
                        pbmax = RealLast();
                    }
                }

                // Finer choice of ParametricMinMax in case when the tool
                // intersects Shapes From and Until
                //       case of several intersections (keep PartsOfTool according to the selection)
                //       position of the face of intersection in PartsOfTool (before or after)
                double delta = Precision.Confusion();

                if (myPerfSelection != BRepFeat_NoSelection)
                {
                    // modif of the test for cts21181 : (prbmax2 and prnmin2) -> (prbmin1 and prbmax1)
                    // correction take into account flag2 for pro15323 and flag3 for pro16060
                    if (!mySUntil.IsNull())
                    {
                        NCollection_Map<TopoDS_Shape> mapFuntil = new(TopTools_ShapeMapHasher.Instance);
                        Descendants(mySUntil, theBuilder, mapFuntil);
                        if (!mapFuntil.IsEmpty())
                        {
                            for (it.Initialize(lshape); it.More(); it.Next())
                            {
                                TopExp_Explorer expf = new();
                                for (expf.Init(it.Value(), TopAbs_FACE); expf.More(); expf.Next())
                                {
                                    if (mapFuntil.Contains(expf.Current()))
                                    {
                                        BRepFeat.ParametricMinMax(expf.Current(), C, out double prmin1, out double prmax1, out double prbmin1, out double prbmax1, out bool flag3);
                                        BRepFeat.ParametricMinMax(it.Value(), C, out double prmin2, out double prmax2, out double prbmin2, out double prbmax2, out bool flag2);
                                        if (sens == 1)
                                        {
                                            bool testOK = !flag2;
                                            if (flag2)
                                            {
                                                testOK = !flag1;
                                                if (flag1 && prmax2 > prmin + delta)
                                                {
                                                    testOK = !flag3;
                                                    if (flag3 && prmax1 == prmax2)
                                                    {
                                                        testOK = true;
                                                    }
                                                }
                                            }
                                            if (prbmin1 < pbmax && testOK)
                                            {
                                                if (flag2)
                                                {
                                                    flag1 = flag2;
                                                    prmax = prmax2;
                                                }
                                                pbmax = prbmin1;
                                            }
                                        }
                                        else if (sens == -1)
                                        {
                                            bool testOK = !flag2;
                                            if (flag2)
                                            {
                                                testOK = !flag1;
                                                if (flag1 && prmin2 < prmax - delta)
                                                {
                                                    testOK = !flag3;
                                                    if (flag3 && prmin1 == prmin2)
                                                    {
                                                        testOK = true;
                                                    }
                                                }
                                            }
                                            if (prbmax1 > pbmin && testOK)
                                            {
                                                if (flag2)
                                                {
                                                    flag1 = flag2;
                                                    prmin = prmin2;
                                                }
                                                pbmin = prbmax1;
                                            }
                                        }
                                        break;
                                    }
                                }
                            }
                            it.Initialize(lshape);
                        }
                    }
                    if (!mySFrom.IsNull())
                    {
                        NCollection_Map<TopoDS_Shape> mapFfrom = new(TopTools_ShapeMapHasher.Instance);
                        Descendants(mySFrom, theBuilder, mapFfrom);
                        if (!mapFfrom.IsEmpty())
                        {
                            for (it.Initialize(lshape); it.More(); it.Next())
                            {
                                TopExp_Explorer expf = new();
                                for (expf.Init(it.Value(), TopAbs_FACE); expf.More(); expf.Next())
                                {
                                    if (mapFfrom.Contains(expf.Current()))
                                    {
                                        BRepFeat.ParametricMinMax(expf.Current(), C, out double prmin1, out double prmax1, out double prbmin1, out double prbmax1, out bool flag3);
                                        BRepFeat.ParametricMinMax(it.Value(), C, out double prmin2, out double prmax2, out double prbmin2, out double prbmax2, out bool flag2);
                                        if (sens == 1)
                                        {
                                            bool testOK = !flag2;
                                            if (flag2)
                                            {
                                                testOK = !flag1;
                                                if (flag1 && prmin2 < prmax - delta)
                                                {
                                                    testOK = !flag3;
                                                    if (flag3 && prmin1 == prmin2)
                                                    {
                                                        testOK = true;
                                                    }
                                                }
                                            }
                                            if (prbmax1 > pbmin && testOK)
                                            {
                                                if (flag2)
                                                {
                                                    flag1 = flag2;
                                                    prmin = prmin2;
                                                }
                                                pbmin = prbmax1;
                                            }
                                        }
                                        else if (sens == -1)
                                        {
                                            bool testOK = !flag2;
                                            if (flag2)
                                            {
                                                testOK = !flag1;
                                                if (flag1 && prmax2 > prmin + delta)
                                                {
                                                    testOK = !flag3;
                                                    if (flag3 && prmax1 == prmax2)
                                                    {
                                                        testOK = true;
                                                    }
                                                }
                                            }
                                            if (prbmin1 < pbmax && testOK)
                                            {
                                                if (flag2)
                                                {
                                                    flag1 = flag2;
                                                    prmax = prmax2;
                                                }
                                                pbmax = prbmin1;
                                            }
                                        }
                                        break;
                                    }
                                }
                            }
                            it.Initialize(lshape);
                        }
                    }
                }

                // Parse PartsOfTool to preserve or not depending on ParametricMinMax
                if (!myJustFeat)
                {
                    bool KeepParts = false;
                    double prmin1, prmax1, prbmin1, prbmax1;
                    double min, max, pmin, pmax;
                    bool flag2;
                    for (it.Initialize(lshape); it.More(); it.Next())
                    {
                        if (C.IsPeriodic())
                        {
                            double period = C.Period();
                            BRepFeat.ParametricMinMax(it.Value(), C, out double pr, out prmax1, out double prb, out prbmax1, out flag2, true);
                            if (flag2)
                            {
                                prmin1 = ElCLib.InPeriod(pr, prmax1 - period, prmax1);
                            }
                            else
                            {
                                prmin1 = pr;
                            }
                            prbmin1 = ElCLib.InPeriod(prb, prbmax1 - period, prbmax1);
                        }
                        else
                        {
                            BRepFeat.ParametricMinMax(it.Value(), C, out prmin1, out prmax1, out prbmin1, out prbmax1, out flag2);
                        }
                        if (!flag2 || !flag1)
                        {
                            pmin = pbmin;
                            pmax = pbmax;
                            min = prbmin1;
                            max = prbmax1;
                        }
                        else
                        {
                            pmin = prmin;
                            pmax = prmax;
                            min = prmin1;
                            max = prmax1;
                        }
                        if ((min <= pmax - delta) && (max >= pmin + delta))
                        {
                            KeepParts = true;
                            TopoDS_Shape S = it.Value();
                            theBuilder.KeepPart(S);
                        }
                    }

                    // Case when no part of the tool is preserved
                    if (!KeepParts)
                    {
                        myStatusError = BRepFeat_NoParts;
                        NotDone();
                        return;
                    }
                }
                else
                {
                    // case JustFeature -> all PartsOfTool are preserved
                    double prmin1, prmax1, prbmin1, prbmax1;
                    double min, max, pmin, pmax;
                    bool flag2;
                    TopoDS_Compound Compo = new();
                    B.MakeCompound(ref Compo);
                    for (it.Initialize(lshape); it.More(); it.Next())
                    {
                        BRepFeat.ParametricMinMax(it.Value(), C, out prmin1, out prmax1, out prbmin1, out prbmax1, out flag2);
                        if (!flag2 || !flag1)
                        {
                            pmin = pbmin;
                            pmax = pbmax;
                            min = prbmin1;
                            max = prbmax1;
                        }
                        else
                        {
                            pmin = prmin;
                            pmax = prmax;
                            min = prmin1;
                            max = prmax1;
                        }
                        if ((min < pmax - delta) && (max > pmin + delta))
                        {
                            if (!it.Value().IsNull())
                            {
                                B.Add(Compo, it.Value());
                            }
                        }
                    }
                    myShape = Compo;
                }
            }

            //--- Generation of result myShape

            if (!myJustFeat)
            {
                // removal of edges of section that have no common vertices
                // with PartsOfTool preserved
                if (bFlag)
                {
                    theBuilder.PerformResult();
                    myShape = theBuilder.InternalShape();
                }
                else
                {
                    myShape = theBuilder.InternalShape();
                }
                Done();
            }
            else
            {
                // all is already done
                Done();
            }
        }

        myStatusError = BRepFeat_OK;
    }

    /// <summary>True if the face <paramref name="F"/> of the basis shape has no descendant in the result.</summary>
    public override bool IsDeleted(in TopoDS_Shape F)
    {
        FinishWorkingShapes();
        if (myMap.IsBound(F))
        {
            return myMap.Find(F).IsEmpty();
        }
        return false;
    }

    /// <summary>The faces generated from the face <paramref name="F"/> (OCCT's virtual; unfrozen).</summary>
    internal override NCollection_List<TopoDS_Shape> InternalModified(in TopoDS_Shape F)
    {
        FinishWorkingShapes();
        myGenerated.Clear();
        if (!IsDone())
        {
            return myGenerated;
        }

        if (mySbase.IsEqual(F))
        {
            myGenerated.Append(myShape);
            return myGenerated;
        }

        if (myMap.IsBound(F))
        {
            NCollection_List<TopoDS_Shape>.Iterator ite = new(myMap.Find(F));
            for (; ite.More(); ite.Next())
            {
                TopoDS_Shape sh = ite.Value();
                if (!sh.IsSame(F) && sh.ShapeType() == F.ShapeType())
                {
                    myGenerated.Append(sh);
                }
            }
        }
        return myGenerated; // empty list
    }

    /// <summary>The faces created from the shape <paramref name="S"/> (OCCT's virtual; unfrozen).</summary>
    internal override NCollection_List<TopoDS_Shape> InternalGenerated(in TopoDS_Shape S)
    {
        FinishWorkingShapes();
        myGenerated.Clear();
        if (!IsDone())
        {
            return myGenerated;
        }
        if (myMap.IsBound(S) && S.ShapeType() != TopAbs_FACE)
        { // check if filter on face or not
            NCollection_List<TopoDS_Shape>.Iterator ite = new(myMap.Find(S));
            for (; ite.More(); ite.Next())
            {
                TopoDS_Shape sh = ite.Value();
                if (!sh.IsSame(S))
                {
                    myGenerated.Append(sh);
                }
            }
            return myGenerated;
        }
        return myGenerated;
    }

    // The history's lists hold sub-shapes of the result: publishing them publishes the result (decision 414).
    private protected override void BeforePublishingLists()
    {
        FinishWorkingShapes();
        myShape.TShape()?.FreezeGraph();
    }

    protected void UpdateDescendants(LocOpe_Gluer G)
    {
        NCollection_List<TopoDS_Shape>.Iterator it = new(), it2 = new();

        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itdm in myMap)
        {
            TopoDS_Shape orig = itdm.Key;
            // OCCT iterates this address-keyed map into the list: insertion order here (decision 9503)
            NCollection_IndexedMap<TopoDS_Shape> newdsc = new(TopTools_ShapeMapHasher.Instance);
            for (it.Initialize(itdm.Value); it.More(); it.Next())
            {
                TopoDS_Face fdsc = TopoDS.Face(it.Value());
                for (it2.Initialize(G.InternalDescendantFaces(fdsc)); it2.More(); it2.Next())
                {
                    newdsc.Add(it2.Value());
                }
            }
            myMap.ChangeFind(orig).Clear();
            foreach (TopoDS_Shape itm in newdsc)
            {
                myMap.ChangeFind(orig).Append(itm);
            }
        }
    }

    /// <summary>The shapes created at the bottom of the created form; it may be empty (frozen).</summary>
    public IBentoList<TopoDS_Shape> FirstShape()
    {
        BeforePublishingLists();
        return PublishedList(InternalFirstShape());
    }

    /// <summary><see cref="FirstShape"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalFirstShape()
    {
        FinishWorkingShapes();
        if (!myFShape.IsNull())
        {
            return myMap.Find(myFShape);
        }
        return myGenerated; // empty list
    }

    /// <summary>The shapes created at the top of the created form; it may be empty (frozen).</summary>
    public IBentoList<TopoDS_Shape> LastShape()
    {
        BeforePublishingLists();
        return PublishedList(InternalLastShape());
    }

    /// <summary><see cref="LastShape"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalLastShape()
    {
        FinishWorkingShapes();
        if (!myLShape.IsNull())
        {
            return myMap.Find(myLShape);
        }
        return myGenerated; // empty list
    }

    /// <summary>
    /// The limiting and glueing edges generated by the feature, which did not exist in the basis shape: the
    /// information needed to add fillets afterwards. It may be empty (frozen).
    /// </summary>
    public IBentoList<TopoDS_Shape> NewEdges()
    {
        BeforePublishingLists();
        return PublishedList(InternalNewEdges());
    }

    /// <summary><see cref="NewEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalNewEdges()
    {
        FinishWorkingShapes();
        return myNewEdges;
    }

    /// <summary>
    /// The tangent edges among <see cref="NewEdges"/>: no fillet is possible on them. It may be empty (frozen).
    /// </summary>
    public IBentoList<TopoDS_Shape> TgtEdges()
    {
        BeforePublishingLists();
        return PublishedList(InternalTgtEdges());
    }

    /// <summary><see cref="TgtEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalTgtEdges()
    {
        FinishWorkingShapes();
        return myTgtEdges;
    }

    /// <summary>Limitation of the from (flag 0) or until (flag 1) shape in the case of infinite faces.</summary>
    protected bool TransformShapeFU(int flag)
    {
        bool Trf = false;

        TopoDS_Shape shapefu;
        if (flag == 0)
        {
            shapefu = mySFrom;
        }
        else if (flag == 1)
        {
            shapefu = mySUntil;
        }
        else
        {
            return Trf;
        }

        TopExp_Explorer exp = new(shapefu, TopAbs_FACE);
        if (!exp.More())
        { // no faces... It is necessary to return an error
            return Trf;
        }

        exp.Next();
        if (!exp.More())
        { // the only face. Is it infinite?
            exp.ReInit();
            TopoDS_Face fac = TopoDS.Face(exp.Current());

            Geom_Surface S = BRep_Tool.Surface(fac)!;
            if (S is Geom_RectangularTrimmedSurface aTrimmed)
            {
                S = aTrimmed.BasisSurface();
            }

            if (S.GetType() == typeof(Geom_Plane) || S.GetType() == typeof(Geom_CylindricalSurface)
                || S.GetType() == typeof(Geom_ConicalSurface))
            {
                TopExp_Explorer exp1 = new(fac, TopAbs_WIRE);
                if (!exp1.More())
                {
                    Trf = true;
                }
                else
                {
                    Trf = BRep_Tool.NaturalRestriction(fac);
                }
            }
            if (Trf)
            {
                BRepFeat.FaceUntil(mySbase, ref fac);
            }

            if (flag == 0)
            {
                NCollection_List<TopoDS_Shape> thelist6 = new();
                myMap.Bind(mySFrom, thelist6);
                myMap[mySFrom].Append(fac);
                mySFrom = fac;
            }
            else if (flag == 1)
            {
                NCollection_List<TopoDS_Shape> thelist7 = new();
                myMap.Bind(mySUntil, thelist7);
                myMap[mySUntil].Append(fac);
                mySUntil = fac;
            }
        }
        else
        {
            for (exp.ReInit(); exp.More(); exp.Next())
            {
                TopoDS_Shape fac = exp.Current();
                NCollection_List<TopoDS_Shape> thelist8 = new();
                myMap.Bind(fac, thelist8);
                myMap[fac].Append(fac);
            }
        }
        return Trf;
    }

    private static void Descendants(in TopoDS_Shape S, BRepFeat_Builder theFB, NCollection_Map<TopoDS_Shape> mapF)
    {
        mapF.Clear();
        TopExp_Explorer exp = new();
        for (exp.Init(S, TopAbs_FACE); exp.More(); exp.Next())
        {
            TopoDS_Face fdsc = TopoDS.Face(exp.Current());
            foreach (TopoDS_Shape aS in theFB.Modified(fdsc))
            {
                mapF.Add(aS);
            }
        }
    }

    protected void UpdateDescendants(BRepAlgoAPI_BooleanOperation aBOP, in TopoDS_Shape S, bool SkipFace = false)
    {
        NCollection_List<TopoDS_Shape>.Iterator it = new(), it2 = new();
        TopExp_Explorer exp = new();

        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itdm in myMap)
        {
            TopoDS_Shape orig = itdm.Key;
            if (SkipFace && orig.ShapeType() == TopAbs_FACE)
            {
                continue;
            }
            // OCCT iterates this address-keyed map into the list: insertion order here (decision 9503)
            NCollection_IndexedMap<TopoDS_Shape> newdsc = new(TopTools_ShapeMapHasher.Instance);

            if (itdm.Value.IsEmpty())
            {
                myMap.ChangeFind(orig).Append(orig);
            }

            for (it.Initialize(itdm.Value); it.More(); it.Next())
            {
                TopoDS_Shape sh = it.Value();
                if (sh.ShapeType() != TopAbs_FACE)
                {
                    continue;
                }
                TopoDS_Face fdsc = TopoDS.Face(it.Value());
                for (exp.Init(S, TopAbs_FACE); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(fdsc))
                    { // preserved
                        newdsc.Add(fdsc);
                        break;
                    }
                }
                if (!exp.More())
                {
                    NCollection_List<TopoDS_Shape> aLM = aBOP.InternalModified(fdsc);
                    it2.Initialize(aLM);
                    for (; it2.More(); it2.Next())
                    {
                        TopoDS_Shape aS = it2.Value();
                        newdsc.Add(aS);
                    }
                }
            }
            myMap.ChangeFind(orig).Clear();
            foreach (TopoDS_Shape itm in newdsc)
            {
                // check the appartenance to the shape...
                for (exp.Init(S, TopAbs_FACE); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(itm))
                    {
                        myMap.ChangeFind(orig).Append(itm);
                        break;
                    }
                }
            }
        }
    }
}
