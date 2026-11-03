// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Form.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Form.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Form.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using static Occt.BRepFeat_PerfSelection;
using static Occt.BRepFeat_StatusError;
using static Occt.LocOpe_Operation;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
    protected NCollection_DataMap<Shape, NCollection_List<Shape>> myMap = new(ShapeHasher.Instance);
    protected Shape myFShape = new();
    protected Shape myLShape = new();
    protected NCollection_List<Shape> myNewEdges = new();
    protected NCollection_List<Shape> myTgtEdges = new();
    protected BRepFeat_PerfSelection myPerfSelection;
    protected bool myJustGluer;
    protected bool myJustFeat;
    protected Shape mySbase = new();
    protected Face mySkface = new();
    protected Shape myGShape = new();
    protected Shape mySFrom = new();
    protected Shape mySUntil = new();
    protected NCollection_DataMap<Shape, Shape> myGluedF = new(ShapeHasher.Instance);

    private bool mySbOK;
    private bool mySkOK;
    private bool myGSOK;
    private bool mySFOK;
    private bool mySUOK;
    private bool myGFOK;
    private bool myPSOK;
    private BRepFeat_StatusError myStatusError;

    // OCCT.NET: the working copies of the frozen shapes the feature is given (decision 1052)
    private protected LocOpe_WorkingShapes? myWork;
    private LocOpe_WorkingShapes? myFinishedWork;

    // OCCT.NET: OCCT's IsSame(sh, F) in the history, where sh may be the changed working copy of the caller's F: OCCT
    // changes F in place, keeping its identity, so the copy counts as F (decision 1052)
    private bool IsSameInHistory(in Shape sh, in Shape F) =>
        sh.IsSame(F) || (myFinishedWork is not null && myFinishedWork.Original(sh).IsSame(F));

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
    public abstract void Curves(NCollection_Sequence<Curve3D> S);

    /// <summary>The curve through the barycentre of the feature, along its direction.</summary>
    public abstract Curve3D? BarycCurve();

    /// <summary>The status of the last construction.</summary>
    public BRepFeat_StatusError CurrentStatusError() => myStatusError;

    /// <summary>
    /// OCCT.NET: starts the feature's working shapes (decision 1052). OCCT's feature writes into the shapes it is
    /// given: the Booleans it runs are destructive (they raise tolerances and add pcurves in place) and the sweeps
    /// write into the profile. Every shape entering the feature goes through <see cref="Working(in Shape)"/>, which gives the
    /// exact copy for write of a frozen one, all copies sharing one map (a face of the basis shape given as the sketch
    /// face or the until shape has its one copy); the result, the history and the lists are given back on the caller's
    /// shapes where the feature left them unchanged, at the first publication (<see cref="FinishWorkingShapes"/>).
    /// </summary>
    private protected void StartWorkingShapes()
    {
        myWork = new LocOpe_WorkingShapes();
        myFinishedWork = null;
    }

    /// <summary>The working shape standing for <paramref name="S"/> (decision 1052).</summary>
    private protected Shape Working(in Shape S)
    {
        myWork ??= new LocOpe_WorkingShapes();
        return myWork.Working(S);
    }

    /// <summary>The working face standing for <paramref name="F"/> (decision 1052).</summary>
    private protected Face Working(in Face F) => F.IsNull ? F : ShapeCasts.Face(Working((Shape)F));

    /// <summary>
    /// OCCT.NET: ends the working shapes (decision 1052): every copy the feature left unchanged is replaced by the
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
        myFinishedWork = aWork;
        var aShapes = new List<Shape> { myShape };
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> anItem in myMap)
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
        var aMap = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> anItem in myMap)
        {
            Shape aKey = aWork.Original(anItem.Key);
            if (!aMap.IsBound(aKey))
            {
                aMap.Bind(aKey, aWork.Final(anItem.Value));
            }
        }
        myMap = aMap;
        myNewEdges = aWork.Final(myNewEdges);
        myTgtEdges = aWork.Final(myTgtEdges);
        mySbase = aWork.Original(mySbase);
        mySkface = ShapeCasts.Face(aWork.Original(mySkface));
        mySFrom = aWork.Original(mySFrom);
        mySUntil = aWork.Original(mySUntil);
        myFShape = aWork.Original(myFShape);
        myLShape = aWork.Original(myLShape);
    }

    /// <summary>The result, unfrozen, on the caller's shapes where they are unchanged (OCCT's virtual).</summary>
    internal override Shape InternalShape()
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
        Explorer exp = new(), exp2 = new();
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

        if (!mySFrom.IsNull)
        {
            FromInShape = true;
            for (exp2.Init(mySFrom, ShapeType.Face); exp2.More(); exp2.Next())
            {
                Shape ffrom = exp2.Current();
                for (exp.Init(mySbase, ShapeType.Face); exp.More(); exp.Next())
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

        if (!mySUntil.IsNull)
        {
            UntilInShape = true;
            for (exp2.Init(mySUntil, ShapeType.Face); exp2.More(); exp2.Next())
            {
                Shape funtil = exp2.Current();
                for (exp.Init(mySbase, ShapeType.Face); exp.More(); exp.Next())
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

        NCollection_List<Shape>.Iterator it = new();
        int sens = 0;

        NCollection_Sequence<Curve3D> scur = new();
        Curves(scur);

        double mf, Mf, mu, Mu;

        Orientation Orifuntil = Internal;
        Orientation Oriffrom = Internal;
        Face FFrom = new(), FUntil = new();

        LocOpe_CSIntersector ASI1 = new();
        LocOpe_CSIntersector ASI2 = new();

        //--- 1) by intersection

        // Intersection Tool Shape From
        if (!mySFrom.IsNull)
        {
            ASI1.Init(mySFrom);
            ASI1.Perform(scur!);
        }

        // Intersection Tool Shape Until
        if (!mySUntil.IsNull)
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
                    if (!scur[jj].IsPeriodic)
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
                        if (Oriffrom == Internal)
                        {
                            Orientation Oript = ASI1.Point(jj, kf).Orientation();
                            if (Oript == Forward || Oript == Reversed)
                            {
                                if (sens == -1)
                                {
                                    Oript = global::Kerf.Engine.Topology.TopAbs.Reverse(Oript);
                                }
                                Oriffrom = global::Kerf.Engine.Topology.TopAbs.Reverse(Oript);
                                FFrom = ASI1.Point(jj, kf).Face();
                            }
                        }
                        if (Orifuntil == Internal)
                        {
                            Orientation Oript = ASI2.Point(jj, ku).Orientation();
                            if (Oript == Forward || Oript == Reversed)
                            {
                                if (sens == -1)
                                {
                                    Oript = global::Kerf.Engine.Topology.TopAbs.Reverse(Oript);
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
                    if (Orifuntil == Internal && sens != 0)
                    {
                        Orientation Oript = ASI2.Point(jj, ku).Orientation();
                        if (Oript == Forward || Oript == Reversed)
                        {
                            if (sens == -1)
                            {
                                Oript = global::Kerf.Engine.Topology.TopAbs.Reverse(Oript);
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
            Compound Comp = new();
            BrepBuilder B = new();
            B.MakeCompound(ref Comp);
            if (!mySFrom.IsNull)
            {
                Solid S = BRepFeat.Tool(mySFrom, FFrom, Oriffrom);
                if (!S.IsNull)
                {
                    B.Add(Comp, S);
                }
            }
            if (!mySUntil.IsNull)
            {
                Solid S = BRepFeat.Tool(mySUntil, FUntil, Orifuntil);
                if (!S.IsNull)
                {
                    B.Add(Comp, S);
                }
            }

            LocOpe_FindEdges theFE = new();
            NCollection_DataMap<Shape, NCollection_List<Shape>> locmap = new(ShapeHasher.Instance);
            Explorer expp = new(Comp, ShapeType.Solid);
            if (expp.More() && !Comp.IsNull && !myGShape.IsNull)
            {
                BRepAlgoAPI_Cut trP = new(myGShape, Comp);
                exp.Init(trP.InternalShape(), ShapeType.Solid);
                if (exp.Current().IsNull)
                {
                    theOpe = 2;
                    ChangeOpe = true;
                    Collage = false;
                }
                else
                { // else X0
                    // Only solids are preserved
                    Compound theGShape = new();
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
                        if (!mySFrom.IsNull)
                        {
                            Explorer ex = new();
                            ex.Init(mySFrom, ShapeType.Face);
                            for (; ex.More(); ex.Next())
                            {
                                Face fac = ShapeCasts.Face(ex.Current());
                                if (!FromInShape)
                                {
                                    NCollection_List<Shape> thelist = new();
                                    myMap.Bind(fac, thelist);
                                }
                                else
                                {
                                    NCollection_List<Shape> thelist1 = new();
                                    locmap.Bind(fac, thelist1);
                                }
                                if (trP.IsDeleted(fac))
                                {
                                }
                                else if (!FromInShape)
                                {
                                    myMap[fac] = new NCollection_List<Shape>(trP.InternalModified(fac));
                                    if (myMap[fac].IsEmpty())
                                    {
                                        myMap[fac].Append(fac);
                                    }
                                }
                                else
                                {
                                    locmap[fac] = new NCollection_List<Shape>(trP.InternalModified(fac));
                                    if (locmap[fac].IsEmpty())
                                    {
                                        locmap[fac].Append(fac);
                                    }
                                }
                            }
                        } // if(!mySFrom.IsNull())
                        //
                        if (!mySUntil.IsNull)
                        {
                            Explorer ex = new();
                            ex.Init(mySUntil, ShapeType.Face);
                            for (; ex.More(); ex.Next())
                            {
                                Face fac = ShapeCasts.Face(ex.Current());
                                if (!UntilInShape)
                                {
                                    NCollection_List<Shape> thelist2 = new();
                                    myMap.Bind(fac, thelist2);
                                }
                                else
                                {
                                    NCollection_List<Shape> thelist3 = new();
                                    locmap.Bind(fac, thelist3);
                                }
                                if (trP.IsDeleted(fac))
                                {
                                }
                                else if (!UntilInShape)
                                {
                                    myMap[fac] = new NCollection_List<Shape>(trP.InternalModified(fac));
                                    if (myMap[fac].IsEmpty())
                                    {
                                        myMap[fac].Append(fac);
                                    }
                                }
                                else
                                {
                                    locmap[fac] = new NCollection_List<Shape>(trP.InternalModified(fac));
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
                        foreach (KeyValuePair<Shape, Shape> itm in myGluedF)
                        {
                            Face gl = ShapeCasts.Face(itm.Key);
                            NCollection_List<Shape> ldsc = new();
                            if (trP.IsDeleted(gl))
                            {
                            }
                            else
                            {
                                ldsc = new NCollection_List<Shape>(trP.InternalModified(gl));
                                if (ldsc.IsEmpty())
                                {
                                    ldsc.Append(gl);
                                }
                            }
                            Face glface = ShapeCasts.Face(itm.Value);
                            for (it.Initialize(ldsc); it.More(); it.Next())
                            {
                                Face fac = ShapeCasts.Face(it.Value());
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
                foreach (KeyValuePair<Shape, Shape> itm in myGluedF)
                {
                    Face glface = ShapeCasts.Face(itm.Key);
                    Face fac = ShapeCasts.Face(myGluedF[glface]);
                    for (exp.Init(myGShape, ShapeType.Face); exp.More(); exp.Next())
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
                Explorer ex = new(mySFrom, ShapeType.Face);
                for (; ex.More(); ex.Next())
                {
                    Face fac2 = ShapeCasts.Face(ex.Current());
                    for (it.Initialize(locmap.Find(fac2)); it.More(); it.Next())
                    {
                        Face fac1 = ShapeCasts.Face(it.Value());
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
                Explorer ex = new(mySUntil, ShapeType.Face);
                for (; ex.More(); ex.Next())
                {
                    Face fac2 = ShapeCasts.Face(ex.Current());
                    for (it.Initialize(locmap.Find(fac2)); it.More(); it.Next())
                    {
                        Face fac1 = ShapeCasts.Face(it.Value());
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
                Shape shshs = theGlue.InternalResultingShape();
                if (BRepAlgo.IsValid(shshs))
                {
                    UpdateDescendants(theGlue);
                    myNewEdges = new NCollection_List<Shape>(theGlue.InternalEdges());
                    myTgtEdges = new NCollection_List<Shape>(theGlue.InternalTgtEdges());
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
            Shape theGShape = myGShape;

            Compound Comp = new();
            BrepBuilder B = new();
            B.MakeCompound(ref Comp);
            if (!mySFrom.IsNull || !mySUntil.IsNull)
            {
                if (!mySFrom.IsNull && !FromInShape)
                {
                    Solid S = BRepFeat.Tool(mySFrom, FFrom, Oriffrom);
                    if (!S.IsNull)
                    {
                        B.Add(Comp, S);
                    }
                }
                if (!mySUntil.IsNull && !UntilInShape)
                {
                    if (!mySFrom.IsNull)
                    {
                        if (!mySFrom.IsSame(mySUntil))
                        {
                            Solid S = BRepFeat.Tool(mySUntil, FUntil, Orifuntil);
                            if (!S.IsNull)
                            {
                                B.Add(Comp, S);
                            }
                        }
                    }
                    else
                    {
                        Solid S = BRepFeat.Tool(mySUntil, FUntil, Orifuntil);
                        if (!S.IsNull)
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

            Explorer expp = new(Comp, ShapeType.Solid);
            if (expp.More() && !Comp.IsNull && !myGShape.IsNull)
            {
                BRepAlgoAPI_Cut trP = new(myGShape, Comp);
                // the result is necessarily a compound.
                exp.Init(trP.InternalShape(), ShapeType.Solid);
                if (!exp.More())
                {
                    myStatusError = BRepFeat_EmptyCutResult;
                    NotDone();
                    return;
                }
                // Only solids are preserved
                Compound aGShape = new();
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
                if (!mySFrom.IsNull)
                {
                    if (!FromInShape)
                    {
                        Explorer ex = new(mySFrom, ShapeType.Face);
                        for (; ex.More(); ex.Next())
                        {
                            Face fac = ShapeCasts.Face(ex.Current());
                            NCollection_List<Shape> thelist4 = new();
                            myMap.Bind(fac, thelist4);
                            if (trP.IsDeleted(fac))
                            {
                            }
                            else
                            {
                                myMap[fac] = new NCollection_List<Shape>(trP.InternalModified(fac));
                                if (myMap[fac].IsEmpty())
                                {
                                    myMap[fac].Append(fac);
                                }
                            }
                        }
                    }
                }
                if (!mySUntil.IsNull)
                {
                    if (!UntilInShape)
                    {
                        Explorer ex = new(mySUntil, ShapeType.Face);
                        for (; ex.More(); ex.Next())
                        {
                            Face fac = ShapeCasts.Face(ex.Current());
                            NCollection_List<Shape> thelist5 = new();
                            myMap.Bind(fac, thelist5);
                            if (trP.IsDeleted(fac))
                            {
                            }
                            else
                            {
                                myMap[fac] = new NCollection_List<Shape>(trP.InternalModified(fac));
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
            NCollection_List<Shape> lshape = new();
            theBuilder.PartsOfTool(lshape);
            //
            double pbmin = RealLast(), pbmax = RealFirst();
            double prmin = RealLast() - 2 * global::Kerf.Engine.Foundation.Precision.Confusion();
            double prmax = RealFirst() + 2 * global::Kerf.Engine.Foundation.Precision.Confusion();
            bool flag1 = false;
            Curve3D? C = null;

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
                    if (C.IsPeriodic)
                    {
                        double period = C.Period();
                        prmax = prmax2;
                        if (flag1)
                        {
                            prmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(prmin1, prmax - period, prmax);
                        }
                        else
                        {
                            prmin = Min(prmin1, prmin2);
                        }
                        pbmax = prbmax2;
                        pbmin = global::Kerf.Engine.Geometry.ElCLib.InPeriod(prbmin1, pbmax - period, pbmax);
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
                double delta = global::Kerf.Engine.Foundation.Precision.Confusion();

                if (myPerfSelection != BRepFeat_NoSelection)
                {
                    // modif of the test for cts21181 : (prbmax2 and prnmin2) -> (prbmin1 and prbmax1)
                    // correction take into account flag2 for pro15323 and flag3 for pro16060
                    if (!mySUntil.IsNull)
                    {
                        NCollection_Map<Shape> mapFuntil = new(ShapeHasher.Instance);
                        Descendants(mySUntil, theBuilder, mapFuntil);
                        if (!mapFuntil.IsEmpty())
                        {
                            for (it.Initialize(lshape); it.More(); it.Next())
                            {
                                Explorer expf = new();
                                for (expf.Init(it.Value(), ShapeType.Face); expf.More(); expf.Next())
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
                    if (!mySFrom.IsNull)
                    {
                        NCollection_Map<Shape> mapFfrom = new(ShapeHasher.Instance);
                        Descendants(mySFrom, theBuilder, mapFfrom);
                        if (!mapFfrom.IsEmpty())
                        {
                            for (it.Initialize(lshape); it.More(); it.Next())
                            {
                                Explorer expf = new();
                                for (expf.Init(it.Value(), ShapeType.Face); expf.More(); expf.Next())
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
                        if (C.IsPeriodic)
                        {
                            double period = C.Period();
                            BRepFeat.ParametricMinMax(it.Value(), C, out double pr, out prmax1, out double prb, out prbmax1, out flag2, true);
                            if (flag2)
                            {
                                prmin1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(pr, prmax1 - period, prmax1);
                            }
                            else
                            {
                                prmin1 = pr;
                            }
                            prbmin1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(prb, prbmax1 - period, prbmax1);
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
                            Shape S = it.Value();
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
                    Compound Compo = new();
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
                            if (!it.Value().IsNull)
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
    public override bool IsDeleted(in Shape F)
    {
        FinishWorkingShapes();
        if (myMap.IsBound(F))
        {
            return myMap.Find(F).IsEmpty();
        }
        return false;
    }

    /// <summary>The faces generated from the face <paramref name="F"/> (OCCT's virtual; unfrozen).</summary>
    internal override NCollection_List<Shape> InternalModified(in Shape F)
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
            NCollection_List<Shape>.Iterator ite = new(myMap.Find(F));
            for (; ite.More(); ite.Next())
            {
                Shape sh = ite.Value();
                if (!IsSameInHistory(sh, F) && sh.ShapeType() == F.ShapeType())
                {
                    myGenerated.Append(sh);
                }
            }
        }
        return myGenerated; // empty list
    }

    /// <summary>The faces created from the shape <paramref name="S"/> (OCCT's virtual; unfrozen).</summary>
    internal override NCollection_List<Shape> InternalGenerated(in Shape S)
    {
        FinishWorkingShapes();
        myGenerated.Clear();
        if (!IsDone())
        {
            return myGenerated;
        }
        if (myMap.IsBound(S) && S.ShapeType() != ShapeType.Face)
        { // check if filter on face or not
            NCollection_List<Shape>.Iterator ite = new(myMap.Find(S));
            for (; ite.More(); ite.Next())
            {
                Shape sh = ite.Value();
                if (!IsSameInHistory(sh, S))
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
        NCollection_List<Shape>.Iterator it = new(), it2 = new();

        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itdm in myMap)
        {
            Shape orig = itdm.Key;
            // OCCT iterates this address-keyed map into the list: insertion order here (decision 1054)
            NCollection_IndexedMap<Shape> newdsc = new(ShapeHasher.Instance);
            for (it.Initialize(itdm.Value); it.More(); it.Next())
            {
                Face fdsc = ShapeCasts.Face(it.Value());
                for (it2.Initialize(G.InternalDescendantFaces(fdsc)); it2.More(); it2.Next())
                {
                    newdsc.Add(it2.Value());
                }
            }
            myMap.ChangeFind(orig).Clear();
            foreach (Shape itm in newdsc)
            {
                myMap.ChangeFind(orig).Append(itm);
            }
        }
    }

    /// <summary>The shapes created at the bottom of the created form; it may be empty (frozen).</summary>
    public ISchematiqList<Shape> FirstShape()
    {
        BeforePublishingLists();
        return PublishedList(InternalFirstShape());
    }

    /// <summary><see cref="FirstShape"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalFirstShape()
    {
        FinishWorkingShapes();
        if (!myFShape.IsNull)
        {
            return myMap.Find(myFShape);
        }
        return myGenerated; // empty list
    }

    /// <summary>The shapes created at the top of the created form; it may be empty (frozen).</summary>
    public ISchematiqList<Shape> LastShape()
    {
        BeforePublishingLists();
        return PublishedList(InternalLastShape());
    }

    /// <summary><see cref="LastShape"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalLastShape()
    {
        FinishWorkingShapes();
        if (!myLShape.IsNull)
        {
            return myMap.Find(myLShape);
        }
        return myGenerated; // empty list
    }

    /// <summary>
    /// The limiting and glueing edges generated by the feature, which did not exist in the basis shape: the
    /// information needed to add fillets afterwards. It may be empty (frozen).
    /// </summary>
    public ISchematiqList<Shape> NewEdges()
    {
        BeforePublishingLists();
        return PublishedList(InternalNewEdges());
    }

    /// <summary><see cref="NewEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalNewEdges()
    {
        FinishWorkingShapes();
        return myNewEdges;
    }

    /// <summary>
    /// The tangent edges among <see cref="NewEdges"/>: no fillet is possible on them. It may be empty (frozen).
    /// </summary>
    public ISchematiqList<Shape> TgtEdges()
    {
        BeforePublishingLists();
        return PublishedList(InternalTgtEdges());
    }

    /// <summary><see cref="TgtEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalTgtEdges()
    {
        FinishWorkingShapes();
        return myTgtEdges;
    }

    /// <summary>Limitation of the from (flag 0) or until (flag 1) shape in the case of infinite faces.</summary>
    protected bool TransformShapeFU(int flag)
    {
        bool Trf = false;

        Shape shapefu;
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

        Explorer exp = new(shapefu, ShapeType.Face);
        if (!exp.More())
        { // no faces... It is necessary to return an error
            return Trf;
        }

        exp.Next();
        if (!exp.More())
        { // the only face. Is it infinite?
            exp.ReInit();
            Face fac = ShapeCasts.Face(exp.Current());

            Surface S = Tool.Surface(fac)!;
            if (S is TrimmedSurface aTrimmed)
            {
                S = aTrimmed.BasisSurface();
            }

            if (S.GetType() == typeof(PlaneSurface) || S.GetType() == typeof(CylindricalSurface)
                || S.GetType() == typeof(ConicalSurface))
            {
                Explorer exp1 = new(fac, ShapeType.Wire);
                if (!exp1.More())
                {
                    Trf = true;
                }
                else
                {
                    Trf = Tool.NaturalRestriction(fac);
                }
            }
            if (Trf)
            {
                BRepFeat.FaceUntil(mySbase, ref fac);
            }

            if (flag == 0)
            {
                NCollection_List<Shape> thelist6 = new();
                myMap.Bind(mySFrom, thelist6);
                myMap[mySFrom].Append(fac);
                mySFrom = fac;
            }
            else if (flag == 1)
            {
                NCollection_List<Shape> thelist7 = new();
                myMap.Bind(mySUntil, thelist7);
                myMap[mySUntil].Append(fac);
                mySUntil = fac;
            }
        }
        else
        {
            for (exp.ReInit(); exp.More(); exp.Next())
            {
                Shape fac = exp.Current();
                NCollection_List<Shape> thelist8 = new();
                myMap.Bind(fac, thelist8);
                myMap[fac].Append(fac);
            }
        }
        return Trf;
    }

    private static void Descendants(in Shape S, BRepFeat_Builder theFB, NCollection_Map<Shape> mapF)
    {
        mapF.Clear();
        Explorer exp = new();
        for (exp.Init(S, ShapeType.Face); exp.More(); exp.Next())
        {
            Face fdsc = ShapeCasts.Face(exp.Current());
            foreach (Shape aS in theFB.Modified(fdsc))
            {
                mapF.Add(aS);
            }
        }
    }

    protected void UpdateDescendants(BRepAlgoAPI_BooleanOperation aBOP, in Shape S, bool SkipFace = false)
    {
        NCollection_List<Shape>.Iterator it = new(), it2 = new();
        Explorer exp = new();

        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itdm in myMap)
        {
            Shape orig = itdm.Key;
            if (SkipFace && orig.ShapeType() == ShapeType.Face)
            {
                continue;
            }
            // OCCT iterates this address-keyed map into the list: insertion order here (decision 1054)
            NCollection_IndexedMap<Shape> newdsc = new(ShapeHasher.Instance);

            if (itdm.Value.IsEmpty())
            {
                myMap.ChangeFind(orig).Append(orig);
            }

            for (it.Initialize(itdm.Value); it.More(); it.Next())
            {
                Shape sh = it.Value();
                if (sh.ShapeType() != ShapeType.Face)
                {
                    continue;
                }
                Face fdsc = ShapeCasts.Face(it.Value());
                for (exp.Init(S, ShapeType.Face); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(fdsc))
                    { // preserved
                        newdsc.Add(fdsc);
                        break;
                    }
                }
                if (!exp.More())
                {
                    NCollection_List<Shape> aLM = aBOP.InternalModified(fdsc);
                    it2.Initialize(aLM);
                    for (; it2.More(); it2.Next())
                    {
                        Shape aS = it2.Value();
                        newdsc.Add(aS);
                    }
                }
            }
            myMap.ChangeFind(orig).Clear();
            foreach (Shape itm in newdsc)
            {
                // check the appartenance to the shape...
                for (exp.Init(S, ShapeType.Face); exp.More(); exp.Next())
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
