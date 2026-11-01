// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevol.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevol.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepFeat_PerfSelection;
using static Occt.BRepFeat_StatusError;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Describes functions to build revolved features: a profile revolved around an axis, as a depression (a cut) or a
/// protrusion (a fusion).
/// </summary>
/// <remarks>
/// <para>The mode is 0 for a depression, 1 for a protrusion, 2 for the feature alone. The revolution goes for an angle,
/// up to a face, from a face up to another, all around, or up to a face at an angle.</para>
/// <para>The basis shape and the profile are read, never written: the revolution sweeps a copy of a frozen profile
/// (LocOpe_Revol, decision 1014).</para>
/// </remarks>
public sealed class BRepFeat_MakeRevol : BRepFeat_Form
{
    private TopoDS_Shape myPbase = new();
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mySlface = new(TopTools_ShapeMapHasher.Instance);
    private gp_Ax1 myAxis = new();
    private readonly NCollection_Sequence<Geom_Curve> myCurves = new();
    private Geom_Curve? myBCurve;
    // OCCT: the class declares its own myStatusError, hiding BRepFeat_Form's; CurrentStatusError reads the form's
#pragma warning disable CS0414 // written, never read: as in OCCT
    private BRepFeat_StatusError myStatusError;
#pragma warning restore CS0414

    /// <summary>An empty revolved feature.</summary>
    public BRepFeat_MakeRevol()
    {
        myStatusError = BRepFeat_OK;
    }

    /// <summary>
    /// A revolved feature of the profile <paramref name="Pbase"/> on <paramref name="Sbase"/> around
    /// <paramref name="Axis"/>, from the sketch face <paramref name="Skface"/>; <paramref name="Fuse"/> is the mode
    /// (0 cut, 1 fuse, 2 the feature alone).
    /// </summary>
    public BRepFeat_MakeRevol(in TopoDS_Shape Sbase, in TopoDS_Shape Pbase, in TopoDS_Face Skface, in gp_Ax1 Axis,
                              int Fuse, bool Modify)
    {
        Init(Sbase, Pbase, Skface, Axis, Fuse, Modify);
    }

    /// <summary>Initializes the revolved feature (see the constructor).</summary>
    public void Init(in TopoDS_Shape Sbase, in TopoDS_Shape Pbase, in TopoDS_Face Skface, in gp_Ax1 Axis, int Mode,
                     bool Modify)
    {
        // OCCT.NET: the working copies of frozen inputs (decision 1052)
        StartWorkingShapes();
        myAxis = Axis;
        myPbase = Working(Pbase);
        mySbase = Working(Sbase);
        BasisShapeValid();
        mySkface = Working(Skface);
        SketchFaceValid();
        myPbase = Working(Pbase);
        mySlface.Clear();
        if (Mode == 0)
        {
            myFuse = false;
            myJustFeat = false;
        }
        else if (Mode == 1)
        {
            myFuse = true;
            myJustFeat = false;
        }
        else if (Mode == 2)
        {
            myFuse = true;
            myJustFeat = true;
        }
        myModify = Modify;
        myJustGluer = false;

        myShape.Nullify();
        myMap.Clear();
        myFShape.Nullify();
        myLShape.Nullify();
        TopExp_Explorer exp = new();
        for (exp.Init(mySbase, TopAbs_FACE); exp.More(); exp.Next())
        {
            NCollection_List<TopoDS_Shape> thelist = new();
            myMap.Bind(exp.Current(), thelist);
            myMap[exp.Current()].Append(exp.Current());
        }
        // OCCT_DEBUG trace left out
    }

    /// <summary>Adds an element of sliding: the edge <paramref name="E"/> of the profile slides on the face <paramref name="F"/>.</summary>
    /// <exception cref="Standard_ConstructionError">F isn't a face of the basis shape, or E an edge of the profile.</exception>
    public void Add(in TopoDS_Edge E, in TopoDS_Face F)
    {
        TopExp_Explorer exp = new();
        for (exp.Init(mySbase, TopAbs_FACE); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(Working(F)))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new Standard_ConstructionError();
        }

        for (exp.Init(myPbase, TopAbs_EDGE); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(Working(E)))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new Standard_ConstructionError();
        }

        if (!mySlface.IsBound(F))
        {
            NCollection_List<TopoDS_Shape> thelist = new();
            mySlface.Bind(F, thelist);
        }
        NCollection_List<TopoDS_Shape>.Iterator itl = new(mySlface[F]);
        for (; itl.More(); itl.Next())
        {
            if (itl.Value().IsSame(E))
            {
                break;
            }
        }
        if (!itl.More())
        {
            mySlface[F].Append(E);
        }
    }

    /// <summary>Builds a revolved feature of the angle <paramref name="Angle"/> (radians).</summary>
    public void Perform(double Angle)
    {
        mySFrom.Nullify();
        ShapeFromValid();
        mySUntil.Nullify();
        ShapeUntilValid();
        myGluedF.Clear();
        myPerfSelection = BRepFeat_NoSelection;
        PerfSelectionValid();
        bool RevolComp = 2 * M_PI - Math.Abs(Angle) <= Precision.Angular();
        LocOpe_Revol theRevol = new();
        double angledec = 0.0;
        TopExp_Explorer exp = new();
        if (RevolComp)
        {
            mySkface.Nullify();
        }
        if (angledec == 0.0)
        {
            theRevol.Perform(myPbase, myAxis, Angle);
        }
        else
        {
            theRevol.Perform(myPbase, myAxis, Angle, angledec);
        }
        TopoDS_Shape VraiRevol = theRevol.InternalShape();

        MajMap(myPbase, theRevol, myMap, ref myFShape, ref myLShape);

        myGShape = VraiRevol;
        GeneratedShapeValid();
        TopoDS_Shape Base = theRevol.InternalFirstShape();
        exp.Init(Base, TopAbs_FACE);
        TopoDS_Face theBase = TopoDS.Face(exp.Current());
        exp.Next();
        if (exp.More())
        {
            NotDone();
            myStatusError = BRepFeat_InvFirstShape;
            return;
        }

        TopoDS_Face FFace = new();

        bool found = false;

        if (!mySkface.IsNull() || !mySlface.IsEmpty())
        {
            if (myLShape.ShapeType() == TopAbs_WIRE)
            {
                TopExp_Explorer ex1 = new(VraiRevol, TopAbs_FACE);
                for (; ex1.More(); ex1.Next())
                {
                    TopExp_Explorer ex2 = new(ex1.Current(), TopAbs_WIRE);
                    for (; ex2.More(); ex2.Next())
                    {
                        if (ex2.Current().IsSame(myLShape))
                        {
                            FFace = TopoDS.Face(ex1.Current());
                            found = true;
                            break;
                        }
                    }
                    if (found)
                    {
                        break;
                    }
                }
            }

            TopExp_Explorer anExp = new(mySbase, TopAbs_FACE);
            for (; anExp.More(); anExp.Next())
            {
                TopoDS_Face ff = TopoDS.Face(anExp.Current());
                if (ToFuse(ff, FFace))
                {
                    if (!FFace.IsSame(myPbase) && BRepFeat.IsInside(ff, FFace))
                    {
                        break;
                    }
                }
            }
        }

        GluedFacesValid();
        if (!mySkface.IsNull())
        {
            VerifGluedFaces(mySkface, theBase, ref myBCurve, myCurves, theRevol, myGluedF);
        }

        if (myGluedF.IsEmpty())
        {
            if (myFuse)
            {
                BRepAlgoAPI_Fuse f = new(mySbase, myGShape);
                myShape = f.InternalShape();
                UpdateDescendants(f, myShape, false);
                Done();
            }
            else
            {
                BRepAlgoAPI_Cut c = new(mySbase, myGShape);
                myShape = c.InternalShape();
                UpdateDescendants(c, myShape, false);
                Done();
            }
        }
        else
        {
            theRevol.Curves(myCurves);
            myBCurve = theRevol.BarycCurve();
            GlobalPerform();
        }
    }

    /// <summary>Builds a revolved feature up to the shape <paramref name="Until"/>.</summary>
    /// <exception cref="Standard_ConstructionError">Until is null or has no face.</exception>
    public void Perform(in TopoDS_Shape Until)
    {
        double Angle = 0.0;
        bool TourComplet = false;

        if (Until.IsNull())
        {
            throw new Standard_ConstructionError();
        }
        TopExp_Explorer exp = new(Until, TopAbs_FACE);
        if (!exp.More())
        {
            throw new Standard_ConstructionError();
        }
        if (!mySkface.IsNull() && Working(Until).IsSame(mySkface))
        {
            Angle = 2 * M_PI;
            TourComplet = true;
        }
        myGluedF.Clear();
        myPerfSelection = BRepFeat_SelectionU;
        PerfSelectionValid();
        mySFrom.Nullify();
        ShapeFromValid();
        mySUntil = Working(Until);
        bool Trf = TransformShapeFU(1);
        ShapeUntilValid();

        // Do systematically almost complete revolution
        LocOpe_Revol theRevol = new();
        if (!TourComplet)
        {
            Angle = 2.0 * M_PI - 3 * M_PI / 180.0;
        }
        theRevol.Perform(myPbase, myAxis, Angle);
        TopoDS_Shape VraiRevol = theRevol.InternalShape();
        MajMap(myPbase, theRevol, myMap, ref myFShape, ref myLShape);

        if (!Trf)
        {
            myGShape = VraiRevol;
            GeneratedShapeValid();

            TopoDS_Shape Base = theRevol.InternalFirstShape();
            exp.Init(Base, TopAbs_FACE);
            exp.Next();
            if (exp.More())
            {
                NotDone();
                myStatusError = BRepFeat_InvFirstShape;
                return;
            }

            GluedFacesValid();
            theRevol.Curves(myCurves);
            myBCurve = theRevol.BarycCurve();
            GlobalPerform();
        }
        else
        {
            NCollection_Sequence<Geom_Curve?> scur = new();
            theRevol.Curves(myCurves);
            myBCurve = theRevol.BarycCurve();
            scur.Clear();
            scur.Append(myBCurve);
            LocOpe_CSIntersector ASI = new(mySUntil);
            ASI.Perform(scur);
            if (ASI.IsDone() && ASI.NbPoints(1) >= 1)
            {
                TopAbs_Orientation Or = ASI.Point(1, 1).Orientation();
                TopoDS_Face FUntil = ASI.Point(1, 1).Face();
                TopoDS_Compound Comp = new();
                BRep_Builder B = new();
                B.MakeCompound(ref Comp);
                TopoDS_Solid S = BRepFeat.Tool(mySUntil, FUntil, Or);
                if (!S.IsNull())
                {
                    B.Add(Comp, S);
                }
                BRepAlgoAPI_Cut trP = new(VraiRevol, Comp);
                TopoDS_Shape Cutsh = trP.InternalShape();
                TopExp_Explorer ex = new(Cutsh, TopAbs_SOLID);
                for (; ex.More(); ex.Next())
                {
                    TopExp_Explorer ex1 = new(ex.Current(), TopAbs_FACE);
                    for (; ex1.More(); ex1.Next())
                    {
                        TopoDS_Face fac = TopoDS.Face(ex1.Current());
                        if (fac.IsSame(myPbase))
                        {
                            VraiRevol = ex.Current();
                            break;
                        }
                    }
                }
                if (myFuse)
                {
                    BRepAlgoAPI_Fuse f = new(mySbase, VraiRevol);
                    myShape = f.InternalShape();
                    UpdateDescendants(f, myShape, false);
                    Done();
                }
                else
                {
                    BRepAlgoAPI_Cut c = new(mySbase, VraiRevol);
                    myShape = c.InternalShape();
                    UpdateDescendants(c, myShape, false);
                    Done();
                }
            }
        }
    }

    /// <summary>Builds a revolved feature from the shape <paramref name="From"/> up to the shape <paramref name="Until"/>.</summary>
    /// <exception cref="Standard_ConstructionError">From or Until is null or has no face.</exception>
    public void Perform(in TopoDS_Shape From, in TopoDS_Shape Until)
    {
        if (From.IsNull() || Until.IsNull())
        {
            throw new Standard_ConstructionError();
        }
        if (!mySkface.IsNull())
        {
            if (Working(From).IsSame(mySkface))
            {
                myJustGluer = true;
                Perform(Until);
                if (myJustGluer)
                {
                    return;
                }
            }
            else if (Working(Until).IsSame(mySkface))
            {
                myJustGluer = true;
                myAxis.Reverse();
                Perform(From);
                if (myJustGluer)
                {
                    return;
                }
            }
        }

        myGluedF.Clear();
        myPerfSelection = BRepFeat_SelectionFU;
        PerfSelectionValid();

        TopExp_Explorer exp = new(From, TopAbs_FACE);
        if (!exp.More())
        {
            throw new Standard_ConstructionError();
        }
        exp.Init(Until, TopAbs_FACE);
        if (!exp.More())
        {
            throw new Standard_ConstructionError();
        }
        mySFrom = Working(From);
        bool Trff = TransformShapeFU(0);
        ShapeFromValid();
        mySUntil = Working(Until);
        bool Trfu = TransformShapeFU(1);
        ShapeUntilValid();
        if (Trfu != Trff)
        {
            NotDone();
            myStatusError = BRepFeat_IncTypes;
            return;
        }

        LocOpe_Revol theRevol = new();
        theRevol.Perform(myPbase, myAxis, 2 * M_PI);
        TopoDS_Shape VraiRevol = theRevol.InternalShape();

        MajMap(myPbase, theRevol, myMap, ref myFShape, ref myLShape);

        if (!Trff)
        {
            myGShape = VraiRevol;
            GeneratedShapeValid();
            GluedFacesValid();
            theRevol.Curves(myCurves);
            myBCurve = theRevol.BarycCurve();
            GlobalPerform();
        }
        else
        {
            theRevol.Curves(myCurves);
            myBCurve = theRevol.BarycCurve();
            NCollection_Sequence<Geom_Curve?> scur = new();
            scur.Clear();
            scur.Append(myBCurve);
            LocOpe_CSIntersector ASI1 = new(mySUntil);
            LocOpe_CSIntersector ASI2 = new(mySFrom);
            ASI1.Perform(scur);
            ASI2.Perform(scur);
            TopAbs_Orientation OrU, OrF;
            TopoDS_Face FFrom, FUntil;
            double PrF, PrU;
            if (ASI1.IsDone() && ASI1.NbPoints(1) >= 1)
            {
                OrU = ASI1.Point(1, 1).Orientation();
                FUntil = ASI1.Point(1, 1).Face();
                PrU = ASI1.Point(1, 1).Parameter();
            }
            else
            {
                NotDone();
                myStatusError = BRepFeat_NoIntersectU;
                return;
            }
            if (ASI2.IsDone() && ASI2.NbPoints(1) >= 1)
            {
                double pr1 = ASI2.Point(1, 1).Parameter();
                pr1 = ElCLib.InPeriod(pr1, PrU - 2 * M_PI, PrU);
                double pr2 = ASI2.Point(1, ASI2.NbPoints(1)).Parameter();
                pr2 = ElCLib.InPeriod(pr2, PrU - 2 * M_PI, PrU);
                OrF = TopAbs.Reverse(OrU);
                FFrom = ASI2.Point(1, 1).Face();
                PrF = Max(pr1, pr2);
            }
            else
            {
                NotDone();
                myStatusError = BRepFeat_NoIntersectF;
                return;
            }
            if (!(PrU > PrF))
            {
                NotDone();
                myStatusError = BRepFeat_IncParameter;
                return;
            }

            TopoDS_Compound Comp = new();
            BRep_Builder B = new();
            B.MakeCompound(ref Comp);
            TopoDS_Solid SF = BRepFeat.Tool(mySFrom, FFrom, OrF);
            if (!SF.IsNull())
            {
                B.Add(Comp, SF);
            }
            TopoDS_Solid SU = BRepFeat.Tool(mySUntil, FUntil, OrU);
            if (!SU.IsNull())
            {
                B.Add(Comp, SU);
            }
            BRepAlgoAPI_Cut trP = new(VraiRevol, Comp);
            TopoDS_Shape Cutsh = trP.InternalShape();
            TopExp_Explorer ex = new(Cutsh, TopAbs_SOLID);
            VraiRevol = ex.Current();
            for (; ex.More(); ex.Next())
            {
                double PrCur = BRepFeat.ParametricBarycenter(ex.Current(), myBCurve!);
                if (PrF <= PrCur && PrU >= PrCur)
                {
                    VraiRevol = ex.Current();
                    break;
                }
            }
            if (myFuse && !myJustFeat)
            {
                BRepAlgoAPI_Fuse f = new(mySbase, VraiRevol);
                myShape = f.InternalShape();
                UpdateDescendants(f, myShape, false);
                Done();
            }
            else if (!myFuse && !myJustFeat)
            {
                BRepAlgoAPI_Cut c = new(mySbase, VraiRevol);
                myShape = c.InternalShape();
                UpdateDescendants(c, myShape, false);
                Done();
            }
            else
            {
                myShape = VraiRevol;
                Done();
            }
        }
    }

    /// <summary>Builds a revolved feature all around the axis.</summary>
    public void PerformThruAll()
    {
        Perform(2.0 * M_PI);
    }

    /// <summary>Builds a revolved feature up to the shape <paramref name="Until"/>, at the angle <paramref name="Angle"/>.</summary>
    /// <exception cref="Standard_ConstructionError">Until has no face (a null Until included, see below).</exception>
    public void PerformUntilAngle(in TopoDS_Shape Until, double Angle)
    {
        // OCCT: neither of the two first calls returns: a null Until goes on and throws below, a zero angle goes on and
        // builds the revolution again with the angle 0
        if (Until.IsNull())
        {
            Perform(Angle);
        }
        if (Angle == 0)
        {
            Perform(Until);
        }
        TopExp_Explorer exp = new(Until, TopAbs_FACE);
        if (!exp.More())
        {
            throw new Standard_ConstructionError();
        }

        if (!mySkface.IsNull() && Working(Until).IsSame(mySkface))
        {
            Perform(Angle);
            return;
        }

        myGluedF.Clear();
        myPerfSelection = BRepFeat_NoSelection;
        PerfSelectionValid();
        mySFrom.Nullify();
        ShapeFromValid();
        mySUntil = Working(Until);
        bool Trf = TransformShapeFU(1);
        ShapeUntilValid();

        // Produce systematicallt an almost complete revolution
        LocOpe_Revol theRevol = new();
        theRevol.Perform(myPbase, myAxis, Angle);
        TopoDS_Shape VraiRevol = theRevol.InternalShape();

        MajMap(myPbase, theRevol, myMap, ref myFShape, ref myLShape);

        // OCCT: the branches are the other way round from Perform(Until): a face until is cut, a support glued
        if (Trf)
        {
            myGShape = VraiRevol;
            GeneratedShapeValid();

            TopoDS_Shape Base = theRevol.InternalFirstShape();
            exp.Init(Base, TopAbs_FACE);
            exp.Next();
            if (exp.More())
            {
                NotDone();
                myStatusError = BRepFeat_InvFirstShape;
                return;
            }

            GluedFacesValid();
            theRevol.Curves(myCurves);
            myBCurve = theRevol.BarycCurve();
            GlobalPerform();
        }
        else
        {
            NCollection_Sequence<Geom_Curve?> scur = new();
            theRevol.Curves(myCurves);
            myBCurve = theRevol.BarycCurve();
            scur.Clear();
            scur.Append(myBCurve);
            LocOpe_CSIntersector ASI = new(mySUntil);
            ASI.Perform(scur);
            if (ASI.IsDone() && ASI.NbPoints(1) >= 1)
            {
                TopAbs_Orientation Or = ASI.Point(1, 1).Orientation();
                TopoDS_Face FUntil = ASI.Point(1, 1).Face();
                TopoDS_Compound Comp = new();
                BRep_Builder B = new();
                B.MakeCompound(ref Comp);
                TopoDS_Solid S = BRepFeat.Tool(mySUntil, FUntil, Or);
                if (!S.IsNull())
                {
                    B.Add(Comp, S);
                }
                BRepAlgoAPI_Cut trP = new(VraiRevol, Comp);
                TopoDS_Shape Cutsh = trP.InternalShape();
                TopExp_Explorer ex = new(Cutsh, TopAbs_SOLID);
                for (; ex.More(); ex.Next())
                {
                    TopExp_Explorer ex1 = new(ex.Current(), TopAbs_FACE);
                    for (; ex1.More(); ex1.Next())
                    {
                        TopoDS_Face fac = TopoDS.Face(ex1.Current());
                        if (fac.IsSame(myPbase))
                        {
                            VraiRevol = ex.Current();
                            break;
                        }
                    }
                }
                if (myFuse)
                {
                    BRepAlgoAPI_Fuse f = new(mySbase, VraiRevol);
                    myShape = f.InternalShape();
                    UpdateDescendants(f, myShape, false);
                    Done();
                }
                else
                {
                    BRepAlgoAPI_Cut c = new(mySbase, VraiRevol);
                    myShape = c.InternalShape();
                    UpdateDescendants(c, myShape, false);
                    Done();
                }
            }
        }
    }

    /// <summary>The curves of the revolution.</summary>
    public override void Curves(NCollection_Sequence<Geom_Curve> scur)
    {
        scur.Assign(myCurves);
    }

    /// <summary>The circle of the revolution through the barycentre of the profile.</summary>
    public override Geom_Curve? BarycCurve()
    {
        return myBCurve;
    }

    // Checks whether the gluing faces are valid: if not, the case without gluing
    private static void VerifGluedFaces(in TopoDS_Face theSkface, in TopoDS_Shape thePbase, ref Geom_Curve? theBCurve,
                                        NCollection_Sequence<Geom_Curve> theCurves, LocOpe_Revol theRevol,
                                        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theMap)
    {
        bool GluedFaces = true;
        TopoDS_Shape VraiRevol = theRevol.InternalShape();

        NCollection_Sequence<Geom_Curve?> scur = new();
        theRevol.Curves(theCurves);
        theBCurve = theRevol.BarycCurve();
        scur.Clear();
        scur.Append(theBCurve);
        LocOpe_CSIntersector ASI = new(theSkface);
        ASI.Perform(scur);
        if (ASI.IsDone() && ASI.NbPoints(1) >= 1)
        {
            TopAbs_Orientation Or = ASI.Point(1, 1).Orientation();
            TopoDS_Face FSk = ASI.Point(1, 1).Face();
            TopoDS_Compound Comp = new();
            BRep_Builder B = new();
            B.MakeCompound(ref Comp);
            TopoDS_Solid S = BRepFeat.Tool(theSkface, FSk, Or);
            if (!S.IsNull())
            {
                B.Add(Comp, S);
            }
            BRepAlgoAPI_Cut trP = new(VraiRevol, Comp);
            TopoDS_Shape Cutsh = trP.InternalShape();
            TopExp_Explorer ex = new(Cutsh, TopAbs_SOLID);
            for (; ex.More(); ex.Next())
            {
                TopExp_Explorer ex1 = new(ex.Current(), TopAbs_FACE);
                for (; ex1.More(); ex1.Next())
                {
                    TopoDS_Face fac1 = TopoDS.Face(ex1.Current());
                    TopExp_Explorer ex2 = new(thePbase, TopAbs_FACE);
                    for (; ex2.More(); ex2.Next())
                    {
                        TopoDS_Face fac2 = TopoDS.Face(ex2.Current());
                        if (fac1.IsSame(fac2))
                        {
                            break;
                        }
                    }
                    if (ex2.More())
                    {
                        break;
                    }
                }
                if (ex1.More())
                {
                    continue;
                }
                GluedFaces = false;
                break;
            }
            if (!GluedFaces)
            {
                theMap.Clear();
            }
        }
    }

    private static void MajMap(in TopoDS_Shape theB, LocOpe_Revol theP,
                               NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMap,
                               ref TopoDS_Shape theFShape, ref TopoDS_Shape theLShape)
    {
        TopExp_Explorer exp = new(theP.InternalFirstShape(), TopAbs_WIRE);
        if (exp.More())
        {
            theFShape = exp.Current();
            NCollection_List<TopoDS_Shape> thelist = new();
            theMap.Bind(theFShape, thelist);
            for (exp.Init(theP.InternalFirstShape(), TopAbs_FACE); exp.More(); exp.Next())
            {
                theMap[theFShape].Append(exp.Current());
            }
        }

        exp.Init(theP.InternalLastShape(), TopAbs_WIRE);
        if (exp.More())
        {
            theLShape = exp.Current();
            NCollection_List<TopoDS_Shape> thelist1 = new();
            theMap.Bind(theLShape, thelist1);
            for (exp.Init(theP.InternalLastShape(), TopAbs_FACE); exp.More(); exp.Next())
            {
                theMap[theLShape].Append(exp.Current());
            }
        }

        for (exp.Init(theB, TopAbs_EDGE); exp.More(); exp.Next())
        {
            if (!theMap.IsBound(exp.Current()))
            {
                theMap.Bind(exp.Current(), new NCollection_List<TopoDS_Shape>(theP.InternalShapes(exp.Current())));
            }
        }
    }

    private static bool ToFuse(in TopoDS_Face F1, in TopoDS_Face F2)
    {
        if (F1.IsNull() || F2.IsNull())
        {
            return false;
        }

        Geom_Surface S1, S2;
        double tollin = Precision.Confusion();
        double tolang = Precision.Angular();

        S1 = BRep_Tool.Surface(F1, out _)!;
        S2 = BRep_Tool.Surface(F2, out _)!;

        if (S1.GetType() == typeof(Geom_RectangularTrimmedSurface))
        {
            S1 = ((Geom_RectangularTrimmedSurface)S1).BasisSurface();
        }

        if (S2.GetType() == typeof(Geom_RectangularTrimmedSurface))
        {
            S2 = ((Geom_RectangularTrimmedSurface)S2).BasisSurface();
        }

        if (S1.GetType() != S2.GetType())
        {
            return false;
        }

        bool ValRet = false;
        if (S1.GetType() == typeof(Geom_Plane))
        {
            S1 = BRep_Tool.Surface(F1)!; // to apply the location.
            S2 = BRep_Tool.Surface(F2)!;
            // OCCT: a trimmed plane gives a null plane here (an access violation natively)
            gp_Pln pl1 = (S1 as Geom_Plane)!.Pln();
            gp_Pln pl2 = (S2 as Geom_Plane)!.Pln();

            if (pl1.Position().IsCoplanar(pl2.Position(), tollin, tolang))
            {
                ValRet = true;
            }
        }

        return ValRet;
    }
}
