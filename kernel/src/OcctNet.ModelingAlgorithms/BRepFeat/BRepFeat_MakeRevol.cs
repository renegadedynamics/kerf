// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevol.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevol.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepFeat_PerfSelection;
using static Occt.BRepFeat_StatusError;
using static Kerf.Topology.ShapeType;

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
    private Shape myPbase = new();
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> mySlface = new(ShapeHasher.Instance);
    private Axis3D myAxis = new();
    private readonly NCollection_Sequence<Curve3D> myCurves = new();
    private Curve3D? myBCurve;
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
    public BRepFeat_MakeRevol(in Shape Sbase, in Shape Pbase, in Face Skface, in Axis3D Axis,
                              int Fuse, bool Modify)
    {
        Init(Sbase, Pbase, Skface, Axis, Fuse, Modify);
    }

    /// <summary>Initializes the revolved feature (see the constructor).</summary>
    public void Init(in Shape Sbase, in Shape Pbase, in Face Skface, in Axis3D Axis, int Mode,
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
        Explorer exp = new();
        for (exp.Init(mySbase, ShapeType.Face); exp.More(); exp.Next())
        {
            NCollection_List<Shape> thelist = new();
            myMap.Bind(exp.Current(), thelist);
            myMap[exp.Current()].Append(exp.Current());
        }
        // OCCT_DEBUG trace left out
    }

    /// <summary>Adds an element of sliding: the edge <paramref name="E"/> of the profile slides on the face <paramref name="F"/>.</summary>
    /// <exception cref="ConstructionErrorException">F isn't a face of the basis shape, or E an edge of the profile.</exception>
    public void Add(in Edge E, in Face F)
    {
        Explorer exp = new();
        for (exp.Init(mySbase, ShapeType.Face); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(Working(F)))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }

        for (exp.Init(myPbase, ShapeType.Edge); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(Working(E)))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }

        if (!mySlface.IsBound(F))
        {
            NCollection_List<Shape> thelist = new();
            mySlface.Bind(F, thelist);
        }
        NCollection_List<Shape>.Iterator itl = new(mySlface[F]);
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
        bool RevolComp = 2 * M_PI - Math.Abs(Angle) <= global::Kerf.Engine.Foundation.Precision.Angular();
        LocOpe_Revol theRevol = new();
        double angledec = 0.0;
        Explorer exp = new();
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
        Shape VraiRevol = theRevol.InternalShape();

        MajMap(myPbase, theRevol, myMap, ref myFShape, ref myLShape);

        myGShape = VraiRevol;
        GeneratedShapeValid();
        Shape Base = theRevol.InternalFirstShape();
        exp.Init(Base, ShapeType.Face);
        Face theBase = ShapeCasts.Face(exp.Current());
        exp.Next();
        if (exp.More())
        {
            NotDone();
            myStatusError = BRepFeat_InvFirstShape;
            return;
        }

        Face FFace = new();

        bool found = false;

        if (!mySkface.IsNull || !mySlface.IsEmpty())
        {
            if (myLShape.ShapeType() == ShapeType.Wire)
            {
                Explorer ex1 = new(VraiRevol, ShapeType.Face);
                for (; ex1.More(); ex1.Next())
                {
                    Explorer ex2 = new(ex1.Current(), ShapeType.Wire);
                    for (; ex2.More(); ex2.Next())
                    {
                        if (ex2.Current().IsSame(myLShape))
                        {
                            FFace = ShapeCasts.Face(ex1.Current());
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

            Explorer anExp = new(mySbase, ShapeType.Face);
            for (; anExp.More(); anExp.Next())
            {
                Face ff = ShapeCasts.Face(anExp.Current());
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
        if (!mySkface.IsNull)
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
    /// <exception cref="ConstructionErrorException">Until is null or has no face.</exception>
    public void Perform(in Shape Until)
    {
        double Angle = 0.0;
        bool TourComplet = false;

        if (Until.IsNull)
        {
            throw new ConstructionErrorException();
        }
        Explorer exp = new(Until, ShapeType.Face);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }
        if (!mySkface.IsNull && Working(Until).IsSame(mySkface))
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
        Shape VraiRevol = theRevol.InternalShape();
        MajMap(myPbase, theRevol, myMap, ref myFShape, ref myLShape);

        if (!Trf)
        {
            myGShape = VraiRevol;
            GeneratedShapeValid();

            Shape Base = theRevol.InternalFirstShape();
            exp.Init(Base, ShapeType.Face);
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
            NCollection_Sequence<Curve3D?> scur = new();
            theRevol.Curves(myCurves);
            myBCurve = theRevol.BarycCurve();
            scur.Clear();
            scur.Append(myBCurve);
            LocOpe_CSIntersector ASI = new(mySUntil);
            ASI.Perform(scur);
            if (ASI.IsDone() && ASI.NbPoints(1) >= 1)
            {
                Orientation Or = ASI.Point(1, 1).Orientation();
                Face FUntil = ASI.Point(1, 1).Face();
                Compound Comp = new();
                BrepBuilder B = new();
                B.MakeCompound(ref Comp);
                Solid S = BRepFeat.Tool(mySUntil, FUntil, Or);
                if (!S.IsNull)
                {
                    B.Add(Comp, S);
                }
                BRepAlgoAPI_Cut trP = new(VraiRevol, Comp);
                Shape Cutsh = trP.InternalShape();
                Explorer ex = new(Cutsh, ShapeType.Solid);
                for (; ex.More(); ex.Next())
                {
                    Explorer ex1 = new(ex.Current(), ShapeType.Face);
                    for (; ex1.More(); ex1.Next())
                    {
                        Face fac = ShapeCasts.Face(ex1.Current());
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
    /// <exception cref="ConstructionErrorException">From or Until is null or has no face.</exception>
    public void Perform(in Shape From, in Shape Until)
    {
        if (From.IsNull || Until.IsNull)
        {
            throw new ConstructionErrorException();
        }
        if (!mySkface.IsNull)
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

        Explorer exp = new(From, ShapeType.Face);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }
        exp.Init(Until, ShapeType.Face);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
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
        Shape VraiRevol = theRevol.InternalShape();

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
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(myBCurve);
            LocOpe_CSIntersector ASI1 = new(mySUntil);
            LocOpe_CSIntersector ASI2 = new(mySFrom);
            ASI1.Perform(scur);
            ASI2.Perform(scur);
            Orientation OrU, OrF;
            Face FFrom, FUntil;
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
                pr1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(pr1, PrU - 2 * M_PI, PrU);
                double pr2 = ASI2.Point(1, ASI2.NbPoints(1)).Parameter();
                pr2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(pr2, PrU - 2 * M_PI, PrU);
                OrF = global::Kerf.Engine.Topology.TopAbs.Reverse(OrU);
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

            Compound Comp = new();
            BrepBuilder B = new();
            B.MakeCompound(ref Comp);
            Solid SF = BRepFeat.Tool(mySFrom, FFrom, OrF);
            if (!SF.IsNull)
            {
                B.Add(Comp, SF);
            }
            Solid SU = BRepFeat.Tool(mySUntil, FUntil, OrU);
            if (!SU.IsNull)
            {
                B.Add(Comp, SU);
            }
            BRepAlgoAPI_Cut trP = new(VraiRevol, Comp);
            Shape Cutsh = trP.InternalShape();
            Explorer ex = new(Cutsh, ShapeType.Solid);
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
    /// <exception cref="ConstructionErrorException">Until has no face (a null Until included, see below).</exception>
    public void PerformUntilAngle(in Shape Until, double Angle)
    {
        // OCCT: neither of the two first calls returns: a null Until goes on and throws below, a zero angle goes on and
        // builds the revolution again with the angle 0
        if (Until.IsNull)
        {
            Perform(Angle);
        }
        if (Angle == 0)
        {
            Perform(Until);
        }
        Explorer exp = new(Until, ShapeType.Face);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }

        if (!mySkface.IsNull && Working(Until).IsSame(mySkface))
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
        Shape VraiRevol = theRevol.InternalShape();

        MajMap(myPbase, theRevol, myMap, ref myFShape, ref myLShape);

        // OCCT: the branches are the other way round from Perform(Until): a face until is cut, a support glued
        if (Trf)
        {
            myGShape = VraiRevol;
            GeneratedShapeValid();

            Shape Base = theRevol.InternalFirstShape();
            exp.Init(Base, ShapeType.Face);
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
            NCollection_Sequence<Curve3D?> scur = new();
            theRevol.Curves(myCurves);
            myBCurve = theRevol.BarycCurve();
            scur.Clear();
            scur.Append(myBCurve);
            LocOpe_CSIntersector ASI = new(mySUntil);
            ASI.Perform(scur);
            if (ASI.IsDone() && ASI.NbPoints(1) >= 1)
            {
                Orientation Or = ASI.Point(1, 1).Orientation();
                Face FUntil = ASI.Point(1, 1).Face();
                Compound Comp = new();
                BrepBuilder B = new();
                B.MakeCompound(ref Comp);
                Solid S = BRepFeat.Tool(mySUntil, FUntil, Or);
                if (!S.IsNull)
                {
                    B.Add(Comp, S);
                }
                BRepAlgoAPI_Cut trP = new(VraiRevol, Comp);
                Shape Cutsh = trP.InternalShape();
                Explorer ex = new(Cutsh, ShapeType.Solid);
                for (; ex.More(); ex.Next())
                {
                    Explorer ex1 = new(ex.Current(), ShapeType.Face);
                    for (; ex1.More(); ex1.Next())
                    {
                        Face fac = ShapeCasts.Face(ex1.Current());
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
    public override void Curves(NCollection_Sequence<Curve3D> scur)
    {
        scur.Assign(myCurves);
    }

    /// <summary>The circle of the revolution through the barycentre of the profile.</summary>
    public override Curve3D? BarycCurve()
    {
        return myBCurve;
    }

    // Checks whether the gluing faces are valid: if not, the case without gluing
    private static void VerifGluedFaces(in Face theSkface, in Shape thePbase, ref Curve3D? theBCurve,
                                        NCollection_Sequence<Curve3D> theCurves, LocOpe_Revol theRevol,
                                        NCollection_DataMap<Shape, Shape> theMap)
    {
        bool GluedFaces = true;
        Shape VraiRevol = theRevol.InternalShape();

        NCollection_Sequence<Curve3D?> scur = new();
        theRevol.Curves(theCurves);
        theBCurve = theRevol.BarycCurve();
        scur.Clear();
        scur.Append(theBCurve);
        LocOpe_CSIntersector ASI = new(theSkface);
        ASI.Perform(scur);
        if (ASI.IsDone() && ASI.NbPoints(1) >= 1)
        {
            Orientation Or = ASI.Point(1, 1).Orientation();
            Face FSk = ASI.Point(1, 1).Face();
            Compound Comp = new();
            BrepBuilder B = new();
            B.MakeCompound(ref Comp);
            Solid S = BRepFeat.Tool(theSkface, FSk, Or);
            if (!S.IsNull)
            {
                B.Add(Comp, S);
            }
            BRepAlgoAPI_Cut trP = new(VraiRevol, Comp);
            Shape Cutsh = trP.InternalShape();
            Explorer ex = new(Cutsh, ShapeType.Solid);
            for (; ex.More(); ex.Next())
            {
                Explorer ex1 = new(ex.Current(), ShapeType.Face);
                for (; ex1.More(); ex1.Next())
                {
                    Face fac1 = ShapeCasts.Face(ex1.Current());
                    Explorer ex2 = new(thePbase, ShapeType.Face);
                    for (; ex2.More(); ex2.Next())
                    {
                        Face fac2 = ShapeCasts.Face(ex2.Current());
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

    private static void MajMap(in Shape theB, LocOpe_Revol theP,
                               NCollection_DataMap<Shape, NCollection_List<Shape>> theMap,
                               ref Shape theFShape, ref Shape theLShape)
    {
        Explorer exp = new(theP.InternalFirstShape(), ShapeType.Wire);
        if (exp.More())
        {
            theFShape = exp.Current();
            NCollection_List<Shape> thelist = new();
            theMap.Bind(theFShape, thelist);
            for (exp.Init(theP.InternalFirstShape(), ShapeType.Face); exp.More(); exp.Next())
            {
                theMap[theFShape].Append(exp.Current());
            }
        }

        exp.Init(theP.InternalLastShape(), ShapeType.Wire);
        if (exp.More())
        {
            theLShape = exp.Current();
            NCollection_List<Shape> thelist1 = new();
            theMap.Bind(theLShape, thelist1);
            for (exp.Init(theP.InternalLastShape(), ShapeType.Face); exp.More(); exp.Next())
            {
                theMap[theLShape].Append(exp.Current());
            }
        }

        for (exp.Init(theB, ShapeType.Edge); exp.More(); exp.Next())
        {
            if (!theMap.IsBound(exp.Current()))
            {
                theMap.Bind(exp.Current(), new NCollection_List<Shape>(theP.InternalShapes(exp.Current())));
            }
        }
    }

    private static bool ToFuse(in Face F1, in Face F2)
    {
        if (F1.IsNull || F2.IsNull)
        {
            return false;
        }

        Surface S1, S2;
        double tollin = global::Kerf.Engine.Foundation.Precision.Confusion();
        double tolang = global::Kerf.Engine.Foundation.Precision.Angular();

        S1 = Tool.Surface(F1, out _)!;
        S2 = Tool.Surface(F2, out _)!;

        if (S1.GetType() == typeof(TrimmedSurface))
        {
            S1 = ((TrimmedSurface)S1).BasisSurface();
        }

        if (S2.GetType() == typeof(TrimmedSurface))
        {
            S2 = ((TrimmedSurface)S2).BasisSurface();
        }

        if (S1.GetType() != S2.GetType())
        {
            return false;
        }

        bool ValRet = false;
        if (S1.GetType() == typeof(PlaneSurface))
        {
            S1 = Tool.Surface(F1)!; // to apply the location.
            S2 = Tool.Surface(F2)!;
            // OCCT: a trimmed plane gives a null plane here (an access violation natively)
            Plane3D pl1 = (S1 as PlaneSurface)!.Pln();
            Plane3D pl2 = (S2 as PlaneSurface)!.Pln();

            if (pl1.Position().IsCoplanar(pl2.Position(), tollin, tolang))
            {
                ValRet = true;
            }
        }

        return ValRet;
    }
}
