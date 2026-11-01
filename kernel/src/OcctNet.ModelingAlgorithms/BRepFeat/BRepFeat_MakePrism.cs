// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakePrism.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakePrism.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakePrism.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepFeat_PerfSelection;
using static Occt.BRepFeat_StatusError;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Describes functions to build prism features: a depression (a cut) or a protrusion (a fusion) along a direction, from
/// a basis face of a sketch face.
/// </summary>
/// <remarks>
/// <para>The mode is 0 for a depression, 1 for a protrusion, 2 for the feature alone (a protrusion not fused). The
/// prism goes for a length, up to a face, from a face up to another, through all, up to the end of the shape, from the
/// end up to a face, or up to a face at a height.</para>
/// <para>The basis shape and the profile are read, never written: the prism sweeps a copy of a frozen profile
/// (LocOpe_Prism, decision 1014).</para>
/// </remarks>
public sealed class BRepFeat_MakePrism : BRepFeat_Form
{
    private TopoDS_Shape myPbase = new();
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mySlface = new(TopTools_ShapeMapHasher.Instance);
    private gp_Dir myDir = new();
    private readonly NCollection_Sequence<Geom_Curve> myCurves = new();
    private Geom_Curve? myBCurve;
    // OCCT: the class declares its own myStatusError, hiding BRepFeat_Form's; CurrentStatusError reads the form's
#pragma warning disable CS0414 // written, never read: as in OCCT
    private BRepFeat_StatusError myStatusError;
#pragma warning restore CS0414

    /// <summary>Builds a prism by projecting a wire along the face of a shape.</summary>
    public BRepFeat_MakePrism()
    {
        myStatusError = BRepFeat_OK;
    }

    /// <summary>
    /// Builds a prism of the profile <paramref name="Pbase"/> on the basis shape <paramref name="Sbase"/>, along
    /// <paramref name="Direction"/>, from the sketch face <paramref name="Skface"/>; <paramref name="Fuse"/> is the mode
    /// (0 cut, 1 fuse, 2 the feature alone).
    /// </summary>
    public BRepFeat_MakePrism(in TopoDS_Shape Sbase, in TopoDS_Shape Pbase, in TopoDS_Face Skface, in gp_Dir Direction,
                              int Fuse, bool Modify)
    {
        Init(Sbase, Pbase, Skface, Direction, Fuse, Modify);
    }

    /// <summary>Initializes the prism (see the constructor).</summary>
    public void Init(in TopoDS_Shape Sbase, in TopoDS_Shape Pbase, in TopoDS_Face Skface, in gp_Dir Direc, int Mode,
                     bool Modify)
    {
        // OCCT.NET: the working copies of frozen inputs (decision 9501)
        StartWorkingShapes();
        mySkface = Working(Skface);
        SketchFaceValid();
        mySbase = Working(Sbase);
        BasisShapeValid();
        myPbase = Working(Pbase);
        mySlface.Clear();
        myDir = Direc;
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
        myNewEdges.Clear();
        myTgtEdges.Clear();
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
            NCollection_List<TopoDS_Shape> thelist1 = new();
            mySlface.Bind(F, thelist1);
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

    /// <summary>Builds a prism of length <paramref name="Length"/>, and the topological reconstruction.</summary>
    public void Perform(double Length)
    {
        mySFrom.Nullify();
        ShapeFromValid();
        mySUntil.Nullify();
        ShapeUntilValid();
        myGluedF.Clear();
        myPerfSelection = BRepFeat_NoSelection;
        PerfSelectionValid();
        gp_Vec V = Length * new gp_Vec(myDir);

        // construction of prism of height Length

        LocOpe_Prism thePrism = new(myPbase, V);
        TopoDS_Shape VraiPrism = thePrism.InternalShape();

        // management of descendants
        MajMap(myPbase, thePrism, myMap, ref myFShape, ref myLShape);

        myGShape = VraiPrism; // the primitive
        GeneratedShapeValid();

        TopoDS_Face FFace = new();

        bool found = false;

        // try to detect the faces of gluing
        // in case if the top of the prism is tangent to the initial shape

        if (!mySkface.IsNull() || !mySlface.IsEmpty())
        {
            if (myLShape.ShapeType() == TopAbs_WIRE)
            {
                TopExp_Explorer ex1 = new(VraiPrism, TopAbs_FACE);
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

            TopExp_Explorer exp = new(mySbase, TopAbs_FACE);
            for (; exp.More(); exp.Next())
            {
                TopoDS_Face ff = TopoDS.Face(exp.Current());
                if (ToFuse(ff, FFace))
                {
                    if (!FFace.IsSame(myPbase) && BRepFeat.IsInside(ff, FFace))
                    {
                        break;
                    }
                }
            }
        }

        // management of faces of gluing given by the user

        GluedFacesValid();

        if (!myGluedF.IsEmpty())
        { // case gluing
            myJustGluer = true;
            thePrism.Curves(myCurves);
            myBCurve = thePrism.BarycCurve();
            GlobalPerform(); // topological reconstruction
        }

        // if there is no gluing  -> call of ope topo
        if (!myJustGluer)
        {
            if (myFuse && !myJustFeat)
            {
                BRepAlgoAPI_Fuse f = new(mySbase, myGShape);
                myShape = f.InternalShape();
                UpdateDescendants(f, myShape, false);
                Done();
            }
            else if (!myFuse)
            {
                BRepAlgoAPI_Cut c = new(mySbase, myGShape);
                myShape = c.InternalShape();
                UpdateDescendants(c, myShape, false);
                Done();
            }
            else
            {
                myShape = myGShape;
                Done();
            }
        }
    }

    /// <summary>Builds a prism up to the shape <paramref name="Until"/>, and the topological reconstruction.</summary>
    /// <exception cref="Standard_ConstructionError">Until is null or has no face.</exception>
    public void Perform(in TopoDS_Shape Until)
    {
        if (Until.IsNull())
        {
            throw new Standard_ConstructionError();
        }
        TopExp_Explorer exp = new(Until, TopAbs_FACE);
        if (!exp.More())
        {
            throw new Standard_ConstructionError();
        }
        myGluedF.Clear();
        myPerfSelection = BRepFeat_SelectionU;
        PerfSelectionValid();
        mySFrom.Nullify();
        ShapeFromValid();
        mySUntil = Working(Until);
        bool Trf = TransformShapeFU(1);
        ShapeUntilValid();
        Geom_Curve C = TestCurve(myPbase, new gp_Vec(myDir));
        int sens = SensOfPrism(C, mySUntil);
        double Height = HeightMax(mySbase, mySkface, mySFrom, mySUntil);
        gp_Vec V = 2 * sens * Height * new gp_Vec(myDir);

        // construction of long prism
        LocOpe_Prism thePrism = new(myPbase, V);
        TopoDS_Shape VraiPrism = thePrism.InternalShape();

        // in case of support of face Until
        if (!Trf)
        {
            MajMap(myPbase, thePrism, myMap, ref myFShape, ref myLShape);
            myGShape = VraiPrism;
            GeneratedShapeValid();
            GluedFacesValid();
            thePrism.Curves(myCurves);
            myBCurve = thePrism.BarycCurve();
            GlobalPerform();
        }
        else
        { // until support -> passage to topological operations
            MajMap(myPbase, thePrism, myMap, ref myFShape, ref myLShape);
            NCollection_Sequence<Geom_Curve?> scur = new();
            scur.Clear();
            scur.Append(C);

            // direction of the prism depending on Until

            LocOpe_CSIntersector ASI = new(mySUntil);
            ASI.Perform(scur);
            TopAbs_Orientation Or;
            if (ASI.IsDone() && ASI.NbPoints(1) >= 1)
            {
                if (myFuse)
                {
                    Or = ASI.Point(1, 1).Orientation();
                }
                else
                {
                    Or = ASI.Point(1, ASI.NbPoints(1)).Orientation();
                }
                if (sens == -1)
                {
                    Or = TopAbs.Reverse(Or);
                }
                TopoDS_Face FUntil = ASI.Point(1, 1).Face();
                TopoDS_Compound Comp = new();
                BRep_Builder B = new();
                B.MakeCompound(ref Comp);
                TopoDS_Solid S = BRepFeat.Tool(mySUntil, FUntil, Or);
                if (!S.IsNull())
                {
                    B.Add(Comp, S);
                }
                BRepAlgoAPI_Cut trP = new(VraiPrism, Comp);
                UpdateDescendants(trP, trP.InternalShape(), false);
                //
                TopExp_Explorer ex = new(trP.InternalShape(), TopAbs_SOLID);
                TopoDS_Shape Cutsh = ex.Current();
                if (myFuse && !myJustFeat)
                {
                    BRepAlgoAPI_Fuse f = new(mySbase, Cutsh);
                    myShape = f.InternalShape();
                    UpdateDescendants(f, myShape, false);
                    Done();
                }
                else if (!myFuse)
                {
                    BRepAlgoAPI_Cut c = new(mySbase, Cutsh);
                    myShape = c.InternalShape();
                    UpdateDescendants(c, myShape, false);
                    Done();
                }
                else
                {
                    myShape = Cutsh;
                    Done();
                }
            }
        }
    }

    /// <summary>Builds a prism from the shape <paramref name="From"/> up to the shape <paramref name="Until"/>.</summary>
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

        // length depending on bounding boxes

        double Height = HeightMax(mySbase, mySkface, mySFrom, mySUntil);
        Geom_Curve C = TestCurve(myPbase, new gp_Vec(myDir));
        int sens; // direction of prism
        int tran; // transfer of prism
        if (From.IsSame(Until))
        {
            sens = 1;
            tran = -1;
        }
        else
        {
            sens = SensOfPrism(C, mySUntil);
            tran = sens * SensOfPrism(C, mySFrom);
        }
        LocOpe_Prism thePrism = new();
        if (tran < 0)
        {
            gp_Vec Vtra = -3 * Height * sens / 2.0 * new gp_Vec(myDir);
            thePrism.Perform(myPbase, 3 * sens * Height * new gp_Vec(myDir), Vtra);
        }
        else
        {
            thePrism.Perform(myPbase, 2 * sens * Height * new gp_Vec(myDir));
        }
        TopoDS_Shape VraiPrism = thePrism.InternalShape();

        if (!Trff)
        {
            MajMap(myPbase, thePrism, myMap, ref myFShape, ref myLShape);

            myGShape = VraiPrism;
            GeneratedShapeValid();
            GluedFacesValid();
            thePrism.Curves(myCurves);
            myBCurve = thePrism.BarycCurve();
            GlobalPerform();
        }
        else
        { // case until support -> topological operation
            MajMap(myPbase, thePrism, myMap, ref myFShape, ref myLShape);
            NCollection_Sequence<Geom_Curve?> scur = new();
            scur.Clear();
            scur.Append(C);
            LocOpe_CSIntersector ASI1 = new(mySUntil);
            LocOpe_CSIntersector ASI2 = new(mySFrom);
            ASI1.Perform(scur);
            ASI2.Perform(scur);
            TopAbs_Orientation OrU, OrF;
            TopoDS_Face FFrom, FUntil;
            double ParF, ParU;
            if (ASI1.IsDone() && ASI1.NbPoints(1) >= 1)
            {
                if (myFuse)
                {
                    OrU = ASI1.Point(1, 1).Orientation();
                }
                else
                {
                    OrU = ASI1.Point(1, ASI1.NbPoints(1)).Orientation();
                }
                if (sens == -1)
                {
                    OrU = TopAbs.Reverse(OrU);
                }
                FUntil = ASI1.Point(1, 1).Face();
                ParU = ASI1.Point(1, 1).Parameter();
            }
            else
            {
                NotDone();
                myStatusError = BRepFeat_NoIntersectU;
                return;
            }
            if (ASI2.IsDone() && ASI2.NbPoints(1) >= 1)
            {
                OrF = ASI2.Point(1, 1).Orientation();
                if (sens == 1)
                {
                    OrF = TopAbs.Reverse(OrF);
                }
                FFrom = ASI2.Point(1, 1).Face();
                ParF = ASI2.Point(1, 1).Parameter();
            }
            else
            {
                NotDone();
                myStatusError = BRepFeat_NoIntersectF;
                return;
            }
            if (tran > 0 && (Math.Abs(ParU) < Math.Abs(ParF)))
            {
                TopAbs_Orientation Or;
                Or = OrU;
                OrU = OrF;
                OrF = Or;
            }
            //
            NCollection_List<TopoDS_Shape> aLTools = new();
            TopoDS_Solid S = BRepFeat.Tool(mySUntil, FUntil, OrU);
            if (!S.IsNull())
            {
                aLTools.Append(S);
            }
            else
            {
                NotDone();
                myStatusError = BRepFeat_NullToolU;
                return;
            }
            TopoDS_Solid SS = BRepFeat.Tool(mySFrom, FFrom, OrF);
            if (!SS.IsNull())
            {
                aLTools.Append(SS);
            }
            else
            {
                NotDone();
                myStatusError = BRepFeat_NullToolF;
                return;
            }
            //
            NCollection_List<TopoDS_Shape> aLObj = new();
            aLObj.Append(VraiPrism);
            //
            BRepAlgoAPI_Cut trP = new();
            trP.SetArguments(aLObj);
            trP.SetTools(aLTools);
            trP.Build();
            UpdateDescendants(trP, trP.InternalShape(), false);
            if (myFuse && !myJustFeat)
            {
                BRepAlgoAPI_Fuse f = new(mySbase, trP.InternalShape());
                myShape = f.InternalShape();
                UpdateDescendants(f, myShape, false);
                Done();
            }
            else if (!myFuse)
            {
                BRepAlgoAPI_Cut c = new(mySbase, trP.InternalShape());
                myShape = c.InternalShape();
                UpdateDescendants(c, myShape, false);
                Done();
            }
            else
            {
                myShape = trP.InternalShape();
                Done();
            }
        }
    }

    /// <summary>Builds a prism up to the end of the basis shape.</summary>
    public void PerformUntilEnd()
    {
        myPerfSelection = BRepFeat_SelectionSh;
        PerfSelectionValid();
        myGluedF.Clear();
        mySUntil.Nullify();
        ShapeUntilValid();
        mySFrom.Nullify();
        ShapeFromValid();
        double Height = HeightMax(mySbase, mySkface, mySFrom, mySUntil);
        gp_Vec V = 2 * Height * new gp_Vec(myDir);

        LocOpe_Prism thePrism = new(myPbase, V);
        TopoDS_Shape VraiPrism = thePrism.InternalShape();

        MajMap(myPbase, thePrism, myMap, ref myFShape, ref myLShape);

        myGShape = VraiPrism;
        GeneratedShapeValid();
        GluedFacesValid();

        if (!myFuse)
        {
            BRepAlgoAPI_Cut c = new(mySbase, myGShape);
            if (c.IsDone())
            {
                myShape = c.InternalShape();
                UpdateDescendants(c, myShape, false);
                Done();
            }
        }
        else
        {
            thePrism.Curves(myCurves);
            myBCurve = thePrism.BarycCurve();
            GlobalPerform();
        }
    }

    /// <summary>Builds a prism from the end of the basis shape up to the shape <paramref name="Until"/>.</summary>
    /// <exception cref="Standard_ConstructionError">Until is null or has no face.</exception>
    public void PerformFromEnd(in TopoDS_Shape Until)
    {
        if (Until.IsNull())
        {
            throw new Standard_ConstructionError();
        }
        if (!mySkface.IsNull() && Working(Until).IsSame(mySkface))
        {
            myDir.Reverse();
            PerformUntilEnd();
            return;
        }
        TopExp_Explorer exp = new();
        exp.Init(Until, TopAbs_FACE);
        if (!exp.More())
        {
            throw new Standard_ConstructionError();
        }
        myPerfSelection = BRepFeat_SelectionShU;
        PerfSelectionValid();
        mySFrom.Nullify();
        ShapeFromValid();
        mySUntil = Working(Until);
        bool Trf = TransformShapeFU(1);
        ShapeUntilValid();
        Geom_Curve C = TestCurve(myPbase, new gp_Vec(myDir));
        int sens = SensOfPrism(C, mySUntil);
        double Height = HeightMax(mySbase, mySkface, mySFrom, mySUntil);
        gp_Vec Vtra = -3 * Height * sens / 2.0 * new gp_Vec(myDir);
        gp_Vec Vect = 3 * sens * Height * new gp_Vec(myDir);
        LocOpe_Prism thePrism = new(myPbase, Vect, Vtra);
        TopoDS_Shape VraiPrism = thePrism.InternalShape();

        if (!Trf)
        { // case face until
            MajMap(myPbase, thePrism, myMap, ref myFShape, ref myLShape);
            myGShape = VraiPrism;
            GeneratedShapeValid();
            myGluedF.Clear();
            GluedFacesValid();
            thePrism.Curves(myCurves);
            myBCurve = thePrism.BarycCurve();
            GlobalPerform();
        }
        else
        { // case support
            MajMap(myPbase, thePrism, myMap, ref myFShape, ref myLShape);
            NCollection_Sequence<Geom_Curve?> scur = new();
            scur.Clear();
            scur.Append(C);
            LocOpe_CSIntersector ASI1 = new(mySUntil);
            LocOpe_CSIntersector ASI2 = new(mySbase);
            ASI1.Perform(scur);
            ASI2.Perform(scur);
            TopAbs_Orientation OrU = TopAbs_FORWARD, OrF = TopAbs_FORWARD;
            TopoDS_Face FUntil = new(), FFrom = new();
            if (ASI1.IsDone() && ASI1.NbPoints(1) >= 1)
            {
                OrU = ASI1.Point(1, 1).Orientation();
                if (sens == -1)
                {
                    OrU = TopAbs.Reverse(OrU);
                }
                FUntil = ASI1.Point(1, 1).Face();
            }
            if (ASI2.IsDone() && ASI2.NbPoints(1) >= 1)
            {
                OrF = ASI2.Point(1, 1).Orientation();
                FFrom = ASI2.Point(1, 1).Face();
                Geom_Surface S = BRep_Tool.Surface(FFrom)!;
                if (S.GetType() == typeof(Geom_RectangularTrimmedSurface))
                {
                    S = ((Geom_RectangularTrimmedSurface)S).BasisSurface();
                }
                BRepLib_MakeFace fac = new(S, Precision.Confusion());
                mySFrom = fac.InternalFace();
                Trf = TransformShapeFU(0);
                FFrom = TopoDS.Face(mySFrom);
            }

            NCollection_List<TopoDS_Shape> aLTools = new();
            TopoDS_Solid Sol = BRepFeat.Tool(mySUntil, FUntil, OrU);
            if (!Sol.IsNull())
            {
                aLTools.Append(Sol);
            }
            else
            {
                NotDone();
                myStatusError = BRepFeat_NullToolU;
                return;
            }

            TopoDS_Solid Sol1 = BRepFeat.Tool(mySFrom, FFrom, OrF);
            if (!Sol1.IsNull())
            {
                aLTools.Append(Sol1);
            }
            else
            {
                NotDone();
                myStatusError = BRepFeat_NullToolF;
                return;
            }
            //
            NCollection_List<TopoDS_Shape> aLObj = new();
            aLObj.Append(VraiPrism);
            //
            BRepAlgoAPI_Cut trP = new();
            trP.SetArguments(aLObj);
            trP.SetTools(aLTools);
            trP.Build();
            //
            UpdateDescendants(trP, trP.InternalShape(), false);
            if (myFuse && !myJustFeat)
            {
                BRepAlgoAPI_Fuse f = new(mySbase, trP.InternalShape());
                myShape = f.InternalShape();
                UpdateDescendants(f, myShape, false);
                Done();
            }
            else if (!myFuse)
            {
                BRepAlgoAPI_Cut c = new(mySbase, trP.InternalShape());
                myShape = c.InternalShape();
                UpdateDescendants(c, myShape, false);
                Done();
            }
            else
            {
                myShape = trP.InternalShape();
                Done();
            }
        }
    }

    /// <summary>Builds a prism through all the basis shape.</summary>
    public void PerformThruAll()
    {
        mySUntil.Nullify();
        ShapeUntilValid();
        mySFrom.Nullify();
        ShapeFromValid();
        if (!myFuse)
        {
            myPerfSelection = BRepFeat_NoSelection;
        }
        else
        {
            myPerfSelection = BRepFeat_SelectionSh;
        }
        PerfSelectionValid();
        myGluedF.Clear();
        GluedFacesValid();

        double Height = HeightMax(mySbase, mySkface, mySFrom, mySUntil);
        gp_Vec V = 3 * Height * new gp_Vec(myDir);
        gp_Vec Vtra = -3 * Height / 2.0 * new gp_Vec(myDir);
        LocOpe_Prism thePrism = new(myPbase, V, Vtra);
        TopoDS_Shape VraiPrism = thePrism.InternalShape();
        MajMap(myPbase, thePrism, myMap, ref myFShape, ref myLShape);

        myGShape = VraiPrism;
        GeneratedShapeValid();

        if (!myFuse)
        {
            BRepAlgoAPI_Cut c = new(mySbase, myGShape);
            if (c.IsDone())
            {
                myShape = c.InternalShape();
                UpdateDescendants(c, myShape, false);
                Done();
            }
        }
        else
        {
            thePrism.Curves(myCurves);
            myBCurve = thePrism.BarycCurve();
            GlobalPerform();
        }
    }

    /// <summary>Builds a prism up to the shape <paramref name="Until"/>, at the height <paramref name="Length"/>.</summary>
    /// <exception cref="Standard_ConstructionError">Until has no face (a null Until included, see below).</exception>
    public void PerformUntilHeight(in TopoDS_Shape Until, double Length)
    {
        // OCCT: neither of the two first calls returns: a null Until goes on and throws below, a zero length goes on
        // and builds the prism again with the length 0
        if (Until.IsNull())
        {
            Perform(Length);
        }
        if (Length == 0)
        {
            Perform(Until);
        }
        TopExp_Explorer exp = new(Until, TopAbs_FACE);
        if (!exp.More())
        {
            throw new Standard_ConstructionError();
        }
        myGluedF.Clear();
        myPerfSelection = BRepFeat_NoSelection;
        PerfSelectionValid();
        mySFrom.Nullify();
        ShapeFromValid();
        mySUntil = Working(Until);
        bool Trf = TransformShapeFU(1);
        ShapeUntilValid();
        Geom_Curve C = TestCurve(myPbase, new gp_Vec(myDir));
        int sens = SensOfPrism(C, mySUntil);
        gp_Vec V = sens * Length * new gp_Vec(myDir);
        LocOpe_Prism thePrism = new(myPbase, V);
        TopoDS_Shape VraiPrism = thePrism.InternalShape();

        if (!Trf)
        {
            MajMap(myPbase, thePrism, myMap, ref myFShape, ref myLShape);

            myGShape = VraiPrism;
            GeneratedShapeValid();
            GluedFacesValid();
            thePrism.Curves(myCurves);
            myBCurve = thePrism.BarycCurve();
            GlobalPerform();
        }
        else
        {
            MajMap(myPbase, thePrism, myMap, ref myFShape, ref myLShape);
            NCollection_Sequence<Geom_Curve?> scur = new();
            scur.Clear();
            scur.Append(C);
            LocOpe_CSIntersector ASI = new(mySUntil);
            ASI.Perform(scur);
            TopAbs_Orientation Or;
            if (ASI.IsDone() && ASI.NbPoints(1) >= 1)
            {
                if (myFuse)
                {
                    Or = ASI.Point(1, 1).Orientation();
                }
                else
                {
                    Or = ASI.Point(1, ASI.NbPoints(1)).Orientation();
                }
                if (sens == -1)
                {
                    Or = TopAbs.Reverse(Or);
                }
                TopoDS_Face FUntil = ASI.Point(1, 1).Face();
                TopoDS_Compound Comp = new();
                BRep_Builder B = new();
                B.MakeCompound(ref Comp);
                TopoDS_Solid S = BRepFeat.Tool(mySUntil, FUntil, Or);
                if (!S.IsNull())
                {
                    B.Add(Comp, S);
                }

                BRepAlgoAPI_Cut trP = new(VraiPrism, Comp);
                UpdateDescendants(trP, trP.InternalShape(), false);
                if (myFuse && !myJustFeat)
                {
                    BRepAlgoAPI_Fuse f = new(mySbase, trP.InternalShape());
                    myShape = f.InternalShape();
                    UpdateDescendants(f, myShape, false);
                    Done();
                }
                else if (!myFuse)
                {
                    BRepAlgoAPI_Cut c = new(mySbase, trP.InternalShape());
                    myShape = c.InternalShape();
                    UpdateDescendants(c, myShape, false);
                    Done();
                }
                else
                {
                    myShape = trP.InternalShape();
                    Done();
                }
            }
        }
    }

    /// <summary>The curves parallel to the axis of the prism.</summary>
    public override void Curves(NCollection_Sequence<Geom_Curve> scur)
    {
        scur.Assign(myCurves);
    }

    /// <summary>The curve parallel to the axis of the prism through the centre of masses.</summary>
    public override Geom_Curve? BarycCurve()
    {
        return myBCurve;
    }

    // Calculate the height of the prism following the parameters of bounding box
    private static double HeightMax(in TopoDS_Shape theSbase, in TopoDS_Face theSkface, in TopoDS_Shape theSFrom,
                                    in TopoDS_Shape theSUntil)
    {
        Bnd_Box Box = new();
        BRepBndLib.Add(theSbase, ref Box);
        BRepBndLib.Add(theSkface, ref Box);
        if (!theSFrom.IsNull())
        {
            bool FacRevolInfini = false;
            TopExp_Explorer exp = new();
            exp.Init(theSFrom, TopAbs_EDGE);
            for (; exp.More(); exp.Next())
            {
                TopExp_Explorer exp1 = new();
                exp1.Init(exp.Current(), TopAbs_VERTEX);
                if (!exp1.More())
                {
                    FacRevolInfini = true;
                    break;
                }
            }
            if (!FacRevolInfini)
            {
                BRepBndLib.Add(theSFrom, ref Box);
            }
        }
        if (!theSUntil.IsNull())
        {
            bool FacRevolInfini = false;
            TopExp_Explorer exp = new();
            exp.Init(theSUntil, TopAbs_EDGE);
            for (; exp.More(); exp.Next())
            {
                TopExp_Explorer exp1 = new();
                exp1.Init(exp.Current(), TopAbs_VERTEX);
                if (!exp1.More())
                {
                    FacRevolInfini = true;
                    break;
                }
            }
            if (!FacRevolInfini)
            {
                BRepBndLib.Add(theSUntil, ref Box);
            }
        }

        double[] c = new double[6];

        Box.Get(out c[0], out c[2], out c[4], out c[1], out c[3], out c[5]);
        double parmin = c[0], parmax = c[0];
        for (int i = 0; i < 6; i++)
        {
            if (c[i] > parmax)
            {
                parmax = c[i];
            }
            if (c[i] < parmin)
            {
                parmin = c[i];
            }
        }
        double Height = Math.Abs(2.0 * (parmax - parmin));
        return Height;
    }

    // Direction of the prism depending on the shape Until
    private static int SensOfPrism(Geom_Curve C, in TopoDS_Shape Until)
    {
        LocOpe_CSIntersector ASI1 = new(Until);
        NCollection_Sequence<Geom_Curve?> scur = new();
        scur.Append(C);
        ASI1.Perform(scur);
        int sens = 1;
        if (ASI1.IsDone() && ASI1.NbPoints(1) >= 1)
        {
            if (ASI1.Point(1, 1).Parameter() + Precision.Confusion() < 0.0
                && ASI1.Point(1, ASI1.NbPoints(1)).Parameter() + Precision.Confusion() < 0.0)
            {
                sens = -1;
            }
        }
        else if (BRepFeat.ParametricBarycenter(Until, C) < 0)
        {
            sens = -1;
        }
        return sens;
    }

    private static void MajMap(in TopoDS_Shape theB, LocOpe_Prism theP,
                               NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMap,
                               ref TopoDS_Shape theFShape, ref TopoDS_Shape theLShape)
    {
        TopExp_Explorer exp = new(theP.InternalFirstShape(), TopAbs_WIRE);
        if (exp.More())
        {
            theFShape = exp.Current();
            NCollection_List<TopoDS_Shape> thelist2 = new();
            theMap.Bind(theFShape, thelist2);
            for (exp.Init(theP.InternalFirstShape(), TopAbs_FACE); exp.More(); exp.Next())
            {
                theMap[theFShape].Append(exp.Current());
            }
        }

        exp.Init(theP.InternalLastShape(), TopAbs_WIRE);
        if (exp.More())
        {
            theLShape = exp.Current();
            NCollection_List<TopoDS_Shape> thelist3 = new();
            theMap.Bind(theLShape, thelist3);
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

    private static Geom_Curve TestCurve(in TopoDS_Shape Base, in gp_Vec V)
    {
        gp_Pnt bar = new(0.0, 0.0, 0.0);
        NCollection_Sequence<gp_Pnt> spt = new();
        LocOpe.SampleEdges(Base, spt);
        for (int jj = 1; jj <= spt.Length(); jj++)
        {
            gp_Pnt pvt = spt[jj];
            bar.ChangeCoord() += pvt.XYZ();
        }
        bar.ChangeCoord().Divide(spt.Length());
        gp_Ax1 newAx = new(bar, new gp_Dir(V));
        Geom_Line theLin = new(newAx);
        return theLin;
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

        S1 = BRep_Tool.Surface(F1, out TopLoc_Location loc1)!;
        S2 = BRep_Tool.Surface(F2, out TopLoc_Location loc2)!;

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
            gp_Pln pl1 = ((Geom_Plane)S1).Pln();
            gp_Pln pl2 = ((Geom_Plane)S2).Pln();

            // apply locations
            if (!loc1.IsIdentity())
            {
                pl1.Transform(loc1.Transformation());
            }
            if (!loc2.IsIdentity())
            {
                pl2.Transform(loc2.Transformation());
            }

            if (pl1.Position().IsCoplanar(pl2.Position(), tollin, tolang))
            {
                ValRet = true;
            }
        }

        return ValRet;
    }
}
