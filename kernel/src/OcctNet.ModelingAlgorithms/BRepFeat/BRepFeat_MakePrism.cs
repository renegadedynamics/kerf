// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakePrism.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakePrism.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakePrism.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepFeat_PerfSelection;
using static Occt.BRepFeat_StatusError;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
    private Shape myPbase = new();
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> mySlface = new(ShapeHasher.Instance);
    private Direction3D myDir = new();
    private readonly NCollection_Sequence<Curve3D> myCurves = new();
    private Curve3D? myBCurve;
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
    public BRepFeat_MakePrism(in Shape Sbase, in Shape Pbase, in Face Skface, in Direction3D Direction,
                              int Fuse, bool Modify)
    {
        Init(Sbase, Pbase, Skface, Direction, Fuse, Modify);
    }

    /// <summary>Initializes the prism (see the constructor).</summary>
    public void Init(in Shape Sbase, in Shape Pbase, in Face Skface, in Direction3D Direc, int Mode,
                     bool Modify)
    {
        // OCCT.NET: the working copies of frozen inputs (decision 1052)
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
            NCollection_List<Shape> thelist1 = new();
            mySlface.Bind(F, thelist1);
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
        Vector3D V = Length * new Vector3D(myDir);

        // construction of prism of height Length

        LocOpe_Prism thePrism = new(myPbase, V);
        Shape VraiPrism = thePrism.InternalShape();

        // management of descendants
        MajMap(myPbase, thePrism, myMap, ref myFShape, ref myLShape);

        myGShape = VraiPrism; // the primitive
        GeneratedShapeValid();

        Face FFace = new();

        bool found = false;

        // try to detect the faces of gluing
        // in case if the top of the prism is tangent to the initial shape

        if (!mySkface.IsNull || !mySlface.IsEmpty())
        {
            if (myLShape.ShapeType() == ShapeType.Wire)
            {
                Explorer ex1 = new(VraiPrism, ShapeType.Face);
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

            Explorer exp = new(mySbase, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                Face ff = ShapeCasts.Face(exp.Current());
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
    /// <exception cref="ConstructionErrorException">Until is null or has no face.</exception>
    public void Perform(in Shape Until)
    {
        if (Until.IsNull)
        {
            throw new ConstructionErrorException();
        }
        Explorer exp = new(Until, ShapeType.Face);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }
        myGluedF.Clear();
        myPerfSelection = BRepFeat_SelectionU;
        PerfSelectionValid();
        mySFrom.Nullify();
        ShapeFromValid();
        mySUntil = Working(Until);
        bool Trf = TransformShapeFU(1);
        ShapeUntilValid();
        Curve3D C = TestCurve(myPbase, new Vector3D(myDir));
        int sens = SensOfPrism(C, mySUntil);
        double Height = HeightMax(mySbase, mySkface, mySFrom, mySUntil);
        Vector3D V = 2 * sens * Height * new Vector3D(myDir);

        // construction of long prism
        LocOpe_Prism thePrism = new(myPbase, V);
        Shape VraiPrism = thePrism.InternalShape();

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
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(C);

            // direction of the prism depending on Until

            LocOpe_CSIntersector ASI = new(mySUntil);
            ASI.Perform(scur);
            Orientation Or;
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
                    Or = global::Kerf.Engine.Topology.TopAbs.Reverse(Or);
                }
                Face FUntil = ASI.Point(1, 1).Face();
                Compound Comp = new();
                BrepBuilder B = new();
                B.MakeCompound(ref Comp);
                Solid S = BRepFeat.Tool(mySUntil, FUntil, Or);
                if (!S.IsNull)
                {
                    B.Add(Comp, S);
                }
                BRepAlgoAPI_Cut trP = new(VraiPrism, Comp);
                UpdateDescendants(trP, trP.InternalShape(), false);
                //
                Explorer ex = new(trP.InternalShape(), ShapeType.Solid);
                Shape Cutsh = ex.Current();
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

        // length depending on bounding boxes

        double Height = HeightMax(mySbase, mySkface, mySFrom, mySUntil);
        Curve3D C = TestCurve(myPbase, new Vector3D(myDir));
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
            Vector3D Vtra = -3 * Height * sens / 2.0 * new Vector3D(myDir);
            thePrism.Perform(myPbase, 3 * sens * Height * new Vector3D(myDir), Vtra);
        }
        else
        {
            thePrism.Perform(myPbase, 2 * sens * Height * new Vector3D(myDir));
        }
        Shape VraiPrism = thePrism.InternalShape();

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
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(C);
            LocOpe_CSIntersector ASI1 = new(mySUntil);
            LocOpe_CSIntersector ASI2 = new(mySFrom);
            ASI1.Perform(scur);
            ASI2.Perform(scur);
            Orientation OrU, OrF;
            Face FFrom, FUntil;
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
                    OrU = global::Kerf.Engine.Topology.TopAbs.Reverse(OrU);
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
                    OrF = global::Kerf.Engine.Topology.TopAbs.Reverse(OrF);
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
                Orientation Or;
                Or = OrU;
                OrU = OrF;
                OrF = Or;
            }
            //
            NCollection_List<Shape> aLTools = new();
            Solid S = BRepFeat.Tool(mySUntil, FUntil, OrU);
            if (!S.IsNull)
            {
                aLTools.Append(S);
            }
            else
            {
                NotDone();
                myStatusError = BRepFeat_NullToolU;
                return;
            }
            Solid SS = BRepFeat.Tool(mySFrom, FFrom, OrF);
            if (!SS.IsNull)
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
            NCollection_List<Shape> aLObj = new();
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
        Vector3D V = 2 * Height * new Vector3D(myDir);

        LocOpe_Prism thePrism = new(myPbase, V);
        Shape VraiPrism = thePrism.InternalShape();

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
    /// <exception cref="ConstructionErrorException">Until is null or has no face.</exception>
    public void PerformFromEnd(in Shape Until)
    {
        if (Until.IsNull)
        {
            throw new ConstructionErrorException();
        }
        if (!mySkface.IsNull && Working(Until).IsSame(mySkface))
        {
            myDir.Reverse();
            PerformUntilEnd();
            return;
        }
        Explorer exp = new();
        exp.Init(Until, ShapeType.Face);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }
        myPerfSelection = BRepFeat_SelectionShU;
        PerfSelectionValid();
        mySFrom.Nullify();
        ShapeFromValid();
        mySUntil = Working(Until);
        bool Trf = TransformShapeFU(1);
        ShapeUntilValid();
        Curve3D C = TestCurve(myPbase, new Vector3D(myDir));
        int sens = SensOfPrism(C, mySUntil);
        double Height = HeightMax(mySbase, mySkface, mySFrom, mySUntil);
        Vector3D Vtra = -3 * Height * sens / 2.0 * new Vector3D(myDir);
        Vector3D Vect = 3 * sens * Height * new Vector3D(myDir);
        LocOpe_Prism thePrism = new(myPbase, Vect, Vtra);
        Shape VraiPrism = thePrism.InternalShape();

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
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(C);
            LocOpe_CSIntersector ASI1 = new(mySUntil);
            LocOpe_CSIntersector ASI2 = new(mySbase);
            ASI1.Perform(scur);
            ASI2.Perform(scur);
            Orientation OrU = Forward, OrF = Forward;
            Face FUntil = new(), FFrom = new();
            if (ASI1.IsDone() && ASI1.NbPoints(1) >= 1)
            {
                OrU = ASI1.Point(1, 1).Orientation();
                if (sens == -1)
                {
                    OrU = global::Kerf.Engine.Topology.TopAbs.Reverse(OrU);
                }
                FUntil = ASI1.Point(1, 1).Face();
            }
            if (ASI2.IsDone() && ASI2.NbPoints(1) >= 1)
            {
                OrF = ASI2.Point(1, 1).Orientation();
                FFrom = ASI2.Point(1, 1).Face();
                Surface S = Tool.Surface(FFrom)!;
                if (S.GetType() == typeof(TrimmedSurface))
                {
                    S = ((TrimmedSurface)S).BasisSurface();
                }
                BRepLib_MakeFace fac = new(S, global::Kerf.Engine.Foundation.Precision.Confusion());
                mySFrom = fac.InternalFace();
                Trf = TransformShapeFU(0);
                FFrom = ShapeCasts.Face(mySFrom);
            }

            NCollection_List<Shape> aLTools = new();
            Solid Sol = BRepFeat.Tool(mySUntil, FUntil, OrU);
            if (!Sol.IsNull)
            {
                aLTools.Append(Sol);
            }
            else
            {
                NotDone();
                myStatusError = BRepFeat_NullToolU;
                return;
            }

            Solid Sol1 = BRepFeat.Tool(mySFrom, FFrom, OrF);
            if (!Sol1.IsNull)
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
            NCollection_List<Shape> aLObj = new();
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
        Vector3D V = 3 * Height * new Vector3D(myDir);
        Vector3D Vtra = -3 * Height / 2.0 * new Vector3D(myDir);
        LocOpe_Prism thePrism = new(myPbase, V, Vtra);
        Shape VraiPrism = thePrism.InternalShape();
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
    /// <exception cref="ConstructionErrorException">Until has no face (a null Until included, see below).</exception>
    public void PerformUntilHeight(in Shape Until, double Length)
    {
        // OCCT: neither of the two first calls returns: a null Until goes on and throws below, a zero length goes on
        // and builds the prism again with the length 0
        if (Until.IsNull)
        {
            Perform(Length);
        }
        if (Length == 0)
        {
            Perform(Until);
        }
        Explorer exp = new(Until, ShapeType.Face);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }
        myGluedF.Clear();
        myPerfSelection = BRepFeat_NoSelection;
        PerfSelectionValid();
        mySFrom.Nullify();
        ShapeFromValid();
        mySUntil = Working(Until);
        bool Trf = TransformShapeFU(1);
        ShapeUntilValid();
        Curve3D C = TestCurve(myPbase, new Vector3D(myDir));
        int sens = SensOfPrism(C, mySUntil);
        Vector3D V = sens * Length * new Vector3D(myDir);
        LocOpe_Prism thePrism = new(myPbase, V);
        Shape VraiPrism = thePrism.InternalShape();

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
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(C);
            LocOpe_CSIntersector ASI = new(mySUntil);
            ASI.Perform(scur);
            Orientation Or;
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
                    Or = global::Kerf.Engine.Topology.TopAbs.Reverse(Or);
                }
                Face FUntil = ASI.Point(1, 1).Face();
                Compound Comp = new();
                BrepBuilder B = new();
                B.MakeCompound(ref Comp);
                Solid S = BRepFeat.Tool(mySUntil, FUntil, Or);
                if (!S.IsNull)
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
    public override void Curves(NCollection_Sequence<Curve3D> scur)
    {
        scur.Assign(myCurves);
    }

    /// <summary>The curve parallel to the axis of the prism through the centre of masses.</summary>
    public override Curve3D? BarycCurve()
    {
        return myBCurve;
    }

    // Calculate the height of the prism following the parameters of bounding box
    private static double HeightMax(in Shape theSbase, in Face theSkface, in Shape theSFrom,
                                    in Shape theSUntil)
    {
        Box Box = new();
        BRepBndLib.Add(theSbase, ref Box);
        BRepBndLib.Add(theSkface, ref Box);
        if (!theSFrom.IsNull)
        {
            bool FacRevolInfini = false;
            Explorer exp = new();
            exp.Init(theSFrom, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Explorer exp1 = new();
                exp1.Init(exp.Current(), ShapeType.Vertex);
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
        if (!theSUntil.IsNull)
        {
            bool FacRevolInfini = false;
            Explorer exp = new();
            exp.Init(theSUntil, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Explorer exp1 = new();
                exp1.Init(exp.Current(), ShapeType.Vertex);
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
    private static int SensOfPrism(Curve3D C, in Shape Until)
    {
        LocOpe_CSIntersector ASI1 = new(Until);
        NCollection_Sequence<Curve3D?> scur = new();
        scur.Append(C);
        ASI1.Perform(scur);
        int sens = 1;
        if (ASI1.IsDone() && ASI1.NbPoints(1) >= 1)
        {
            if (ASI1.Point(1, 1).Parameter() + global::Kerf.Engine.Foundation.Precision.Confusion() < 0.0
                && ASI1.Point(1, ASI1.NbPoints(1)).Parameter() + global::Kerf.Engine.Foundation.Precision.Confusion() < 0.0)
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

    private static void MajMap(in Shape theB, LocOpe_Prism theP,
                               NCollection_DataMap<Shape, NCollection_List<Shape>> theMap,
                               ref Shape theFShape, ref Shape theLShape)
    {
        Explorer exp = new(theP.InternalFirstShape(), ShapeType.Wire);
        if (exp.More())
        {
            theFShape = exp.Current();
            NCollection_List<Shape> thelist2 = new();
            theMap.Bind(theFShape, thelist2);
            for (exp.Init(theP.InternalFirstShape(), ShapeType.Face); exp.More(); exp.Next())
            {
                theMap[theFShape].Append(exp.Current());
            }
        }

        exp.Init(theP.InternalLastShape(), ShapeType.Wire);
        if (exp.More())
        {
            theLShape = exp.Current();
            NCollection_List<Shape> thelist3 = new();
            theMap.Bind(theLShape, thelist3);
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

    private static Curve3D TestCurve(in Shape Base, in Vector3D V)
    {
        Point3D bar = new(0.0, 0.0, 0.0);
        NCollection_Sequence<Point3D> spt = new();
        LocOpe.SampleEdges(Base, spt);
        for (int jj = 1; jj <= spt.Length(); jj++)
        {
            Point3D pvt = spt[jj];
            bar.ChangeCoord() += pvt.XYZ();
        }
        bar.ChangeCoord().Divide(spt.Length());
        Axis3D newAx = new(bar, new Direction3D(V));
        LineCurve3D theLin = new(newAx);
        return theLin;
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

        S1 = Tool.Surface(F1, out Placement loc1)!;
        S2 = Tool.Surface(F2, out Placement loc2)!;

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
            Plane3D pl1 = ((PlaneSurface)S1).Pln();
            Plane3D pl2 = ((PlaneSurface)S2).Pln();

            // apply locations
            if (!loc1.IsIdentity)
            {
                pl1.Transform(loc1.Transformation());
            }
            if (!loc2.IsIdentity)
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
