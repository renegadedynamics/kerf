// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeDPrism.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeDPrism.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Schematiq.Collections;
using static Occt.BRepFeat_PerfSelection;
using static Occt.BRepFeat_StatusError;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Describes functions to build draft prism topologies from basis shape surfaces: a planar face swept along its normal
/// with a taper angle, as a depression (a cut) or a protrusion (a fusion).
/// </summary>
/// <remarks>
/// <para>The mode is 0 for a depression, 1 for a protrusion, 2 for the feature alone. The draft prism goes for a
/// height, up to a face, from a face up to another, through all, up to the end of the shape, from the end up to a face,
/// or up to a face at a height.</para>
/// <para>The basis shape and the profile are read, never written: the draft prism evolves a copy of a frozen profile
/// (LocOpe_DPrism, decision 1014).</para>
/// </remarks>
public sealed class BRepFeat_MakeDPrism : BRepFeat_Form
{
    private TopoDS_Face myPbase = new();
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mySlface = new(TopTools_ShapeMapHasher.Instance);
    private double myAngle;
    private readonly NCollection_Sequence<Geom_Curve> myCurves = new();
    private Geom_Curve? myBCurve;
    private NCollection_List<TopoDS_Shape> myTopEdges = new();
    private NCollection_List<TopoDS_Shape> myLatEdges = new();
    // OCCT: the class declares its own myStatusError, hiding BRepFeat_Form's; CurrentStatusError reads the form's
#pragma warning disable CS0414 // written, never read: as in OCCT
    private BRepFeat_StatusError myStatusError;
#pragma warning restore CS0414

    /// <summary>
    /// A draft prism of the planar face <paramref name="Pbase"/> on <paramref name="Sbase"/>, with the taper
    /// <paramref name="Angle"/> (radians), from the sketch face <paramref name="Skface"/>; <paramref name="Fuse"/> is
    /// the mode (0 cut, 1 fuse, 2 the feature alone).
    /// </summary>
    public BRepFeat_MakeDPrism(in TopoDS_Shape Sbase, in TopoDS_Face Pbase, in TopoDS_Face Skface, double Angle, int Fuse,
                               bool Modify)
    {
        Init(Sbase, Pbase, Skface, Angle, Fuse, Modify);
    }

    /// <summary>An empty draft prism.</summary>
    public BRepFeat_MakeDPrism()
    {
        myAngle = RealLast();
        myStatusError = BRepFeat_OK;
    }

    /// <summary>Initializes the draft prism (see the constructor).</summary>
    public void Init(in TopoDS_Shape Sbase, in TopoDS_Face Pbase, in TopoDS_Face Skface, double Angle, int Mode,
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
        myTopEdges.Clear();
        myLatEdges.Clear();
        TopExp_Explorer exp = new();
        for (exp.Init(mySbase, TopAbs_FACE); exp.More(); exp.Next())
        {
            NCollection_List<TopoDS_Shape> thelist = new();
            myMap.Bind(exp.Current(), thelist);
            myMap[exp.Current()].Append(exp.Current());
        }
        myAngle = Angle;
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

    /// <summary>Builds a draft prism of the height <paramref name="Height"/>, and the topological reconstruction.</summary>
    public void Perform(double Height)
    {
        mySFrom.Nullify();
        ShapeFromValid();
        mySUntil.Nullify();
        ShapeUntilValid();
        myGluedF.Clear();
        myPerfSelection = BRepFeat_NoSelection;
        PerfSelectionValid();

        double theheight = Height / Math.Cos(myAngle);
        LocOpe_DPrism theDPrism = new(myPbase, theheight, myAngle);
        TopoDS_Shape VraiDPrism = theDPrism.InternalShape();

        MajMap(myPbase, theDPrism, myMap, ref myFShape, ref myLShape);

        myGShape = VraiDPrism;
        GeneratedShapeValid();

        TopoDS_Shape Base = theDPrism.InternalFirstShape();
        TopExp_Explorer exp = new(Base, TopAbs_FACE);
        exp.Next();
        if (exp.More())
        {
            NotDone();
            myStatusError = BRepFeat_InvFirstShape;
            return;
        }

        // management of gluing faces

        GluedFacesValid();

        if (!myGluedF.IsEmpty())
        { // case gluing
            myJustGluer = true;
            theDPrism.Curves(myCurves);
            myBCurve = theDPrism.BarycCurve();
            GlobalPerform();
        }

        // if there is no gluing -> call topological operations
        if (!myJustGluer)
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
    }

    /// <summary>Builds a draft prism up to the shape <paramref name="Until"/>.</summary>
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
        Geom_Curve? C = TestCurve(myPbase);
        int sens = SensOfPrism(C, mySUntil);

        BRep_Builder bB = new();
        double Height = sens * HeightMax(mySbase, mySkface, mySFrom, mySUntil);
        LocOpe_DPrism theDPrism = new(myPbase, Height, myAngle);
        TopoDS_Shape VraiDPrism = theDPrism.InternalShape();
        if (!Trf)
        {
            MajMap(myPbase, theDPrism, myMap, ref myFShape, ref myLShape);
            myGShape = VraiDPrism;
            GeneratedShapeValid();
            TopoDS_Shape Base = theDPrism.InternalFirstShape();
            exp.Init(Base, TopAbs_FACE);
            exp.Next();
            if (exp.More())
            {
                NotDone();
                myStatusError = BRepFeat_InvFirstShape;
                return;
            }

            GluedFacesValid();
            theDPrism.Curves(myCurves);
            myBCurve = theDPrism.BarycCurve();
            GlobalPerform();
        }
        else
        {
            MajMap(myPbase, theDPrism, myMap, ref myFShape, ref myLShape);
            Geom_Curve? C1;
            if (sens == -1)
            {
                C1 = C!.Reversed();
            }
            else
            {
                C1 = C;
            }
            NCollection_Sequence<Geom_Curve?> scur = new();
            scur.Clear();
            scur.Append(C1);
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
                TopoDS_Face FUntil = ASI.Point(1, 1).Face();
                TopoDS_Compound Comp = new();
                bB.MakeCompound(ref Comp);
                TopoDS_Solid S = BRepFeat.Tool(mySUntil, FUntil, Or);
                if (!S.IsNull())
                {
                    bB.Add(Comp, S);
                }

                BRepAlgoAPI_Cut trP = new(VraiDPrism, Comp);
                UpdateDescendants(trP, trP.InternalShape(), false);
                TopExp_Explorer ex = new(trP.InternalShape(), TopAbs_SOLID);
                TopoDS_Shape Cutsh = ex.Current();
                if (myFuse)
                {
                    BRepAlgoAPI_Fuse f = new(mySbase, Cutsh);
                    myShape = f.InternalShape();
                    UpdateDescendants(f, myShape, false);
                    Done();
                }
                else
                {
                    BRepAlgoAPI_Cut c = new(mySbase, Cutsh);
                    myShape = c.InternalShape();
                    UpdateDescendants(c, myShape, false);
                    Done();
                }
            }
        }
        // The new edges are the gluing's own edges of the result (unfrozen), written in place as in OCCT
        NCollection_List<TopoDS_Shape>.Iterator ited = new(myNewEdges);
        for (; ited.More(); ited.Next())
        {
            TopoDS_Edge ledg = TopoDS.Edge(ited.Value());
            if (!BRepAlgo.IsValid(ledg))
            {
                bB.SameRange(ledg, false);
                bB.SameParameter(ledg, false);
                BRepLib.SameParameter(ledg, BRep_Tool.Tolerance(ledg));
            }
        }
    }

    /// <summary>Builds a draft prism from the shape <paramref name="From"/> up to the shape <paramref name="Until"/>.</summary>
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
        Geom_Curve? C = TestCurve(myPbase);
        int sens;
        if (From.IsSame(Until))
        {
            sens = 1;
        }
        else
        {
            sens = SensOfPrism(C, mySUntil);
        }

        double Height = sens * HeightMax(mySbase, myPbase, mySFrom, mySUntil);
        LocOpe_DPrism theDPrism = new(myPbase, Height, Height, myAngle);
        TopoDS_Shape VraiDPrism = theDPrism.InternalShape();
        if (!Trff)
        {
            MajMap(myPbase, theDPrism, myMap, ref myFShape, ref myLShape);

            // Make systematically dprism
            myGShape = VraiDPrism;
            GeneratedShapeValid();

            // management of gluing faces
            GluedFacesValid();
            theDPrism.Curves(myCurves);
            myBCurve = theDPrism.BarycCurve();
            // topologic reconstruction
            GlobalPerform();
        }
        else
        {
            // management of descendants
            MajMap(myPbase, theDPrism, myMap, ref myFShape, ref myLShape);
            Geom_Curve? C1;
            if (sens == -1)
            {
                C1 = C!.Reversed();
            }
            else
            {
                C1 = C;
            }
            NCollection_Sequence<Geom_Curve?> scur = new();
            scur.Clear();
            scur.Append(C1);
            LocOpe_CSIntersector ASI1 = new(mySUntil);
            LocOpe_CSIntersector ASI2 = new(mySFrom);
            ASI1.Perform(scur);
            ASI2.Perform(scur);
            TopAbs_Orientation OrU, OrF;
            TopoDS_Face FFrom, FUntil;
            // direction of dprism
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
                FUntil = ASI1.Point(1, 1).Face();
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
                OrF = TopAbs.Reverse(OrF);
                FFrom = ASI2.Point(1, 1).Face();
            }
            else
            {
                NotDone();
                myStatusError = BRepFeat_NoIntersectF;
                return;
            }
            TopoDS_Compound Comp = new();
            BRep_Builder B = new();
            B.MakeCompound(ref Comp);
            TopoDS_Solid S = BRepFeat.Tool(mySUntil, FUntil, OrU);
            if (!S.IsNull())
            {
                B.Add(Comp, S);
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
                B.Add(Comp, SS);
            }
            else
            {
                NotDone();
                myStatusError = BRepFeat_NullToolF;
                return;
            }

            BRepAlgoAPI_Cut trP = new(VraiDPrism, Comp);
            if (myFuse)
            {
                BRepAlgoAPI_Fuse f = new(mySbase, trP.InternalShape());
                myShape = f.InternalShape();
                UpdateDescendants(f, myShape, false);
                Done();
            }
            else
            {
                BRepAlgoAPI_Cut c = new(mySbase, trP.InternalShape());
                myShape = c.InternalShape();
                UpdateDescendants(c, myShape, false);
                Done();
            }
        }
    }

    /// <summary>Builds a draft prism up to the end of the basis shape.</summary>
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
        LocOpe_DPrism theDPrism = new(myPbase, Height, myAngle);
        TopoDS_Shape VraiDPrism = theDPrism.InternalShape();

        MajMap(myPbase, theDPrism, myMap, ref myFShape, ref myLShape);
        myGShape = VraiDPrism;
        GeneratedShapeValid();
        GluedFacesValid();

        theDPrism.Curves(myCurves);
        myBCurve = theDPrism.BarycCurve();
        GlobalPerform();
    }

    /// <summary>Builds a draft prism from the end of the basis shape up to the shape <paramref name="Until"/>.</summary>
    /// <exception cref="Standard_ConstructionError">Until is null or has no face.</exception>
    public void PerformFromEnd(in TopoDS_Shape Until)
    {
        if (Until.IsNull())
        {
            throw new Standard_ConstructionError();
        }
        if (!mySkface.IsNull() && Working(Until).IsSame(mySkface))
        {
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
        Geom_Curve? C = TestCurve(myPbase);
        int sens = SensOfPrism(C, mySUntil);
        double Height = sens * HeightMax(mySbase, mySkface, mySFrom, mySUntil);
        LocOpe_DPrism theDPrism = new(myPbase, Height, Height, myAngle);
        TopoDS_Shape VraiDPrism = theDPrism.InternalShape();
        if (VraiDPrism.IsNull())
        {
            NotDone();
            myStatusError = BRepFeat_NullRealTool;
            return;
        }

        if (!Trf)
        { // case finite face
            MajMap(myPbase, theDPrism, myMap, ref myFShape, ref myLShape);
            myGShape = VraiDPrism;
            GeneratedShapeValid();
            myGluedF.Clear();
            GluedFacesValid();
            theDPrism.Curves(myCurves);
            myBCurve = theDPrism.BarycCurve();
            GlobalPerform();
        }
        else
        { // case support
            MajMap(myPbase, theDPrism, myMap, ref myFShape, ref myLShape);
            Geom_Curve? C2;
            if (sens == -1)
            {
                C2 = C!.Reversed();
            }
            else
            {
                C2 = C;
            }
            NCollection_Sequence<Geom_Curve?> scur = new();
            scur.Clear();
            scur.Append(C2);
            LocOpe_CSIntersector ASI1 = new(mySUntil);
            LocOpe_CSIntersector ASI2 = new(mySbase);
            ASI1.Perform(scur);
            ASI2.Perform(scur);
            TopAbs_Orientation OrU = TopAbs_FORWARD, OrF = TopAbs_FORWARD;
            TopoDS_Face FUntil = new(), FFrom = new();
            if (ASI1.IsDone() && ASI1.NbPoints(1) >= 1)
            {
                OrU = ASI1.Point(1, 1).Orientation();
                double prm = ASI1.Point(1, 1).Parameter();
                if (prm < 0)
                {
                    OrU = TopAbs.Reverse(OrU);
                }
                FUntil = ASI1.Point(1, 1).Face();
            }

            if (ASI2.IsDone() && ASI2.NbPoints(1) >= 1)
            {
                int jj = ASI2.NbPoints(1);
                double prm = ASI2.Point(1, 1).Parameter();
                FFrom = ASI2.Point(1, 1).Face();
                OrF = ASI2.Point(1, 1).Orientation();
                OrF = TopAbs.Reverse(OrF);
                for (int iii = 1; iii <= jj; iii++)
                {
                    if (ASI2.Point(1, iii).Parameter() < prm)
                    {
                        prm = ASI2.Point(1, iii).Parameter();
                        FFrom = ASI2.Point(1, iii).Face();
                        OrF = ASI2.Point(1, iii).Orientation();
                        OrF = TopAbs.Reverse(OrF);
                    }
                }
                Geom_Surface S = BRep_Tool.Surface(FFrom)!;
                if (S.GetType() == typeof(Geom_RectangularTrimmedSurface))
                {
                    S = ((Geom_RectangularTrimmedSurface)S).BasisSurface();
                }
                BRepLib_MakeFace fac = new(S, Precision.Confusion());
                mySFrom = fac.InternalFace();
                Trf = TransformShapeFU(0);
            }

            TopoDS_Compound Comp = new();
            BRep_Builder B = new();
            B.MakeCompound(ref Comp);
            TopoDS_Solid Sol = BRepFeat.Tool(mySUntil, FUntil, OrU);
            if (!Sol.IsNull())
            {
                B.Add(Comp, Sol);
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
                B.Add(Comp, Sol1);
            }
            else
            {
                NotDone();
                myStatusError = BRepFeat_NullToolF;
                return;
            }

            BRepAlgoAPI_Cut trP = new(VraiDPrism, Comp);
            if (myFuse)
            {
                BRepAlgoAPI_Fuse f = new(mySbase, trP.InternalShape());
                myShape = f.InternalShape();
                UpdateDescendants(f, myShape, false);
                Done();
            }
            else
            {
                BRepAlgoAPI_Cut c = new(mySbase, trP.InternalShape());
                myShape = c.InternalShape();
                UpdateDescendants(c, myShape, false);
                Done();
            }
        }
    }

    /// <summary>Builds a draft prism through all the basis shape.</summary>
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
        LocOpe_DPrism theDPrism = new(myPbase, Height, Height, myAngle);
        TopoDS_Shape VraiDPrism = theDPrism.InternalShape();
        MajMap(myPbase, theDPrism, myMap, ref myFShape, ref myLShape);

        myGShape = VraiDPrism;
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
            theDPrism.Curves(myCurves);
            myBCurve = theDPrism.BarycCurve();
            GlobalPerform();
        }
    }

    /// <summary>Builds a draft prism up to the shape <paramref name="Until"/>, at the height <paramref name="Height"/>.</summary>
    /// <exception cref="Standard_ConstructionError">Until has no face (a null Until included, see below).</exception>
    public void PerformUntilHeight(in TopoDS_Shape Until, double Height)
    {
        // OCCT: neither of the two first calls returns: a null Until goes on and throws below, a zero height goes on
        // and builds the draft prism again with the height 0
        if (Until.IsNull())
        {
            Perform(Height);
        }
        if (Height == 0)
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
        Geom_Curve? C = TestCurve(myPbase);
        int sens = SensOfPrism(C, mySUntil);
        LocOpe_DPrism theDPrism = new(myPbase, sens * Height, myAngle);
        TopoDS_Shape VraiDPrism = theDPrism.InternalShape();

        if (!Trf)
        { // case face finished
            MajMap(myPbase, theDPrism, myMap, ref myFShape, ref myLShape);
            myGShape = VraiDPrism;
            GeneratedShapeValid();
            TopoDS_Shape Base = theDPrism.InternalFirstShape();
            exp.Init(Base, TopAbs_FACE);
            exp.Next();
            if (exp.More())
            {
                NotDone();
                myStatusError = BRepFeat_InvFirstShape;
                return;
            }

            GluedFacesValid();
            theDPrism.Curves(myCurves);
            myBCurve = theDPrism.BarycCurve();
            GlobalPerform();
        }
        else
        { // case support
            MajMap(myPbase, theDPrism, myMap, ref myFShape, ref myLShape);
            Geom_Curve? C1;
            if (sens == -1)
            {
                C1 = C!.Reversed();
            }
            else
            {
                C1 = C;
            }
            NCollection_Sequence<Geom_Curve?> scur = new();
            scur.Clear();
            scur.Append(C1);
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
                TopoDS_Face FUntil = ASI.Point(1, 1).Face();
                TopoDS_Compound Comp = new();
                BRep_Builder B = new();
                B.MakeCompound(ref Comp);
                TopoDS_Solid S = BRepFeat.Tool(mySUntil, FUntil, Or);
                if (!S.IsNull())
                {
                    B.Add(Comp, S);
                }
                BRepAlgoAPI_Cut trP = new(VraiDPrism, Comp);
                if (myFuse)
                {
                    BRepAlgoAPI_Fuse f = new(mySbase, trP.InternalShape());
                    myShape = f.InternalShape();
                    UpdateDescendants(f, myShape, false);
                    Done();
                }
                else
                {
                    BRepAlgoAPI_Cut c = new(mySbase, trP.InternalShape());
                    myShape = c.InternalShape();
                    UpdateDescendants(c, myShape, false);
                    Done();
                }
            }
        }
    }

    /// <summary>The curves along the draft prism.</summary>
    public override void Curves(NCollection_Sequence<Geom_Curve> scur)
    {
        scur.Assign(myCurves);
    }

    /// <summary>
    /// Determines the top and lateral edges of the boss: <paramref name="signature"/> 1 or -1 takes the first shape of
    /// the draft prism as its top, 2 or -2 its last shape; a negative signature takes the new edges as lateral edges.
    /// </summary>
    public void BossEdges(int signature)
    {
        NCollection_List<TopoDS_Shape> theLastShape = new();
        theLastShape.Clear();
        if (signature == 1 || signature == -1)
        {
            theLastShape = new NCollection_List<TopoDS_Shape>(InternalFirstShape());
        }
        else if (signature == 2 || signature == -2)
        {
            theLastShape = new NCollection_List<TopoDS_Shape>(InternalLastShape());
        }
        else
        {
            return;
        }

        // Edges Top
        NCollection_List<TopoDS_Shape>.Iterator itLS = new();
        TopExp_Explorer ExpE = new();
        for (itLS.Initialize(theLastShape); itLS.More(); itLS.Next())
        {
            TopoDS_Face FF = TopoDS.Face(itLS.Value());
            for (ExpE.Init(FF, TopAbs_EDGE); ExpE.More(); ExpE.Next())
            {
                TopoDS_Edge EE = TopoDS.Edge(ExpE.Current());
                myTopEdges.Append(EE);
            }
        }

        // Edges Bottom
        if (signature < 0)
        {
            // Attention check if TgtEdges is important
            myLatEdges = new NCollection_List<TopoDS_Shape>(InternalNewEdges());
        }
        else if (signature > 0)
        {
            if (!myShape.IsNull())
            {
                // OCCT iterates this address-keyed map into the lateral edges: the port in the order of its additions and
                // removals (decision 1054)
                NCollection_Map<TopoDS_Shape> MapE = new(TopTools_ShapeMapHasher.Instance);
                bool Found;

                TopExp_Explorer ExpF = new();
                for (ExpF.Init(myShape, TopAbs_FACE); ExpF.More(); ExpF.Next())
                {
                    Found = false;
                    TopoDS_Face FF = TopoDS.Face(ExpF.Current());
                    for (itLS.Initialize(theLastShape); itLS.More(); itLS.Next())
                    {
                        TopoDS_Face TopFace = TopoDS.Face(itLS.Value());
                        if (!FF.IsSame(TopFace))
                        {
                            for (ExpE.Init(FF, TopAbs_EDGE); ExpE.More() && !Found; ExpE.Next())
                            {
                                TopoDS_Edge E1 = TopoDS.Edge(ExpE.Current());
                                TopExp.Vertices(E1, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                                NCollection_List<TopoDS_Shape>.Iterator it = new(myTopEdges);
                                for (; it.More() && !Found; it.Next())
                                {
                                    TopoDS_Edge E2 = TopoDS.Edge(it.Value());
                                    TopExp.Vertices(E2, out TopoDS_Vertex VT1, out TopoDS_Vertex VT2);

                                    if (V1.IsSame(VT1) || V1.IsSame(VT2) || V2.IsSame(VT1) || V2.IsSame(VT2))
                                    {
                                        Found = true;
                                        TopExp_Explorer ExpE2 = new();
                                        for (ExpE2.Init(FF, TopAbs_EDGE); ExpE2.More(); ExpE2.Next())
                                        {
                                            TopoDS_Edge E3 = TopoDS.Edge(ExpE2.Current());
                                            if (MapE.Contains(E3))
                                            {
                                                MapE.Remove(E3);
                                            }
                                            else
                                            {
                                                MapE.Add(E3);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                NCollection_List<TopoDS_Shape>.Iterator it2 = new(myTopEdges);
                for (; it2.More(); it2.Next())
                {
                    if (MapE.Contains(it2.Value()))
                    {
                        MapE.Remove(it2.Value());
                    }
                }

                foreach (TopoDS_Shape itMap in MapE)
                {
                    if (!BRep_Tool.Degenerated(TopoDS.Edge(itMap)))
                    {
                        myLatEdges.Append(itMap);
                    }
                }
            }
        }
    }

    /// <summary>The edges of the top of the boss (frozen).</summary>
    public ISchematiqList<TopoDS_Shape> TopEdges()
    {
        BeforePublishingTopology();
        return PublishedList(myTopEdges);
    }

    /// <summary><see cref="TopEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalTopEdges() => myTopEdges;

    /// <summary>The lateral edges of the boss (frozen).</summary>
    public ISchematiqList<TopoDS_Shape> LatEdges()
    {
        BeforePublishingTopology();
        return PublishedList(myLatEdges);
    }

    /// <summary><see cref="LatEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<TopoDS_Shape> InternalLatEdges() => myLatEdges;

    private void BeforePublishingTopology() => myShape.TShape()?.FreezeGraph();

    /// <summary>The curve along the draft prism through the barycentre of its profile.</summary>
    public override Geom_Curve? BarycCurve()
    {
        return myBCurve;
    }

    private static double HeightMax(in TopoDS_Shape theSbase,  // shape initial
                                    in TopoDS_Face theSkface,  // face de sketch
                                    in TopoDS_Shape theSFrom,  // shape from
                                    in TopoDS_Shape theSUntil) // shape until
    {
        Bnd_Box Box = new();
        BRepBndLib.Add(theSbase, ref Box);
        BRepBndLib.Add(theSkface, ref Box);
        if (!theSFrom.IsNull())
        {
            BRepBndLib.Add(theSFrom, ref Box);
        }
        if (!theSUntil.IsNull())
        {
            BRepBndLib.Add(theSUntil, ref Box);
        }

        double[] c = new double[6];

        Box.Get(out c[0], out c[2], out c[4], out c[1], out c[3], out c[5]);
        double par = Max(Max(Math.Abs(c[1] - c[0]), Math.Abs(c[3] - c[2])), Math.Abs(c[5] - c[4]));
        return par;
    }

    private static int SensOfPrism(Geom_Curve? C, in TopoDS_Shape Until)
    {
        LocOpe_CSIntersector ASI1 = new(Until);
        NCollection_Sequence<Geom_Curve?> scur = new();
        scur.Append(C);
        ASI1.Perform(scur);
        int sens = 1;
        if (ASI1.IsDone() && ASI1.NbPoints(1) >= 1)
        {
            int nb = ASI1.NbPoints(1);
            double prm1 = ASI1.Point(1, 1).Parameter();
            double prm2 = ASI1.Point(1, nb).Parameter();
            if (prm1 < 0.0 && prm2 < 0.0)
            {
                sens = -1;
            }
        }
        else if (BRepFeat.ParametricBarycenter(Until, C!) < 0)
        {
            sens = -1;
        }
        return sens;
    }

    private static void MajMap(in TopoDS_Shape theB, LocOpe_DPrism theP,
                               NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMap,
                               ref TopoDS_Shape theFShape, ref TopoDS_Shape theLShape)
    {
        TopExp_Explorer exp = new();
        if (!theP.InternalFirstShape().IsNull())
        {
            exp.Init(theP.InternalFirstShape(), TopAbs_WIRE);
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
        }

        if (!theP.InternalLastShape().IsNull())
        {
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
        }

        for (exp.Init(theB, TopAbs_EDGE); exp.More(); exp.Next())
        {
            if (!theMap.IsBound(exp.Current()))
            {
                TopoDS_Edge edg = TopoDS.Edge(exp.Current());
                theMap.Bind(edg, new NCollection_List<TopoDS_Shape>(theP.InternalShapes(edg)));
            }
        }
    }

    private static Geom_Curve? TestCurve(in TopoDS_Face Base)
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
        Geom_Surface s = BRep_Tool.Surface(Base)!;
        if (s.GetType() == typeof(Geom_RectangularTrimmedSurface))
        {
            s = ((Geom_RectangularTrimmedSurface)s).BasisSurface();
        }
        if (s is not Geom_Plane P)
        {
            return null;
        }
        gp_Pln pp = P.Pln();
        gp_Dir Normale = pp.Position().XDirection() ^ pp.Position().YDirection();
        gp_Ax1 theAx = new(bar, Normale);
        Geom_Line theLin = new(theAx);
        return theLin;
    }
}
