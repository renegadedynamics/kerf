// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeDPrism.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeDPrism.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using static Occt.BRepFeat_PerfSelection;
using static Occt.BRepFeat_StatusError;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
    private Face myPbase = new();
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> mySlface = new(ShapeHasher.Instance);
    private double myAngle;
    private readonly NCollection_Sequence<Curve3D> myCurves = new();
    private Curve3D? myBCurve;
    private NCollection_List<Shape> myTopEdges = new();
    private NCollection_List<Shape> myLatEdges = new();
    // OCCT: the class declares its own myStatusError, hiding BRepFeat_Form's; CurrentStatusError reads the form's
#pragma warning disable CS0414 // written, never read: as in OCCT
    private BRepFeat_StatusError myStatusError;
#pragma warning restore CS0414

    /// <summary>
    /// A draft prism of the planar face <paramref name="Pbase"/> on <paramref name="Sbase"/>, with the taper
    /// <paramref name="Angle"/> (radians), from the sketch face <paramref name="Skface"/>; <paramref name="Fuse"/> is
    /// the mode (0 cut, 1 fuse, 2 the feature alone).
    /// </summary>
    public BRepFeat_MakeDPrism(in Shape Sbase, in Face Pbase, in Face Skface, double Angle, int Fuse,
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
    public void Init(in Shape Sbase, in Face Pbase, in Face Skface, double Angle, int Mode,
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
        Explorer exp = new();
        for (exp.Init(mySbase, ShapeType.Face); exp.More(); exp.Next())
        {
            NCollection_List<Shape> thelist = new();
            myMap.Bind(exp.Current(), thelist);
            myMap[exp.Current()].Append(exp.Current());
        }
        myAngle = Angle;
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
        Shape VraiDPrism = theDPrism.InternalShape();

        MajMap(myPbase, theDPrism, myMap, ref myFShape, ref myLShape);

        myGShape = VraiDPrism;
        GeneratedShapeValid();

        Shape Base = theDPrism.InternalFirstShape();
        Explorer exp = new(Base, ShapeType.Face);
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
        Curve3D? C = TestCurve(myPbase);
        int sens = SensOfPrism(C, mySUntil);

        BrepBuilder bB = new();
        double Height = sens * HeightMax(mySbase, mySkface, mySFrom, mySUntil);
        LocOpe_DPrism theDPrism = new(myPbase, Height, myAngle);
        Shape VraiDPrism = theDPrism.InternalShape();
        if (!Trf)
        {
            MajMap(myPbase, theDPrism, myMap, ref myFShape, ref myLShape);
            myGShape = VraiDPrism;
            GeneratedShapeValid();
            Shape Base = theDPrism.InternalFirstShape();
            exp.Init(Base, ShapeType.Face);
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
            Curve3D? C1;
            if (sens == -1)
            {
                C1 = C!.Reversed();
            }
            else
            {
                C1 = C;
            }
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(C1);
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
                Face FUntil = ASI.Point(1, 1).Face();
                Compound Comp = new();
                bB.MakeCompound(ref Comp);
                Solid S = BRepFeat.Tool(mySUntil, FUntil, Or);
                if (!S.IsNull)
                {
                    bB.Add(Comp, S);
                }

                BRepAlgoAPI_Cut trP = new(VraiDPrism, Comp);
                UpdateDescendants(trP, trP.InternalShape(), false);
                Explorer ex = new(trP.InternalShape(), ShapeType.Solid);
                Shape Cutsh = ex.Current();
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
        NCollection_List<Shape>.Iterator ited = new(myNewEdges);
        for (; ited.More(); ited.Next())
        {
            Edge ledg = ShapeCasts.Edge(ited.Value());
            if (!BRepAlgo.IsValid(ledg))
            {
                bB.SameRange(ledg, false);
                bB.SameParameter(ledg, false);
                BRepLib.SameParameter(ledg, Tool.Tolerance(ledg));
            }
        }
    }

    /// <summary>Builds a draft prism from the shape <paramref name="From"/> up to the shape <paramref name="Until"/>.</summary>
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
        Curve3D? C = TestCurve(myPbase);
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
        Shape VraiDPrism = theDPrism.InternalShape();
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
            Curve3D? C1;
            if (sens == -1)
            {
                C1 = C!.Reversed();
            }
            else
            {
                C1 = C;
            }
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(C1);
            LocOpe_CSIntersector ASI1 = new(mySUntil);
            LocOpe_CSIntersector ASI2 = new(mySFrom);
            ASI1.Perform(scur);
            ASI2.Perform(scur);
            Orientation OrU, OrF;
            Face FFrom, FUntil;
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
                OrF = global::Kerf.Engine.Topology.TopAbs.Reverse(OrF);
                FFrom = ASI2.Point(1, 1).Face();
            }
            else
            {
                NotDone();
                myStatusError = BRepFeat_NoIntersectF;
                return;
            }
            Compound Comp = new();
            BrepBuilder B = new();
            B.MakeCompound(ref Comp);
            Solid S = BRepFeat.Tool(mySUntil, FUntil, OrU);
            if (!S.IsNull)
            {
                B.Add(Comp, S);
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
        Shape VraiDPrism = theDPrism.InternalShape();

        MajMap(myPbase, theDPrism, myMap, ref myFShape, ref myLShape);
        myGShape = VraiDPrism;
        GeneratedShapeValid();
        GluedFacesValid();

        theDPrism.Curves(myCurves);
        myBCurve = theDPrism.BarycCurve();
        GlobalPerform();
    }

    /// <summary>Builds a draft prism from the end of the basis shape up to the shape <paramref name="Until"/>.</summary>
    /// <exception cref="ConstructionErrorException">Until is null or has no face.</exception>
    public void PerformFromEnd(in Shape Until)
    {
        if (Until.IsNull)
        {
            throw new ConstructionErrorException();
        }
        if (!mySkface.IsNull && Working(Until).IsSame(mySkface))
        {
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
        Curve3D? C = TestCurve(myPbase);
        int sens = SensOfPrism(C, mySUntil);
        double Height = sens * HeightMax(mySbase, mySkface, mySFrom, mySUntil);
        LocOpe_DPrism theDPrism = new(myPbase, Height, Height, myAngle);
        Shape VraiDPrism = theDPrism.InternalShape();
        if (VraiDPrism.IsNull)
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
            Curve3D? C2;
            if (sens == -1)
            {
                C2 = C!.Reversed();
            }
            else
            {
                C2 = C;
            }
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(C2);
            LocOpe_CSIntersector ASI1 = new(mySUntil);
            LocOpe_CSIntersector ASI2 = new(mySbase);
            ASI1.Perform(scur);
            ASI2.Perform(scur);
            Orientation OrU = Forward, OrF = Forward;
            Face FUntil = new(), FFrom = new();
            if (ASI1.IsDone() && ASI1.NbPoints(1) >= 1)
            {
                OrU = ASI1.Point(1, 1).Orientation();
                double prm = ASI1.Point(1, 1).Parameter();
                if (prm < 0)
                {
                    OrU = global::Kerf.Engine.Topology.TopAbs.Reverse(OrU);
                }
                FUntil = ASI1.Point(1, 1).Face();
            }

            if (ASI2.IsDone() && ASI2.NbPoints(1) >= 1)
            {
                int jj = ASI2.NbPoints(1);
                double prm = ASI2.Point(1, 1).Parameter();
                FFrom = ASI2.Point(1, 1).Face();
                OrF = ASI2.Point(1, 1).Orientation();
                OrF = global::Kerf.Engine.Topology.TopAbs.Reverse(OrF);
                for (int iii = 1; iii <= jj; iii++)
                {
                    if (ASI2.Point(1, iii).Parameter() < prm)
                    {
                        prm = ASI2.Point(1, iii).Parameter();
                        FFrom = ASI2.Point(1, iii).Face();
                        OrF = ASI2.Point(1, iii).Orientation();
                        OrF = global::Kerf.Engine.Topology.TopAbs.Reverse(OrF);
                    }
                }
                Surface S = Tool.Surface(FFrom)!;
                if (S.GetType() == typeof(TrimmedSurface))
                {
                    S = ((TrimmedSurface)S).BasisSurface();
                }
                BRepLib_MakeFace fac = new(S, global::Kerf.Engine.Foundation.Precision.Confusion());
                mySFrom = fac.InternalFace();
                Trf = TransformShapeFU(0);
            }

            Compound Comp = new();
            BrepBuilder B = new();
            B.MakeCompound(ref Comp);
            Solid Sol = BRepFeat.Tool(mySUntil, FUntil, OrU);
            if (!Sol.IsNull)
            {
                B.Add(Comp, Sol);
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
        Shape VraiDPrism = theDPrism.InternalShape();
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
    /// <exception cref="ConstructionErrorException">Until has no face (a null Until included, see below).</exception>
    public void PerformUntilHeight(in Shape Until, double Height)
    {
        // OCCT: neither of the two first calls returns: a null Until goes on and throws below, a zero height goes on
        // and builds the draft prism again with the height 0
        if (Until.IsNull)
        {
            Perform(Height);
        }
        if (Height == 0)
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
        Curve3D? C = TestCurve(myPbase);
        int sens = SensOfPrism(C, mySUntil);
        LocOpe_DPrism theDPrism = new(myPbase, sens * Height, myAngle);
        Shape VraiDPrism = theDPrism.InternalShape();

        if (!Trf)
        { // case face finished
            MajMap(myPbase, theDPrism, myMap, ref myFShape, ref myLShape);
            myGShape = VraiDPrism;
            GeneratedShapeValid();
            Shape Base = theDPrism.InternalFirstShape();
            exp.Init(Base, ShapeType.Face);
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
            Curve3D? C1;
            if (sens == -1)
            {
                C1 = C!.Reversed();
            }
            else
            {
                C1 = C;
            }
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(C1);
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
                Face FUntil = ASI.Point(1, 1).Face();
                Compound Comp = new();
                BrepBuilder B = new();
                B.MakeCompound(ref Comp);
                Solid S = BRepFeat.Tool(mySUntil, FUntil, Or);
                if (!S.IsNull)
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
    public override void Curves(NCollection_Sequence<Curve3D> scur)
    {
        scur.Assign(myCurves);
    }

    /// <summary>
    /// Determines the top and lateral edges of the boss: <paramref name="signature"/> 1 or -1 takes the first shape of
    /// the draft prism as its top, 2 or -2 its last shape; a negative signature takes the new edges as lateral edges.
    /// </summary>
    public void BossEdges(int signature)
    {
        NCollection_List<Shape> theLastShape = new();
        theLastShape.Clear();
        if (signature == 1 || signature == -1)
        {
            theLastShape = new NCollection_List<Shape>(InternalFirstShape());
        }
        else if (signature == 2 || signature == -2)
        {
            theLastShape = new NCollection_List<Shape>(InternalLastShape());
        }
        else
        {
            return;
        }

        // Edges Top
        NCollection_List<Shape>.Iterator itLS = new();
        Explorer ExpE = new();
        for (itLS.Initialize(theLastShape); itLS.More(); itLS.Next())
        {
            Face FF = ShapeCasts.Face(itLS.Value());
            for (ExpE.Init(FF, ShapeType.Edge); ExpE.More(); ExpE.Next())
            {
                Edge EE = ShapeCasts.Edge(ExpE.Current());
                myTopEdges.Append(EE);
            }
        }

        // Edges Bottom
        if (signature < 0)
        {
            // Attention check if TgtEdges is important
            myLatEdges = new NCollection_List<Shape>(InternalNewEdges());
        }
        else if (signature > 0)
        {
            if (!myShape.IsNull)
            {
                // OCCT iterates this address-keyed map into the lateral edges: the port in the order of its additions and
                // removals (decision 1054)
                NCollection_Map<Shape> MapE = new(ShapeHasher.Instance);
                bool Found;

                Explorer ExpF = new();
                for (ExpF.Init(myShape, ShapeType.Face); ExpF.More(); ExpF.Next())
                {
                    Found = false;
                    Face FF = ShapeCasts.Face(ExpF.Current());
                    for (itLS.Initialize(theLastShape); itLS.More(); itLS.Next())
                    {
                        Face TopFace = ShapeCasts.Face(itLS.Value());
                        if (!FF.IsSame(TopFace))
                        {
                            for (ExpE.Init(FF, ShapeType.Edge); ExpE.More() && !Found; ExpE.Next())
                            {
                                Edge E1 = ShapeCasts.Edge(ExpE.Current());
                                Exploration.Vertices(E1, out Vertex V1, out Vertex V2);
                                NCollection_List<Shape>.Iterator it = new(myTopEdges);
                                for (; it.More() && !Found; it.Next())
                                {
                                    Edge E2 = ShapeCasts.Edge(it.Value());
                                    Exploration.Vertices(E2, out Vertex VT1, out Vertex VT2);

                                    if (V1.IsSame(VT1) || V1.IsSame(VT2) || V2.IsSame(VT1) || V2.IsSame(VT2))
                                    {
                                        Found = true;
                                        Explorer ExpE2 = new();
                                        for (ExpE2.Init(FF, ShapeType.Edge); ExpE2.More(); ExpE2.Next())
                                        {
                                            Edge E3 = ShapeCasts.Edge(ExpE2.Current());
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

                NCollection_List<Shape>.Iterator it2 = new(myTopEdges);
                for (; it2.More(); it2.Next())
                {
                    if (MapE.Contains(it2.Value()))
                    {
                        MapE.Remove(it2.Value());
                    }
                }

                foreach (Shape itMap in MapE)
                {
                    if (!Tool.Degenerated(ShapeCasts.Edge(itMap)))
                    {
                        myLatEdges.Append(itMap);
                    }
                }
            }
        }
    }

    /// <summary>The edges of the top of the boss (frozen).</summary>
    public ISchematiqList<Shape> TopEdges()
    {
        BeforePublishingTopology();
        return PublishedList(myTopEdges);
    }

    /// <summary><see cref="TopEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalTopEdges() => myTopEdges;

    /// <summary>The lateral edges of the boss (frozen).</summary>
    public ISchematiqList<Shape> LatEdges()
    {
        BeforePublishingTopology();
        return PublishedList(myLatEdges);
    }

    /// <summary><see cref="LatEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalLatEdges() => myLatEdges;

    private void BeforePublishingTopology() => myShape.TShape()?.FreezeGraph();

    /// <summary>The curve along the draft prism through the barycentre of its profile.</summary>
    public override Curve3D? BarycCurve()
    {
        return myBCurve;
    }

    private static double HeightMax(in Shape theSbase,  // shape initial
                                    in Face theSkface,  // face de sketch
                                    in Shape theSFrom,  // shape from
                                    in Shape theSUntil) // shape until
    {
        Box Box = new();
        BRepBndLib.Add(theSbase, ref Box);
        BRepBndLib.Add(theSkface, ref Box);
        if (!theSFrom.IsNull)
        {
            BRepBndLib.Add(theSFrom, ref Box);
        }
        if (!theSUntil.IsNull)
        {
            BRepBndLib.Add(theSUntil, ref Box);
        }

        double[] c = new double[6];

        Box.Get(out c[0], out c[2], out c[4], out c[1], out c[3], out c[5]);
        double par = Max(Max(Math.Abs(c[1] - c[0]), Math.Abs(c[3] - c[2])), Math.Abs(c[5] - c[4]));
        return par;
    }

    private static int SensOfPrism(Curve3D? C, in Shape Until)
    {
        LocOpe_CSIntersector ASI1 = new(Until);
        NCollection_Sequence<Curve3D?> scur = new();
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

    private static void MajMap(in Shape theB, LocOpe_DPrism theP,
                               NCollection_DataMap<Shape, NCollection_List<Shape>> theMap,
                               ref Shape theFShape, ref Shape theLShape)
    {
        Explorer exp = new();
        if (!theP.InternalFirstShape().IsNull)
        {
            exp.Init(theP.InternalFirstShape(), ShapeType.Wire);
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
        }

        if (!theP.InternalLastShape().IsNull)
        {
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
        }

        for (exp.Init(theB, ShapeType.Edge); exp.More(); exp.Next())
        {
            if (!theMap.IsBound(exp.Current()))
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                theMap.Bind(edg, new NCollection_List<Shape>(theP.InternalShapes(edg)));
            }
        }
    }

    private static Curve3D? TestCurve(in Face Base)
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
        Surface s = Tool.Surface(Base)!;
        if (s.GetType() == typeof(TrimmedSurface))
        {
            s = ((TrimmedSurface)s).BasisSurface();
        }
        if (s is not PlaneSurface P)
        {
            return null;
        }
        Plane3D pp = P.Pln();
        Direction3D Normale = pp.Position().XDirection() ^ pp.Position().YDirection();
        Axis3D theAx = new(bar, Normale);
        LineCurve3D theLin = new(theAx);
        return theLin;
    }
}
