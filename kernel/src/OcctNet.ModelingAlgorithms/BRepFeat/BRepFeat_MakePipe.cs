// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakePipe.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakePipe.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakePipe.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.BRepFeat_PerfSelection;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Constructs compound shapes with pipe features: a profile swept along a spine, as a depression (a cut) or a
/// protrusion (a fusion).
/// </summary>
/// <remarks>
/// <para>The mode is 0 for a depression, 1 for a protrusion, 2 for the feature alone. The pipe goes along the whole
/// spine, up to a face, or from a face up to another.</para>
/// <para>The basis shape, profile and spine are read, never written: the pipe sweeps a copy of a frozen profile
/// (LocOpe_Pipe, decision 1014).</para>
/// </remarks>
public sealed class BRepFeat_MakePipe : BRepFeat_Form
{
    private TopoDS_Shape myPbase = new();
    // OCCT: the class declares its own mySkface, hiding BRepFeat_Form's, which stays null
    private new TopoDS_Face mySkface = new();
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mySlface = new(TopTools_ShapeMapHasher.Instance);
    private TopoDS_Wire mySpine = new();
    private NCollection_Sequence<Geom_Curve> myCurves = new();
    private Geom_Curve? myBCurve;

    /// <summary>An empty pipe feature.</summary>
    public BRepFeat_MakePipe()
    {
    }

    /// <summary>
    /// A pipe feature of the profile <paramref name="Pbase"/> along <paramref name="Spine"/> on
    /// <paramref name="Sbase"/>, from the sketch face <paramref name="Skface"/>; <paramref name="Fuse"/> is the mode
    /// (0 cut, 1 fuse, 2 the feature alone).
    /// </summary>
    public BRepFeat_MakePipe(in TopoDS_Shape Sbase, in TopoDS_Shape Pbase, in TopoDS_Face Skface, in TopoDS_Wire Spine,
                             int Fuse, bool Modify)
    {
        Init(Sbase, Pbase, Skface, Spine, Fuse, Modify);
    }

    /// <summary>Initializes the pipe feature (see the constructor).</summary>
    public void Init(in TopoDS_Shape Sbase, in TopoDS_Shape Pbase, in TopoDS_Face Skface, in TopoDS_Wire Spine, int Mode,
                     bool Modify)
    {
        // OCCT.NET: the working copies of frozen inputs (decision 1052)
        StartWorkingShapes();
        mySbase = Working(Sbase);
        BasisShapeValid();
        mySkface = Working(Skface);
        SketchFaceValid();
        myPbase = Working(Pbase);
        mySlface.Clear();
        mySpine = TopoDS.Wire(Working(Spine));
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
    /// <exception cref="ConstructionErrorException">F isn't a face of the basis shape, or E an edge of the profile.</exception>
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
            throw new ConstructionErrorException();
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
            throw new ConstructionErrorException();
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

    /// <summary>Builds the pipe along the whole spine.</summary>
    public void Perform()
    {
        mySFrom.Nullify();
        ShapeFromValid();
        mySUntil.Nullify();
        ShapeUntilValid();
        myGluedF.Clear();
        myPerfSelection = BRepFeat_NoSelection;
        PerfSelectionValid();
        TopoDS_Shape theBase = myPbase;
        LocOpe_Pipe thePipe = new(mySpine, theBase);
        TopoDS_Shape VraiPipe = thePipe.InternalShape();
        MajMap(myPbase, thePipe, myMap, ref myFShape, ref myLShape);
        myGShape = VraiPipe;
        GeneratedShapeValid();

        GluedFacesValid();

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
            myFShape = thePipe.InternalFirstShape();
            NCollection_Sequence<Point3D> spt = new();
            LocOpe.SampleEdges(myFShape, spt);
            myCurves = new NCollection_Sequence<Geom_Curve>(thePipe.Curves(spt)!);
            myBCurve = thePipe.BarycCurve();
            GlobalPerform();
        }
    }

    /// <summary>Builds the pipe up to the shape <paramref name="Until"/>.</summary>
    /// <exception cref="ConstructionErrorException">Until is null or has no face.</exception>
    public void Perform(in TopoDS_Shape Until)
    {
        if (Until.IsNull())
        {
            throw new ConstructionErrorException();
        }
        TopExp_Explorer exp = new(Until, TopAbs_FACE);
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
        TransformShapeFU(1);
        ShapeUntilValid();
        LocOpe_Pipe thePipe = new(mySpine, myPbase);
        TopoDS_Shape VraiTuyau = thePipe.InternalShape();
        MajMap(myPbase, thePipe, myMap, ref myFShape, ref myLShape);
        myGShape = VraiTuyau;
        GeneratedShapeValid();
        GluedFacesValid();
        // OCCT: myFShape becomes the pipe's first shape, which the map doesn't hold (FirstShape then throws)
        myFShape = thePipe.InternalFirstShape();
        NCollection_Sequence<Point3D> spt = new();
        LocOpe.SampleEdges(myFShape, spt);
        myCurves = new NCollection_Sequence<Geom_Curve>(thePipe.Curves(spt)!);
        myBCurve = thePipe.BarycCurve();
        GlobalPerform();
    }

    /// <summary>Builds the pipe from the shape <paramref name="From"/> up to the shape <paramref name="Until"/>.</summary>
    /// <exception cref="ConstructionErrorException">From or Until is null or has no face.</exception>
    public void Perform(in TopoDS_Shape From, in TopoDS_Shape Until)
    {
        if (From.IsNull() || Until.IsNull())
        {
            throw new ConstructionErrorException();
        }
        if (!mySkface.IsNull())
        {
            if (Working(From).IsSame(mySkface))
            {
                Perform(Until);
                return;
            }
            else if (Working(Until).IsSame(mySkface))
            {
                Perform(From);
                return;
            }
        }
        myGluedF.Clear();
        myPerfSelection = BRepFeat_SelectionFU;
        PerfSelectionValid();
        TopExp_Explorer exp = new(From, TopAbs_FACE);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }
        exp.Init(Until, TopAbs_FACE);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }
        mySFrom = Working(From);
        TransformShapeFU(0);
        ShapeFromValid();
        mySUntil = Working(Until);
        TransformShapeFU(1);
        ShapeUntilValid();
        LocOpe_Pipe thePipe = new(mySpine, myPbase);
        TopoDS_Shape VraiTuyau = thePipe.InternalShape();
        MajMap(myPbase, thePipe, myMap, ref myFShape, ref myLShape);
        myGShape = VraiTuyau;
        GeneratedShapeValid();
        GluedFacesValid();
        myFShape = thePipe.InternalFirstShape();
        NCollection_Sequence<Point3D> spt = new();
        LocOpe.SampleEdges(myFShape, spt);
        myCurves = new NCollection_Sequence<Geom_Curve>(thePipe.Curves(spt)!);
        myBCurve = thePipe.BarycCurve();
        GlobalPerform();
    }

    /// <summary>The curves of the pipe through the sample points of its first shape.</summary>
    public override void Curves(NCollection_Sequence<Geom_Curve> scur)
    {
        scur.Assign(myCurves);
    }

    /// <summary>The curve of the pipe through the barycentre of its first shape.</summary>
    public override Geom_Curve? BarycCurve()
    {
        return myBCurve;
    }

    private static void MajMap(in TopoDS_Shape theB, LocOpe_Pipe theP,
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
}
