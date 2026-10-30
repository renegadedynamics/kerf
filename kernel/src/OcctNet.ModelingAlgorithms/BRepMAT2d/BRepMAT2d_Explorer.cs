// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepMAT2d/BRepMAT2d_Explorer.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepMAT2d/BRepMAT2d_Explorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Convert_ParameterisationType;
using static Occt.GeomAbs_CurveType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The contours of a face as sequences of 2D curves (its pcurves, oriented along the wires), the input of
/// BRepMAT2d_BisectingLocus. Where two consecutive curves don't meet (more than 1e-8 apart), one of them is adjusted
/// (a new segment, or a B-spline whose end pole is moved) and a new edge made for it on a new face; ModifiedShape
/// gives the new edges, wires and face.
/// </summary>
/// <remarks>
/// A working object. The face given is never written: its pcurves are read (and copied by Geom2d_TrimmedCurve), and
/// the new face is built on its surface from the caller's wires, or new wires where edges were replaced.
/// BRepLib::BuildCurves3d then computes the 3D curves the new face's edges lack: only the new edges (the caller's
/// edges of a planar face have their 3D curves; an edge without one, frozen, would throw Standard_ImmutableObject,
/// where OCCT writes the curve into the caller's edge). The new face is handed out unfrozen, as OCCT's working
/// shape (BRepFill_OffsetWire builds on it).
/// </remarks>
public sealed class BRepMAT2d_Explorer
{
    private readonly NCollection_Sequence<NCollection_Sequence<Geom2d_Curve>> theCurves = new();
    private int current;
    private int currentContour;
    private TopoDS_Shape myShape;
    private readonly NCollection_Sequence<bool> myIsClosed = new();
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> myModifShapes =
        new(TopTools_ShapeMapHasher.Instance);

    public BRepMAT2d_Explorer()
    {
        Clear();
    }

    public BRepMAT2d_Explorer(in TopoDS_Face aFace)
    {
        Perform(aFace);
    }

    /// <summary>Reads the contours of aFace.</summary>
    public void Perform(in TopoDS_Face aFace)
    {
        Clear();
        myShape = aFace;
        TopoDS_Face F = TopoDS.Face(aFace);
        F.Orientation(TopAbs_FORWARD);
        TopExp_Explorer Exp = new(F, TopAbs_WIRE);
        Geom_Surface aSurf = BRep_Tool.Surface(F)!;
        // OCCT.NET: InternalFace, unfrozen: the wires are added to it.
        TopoDS_Face aNewF = new BRepBuilderAPI_MakeFace(aSurf, Precision.Confusion()).InternalFace();

        while (Exp.More())
        {
            Add(TopoDS.Wire(Exp.Current()), F, aNewF);
            Exp.Next();
        }

        BRepLib.BuildCurves3d(aNewF);

        myModifShapes.Add(aFace, aNewF);
    }

    // Adds the contour of the wire Spine of aFace, checking the connection of its curves, and adds the wire (or its
    // new version) to aNewFace.
    private void Add(in TopoDS_Wire Spine, in TopoDS_Face aFace, in TopoDS_Face aNewFace)
    {
        // Taking into account the side of the bisecting loci: the wire as it is in the face.
        BRepTools_WireExplorer anExp = new(Spine, aFace);
        NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> anOldNewE = new(TopTools_ShapeMapHasher.Instance);

        if (!anExp.More())
        {
            return;
        }

        // Checks the connection and creates a new spine.
        NewContour();
        myIsClosed[currentContour] = Spine.Closed();

        TopoDS_Edge aFirstEdge = anExp.Current();
        TopoDS_Edge aPrevEdge = aFirstEdge;
        double UFirst, ULast, aD;
        Geom2d_Curve C2d;
        Geom2d_TrimmedCurve CT2d;
        Geom2d_TrimmedCurve aFirstCurve;
        gp_Pnt2d aPFirst;
        gp_Pnt2d aPLast;
        gp_Pnt2d aPCurFirst;
        // The confusion tolerance of the further algorithm.
        double aTolConf = 1.0e-8;
        bool isModif = false;

        // The first edge of the wire.
        anOldNewE.Add(aFirstEdge, aFirstEdge);
        C2d = BRep_Tool.CurveOnSurface(aFirstEdge, aFace, out UFirst, out ULast)!;
        CT2d = new Geom2d_TrimmedCurve(C2d, UFirst, ULast);

        if (aFirstEdge.Orientation() == TopAbs_REVERSED)
        {
            CT2d.Reverse();
        }

        aPFirst = CT2d.Value(CT2d.FirstParameter());
        aPLast = CT2d.Value(CT2d.LastParameter());

        Add(CT2d);
        aFirstCurve = CT2d;
        anExp.Next();

        // The next edges.
        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Edge anEdge = anExp.Current();

            anOldNewE.Add(anEdge, anEdge);
            C2d = BRep_Tool.CurveOnSurface(anEdge, aFace, out UFirst, out ULast)!;
            CT2d = new Geom2d_TrimmedCurve(C2d, UFirst, ULast);

            if (anEdge.Orientation() == TopAbs_REVERSED)
            {
                CT2d.Reverse();
            }

            aPCurFirst = CT2d.Value(CT2d.FirstParameter());
            aD = aPLast.Distance(aPCurFirst);

            if (aD > aTolConf)
            {
                // The hole is filled by moving the end of one of the two curves (a new line, or a B-spline whose
                // end pole is moved).
                isModif = true;

                int aNbC = theCurves.Value(currentContour).Length();
                Geom2d_BoundedCurve CPrev = (Geom2d_BoundedCurve)theCurves.ChangeValue(currentContour).ChangeValue(aNbC);
                GeomAbs_CurveType TCPrev = GetCurveType(CPrev);
                GeomAbs_CurveType TCCurr = GetCurveType(CT2d);

                if (TCCurr <= TCPrev)
                {
                    CT2d = AdjustCurveEnd(CT2d, aPLast, true);
                    // Creation of a new edge.
                    TopoDS_Edge aNewEdge;
                    TopoDS_Vertex aVf = TopExp.FirstVertex(anEdge);
                    TopoDS_Vertex aVl = TopExp.LastVertex(anEdge);

                    if (anEdge.Orientation() == TopAbs_FORWARD)
                    {
                        aNewEdge = MakeEdge(CT2d, aNewFace, aVf, aVl);
                    }
                    else
                    {
                        aNewEdge = MakeEdge(CT2d.Reversed(), aNewFace, aVf, aVl);
                    }

                    aNewEdge.Orientation(anEdge.Orientation());
                    anOldNewE.ChangeFromKey(anEdge) = aNewEdge;
                }
                else
                {
                    gp_Pnt2d aP = CT2d.Value(CT2d.FirstParameter());

                    CPrev = AdjustCurveEnd(CPrev, aP, false);
                    theCurves.ChangeValue(currentContour).ChangeValue(aNbC) = CPrev;
                    // Change of the previous edge.
                    TopoDS_Edge aNewEdge;
                    TopoDS_Vertex aVf = TopExp.FirstVertex(aPrevEdge);
                    TopoDS_Vertex aVl = TopExp.LastVertex(aPrevEdge);

                    if (aPrevEdge.Orientation() == TopAbs_FORWARD)
                    {
                        aNewEdge = MakeEdge(CPrev, aNewFace, aVf, aVl);
                    }
                    else
                    {
                        aNewEdge = MakeEdge(CPrev.Reversed(), aNewFace, aVf, aVl);
                    }

                    aNewEdge.Orientation(aPrevEdge.Orientation());
                    anOldNewE.ChangeFromKey(aPrevEdge) = aNewEdge;
                }
            }

            aPLast = CT2d.Value(CT2d.LastParameter());
            Add(CT2d);
            aPrevEdge = anEdge;
        }

        // The distance between the first and the last point of the wire, if it is closed.
        if (myIsClosed[currentContour] && aPLast.Distance(aPFirst) > aTolConf)
        {
            isModif = true;

            int aNbC = theCurves.Value(currentContour).Length();
            Geom2d_BoundedCurve CPrev = (Geom2d_BoundedCurve)theCurves.ChangeValue(currentContour).ChangeValue(aNbC);
            GeomAbs_CurveType TCPrev = GetCurveType(CPrev);
            GeomAbs_CurveType TCCurr = GetCurveType(aFirstCurve);

            if (TCCurr <= TCPrev)
            {
                aFirstCurve = AdjustCurveEnd(aFirstCurve, aPLast, true);
                theCurves.ChangeValue(currentContour).ChangeValue(1) = aFirstCurve;
                // Creation of a new edge.
                TopoDS_Edge aNewEdge;
                TopoDS_Vertex aVf = TopExp.FirstVertex(aFirstEdge);
                TopoDS_Vertex aVl = TopExp.LastVertex(aFirstEdge);

                if (aFirstEdge.Orientation() == TopAbs_FORWARD)
                {
                    aNewEdge = MakeEdge(aFirstCurve, aNewFace, aVf, aVl);
                }
                else
                {
                    aNewEdge = MakeEdge(aFirstCurve.Reversed(), aNewFace, aVf, aVl);
                }

                aNewEdge.Orientation(aFirstEdge.Orientation());
                anOldNewE.ChangeFromKey(aFirstEdge) = aNewEdge;
            }
            else
            {
                gp_Pnt2d aP = aFirstCurve.Value(aFirstCurve.FirstParameter());

                CPrev = AdjustCurveEnd(CPrev, aP, false);
                theCurves.ChangeValue(currentContour).ChangeValue(aNbC) = CPrev;
                // Change of the previous edge.
                TopoDS_Edge aNewEdge;
                TopoDS_Vertex aVf = TopExp.FirstVertex(aPrevEdge);
                TopoDS_Vertex aVl = TopExp.LastVertex(aPrevEdge);

                if (aPrevEdge.Orientation() == TopAbs_FORWARD)
                {
                    aNewEdge = MakeEdge(CPrev, aNewFace, aVf, aVl);
                }
                else
                {
                    aNewEdge = MakeEdge(CPrev.Reversed(), aNewFace, aVf, aVl);
                }

                aNewEdge.Orientation(aPrevEdge.Orientation());
                anOldNewE.ChangeFromKey(aPrevEdge) = aNewEdge;
            }
        }

        TopoDS_Wire aNewWire = new();
        BRep_Builder aBuilder = new();

        if (isModif)
        {
            int i;
            int aNbEdges = anOldNewE.Extent();

            aBuilder.MakeWire(ref aNewWire);

            for (i = 1; i <= aNbEdges; i++)
            {
                TopoDS_Shape aKey = anOldNewE.FindKey(i);
                TopoDS_Shape aNewEdge = anOldNewE.FindFromIndex(i);

                aBuilder.Add(aNewWire, aNewEdge);
                myModifShapes.Add(aKey, aNewEdge);
            }

            if (myIsClosed[currentContour])
            {
                aNewWire.Closed(true);
            }

            // The orientation of the wire is already taken into account.
            myModifShapes.Add(Spine, aNewWire);
        }
        else
        {
            aNewWire = Spine;
        }

        aBuilder.Add(aNewFace, aNewWire);
    }

    /// <summary>Clears the contents.</summary>
    public void Clear()
    {
        theCurves.Clear();
        currentContour = 0;
        myIsClosed.Clear();
        myModifShapes.Clear();
    }

    private void NewContour()
    {
        NCollection_Sequence<Geom2d_Curve> Contour = new();
        theCurves.Append(Contour);
        myIsClosed.Append(false);
        currentContour++;
    }

    // Adds aCurve to the current contour.
    private void Add(Geom2d_Curve aCurve) => theCurves.ChangeValue(currentContour).Append(aCurve);

    /// <summary>The number of contours.</summary>
    public int NumberOfContours() => theCurves.Length();

    /// <summary>The number of curves of the contour IndexContour.</summary>
    public int NumberOfCurves(int IndexContour) => theCurves.Value(IndexContour).Length();

    /// <summary>Starts an iteration over the curves of the contour IndexContour.</summary>
    public void Init(int IndexContour)
    {
        currentContour = IndexContour;
        current = 1;
    }

    /// <summary>False when there are no more curves on the contour.</summary>
    public bool More() => current <= NumberOfCurves(currentContour);

    /// <summary>Moves to the next curve of the contour.</summary>
    public void Next() => current++;

    /// <summary>The current curve of the contour.</summary>
    public Geom2d_Curve Value() => theCurves.Value(currentContour).Value(current);

    /// <summary>The face read.</summary>
    public TopoDS_Shape Shape() => myShape;

    /// <summary>The curves of the contour IC.</summary>
    public NCollection_Sequence<Geom2d_Curve> Contour(int IC) => theCurves.Value(IC);

    /// <summary>True if aShape (the face, a wire or an edge read) was replaced.</summary>
    public bool IsModified(in TopoDS_Shape aShape)
    {
        if (myModifShapes.Contains(aShape))
        {
            TopoDS_Shape aNewShape = myModifShapes.FindFromKey(aShape);
            bool isSame = aNewShape.IsSame(aShape);

            return !isSame;
        }

        return false;
    }

    /// <summary>The shape that replaces aShape, or aShape itself if it isn't replaced.</summary>
    public TopoDS_Shape ModifiedShape(in TopoDS_Shape aShape)
    {
        if (myModifShapes.Contains(aShape))
        {
            TopoDS_Shape aNewShape = myModifShapes.FindFromKey(aShape);

            return aNewShape;
        }

        return aShape;
    }

    /// <summary>For each contour, whether its wire is closed.</summary>
    public NCollection_Sequence<bool> GetIsClosed() => myIsClosed;

    // An edge from a 2D curve on a face and two vertices.
    private static TopoDS_Edge MakeEdge(Geom2d_Curve theCurve, in TopoDS_Face theFace, in TopoDS_Vertex theVFirst,
        in TopoDS_Vertex theVLast)
    {
        TopoDS_Edge aNewEdge = new();
        BRep_Builder aBuilder = new();
        double aTol = Precision.Confusion();
        double aFPar = theCurve.FirstParameter();
        double aLPar = theCurve.LastParameter();

        aBuilder.MakeEdge(ref aNewEdge);
        aBuilder.UpdateEdge(aNewEdge, theCurve, theFace, aTol);
        aBuilder.Add(aNewEdge, theVFirst.Oriented(TopAbs_FORWARD));
        aBuilder.Add(aNewEdge, theVLast.Oriented(TopAbs_REVERSED));
        aBuilder.Range(aNewEdge, aFPar, aLPar);

        return aNewEdge;
    }

    private static GeomAbs_CurveType GetCurveType(Geom2d_Curve theC2d)
    {
        GeomAbs_CurveType aTypeCurve;
        Type TheType = theC2d.GetType();

        if (TheType == typeof(Geom2d_TrimmedCurve))
        {
            TheType = ((Geom2d_TrimmedCurve)theC2d).ChangeBasisCurve().GetType();
        }

        if (TheType == typeof(Geom2d_Circle))
        {
            aTypeCurve = GeomAbs_Circle;
        }
        else if (TheType == typeof(Geom2d_Line))
        {
            aTypeCurve = GeomAbs_Line;
        }
        else if (TheType == typeof(Geom2d_Ellipse))
        {
            aTypeCurve = GeomAbs_Ellipse;
        }
        else if (TheType == typeof(Geom2d_Parabola))
        {
            aTypeCurve = GeomAbs_Parabola;
        }
        else if (TheType == typeof(Geom2d_Hyperbola))
        {
            aTypeCurve = GeomAbs_Hyperbola;
        }
        else if (TheType == typeof(Geom2d_BezierCurve))
        {
            aTypeCurve = GeomAbs_BezierCurve;
        }
        else if (TheType == typeof(Geom2d_BSplineCurve))
        {
            aTypeCurve = GeomAbs_BSplineCurve;
        }
        else if (TheType == typeof(Geom2d_OffsetCurve))
        {
            aTypeCurve = GeomAbs_OffsetCurve;
        }
        else
        {
            aTypeCurve = GeomAbs_OtherCurve;
        }

        return aTypeCurve;
    }

    private static Geom2d_TrimmedCurve AdjustCurveEnd(Geom2d_BoundedCurve theC2d, gp_Pnt2d theP, bool isFirst)
    {
        GeomAbs_CurveType aType = GetCurveType(theC2d);

        if (aType == GeomAbs_Line)
        {
            // A new line.
            if (isFirst)
            {
                gp_Pnt2d aP = theC2d.Value(theC2d.LastParameter());
                return new GC_MakeSegment2d(theP, aP).Value();
            }
            else
            {
                gp_Pnt2d aP = theC2d.Value(theC2d.FirstParameter());
                return new GC_MakeSegment2d(aP, theP).Value();
            }
        }
        else
        {
            // Converted to a B-spline whose end pole is moved.
            Geom2d_BSplineCurve BCurve = Geom2dConvert.CurveToBSplineCurve(theC2d, Convert_QuasiAngular);

            if (isFirst)
            {
                BCurve.SetPole(1, theP);
                return new Geom2d_TrimmedCurve(BCurve, BCurve.FirstParameter(), BCurve.LastParameter());
            }
            else
            {
                BCurve.SetPole(BCurve.NbPoles(), theP);
                return new Geom2d_TrimmedCurve(BCurve, BCurve.FirstParameter(), BCurve.LastParameter());
            }
        }
    }
}
