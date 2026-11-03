// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepMAT2d/BRepMAT2d_Explorer.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepMAT2d/BRepMAT2d_Explorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
    private readonly NCollection_Sequence<NCollection_Sequence<Curve2D>> theCurves = new();
    private int current;
    private int currentContour;
    private Shape myShape;
    private readonly NCollection_Sequence<bool> myIsClosed = new();
    private readonly NCollection_IndexedDataMap<Shape, Shape> myModifShapes =
        new(ShapeHasher.Instance);

    public BRepMAT2d_Explorer()
    {
        Clear();
    }

    public BRepMAT2d_Explorer(in Face aFace)
    {
        Perform(aFace);
    }

    /// <summary>Reads the contours of aFace.</summary>
    public void Perform(in Face aFace)
    {
        Clear();
        myShape = aFace;
        Face F = ShapeCasts.Face(aFace);
        F.Orientation(Forward);
        Explorer Exp = new(F, ShapeType.Wire);
        Surface aSurf = Tool.Surface(F)!;
        // OCCT.NET: InternalFace, unfrozen: the wires are added to it.
        Face aNewF = new BRepBuilderAPI_MakeFace(aSurf, global::Kerf.Engine.Foundation.Precision.Confusion()).InternalFace();

        while (Exp.More())
        {
            Add(ShapeCasts.Wire(Exp.Current()), F, aNewF);
            Exp.Next();
        }

        BRepLib.BuildCurves3d(aNewF);

        myModifShapes.Add(aFace, aNewF);
    }

    // Adds the contour of the wire Spine of aFace, checking the connection of its curves, and adds the wire (or its
    // new version) to aNewFace.
    private void Add(in Wire Spine, in Face aFace, in Face aNewFace)
    {
        // Taking into account the side of the bisecting loci: the wire as it is in the face.
        WireExplorer anExp = new(Spine, aFace);
        NCollection_IndexedDataMap<Shape, Shape> anOldNewE = new(ShapeHasher.Instance);

        if (!anExp.More())
        {
            return;
        }

        // Checks the connection and creates a new spine.
        NewContour();
        myIsClosed[currentContour] = Spine.Closed();

        Edge aFirstEdge = anExp.Current();
        Edge aPrevEdge = aFirstEdge;
        double UFirst, ULast, aD;
        Curve2D C2d;
        TrimmedCurve2D CT2d;
        TrimmedCurve2D aFirstCurve;
        Point2D aPFirst;
        Point2D aPLast;
        Point2D aPCurFirst;
        // The confusion tolerance of the further algorithm.
        double aTolConf = 1.0e-8;
        bool isModif = false;

        // The first edge of the wire.
        anOldNewE.Add(aFirstEdge, aFirstEdge);
        C2d = Tool.CurveOnSurface(aFirstEdge, aFace, out UFirst, out ULast)!;
        CT2d = new TrimmedCurve2D(C2d, UFirst, ULast);

        if (aFirstEdge.Orientation() == Reversed)
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
            Edge anEdge = anExp.Current();

            anOldNewE.Add(anEdge, anEdge);
            C2d = Tool.CurveOnSurface(anEdge, aFace, out UFirst, out ULast)!;
            CT2d = new TrimmedCurve2D(C2d, UFirst, ULast);

            if (anEdge.Orientation() == Reversed)
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
                BoundedCurve2D CPrev = (BoundedCurve2D)theCurves.ChangeValue(currentContour).ChangeValue(aNbC);
                CurveKind TCPrev = GetCurveType(CPrev);
                CurveKind TCCurr = GetCurveType(CT2d);

                if (TCCurr <= TCPrev)
                {
                    CT2d = AdjustCurveEnd(CT2d, aPLast, true);
                    // Creation of a new edge.
                    Edge aNewEdge;
                    Vertex aVf = Exploration.FirstVertex(anEdge);
                    Vertex aVl = Exploration.LastVertex(anEdge);

                    if (anEdge.Orientation() == Forward)
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
                    Point2D aP = CT2d.Value(CT2d.FirstParameter());

                    CPrev = AdjustCurveEnd(CPrev, aP, false);
                    theCurves.ChangeValue(currentContour).ChangeValue(aNbC) = CPrev;
                    // Change of the previous edge.
                    Edge aNewEdge;
                    Vertex aVf = Exploration.FirstVertex(aPrevEdge);
                    Vertex aVl = Exploration.LastVertex(aPrevEdge);

                    if (aPrevEdge.Orientation() == Forward)
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
            BoundedCurve2D CPrev = (BoundedCurve2D)theCurves.ChangeValue(currentContour).ChangeValue(aNbC);
            CurveKind TCPrev = GetCurveType(CPrev);
            CurveKind TCCurr = GetCurveType(aFirstCurve);

            if (TCCurr <= TCPrev)
            {
                aFirstCurve = AdjustCurveEnd(aFirstCurve, aPLast, true);
                theCurves.ChangeValue(currentContour).ChangeValue(1) = aFirstCurve;
                // Creation of a new edge.
                Edge aNewEdge;
                Vertex aVf = Exploration.FirstVertex(aFirstEdge);
                Vertex aVl = Exploration.LastVertex(aFirstEdge);

                if (aFirstEdge.Orientation() == Forward)
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
                Point2D aP = aFirstCurve.Value(aFirstCurve.FirstParameter());

                CPrev = AdjustCurveEnd(CPrev, aP, false);
                theCurves.ChangeValue(currentContour).ChangeValue(aNbC) = CPrev;
                // Change of the previous edge.
                Edge aNewEdge;
                Vertex aVf = Exploration.FirstVertex(aPrevEdge);
                Vertex aVl = Exploration.LastVertex(aPrevEdge);

                if (aPrevEdge.Orientation() == Forward)
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

        Wire aNewWire = new();
        BrepBuilder aBuilder = new();

        if (isModif)
        {
            int i;
            int aNbEdges = anOldNewE.Extent();

            aBuilder.MakeWire(ref aNewWire);

            for (i = 1; i <= aNbEdges; i++)
            {
                Shape aKey = anOldNewE.FindKey(i);
                Shape aNewEdge = anOldNewE.FindFromIndex(i);

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
        NCollection_Sequence<Curve2D> Contour = new();
        theCurves.Append(Contour);
        myIsClosed.Append(false);
        currentContour++;
    }

    // Adds aCurve to the current contour.
    private void Add(Curve2D aCurve) => theCurves.ChangeValue(currentContour).Append(aCurve);

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
    public Curve2D Value() => theCurves.Value(currentContour).Value(current);

    /// <summary>The face read.</summary>
    public Shape Shape() => myShape;

    /// <summary>The curves of the contour IC.</summary>
    public NCollection_Sequence<Curve2D> Contour(int IC) => theCurves.Value(IC);

    /// <summary>True if aShape (the face, a wire or an edge read) was replaced.</summary>
    public bool IsModified(in Shape aShape)
    {
        if (myModifShapes.Contains(aShape))
        {
            Shape aNewShape = myModifShapes.FindFromKey(aShape);
            bool isSame = aNewShape.IsSame(aShape);

            return !isSame;
        }

        return false;
    }

    /// <summary>The shape that replaces aShape, or aShape itself if it isn't replaced.</summary>
    public Shape ModifiedShape(in Shape aShape)
    {
        if (myModifShapes.Contains(aShape))
        {
            Shape aNewShape = myModifShapes.FindFromKey(aShape);

            return aNewShape;
        }

        return aShape;
    }

    /// <summary>For each contour, whether its wire is closed.</summary>
    public NCollection_Sequence<bool> GetIsClosed() => myIsClosed;

    // An edge from a 2D curve on a face and two vertices.
    private static Edge MakeEdge(Curve2D theCurve, in Face theFace, in Vertex theVFirst,
        in Vertex theVLast)
    {
        Edge aNewEdge = new();
        BrepBuilder aBuilder = new();
        double aTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        double aFPar = theCurve.FirstParameter();
        double aLPar = theCurve.LastParameter();

        aBuilder.MakeEdge(ref aNewEdge);
        aBuilder.UpdateEdge(aNewEdge, theCurve, theFace, aTol);
        aBuilder.Add(aNewEdge, theVFirst.Oriented(Forward));
        aBuilder.Add(aNewEdge, theVLast.Oriented(Reversed));
        aBuilder.Range(aNewEdge, aFPar, aLPar);

        return aNewEdge;
    }

    private static CurveKind GetCurveType(Curve2D theC2d)
    {
        CurveKind aTypeCurve;
        Type TheType = theC2d.GetType();

        if (TheType == typeof(TrimmedCurve2D))
        {
            TheType = ((TrimmedCurve2D)theC2d).ChangeBasisCurve().GetType();
        }

        if (TheType == typeof(CircleCurve2D))
        {
            aTypeCurve = Circle;
        }
        else if (TheType == typeof(LineCurve2D))
        {
            aTypeCurve = Line;
        }
        else if (TheType == typeof(EllipseCurve2D))
        {
            aTypeCurve = Ellipse;
        }
        else if (TheType == typeof(ParabolaCurve2D))
        {
            aTypeCurve = Parabola;
        }
        else if (TheType == typeof(HyperbolaCurve2D))
        {
            aTypeCurve = Hyperbola;
        }
        else if (TheType == typeof(BezierCurve2D))
        {
            aTypeCurve = BezierCurve;
        }
        else if (TheType == typeof(BSplineCurve2D))
        {
            aTypeCurve = BSplineCurve;
        }
        else if (TheType == typeof(OffsetCurve2D))
        {
            aTypeCurve = OffsetCurve;
        }
        else
        {
            aTypeCurve = OtherCurve;
        }

        return aTypeCurve;
    }

    private static TrimmedCurve2D AdjustCurveEnd(BoundedCurve2D theC2d, Point2D theP, bool isFirst)
    {
        CurveKind aType = GetCurveType(theC2d);

        if (aType == Line)
        {
            // A new line.
            if (isFirst)
            {
                Point2D aP = theC2d.Value(theC2d.LastParameter());
                return new MakeSegment2d(theP, aP).Value();
            }
            else
            {
                Point2D aP = theC2d.Value(theC2d.FirstParameter());
                return new MakeSegment2d(aP, theP).Value();
            }
        }
        else
        {
            // Converted to a B-spline whose end pole is moved.
            BSplineCurve2D BCurve = Geom2dConvert2D.CurveToBSplineCurve(theC2d, QuasiAngular);

            if (isFirst)
            {
                BCurve.SetPole(1, theP);
                return new TrimmedCurve2D(BCurve, BCurve.FirstParameter(), BCurve.LastParameter());
            }
            else
            {
                BCurve.SetPole(BCurve.PoleCount, theP);
                return new TrimmedCurve2D(BCurve, BCurve.FirstParameter(), BCurve.LastParameter());
            }
        }
    }
}
