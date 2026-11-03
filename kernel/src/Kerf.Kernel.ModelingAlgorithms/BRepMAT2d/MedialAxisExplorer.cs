// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepMAT2d/BRepMAT2d_Explorer.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepMAT2d/BRepMAT2d_Explorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Offsets.MedialAxis;

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
public sealed class MedialAxisExplorer
{
    private readonly NCollection_Sequence<NCollection_Sequence<Curve2D>> _curves = new();
    private int _current;
    private int _currentContour;
    private Shape _shape;
    private readonly NCollection_Sequence<bool> _isClosed = new();
    private readonly NCollection_IndexedDataMap<Shape, Shape> _modifShapes =
        new(ShapeHasher.Instance);

    public MedialAxisExplorer()
    {
        Clear();
    }

    public MedialAxisExplorer(in Face aFace)
    {
        Perform(aFace);
    }

    /// <summary>Reads the contours of aFace.</summary>
    public void Perform(in Face aFace)
    {
        Clear();
        _shape = aFace;
        Face f = ShapeCasts.Face(aFace);
        f.Orientation(Forward);
        Explorer exp = new(f, ShapeType.Wire);
        Surface surf = Tool.Surface(f)!;
        // OCCT.NET: InternalFace, unfrozen: the wires are added to it.
        Face newF = new MakeFace(surf, global::Kerf.Engine.Foundation.Precision.Confusion()).InternalFace();

        while (exp.More())
        {
            Add(ShapeCasts.Wire(exp.Current()), f, newF);
            exp.Next();
        }

        BrepLibrary.BuildCurves3d(newF);

        _modifShapes.Add(aFace, newF);
    }

    // Adds the contour of the wire Spine of aFace, checking the connection of its curves, and adds the wire (or its
    // new version) to aNewFace.
    private void Add(in Wire spine, in Face aFace, in Face aNewFace)
    {
        // Taking into account the side of the bisecting loci: the wire as it is in the face.
        WireExplorer exp = new(spine, aFace);
        NCollection_IndexedDataMap<Shape, Shape> oldNewE = new(ShapeHasher.Instance);

        if (!exp.More())
        {
            return;
        }

        // Checks the connection and creates a new spine.
        NewContour();
        _isClosed[_currentContour] = spine.Closed();

        Edge firstEdge = exp.Current();
        Edge prevEdge = firstEdge;
        double uFirst, uLast, d;
        Curve2D c2d;
        TrimmedCurve2D ct2d;
        TrimmedCurve2D firstCurve;
        Point2D pFirst;
        Point2D pLast;
        Point2D pCurFirst;
        // The confusion tolerance of the further algorithm.
        double tolConf = 1.0e-8;
        bool isModif = false;

        // The first edge of the wire.
        oldNewE.Add(firstEdge, firstEdge);
        c2d = Tool.CurveOnSurface(firstEdge, aFace, out uFirst, out uLast)!;
        ct2d = new TrimmedCurve2D(c2d, uFirst, uLast);

        if (firstEdge.Orientation() == Reversed)
        {
            ct2d.Reverse();
        }

        pFirst = ct2d.Value(ct2d.FirstParameter());
        pLast = ct2d.Value(ct2d.LastParameter());

        Add(ct2d);
        firstCurve = ct2d;
        exp.Next();

        // The next edges.
        for (; exp.More(); exp.Next())
        {
            Edge edge = exp.Current();

            oldNewE.Add(edge, edge);
            c2d = Tool.CurveOnSurface(edge, aFace, out uFirst, out uLast)!;
            ct2d = new TrimmedCurve2D(c2d, uFirst, uLast);

            if (edge.Orientation() == Reversed)
            {
                ct2d.Reverse();
            }

            pCurFirst = ct2d.Value(ct2d.FirstParameter());
            d = pLast.Distance(pCurFirst);

            if (d > tolConf)
            {
                // The hole is filled by moving the end of one of the two curves (a new line, or a B-spline whose
                // end pole is moved).
                isModif = true;

                int nbC = _curves.Value(_currentContour).Length();
                BoundedCurve2D cPrev = (BoundedCurve2D)_curves.ChangeValue(_currentContour).ChangeValue(nbC);
                CurveKind tcPrev = GetCurveType(cPrev);
                CurveKind tcCurr = GetCurveType(ct2d);

                if (tcCurr <= tcPrev)
                {
                    ct2d = AdjustCurveEnd(ct2d, pLast, true);
                    // Creation of a new edge.
                    Edge newEdge;
                    Vertex vf = Exploration.FirstVertex(edge);
                    Vertex vl = Exploration.LastVertex(edge);

                    if (edge.Orientation() == Forward)
                    {
                        newEdge = MakeEdge(ct2d, aNewFace, vf, vl);
                    }
                    else
                    {
                        newEdge = MakeEdge(ct2d.Reversed(), aNewFace, vf, vl);
                    }

                    newEdge.Orientation(edge.Orientation());
                    oldNewE.ChangeFromKey(edge) = newEdge;
                }
                else
                {
                    Point2D p = ct2d.Value(ct2d.FirstParameter());

                    cPrev = AdjustCurveEnd(cPrev, p, false);
                    _curves.ChangeValue(_currentContour).ChangeValue(nbC) = cPrev;
                    // Change of the previous edge.
                    Edge newEdge;
                    Vertex vf = Exploration.FirstVertex(prevEdge);
                    Vertex vl = Exploration.LastVertex(prevEdge);

                    if (prevEdge.Orientation() == Forward)
                    {
                        newEdge = MakeEdge(cPrev, aNewFace, vf, vl);
                    }
                    else
                    {
                        newEdge = MakeEdge(cPrev.Reversed(), aNewFace, vf, vl);
                    }

                    newEdge.Orientation(prevEdge.Orientation());
                    oldNewE.ChangeFromKey(prevEdge) = newEdge;
                }
            }

            pLast = ct2d.Value(ct2d.LastParameter());
            Add(ct2d);
            prevEdge = edge;
        }

        // The distance between the first and the last point of the wire, if it is closed.
        if (_isClosed[_currentContour] && pLast.Distance(pFirst) > tolConf)
        {
            isModif = true;

            int nbC = _curves.Value(_currentContour).Length();
            BoundedCurve2D cPrev = (BoundedCurve2D)_curves.ChangeValue(_currentContour).ChangeValue(nbC);
            CurveKind tcPrev = GetCurveType(cPrev);
            CurveKind tcCurr = GetCurveType(firstCurve);

            if (tcCurr <= tcPrev)
            {
                firstCurve = AdjustCurveEnd(firstCurve, pLast, true);
                _curves.ChangeValue(_currentContour).ChangeValue(1) = firstCurve;
                // Creation of a new edge.
                Edge newEdge;
                Vertex vf = Exploration.FirstVertex(firstEdge);
                Vertex vl = Exploration.LastVertex(firstEdge);

                if (firstEdge.Orientation() == Forward)
                {
                    newEdge = MakeEdge(firstCurve, aNewFace, vf, vl);
                }
                else
                {
                    newEdge = MakeEdge(firstCurve.Reversed(), aNewFace, vf, vl);
                }

                newEdge.Orientation(firstEdge.Orientation());
                oldNewE.ChangeFromKey(firstEdge) = newEdge;
            }
            else
            {
                Point2D p = firstCurve.Value(firstCurve.FirstParameter());

                cPrev = AdjustCurveEnd(cPrev, p, false);
                _curves.ChangeValue(_currentContour).ChangeValue(nbC) = cPrev;
                // Change of the previous edge.
                Edge newEdge;
                Vertex vf = Exploration.FirstVertex(prevEdge);
                Vertex vl = Exploration.LastVertex(prevEdge);

                if (prevEdge.Orientation() == Forward)
                {
                    newEdge = MakeEdge(cPrev, aNewFace, vf, vl);
                }
                else
                {
                    newEdge = MakeEdge(cPrev.Reversed(), aNewFace, vf, vl);
                }

                newEdge.Orientation(prevEdge.Orientation());
                oldNewE.ChangeFromKey(prevEdge) = newEdge;
            }
        }

        Wire newWire = new();
        BrepBuilder builder = new();

        if (isModif)
        {
            int i;
            int nbEdges = oldNewE.Extent();

            builder.MakeWire(ref newWire);

            for (i = 1; i <= nbEdges; i++)
            {
                Shape key = oldNewE.FindKey(i);
                Shape newEdge = oldNewE.FindFromIndex(i);

                builder.Add(newWire, newEdge);
                _modifShapes.Add(key, newEdge);
            }

            if (_isClosed[_currentContour])
            {
                newWire.Closed(true);
            }

            // The orientation of the wire is already taken into account.
            _modifShapes.Add(spine, newWire);
        }
        else
        {
            newWire = spine;
        }

        builder.Add(aNewFace, newWire);
    }

    /// <summary>Clears the contents.</summary>
    public void Clear()
    {
        _curves.Clear();
        _currentContour = 0;
        _isClosed.Clear();
        _modifShapes.Clear();
    }

    private void NewContour()
    {
        NCollection_Sequence<Curve2D> contour = new();
        _curves.Append(contour);
        _isClosed.Append(false);
        _currentContour++;
    }

    // Adds aCurve to the current contour.
    private void Add(Curve2D aCurve) => _curves.ChangeValue(_currentContour).Append(aCurve);

    /// <summary>The number of contours.</summary>
    public int NumberOfContours() => _curves.Length();

    /// <summary>The number of curves of the contour IndexContour.</summary>
    public int NumberOfCurves(int indexContour) => _curves.Value(indexContour).Length();

    /// <summary>Starts an iteration over the curves of the contour IndexContour.</summary>
    public void Init(int indexContour)
    {
        _currentContour = indexContour;
        _current = 1;
    }

    /// <summary>False when there are no more curves on the contour.</summary>
    public bool More() => _current <= NumberOfCurves(_currentContour);

    /// <summary>Moves to the next curve of the contour.</summary>
    public void Next() => _current++;

    /// <summary>The current curve of the contour.</summary>
    public Curve2D Value() => _curves.Value(_currentContour).Value(_current);

    /// <summary>The face read.</summary>
    public Shape Shape() => _shape;

    /// <summary>The curves of the contour IC.</summary>
    public NCollection_Sequence<Curve2D> Contour(int ic) => _curves.Value(ic);

    /// <summary>True if aShape (the face, a wire or an edge read) was replaced.</summary>
    public bool IsModified(in Shape aShape)
    {
        if (_modifShapes.Contains(aShape))
        {
            Shape newShape = _modifShapes.FindFromKey(aShape);
            bool isSame = newShape.IsSame(aShape);

            return !isSame;
        }

        return false;
    }

    /// <summary>The shape that replaces aShape, or aShape itself if it isn't replaced.</summary>
    public Shape ModifiedShape(in Shape aShape)
    {
        if (_modifShapes.Contains(aShape))
        {
            Shape newShape = _modifShapes.FindFromKey(aShape);

            return newShape;
        }

        return aShape;
    }

    /// <summary>For each contour, whether its wire is closed.</summary>
    public NCollection_Sequence<bool> GetIsClosed() => _isClosed;

    // An edge from a 2D curve on a face and two vertices.
    private static Edge MakeEdge(Curve2D curve, in Face face, in Vertex vFirst,
        in Vertex vLast)
    {
        Edge newEdge = new();
        BrepBuilder builder = new();
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        double fPar = curve.FirstParameter();
        double lPar = curve.LastParameter();

        builder.MakeEdge(ref newEdge);
        builder.UpdateEdge(newEdge, curve, face, tol);
        builder.Add(newEdge, vFirst.Oriented(Forward));
        builder.Add(newEdge, vLast.Oriented(Reversed));
        builder.Range(newEdge, fPar, lPar);

        return newEdge;
    }

    private static CurveKind GetCurveType(Curve2D c2d)
    {
        CurveKind typeCurve;
        Type theType = c2d.GetType();

        if (theType == typeof(TrimmedCurve2D))
        {
            theType = ((TrimmedCurve2D)c2d).ChangeBasisCurve().GetType();
        }

        if (theType == typeof(CircleCurve2D))
        {
            typeCurve = Circle;
        }
        else if (theType == typeof(LineCurve2D))
        {
            typeCurve = Line;
        }
        else if (theType == typeof(EllipseCurve2D))
        {
            typeCurve = Ellipse;
        }
        else if (theType == typeof(ParabolaCurve2D))
        {
            typeCurve = Parabola;
        }
        else if (theType == typeof(HyperbolaCurve2D))
        {
            typeCurve = Hyperbola;
        }
        else if (theType == typeof(BezierCurve2D))
        {
            typeCurve = BezierCurve;
        }
        else if (theType == typeof(BSplineCurve2D))
        {
            typeCurve = BSplineCurve;
        }
        else if (theType == typeof(OffsetCurve2D))
        {
            typeCurve = OffsetCurve;
        }
        else
        {
            typeCurve = OtherCurve;
        }

        return typeCurve;
    }

    private static TrimmedCurve2D AdjustCurveEnd(BoundedCurve2D c2d, Point2D p, bool isFirst)
    {
        CurveKind type = GetCurveType(c2d);

        if (type == Line)
        {
            // A new line.
            if (isFirst)
            {
                Point2D aP = c2d.Value(c2d.LastParameter());
                return new MakeSegment2d(p, aP).Value();
            }
            else
            {
                Point2D aP = c2d.Value(c2d.FirstParameter());
                return new MakeSegment2d(aP, p).Value();
            }
        }
        else
        {
            // Converted to a B-spline whose end pole is moved.
            BSplineCurve2D bCurve = Geom2dConvert2D.CurveToBSplineCurve(c2d, QuasiAngular);

            if (isFirst)
            {
                bCurve.SetPole(1, p);
                return new TrimmedCurve2D(bCurve, bCurve.FirstParameter(), bCurve.LastParameter());
            }
            else
            {
                bCurve.SetPole(bCurve.PoleCount, p);
                return new TrimmedCurve2D(bCurve, bCurve.FirstParameter(), bCurve.LastParameter());
            }
        }
    }
}
