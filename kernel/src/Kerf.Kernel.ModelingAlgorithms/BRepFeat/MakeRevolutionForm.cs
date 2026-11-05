// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevolutionForm.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevolutionForm.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevolutionForm.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Features.PerfSelection;
using static Kerf.Engine.Features.StatusError;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Features;

/// <summary>
/// Builds a rib or a groove along a surface of revolution: a contour (an open wire) in a plane through the axis,
/// revolved and thickened on both sides of the plane until it meets the basis shape.
/// </summary>
/// <remarks>
/// <para>The mode is 0 for a groove (a cut), 1 for a rib (a fusion). With sliding, the rib slides on the faces of
/// revolution it meets.</para>
/// <para>The basis shape and the contour are read, never written (LocOpe_RevolutionForm sweeps a copy of a frozen
/// profile, decision 1014; the gluing and the same-parameter pass, BRepFeat_RibSlot's).</para>
/// </remarks>
internal sealed class MakeRevolutionForm : RibSlot
{
    private Axis3D _axe = new();
    private double _height1;
    private double _height2;
    // OCCT: the class declares its own mySliding, hiding BRepFeat_RibSlot's, which stays false
    private new bool _sliding;
    private PlaneSurface? _pln;
    private double _bnd;
    private Dictionary<Shape, NCollection_List<Shape>> _slface = new(ShapeHasher.Instance);
    private readonly NCollection_List<Shape> _listOfEdges = new();
    private double _tol;
    private double _angle1;
    private double _angle2;

    /// <summary>An empty revolution form.</summary>
    public MakeRevolutionForm()
    {
        _height1 = 0.0;
        _height2 = 0.0;
        _sliding = false;
        _bnd = 0.0;
        _tol = 0.0;
        _angle1 = RealLast();
        _angle2 = RealLast();
    }

    /// <summary>
    /// A rib or groove of the contour <paramref name="w"/> in the plane <paramref name="plane"/> on
    /// <paramref name="sbase"/>, revolved around <paramref name="axis"/>, of the thicknesses <paramref name="height1"/>
    /// and <paramref name="height2"/> on each side of the plane; <paramref name="fuse"/> 0 cuts, 1 fuses;
    /// <paramref name="sliding"/> asks for sliding (OCCT takes it by reference and never writes it).
    /// </summary>
    public MakeRevolutionForm(in Shape sbase, in Wire w, PlaneSurface plane, in Axis3D axis,
                                       double height1, double height2, int fuse, bool sliding)
    {
        Init(sbase, w, plane, axis, height1, height2, fuse, sliding);
    }

    /// <summary>Initializes the revolution form (see the constructor).</summary>
    public void Init(in Shape sbase, in Wire W, PlaneSurface plane, in Axis3D axis, double h1, double h2,
                     int mode, bool modify)
    {
        bool revolRib = true;
        Done();

        // modify = 0 if it is not required to make sliding
        //        = 1 if it is intended to try to make sliding
        bool sliding = modify;

        _axe = axis;
        LineCurve3D line = new(axis);
        double lineFirst, lineLast;

        CSIntersector asi = new(sbase);
        NCollection_Sequence<Curve3D?> scur = new();
        scur.Clear();
        scur.Append(line);
        asi.Perform(scur);
        if (asi.IsDone && asi.NbPoints(1) >= 2)
        {
            lineLast = asi.Point(1, asi.NbPoints(1)).Parameter();
            lineFirst = asi.Point(1, 1).Parameter();
        }
        else
        {
            lineFirst = RealFirst();
            lineLast = RealLast();
        }

        Curve2D ln2d = Projections.To2d(line, plane.Plane)!;

        Explorer exx = new();
        double Rad = RealLast();

        exx.Init(W, ShapeType.Edge);
        for (; exx.More(); exx.Next())
        {
            Edge e = ShapeCasts.Edge(exx.Current());
            Curve3D c = Tool.Curve(e, out double f, out double l)!;
            Curve2D c2d = Projections.To2d(c, plane.Plane)!;
            ExtremaCurveCurve2D extr = new(ln2d, c2d, lineFirst, lineLast, f, l);
            double L = RealLast();
            if (extr.NbExtrema() >= 1)
            {
                L = extr.LowerDistance();
            }
            Point3D p1 = c.Value(f);
            Point3D p2 = c.Value(l);
            ProjectPointOnCurve proj1 = new(p1, line);
            ProjectPointOnCurve proj2 = new(p2, line);
            if (proj1.NbPoints() < 1 || proj2.NbPoints() < 1)
            {
                _statusError = StatusError.NoProjPt;
                NotDone();
                return;
            }
            double par1 = proj1.Distance(1);
            double par2 = proj2.Distance(1);
            double Par = Min(par1, par2);
            if (Par < L)
            {
                L = Par;
            }
            if (L < Rad && L > 0.0)
            {
                Rad = L;
            }
        }

        double height = Min(h1, h2);

        if (Rad <= height)
        {
            Rad = height + 0.01 * height;
        }

        _angle1 = Math.Asin(h1 / Rad) + M_PI / 10.0;
        _angle2 = Math.Asin(h2 / Rad) + M_PI / 10.0;

        if ((_angle1 - M_PI / 2) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            _angle1 = M_PI / 2;
        }
        if ((_angle2 - M_PI / 2) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            _angle2 = M_PI / 2;
        }

        _skface.Nullify();
        _pbase.Nullify();

        _fuse = mode != 0;

        // ---Determination Tolerance : tolerance max on parameters
        _tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        exx.Init(W, ShapeType.Vertex);
        for (; exx.More(); exx.Next())
        {
            double tol = Tool.Tolerance(ShapeCasts.Vertex(exx.Current()));
            if (tol > _tol)
            {
                _tol = tol;
            }
        }

        exx.Init(sbase, ShapeType.Vertex);
        for (; exx.More(); exx.Next())
        {
            double tol = Tool.Tolerance(ShapeCasts.Vertex(exx.Current()));
            if (tol > _tol)
            {
                _tol = tol;
            }
        }

        Shape localShapeW = W.Oriented(Forward);
        _wire = ShapeCasts.Wire(localShapeW);
        _pln = plane;
        _height1 = h1;
        _height2 = h2;

        _sbase = sbase;
        _slface.Clear();
        _shape.Nullify();
        _map.Clear();
        _fShape.Nullify();
        _lShape.Nullify();

        // ---Calculate bounding box
        BrepBuilder bb = new();

        NCollection_List<Shape> theList = new();

        Shape u = new();
        u.Nullify();
        Point3D firstCorner = new(), lastCorner = new();
        double bnd = HeightMax(_sbase, u, ref firstCorner, ref lastCorner);
        _bnd = bnd;

        MakeBox bndbox = new(firstCorner, lastCorner);
        Solid bndBox = bndbox.InternalSolid();

        // ---Construction of the working plane face (section bounding box)
        LibraryMakeFace planeF = new(_pln.Plane, -6.0 * _bnd, 6.0 * _bnd, -6.0 * _bnd, 6.0 * _bnd);
        Face planeFace = ShapeCasts.Face(planeF.InternalShape());

        Common planeS = new(bndBox, planeFace);
        Explorer EXP = new();
        Shape planeSect = planeS.InternalShape();
        EXP.Init(planeSect, ShapeType.Wire);
        Wire www = ShapeCasts.Wire(EXP.Current());
        LibraryMakeFace bndface = new(_pln.Plane, www, true);
        Face bndFace = ShapeCasts.Face(bndface.InternalShape());

        // ---Find base faces of the rib
        Edge firstEdge = new(), lastEdge = new();
        Face firstFace = new(), lastFace = new();
        Vertex firstVertex = new(), lastVertex = new();

        bool onFirstFace = false;
        bool onLastFace = false;
        bool ptOnFirstEdge = false;
        bool ptOnLastEdge = false;
        Edge onFirstEdge = new(), onLastEdge = new();
        onFirstEdge.Nullify();
        onLastEdge.Nullify();

        bool data = ExtremeFaces(revolRib, _bnd, _pln, ref firstEdge, ref lastEdge, ref firstFace, ref lastFace,
                                 ref firstVertex, ref lastVertex, ref onFirstFace, ref onLastFace, ref ptOnFirstEdge,
                                 ref ptOnLastEdge, ref onFirstEdge, ref onLastEdge);

        if (!data)
        {
            _statusError = StatusError.NoExtFace;
            NotDone();
            return;
        }

        // ---Proofing Point for the side of the wire to be filled - material side
        Point3D checkPnt = CheckPoint(firstEdge, bnd / 10.0, _pln);

        // ---Control sliding valid
        // Many cases when the sliding is abandoned
        int concavite = 3; // a priori the profile is not concave

        _firstPnt = Tool.Pnt(firstVertex);
        _lastPnt = Tool.Pnt(lastVertex);

        // SliList : list of faces concerned by the rib
        NCollection_List<Shape> sliList = new();
        sliList.Append(firstFace);

        if (sliding)
        { // sliding
            Surface s = Tool.Surface(firstFace)!;
            if (s.GetType() == typeof(TrimmedSurface))
            {
                s = ((TrimmedSurface)s).BasisSurface;
            }
            if (s.GetType() != typeof(PlaneSurface) && s.GetType() != typeof(CylindricalSurface)
                && s.GetType() != typeof(ConicalSurface) && s.GetType() != typeof(ToroidalSurface))
            {
                sliding = false;
            }
        }

        if (sliding)
        { // sliding
            Surface ss = Tool.Surface(lastFace)!;
            if (ss.GetType() == typeof(TrimmedSurface))
            {
                ss = ((TrimmedSurface)ss).BasisSurface;
            }
            if (ss.GetType() != typeof(PlaneSurface) && ss.GetType() != typeof(CylindricalSurface)
                && ss.GetType() != typeof(ConicalSurface) && ss.GetType() != typeof(ToroidalSurface))
            {
                sliding = false;
            }
        }

        // Control only start and end points no control at the middle to improve
        // If make a control between Surface and segment 2 points limited
        // -> too expensive - to improve
        Circle3D firstCircle, lastCircle;
        Curve3D firstCrv, lastCrv;

        if (sliding)
        { // sliding
            ProjectPointOnCurve proj = new(_firstPnt, line);
            if (proj.NbPoints() < 1)
            {
                _statusError = StatusError.NoProjPt;
                NotDone();
                return;
            }
            double firstRayon = proj.Distance(1);
            Point3D firstCenter = proj.Point(1);

            ProjectPointOnCurve proj1 = new(_lastPnt, line);
            if (proj.NbPoints() < 1) // OCCT: tests proj again, not proj1
            {
                _statusError = StatusError.NoProjPt;
                NotDone();
                return;
            }
            double lastRayon = proj1.Distance(1);
            Point3D lastCenter = proj1.Point(1);

            Vector3D axv = new(_axe.Direction);
            Frame3D ax2 = new(firstCenter, new Direction3D(axv));
            Frame3D ax2p = new(lastCenter, new Direction3D(axv));
            Circle3D theFC = new(ax2, firstRayon);
            Circle3D theLC = new(ax2p, lastRayon);

            Point3D rFirstPnt1 = _firstPnt.Rotated(_axe, _angle1);
            Point3D rLastPnt1 = _lastPnt.Rotated(_axe, _angle1);
            Point3D rFirstPnt2 = _firstPnt.Rotated(_axe, -_angle2);
            Point3D rLastPnt2 = _lastPnt.Rotated(_axe, -_angle2);

            BrepBuilder bu = new();
            Vertex v1 = new(), v2 = new(), v3 = new(), v4 = new();
            bu.MakeVertex(ref v1, rFirstPnt2, global::Kerf.Engine.Foundation.Precision.Confusion());
            bu.MakeVertex(ref v2, rFirstPnt1, global::Kerf.Engine.Foundation.Precision.Confusion());
            bu.MakeVertex(ref v3, rLastPnt2, global::Kerf.Engine.Foundation.Precision.Confusion());
            bu.MakeVertex(ref v4, rLastPnt1, global::Kerf.Engine.Foundation.Precision.Confusion());

            LibraryMakeEdge ee1 = new(theFC, v1, v2);
            LibraryMakeEdge ee2 = new(theLC, v3, v4);

            if (sliding && !ptOnFirstEdge)
            {
                ExtCF ext1 = new(ShapeCasts.Edge(ee1.InternalShape()), firstFace);
                if (ext1.ExtCount < 1 || ext1.SquareDistance(1) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    sliding = false;
                }
            }
            if (sliding && !ptOnLastEdge)
            {
                ExtCF ext2 = new(ee2.InternalEdge(), lastFace); // ExtCF : curves and surfaces
                if (ext2.ExtCount < 1 || ext2.SquareDistance(1) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    sliding = false;
                }
            }
            if (sliding && ptOnFirstEdge)
            {
                firstCrv = Tool.Curve(onFirstEdge, out double f, out double l)!;
                if (firstCrv.GetType() != typeof(CircleCurve3D))
                {
                    sliding = false;
                }
                else
                {
                    CircleCurve3D c1 = (CircleCurve3D)firstCrv;
                    Circle3D circ = c1.Circle;
                    firstCircle = circ;
                    Axis3D circax = firstCircle.Axis;
                    if (!circax.IsCoaxial(_axe, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion()))
                    {
                        sliding = false;
                    }
                    else
                    {
                        if (Math.Abs(firstCircle.Radius - firstRayon) >= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            sliding = false;
                        }
                    }
                }
            }

            if (sliding && ptOnLastEdge)
            {
                lastCrv = Tool.Curve(onLastEdge, out double f, out double l)!;
                if (lastCrv.GetType() != typeof(CircleCurve3D))
                {
                    sliding = false;
                }
                else
                {
                    CircleCurve3D c1 = (CircleCurve3D)lastCrv;
                    Circle3D circ = c1.Circle;
                    lastCircle = circ;
                    Axis3D circax = lastCircle.Axis;
                    if (!circax.IsCoaxial(_axe, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion()))
                    {
                        sliding = false;
                    }
                    else
                    {
                        double rad = lastCircle.Radius;
                        if (Math.Abs(rad - lastRayon) >= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            sliding = false;
                        }
                    }
                }
            }
        }

        // Construct a great profile that goes till the bounding box
        // -> by tangency with first and last edge of the Wire
        // -> by normals to base faces : statistically better
        // Intersect everything to find the final profile

        // ---case of sliding : construction of the face profile
        if (sliding)
        {
            Face prof = new();
            bool profileOK;
            profileOK = SlidingProfile(ref prof, revolRib, _tol, ref concavite, _pln, bndFace, checkPnt, firstFace,
                                       lastFace, firstVertex, lastVertex, firstEdge, lastEdge);

            if (!profileOK)
            {
                _statusError = StatusError.NoFaceProf;
                NotDone();
                return;
            }

            // ---Propagation on faces of the initial shape
            // to find the faces concerned by the rib
            bool falseside = true;
            sliding = Propagate(sliList, prof, _firstPnt, _lastPnt, ref falseside);
            // Control if there is everything required to have the material at the proper side
            if (!falseside)
            {
                _statusError = StatusError.FalseSide;
                NotDone();
                return;
            }
        }

        // ---Generation of the base profile of the rib

        Wire w = new();
        bb.MakeWire(ref w);
        Edge thePreviousEdge = new();
        Vertex theFV = new();
        thePreviousEdge.Nullify();

        // counter of the number of edges to fill the map
        int counter = 1;

        // ---case of sliding
        if (sliding && !_listOfEdges.IsEmpty())
        {
            WireExplorer ex1 = new(_wire);
            for (; ex1.More(); ex1.Next())
            {
                Edge E = ex1.Current();
                if (!_lfMap.ContainsKey(E))
                {
                    NCollection_List<Shape> theTmpList = new();
                    _lfMap[E] = theTmpList;
                }
                if (E.IsSame(firstEdge))
                {
                    Curve3D cc = Tool.Curve(E, out double f, out double l)!;
                    Point3D pt;
                    if (!firstEdge.IsSame(lastEdge))
                    {
                        pt = Tool.Pnt(Exploration.LastVertex(E, true));
                    }
                    else
                    {
                        pt = _lastPnt;
                        double fpar = IntPar(cc, _firstPnt);
                        double lpar = IntPar(cc, pt);
                        if (fpar > lpar)
                        {
                            cc = cc.Reversed();
                        }
                    }
                    Edge ee1;
                    if (thePreviousEdge.IsNull)
                    {
                        LibraryMakeVertex v1 = new(_firstPnt);
                        LibraryMakeVertex v2 = new(pt);
                        LibraryMakeEdge e = new(cc, v1.InternalVertex(), v2.InternalVertex());
                        ee1 = ShapeCasts.Edge(e.InternalShape());
                    }
                    else
                    {
                        Vertex v1 = Exploration.LastVertex(thePreviousEdge, true);
                        LibraryMakeVertex v2 = new(pt);
                        LibraryMakeEdge e = new(cc, v1, v2.InternalVertex());
                        ee1 = ShapeCasts.Edge(e.InternalShape());
                    }
                    Shape localShape = ee1.Oriented(E.Orientation);
                    ee1 = ShapeCasts.Edge(localShape);
                    if (counter == 1)
                    {
                        theFV = Exploration.FirstVertex(ee1, true);
                    }
                    _lfMap.GetValue(E).Append(ee1);
                    bb.Add(w, ee1);
                    thePreviousEdge = ee1;
                    counter++;
                    ex1.Next();
                    break;
                }
            }

            // Case of several edges
            if (!firstEdge.IsSame(lastEdge))
            {
                for (; ex1.More(); ex1.Next())
                {
                    Edge E = ex1.Current();
                    if (!_lfMap.ContainsKey(E))
                    {
                        NCollection_List<Shape> thelist1 = new();
                        _lfMap[E] = thelist1;
                    }
                    theList.Append(E);
                    if (!E.IsSame(lastEdge))
                    {
                        Curve3D ccc = Tool.Curve(E, out double f, out double l)!;
                        Vertex v1, v2;
                        if (!thePreviousEdge.IsNull)
                        {
                            v1 = Exploration.LastVertex(thePreviousEdge, true);
                            v2 = Exploration.LastVertex(E, true);
                        }
                        else
                        {
                            v1 = Exploration.FirstVertex(E, true);
                            v2 = Exploration.LastVertex(E, true);
                        }
                        LibraryMakeEdge E1 = new(ccc, v1, v2);
                        Edge e11 = ShapeCasts.Edge(E1.InternalShape());
                        Shape localShape = e11.Oriented(E.Orientation);
                        e11 = ShapeCasts.Edge(localShape);
                        thePreviousEdge = e11;
                        _lfMap.GetValue(E).Append(e11);
                        bb.Add(w, e11);
                        if (counter == 1)
                        {
                            theFV = Exploration.FirstVertex(e11, true);
                        }
                        counter++;
                    }
                    else
                    {
                        Curve3D cc = Tool.Curve(E, out double f, out double l)!;
                        Point3D pf = Tool.Pnt(Exploration.FirstVertex(E, true));
                        Point3D pl = _lastPnt;
                        Edge ee;
                        if (thePreviousEdge.IsNull)
                        {
                            LibraryMakeEdge e = new(cc, pf, pl);
                            ee = ShapeCasts.Edge(e.InternalShape());
                        }
                        else
                        {
                            Vertex v1 = Exploration.LastVertex(thePreviousEdge, true);
                            LibraryMakeVertex v2 = new(pl);
                            LibraryMakeEdge e = new(cc, v1, v2.InternalVertex());
                            ee = ShapeCasts.Edge(e.InternalShape());
                        }
                        Shape localShape = ee.Oriented(E.Orientation);
                        ee = ShapeCasts.Edge(localShape);
                        bb.Add(w, ee);
                        _lfMap.GetValue(E).Append(ee);
                        if (counter == 1)
                        {
                            theFV = Exploration.FirstVertex(ee, true);
                        }
                        thePreviousEdge = ee;
                        counter++;
                        break;
                    }
                }
            }

            NCollection_List<Shape>.Iterator it = new(_listOfEdges);
            bool firstOK = false;
            bool lastOK = false;

            Point3D theLastPnt = _lastPnt;
            int sens = 0;
            Edge theEdge = new(), theLEdge = new(), theFEdge = new();
            int counter1 = counter;
            NCollection_List<Shape> newListOfEdges = new();
            newListOfEdges.Clear();
            while (!firstOK)
            {
                Edge edg = ShapeCasts.Edge(it.Value());
                Point3D fp, lp;
                Curve3D ccc = Tool.Curve(edg, out double f, out double l)!;
                TrimmedCurve3D cc = new(ccc, f, l);
                if (edg.Orientation == Reversed)
                {
                    cc.Reverse();
                }

                fp = cc.Value(cc.FirstParameter);
                lp = cc.Value(cc.LastParameter);
                double dist = fp.DistanceTo(theLastPnt);
                if (dist <= _tol)
                {
                    sens = 1;
                    lastOK = true;
                }
                else
                {
                    dist = lp.DistanceTo(theLastPnt);
                    if (dist <= _tol)
                    {
                        sens = 2;
                        lastOK = true;
                        cc.Reverse();
                    }
                }
                int firstFlag = 0;
                if (sens == 1 && lp.DistanceTo(_firstPnt) <= _tol)
                {
                    firstOK = true;
                    firstFlag = 1;
                }
                else if (sens == 2 && fp.DistanceTo(_firstPnt) <= _tol)
                {
                    firstOK = true;
                    firstFlag = 2;
                }
                if (lastOK)
                {
                    Edge eeee;
                    double fpar = cc.FirstParameter;
                    double lpar = cc.LastParameter;
                    if (!firstOK)
                    {
                        if (thePreviousEdge.IsNull)
                        {
                            LibraryMakeEdge e = new(cc, fpar, lpar);
                            eeee = ShapeCasts.Edge(e.InternalShape());
                        }
                        else
                        {
                            Vertex v1 = Exploration.LastVertex(thePreviousEdge, true);
                            LibraryMakeVertex v2 = new(cc.Value(lpar));
                            LibraryMakeEdge e = new(cc, v1, v2.InternalVertex());
                            eeee = ShapeCasts.Edge(e.InternalShape());
                        }
                    }
                    else
                    {
                        if (thePreviousEdge.IsNull)
                        {
                            LibraryMakeVertex v1 = new(cc.Value(fpar));
                            LibraryMakeEdge e = new(cc, v1.InternalVertex(), theFV);
                            eeee = ShapeCasts.Edge(e.InternalShape());
                        }
                        else
                        {
                            Vertex v1 = Exploration.LastVertex(thePreviousEdge, true);
                            LibraryMakeEdge e = new(cc, v1, theFV);
                            eeee = ShapeCasts.Edge(e.InternalShape());
                        }
                    }

                    thePreviousEdge = eeee;
                    bb.Add(w, eeee);
                    if (counter == 1)
                    {
                        theFV = Exploration.FirstVertex(eeee, true);
                    }
                    counter1++;
                    newListOfEdges.Append(edg);
                    theEdge = eeee;

                    if (dist <= _tol)
                    {
                        theFEdge = edg;
                    }
                    theLastPnt = Tool.Pnt(Exploration.LastVertex(theEdge, true));
                }
                if (firstFlag == 1)
                {
                    theLEdge = edg;
                }
                else if (firstFlag == 2)
                {
                    theLEdge = theEdge;
                }

                if (lastOK)
                {
                    it.Initialize(_listOfEdges);
                    lastOK = false;
                }
                else if (it.More())
                {
                    it.Next();
                }
                else
                {
                    sliding = false;
                    break;
                }
                sens = 0;
            }

            Dictionary<Shape, NCollection_List<Shape>> slidMap = new(ShapeHasher.Instance);
            slidMap.Clear();

            if (sliding && counter1 > counter)
            {
                Explorer ex2 = new(w, ShapeType.Edge);
                int ii = 0;
                for (; ex2.More(); ex2.Next())
                {
                    Edge E = ShapeCasts.Edge(ex2.Current());
                    ii++;
                    if (ii >= counter && ii <= counter1)
                    {
                        it.Initialize(newListOfEdges);
                        int jj = 0;
                        for (; it.More(); it.Next())
                        {
                            Edge e2 = ShapeCasts.Edge(it.Value());
                            jj++;
                            if (jj == (ii - counter + 1))
                            {
                                foreach (KeyValuePair<Shape, NCollection_List<Shape>> itm in _slface)
                                {
                                    Face fac = ShapeCasts.Face(itm.Key);
                                    NCollection_List<Shape> ledg = itm.Value;
                                    NCollection_List<Shape>.Iterator itedg = new(ledg);
                                    for (; itedg.More(); itedg.Next())
                                    {
                                        Edge e1 = ShapeCasts.Edge(itedg.Value());
                                        if (e1.IsSame(e2))
                                        {
                                            if (!slidMap.ContainsKey(fac))
                                            {
                                                NCollection_List<Shape> thelist2 = new();
                                                slidMap[fac] = thelist2;
                                            }
                                            slidMap.GetValue(fac).Append(E);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            _slface.Clear();
            _slface = slidMap;
        }

        // ---Arguments of LocOpe_LinearForm : arguments of the prism
        // sliding
        if (sliding)
        {
            Face F = new();
            bb.MakeFace(ref F, _pln, _tol);
            w.Closed(Tool.IsClosed(w));
            bb.Add(F, w);
            _skface = F;
            _pbase = _skface;
            _sUntil.Nullify();
        }

        // ---Case without sliding : construction of the face profile
        if (!sliding)
        {
            Explorer explo1 = new(bndFace, ShapeType.Wire);
            Wire WWW = ShapeCasts.Wire(explo1.Current());
            WireExplorer explo = new(WWW);
            BrepBuilder bu = new();
            Wire wiwiwi = new();
            bu.MakeWire(ref wiwiwi);
            Vertex newV1 = new(), newV2 = new(), lastV = new();
            newV1.Nullify();
            newV2.Nullify();
            lastV.Nullify();

            for (; explo.More(); explo.Next())
            {
                Edge e = ShapeCasts.Edge(explo.Current());
                Vertex v1 = Exploration.FirstVertex(e, true);
                Vertex v2 = Exploration.LastVertex(e, true);

                Curve3D ln = Tool.Curve(e, out double f, out double l)!;
                Curve2D l2d = Projections.To2d(ln, plane.Plane)!;
                CurveCurveIntersection2D intcc = new(l2d, ln2d, global::Kerf.Engine.Foundation.Precision.Confusion());
                Vertex vv = new();
                vv.Nullify();

                if (intcc.NbPoints() > 0)
                {
                    Point2D p = intcc.Point(1);
                    _pln.D0(p.X, p.Y, out Point3D point);
                    double par = IntPar(ln, point);
                    if (f <= par && l >= par)
                    {
                        bu.MakeVertex(ref vv, point, global::Kerf.Engine.Foundation.Precision.Confusion());
                    }
                }

                if (vv.IsNull && newV1.IsNull)
                {
                    continue;
                }

                if (!vv.IsNull && newV1.IsNull)
                {
                    newV1 = vv;
                    lastV = v2;
                    LibraryMakeEdge ee1 = new(newV1, lastV);
                    bu.Add(wiwiwi, ee1.InternalEdge());
                    continue;
                }

                if (vv.IsNull && !newV1.IsNull)
                {
                    LibraryMakeEdge ee1 = new(lastV, v2);
                    lastV = v2;
                    bu.Add(wiwiwi, e);
                    continue;
                }

                if (!vv.IsNull && !newV1.IsNull)
                {
                    newV2 = vv;
                    LibraryMakeEdge ee1 = new(lastV, newV2);
                    lastV = newV2;
                    bu.Add(wiwiwi, ee1.InternalEdge());
                    LibraryMakeEdge ee2 = new(lastV, newV1);
                    bu.Add(wiwiwi, ee2.InternalEdge());
                    break;
                }
            }

            wiwiwi.Closed(Tool.IsClosed(wiwiwi));
            LibraryMakeFace newbndface = new(_pln.Plane, wiwiwi, true);
            Face newBndFace = ShapeCasts.Face(newbndface.InternalShape());

            FClass2d cl = new(newBndFace, global::Kerf.Engine.Foundation.Precision.Confusion());
            global::Kerf.Engine.Geometry.ElSLib.Parameters(_pln.Plane, checkPnt, out double paru, out double parv);
            Point2D checkpnt2d = new(paru, parv);
            if (cl.Perform(checkpnt2d, true) == Out)
            {
                Cut c = new(bndFace, newBndFace);
                Explorer exp = new(c.InternalShape(), ShapeType.Wire);
                Wire curWire = ShapeCasts.Wire(exp.Current());
                LibraryMakeFace ff = new(_pln.Plane, curWire, true);
                newBndFace = ShapeCasts.Face(ff.InternalShape());
            }

            if (!global::Kerf.Engine.Booleans.Algo.BRepAlgo.IsValid(newBndFace))
            {
                _statusError = StatusError.InvShape;
                NotDone();
                return;
            }

            bndFace = newBndFace;

            Face prof = new();
            bool profileOK;
            profileOK = NoSlidingProfile(ref prof, revolRib, _tol, ref concavite, _pln, bnd, bndFace, checkPnt,
                                         firstFace, lastFace, firstVertex, lastVertex, firstEdge, lastEdge,
                                         onFirstFace, onLastFace);

            if (!profileOK)
            {
                _statusError = StatusError.NoFaceProf;
                NotDone();
                return;
            }

            // ---Propagation on the faces of the initial shape
            // to find the faces concerned by the rib
            bool falseside = true;
            Propagate(sliList, prof, _firstPnt, _lastPnt, ref falseside);
            // Control if there is everything required to have the material at the proper side
            if (!falseside)
            {
                _statusError = StatusError.FalseSide;
                NotDone();
                return;
            }

            _slface.Clear();

            NCollection_List<Shape>.Iterator it = new();
            it.Initialize(sliList);

            Shell comp = new();
            bb.MakeShell(ref comp);

            for (; it.More(); it.Next())
            {
                bb.Add(comp, it.Value());
            }
            comp.Closed(Tool.IsClosed(comp));

            _sUntil = comp;

            _skface = prof;
            _pbase = prof;
        }

        _sliding = sliding;

        Explorer exp1 = new();
        for (exp1.Init(_sbase, ShapeType.Face); exp1.More(); exp1.Next())
        {
            NCollection_List<Shape> thelist3 = new();
            _map[exp1.Current()] = thelist3;
            _map.GetValue(exp1.Current()).Append(exp1.Current());
        }
    }

    /// <summary>Adds an element of sliding: the edge <paramref name="e"/> slides on the face <paramref name="f"/>.</summary>
    /// <exception cref="ConstructionErrorException">F isn't a face of the basis shape.</exception>
    public void Add(in Edge e, in Face f)
    {
        if (_slface.Count == 0)
        {
            Explorer exp = new();
            for (exp.Init(_sbase, ShapeType.Face); exp.More(); exp.Next())
            {
                if (exp.Current().IsSame(f))
                {
                    break;
                }
            }
            if (!exp.More())
            {
                throw new ConstructionErrorException();
            }

            if (!_slface.ContainsKey(f))
            {
                NCollection_List<Shape> thelist = new();
                _slface[f] = thelist;
            }
            NCollection_List<Shape>.Iterator itl = new(_slface.GetValue(f));
            for (; itl.More(); itl.Next())
            {
                if (itl.Value().IsSame(e))
                {
                    break;
                }
            }
            if (!itl.More())
            {
                _slface.GetValue(f).Append(e);
            }
        }
    }

    /// <summary>Builds the rib or groove.</summary>
    public void Perform()
    {
        if (_sbase.IsNull || _skface.IsNull || _pbase.IsNull)
        {
            _statusError = StatusError.NotInitialized;
            NotDone();
            return;
        }

        Point3D pt = new();

        Explorer exx = new(_pbase, ShapeType.Vertex);
        for (; exx.More(); exx.Next())
        {
            Vertex vv = ShapeCasts.Vertex(exx.Current());
            if (!vv.IsNull)
            {
                pt = Tool.Pnt(vv);
                break;
            }
        }

        if (_angle2 != 0)
        {
            Transform3D t = new Transform3D().WithRotation(_axe, -_angle2);
            Transform trsf = new(t);
            trsf.Perform(_pbase, false);
            Face pbase = ShapeCasts.Face(trsf.InternalShape());
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> iter in _lfMap)
            {
                Shape e1 = iter.Value.First();
                Explorer ex1 = new(_pbase, ShapeType.Edge);
                Explorer ex2 = new(pbase, ShapeType.Edge);
                for (; ex1.More(); ex1.Next())
                {
                    if (ex1.Current().IsSame(e1))
                    {
                        _lfMap.GetValue(iter.Key).Clear();
                        _lfMap.GetValue(iter.Key).Append(ex2.Current());
                        break; // break the cycle (e1 became a dead reference)
                    }
                    ex2.Next();
                }
            }

            foreach (KeyValuePair<Shape, NCollection_List<Shape>> iter1 in _slface)
            {
                Shape e1 = iter1.Value.First();
                Explorer ex1 = new(_pbase, ShapeType.Edge);
                Explorer ex2 = new(pbase, ShapeType.Edge);
                for (; ex1.More(); ex1.Next())
                {
                    if (ex1.Current().IsSame(e1))
                    {
                        _slface.GetValue(iter1.Key).Clear();
                        _slface.GetValue(iter1.Key).Append(ex2.Current());
                        break; // break the cycle (e1 became a dead reference)
                    }
                    ex2.Next();
                }
            }
            _pbase = pbase;
            trsf.Perform(_skface, false);
            _skface = ShapeCasts.Face(trsf.InternalShape());
        }

        RevolutionForm theForm = new();
        theForm.Perform(_pbase, _axe, _angle1 + _angle2);
        Shape vraiForm = theForm.InternalShape(); // uncut  primitive

        // management of descendants
        MajMap(_pbase, theForm, _map, ref _fShape, ref _lShape);

        _gluedF.Clear();

        Plane3D pln0 = _pln!.Plane;
        LibraryMakeFace f = new(pln0);

        Vector3D vec1 = _height1 * new Vector3D(Normal(f.InternalFace(), pt));
        Vector3D vec2 = -_height2 * new Vector3D(Normal(f.InternalFace(), pt));

        Plane3D pln1 = pln0.Translated(vec1);
        Plane3D pln2 = pln0.Translated(vec2);

        LibraryMakeFace ff1 = new(pln1);
        LibraryMakeFace ff2 = new(pln2);
        Face f1 = ShapeCasts.Face(ff1.InternalShape());
        Face f2 = ShapeCasts.Face(ff2.InternalShape());
        global::Kerf.Engine.Features.BRepFeat.FaceUntil(_sbase, ref f1);
        global::Kerf.Engine.Features.BRepFeat.FaceUntil(_sbase, ref f2);

        CSIntersector asi1 = new(f1);
        CSIntersector asi2 = new(f2);

        LineCurve3D normale = new(pt, new Direction3D(vec1));
        NCollection_Sequence<Curve3D?> scur = new();
        scur.Append(normale);

        asi1.Perform(scur);
        asi2.Perform(scur);

        if (!asi1.IsDone || !asi2.IsDone || asi1.NbPoints(1) != 1 || asi2.NbPoints(1) != 1)
        {
            _statusError = StatusError.BadIntersect;
            NotDone();
            return;
        }

        Orientation ori1 = asi1.Point(1, 1).Orientation();
        Orientation ori2 = global::Kerf.Engine.Topology.TopAbs.Reverse(asi2.Point(1, 1).Orientation());
        Face FF1 = asi1.Point(1, 1).Face();
        Face FF2 = asi2.Point(1, 1).Face();

        Compound comp = new();
        BrepBuilder b = new();
        b.MakeCompound(ref comp);
        Solid s1 = global::Kerf.Engine.Features.BRepFeat.Tool(f1, FF1, ori1);
        Solid s2 = global::Kerf.Engine.Features.BRepFeat.Tool(f2, FF2, ori2);
        if (!s1.IsNull)
        {
            b.Add(comp, s1);
        }
        if (!s2.IsNull)
        {
            b.Add(comp, s2);
        }

        Cut trP = new(vraiForm, comp);
        // coupe de la nervure par deux plans parallels
        Dictionary<Shape, NCollection_List<Shape>> slidingMap = new(ShapeHasher.Instance);

        // management of descendants
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> it1 in _map)
        {
            Shape orig = it1.Key;
            if (it1.Value.IsEmpty())
            {
                continue;
            }
            Shape sh = it1.Value.First();
            exx.Init(vraiForm, ShapeType.Face);
            for (; exx.More(); exx.Next())
            {
                Face fac = ShapeCasts.Face(exx.Current());
                Explorer exx1 = new(fac, ShapeType.Wire);
                Wire thew = ShapeCasts.Wire(exx1.Current());
                if (thew.IsSame(_fShape))
                {
                    _map.GetValueRef(_fShape) = new NCollection_List<Shape>(trP.InternalModified(f2));
                    continue;
                }
                else if (thew.IsSame(_lShape))
                {
                    _map.GetValueRef(_lShape) = new NCollection_List<Shape>(trP.InternalModified(f1));
                    continue;
                }
                if (fac.IsSame(sh))
                {
                    if (!trP.IsDeleted(fac))
                    {
                        NCollection_List<Shape> desfaces = trP.InternalModified(fac);
                        if (!desfaces.IsEmpty())
                        {
                            _map.GetValue(orig).Clear();
                            _map.GetValueRef(orig) = new NCollection_List<Shape>(trP.InternalModified(fac));
                            break; // break the cycle (sh became a dead reference)
                        }
                    }
                }
            }
        }

        exx.Init(vraiForm, ShapeType.Face);
        for (; exx.More(); exx.Next())
        {
            Face fac = ShapeCasts.Face(exx.Current());
            NCollection_List<Shape> thelist = new();
            slidingMap[fac] = thelist;
            if (trP.IsDeleted(fac))
            {
            }
            else
            {
                NCollection_List<Shape> desfaces = trP.InternalModified(fac);
                if (!desfaces.IsEmpty())
                {
                    slidingMap.GetValueRef(fac) = new NCollection_List<Shape>(desfaces);
                }
                else
                {
                    slidingMap.GetValue(fac).Append(fac);
                }
            }
        }

        // gestion of faces of sliding
        SetGluedFaces(_slface, theForm, slidingMap, _gluedF);

        vraiForm = trP.InternalShape(); // primitive cut

        if (_gluedF.Count != 0)
        {
            _perfSelection = PerfSelection.NoSelection;
        }
        else
        {
            _perfSelection = PerfSelection.SelectionSh;
        }

        exx.Init(_pbase, ShapeType.Edge);
        for (; exx.More(); exx.Next())
        {
            Edge e = ShapeCasts.Edge(exx.Current());
            if (!_map.ContainsKey(e))
            {
                _statusError = StatusError.IncSlidFace;
                NotDone();
                return;
            }
        }

        _gShape = vraiForm;

        if (_gluedF.Count != 0 && !_sUntil.IsNull)
        {
            _statusError = StatusError.InvShape;
            NotDone();
            return;
        }

        LFPerform(); // topological reconstruction
    }

    /// <summary>
    /// Propagates the profile <paramref name="fac"/> over the faces of the basis shape from the first face of
    /// <paramref name="sliList"/>, adding the faces the rib meets; <paramref name="falseside"/> becomes false when the
    /// material isn't on the profile's side.
    /// </summary>
    public bool Propagate(NCollection_List<Shape> sliList, in Face fac, in Point3D firstpnt,
                          in Point3D lastpnt, ref bool falseside)
    {
        Point3D firstpoint = firstpnt;
        Point3D lastpoint = lastpnt;

        bool result = true;
        Face currentFace, saveFace;
        currentFace = ShapeCasts.Face(sliList.First());
        saveFace = currentFace;

        bool lastOK = false, firstOK = false;
        Vertex v1 = new(), v2 = new(), v3, v4, vert;

        SectionOperation sect = new(fac, currentFace, false);
        sect.Approximation(true);
        sect.Build();
        Explorer Ex = new();
        Edge e = new(), e1 = new();
        Point3D fp, lp;
        int ii = 0;
        for (Ex.Init(sect.InternalShape(), ShapeType.Edge); Ex.More(); Ex.Next())
        {
            ii++;
            if (ii == 1)
            {
                e = ShapeCasts.Edge(Ex.Current());
            }
            else if (ii > 1)
            {
                e1 = ShapeCasts.Edge(Ex.Current());
                break;
            }
        }
        if (e.IsNull)
        {
            falseside = false;
            return false;
        }
        //
        if (!e1.IsNull)
        {
            double tolV1, tolV2;
            _listOfEdges.Clear();
            NCollection_List<Shape> thelist = new();
            _slface[currentFace] = thelist;
            _slface.GetValue(currentFace).Append(e1);

            _listOfEdges.Append(e1);

            v1 = Exploration.FirstVertex(e1, true);
            v2 = Exploration.LastVertex(e1, true);

            fp = Tool.Pnt(v1);
            lp = Tool.Pnt(v2);

            tolV1 = Tool.Tolerance(v1);
            tolV2 = Tool.Tolerance(v2);

            if (fp.DistanceTo(firstpoint) <= tolV1 || fp.DistanceTo(lastpoint) <= tolV1)
            {
                firstOK = true;
            }
            if (lp.DistanceTo(firstpoint) <= tolV2 || lp.DistanceTo(lastpoint) <= tolV2)
            {
                lastOK = true;
            }

            if (lastOK && firstOK)
            {
                return result;
            }
            else
            {
                _listOfEdges.Clear();
            }
        }
        //
        if (!e1.IsNull)
        {
            _listOfEdges.Clear();
            NCollection_List<Shape> thelist1 = new();
            _slface[currentFace] = thelist1;
            _slface.GetValue(currentFace).Append(e);

            _listOfEdges.Append(e);

            _slface.GetValue(currentFace).Append(e1);

            v1 = Exploration.FirstVertex(e, true);
            v2 = Exploration.LastVertex(e, true);
            v3 = Exploration.FirstVertex(e1, true);
            v4 = Exploration.LastVertex(e1, true);
            Point3D p1, p2, p3, p4;
            p1 = Tool.Pnt(v1);
            fp = p1;
            p2 = Tool.Pnt(v2);
            lp = p2;
            p3 = Tool.Pnt(v3);
            p4 = Tool.Pnt(v4);
            if (p1.DistanceTo(firstpoint) <= Tool.Tolerance(v1))
            {
                if (p3.DistanceTo(lastpoint) <= Tool.Tolerance(v3))
                {
                    firstOK = true;
                    lastpoint = p4;
                }
                else if (p4.DistanceTo(lastpoint) <= Tool.Tolerance(v4))
                {
                    firstOK = true;
                    lastpoint = p3;
                }
                else
                {
                    e1.Nullify();
                }
            }
            else if (p1.DistanceTo(lastpoint) <= Tool.Tolerance(v1))
            {
                if (p3.DistanceTo(firstpoint) <= Tool.Tolerance(v3))
                {
                    firstOK = true;
                    firstpoint = p4;
                }
                else if (p4.DistanceTo(firstpoint) <= Tool.Tolerance(v4))
                {
                    firstOK = true;
                    firstpoint = p3;
                }
                else
                {
                    e1.Nullify();
                }
            }
            else if (p2.DistanceTo(firstpoint) <= Tool.Tolerance(v2))
            {
                if (p3.DistanceTo(lastpoint) <= Tool.Tolerance(v3))
                {
                    lastOK = true;
                    lastpoint = p4;
                }
                else if (p4.DistanceTo(lastpoint) <= Tool.Tolerance(v4))
                {
                    lastOK = true;
                    lastpoint = p3;
                }
                else
                {
                    e1.Nullify();
                }
            }
            else if (p2.DistanceTo(lastpoint) <= Tool.Tolerance(v2))
            {
                if (p3.DistanceTo(firstpoint) <= Tool.Tolerance(v3))
                {
                    lastOK = true;
                    firstpoint = p4;
                }
                else if (p4.DistanceTo(firstpoint) <= Tool.Tolerance(v4))
                {
                    lastOK = true;
                    firstpoint = p3;
                }
                else
                {
                    e1.Nullify();
                }
            }
            else
            {
                e = e1;
                e1.Nullify();
            }
        }
        if (e1.IsNull)
        {
            _listOfEdges.Clear();
            NCollection_List<Shape> thelist2 = new();
            _slface[currentFace] = thelist2;
            _slface.GetValue(currentFace).Append(e);

            _listOfEdges.Append(e);

            v1 = Exploration.FirstVertex(e, true);
            v2 = Exploration.LastVertex(e, true);

            fp = Tool.Pnt(v1);
            lp = Tool.Pnt(v2);

            if (fp.DistanceTo(firstpoint) <= Tool.Tolerance(v1) || fp.DistanceTo(lastpoint) <= Tool.Tolerance(v1))
            {
                firstOK = true;
            }
            if (lp.DistanceTo(firstpoint) <= Tool.Tolerance(v2) || lp.DistanceTo(lastpoint) <= Tool.Tolerance(v2))
            {
                lastOK = true;
            }

            if (lastOK && firstOK)
            {
                return result;
            }
        }

        IndexedMap<Shape, NCollection_List<Shape>> mapedges = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(_sbase, ShapeType.Edge, ShapeType.Face, mapedges);
        Explorer ex = new();
        Edge firstEdge = new();

        Vertex vprevious = new();
        vprevious.Nullify();
        Vertex vpreprevious = new();
        vpreprevious.Nullify();

        while (!firstOK)
        {
            // find edge connected to v1:
            Point3D pt = new();
            if (!v1.IsNull)
            {
                pt = Tool.Pnt(v1);
            }
            Point3D ptprev = new();
            if (!vprevious.IsNull)
            {
                ptprev = Tool.Pnt(vprevious);
            }
            Point3D ptpreprev = new();
            if (!vpreprevious.IsNull)
            {
                ptpreprev = Tool.Pnt(vpreprevious);
            }

            if ((!vprevious.IsNull && ptprev.DistanceTo(pt) <= _tol)
                || (!vpreprevious.IsNull && ptpreprev.DistanceTo(pt) <= _tol))
            {
                falseside = false;
                return false;
            }

            for (ex.Init(currentFace, ShapeType.Edge); ex.More(); ex.Next())
            {
                Edge curEdge = ShapeCasts.Edge(ex.Current());

                ExtPC projF = new(v1, curEdge);

                if (projF.IsDone && projF.ExtCount >= 1)
                {
                    double dist2min = RealLast();
                    int index = 0;
                    for (int sol = 1; sol <= projF.ExtCount; sol++)
                    {
                        if (projF.SquareDistance(sol) <= dist2min)
                        {
                            index = sol;
                            dist2min = projF.SquareDistance(sol);
                        }
                    }
                    if (index != 0)
                    {
                        if (dist2min <= Tool.Tolerance(curEdge) * Tool.Tolerance(curEdge))
                        {
                            firstEdge = curEdge;
                            break;
                        }
                    }
                }
            }

            NCollection_List<Shape> l = mapedges.GetValue(firstEdge);
            NCollection_List<Shape>.Iterator it = new(l);

            for (; it.More(); it.Next())
            {
                Face ff = ShapeCasts.Face(it.Value());
                if (!ff.IsSame(currentFace))
                {
                    currentFace = ff;
                    break;
                }
            }

            SectionOperation sectf = new(fac, currentFace, false);
            sectf.Approximation(true);
            sectf.Build();

            Edge edg1 = new();
            for (Ex.Init(sectf.InternalShape(), ShapeType.Edge); Ex.More(); Ex.Next())
            {
                edg1 = ShapeCasts.Edge(Ex.Current());
                Point3D ppp1 = Tool.Pnt(Exploration.FirstVertex(edg1, true));
                Point3D ppp2 = Tool.Pnt(Exploration.LastVertex(edg1, true));
                if (ppp1.DistanceTo(Tool.Pnt(v1)) <= Tool.Tolerance(v1)
                    || ppp2.DistanceTo(Tool.Pnt(v1)) <= Tool.Tolerance(v1))
                {
                    break;
                }
            }

            NCollection_List<Shape> thelist3 = new();
            _slface[currentFace] = thelist3;
            _slface.GetValue(currentFace).Append(edg1);
            _listOfEdges.Append(edg1);

            if (!edg1.IsNull)
            {
                sliList.Prepend(currentFace);
            }
            else
            {
                return false;
            }

            vert = Exploration.FirstVertex(edg1, true);
            Point3D pp = Tool.Pnt(vert);
            fp = Tool.Pnt(v1);
            double tol = Tool.Tolerance(edg1);
            double tol1 = Tool.Tolerance(v1);
            if (tol1 > tol)
            {
                tol = tol1;
            }
            double dist = pp.DistanceTo(fp);
            if (dist <= tol)
            {
                vpreprevious = vprevious;
                vprevious = v1;
                v1 = Exploration.LastVertex(edg1, true);
            }
            else
            {
                vpreprevious = vprevious;
                vprevious = v1;
                v1 = vert;
            }

            fp = Tool.Pnt(v1);

            if (fp.DistanceTo(firstpoint) <= Tool.Tolerance(v1) || fp.DistanceTo(lastpoint) <= Tool.Tolerance(v1))
            {
                firstOK = true;
            }
        }

        currentFace = saveFace;
        vprevious.Nullify();
        vpreprevious.Nullify();

        while (!lastOK)
        {
            // find edge connected to v2:
            Point3D pt = new();
            if (!v2.IsNull)
            {
                pt = Tool.Pnt(v2);
            }
            Point3D ptprev = new();
            if (!vprevious.IsNull)
            {
                ptprev = Tool.Pnt(vprevious);
            }
            Point3D ptpreprev = new();
            if (!vpreprevious.IsNull)
            {
                ptpreprev = Tool.Pnt(vpreprevious);
            }

            if ((!vprevious.IsNull && ptprev.DistanceTo(pt) <= _tol)
                || (!vpreprevious.IsNull && ptpreprev.DistanceTo(pt) <= _tol))
            {
                falseside = false;
                return false;
            }

            for (ex.Init(currentFace, ShapeType.Edge); ex.More(); ex.Next())
            {
                Edge curEdge = ShapeCasts.Edge(ex.Current());
                ExtPC projF = new(v2, curEdge);

                if (projF.IsDone && projF.ExtCount >= 1)
                {
                    double dist2min = RealLast();
                    int index = 0;
                    for (int sol = 1; sol <= projF.ExtCount; sol++)
                    {
                        if (projF.SquareDistance(sol) <= dist2min)
                        {
                            index = sol;
                            dist2min = projF.SquareDistance(sol);
                        }
                    }
                    if (index != 0)
                    {
                        if (dist2min <= Tool.Tolerance(curEdge) * Tool.Tolerance(curEdge))
                        {
                            firstEdge = curEdge;
                            break;
                        }
                    }
                }
            }

            NCollection_List<Shape> l = mapedges.GetValue(firstEdge);
            NCollection_List<Shape>.Iterator it = new(l);

            for (; it.More(); it.Next())
            {
                Face ff = ShapeCasts.Face(it.Value());
                if (!ff.IsSame(currentFace))
                {
                    currentFace = ff;
                    break;
                }
            }

            ii = 0;

            SectionOperation sectf = new(fac, currentFace, false);
            sectf.Approximation(true);
            sectf.Build();

            Edge edg2 = new();
            for (Ex.Init(sectf.InternalShape(), ShapeType.Edge); Ex.More(); Ex.Next())
            {
                edg2 = ShapeCasts.Edge(Ex.Current());
                Point3D ppp1 = Tool.Pnt(Exploration.FirstVertex(edg2, true));
                Point3D ppp2 = Tool.Pnt(Exploration.LastVertex(edg2, true));
                if (ppp1.DistanceTo(Tool.Pnt(v2)) <= Tool.Tolerance(v2)
                    || ppp2.DistanceTo(Tool.Pnt(v2)) <= Tool.Tolerance(v2))
                {
                    break;
                }
            }
            NCollection_List<Shape> thelist4 = new();
            _slface[currentFace] = thelist4;
            _slface.GetValue(currentFace).Append(edg2);
            _listOfEdges.Append(edg2);

            if (!edg2.IsNull)
            {
                sliList.Append(currentFace);
            }
            else
            {
                return false;
            }

            vert = Exploration.FirstVertex(edg2, true);
            Point3D pp = Tool.Pnt(vert);
            fp = Tool.Pnt(v2);
            if (pp.DistanceTo(fp) <= Tool.Tolerance(v2))
            {
                vpreprevious = vprevious;
                vprevious = v2;
                v2 = Exploration.LastVertex(edg2, true);
            }
            else
            {
                v2 = vert;
            }
            fp = Tool.Pnt(v2);

            if (fp.DistanceTo(firstpoint) <= Tool.Tolerance(v2) || fp.DistanceTo(lastpoint) <= Tool.Tolerance(v2))
            {
                lastOK = true;
            }
        }
        if (!e1.IsNull)
        {
            _listOfEdges.Append(e1);
        }
        return result;
    }

    private static void MajMap(in Shape b, RevolutionForm p,
                               Dictionary<Shape, NCollection_List<Shape>> map,
                               ref Shape fShape, ref Shape lShape)
    {
        Explorer exp = new(p.InternalFirstShape(), ShapeType.Wire);
        if (exp.More())
        {
            fShape = exp.Current();
            NCollection_List<Shape> thelist = new();
            map[fShape] = thelist;
            for (exp.Init(p.InternalFirstShape(), ShapeType.Face); exp.More(); exp.Next())
            {
                Shape sh = exp.Current();
                map.GetValue(fShape).Append(sh);
            }
        }

        exp.Init(p.InternalLastShape(), ShapeType.Wire);
        if (exp.More())
        {
            lShape = exp.Current();
            NCollection_List<Shape> thelist1 = new();
            map[lShape] = thelist1;
            for (exp.Init(p.InternalLastShape(), ShapeType.Face); exp.More(); exp.Next())
            {
                Shape sh = exp.Current();
                map.GetValue(lShape).Append(sh);
            }
        }

        for (exp.Init(b, ShapeType.Edge); exp.More(); exp.Next())
        {
            if (!map.ContainsKey(exp.Current()))
            {
                map[exp.Current()] = new NCollection_List<Shape>(p.InternalShapes(exp.Current()));
            }
        }
    }

    private static void SetGluedFaces(Dictionary<Shape, NCollection_List<Shape>> slmap,
                                      RevolutionForm prism,
                                      Dictionary<Shape, NCollection_List<Shape>> slidingMap,
                                      Dictionary<Shape, Shape> map)
    {
        // Slidings
        if (slmap.Count != 0)
        {
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> itm in slmap)
            {
                Face fac = ShapeCasts.Face(itm.Key);
                NCollection_List<Shape> ledg = itm.Value;
                NCollection_List<Shape>.Iterator it = new();
                for (it.Initialize(ledg); it.More(); it.Next())
                {
                    NCollection_List<Shape> gfac = prism.InternalShapes(it.Value());
                    foreach (KeyValuePair<Shape, NCollection_List<Shape>> iterm in slidingMap)
                    {
                        Face ff = ShapeCasts.Face(iterm.Key);
                        NCollection_List<Shape> lfaces = iterm.Value;
                        if (lfaces.IsEmpty())
                        {
                            continue;
                        }
                        Face fff = ShapeCasts.Face(lfaces.First());
                        if (gfac.First().IsSame(ff))
                        {
                            map[fff] = fac;
                        }
                    }
                }
            }
        }
    }
}
