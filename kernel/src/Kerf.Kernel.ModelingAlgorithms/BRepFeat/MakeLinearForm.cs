// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeLinearForm.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeLinearForm.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeLinearForm.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Features.PerfSelection;
using static Kerf.Engine.Features.StatusError;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features;

/// <summary>
/// Builds a rib or a groove along a developable, planar surface: a contour (an open wire) in a plane, thickened along
/// one or two directions until it meets the basis shape.
/// </summary>
/// <remarks>
/// <para>The mode is 0 for a groove (a cut), 1 for a rib (a fusion). With sliding (<c>Modify</c>), the rib slides on
/// the faces of the basis shape it meets.</para>
/// <para>The basis shape and the contour are read, never written. OCCT raises the tolerance of a vertex of the profile
/// it builds, which may be the contour's own, and translates the plane it is given: a frozen contour is worked on as a
/// copy (its unchanged sub-shapes come back in the result, its edges answer <see cref="MakeShape.Generated"/>),
/// a frozen plane is translated on a copy (decision 1052).</para>
/// </remarks>
public sealed class MakeLinearForm : RibSlot
{
#pragma warning disable CS0169 // declared, never used: as in OCCT
    private Curve3D? _crv;
#pragma warning restore CS0169
    private Vector3D _dir = new();
    private Vector3D _dir1 = new();
    private PlaneSurface? _pln;
    private double _bnd;
    private NCollection_DataMap<Shape, NCollection_List<Shape>> _slface = new(ShapeHasher.Instance);
    private readonly NCollection_List<Shape> _listOfEdges = new();
    private double _tol;
    // OCCT.NET: the contour's copy for write (decision 1052), ended after the first Perform
    private CopyAtEntry? _wireCopy;

    /// <summary>An empty linear form.</summary>
    public MakeLinearForm()
    {
        _bnd = 0.0;
        _tol = 0.0;
    }

    /// <summary>
    /// A rib or groove of the contour <paramref name="w"/> in the plane <paramref name="p"/> on
    /// <paramref name="sbase"/>, thickened along <paramref name="direction"/> and <paramref name="direction1"/> (null
    /// for one side); <paramref name="fuse"/> 0 cuts, 1 fuses; <paramref name="modify"/> asks for sliding.
    /// </summary>
    public MakeLinearForm(in Shape sbase, in Wire w, PlaneSurface p, in Vector3D direction,
                                   in Vector3D direction1, int fuse, bool modify)
    {
        Init(sbase, w, p, direction, direction1, fuse, modify);
    }

    /// <summary>Initializes the linear form (see the constructor).</summary>
    public void Init(in Shape sbase, in Wire W, PlaneSurface plane, in Vector3D direc, in Vector3D direc1,
                     int mode, bool modify)
    {
        bool revolRib = false;
        Done();
        _generated.Clear();

        // modify = 0 if there is no intention to make sliding
        //        = 1 if one tries to make sliding
        bool sliding = modify;
        _lfMap.Clear();

        _shape.Nullify();
        _map.Clear();
        _fShape.Nullify();
        _lShape.Nullify();
        _sbase = sbase;
        _skface.Nullify();
        _pbase.Nullify();

        _gShape.Nullify();
        _sUntil.Nullify();
        _listOfEdges.Clear();
        _slface.Clear();

        // OCCT.NET: the profile is built of the contour's vertices, one of which may get a raised tolerance: a frozen
        // contour is worked on as a copy (decision 1052)
        _wireCopy = new CopyAtEntry(W);
        Shape localShapeW = _wireCopy.Shape().Oriented(Forward);
        _wire = ShapeCasts.Wire(localShapeW);
        _dir = direc;
        _dir1 = direc1;
        _pln = plane;

        _fuse = mode != 0;

        // ---Determine Tolerance : max tolerance on parameters
        _tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        Explorer exx = new();
        exx.Init(_wire, ShapeType.Vertex);
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

        // ---Control of directions
        //    the wire should be in the rib
        Vector3D nulldir = new(0, 0, 0);
        if (!_dir1.IsEqual(nulldir, _tol, _tol))
        {
            double ang = _dir1.Angle(_dir);
            if (ang != M_PI)
            {
                _statusError = StatusError.BadDirect;
                NotDone();
                return;
            }
        }
        else
        {
            // Rib is centre in the middle of translation
            Vector3D dirTranslation = (direc + direc1) * 0.5;
            Transform3D t = new();
            t.SetTranslation(dirTranslation);
            Transform trf = new(t);
            trf.Perform(_wire);
            _wire = ShapeCasts.Wire(trf.InternalShape());
            _dir = direc - dirTranslation;
            _dir1 = direc1 - dirTranslation;
            // OCCT.NET: OCCT translates the caller's plane; a frozen one is translated on a copy (decision 974)
            if (_pln.IsFrozen)
            {
                _pln = (PlaneSurface)_pln.Copy();
            }
            _pln.Transform(t);
        }

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

        // ---Construction of the face workplane (section bounding box)
        LibraryMakeFace planeF = new(_pln.Pln(), -6.0 * _bnd, 6.0 * _bnd, -6.0 * _bnd, 6.0 * _bnd);
        Face planeFace = ShapeCasts.Face(planeF.InternalShape());

        Common planeS = new(bndBox, planeFace);
        Explorer EXP = new();
        Shape planeSect = planeS.InternalShape();
        EXP.Init(planeSect, ShapeType.Wire);
        Wire www = ShapeCasts.Wire(EXP.Current());
        LibraryMakeFace bndface = new(_pln.Pln(), www, true);
        Face bndFace = ShapeCasts.Face(bndface.InternalShape());

        // ---Find support faces of the rib
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

        // ---Proofing Point for the side of the wire to be filled - side material
        Point3D checkPnt = CheckPoint(firstEdge, bnd / 10.0, _pln);

        // ---Control sliding valuable
        // Many cases when the sliding is abandoned
        int concavite = 3; // a priori the profile is not concave

        _firstPnt = Tool.Pnt(firstVertex);
        _lastPnt = Tool.Pnt(lastVertex);

        // SliList : list of faces concerned by the rib
        NCollection_List<Shape> sliList = new();
        sliList.Append(firstFace);

        if (sliding)
        { // sliding
            sliding = false;
            Surface s = Tool.Surface(firstFace)!;
            if (s.GetType() == typeof(TrimmedSurface))
            {
                s = ((TrimmedSurface)s).BasisSurface();
            }
            if (s.GetType() == typeof(PlaneSurface) || s.GetType() == typeof(CylindricalSurface))
            {
                // if plane or cylinder : sliding is possible
                sliding = true;
            }
        }

        // Control only start and end points
        // -> no control at the middle - improve
        // Controle between Surface and segment between 2 limit points
        // is too expensive - improve
        if (sliding)
        {
            Point3D p1 = new(_firstPnt.X() + _dir.X(), _firstPnt.Y() + _dir.Y(), _firstPnt.Z() + _dir.Z());
            LibraryMakeEdge ee1 = new(_firstPnt, p1);
            ExtCF ext1 = new(ee1.InternalEdge(), firstFace);
            if (ext1.ExtCount == 1
                && ext1.SquareDistance(1) <= Tool.Tolerance(firstFace) * Tool.Tolerance(firstFace))
            {
                Point3D p2 = new(_lastPnt.X() + _dir.X(), _lastPnt.Y() + _dir.Y(), _lastPnt.Z() + _dir.Z());
                LibraryMakeEdge ee2 = new(_lastPnt, p2);
                ExtCF ext2 = new(ee2.InternalEdge(), lastFace); // ExtCF : curves and surfaces
                sliding = ext2.ExtCount == 1
                          && ext2.SquareDistance(1) <= Tool.Tolerance(lastFace) * Tool.Tolerance(lastFace);
            }
            else
            {
                sliding = false;
            }
        }

        if (!_dir1.IsEqual(nulldir, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            if (sliding)
            {
                Point3D p1 = new(_firstPnt.X() + _dir1.X(), _firstPnt.Y() + _dir1.Y(), _firstPnt.Z() + _dir1.Z());
                LibraryMakeEdge ee1 = new(_firstPnt, p1);
                ExtCF ext1 = new(ee1.InternalEdge(), firstFace);
                if (ext1.ExtCount == 1
                    && ext1.SquareDistance(1) <= Tool.Tolerance(firstFace) * Tool.Tolerance(firstFace))
                {
                    Point3D p2 = new(_lastPnt.X() + _dir1.X(), _lastPnt.Y() + _dir1.Y(), _lastPnt.Z() + _dir1.Z());
                    LibraryMakeEdge ee2 = new(_lastPnt, p2);
                    ExtCF ext2 = new(ee2.InternalEdge(), lastFace);
                    sliding = ext2.ExtCount == 1
                              && ext2.SquareDistance(1) <= Tool.Tolerance(lastFace) * Tool.Tolerance(lastFace);
                }
                else
                {
                    sliding = false;
                }
            }
        }

        // Construct a great profile that goes till the bounding box
        // -> by tangency with the first and the last edge of the Wire
        // -> by normals to the support faces : statistically better
        // Intersect everything to find the final profile

        // ---case of sliding : construction of the profile face
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

        // ---Generation of the base of the rib profile

        Wire w = new();
        bb.MakeWire(ref w);
        Edge thePreviousEdge = new();
        Vertex theFV = new();
        thePreviousEdge.Nullify();

        // calculate the number of edges to fill the map
        int counter = 1;

        // ---case of sliding
        if (sliding && !_listOfEdges.IsEmpty())
        {
            WireExplorer ex1 = new(_wire);
            for (; ex1.More(); ex1.Next())
            {
                Edge E = ex1.Current();
                if (!_lfMap.IsBound(E))
                {
                    NCollection_List<Shape> theTmpList = new();
                    _lfMap.Bind(E, theTmpList);
                }
                if (E.IsSame(firstEdge))
                {
                    Curve3D cc = Tool.Curve(E, out double f, out double l)!;
                    cc = new TrimmedCurve3D(cc, f, l);
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
                    Shape localShape = ee1.Oriented(E.Orientation());
                    ee1 = ShapeCasts.Edge(localShape);
                    if (counter == 1)
                    {
                        theFV = Exploration.FirstVertex(ee1, true);
                    }
                    _lfMap[E].Append(ee1);
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
                    if (!_lfMap.IsBound(E))
                    {
                        NCollection_List<Shape> thelist1 = new();
                        _lfMap.Bind(E, thelist1);
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
                        Shape localShape = e11.Oriented(E.Orientation());
                        e11 = ShapeCasts.Edge(localShape);
                        thePreviousEdge = e11;
                        _lfMap[E].Append(e11);
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
                        Shape localShape = ee.Oriented(E.Orientation());
                        ee = ShapeCasts.Edge(localShape);
                        bb.Add(w, ee);
                        _lfMap[E].Append(ee);
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
                if (edg.Orientation() == Reversed)
                {
                    cc.Reverse();
                }

                fp = cc.Value(cc.FirstParameter());
                lp = cc.Value(cc.LastParameter());
                double dist = fp.Distance(theLastPnt);
                if (dist <= _tol)
                {
                    sens = 1;
                    lastOK = true;
                }
                else
                {
                    dist = lp.Distance(theLastPnt);
                    if (dist <= _tol)
                    {
                        sens = 2;
                        lastOK = true;
                        cc.Reverse();
                    }
                }
                int firstFlag = 0;
                if (sens == 1 && lp.Distance(_firstPnt) <= _tol)
                {
                    firstOK = true;
                    firstFlag = 1;
                }
                else if (sens == 2 && fp.Distance(_firstPnt) <= _tol)
                {
                    firstOK = true;
                    firstFlag = 2;
                }
                if (lastOK)
                {
                    Edge eeee;
                    double fpar = cc.FirstParameter();
                    double lpar = cc.LastParameter();
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
                            bb.UpdateVertex(v1, dist);
                            LibraryMakeVertex v2 = new(cc.Value(lpar));
                            Vertex nv = v2.InternalVertex();
                            LibraryMakeEdge e = new(cc, v1, nv);
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
                    _listOfEdges.Remove(it);
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

            NCollection_DataMap<Shape, NCollection_List<Shape>> slidMap = new(ShapeHasher.Instance);
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
                                            if (!slidMap.IsBound(fac))
                                            {
                                                NCollection_List<Shape> thelist2 = new();
                                                slidMap.Bind(fac, thelist2);
                                            }
                                            slidMap[fac].Append(E);
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

        // ---Arguments of LocOpe_LinearForm : arguments of the prism sliding
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

        // ---Case without sliding : construction of the profile face
        if (!sliding)
        {
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

            // ---Propagation on faces of the initial shape
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

        Explorer exp = new();
        for (exp.Init(_sbase, ShapeType.Face); exp.More(); exp.Next())
        {
            NCollection_List<Shape> thelist3 = new();
            _map.Bind(exp.Current(), thelist3);
            _map[exp.Current()].Append(exp.Current());
        }
    }

    /// <summary>Adds an element of sliding: the edge <paramref name="e"/> slides on the face <paramref name="f"/>.</summary>
    /// <exception cref="ConstructionErrorException">F isn't a face of the basis shape.</exception>
    public void Add(in Edge e, in Face f)
    {
        if (_slface.IsEmpty())
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

            if (!_slface.IsBound(f))
            {
                NCollection_List<Shape> thelist = new();
                _slface.Bind(f, thelist);
            }
            NCollection_List<Shape>.Iterator itl = new(_slface[f]);
            for (; itl.More(); itl.Next())
            {
                if (itl.Value().IsSame(e))
                {
                    break;
                }
            }
            if (!itl.More())
            {
                _slface[f].Append(e);
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

        Vector3D nulldir = new(0, 0, 0);

        double length = _dir.Magnitude() + _dir1.Magnitude();

        _gluedF.Clear();

        if (!_sUntil.IsNull)
        {
            _perfSelection = PerfSelection.SelectionU;
        }
        else
        {
            _perfSelection = PerfSelection.NoSelection;
        }

        Direction3D dir = new(_dir);
        Vector3D v = length * new Vector3D(dir);

        LinearForm theForm = new();

        if (_dir1.IsEqual(nulldir, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            theForm.Perform(_pbase, v, _firstPnt, _lastPnt);
        }
        else
        {
            theForm.Perform(_pbase, v, _dir1, _firstPnt, _lastPnt);
        }

        Shape vraiForm = theForm.InternalShape(); // primitive of the rib

        _facesForDraft.Append(theForm.InternalFirstShape());
        _facesForDraft.Append(theForm.InternalLastShape());
        MajMap(_pbase, theForm, _map, ref _fShape, ref _lShape); // management of descendants

        Explorer exx = new(_pbase, ShapeType.Edge);
        for (; exx.More(); exx.Next())
        {
            Edge e = ShapeCasts.Edge(exx.Current());
            if (!_map.IsBound(e))
            {
                _statusError = StatusError.IncSlidFace;
                NotDone();
                return;
            }
        }

        _gShape = vraiForm;
        SetGluedFaces(_slface, theForm, _gluedF); // management of sliding faces

        if (!_gluedF.IsEmpty() && !_sUntil.IsNull)
        {
            _statusError = StatusError.InvShape;
            NotDone();
            return;
        }

        LFPerform();
        FinishTheContour();
    }

    // OCCT.NET: ends the contour's copy (decision 1052): the result and the history get the caller's sub-shapes back
    // where the construction left them unchanged
    private void FinishTheContour()
    {
        if (_wireCopy is null || !_wireCopy.IsCopied)
        {
            return;
        }
        CopyAtEntry copy = _wireCopy;
        _wireCopy = null;
        var others = new List<Shape>();
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> item in _map)
        {
            others.AddRange(item.Value);
        }
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> item in _lfMap)
        {
            others.AddRange(item.Value);
        }
        others.AddRange(_newEdges);
        others.AddRange(_tgtEdges);
        others.AddRange(_facesForDraft);
        _shape = copy.Finish(_shape, others);
        foreach (Shape key in _map.Keys.ToList())
        {
            _map[key] = Final(copy, _map[key]);
        }
        // the contour's edges answer Generated: keyed by the caller's edges
        var lfMap = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> item in _lfMap)
        {
            lfMap.Bind(copy.Original(item.Key), Final(copy, item.Value));
        }
        _lfMap = lfMap;
        _newEdges = Final(copy, _newEdges);
        _tgtEdges = Final(copy, _tgtEdges);
        _facesForDraft = Final(copy, _facesForDraft);
    }

    private static NCollection_List<Shape> Final(CopyAtEntry copy, NCollection_List<Shape> list)
    {
        NCollection_List<Shape> aList = new();
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            aList.Append(copy.Final(it.Value()));
        }
        return aList;
    }

    /// <summary>Limitation of the shape until the case of infinite faces: does nothing (as in OCCT).</summary>
    public void TransformShapeFU(int flag)
    {
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
        bool v1OK = false, v2OK = false;
        Vertex v1 = new(), v2 = new(), ve1, ve2;

        SectionOperation sect = new(fac, currentFace, false);

        sect.Approximation(true);
        sect.Build();

        Explorer Ex = new();
        Edge eb = new(), ec;
        Point3D p1 = new(), p2 = new();
        double t1 = 0.0, t2 = 0.0;
        bool c1f, c2f, c1l, c2l;

        for (Ex.Init(sect.InternalShape(), ShapeType.Edge); Ex.More(); Ex.Next())
        {
            ec = ShapeCasts.Edge(Ex.Current());
            v1 = Exploration.FirstVertex(ec, true);
            v2 = Exploration.LastVertex(ec, true);
            p1 = Tool.Pnt(v1);
            p2 = Tool.Pnt(v2);
            t1 = Tool.Tolerance(v1);
            t2 = Tool.Tolerance(v2);
            c1f = p1.Distance(firstpoint) <= t1;
            c2f = p2.Distance(firstpoint) <= t2;
            c1l = p1.Distance(lastpoint) <= t1;
            c2l = p2.Distance(lastpoint) <= t2;
            if (c1f || c2f || c1l || c2l)
            {
                eb = ec;
                if (c1f || c1l)
                {
                    v1OK = true;
                }
                if (c2f || c2l)
                {
                    v2OK = true;
                }
                if (c1f || c2f)
                {
                    firstOK = true;
                }
                if (c1l || c2l)
                {
                    lastOK = true;
                }
                break;
            }
        }

        if (eb.IsNull)
        {
            falseside = false;
            return false;
        }
        NCollection_List<Shape> thelist = new();
        _slface.Bind(currentFace, thelist);
        _slface[currentFace].Append(eb);

        _listOfEdges.Clear();
        _listOfEdges.Append(eb);

        // two points are on the same face.
        if (lastOK && firstOK)
        {
            return result;
        }

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mapedges = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(_sbase, ShapeType.Edge, ShapeType.Face, mapedges);
        Explorer ex = new();
        Edge firstEdge = new();
        BrepBuilder bb = new();

        Vertex vprevious;
        Point3D ptprev;
        double dp;

        while (!(lastOK && firstOK))
        {
            if (v1OK)
            {
                vprevious = v2;
                ptprev = p2;
            }
            else
            {
                vprevious = v1;
                ptprev = p1;
            }

            // find edge connected to v1 or v2:
            for (ex.Init(currentFace, ShapeType.Edge); ex.More(); ex.Next())
            {
                Edge rfe = ShapeCasts.Edge(ex.Current());

                ExtPC projF = new(vprevious, rfe);

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
                        if (dist2min <= Tool.Tolerance(rfe) * Tool.Tolerance(rfe))
                        {
                            firstEdge = rfe;
                            // If the edge is not perpendicular to the plane of the rib
                            // it is required to set Sliding(result) to false.
                            if (result)
                            {
                                result = false;
                                ve1 = Exploration.FirstVertex(rfe, true);
                                ve2 = Exploration.LastVertex(rfe, true);
                                ExtPF perp = new(ve1, fac);
                                if (perp.IsDone)
                                {
                                    Point3D pe1 = perp.Point(1);
                                    perp.Perform(ve2, fac);
                                    if (perp.IsDone)
                                    {
                                        Point3D pe2 = perp.Point(1);
                                        if (pe1.Distance(pe2) <= Tool.Tolerance(rfe))
                                        {
                                            result = true;
                                        }
                                    }
                                }
                            }
                            break;
                        }
                    }
                }
            }

            NCollection_List<Shape> l = mapedges.FindFromKey(firstEdge);
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
                v1 = Exploration.FirstVertex(edg1, true);
                v2 = Exploration.LastVertex(edg1, true);
                t1 = Tool.Tolerance(v1);
                t2 = Tool.Tolerance(v2);
                p1 = Tool.Pnt(v1);
                p2 = Tool.Pnt(v2);
                v1OK = p1.Distance(ptprev) <= t1;
                v2OK = p2.Distance(ptprev) <= t2;
                if (v1OK || v2OK)
                {
                    break;
                }
            }

            if (v1OK)
            {
                if (!firstOK)
                {
                    dp = p2.Distance(firstpoint);
                    if (dp <= 2 * t2)
                    {
                        firstOK = true;
                        bb.UpdateVertex(v2, dp);
                    }
                }
                if (!lastOK)
                {
                    dp = p2.Distance(lastpoint);
                    if (dp <= 2 * t2)
                    {
                        lastOK = true;
                        bb.UpdateVertex(v2, dp);
                    }
                }
            }
            else if (v2OK)
            {
                if (!firstOK)
                {
                    dp = p1.Distance(firstpoint);
                    if (dp <= 2 * t1)
                    {
                        firstOK = true;
                        bb.UpdateVertex(v1, dp);
                    }
                }
                if (!lastOK)
                {
                    dp = p1.Distance(lastpoint);
                    if (dp <= 2 * t1)
                    {
                        lastOK = true;
                        bb.UpdateVertex(v1, dp);
                    }
                }
            }
            else
            {
                // end by chaining the section
                return false;
            }
            NCollection_List<Shape> thelist1 = new();
            _slface.Bind(currentFace, thelist1);
            _slface[currentFace].Append(edg1);
            _listOfEdges.Append(edg1);
        }

        return result;
    }

    private static void MajMap(in Shape b, LinearForm p,
                               NCollection_DataMap<Shape, NCollection_List<Shape>> map,
                               ref Shape fShape, ref Shape lShape)
    {
        Explorer exp = new(p.InternalFirstShape(), ShapeType.Wire);
        if (exp.More())
        {
            fShape = exp.Current();
            NCollection_List<Shape> thelist = new();
            map.Bind(fShape, thelist);
            for (exp.Init(p.InternalFirstShape(), ShapeType.Face); exp.More(); exp.Next())
            {
                map[fShape].Append(exp.Current());
            }
        }

        exp.Init(p.InternalLastShape(), ShapeType.Wire);
        if (exp.More())
        {
            lShape = exp.Current();
            NCollection_List<Shape> thelist1 = new();
            map.Bind(lShape, thelist1);
            for (exp.Init(p.InternalLastShape(), ShapeType.Face); exp.More(); exp.Next())
            {
                map[lShape].Append(exp.Current());
            }
        }

        for (exp.Init(b, ShapeType.Edge); exp.More(); exp.Next())
        {
            if (!map.IsBound(exp.Current()))
            {
                map.Bind(exp.Current(), new NCollection_List<Shape>(p.InternalShapes(exp.Current())));
            }
        }
    }

    private static void SetGluedFaces(NCollection_DataMap<Shape, NCollection_List<Shape>> slmap,
                                      LinearForm prism,
                                      NCollection_DataMap<Shape, Shape> map)
    {
        // Slidings
        if (!slmap.IsEmpty())
        {
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> itm in slmap)
            {
                Face fac = ShapeCasts.Face(itm.Key);
                NCollection_List<Shape> ledg = itm.Value;
                NCollection_List<Shape>.Iterator it = new();
                for (it.Initialize(ledg); it.More(); it.Next())
                {
                    NCollection_List<Shape> gfac = prism.InternalShapes(it.Value());
                    map.Bind(gfac.First(), fac);
                }
            }
        }
    }
}
