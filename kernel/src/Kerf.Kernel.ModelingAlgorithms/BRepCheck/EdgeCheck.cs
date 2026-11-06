// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Edge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Engine.Healing.Checks;

/// <summary>
/// The checks of an edge: one 3D curve and a valid range (<see cref="Minimum"/>); in a face, its same-parameter and
/// same-range flags and the distance between its 3D curve and its pcurves (<see cref="InContext"/>); in a solid, that
/// it bounds two faces.
/// </summary>
internal class EdgeCheck : ResultCheck
{
    private const int Ncontrol = 23;

    private CurveRepresentation? _cref;
    private CurveAdaptorBase? _hCurve;
    private bool _gctrl;
    private bool _isExactMethod;

    /// <summary>Checks the edge on its own (<see cref="Minimum"/>).</summary>
    public EdgeCheck(in Edge e)
    {
        Init(e);
        _gctrl = true;
        _isExactMethod = false;
    }

    /// <summary>
    /// Checks the flags and the 3D representation of the edge, and chooses the reference curve: the first 3D curve,
    /// or else the first curve on a surface.
    /// </summary>
    public override void Minimum()
    {
        if (!_min)
        {
            var newList = new NCollection_List<StatusCheck>();
            NCollection_List<StatusCheck> lst = _map.SetValueRef(_shape, newList);
            _cref = null;

            // Existence et unicite d`une representation 3D
            var te = (BrepTEdge)_shape.TShape()!;
            var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());
            bool exist = false;
            bool unique = true;
            // Search for a 3D reference. If no existent one, creates it with the
            // first encountered CurveOnSurf; if multiple, chooses the first one...

            bool degenerated = te.Degenerated();
            bool sameParameter = te.SameParameter();
            bool sameRange = te.SameRange();
            if (!sameRange && sameParameter)
            {
                BRepCheckCheck.Add(lst, StatusCheck.InvalidSameParameterFlag);
            }

            while (itcr.More())
            {
                CurveRepresentation cr = itcr.Value();
                if (cr.IsCurve3D)
                {
                    if (!exist)
                    {
                        exist = true;
                    }
                    else
                    {
                        unique = false;
                    }

                    if (_cref is null && cr.Curve3D() is not null)
                    {
                        _cref = cr;
                    }
                }

                itcr.Next();
            }

            if (!exist)
            {
                BRepCheckCheck.Add(lst, StatusCheck.No3DCurve);
                // myCref est nulle
            }
            else if (!unique)
            {
                BRepCheckCheck.Add(lst, StatusCheck.Multiple3DCurve);
            }

            if (_cref is null && !degenerated)
            {
                itcr.Initialize(te.Curves());
                while (itcr.More())
                {
                    CurveRepresentation cr = itcr.Value();
                    if (cr.IsCurveOnSurface())
                    {
                        _cref = cr;
                        break;
                    }

                    itcr.Next();
                }
            }
            else if (_cref is not null && degenerated)
            {
                BRepCheckCheck.Add(lst, StatusCheck.InvalidDegeneratedFlag);
            }

            if (_cref is not null)
            {
                var gCref = (GCurve)_cref;
                double eps = global::Kerf.Engine.Foundation.Precision.PConfusion();
                gCref.Range(out double first, out double last);
                if (last <= first)
                {
                    _cref = null;
                    BRepCheckCheck.Add(lst, StatusCheck.InvalidRange);
                }
                else
                {
                    if (_cref.IsCurve3D)
                    {
                        // eap 6 Jun 2002 occ332
                        // better transform C3d instead of transforming Surf upto C3d initial location,
                        // on transformed BSpline surface 'same parameter' may seem wrong
                        Placement L = _shape.Placement * _cref.Location();
                        var c3d = (Curve3D)_cref.Curve3D()!.Transformed(L.Transformation);
                        bool isPeriodic = c3d.IsPeriodic;
                        double period = RealLast();
                        if (isPeriodic)
                        {
                            period = c3d.Period;
                        }

                        double f = c3d.FirstParameter, l = c3d.LastParameter;
                        if (c3d.GetType() == typeof(TrimmedCurve3D))
                        {
                            Curve3D c = ((TrimmedCurve3D)c3d).BasisCurve;
                            f = c.FirstParameter;
                            l = c.LastParameter;
                            isPeriodic = c.IsPeriodic;
                            if (isPeriodic)
                            {
                                period = c.Period;
                            }
                        }

                        if (isPeriodic && (last - first > period + eps))
                        {
                            _cref = null;
                            BRepCheckCheck.Add(lst, StatusCheck.InvalidRange);
                        }
                        else if (!isPeriodic && (first < f - eps || last > l + eps))
                        {
                            _cref = null;
                            BRepCheckCheck.Add(lst, StatusCheck.InvalidRange);
                        }
                        else
                        {
                            var gac3d = new CurveAdaptor(c3d,
                                                              c3d.TransformedParameter(first, L.Transformation),
                                                              c3d.TransformedParameter(last, L.Transformation));
                            _hCurve = gac3d;
                        }
                    }
                    else
                    {
                        // curve on surface
                        Surface sref = _cref.Surface()!;
                        sref = (Surface)sref.Transformed(_cref.Location().Transformation);
                        Curve2D pCref = _cref.PCurve()!;
                        bool isPeriodic = pCref.IsPeriodic;
                        double period = RealLast();
                        if (isPeriodic)
                        {
                            period = pCref.Period;
                        }

                        double f = pCref.FirstParameter, l = pCref.LastParameter;
                        if (pCref.GetType() == typeof(TrimmedCurve2D))
                        {
                            Curve2D c = ((TrimmedCurve2D)pCref).BasisCurve;
                            f = c.FirstParameter;
                            l = c.LastParameter;
                            isPeriodic = c.IsPeriodic;
                            if (isPeriodic)
                            {
                                period = c.Period;
                            }
                        }

                        if (isPeriodic && (last - first > period + eps))
                        {
                            _cref = null;
                            BRepCheckCheck.Add(lst, StatusCheck.InvalidRange);
                        }
                        else if (!isPeriodic && (first < f - eps || last > l + eps))
                        {
                            _cref = null;
                            BRepCheckCheck.Add(lst, StatusCheck.InvalidRange);
                        }
                        else
                        {
                            var gahSref = new SurfaceAdaptor(sref);
                            var ghpCref = new CurveAdaptor2D(pCref, first, last);
                            var acSref = new CurveOnSurface(ghpCref, gahSref);
                            _hCurve = acSref;
                        }
                    }
                }
            }

            if (lst.IsEmpty())
            {
                lst.Append(StatusCheck.NoError);
            }

            _min = true;
        }
    }

    /// <summary>
    /// Checks the edge in a wire (nothing), a face (flags, range and the distance between the reference curve and
    /// the pcurves on the face, or the projection on a plane without pcurve) or a solid (bounds two faces).
    /// </summary>
    public override void InContext(in Shape s)
    {
        NCollection_List<StatusCheck> hList;
        {
            bool locked = _isParallel;
            if (locked)
            {
                _mutex.Enter();
            }

            try
            {
                if (_map.ContainsKey(s))
                {
                    return;
                }

                var newList = new NCollection_List<StatusCheck>();
                hList = _map.SetValueRef(s, newList);
            }
            finally
            {
                if (locked)
                {
                    _mutex.Exit();
                }
            }
        }

        NCollection_List<StatusCheck> lst = hList;

        var te = (BrepTEdge)_shape.TShape()!;
        double tol = Tool.Tolerance(ShapeCasts.Edge(_shape));

        ShapeType styp = s.Type;
        var exp = new Explorer(s, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(_shape))
            {
                break;
            }
        }

        if (!exp.More())
        {
            BRepCheckCheck.Add(lst, StatusCheck.SubshapeNotInShape);
            return;
        }

        switch (styp)
        {
            case ShapeType.Wire:
            {
            }

                break;

            case ShapeType.Face:
                if (_cref is not null)
                {
                    bool sameParameter = te.SameParameter();
                    bool sameRange = te.SameRange();
                    //  Modified by skv - Tue Apr 27 11:48:13 2004 Begin
                    if (!sameParameter || !sameRange)
                    {
                        if (!sameParameter)
                        {
                            BRepCheckCheck.Add(lst, StatusCheck.InvalidSameParameterFlag);
                        }

                        if (!sameRange)
                        {
                            BRepCheckCheck.Add(lst, StatusCheck.InvalidSameRangeFlag);
                        }

                        return;
                    }

                    //  Modified by skv - Tue Apr 27 11:48:14 2004 End
                    // myCref is not null only when Minimum built myHCurve
                    double first = _hCurve!.FirstParameter();
                    double last = _hCurve.LastParameter();
                    // Kerf: in parallel mode the analyzer may check this edge in several faces at once, and the
                    // reference adaptor's evaluation cache isn't thread-safe; each call evaluates its own copy
                    // (OCCT shares the adaptor: occt-issues.md)
                    CurveAdaptorBase hCurve = _isParallel ? _hCurve.ShallowCopy() : _hCurve;

                    var tf = (BrepTFace)s.TShape()!;
                    Placement floc = s.Placement;
                    Placement tFloc = tf.Location();
                    Surface? su = tf.Surface();
                    Placement L = (floc * tFloc).Predivided(_shape.Placement);
                    Placement le = _shape.Placement * _cref.Location();
                    Transform3D etrsf = le.Transformation;
                    bool pcurvefound = false;

                    var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());
                    double eps = global::Kerf.Engine.Foundation.Precision.PConfusion();
                    bool toRunParallel = _isParallel;
                    while (itcr.More())
                    {
                        CurveRepresentation cr = itcr.Value();
                        if (cr != _cref && cr.IsCurveOnSurface(su, L))
                        {
                            pcurvefound = true;
                            var gc = (GCurve)cr;
                            gc.Range(out double f, out double l);
                            double ff = f, ll = l;
                            if (_cref.IsCurve3D)
                            {
                                ff = _cref.Curve3D()!.TransformedParameter(f, etrsf);
                                ll = _cref.Curve3D()!.TransformedParameter(l, etrsf);
                            }

                            // gka OCC
                            //  Modified by skv - Tue Apr 27 11:50:35 2004 Begin
                            if (Math.Abs(ff - first) > eps || Math.Abs(ll - last) > eps)
                            {
                                BRepCheckCheck.Add(lst, StatusCheck.InvalidSameRangeFlag);
                                BRepCheckCheck.Add(lst, StatusCheck.InvalidSameParameterFlag);
                            }

                            //  Modified by skv - Tue Apr 27 11:50:37 2004 End
                            //
                            Curve2D pc = cr.PCurve()!;
                            bool isPeriodic = pc.IsPeriodic;
                            double period = RealLast();
                            if (isPeriodic)
                            {
                                period = pc.Period;
                            }

                            double fp = pc.FirstParameter, lp = pc.LastParameter;
                            if (pc.GetType() == typeof(TrimmedCurve2D))
                            {
                                Curve2D c = ((TrimmedCurve2D)pc).BasisCurve;
                                fp = c.FirstParameter;
                                lp = c.LastParameter;
                                isPeriodic = c.IsPeriodic;
                                if (isPeriodic)
                                {
                                    period = c.Period;
                                }
                            }

                            if (isPeriodic && (l - f > period + eps))
                            {
                                BRepCheckCheck.Add(lst, StatusCheck.InvalidRange);
                                return;
                            }
                            else if (!isPeriodic && (f < fp - eps || l > lp + eps))
                            {
                                BRepCheckCheck.Add(lst, StatusCheck.InvalidRange);
                                return;
                            }

                            if (_gctrl)
                            {
                                var sb = (Surface)su!.Transformed((floc * tFloc).Transformation);
                                Curve2D PC = cr.PCurve()!;
                                var gahs = new SurfaceAdaptor(sb);
                                var ghpc = new CurveAdaptor2D(PC, f, l);
                                var acs = new CurveOnSurface(ghpc, gahs);

                                var validateEdge = new ValidateEdge(hCurve, acs, sameParameter);
                                validateEdge.SetExitIfToleranceExceeded(tol);
                                validateEdge.SetExactMethod(_isExactMethod);
                                validateEdge.SetParallel(toRunParallel);
                                validateEdge.Process();
                                if (!validateEdge.IsDone || !validateEdge.CheckTolerance(tol))
                                {
                                    if (cr.IsCurveOnClosedSurface)
                                    {
                                        BRepCheckCheck.Add(lst, StatusCheck.InvalidCurveOnClosedSurface);
                                    }
                                    else
                                    {
                                        BRepCheckCheck.Add(lst, StatusCheck.InvalidCurveOnSurface);
                                    }

                                    //  Modified by skv - Tue Apr 27 11:53:00 2004 Begin
                                    BRepCheckCheck.Add(lst, StatusCheck.InvalidSameParameterFlag);
                                    //  Modified by skv - Tue Apr 27 11:53:01 2004 End
                                }

                                if (cr.IsCurveOnClosedSurface)
                                {
                                    ghpc.Load(cr.PCurve2()!, f, l); // same bounds
                                    acs.Load(ghpc, gahs); // sans doute inutile

                                    var validateEdgeOnClosedSurf = new ValidateEdge(hCurve, acs, sameParameter);
                                    validateEdgeOnClosedSurf.SetExitIfToleranceExceeded(tol);
                                    validateEdgeOnClosedSurf.SetExactMethod(_isExactMethod);
                                    validateEdgeOnClosedSurf.SetParallel(toRunParallel);
                                    validateEdgeOnClosedSurf.Process();
                                    if (!validateEdgeOnClosedSurf.IsDone
                                        || !validateEdgeOnClosedSurf.CheckTolerance(tol))
                                    {
                                        BRepCheckCheck.Add(lst, StatusCheck.InvalidCurveOnClosedSurface);
                                        //  Modified by skv - Tue Apr 27 11:53:20 2004 Begin
                                        if (sameParameter)
                                        {
                                            BRepCheckCheck.Add(lst, StatusCheck.InvalidSameParameterFlag);
                                        }

                                        //  Modified by skv - Tue Apr 27 11:53:23 2004 End
                                    }
                                }
                            }
                        }

                        itcr.Next();
                    }

                    if (!pcurvefound)
                    {
                        PlaneSurface? p;
                        if (su is TrimmedSurface rts && su.GetType() == typeof(TrimmedSurface))
                        {
                            p = rts.BasisSurface as PlaneSurface;
                        }
                        else
                        {
                            p = su as PlaneSurface;
                        }

                        if (p is null)
                        {
                            // not a plane
                            BRepCheckCheck.Add(lst, StatusCheck.NoCurveOnSurface);
                        }
                        else
                        {
                            // on fait la projection a la volee, comme BRep_Tool
                            // plan en position
                            if (_gctrl)
                            {
                                p = (PlaneSurface)p.Transformed((floc * tFloc).Transformation); // eap occ332
                                // on projette Cref sur ce plan
                                var gahs = new SurfaceAdaptor(p);

                                // Dub - Normalement myHCurve est une GeomAdaptor_Curve
                                // OCCT down-casts and dereferences: a reference curve on a surface (no 3D curve)
                                // gives a null handle here, which the analyzer catches as a failure
                                var gac = hCurve as CurveAdaptor
                                          ?? throw new NullObjectException("BRepCheck_Edge::InContext - no 3D curve");
                                Curve3D c3d = gac.Curve()!;
                                Curve3D projOnPlane =
                                    global::Kerf.Engine.Geometry.Projection.GeomProjLib.ProjectOnPlane(new TrimmedCurve3D(c3d, first, last),
                                                               p,
                                                               p.Position.Direction,
                                                               true)!;
                                var projCurve = new CurveAdaptor(projOnPlane);

                                var proj = new ProjectedCurve(gahs, projCurve);
                                Curve2D PC = global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(proj);
                                var ghpc = new CurveAdaptor2D(PC, hCurve.FirstParameter(), hCurve.LastParameter());

                                var acs = new CurveOnSurface(ghpc, gahs);

                                var validateEdgeProj = new ValidateEdge(hCurve, acs, sameParameter);
                                validateEdgeProj.SetExitIfToleranceExceeded(tol);
                                validateEdgeProj.SetExactMethod(_isExactMethod);
                                validateEdgeProj.SetParallel(toRunParallel);
                                validateEdgeProj.Process();
                                if (!validateEdgeProj.IsDone || !validateEdgeProj.CheckTolerance(tol))
                                {
                                    BRepCheckCheck.Add(lst, StatusCheck.InvalidCurveOnSurface);
                                }
                            }
                        }
                    }
                }

                break;
            case ShapeType.Solid:
            {
                // on verifie que l`edge est bien connectee 2 fois (pas de bord libre)
                int nbconnection = 0;
                for (exp.Init(s, ShapeType.Face); exp.More(); exp.Next())
                {
                    Face fac = ShapeCasts.Face(exp.Current());
                    var exp2 = new Explorer();
                    for (exp2.Init(fac, ShapeType.Edge); exp2.More(); exp2.Next())
                    {
                        if (exp2.Current().IsSame(_shape))
                        {
                            nbconnection++;
                        }
                    }
                }

                if (nbconnection < 2 && !te.Degenerated())
                {
                    BRepCheckCheck.Add(lst, StatusCheck.FreeEdge);
                }
                else if (nbconnection > 2)
                {
                    BRepCheckCheck.Add(lst, StatusCheck.InvalidMultiConnexity);
                }
                else
                {
                    BRepCheckCheck.Add(lst, StatusCheck.NoError);
                }
            }

                break;
            default:
                break;
        }

        if (lst.IsEmpty())
        {
            lst.Append(StatusCheck.NoError);
        }
    }

    /// <summary>Nothing more than the minimum (OCCT removed the body).</summary>
    public override void Blind()
    {
        //  Modified by skv - Tue Apr 27 11:36:01 2004 Begin
        // The body of this function is removed because of its useless.
        if (!_blind)
        {
            _blind = true;
        }
    }

    /// <summary>Sets whether <see cref="InContext"/> measures the pcurves against the reference curve.</summary>
    public void GeometricControls(bool b) => _gctrl = b;

    /// <summary>Whether <see cref="InContext"/> measures the pcurves against the reference curve.</summary>
    public bool GeometricControls() => _gctrl;

    /// <summary>Adds <paramref name="status"/> to the statuses of the edge.</summary>
    public void SetStatus(StatusCheck status)
    {
        bool locked = _isParallel;
        if (locked)
        {
            _mutex.Enter();
        }

        try
        {
            BRepCheckCheck.Add(_map.GetValue(_shape), status);
        }
        finally
        {
            if (locked)
            {
                _mutex.Exit();
            }
        }
    }

    /// <summary>
    /// Sets the method of <see cref="ValidateEdge"/>: exact (BRepLib_CheckCurveOnSurface, slower, used only
    /// when the edge is same-parameter) or in a finite number of points (the default).
    /// </summary>
    public void SetExactMethod(bool isExact) => _isExactMethod = isExact;

    /// <summary>Whether the exact method is selected.</summary>
    public bool IsExactMethod => _isExactMethod;

    /// <summary>
    /// The largest distance between the edge's representations at 23 parameters, with a 5% margin
    /// (<see cref="Precision.Confusion"/> for an edge with one representation).
    /// </summary>
    public double Tolerance()
    {
        var te = (BrepTEdge)_shape.TShape()!;
        int it, iRep = 1, nbRep = te.Curves().Extent();
        if (nbRep <= 1)
        {
            return global::Kerf.Engine.Foundation.Precision.Confusion();
        }

        var theRep = new NCollection_Array1<CurveAdaptorBase?>(1, nbRep * 2);
        double first, last;
        if (_hCurve is not null)
        {
            first = _hCurve.FirstParameter();
            last = _hCurve.LastParameter();
        }
        else
        {
            Tool.Range(ShapeCasts.Edge(_shape), out first, out last);
        }

        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());
        for (; itcr.More(); itcr.Next())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsCurve3D && !te.Degenerated())
            {
                //// modified by jgv, 20.03.03 ////
                Placement loc = _shape.Placement * cr.Location();
                var c3d = (Curve3D)cr.Curve3D()!.Transformed(loc.Transformation);
                ///////////////////////////////////
                var gac3d = new CurveAdaptor(c3d, first, last);
                it = iRep;
                if (iRep > 1)
                {
                    theRep[iRep] = theRep[1];
                    it = 1;
                }

                theRep[it] = gac3d;
                iRep++;
            }
            else if (cr.IsCurveOnSurface())
            {
                {
                    Surface sref = cr.Surface()!;
                    //// modified by jgv, 20.03.03 ////
                    Placement loc = _shape.Placement * cr.Location();
                    sref = (Surface)sref.Transformed(loc.Transformation);
                    ///////////////////////////////////
                    Curve2D pCref = cr.PCurve()!;
                    var gahSref = new SurfaceAdaptor(sref);
                    var ghpCref = new CurveAdaptor2D(pCref, first, last);
                    theRep[iRep] = new CurveOnSurface(ghpCref, gahSref);
                    iRep++;
                }

                if (cr.IsCurveOnClosedSurface)
                {
                    Surface sref = cr.Surface()!;
                    // OCCT: the second pcurve's surface is placed without the edge's location, unlike the first
                    sref = (Surface)sref.Transformed(cr.Location().Transformation);
                    Curve2D pCref = cr.PCurve2()!;
                    var gahSref = new SurfaceAdaptor(sref);
                    var ghpCref = new CurveAdaptor2D(pCref, first, last);
                    theRep[iRep] = new CurveOnSurface(ghpCref, gahSref);
                    iRep++;
                    nbRep++;
                }
            }
            else
            {
                nbRep--;
            }
        }

        double dist2, tol2, tolCal = 0.0, prm;
        Point3D center, othP;
        int i;
        for (i = 0; i < Ncontrol; i++)
        {
            prm = ((Ncontrol - 1 - i) * first + i * last) / (Ncontrol - 1);
            tol2 = dist2 = 0.0;
            center = theRep[1]!.Value(prm);
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(center.X) || global::Kerf.Engine.Foundation.Precision.IsInfinite(center.Y)
                || global::Kerf.Engine.Foundation.Precision.IsInfinite(center.Z))
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }

            for (iRep = 2; iRep <= nbRep; iRep++)
            {
                othP = theRep[iRep]!.Value(prm);
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(othP.X) || global::Kerf.Engine.Foundation.Precision.IsInfinite(othP.Y)
                    || global::Kerf.Engine.Foundation.Precision.IsInfinite(othP.Z))
                {
                    return global::Kerf.Engine.Foundation.Precision.Infinite();
                }

                dist2 = center.SquareDistanceTo(othP);
                if (dist2 > tolCal)
                {
                    tolCal = dist2;
                }
            }

            if (tol2 > tolCal)
            {
                tolCal = tol2;
            }
        }

        // On prend 5% de marge car au dessus on crontrole severement
        return Math.Sqrt(tolCal) * 1.05;
    }

    /// <summary>
    /// Checks that the edge's polygons on triangulations lie within their deflection plus the edge's tolerance of
    /// its 3D curve.
    /// </summary>
    public StatusCheck CheckPolygonOnTriangulation(in Edge edge)
    {
        // Kerf: OCCT takes ChangeCurves() and only reads it
        NCollection_List<CurveRepresentation> listOfCR = ((BrepTEdge)edge.TShape()!).Curves();
        bool hasPolygonOnTriangulation = false;
        bool hasCurve3D = false;
        for (var itcr = new NCollection_List<CurveRepresentation>.Iterator(listOfCR);
             itcr.More();
             itcr.Next())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsPolygonOnTriangulation())
            {
                hasPolygonOnTriangulation = true;
            }

            if (cr.IsCurve3D && cr.Curve3D() is not null)
            {
                hasCurve3D = true;
            }

            if (hasPolygonOnTriangulation && hasCurve3D)
            {
                break;
            }
        }

        if (!hasPolygonOnTriangulation || !hasCurve3D)
        {
            return StatusCheck.NoError;
        }

        var bc = new EdgeCurveAdaptor();
        bc.Initialize(edge);

        if (!bc.Is3DCurve())
        {
            return StatusCheck.NoError;
        }

        var it = new NCollection_List<CurveRepresentation>.Iterator(listOfCR);

        while (it.More())
        {
            if (!it.Value().IsPolygonOnTriangulation())
            {
                it.Next();
                continue;
            }

            CurveRepresentation cr = it.Value();
            var pt = (PolygonOnTriangulationRepresentation)cr;

            Placement ll = edge.Placement * pt.Location();
            Transform3D trsf = ll.Transformation;

            Triangulation triang = cr.Triangulation()!;
            PolygonOnTriangulation pOnTriag = cr.IsPolygonOnClosedTriangulation
                ? cr.PolygonOnTriangulation2()!
                : cr.PolygonOnTriangulation()!;
            NCollection_Array1<int> indices = pOnTriag.Nodes();
            int nbNodes = indices.Length();

            double tol = pOnTriag.Deflection() + Tool.Tolerance(edge);

            if (pOnTriag.HasParameters)
            {
                NCollection_HArray1<double> aParams = pOnTriag.Parameters()!;
                for (int i = aParams.Lower(); i <= aParams.Upper(); i++)
                {
                    double param = aParams.Value(i);
                    Point3D pe = bc.Value(param);
                    Point3D pnt = triang.Node(indices[i]).Transformed(trsf);

                    double sqDist = pe.SquareDistanceTo(pnt);
                    if (sqDist > tol * tol)
                    {
                        return StatusCheck.InvalidPolygonOnTriangulation;
                    }
                }
            }
            else
            {
                // If aPOnTriag does not have any parameters we will check if it
                // inscribes into Bounding box, which is built on the edge triangulation.

                var b = new Box();

                for (int i = 1; i <= nbNodes; i++)
                {
                    if (trsf.Form == Identity)
                    {
                        b.Add(triang.Node(indices[i]));
                    }
                    else
                    {
                        b.Add(triang.Node(indices[i]).Transformed(trsf));
                    }
                }

                b.Enlarge(tol);

                double fp = bc.FirstParameter();
                double lp = bc.LastParameter();

                double step = (lp - fp) / (double)Ncontrol;
                Point3D p;
                double par = fp;

                for (int i = 1; i < Ncontrol; i++)
                {
                    bc.D0(par, out p);
                    if (b.IsOut(p))
                    {
                        return StatusCheck.InvalidPolygonOnTriangulation;
                    }

                    par += step;
                }

                bc.D0(lp, out p);
                if (b.IsOut(p))
                {
                    return StatusCheck.InvalidPolygonOnTriangulation;
                }
            }

            it.Next();
        }

        return StatusCheck.NoError;
    }
}
