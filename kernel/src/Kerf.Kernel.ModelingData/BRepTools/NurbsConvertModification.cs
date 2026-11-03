// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_NurbsConvertModification.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_NurbsConvertModification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Meshing;

using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Tools;

/// <summary>
/// The modification that converts the geometry of a shape to NURBS: surfaces to B-spline surfaces (trimmed to the
/// face's UV bounds), curves to B-spline curves reparametrised to the edge's range, and pcurves to B-splines on the
/// new surfaces (projected again where the parametrisation changes). Meshes are kept, their parameters mapped onto
/// the new geometry.
/// </summary>
/// <remarks>
/// The pcurves are recomputed with ProjLib_ComputeApprox (on planes) and ProjLib_ComputeApproxOnPolarSurface. The maps
/// of converted geometry compare handles by reference, as OCCT's do.
/// </remarks>
public class NurbsConvertModification : CopyModification
{
    private readonly NCollection_List<Shape> _myled = new();
    private readonly NCollection_List<Transient?> _mylcu = new();
    private readonly IndexedMap<object, object> _map = new(ReferenceEqualityComparer.Instance);
    private readonly NCollection_List<Shape> _updatedEdges = new();

    /// <summary>A NURBS conversion.</summary>
    public NurbsConvertModification()
    {
    }

    private static void GeomLib_ChangeUBounds(BSplineSurface aSurface, double newU1, double newU2)
    {
        NCollection_Array1<double> knots = new(aSurface.InternalUKnots());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(newU1, newU2, knots);
        aSurface.SetUKnots(knots);
    }

    private static void GeomLib_ChangeVBounds(BSplineSurface aSurface, double newV1, double newV2)
    {
        NCollection_Array1<double> knots = new(aSurface.InternalVKnots());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(newV1, newV2, knots);
        aSurface.SetVKnots(knots);
    }

    // find 3D curve from theEdge in theMap, and return the transformed curve or NULL
    private static Curve3D? NewCurveCore(IndexedMap<object, object> map,
                                        in Edge edge,
                                        out double first,
                                        out double last)
    {
        Curve3D? newCurve = null;

        Curve3D? curve = Tool.Curve(edge, out Placement loc, out first, out last);
        if (curve is not null && map.ContainsKey(curve))
        {
            newCurve = (Curve3D)map.GetValue(curve);
            newCurve = (Curve3D)newCurve.Transformed(loc.Transformation);
        }
        return newCurve;
    }

    // find 2D curve from theEdge on theFace in theMap, and return the transformed curve or NULL
    private static Curve2D? NewCurveCore(IndexedMap<object, object> map,
                                          in Edge edge,
                                          in Face face,
                                          out double first,
                                          out double last)
    {
        Curve2D? c2d = Tool.CurveOnSurface(edge, face, out first, out last);
        return (c2d is not null && map.ContainsKey(c2d))
                 ? (Curve2D)map.GetValue(c2d)
                 : null;
    }

    // find surface from theFace in theMap, and return the transformed surface or NULL
    private static Surface? NewSurfaceCore(IndexedMap<object, object> map, in Face face)
    {
        Surface? newSurf = null;

        Surface? surf = Tool.Surface(face, out Placement loc);
        if (surf is not null && map.ContainsKey(surf))
        {
            newSurf = (Surface)map.GetValue(surf);
            newSurf = (Surface)newSurf.Transformed(loc.Transformation);
        }
        return newSurf;
    }

    private static bool NewParameterCore(in Point3D point,
                                     Curve3D curve,
                                     double first,
                                     double last,
                                     double tol,
                                     ref double param)
    {
        CurveAdaptor adaptor = new CurveAdaptor(curve);
        LocateExtPC proj = new LocateExtPC(point,
                                                           adaptor,
                                                           param,
                                                           first,
                                                           last,
                                                           global::Kerf.Engine.Foundation.Precision.PConfusion());
        if (proj.IsDone)
        {
            double dist2Min = proj.SquareDistance();
            if (dist2Min < tol * tol)
            {
                param = proj.Point().Parameter();
                return true;
            }
        }
        return false;
    }

    private static bool NewParameterCore(in Point2D uv,
                                     Curve2D curve2d,
                                     double first,
                                     double last,
                                     double tol,
                                     ref double param)
    {
        CurveAdaptor2D adaptor = new CurveAdaptor2D(curve2d);
        LocateExtPC2d proj = new LocateExtPC2d(uv, adaptor, param, global::Kerf.Engine.Foundation.Precision.PConfusion());
        if (proj.IsDone)
        {
            double dist2Min = proj.SquareDistance();
            if (dist2Min < tol * tol)
            {
                param = proj.Point().Parameter();
                return true;
            }
        }
        else
        {
            // Try to use general extrema to find the parameter, because Extrema_LocateExtPC2d
            // sometimes could not find a solution if the parameter's first approach is several
            // spans away from the expected solution (test bugs/modalg_7/bug28722).
            ExtPC2d ext = new ExtPC2d(uv, adaptor, first, last);
            if (ext.IsDone)
            {
                int minInd = 0;
                double minSqDist = global::Kerf.Engine.Foundation.Precision.Infinite();
                for (int index = 1; index <= ext.ExtCount; ++index)
                {
                    if (ext.SquareDistance(index) < minSqDist)
                    {
                        minSqDist = ext.SquareDistance(index);
                        minInd = index;
                    }
                }
                if (minSqDist < tol * tol)
                {
                    param = ext.Point(minInd).Parameter();
                    return true;
                }
            }
        }
        return false;
    }

    private static bool NewUV(in Point3D point, Surface surf, double tol, ref Point2D uv)
    {
        SurfaceAdaptor adaptor = new SurfaceAdaptor(surf);
        GenLocateExtPS proj = new GenLocateExtPS(adaptor);
        proj.Perform(point, uv.X, uv.Y);
        if (proj.IsDone)
        {
            double dist2Min = proj.SquareDistance();
            if (dist2Min < tol * tol)
            {
                proj.Point().Parameter(out double u, out double v);
                uv = uv.WithCoord(u, v);
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// The face's surface converted to a B-spline surface (trimmed to the face's UV bounds where they are inside the
    /// surface's); false if the surface is already a B-spline or Bezier surface, or the face has none.
    /// </summary>
    public override bool NewSurface(in Face f,
                                    out Surface? s,
                                    out Placement l,
                                    out double tol,
                                    out bool revWires,
                                    out bool revFace)
    {
        double u1, u2, curvU1, curvU2, surfU1, surfU2, uTol;
        double v1, v2, curvV1, curvV2, surfV1, surfV2, vTol;
        s = null;
        tol = 0.0;
        revWires = false;
        revFace = false;
        Surface? ss = Tool.Surface(f, out l);
        if (ss is null)
        {
            // processing the case when there is no geometry
            return false;
        }

        Type theTypeSS = ss.GetType();
        if ((theTypeSS == typeof(BSplineSurface))
            || (theTypeSS == typeof(BezierSurface)))
        {
            return false;
        }
        s = ss;
        BrepTools.UVBounds(f, out curvU1, out curvU2, out curvV1, out curvV2);
        tol = Tool.Tolerance(f);
        double tolPar = 0.1 * tol;
        bool isUp = s.IsUPeriodic, isVp = s.IsVPeriodic;
        // OCC466(apo)->
        u1 = curvU1;
        u2 = curvU2;
        v1 = curvV1;
        v2 = curvV2;
        s.ParameterBounds(out surfU1, out surfU2, out surfV1, out surfV2);

        if (Math.Abs(u1 - surfU1) <= tolPar)
        {
            u1 = surfU1;
        }
        if (Math.Abs(u2 - surfU2) <= tolPar)
        {
            u2 = surfU2;
        }
        if (Math.Abs(v1 - surfV1) <= tolPar)
        {
            v1 = surfV1;
        }
        if (Math.Abs(v2 - surfV2) <= tolPar)
        {
            v2 = surfV2;
        }

        if (!isUp)
        {
            u1 = Max(surfU1, curvU1);
            u2 = Min(surfU2, curvU2);
        }
        if (!isVp)
        {
            v1 = Max(surfV1, curvV1);
            v2 = Min(surfV2, curvV2);
        }
        //<-OCC466(apo)

        if (isUp)
        {
            double up = s.UPeriod;
            if (u2 - u1 > up)
            {
                u2 = u1 + up;
            }
        }
        if (isVp)
        {
            double vp = s.VPeriod;
            if (v2 - v1 > vp)
            {
                v2 = v1 + vp;
            }
        }

        if (Math.Abs(surfU1 - u1) > tol || Math.Abs(surfU2 - u2) > tol || Math.Abs(surfV1 - v1) > tol
            || Math.Abs(surfV2 - v2) > tol)
        {
            s = new TrimmedSurface(s, u1, u2, v1, v2);
        }
        s.ParameterBounds(out surfU1, out surfU2, out surfV1, out surfV2);

        s = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(s);
        BSplineSurface bs = (BSplineSurface)s;
        bs.Resolution(tol, out uTol, out vTol);

        //
        // Readjust the bounds of S, otherwise the old PCurves become invalid
        //

        if (Math.Abs(curvU1 - surfU1) > uTol && !bs.IsUPeriodic)
        {
            GeomLib_ChangeUBounds(bs, u1, u2);
        }
        if (Math.Abs(curvV1 - surfV1) > vTol && !bs.IsVPeriodic)
        {
            GeomLib_ChangeVBounds(bs, v1, v2);
        }

        if (!_map.ContainsKey(ss))
        {
            _map.Add(ss, s);
        }
        return true;
    }

    private static bool IsConvert(in Edge e)
    {
        bool isConvert = false;
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        // iterate on pcurves
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());
        for (; itcr.More() && !isConvert; itcr.Next())
        {
            GCurve? gc = itcr.Value() as GCurve;
            if (gc is null || !gc.IsCurveOnSurface())
            {
                continue;
            }
            Surface surface = gc.Surface()!;
            Curve2D curve2d = gc.PCurve()!;
            isConvert = ((surface is not BSplineSurface
                          && surface is not BezierSurface)
                         || (curve2d is not BSplineCurve2D
                             && curve2d is not BezierCurve2D));
        }
        return isConvert;
    }

    /// <summary>The face's triangulation, its UV nodes projected on the new surface (see the copy modification for when).</summary>
    public override bool NewTriangulation(in Face face, out Triangulation? tri)
    {
        if (!base.NewTriangulation(face, out tri))
        {
            return false;
        }

        // convert UV nodes of the mesh
        if (tri!.HasUVNodes)
        {
            Surface? surf = Tool.Surface(face, out _);
            Surface? newSurf = NewSurfaceCore(_map, face);
            if (surf is not null && newSurf is not null)
            {
                double tol = Tool.Tolerance(face);
                for (int ind = 1; ind <= tri.NodeCount; ++ind)
                {
                    Point2D uv = tri.UVNode(ind);
                    Point3D point = surf.Value(uv.X, uv.Y);
                    if (NewUV(point, newSurf, tol, ref uv))
                    {
                        tri.SetUVNode(ind, uv);
                    }
                }
            }
        }

        return true;
    }

    /// <summary>
    /// The edge's 3D curve converted to a B-spline reparametrised to the edge's range (a copy of a B-spline or Bezier
    /// curve if one of the edge's pcurves or surfaces needs converting); none for a degenerated edge.
    /// </summary>
    public override bool NewCurve(in Edge e, out Curve3D? c, out Placement L, out double tol)
    {
        c = null;
        tol = Tool.Tolerance(e);
        if (Tool.Degenerated(e))
        {
            c = null;
            L = new Placement();
            return true;
        }

        Curve3D? caux = Tool.Curve(e, out L, out double f, out double l);

        if (caux is null)
        {
            L = new Placement();
            return false;
        }
        Type theType = caux.GetType();
        if ((theType == typeof(BSplineCurve3D)) || (theType == typeof(BezierCurve3D)))
        {
            if (IsConvert(e))
            {
                c = (Curve3D)caux.Copy();
                return true;
            }
            return false;
        }

        c = caux;

        double tolPar = tol * .1;

        if (c.IsPeriodic)
        {
            double p = c.Period;
            double d = Math.Abs(l - f);
            if (Math.Abs(d - p) <= tolPar && l <= p)
            {
            }
            else
            {
                c = new TrimmedCurve3D(c, f, l);
            }
        }
        else
        {
            c = new TrimmedCurve3D(c, f, l);
        }

        // modification for WOK++ HP porting (fbi 14/03/97)
        //   gp_Trsf trsf(L);
        //   gp_Trsf trsf = L.Transformation();

        //  C = GeomConvert::CurveToBSplineCurve(C,Convert_QuasiAngular);

        c = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(c);

        double fnew = c.FirstParameter, lnew = c.LastParameter, uTol;

        BSplineCurve3D bc = (BSplineCurve3D)c;

        if (!bc.IsPeriodic)
        {
            bc.Resolution(tol, out uTol);
            if (Math.Abs(f - fnew) > uTol || Math.Abs(l - lnew) > uTol)
            {
                NCollection_Array1<double> knots = new(bc.InternalKnots());
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(f, l, knots);
                bc.SetKnots(knots);
            }
        }

        if (!_map.ContainsKey(caux))
        {
            _map.Add(caux, c);
        }
        return true;
    }

    /// <summary>The edge's 3D polygon, its parameters projected on the new curve (see the copy modification for when).</summary>
    public override bool NewPolygon(in Edge edge, out Polygon3D? poly)
    {
        if (!base.NewPolygon(edge, out poly))
        {
            return false;
        }

        if (!poly!.HasParameters)
        {
            return false;
        }
        // update parameters of polygon
        double tol = Tool.Tolerance(edge);
        Curve3D? curve = Tool.Curve(edge, out double first, out double last);
        Curve3D? newCurve = NewCurveCore(_map, edge, out first, out last);
        if (curve is null || newCurve is null) // skip processing degenerated edges
        {
            return false;
        }
        NCollection_Array1<double> aParams = poly.ChangeParameters();
        for (int ind = aParams.Lower(); ind <= aParams.Upper(); ++ind)
        {
            ref double param = ref aParams[ind];
            Point3D point = curve.Value(param);
            NewParameterCore(point, newCurve, first, last, tol, ref param);
        }
        return true;
    }

    /// <summary>Vertices keep their points: always false.</summary>
    public override bool NewPoint(in Vertex v, out Point3D p, out double tol)
    {
        p = new Point3D();
        tol = 0.0;
        return false;
    }

    /// <summary>
    /// The edge's pcurve on the face as a B-spline on the new surface, the edge's tolerance raised to the new
    /// deviation (recorded in <see cref="GetUpdatedEdges"/>).
    /// </summary>
    public override bool NewCurve2d(in Edge e,
                                    in Face f,
                                    in Edge newE,
                                    in Face newF,
                                    out Curve2D? curve2d,
                                    out double tol)
    {
        curve2d = null;
        tol = Tool.Tolerance(e);
        Curve2D? baseC2d = Tool.CurveOnSurface(e, f, out double f2d, out double l2d);
        Curve3D? c3d = Tool.Curve(e, out Placement loc, out double f3d, out double l3d);
        bool isConvert2d = ((c3d is not null && c3d is not BSplineCurve3D
                             && c3d is not BezierCurve3D)
                            || IsConvert(e));

        Curve2D c2d = baseC2d!;
        if (Tool.Degenerated(e))
        {
            // Curve2d = C2d;
            if (c2d is not TrimmedCurve2D)
            {
                TrimmedCurve2D trimC = new TrimmedCurve2D(c2d, f2d, l2d);
                c2d = trimC;
            }
            curve2d = Geom2dConvert2D.CurveToBSplineCurve(c2d);
            _map.Add(baseC2d!, curve2d);
            return true;
        }
        if (!BrepTools.IsReallyClosed(e, f))
        {
            Type theTypeC2d = c2d.GetType();

            if (theTypeC2d == typeof(TrimmedCurve2D))
            {
                TrimmedCurve2D tc = (TrimmedCurve2D)c2d;
                c2d = tc.BasisCurve;
            }

            double fc = c2d.FirstParameter, lc = c2d.LastParameter;

            if (!c2d.IsPeriodic)
            {
                if (fc - f2d > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    f2d = fc;
                }
                if (l2d - lc > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    l2d = lc;
                }
            }

            c2d = new TrimmedCurve2D(c2d, f2d, l2d);

            CurveAdaptor2D g2dAC = new CurveAdaptor2D(c2d, f2d, l2d);
            CurveAdaptor2D g2dAHC = (CurveAdaptor2D)g2dAC.ShallowCopy();

            if (!newE.IsNull)
            {
                c3d = Tool.Curve(newE, out f3d, out l3d);
            }
            if (c3d is null)
            {
                c3d = Tool.Curve(e, out f3d, out l3d);
            }
            if (c3d is null)
            {
                return false;
            }
            CurveAdaptor g3dAC = new CurveAdaptor(c3d, f3d, l3d);
            CurveAdaptor g3dAHC = (CurveAdaptor)g3dAC.ShallowCopy();

            double uinf, usup, vinf, vsup, u = 0, v = 0;
            Surface S = Tool.Surface(f)!;
            Type myT = S.GetType();
            if (myT != typeof(PlaneSurface))
            {
                if (newF.IsNull)
                {
                    Type st = c2d.GetType();
                    if ((st == typeof(BSplineCurve2D)) || (st == typeof(BezierCurve2D)))
                    {
                        if (isConvert2d)
                        {
                            curve2d = (Curve2D)c2d.Copy();
                            double newTol = BrepTools.EvalAndUpdateTol(newE, c3d, curve2d, S, f3d, l3d);
                            if (newTol > tol)
                            {
                                tol = newTol;
                                _updatedEdges.Append(newE);
                            }
                            _map.Add(baseC2d!, curve2d);
                            return true;
                        }
                        return false;
                    }
                }
                else
                {
                    Surface? newS = Tool.Surface(newF);
                    if (newS is not null)
                    {
                        S = newS;
                    }
                }
                S.ParameterBounds(out uinf, out usup, out vinf, out vsup);
                // Uinf -= 1e-9; Usup += 1e-9; Vinf -= 1e-9; Vsup += 1e-9;
                u = (usup - uinf) * 0.1;
                v = (vsup - vinf) * 0.1;
                if (S.IsUPeriodic)
                {
                    double uperiod = S.UPeriod;
                    if (uperiod < (usup + 2 * u - uinf))
                    {
                        if (uperiod <= (usup - uinf))
                        {
                            u = 0;
                        }
                        else
                        {
                            u = (uperiod - (usup - uinf)) * 0.5;
                        }
                    }
                }
                if (S.IsVPeriodic)
                {
                    double vperiod = S.VPeriod;
                    if (vperiod < (vsup + 2 * v - vinf))
                    {
                        if (vperiod <= (vsup - vinf))
                        {
                            v = 0;
                        }
                        else
                        {
                            v = (vperiod - (vsup - vinf)) * 0.5;
                        }
                    }
                }
            }
            else
            {
                S = Tool.Surface(f)!; // If S is a plane, no change of parametrization
                SurfaceAdaptor gahs = new SurfaceAdaptor(S);
                ComputeApprox projOnCurve = new ComputeApprox(g3dAHC, gahs, tol);
                if (projOnCurve.InternalBSpline() is null)
                {
                    curve2d = Geom2dConvert2D.CurveToBSplineCurve(projOnCurve.InternalBezier()!);
                    double newTol = BrepTools.EvalAndUpdateTol(newE, c3d, curve2d, S, f3d, l3d);
                    if (newTol > tol)
                    {
                        tol = newTol;
                        _updatedEdges.Append(newE);
                    }
                    _map.Add(baseC2d!, curve2d);
                    return true;
                }
                curve2d = projOnCurve.InternalBSpline();
                {
                    double newTol = BrepTools.EvalAndUpdateTol(newE, c3d, curve2d!, S, f3d, l3d);
                    if (newTol > tol)
                    {
                        tol = newTol;
                        _updatedEdges.Append(newE);
                    }
                }
                _map.Add(baseC2d!, curve2d!);
                return true;
            }

            //
            {
                SurfaceAdaptor gahs = new SurfaceAdaptor(S, uinf - u, usup + u, vinf - v, vsup + v);

                ComputeApproxOnPolarSurface projOnCurve =
                    new ComputeApproxOnPolarSurface(g2dAHC, g3dAHC, gahs, tol);

                if (projOnCurve.IsDone)
                {
                    curve2d = projOnCurve.InternalBSpline()!;
                    if (S.IsUPeriodic || S.IsVPeriodic)
                    {
                        // Surface is periodic, checking curve2d domain
                        // Old domain
                        double minDist = global::Kerf.Engine.Foundation.Precision.Infinite();
                        if (S.IsUPeriodic)
                        {
                            minDist = Min(0.5 * S.UPeriod, minDist);
                        }
                        if (S.IsVPeriodic)
                        {
                            minDist = Min(0.5 * S.VPeriod, minDist);
                        }
                        minDist *= minDist;
                        // Old domain
                        Point2D pf = c2d.Value(f2d);
                        // New domain
                        Point2D newPf = curve2d.Value(f2d);
                        Vector2D t = new Vector2D(newPf, pf);
                        if (t.SquareMagnitude > minDist)
                        {
                            curve2d = (Curve2D)curve2d.Translated(t);
                        }
                    }
                    double newTol = BrepTools.EvalAndUpdateTol(newE, c3d, curve2d, S, f3d, l3d);
                    if (newTol > tol)
                    {
                        tol = newTol;
                        _updatedEdges.Append(newE);
                    }
                    _map.Add(baseC2d!, curve2d);
                    return true;
                }
                else
                {
                    curve2d = Geom2dConvert2D.CurveToBSplineCurve(c2d);
                    double newTol = BrepTools.EvalAndUpdateTol(newE, c3d, curve2d, S, f3d, l3d);
                    if (newTol > tol)
                    {
                        tol = newTol;
                        _updatedEdges.Append(newE);
                    }
                    _map.Add(baseC2d!, curve2d);
                    return true;
                }
            }
        }
        else
        {
            var itled = new NCollection_List<Shape>.Iterator();
            var itlcu = new NCollection_List<Transient?>.Iterator();

            for (itled.Initialize(_myled), itlcu.Initialize(_mylcu); itled.More(); // itlcu.More()
                 itled.Next(), itlcu.Next())
            {
                if (itled.Value().IsSame(e))
                {
                    // already processed
                    break;
                }
            }
            if (!itled.More())
            { // store the edge and the 2D curve
                Curve2D? c2dBis;
                c2d = new TrimmedCurve2D(c2d, f2d, l2d);
                CurveAdaptor2D g2dAHC = new CurveAdaptor2D(c2d, f2d, l2d);
                Edge eRevers = e;
                eRevers.Reverse();
                c2dBis = Tool.CurveOnSurface(eRevers, f, out double f2dBis, out double l2dBis);
                c2dBis = new TrimmedCurve2D(c2dBis!, f2dBis, l2dBis);
                CurveAdaptor2D g2dAHCBis = new CurveAdaptor2D(c2dBis, f2dBis, l2dBis);

                if (c3d is null)
                {
                    if (isConvert2d)
                    {
                        curve2d = (Curve2D)c2d.Copy();

                        // OCCT.NET: OCCT's S, renamed: C# can't shadow the S declared below
                        Surface? aS;
                        if (newF.IsNull)
                        {
                            aS = Tool.Surface(f);
                        }
                        else
                        {
                            aS = Tool.Surface(newF);
                        }
                        //
                        // OCCT: EvalAndUpdateTol dereferences the null 3D curve here (C3d->IsPeriodic())
                        double newTol = BrepTools.EvalAndUpdateTol(newE, c3d!, curve2d, aS!, f3d, l3d);
                        if (newTol > tol)
                        {
                            tol = newTol;
                            _updatedEdges.Append(newE);
                        }
                        _map.Add(baseC2d!, curve2d);
                        return true;
                    }
                    return false;
                }
                if (!newE.IsNull)
                {
                    c3d = Tool.Curve(newE, out f3d, out l3d);
                }
                CurveAdaptor g3dAHC = new CurveAdaptor(c3d!, f3d, l3d);

                Surface s = Tool.Surface(f)!;
                if (newF.IsNull)
                {
                    _mylcu.Append(c2dBis);
                    Type st = c2d.GetType();
                    if ((st == typeof(BSplineCurve2D)) || (st == typeof(BezierCurve2D)))
                    {
                        if (isConvert2d)
                        {
                            curve2d = (Curve2D)c2d.Copy();
                            double newTol = BrepTools.EvalAndUpdateTol(newE, c3d!, curve2d, s, f3d, l3d);
                            if (newTol > tol)
                            {
                                tol = newTol;
                                _updatedEdges.Append(newE);
                            }
                            _map.Add(baseC2d!, curve2d);
                            return true;
                        }
                        return false;
                    }
                }
                else
                {
                    s = Tool.Surface(newF)!; // S is a BSplineSurface: no need for the trimmed one
                }
                double uinf, usup, vinf, vsup, u = 0, v = 0;
                s.ParameterBounds(out uinf, out usup, out vinf, out vsup);
                // Uinf -= 1e-9; Usup += 1e-9; Vinf -= 1e-9; Vsup += 1e-9;
                u = (usup - uinf) * 0.1;
                v = (vsup - vinf) * 0.1;
                if (s.IsUPeriodic)
                {
                    double uperiod = s.UPeriod;
                    if (uperiod < (usup + 2 * u - uinf))
                    {
                        if (uperiod <= (usup - uinf))
                        {
                            u = 0;
                        }
                        else
                        {
                            u = (uperiod - (usup - uinf)) * 0.5;
                        }
                    }
                }
                if (s.IsVPeriodic)
                {
                    double vperiod = s.VPeriod;
                    if (vperiod < (vsup + 2 * v - vinf))
                    {
                        if (vperiod <= (vsup - vinf))
                        {
                            v = 0;
                        }
                        else
                        {
                            v = (vperiod - (vsup - vinf)) * 0.5;
                        }
                    }
                }
                SurfaceAdaptor gahs = new SurfaceAdaptor(s, uinf - u, usup + u, vinf - v, vsup + v);
                _myled.Append(e);

                ComputeApproxOnPolarSurface projOnCurve =
                    new ComputeApproxOnPolarSurface(g2dAHC, g2dAHCBis, g3dAHC, gahs, tol);

                if (projOnCurve.IsDone)
                {
                    curve2d = projOnCurve.InternalBSpline()!;
                    _mylcu.Append(projOnCurve.InternalCurve2d());
                    double newTol = BrepTools.EvalAndUpdateTol(newE, c3d!, curve2d, s, f3d, l3d);
                    if (newTol > tol)
                    {
                        tol = newTol;
                        _updatedEdges.Append(newE);
                    }
                    _map.Add(baseC2d!, curve2d);
                    return true;
                }
                else
                {
                    curve2d = Geom2dConvert2D.CurveToBSplineCurve(c2d);
                    double newTol = BrepTools.EvalAndUpdateTol(newE, c3d!, curve2d, s, f3d, l3d);
                    if (newTol > tol)
                    {
                        tol = newTol;
                        _updatedEdges.Append(newE);
                    }
                    _mylcu.Append(c2dBis);
                    _map.Add(baseC2d!, curve2d);
                    return true;
                }
            }
            else
            { // we are on the second pass
                c2d = (Curve2D)itlcu.Value()!;
                Type st = c2d.GetType();
                if (!(st == typeof(BSplineCurve2D)) && !(st == typeof(BezierCurve2D)))
                {
                    return false;
                }
                curve2d = Geom2dConvert2D.CurveToBSplineCurve(c2d);
                _map.Add(baseC2d!, curve2d);
                return true;
            }
        }
    }

    /// <summary>The edge's polygon on the face's triangulation, its parameters projected on the new pcurve.</summary>
    public override bool NewPolygonOnTriangulation(in Edge edge, in Face face, out PolygonOnTriangulation? poly)
    {
        if (!base.NewPolygonOnTriangulation(edge, face, out poly))
        {
            return false;
        }

        // update parameters of 2D polygon
        if (poly!.HasParameters)
        {
            double tol = Max(Tool.Tolerance(edge), poly.Deflection());
            Surface? surf = Tool.Surface(face, out _);
            Surface? newSurf = NewSurfaceCore(_map, face);
            Curve2D? c2d = Tool.CurveOnSurface(edge, face, out double first, out double last);
            Curve2D? newC2d = NewCurveCore(_map, edge, face, out first, out last);
            if (surf is not null && c2d is not null && newSurf is not null && newC2d is not null)
            {
                // compute 2D tolerance
                SurfaceAdaptor surfAdapt = new SurfaceAdaptor(surf);
                double tol2D = Max(surfAdapt.UResolution(tol), surfAdapt.VResolution(tol));

                for (int ind = 1; ind <= poly.NodeCount; ++ind)
                {
                    double param = poly.Parameter(ind);
                    Point2D uv = c2d.Value(param);
                    Point3D point = surf.Value(uv.X, uv.Y);
                    if (NewUV(point, newSurf, tol, ref uv)
                        && NewParameterCore(uv, newC2d, first, last, tol2D, ref param))
                    {
                        poly.SetParameter(ind, param);
                    }
                }
            }
        }
        return true;
    }

    /// <summary>The vertex's parameter projected on the edge's new curve; false on a degenerated edge or without one.</summary>
    public override bool NewParameter(in Vertex v, in Edge e, ref double p, ref double tol)
    {
        tol = Tool.Tolerance(v);
        if (Tool.Degenerated(e))
        {
            return false;
        }

        Point3D pnt = Tool.Pnt(v);
        p = Tool.Parameter(v, e);
        Curve3D? newCurve = NewCurveCore(_map, e, out double first, out double last);
        return newCurve is not null && NewParameterCore(pnt, newCurve, first, last, tol, ref p);
    }

    /// <summary>The edge's continuity between the original faces.</summary>
    public override Continuity Continuity(in Edge e,
                                             in Face f1,
                                             in Face f2,
                                             in Edge newE,
                                             in Face newF1,
                                             in Face newF2)
    {
        return Tool.Continuity(e, f1, f2);
    }

    /// <summary>
    /// The new edges whose tolerance the conversion raised. Internal: they are the modifier's unfrozen edges, which
    /// BRepBuilderAPI_NurbsConvert goes on updating.
    /// </summary>
    internal NCollection_List<Shape> GetUpdatedEdges() => _updatedEdges;
}
