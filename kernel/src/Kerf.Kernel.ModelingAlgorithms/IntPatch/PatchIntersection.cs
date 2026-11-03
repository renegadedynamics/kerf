// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Intersection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Intersection.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Intersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// The intersection of two surfaces (or the self-intersection of one) bounded by their domains: analytic pairs
/// through IntPatch_ImpImpIntersection (analytic lines converted to walking lines), a quadric and a parametric
/// surface through IntPatch_ImpPrmIntersection, parametric pairs through IntPatch_PrmPrmIntersection; then the
/// walking lines purged (IntPatch_WLineTool).
/// </summary>
/// <remarks>
/// A working object, as OCCT's: Perform fills it, and its lines are the caller's to edit (decision 609).
/// <see cref="Line"/> returns the line itself, <see cref="Point"/> a copy. The file-static functions of OCCT's
/// .cxx are private static methods (IntPatch_Intersection.Tools.cs).
/// </remarks>
public partial class PatchIntersection
{
    private bool _done;
    private bool _empt;
    private bool _tgte;
    private bool _oppo;
    private readonly NCollection_Sequence<PatchPoint> _spnt = new();
    private readonly NCollection_Sequence<PatchLine> _slin = new();
    private double _tolArc;
    private double _tolTang;
    private double _uvMaxStep;
    private double _fleche;
    private bool _isStartPnt;
    private double _u1Start;
    private double _v1Start;
    private double _u2Start;
    private double _v2Start;

    /// <summary>The lines of the intersection (the sequence itself).</summary>
    public NCollection_Sequence<PatchLine> SequenceOfLine() => _slin;

    public PatchIntersection()
    {
        _done = false;
        _empt = true;
        _tgte = false;
        _oppo = false;
        _tolArc = 0.0;
        _tolTang = 0.0;
        _uvMaxStep = 0.0;
        _fleche = 0.0;
        _isStartPnt = false;
        _u1Start = 0.0;
        _v1Start = 0.0;
        _u2Start = 0.0;
        _v2Start = 0.0;
    }

    public PatchIntersection(SurfaceAdaptorBase s1, TopolTool d1, SurfaceAdaptorBase s2,
                                 TopolTool d2, double tolArc, double tolTang)
        : this()
    {
        _tolArc = tolArc;
        _tolTang = tolTang;
        if (_tolArc < 1e-8)
        {
            _tolArc = 1e-8;
        }
        if (_tolTang < 1e-8)
        {
            _tolTang = 1e-8;
        }
        if (_tolArc > 0.5)
        {
            _tolArc = 0.5;
        }
        if (_tolTang > 0.5)
        {
            _tolTang = 0.5;
        }
        Perform(s1, d1, s2, d2, tolArc, tolTang);
    }

    public PatchIntersection(SurfaceAdaptorBase s1, TopolTool d1, double tolArc, double tolTang)
        : this()
    {
        _tolArc = tolArc;
        _tolTang = tolTang;
        Perform(s1, d1, tolArc, tolTang);
    }

    /// <summary>
    /// Sets the tolerances: TolArc and TolTang clamped to [1e-8, 0.5], Fleche (the deflection of walking lines) to
    /// [1e-3, 10], UVMaxStep (the largest step of the walking in parameters) at most 0.5.
    /// </summary>
    public void SetTolerances(double tolArc, double tolTang, double uvMaxStep, double fleche)
    {
        _tolArc = tolArc;
        _tolTang = tolTang;
        _uvMaxStep = uvMaxStep;
        _fleche = fleche;
        if (_tolArc < 1e-8)
        {
            _tolArc = 1e-8;
        }
        if (_tolTang < 1e-8)
        {
            _tolTang = 1e-8;
        }
        if (_tolArc > 0.5)
        {
            _tolArc = 0.5;
        }
        if (_tolTang > 0.5)
        {
            _tolTang = 0.5;
        }
        if (_fleche < 1.0e-3)
        {
            _fleche = 1e-3;
        }
        // if(myUVMaxStep<1.0e-3) myUVMaxStep=1e-3;
        if (_fleche > 10)
        {
            _fleche = 10;
        }
        if (_uvMaxStep > 0.5)
        {
            _uvMaxStep = 0.5;
        }
    }

    /// <summary>The self-intersection of S1 (parametric surfaces only; quadrics and extrusions of curves without
    /// self-intersection give an empty result).</summary>
    public void Perform(SurfaceAdaptorBase s1, TopolTool d1, double tolArc, double tolTang)
    {
        _tolArc = tolArc;
        _tolTang = tolTang;
        if (_fleche == 0.0)
        {
            _fleche = 0.01;
        }
        if (_uvMaxStep == 0.0)
        {
            _uvMaxStep = 0.01;
        }

        _done = true;
        _spnt.Clear();
        _slin.Clear();

        _empt = true;
        _tgte = false;
        _oppo = false;

        switch (s1.GetType())
        {
            case Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Torus:
                break;
            case SurfaceOfExtrusion:
            {
                Direction3D direction = s1.Direction();
                CoordinateSystem axis = new(Constants.Origin(), direction);
                CurveAdaptorBase basisCurve = s1.BasisCurve();
                ProjectOnPlane projector = new(axis);
                projector.Load(basisCurve, global::Kerf.Engine.Foundation.Precision.Confusion());
                CurveAdaptor projCurve = projector.GetResult()!;
                PlaneSurface plane = new(axis);
                SurfaceAdaptor gaHsurf = new(plane);
                ProjectedCurve projectedCurve = new(gaHsurf, projCurve);
                global::Kerf.Engine.Geometry.Projection.ProjLib.MakePCurveOfType(projectedCurve, out Curve2D pCurve);
                CurveAdaptor2D ac = new(pCurve, projectedCurve.FirstParameter(),
                                             projectedCurve.LastParameter());
                GInter intersector = new(ac, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion());
                if (intersector.IsDone && (intersector.IsEmpty || (ac.IsClosed && intersector.PointCount == 1)))
                {
                    break;
                }
            }
                goto default; // [[fallthrough]]
            default:
            {
                PrmPrmIntersection interpp = new();
                interpp.Perform(s1, d1, tolTang, tolArc, _fleche, _uvMaxStep);
                if (interpp.IsDone)
                {
                    _done = true;
                    _tgte = false;
                    _empt = interpp.IsEmpty;
                    int nblm = interpp.LineCount;
                    for (int i = 1; i <= nblm; i++)
                    {
                        _slin.Append(interpp.Line(i));
                    }
                }
            }
                break;
        }
    }

    //=================================================================================================

    /// <summary>
    /// Intersects S1 and S2 within their domains. isGeomInt false sends analytic pairs to the parametric
    /// intersector; theIsReqToKeepRLine keeps the restriction lines of analytic pairs; theIsReqToPostWLProc false
    /// skips the purging of the walking lines.
    /// </summary>
    public void Perform(SurfaceAdaptorBase s1, TopolTool d1, SurfaceAdaptorBase s2,
                        TopolTool d2, double tolArc, double tolTang, bool isGeomInt = true,
                        bool isReqToKeepRLine = false, bool isReqToPostWLProc = true)
    {
        _tolArc = tolArc;
        _tolTang = tolTang;
        if (_fleche <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            _fleche = 0.01;
        }
        if (_uvMaxStep <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            _uvMaxStep = 0.01;
        }

        _done = false;
        _spnt.Clear();
        _slin.Clear();
        _empt = true;
        _tgte = false;
        _oppo = false;

        SurfaceKind typs1 = s1.GetType();
        SurfaceKind typs2 = s2.GetType();

        // treatment of the cases with cone or torus
        bool treatAsBiParametric = false;
        int bGeomGeom = 0;
        //
        if (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs2 == global::Kerf.Geometry.SurfaceKind.Cone || typs1 == global::Kerf.Geometry.SurfaceKind.Torus || typs2 == global::Kerf.Geometry.SurfaceKind.Torus)
        {
            Axis3D ctAx = new(), geomAx = new();
            SurfaceKind ctType;
            bool bToCheck;
            //
            SurfaceAdaptorBase ctSurf = (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs1 == global::Kerf.Geometry.SurfaceKind.Torus) ? s1 : s2;
            SurfaceAdaptorBase geomSurf = (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs1 == global::Kerf.Geometry.SurfaceKind.Torus) ? s2 : s1;
            //
            ctType = ctSurf.GetType();
            bToCheck = false;
            //
            if (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs2 == global::Kerf.Geometry.SurfaceKind.Cone)
            {
                Cone con1 = (ctType == global::Kerf.Geometry.SurfaceKind.Cone) ? ctSurf.Cone() : geomSurf.Cone();
                double a1 = Math.Abs(con1.SemiAngle());
                bToCheck = (a1 < 0.02) || (a1 > 1.55);
                //
                if (typs1 == typs2)
                {
                    Cone con2 = geomSurf.Cone();
                    double a2 = Math.Abs(con2.SemiAngle());
                    bToCheck = bToCheck || (a2 < 0.02) || (a2 > 1.55);
                    //
                    if (a1 > 1.55 && a2 > 1.55)
                    { // quasi-planes: if same domain, treat as canonic
                        Axis3D A1 = con1.Axis(), A2 = con2.Axis();
                        if (A1.IsParallel(A2, global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Point3D apex1 = con1.Apex(), apex2 = con2.Apex();
                            Plane3D plan1 = new(apex1, A1.Direction());
                            if (plan1.Distance(apex2) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                bToCheck = false;
                            }
                        }
                    }
                }
                //
                treatAsBiParametric = bToCheck;
                if (ctType == global::Kerf.Geometry.SurfaceKind.Cone)
                {
                    ctAx = con1.Axis();
                }
            }
            //
            if (typs1 == global::Kerf.Geometry.SurfaceKind.Torus || typs2 == global::Kerf.Geometry.SurfaceKind.Torus)
            {
                Torus tor1 = (ctType == global::Kerf.Geometry.SurfaceKind.Torus) ? ctSurf.Torus() : geomSurf.Torus();
                bToCheck = tor1.MajorRadius() > tor1.MinorRadius();
                if (typs1 == typs2)
                {
                    Torus tor2 = geomSurf.Torus();
                    // OCCT: this overload ignores aTor1's test for two tori (the list-of-points Perform combines
                    // both, and also accepts equal radii); ported as written.
                    bToCheck = tor2.MajorRadius() > tor2.MinorRadius();
                }
                //
                if (ctType == global::Kerf.Geometry.SurfaceKind.Torus)
                {
                    ctAx = tor1.Axis();
                }
            }
            //
            CheckGeomGeom(ref bToCheck, ref bGeomGeom, ref treatAsBiParametric, ctAx, ref geomAx, ctType, ctSurf,
                          geomSurf);
        }
        //

        if (d1.DomainIsInfinite() || d2.DomainIsInfinite())
        {
            treatAsBiParametric = false;
        }

        //  Modified by skv - Mon Sep 26 14:58:30 2005 Begin
        //   if(TreatAsBiParametric) { typs1 = typs2 = GeomAbs_BezierSurface; }
        if (treatAsBiParametric)
        {
            if (typs1 == global::Kerf.Geometry.SurfaceKind.Cone && typs2 == Plane)
            {
                typs1 = global::Kerf.Geometry.SurfaceKind.BezierSurface; // Using Imp-Prm Intersector
            }
            else if (typs1 == Plane && typs2 == global::Kerf.Geometry.SurfaceKind.Cone)
            {
                typs2 = global::Kerf.Geometry.SurfaceKind.BezierSurface; // Using Imp-Prm Intersector
            }
            else
            {
                // Using Prm-Prm Intersector
                typs1 = global::Kerf.Geometry.SurfaceKind.BezierSurface;
                typs2 = global::Kerf.Geometry.SurfaceKind.BezierSurface;
            }
        }
        //  Modified by skv - Mon Sep 26 14:58:30 2005 End

        // Surface type definition
        int ts1 = SurfaceKind(typs1, bGeomGeom);
        int ts2 = SurfaceKind(typs2, bGeomGeom);
        //
        // treatment of the cases with torus and any other geom surface
        //
        // Possible intersection types: 1. ts1 == ts2 == 1 <Geom-Geom>
        //                              2. ts1 != ts2      <Geom-Param>
        //                              3. ts1 == ts2 == 0 <Param-Param>

        // Geom - Geom
        if (ts1 == ts2 && ts1 == 1)
        {
            NCollection_List<PntOn2S> listOfPnts = new();
            listOfPnts.Clear();
            if (isGeomInt)
            {
                GeomGeomPerfom(s1, d1, s2, d2, tolArc, tolTang, listOfPnts, typs1, typs2,
                               isReqToKeepRLine);
            }
            else
            {
                ParamParamPerfom(s1, d1, s2, d2, tolArc, tolTang, listOfPnts, typs1, typs2);
            }
        }

        // Geom - Param
        if (ts1 != ts2)
        {
            GeomParamPerfom(s1, d1, s2, d2, ts1 == 0, typs1, typs2);
        }

        // Param - Param
        if (ts1 == ts2 && ts1 == 0)
        {
            NCollection_List<PntOn2S> listOfPnts = new();
            listOfPnts.Clear();

            ParamParamPerfom(s1, d1, s2, d2, tolArc, tolTang, listOfPnts, typs1, typs2);
        }

        if (!isReqToPostWLProc)
        {
            return;
        }

        PurgeWLines(s1, d1, s2, d2);
    }

    //=================================================================================================

    /// <summary>
    /// As the Perform above, starting from the given points too (ListOfPnts, for the parametric intersector).
    /// The list may gain the points' images across the seams of closed surfaces (decision 706).
    /// </summary>
    public void Perform(SurfaceAdaptorBase s1, TopolTool d1, SurfaceAdaptorBase s2,
                        TopolTool d2, double tolArc, double tolTang,
                        NCollection_List<PntOn2S> listOfPnts, bool isGeomInt = true,
                        bool isReqToKeepRLine = false, bool isReqToPostWLProc = true)
    {
        _tolArc = tolArc;
        _tolTang = tolTang;
        if (_fleche <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            _fleche = 0.01;
        }
        if (_uvMaxStep <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            _uvMaxStep = 0.01;
        }
        _done = false;
        _spnt.Clear();
        _slin.Clear();
        _empt = true;
        _tgte = false;
        _oppo = false;

        SurfaceKind typs1 = s1.GetType();
        SurfaceKind typs2 = s2.GetType();
        //
        // treatment of the cases with cone or torus
        bool treatAsBiParametric = false;
        int bGeomGeom = 0;
        //
        if (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs2 == global::Kerf.Geometry.SurfaceKind.Cone || typs1 == global::Kerf.Geometry.SurfaceKind.Torus || typs2 == global::Kerf.Geometry.SurfaceKind.Torus)
        {
            Axis3D ctAx = new(), geomAx = new();
            SurfaceKind ctType;
            bool bToCheck;
            //
            SurfaceAdaptorBase ctSurf = (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs1 == global::Kerf.Geometry.SurfaceKind.Torus) ? s1 : s2;
            SurfaceAdaptorBase geomSurf = (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs1 == global::Kerf.Geometry.SurfaceKind.Torus) ? s2 : s1;
            //
            ctType = ctSurf.GetType();
            bToCheck = false;
            //
            if (typs1 == global::Kerf.Geometry.SurfaceKind.Cone || typs2 == global::Kerf.Geometry.SurfaceKind.Cone)
            {
                Cone con1 = (ctType == global::Kerf.Geometry.SurfaceKind.Cone) ? ctSurf.Cone() : geomSurf.Cone();
                double a1 = Math.Abs(con1.SemiAngle());
                bToCheck = (a1 < 0.02) || (a1 > 1.55);
                //
                if (typs1 == typs2)
                {
                    Cone con2 = geomSurf.Cone();
                    double a2 = Math.Abs(con2.SemiAngle());
                    bToCheck = bToCheck || (a2 < 0.02) || (a2 > 1.55);
                    //
                    if (a1 > 1.55 && a2 > 1.55)
                    { // quasi-planes: if same domain, treat as canonic
                        Axis3D A1 = con1.Axis(), A2 = con2.Axis();
                        if (A1.IsParallel(A2, global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Point3D apex1 = con1.Apex(), apex2 = con2.Apex();
                            Plane3D plan1 = new(apex1, A1.Direction());
                            if (plan1.Distance(apex2) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                bToCheck = false;
                            }
                        }
                    }
                }
                //
                treatAsBiParametric = bToCheck;
                if (ctType == global::Kerf.Geometry.SurfaceKind.Cone)
                {
                    ctAx = con1.Axis();
                }
            }
            //
            if (typs1 == global::Kerf.Geometry.SurfaceKind.Torus || typs2 == global::Kerf.Geometry.SurfaceKind.Torus)
            {
                Torus tor1 = (ctType == global::Kerf.Geometry.SurfaceKind.Torus) ? ctSurf.Torus() : geomSurf.Torus();
                bToCheck = tor1.MajorRadius() > tor1.MinorRadius();
                if (typs1 == typs2)
                {
                    Torus tor2 = geomSurf.Torus();
                    bToCheck = (bToCheck && (tor2.MajorRadius() > tor2.MinorRadius()))
                               || (Math.Abs(tor1.MajorRadius() - tor2.MajorRadius()) < tolTang
                                   && Math.Abs(tor1.MinorRadius() - tor2.MinorRadius()) < tolTang);
                }
                //
                if (ctType == global::Kerf.Geometry.SurfaceKind.Torus)
                {
                    ctAx = tor1.Axis();
                }
            }
            //
            CheckGeomGeom(ref bToCheck, ref bGeomGeom, ref treatAsBiParametric, ctAx, ref geomAx, ctType, ctSurf,
                          geomSurf);
        }
        //

        if (d1.DomainIsInfinite() || d2.DomainIsInfinite())
        {
            treatAsBiParametric = false;
        }

        if (treatAsBiParametric)
        {
            // Using Prm-Prm Intersector
            typs1 = global::Kerf.Geometry.SurfaceKind.BezierSurface;
            typs2 = global::Kerf.Geometry.SurfaceKind.BezierSurface;
        }

        // Surface type definition
        int ts1 = SurfaceKind(typs1, bGeomGeom);
        int ts2 = SurfaceKind(typs2, bGeomGeom);
        //
        // Possible intersection types: 1. ts1 == ts2 == 1 <Geom-Geom>
        //                              2. ts1 != ts2      <Geom-Param>
        //                              3. ts1 == ts2 == 0 <Param-Param>

        if (!isGeomInt)
        {
            ParamParamPerfom(s1, d1, s2, d2, tolArc, tolTang, listOfPnts, typs1, typs2);
        }
        else if (ts1 != ts2)
        {
            GeomParamPerfom(s1, d1, s2, d2, ts1 == 0, typs1, typs2);
        }
        else if (ts1 == 0)
        {
            ParamParamPerfom(s1, d1, s2, d2, tolArc, tolTang, listOfPnts, typs1, typs2);
        }
        else if (ts1 == 1)
        {
            GeomGeomPerfom(s1, d1, s2, d2, tolArc, tolTang, listOfPnts, typs1, typs2,
                           isReqToKeepRLine);
        }

        if (!isReqToPostWLProc)
        {
            return;
        }

        PurgeWLines(s1, d1, s2, d2);
    }

    // The common tail of OCCT's two Performs above, written out in each: the switch on the type of the surface
    // that isn't the cone or the torus, then the coaxial test (decision 671's pattern for verbatim repeats).
    private static void CheckGeomGeom(ref bool bToCheck, ref int bGeomGeom, ref bool treatAsBiParametric,
                                      in Axis3D aCTAx, ref Axis3D aGeomAx, SurfaceKind aCTType,
                                      SurfaceAdaptorBase aCTSurf, SurfaceAdaptorBase aGeomSurf)
    {
        if (bToCheck)
        {
            Line3D l1 = new(aCTAx);
            //
            switch (aGeomSurf.GetType())
            {
                case Plane:
                {
                    aGeomAx = aGeomSurf.Plane().Axis();
                    if (aCTType == global::Kerf.Geometry.SurfaceKind.Cone)
                    {
                        bGeomGeom = 1;
                        if (Math.Abs(aCTSurf.Cone().SemiAngle()) < 0.02)
                        {
                            double ps = Math.Abs(aCTAx.Direction().Dot(aGeomAx.Direction()));
                            if (ps < 0.015)
                            {
                                bGeomGeom = 0;
                            }
                        }
                    }
                    else
                    {
                        if (aCTAx.IsParallel(aGeomAx, global::Kerf.Engine.Foundation.Precision.Angular())
                            || (aCTAx.IsNormal(aGeomAx, global::Kerf.Engine.Foundation.Precision.Angular())
                                && (aGeomSurf.Plane().Distance(aCTAx.Location()) < global::Kerf.Engine.Foundation.Precision.Confusion())))
                        {
                            bGeomGeom = 1;
                        }
                    }
                    bToCheck = false;
                    break;
                }
                case global::Kerf.Geometry.SurfaceKind.Sphere:
                {
                    if (l1.Distance(aGeomSurf.Sphere().Location()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        bGeomGeom = 1;
                    }
                    bToCheck = false;
                    break;
                }
                case global::Kerf.Geometry.SurfaceKind.Cylinder:
                    aGeomAx = aGeomSurf.Cylinder().Axis();
                    break;
                case global::Kerf.Geometry.SurfaceKind.Cone:
                    aGeomAx = aGeomSurf.Cone().Axis();
                    break;
                case global::Kerf.Geometry.SurfaceKind.Torus:
                    aGeomAx = aGeomSurf.Torus().Axis();
                    break;
                default:
                    bToCheck = false;
                    break;
            }
            //
            if (bToCheck)
            {
                if (aCTAx.IsParallel(aGeomAx, global::Kerf.Engine.Foundation.Precision.Angular())
                    && (l1.Distance(aGeomAx.Location()) <= global::Kerf.Engine.Foundation.Precision.Confusion()))
                {
                    bGeomGeom = 1;
                }
            }
            //
            if (bGeomGeom == 1)
            {
                treatAsBiParametric = false;
            }
        }
    }

    // OCCT's two "Surface type definition" switches (repeated verbatim for ts1 and ts2).
    private static int SurfaceKind(SurfaceKind typs, int bGeomGeom)
    {
        int ts = 0;
        switch (typs)
        {
            case Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Cone:
                ts = 1;
                break;
            case global::Kerf.Geometry.SurfaceKind.Torus:
                ts = bGeomGeom;
                break;
            default:
                break;
        }
        return ts;
    }

    // The purging loop at the end of OCCT's three two-surface Performs (repeated verbatim).
    private void PurgeWLines(SurfaceAdaptorBase s1, TopolTool d1, SurfaceAdaptorBase s2,
                             TopolTool d2)
    {
        for (int i = NCollection_Sequence<PatchLine>.Lower(); i <= _slin.Upper(); i++)
        {
            WLine? wl = _slin.Value(i) as WLine;

            if (wl is null)
            {
                continue;
            }

            if (!wl.IsPurgingAllowed)
            {
                continue;
            }

            WLine? rw = WLineTool.ComputePurgedWLine(wl, s1, s2, d1, d2);

            if (rw is null)
            {
                continue;
            }

            _slin.InsertAfter(i, rw);
            _slin.Remove(i);
        }
    }

    //=================================================================================================

    private void ParamParamPerfom(SurfaceAdaptorBase s1, TopolTool d1, SurfaceAdaptorBase s2,
                                  TopolTool d2, double tolArc, double tolTang,
                                  NCollection_List<PntOn2S> listOfPnts, SurfaceKind typs1,
                                  SurfaceKind typs2)
    {
        PrmPrmIntersection interpp = new();
        //
        if (!d1.DomainIsInfinite() && !d2.DomainIsInfinite())
        {
            bool clearFlag = true;
            if (!listOfPnts.IsEmpty())
            {
                interpp.Perform(s1, d1, s2, d2, tolTang, tolArc, _fleche, _uvMaxStep, listOfPnts);
                clearFlag = false;
            }
            interpp.Perform(s1, d1, s2, d2, tolTang, tolArc, _fleche, _uvMaxStep, clearFlag);
        }
        else if ((d1.DomainIsInfinite()) ^ (d2.DomainIsInfinite()))
        {
            if (d1.DomainIsInfinite())
            {
                FUN_GetMinMaxXYZPnt(s2, out Point3D pMinXYZ, out Point3D pMaxXYZ);
                double mu = Max(Math.Abs(s2.FirstUParameter()), Math.Abs(s2.LastUParameter()));
                double mv = Max(Math.Abs(s2.FirstVParameter()), Math.Abs(s2.LastVParameter()));
                double ap = Max(mu, mv);
                FUN_TrimInfSurf(pMinXYZ, pMaxXYZ, s1, ap, out SurfaceAdaptorBase? ss);
                interpp.Perform(ss!, d1, s2, d2, tolTang, tolArc, _fleche, _uvMaxStep);
            }
            else
            {
                FUN_GetMinMaxXYZPnt(s1, out Point3D pMinXYZ, out Point3D pMaxXYZ);
                double mu = Max(Math.Abs(s1.FirstUParameter()), Math.Abs(s1.LastUParameter()));
                double mv = Max(Math.Abs(s1.FirstVParameter()), Math.Abs(s1.LastVParameter()));
                double ap = Max(mu, mv);
                FUN_TrimInfSurf(pMinXYZ, pMaxXYZ, s2, ap, out SurfaceAdaptorBase? ss);
                interpp.Perform(s1, d1, ss!, d2, tolTang, tolArc, _fleche, _uvMaxStep);
            }
        } //(theD1->DomainIsInfinite()) ^ (theD2->DomainIsInfinite())
        else
        {
            if (typs1 == OtherSurface || typs2 == OtherSurface)
            {
                _done = false;
                return;
            }

            NCollection_Sequence<Point3D> sop = new();
            FUN_PL_Intersection(s1, typs1, s2, typs2, out bool isPLInt, sop, out Vector3D v);

            if (isPLInt)
            {
                if (sop.Length() > 0)
                {
                    for (int ip = 1; ip <= sop.Length(); ip++)
                    {
                        Line3D lin = new(sop.Value(ip), new Direction3D(v));
                        PatchLine gl = new GLine(lin, false);
                        _slin.Append(gl);
                    }

                    _done = true;
                }
                else
                {
                    _done = false;
                }

                return;
            } // 'COLLINEAR LINES'
            else
            {
                SurfaceAdaptorBase nS1 = s1;
                SurfaceAdaptorBase nS2 = s2;
                FUN_TrimBothSurf(s1, typs1, s2, typs2, 1.0e+8, ref nS1, ref nS2);
                interpp.Perform(nS1, d1, nS2, d2, tolTang, tolArc, _fleche, _uvMaxStep);
            } // 'NON - COLLINEAR LINES'
        } // both domains are infinite

        if (interpp.IsDone)
        {
            _done = true;
            _tgte = false;
            _empt = interpp.IsEmpty;

            for (int i = 1; i <= interpp.LineCount; i++)
            {
                if (interpp.Line(i).ArcType() != PatchLineType.Walking)
                {
                    _slin.Append(interpp.Line(i));
                }
            }

            for (int i = 1; i <= interpp.LineCount; i++)
            {
                if (interpp.Line(i).ArcType() == PatchLineType.Walking)
                {
                    _slin.Append(interpp.Line(i));
                }
            }
        }
    }

    //=================================================================================================

    private void GeomGeomPerfom(SurfaceAdaptorBase s1, TopolTool d1, SurfaceAdaptorBase s2,
                                TopolTool d2, double tolArc, double tolTang,
                                NCollection_List<PntOn2S> listOfPnts, SurfaceKind typs1,
                                SurfaceKind typs2, bool isReqToKeepRLine)
    {
        ImpImpIntersection interii = new(s1, d1, s2, d2, _tolArc, _tolTang,
                                                  isReqToKeepRLine);

        if (!interii.IsDone)
        {
            _done = false;
            ParamParamPerfom(s1, d1, s2, d2, tolArc, tolTang, listOfPnts, typs1, typs2);
            return;
        }

        _done = (interii.GetStatus() == ImpImpIntersection.IntStatus.IntStatus_OK);
        _empt = interii.IsEmpty;

        if (_empt)
        {
            return;
        }

        const int nbPointsInALine = 200;

        _tgte = interii.TangentFaces();
        if (_tgte)
        {
            _oppo = interii.OppositeFaces();
        }

        bool isWLExist = false;
        ALineToWLine aToW = new(s1, s2, nbPointsInALine);

        for (int i = 1; i <= interii.LineCount; i++)
        {
            PatchLine line = interii.Line(i);
            if (line.ArcType() == PatchLineType.Analytic)
            {
                isWLExist = true;
                aToW.MakeWLine((ALine)line, _slin);
            }
            else
            {
                if (line.ArcType() == PatchLineType.Walking)
                {
                    ((WLine)line).EnablePurging(false);
                }

                if ((line.ArcType() != PatchLineType.Restriction) || isReqToKeepRLine)
                {
                    _slin.Append(line);
                }
            }
        }

        for (int i = 1; i <= interii.PntCount; i++)
        {
            _spnt.Append(interii.Point(i));
        }

        if ((typs1 == global::Kerf.Geometry.SurfaceKind.Cylinder) && (typs2 == global::Kerf.Geometry.SurfaceKind.Cylinder))
        {
            WLineTool.JoinWLines(_slin, _spnt, s1, s2, tolTang);
        }

        if (isWLExist)
        {
            Box2d bx1 = new(), bx2 = new();
            double u1F = s1.FirstUParameter(), u1L = s1.LastUParameter(),
                   v1F = s1.FirstVParameter(), v1L = s1.LastVParameter(),
                   u2F = s2.FirstUParameter(), u2L = s2.LastUParameter(),
                   v2F = s2.FirstVParameter(), v2L = s2.LastVParameter();
            bx1.Add(new Point2D(u1F, v1F));
            bx1.Add(new Point2D(u1L, v1F));
            bx1.Add(new Point2D(u1L, v1L));
            bx1.Add(new Point2D(u1F, v1L));
            bx2.Add(new Point2D(u2F, v2F));
            bx2.Add(new Point2D(u2L, v2F));
            bx2.Add(new Point2D(u2L, v2L));
            bx2.Add(new Point2D(u2F, v2L));

            bx1.Enlarge(global::Kerf.Engine.Foundation.Precision.PConfusion());
            bx2.Enlarge(global::Kerf.Engine.Foundation.Precision.PConfusion());

            ReadOnlySpan<double> arrOfPeriod = [s1.IsUPeriodic ? s1.UPeriod() : 0.0,
                                                  s1.IsVPeriodic ? s1.VPeriod() : 0.0,
                                                  s2.IsUPeriodic ? s2.UPeriod() : 0.0,
                                                  s2.IsVPeriodic ? s2.VPeriod() : 0.0];

            NCollection_List<Point3D> listOfCriticalPoints = new();

            if (s1.GetType() == global::Kerf.Geometry.SurfaceKind.Cone)
            {
                listOfCriticalPoints.Append(s1.Cone().Apex());
            }
            else if (s1.GetType() == global::Kerf.Geometry.SurfaceKind.Sphere)
            {
                listOfCriticalPoints.Append(s1.Value(0.0, M_PI_2));
                listOfCriticalPoints.Append(s1.Value(0.0, -M_PI_2));
            }

            if (s2.GetType() == global::Kerf.Geometry.SurfaceKind.Cone)
            {
                listOfCriticalPoints.Append(s2.Cone().Apex());
            }
            else if (s2.GetType() == global::Kerf.Geometry.SurfaceKind.Sphere)
            {
                listOfCriticalPoints.Append(s2.Value(0.0, M_PI_2));
                listOfCriticalPoints.Append(s2.Value(0.0, -M_PI_2));
            }

            WLineTool.ExtendTwoWLines(_slin, s1, s2, tolTang, arrOfPeriod, bx1, bx2,
                                               listOfCriticalPoints);
        }
    }

    //=================================================================================================

    private void GeomParamPerfom(SurfaceAdaptorBase s1, TopolTool d1, SurfaceAdaptorBase s2,
                                 TopolTool d2, bool isNotAnalitical, SurfaceKind typs1,
                                 SurfaceKind typs2)
    {
        ImpPrmIntersection interip = new();
        if (_isStartPnt)
        {
            if (isNotAnalitical /*ts1 == 0*/)
            {
                interip.SetStartPoint(_u1Start, _v1Start);
            }
            else
            {
                interip.SetStartPoint(_u2Start, _v2Start);
            }
        }

        if (d1.DomainIsInfinite() && d2.DomainIsInfinite())
        {
            NCollection_Sequence<Point3D> sop = new();
            FUN_PL_Intersection(s1, typs1, s2, typs2, out bool isPLInt, sop, out Vector3D v);

            if (isPLInt)
            {
                if (sop.Length() > 0)
                {
                    for (int ip = 1; ip <= sop.Length(); ip++)
                    {
                        Line3D lin = new(sop.Value(ip), new Direction3D(v));
                        PatchLine gl = new GLine(lin, false);
                        _slin.Append(gl);
                    }

                    _done = true;
                }
                else
                {
                    _done = false;
                }

                return;
            }
            else
            {
                SurfaceAdaptorBase nS1 = s1;
                SurfaceAdaptorBase nS2 = s2;
                FUN_TrimBothSurf(s1, typs1, s2, typs2, 1.0e+5, ref nS1, ref nS2);
                interip.Perform(nS1, d1, nS2, d2, _tolArc, _tolTang, _fleche, _uvMaxStep);
            }
        }
        else
        {
            interip.Perform(s1, d1, s2, d2, _tolArc, _tolTang, _fleche, _uvMaxStep);
        }

        if (interip.IsDone)
        {
            _done = true;
            _empt = interip.IsEmpty;

            if (!_empt)
            {
                int nbLines = interip.LineCount;
                for (int i = 1; i <= nbLines; i++)
                {
                    if (interip.Line(i).ArcType() != PatchLineType.Walking)
                    {
                        _slin.Append(interip.Line(i));
                    }
                }

                for (int i = 1; i <= nbLines; i++)
                {
                    if (interip.Line(i).ArcType() == PatchLineType.Walking)
                    {
                        _slin.Append(interip.Line(i));
                    }
                }

                for (int i = 1; i <= interip.PntCount; i++)
                {
                    _spnt.Append(interip.Point(i));
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Intersects S1 and S2 from a start point (U1, V1) on S1 and (U2, V2) on S2: through the analytic
    /// dispatch (the point given to IntPatch_ImpPrmIntersection) when one surface is a plane, cylinder, sphere or
    /// cone, by marching from the point (IntPatch_PrmPrmIntersection) otherwise.</summary>
    public void Perform(SurfaceAdaptorBase s1, TopolTool d1, SurfaceAdaptorBase s2, TopolTool d2,
                        double u1, double v1, double u2, double v2, double tolArc, double tolTang)
    {
        _tolArc = tolArc;
        _tolTang = tolTang;
        if (_fleche == 0.0)
        {
            _fleche = 0.01;
        }
        if (_uvMaxStep == 0.0)
        {
            _uvMaxStep = 0.01;
        }

        _done = false;
        _spnt.Clear();
        _slin.Clear();

        _empt = true;
        _tgte = false;
        _oppo = false;

        SurfaceKind typs1 = s1.GetType();
        SurfaceKind typs2 = s2.GetType();

        if (typs1 == Plane || typs1 == global::Kerf.Geometry.SurfaceKind.Cylinder || typs1 == global::Kerf.Geometry.SurfaceKind.Sphere || typs1 == global::Kerf.Geometry.SurfaceKind.Cone
            || typs2 == Plane || typs2 == global::Kerf.Geometry.SurfaceKind.Cylinder || typs2 == global::Kerf.Geometry.SurfaceKind.Sphere
            || typs2 == global::Kerf.Geometry.SurfaceKind.Cone)
        {
            _isStartPnt = true;
            _u1Start = u1;
            _v1Start = v1;
            _u2Start = u2;
            _v2Start = v2;
            Perform(s1, d1, s2, d2, tolArc, tolTang);
            _isStartPnt = false;
        }
        else
        {
            PrmPrmIntersection interpp = new();
            interpp.Perform(s1, d1, s2, d2, u1, v1, u2, v2, tolTang, tolArc, _fleche, _uvMaxStep);
            if (interpp.IsDone)
            {
                _done = true;
                _tgte = false;
                _empt = interpp.IsEmpty;
                int nblm = interpp.LineCount;
                int i = 1;
                for (; i <= nblm; i++)
                {
                    _slin.Append(interpp.Line(i));
                }
            }
        }

        PurgeWLines(s1, d1, s2, d2);
    }

    /// <summary>
    /// Writes the dump of the intersection. OCCT's body is compiled only with DUMPOFIntPatch_Intersection (a debug
    /// define), so a release build, which the port follows, writes nothing.
    /// </summary>
    public void Dump(int mode, SurfaceAdaptorBase s1, TopolTool d1, SurfaceAdaptorBase s2,
                     TopolTool d2, TextWriter theOut)
    {
        _ = mode;
        _ = s1;
        _ = d1;
        _ = s2;
        _ = d2;
        _ = theOut;
    }

    // ----- IntPatch_Intersection.lxx -----

    /// <summary>True if the computation succeeded.</summary>
    public bool IsDone => _done;

    /// <summary>True if there is no intersection. Throws StdFail_NotDone if not done.</summary>
    public bool IsEmpty
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _empt;
        }
    }

    /// <summary>True if the surfaces are tangent (identical within the tolerances). Throws StdFail_NotDone if not
    /// done.</summary>
    public bool TangentFaces()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _tgte;
    }

    /// <summary>True if the tangent surfaces' normals are opposite. Throws StdFail_NotDone if not done,
    /// Standard_DomainError if the surfaces aren't tangent.</summary>
    public bool OppositeFaces()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (!_tgte)
        {
            throw new DomainErrorException();
        }
        return _oppo;
    }

    /// <summary>The number of isolated points. Throws StdFail_NotDone if not done.</summary>
    public int PntCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _spnt.Length();
        }
    }

    /// <summary>The isolated point of index Index (a copy). Throws StdFail_NotDone if not done.</summary>
    public PatchPoint Point(int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _spnt[index];
    }

    /// <summary>The number of intersection lines. Throws StdFail_NotDone if not done.</summary>
    public int LineCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _slin.Length();
        }
    }

    /// <summary>The line of index Index (the line itself). Throws StdFail_NotDone if not done.</summary>
    public PatchLine Line(int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _slin[index];
    }

    //=================================================================================================

    /// <summary>
    /// True if a boundary of theS1 is degenerate (a pole or an apex); theDist is then the smallest distance from
    /// such a point to theS2, Precision.Infinite() otherwise.
    /// </summary>
    public static bool CheckSingularPoints(SurfaceAdaptorBase s1, TopolTool d1,
                                           SurfaceAdaptorBase s2, out double dist)
    {
        dist = global::Kerf.Engine.Foundation.Precision.Infinite();
        bool isSingular = false;
        if (ReferenceEquals(s1, s2))
        {
            return isSingular;
        }
        //
        const int nbBndPnts = 5;
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        int i;
        d1.Init();
        bool isU = true;
        for (; d1.More(); d1.Next())
        {
            Curve2DAdaptorBase bnd = d1.Value();
            double pinf = bnd.FirstParameter(), psup = bnd.LastParameter();
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(pinf) || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(psup))
            {
                continue;
            }
            double t, dt = (psup - pinf) / (nbBndPnts - 1);
            bnd.D1((pinf + psup) / 2.0, out Point2D p1, out Vector2D dir);
            isU = Math.Abs(dir.X()) > Math.Abs(dir.Y());
            Point3D pp1 = new();
            double d1NormMax = 0.0;
            Xyz pmid = new(0.0, 0.0, 0.0);
            int nb = 0;
            for (t = pinf; t <= psup; t += dt)
            {
                p1 = bnd.Value(t);
                s1.D1(p1.X(), p1.Y(), out pp1, out Vector3D du, out Vector3D dv);
                if (isU)
                {
                    d1NormMax = Max(d1NormMax, du.Magnitude());
                }
                else
                {
                    d1NormMax = Max(d1NormMax, dv.Magnitude());
                }

                pmid += pp1.XYZ();
                nb++;

                if (d1NormMax > tol)
                {
                    break;
                }
            }

            if (d1NormMax <= tol)
            {
                // Singular point aPP1;
                pmid /= nb;
                pp1.SetXYZ(pmid);
                double tolU = global::Kerf.Engine.Foundation.Precision.PConfusion(), tolV = global::Kerf.Engine.Foundation.Precision.PConfusion();
                ExtPS proj = new(pp1, s2, tolU, tolV, ExtFlag.ExtFlag_MIN);

                if (proj.IsDone)
                {
                    int nbExt = proj.ExtCount;
                    for (i = 1; i <= nbExt; ++i)
                    {
                        dist = Min(dist, proj.SquareDistance(i));
                    }
                }
            }
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(dist))
        {
            dist = Math.Sqrt(dist);
            isSingular = true;
        }

        return isSingular;
    }

    //=================================================================================================

    /// <summary>The largest step in parameters for the walking: 1e-4 when a singular point of one surface is
    /// within 1e-5 (but farther than Confusion) of the other, 1e-3 otherwise.</summary>
    public static double DefineUVMaxStep(SurfaceAdaptorBase s1, TopolTool d1, SurfaceAdaptorBase s2,
                                         TopolTool d2)
    {
        double uvMaxStep = 0.001;
        double tolMin = global::Kerf.Engine.Foundation.Precision.Confusion(), tolMax = 1.0e-5;
        if (!ReferenceEquals(s1, s2))
        {
            bool isSing1 = CheckSingularPoints(s1, d1, s2, out double distToSing1);
            if (isSing1)
            {
                if (distToSing1 > tolMin && distToSing1 < tolMax)
                {
                    uvMaxStep = 0.0001;
                }
                else
                {
                    isSing1 = false;
                }
            }
            if (!isSing1)
            {
                bool isSing2 = CheckSingularPoints(s2, d2, s1, out double distToSing2);
                if (isSing2)
                {
                    if (distToSing2 > tolMin && distToSing2 < tolMax)
                    {
                        uvMaxStep = 0.0001;
                    }
                }
            }
        }
        return uvMaxStep;
    }

    //=================================================================================================

    private static void SplitCone(SurfaceAdaptorBase s, TopolTool d, double tol,
                                  NCollection_DynamicArray<SurfaceAdaptorBase> vecHS)
    {
        if (s.GetType() != global::Kerf.Geometry.SurfaceKind.Cone)
        {
            throw new NoSuchObjectException("IntPatch_Intersection : Surface is not Cone");
        }

        Cone cone = s.Cone();

        TopolTool.GetConeApexParam(cone, out double u0, out double v0);

        Containment state = d.Classify(new Point2D(u0, v0), tol);

        if (state == In || state == On)
        {
            SurfaceAdaptorBase hsDn = s.VTrim(s.FirstVParameter(), v0, global::Kerf.Engine.Foundation.Precision.PConfusion());
            SurfaceAdaptorBase hsUp = s.VTrim(v0, s.LastVParameter(), global::Kerf.Engine.Foundation.Precision.PConfusion());

            vecHS.Append(hsDn);
            vecHS.Append(hsUp);
        }
        else
        {
            vecHS.Append(s);
        }
    }

    //=======================================================================
    // function : PrepareSurfaces
    // purpose  : Prepares surfaces for intersection
    //=======================================================================
    /// <summary>
    /// Prepares the surfaces for the intersection: a cone whose semi-angle is within theTol of π/2 is split at its
    /// apex when the apex is in its domain; every other surface is appended as is.
    /// </summary>
    public static void PrepareSurfaces(SurfaceAdaptorBase s1, TopolTool d1, SurfaceAdaptorBase s2,
                                       TopolTool d2, double tol,
                                       NCollection_DynamicArray<SurfaceAdaptorBase> vecHS1,
                                       NCollection_DynamicArray<SurfaceAdaptorBase> vecHS2)
    {
        if ((s1.GetType() == global::Kerf.Geometry.SurfaceKind.Cone) && (Math.Abs(M_PI / 2.0 - Math.Abs(s1.Cone().SemiAngle())) < tol))
        {
            SplitCone(s1, d1, tol, vecHS1);
        }
        else
        {
            vecHS1.Append(s1);
        }

        if ((s2.GetType() == global::Kerf.Geometry.SurfaceKind.Cone) && (Math.Abs(M_PI / 2.0 - Math.Abs(s2.Cone().SemiAngle())) < tol))
        {
            SplitCone(s2, d2, tol, vecHS2);
        }
        else
        {
            vecHS2.Append(s2);
        }
    }
}
