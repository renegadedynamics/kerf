// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_BuildPlateSurface.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_BuildPlateSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// Builds a plate surface through curve and point constraints: an initial surface (given, or an average plane, or a
/// first plate's approximation), deformed by a plate of minimal energy that meets the constraints, refined over
/// several iterations until the G0/G1 criteria of the curve constraints are met.
/// </summary>
/// <remarks>
/// A working object (decision 21), as OCCT's: load constraints with <c>Add</c>, then <see cref="Perform"/>. It keeps
/// the caller's constraints and changes them as OCCT does (their 2D curves, numbers of points and point projections;
/// <c>Perform</c> also reorders its list of curve constraints). <see cref="Surface"/> and <see cref="SurfInit"/>
/// freeze the surfaces they return, and <see cref="Curves2d"/> the 2D curves.
/// </remarks>
internal class BuildPlateSurface
{
    private NCollection_HSequence<CurveConstraint> _linCont;
    private NCollection_HArray1<NCollection_Sequence<double>>? _parCont;
    private NCollection_HArray1<NCollection_Sequence<double>>? _plateCont;
    private NCollection_HSequence<PointConstraint> _pntCont;
    private Surface? _surfInit;
    private Surface? _planarSurfInit;
    private PlateSurface? _geomPlateSurface;
    private readonly PlateSolver _plate = new PlateSolver();
    private readonly PlateSolver _prevPlate = new PlateSolver();
    private readonly bool _anisotropie;
    private NCollection_HArray1<int>? _sense;
    private readonly int _degree;
    private NCollection_HArray1<int>? _initOrder;
    private double _g0Error;
    private double _g1Error;
    private double _g2Error;
    private int _nbPtsOnCur;
    private bool _surfInitIsGive;
    private readonly int _nbIter;
    private readonly ExtPS _proj = new ExtPS();
    private readonly double _tol2d;
    private readonly double _tol3d;
    private readonly double _tolAng;
    private double _tolU;
    private double _tolV;
    private int _nbBounds;
    private bool _isLinear;
    private bool _free;

    // Arrays of sequences: OCCT default-constructs the elements, a C# array of classes holds nulls.
    private static NCollection_HArray1<NCollection_Sequence<double>> NewSequences(int upper)
    {
        NCollection_HArray1<NCollection_Sequence<double>> array = new NCollection_HArray1<NCollection_Sequence<double>>(1, upper);
        for (int i = 1; i <= upper; i++)
        {
            array[i] = new NCollection_Sequence<double>();
        }
        return array;
    }

    /// <summary>The constructor compatible with the old version: one curve constraint per curve.</summary>
    /// <exception cref="ConstructionErrorException">Fewer than one iteration, no curve, no order, no point, or a
    /// degree below 2.</exception>
    public BuildPlateSurface(NCollection_HArray1<int> nPoints, NCollection_HArray1<CurveAdaptorBase> tabCurve,
                                       NCollection_HArray1<int> tang, int degree, int nbIter = 3, double tol2d = 0.00001,
                                       double tol3d = 0.0001, double tolAng = 0.01, double tolCurv = 0.1,
                                       bool anisotropie = false)
    {
        _anisotropie = anisotropie;
        _degree = degree;
        _nbIter = nbIter;
        _tol2d = tol2d;
        _tol3d = tol3d;
        _tolAng = tolAng;
        _nbBounds = 0;
        int ntCurve = tabCurve.Length(); // Number of linear constraints
        _nbPtsOnCur = 0; // Different calculation of the number of points depending on the length
        _linCont = new NCollection_HSequence<CurveConstraint>();
        _pntCont = new NCollection_HSequence<PointConstraint>();
        if (_nbIter < 1)
        {
            throw new ConstructionErrorException("GeomPlate :  Number of iteration must be >= 1");
        }

        if (ntCurve == 0)
        {
            throw new ConstructionErrorException("GeomPlate : the bounds Array is null");
        }
        if (tang.Length() == 0)
        {
            throw new ConstructionErrorException("GeomPlate : the constraints Array is null");
        }
        int nbp = 0;
        int i;
        for (i = 1; i <= ntCurve; i++)
        {
            nbp += nPoints.Value(i);
        }
        if (nbp == 0)
        {
            throw new ConstructionErrorException(
                "GeomPlate : the resolution is impossible if the number of constraints points is 0");
        }
        if (_degree < 2)
        {
            throw new ConstructionErrorException("GeomPlate ; the degree resolution must be upper of 2");
        }
        // Filling fields passing from the old constructor to the new one
        for (i = 1; i <= ntCurve; i++)
        {
            CurveConstraint cont = new CurveConstraint(tabCurve.Value(i), tang.Value(i), nPoints.Value(i));
            _linCont.Append(cont);
        }
        _surfInitIsGive = false;

        _isLinear = true;
        _free = false;
    }

    /// <summary>A builder deforming the initial surface <paramref name="surf"/>.</summary>
    /// <exception cref="ConstructionErrorException">Fewer than one iteration, or a degree below 2.</exception>
    public BuildPlateSurface(Surface surf, int degree = 3, int nbPtsOnCur = 10, int nbIter = 3,
                                       double tol2d = 0.00001, double tol3d = 0.0001, double tolAng = 0.01,
                                       double tolCurv = 0.1, bool anisotropie = false)
    {
        _surfInit = surf;
        _anisotropie = anisotropie;
        _degree = degree;
        _nbPtsOnCur = nbPtsOnCur;
        _nbIter = nbIter;
        _tol2d = tol2d;
        _tol3d = tol3d;
        _tolAng = tolAng;
        _nbBounds = 0;
        if (_nbIter < 1)
        {
            throw new ConstructionErrorException("GeomPlate :  Number of iteration must be >= 1");
        }
        if (_degree < 2)
        {
            throw new ConstructionErrorException("GeomPlate : the degree must be above 2");
        }
        _linCont = new NCollection_HSequence<CurveConstraint>();
        _pntCont = new NCollection_HSequence<PointConstraint>();
        _surfInitIsGive = true;

        _isLinear = true;
        _free = false;
    }

    /// <summary>A builder that computes its initial surface.</summary>
    /// <exception cref="ConstructionErrorException">Fewer than one iteration, or a degree below 2.</exception>
    public BuildPlateSurface(int degree = 3, int nbPtsOnCur = 10, int nbIter = 3, double tol2d = 0.00001,
                                       double tol3d = 0.0001, double tolAng = 0.01, double tolCurv = 0.1,
                                       bool anisotropie = false)
    {
        _anisotropie = anisotropie;
        _degree = degree;
        _nbPtsOnCur = nbPtsOnCur;
        _nbIter = nbIter;
        _tol2d = tol2d;
        _tol3d = tol3d;
        _tolAng = tolAng;
        _nbBounds = 0;
        if (_nbIter < 1)
        {
            throw new ConstructionErrorException("GeomPlate :  Number of iteration must be >= 1");
        }
        if (_degree < 2)
        {
            throw new ConstructionErrorException("GeomPlate : the degree resolution must be upper of 2");
        }
        _linCont = new NCollection_HSequence<CurveConstraint>();
        _pntCont = new NCollection_HSequence<PointConstraint>();
        _surfInitIsGive = false;

        _isLinear = true;
        _free = false;
    }

    // Reorder the table of transformations
    // After the call of CourbeJointive the order of curves is modified
    // Ex : initial order of curves ==> A B C D E F
    //      In TabInit we note ==> 1 2 3 4 5 6
    //      after CourbeJointive ==> A E C B D F
    //            TabInit ==> 1 5 3 2 4 6
    //      after TrierTab the Table contains ==> 1 4 3 5 2 6
    // It is also possible to access the 2nd curve by taking TabInit[2]
    // i.e. the 4th from the table of classified curves
    private static void TrierTab(NCollection_HArray1<int> tab)
    {
        // Parse the table of transformations to find the initial order
        int nb = tab.Length();
        NCollection_Array1<int> tabTri = new NCollection_Array1<int>(1, nb);
        for (int i = 1; i <= nb; i++)
        {
            tabTri.SetValue(tab.Value(i), i);
        }
        tab.ChangeArray1().Assign(tabTri);
    }

    private Curve2D? ProjectCurve(CurveAdaptorBase curv)
    {
        // Project a curve on a plane
        Curve2D? curve2d;
        SurfaceAdaptor hsur = new SurfaceAdaptor(_surfInit!);
        Point2D p2d;

        CompProjectedCurve hProjector = new CompProjectedCurve(hsur, curv, _tol3d / 10, _tol3d / 10);

        double udebCheck, ufinCheck, projUdeb, projUfin;
        udebCheck = curv.FirstParameter();
        ufinCheck = curv.LastParameter();
        hProjector.Bounds(1, out projUdeb, out projUfin);

        if (hProjector.CurveCount != 1 || Math.Abs(udebCheck - projUdeb) > global::Kerf.Engine.Foundation.Precision.PConfusion()
            || Math.Abs(ufinCheck - projUfin) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            if (hProjector.IsSinglePnt(1, out p2d))
            {
                // solution in a point
                NCollection_Array1<Point2D> poles = new NCollection_Array1<Point2D>(1, 2);
                poles.Init(p2d);
                curve2d = new BezierCurve2D(poles);
            }
            else
            {
                curve2d = null; // No continuous solution
            }
        }
        else
        {
            Continuity continuity = C1;
            int maxDegree = 10, maxSeg;
            double udeb, ufin;
            hProjector.Bounds(1, out udeb, out ufin);

            maxSeg = 20 + hProjector.NbIntervals(C3);
            CurveOnSurfaceApproximation appr = new CurveOnSurfaceApproximation(hProjector, hsur, udeb, ufin, _tol3d);
            appr.Perform(maxSeg, maxDegree, continuity, false, true);

            curve2d = appr.Curve2d();
        }
        return curve2d;
    }

    // Projects a point on the initial surface
    private Point2D ProjectPoint(in Point3D p3d)
    {
        POnSurf p;
        _proj.Perform(p3d);
        int nearest = 1;
        if (_proj.ExtCount > 1)
        {
            double dist2mini = _proj.SquareDistance(1);
            for (int i = 2; i <= _proj.ExtCount; i++)
            {
                if (_proj.SquareDistance(i) < dist2mini)
                {
                    dist2mini = _proj.SquareDistance(i);
                    nearest = i;
                }
            }
        }
        p = _proj.Point(nearest);
        double u, v;
        p.Parameter(out u, out v);
        Point2D p2d = new Point2D().WithCoord(u, v);

        return p2d;
    }

    /// <summary>Removes the curve and point constraints.</summary>
    public void Init()
    {
        _linCont.Clear();
        _pntCont.Clear();
        _pntCont = new NCollection_HSequence<PointConstraint>();
        _linCont = new NCollection_HSequence<CurveConstraint>();
    }

    /// <summary>Loads the initial surface.</summary>
    public void LoadInitSurface(Surface surf)
    {
        _surfInit = surf;
        _surfInitIsGive = true;
    }

    /// <summary>Adds a curve constraint.</summary>
    public void Add(CurveConstraint cont)
    {
        _linCont.Append(cont);
    }

    public void SetNbBounds(int nbBounds)
    {
        _nbBounds = nbBounds;
    }

    /// <summary>Adds a point constraint.</summary>
    public void Add(PointConstraint cont)
    {
        _pntCont.Append(cont);
    }

    /// <summary>Computes the surface filled with the loaded constraints.</summary>
    public void Perform(ProgressRange progress = default)
    {
        _geomPlateSurface = null;

        if (_nbBounds == 0)
        {
            _nbBounds = _linCont.Length();
        }

        _plate.Init();
        //=====================================================================
        // Declaration of variables.
        //=====================================================================
        int ntLinCont = _linCont.Length(), ntPntCont = _pntCont.Length(), nbBoucle = 0;
        bool fini = true;
        if ((ntLinCont + ntPntCont) == 0)
        {
            return;
        }

        //======================================================================
        // Initial Surface
        //======================================================================
        using ProgressScope ps = new ProgressScope(progress, "Calculating the surface filled", 100, true);
        if (!_surfInitIsGive)
        {
            ComputeSurfInit(ps.Next(10));
            if (ps.UserBreak())
            {
                return;
            }
        }

        else
        {
            if (ntLinCont >= 2)
            { // Table of transformations to preserve the initial order, see TrierTab
                _initOrder = new NCollection_HArray1<int>(1, ntLinCont);
                for (int l = 1; l <= ntLinCont; l++)
                {
                    _initOrder.SetValue(l, l);
                }
                if (!CourbeJointive(_tol3d))
                { //    throw Standard_Failure("Curves are not joined");
                }
                TrierTab(_initOrder); // Reorder the table of transformations
            }
            else if (ntLinCont > 0) // Patch
            {
                _sense = new NCollection_HArray1<int>(1, ntLinCont, 0);
                _initOrder = new NCollection_HArray1<int>(1, ntLinCont, 1);
            }
        }

        if (_surfInit is null)
        {
            return;
        }

        double u1, v1, u2, v2;
        _surfInit.ParameterBounds(out u1, out v1, out u2, out v2);
        SurfaceAdaptor surfInit = new SurfaceAdaptor(_surfInit);
        _tolU = surfInit.UResolution(_tol3d);
        _tolV = surfInit.VResolution(_tol3d);
        _proj.Initialize(surfInit, u1, v1, u2, v2, _tolU, _tolV);

        //======================================================================
        // Projection of curves
        //======================================================================
        bool ok = true;
        for (int i = 1; i <= ntLinCont; i++)
        {
            if (_linCont.Value(i).Curve2dOnSurf() is null)
            {
                Curve2D? curve2d = ProjectCurve(_linCont.Value(i).Curve3d());
                if (curve2d is null)
                {
                    ok = false;
                    break;
                }
                _linCont.ChangeValue(i).SetCurve2dOnSurf(curve2d);
            }
        }
        if (!ok)
        {
            // OCCT: myGeomPlateSurface was nullified at the start of Perform, and is set again only when
            // ComputeSurfInit computed a first plate; otherwise native OCCT dereferences a null handle here
            // (an access violation). The port throws Standard_NullObject (occt-issues.md).
            if (_geomPlateSurface is null)
            {
                throw new NullObjectException("GeomPlate_BuildPlateSurface::Perform: no plate surface to approximate");
            }
            MakeApprox app = new MakeApprox(_geomPlateSurface, _tol3d, 1, 3, 15 * _tol3d, -1, C0, 1.3);
            _surfInit = app.Surface()!;

            _surfInit.ParameterBounds(out u1, out v1, out u2, out v2);
            SurfaceAdaptor surf = new SurfaceAdaptor(_surfInit);
            _tolU = surf.UResolution(_tol3d);
            _tolV = surf.VResolution(_tol3d);
            _proj.Initialize(surf, u1, v1, u2, v2, _tolU, _tolV);

            ok = true;
            for (int i = 1; i <= ntLinCont; i++)
            {
                Curve2D? curve2d = ProjectCurve(_linCont.Value(i).Curve3d());
                if (curve2d is null)
                {
                    ok = false;
                    break;
                }
                _linCont.ChangeValue(i).SetCurve2dOnSurf(curve2d);
            }
            if (!ok)
            {
                _surfInit = _planarSurfInit!;

                _surfInit.ParameterBounds(out u1, out v1, out u2, out v2);
                SurfaceAdaptor surfNew = new SurfaceAdaptor(_surfInit);
                _tolU = surfNew.UResolution(_tol3d);
                _tolV = surfNew.VResolution(_tol3d);
                _proj.Initialize(surfNew, u1, v1, u2, v2, _tolU, _tolV);

                for (int i = 1; i <= ntLinCont; i++)
                {
                    _linCont.ChangeValue(i).SetCurve2dOnSurf(ProjectCurve(_linCont.Value(i).Curve3d()));
                }
            }
            else
            { // Project the points
                for (int i = 1; i <= ntPntCont; i++)
                {
                    Point3D p;
                    _pntCont.Value(i).D0(out p);
                    _pntCont.ChangeValue(i).SetPnt2dOnSurf(ProjectPoint(p));
                }
            }
        }

        //======================================================================
        // Projection of points
        //======================================================================
        for (int i = 1; i <= ntPntCont; i++)
        {
            if (!_pntCont.Value(i).HasPnt2dOnSurf)
            {
                Point3D p;
                _pntCont.Value(i).D0(out p);
                _pntCont.ChangeValue(i).SetPnt2dOnSurf(ProjectPoint(p));
            }
        }

        //======================================================================
        // Number of points by curve
        //======================================================================
        if ((ntLinCont != 0) && (_nbPtsOnCur != 0))
        {
            CalculNbPtsInit();
        }

        //======================================================================
        // Management of incompatibilites between curves
        //======================================================================
        NCollection_HArray1<NCollection_Sequence<double>>? pntInter = null;
        NCollection_HArray1<NCollection_Sequence<double>>? pntG1G1 = null;
        if (ntLinCont != 0)
        {
            pntInter = NewSequences(ntLinCont);
            pntG1G1 = NewSequences(ntLinCont);
            Intersect(pntInter, pntG1G1);
        }

        //======================================================================
        // Loop to obtain a better surface
        //======================================================================

        _free = !_isLinear;

        do
        {
            nbBoucle++;
            if (ntLinCont != 0)
            { //====================================================================
              // Calculate the total number of points and the maximum of points by curve
              //====================================================================
                int nPointMax = 0;
                for (int i = 1; i <= ntLinCont; i++)
                {
                    if ((_linCont.Value(i).PointCount) > nPointMax)
                    {
                        nPointMax = (int)(_linCont.Value(i).PointCount);
                    }
                }
                //====================================================================
                // Discretization of curves
                //====================================================================
                Discretise(pntInter!, pntG1G1!);
                //====================================================================
                // Preparation of constraint points for plate
                //====================================================================
                LoadCurve(nbBoucle);
                if (_pntCont.Length() != 0)
                {
                    LoadPoint(nbBoucle);
                }
                //====================================================================
                // Construction of the surface
                //====================================================================

                _plate.SolveTI(_degree, ComputeAnisotropie(), ps.Next(90));

                if (ps.UserBreak())
                {
                    return;
                }

                if (!_plate.IsDone)
                {
                    return;
                }

                _geomPlateSurface = new PlateSurface(_surfInit, _plate);
                double umin, umax, vmin, vmax;
                _plate.UVBox(out umin, out umax, out vmin, out vmax);
                _geomPlateSurface.SetBounds(umin, umax, vmin, vmax);

                fini = VerifSurface(nbBoucle);
                if ((nbBoucle >= _nbIter) && (!fini))
                {
                    fini = true;
                }

                if ((ntPntCont != 0) && (fini))
                {
                    double di, an, cu;
                    VerifPoints(out di, out an, out cu);
                }
            }
            else
            {
                LoadPoint(nbBoucle);
                //====================================================================
                // Construction of the surface
                //====================================================================
                _plate.SolveTI(_degree, ComputeAnisotropie(), ps.Next(90));

                if (ps.UserBreak())
                {
                    return;
                }

                if (!_plate.IsDone)
                {
                    return;
                }

                _geomPlateSurface = new PlateSurface(_surfInit, _plate);
                double umin, umax, vmin, vmax;
                _plate.UVBox(out umin, out umax, out vmin, out vmax);
                _geomPlateSurface.SetBounds(umin, umax, vmin, vmax);
                fini = true;
                double di, an, cu;
                VerifPoints(out di, out an, out cu);
            }
        } while (!fini); // End loop for better surface
    }

    private void EcartContraintesMil(int c, NCollection_HArray1<double> d, NCollection_HArray1<double> an,
                                     NCollection_HArray1<double> courb)
    {
        int nbPt = _parCont!.Value(c).Length();
        double u;
        if (nbPt < 3)
        {
            nbPt = 4;
        }
        else
        {
            nbPt = _parCont.Value(c).Length();
        }
        Vector3D v1i, v1f, v2i, v2f, v3i, v3f;
        Point3D pi, pf;
        Point2D p2d;
        int i;
        CurveConstraint linCont = _linCont.Value(c);
        switch (linCont.Order())
        {
            case 0:
                for (i = 1; i < nbPt; i++)
                {
                    u = (_parCont.Value(c).Value(i) + _parCont.Value(c).Value(i + 1)) / 2;
                    linCont.D0(u, out pi);
                    if (linCont.ProjectedCurve() is not null)
                    {
                        p2d = linCont.ProjectedCurve()!.Value(u);
                    }
                    else
                    {
                        if (linCont.Curve2dOnSurf() is not null)
                        {
                            p2d = linCont.Curve2dOnSurf()!.Value(u);
                        }
                        else
                        {
                            p2d = ProjectPoint(pi);
                        }
                    }
                    _geomPlateSurface!.D0(p2d.Coord(1), p2d.Coord(2), out pf);
                    an.Init(0);
                    courb.Init(0);
                    d.ChangeValue(i) = pf.DistanceTo(pi);
                }
                break;
            case 1:
                for (i = 1; i < nbPt; i++)
                {
                    u = (_parCont.Value(c).Value(i) + _parCont.Value(c).Value(i + 1)) / 2;
                    linCont.D1(u, out pi, out v1i, out v2i);
                    if (linCont.ProjectedCurve() is not null)
                    {
                        p2d = linCont.ProjectedCurve()!.Value(u);
                    }
                    else
                    {
                        if (linCont.Curve2dOnSurf() is not null)
                        {
                            p2d = linCont.Curve2dOnSurf()!.Value(u);
                        }
                        else
                        {
                            p2d = ProjectPoint(pi);
                        }
                    }
                    _geomPlateSurface!.D1(p2d.Coord(1), p2d.Coord(2), out pf, out v1f, out v2f);
                    d.ChangeValue(i) = pf.DistanceTo(pi);
                    v3i = v1i ^ v2i;
                    v3f = v1f ^ v2f;
                    double angle = v3f.AngleTo(v3i);
                    if (angle > (M_PI / 2))
                    {
                        an.ChangeValue(i) = M_PI - angle;
                    }
                    else
                    {
                        an.ChangeValue(i) = angle;
                    }
                    courb.Init(0);
                }
                break;
            case 2:
                Surface splate = _geomPlateSurface!;
                SurfaceContinuity cg2 = new SurfaceContinuity();
                for (i = 1; i < nbPt; i++)
                {
                    u = (_parCont.Value(c).Value(i) + _parCont.Value(c).Value(i + 1)) / 2;
                    linCont.D0(u, out pi);
                    if (linCont.ProjectedCurve() is not null)
                    {
                        p2d = linCont.ProjectedCurve()!.Value(u);
                    }
                    else
                    {
                        if (linCont.Curve2dOnSurf() is not null)
                        {
                            p2d = linCont.Curve2dOnSurf()!.Value(u);
                        }
                        else
                        {
                            p2d = ProjectPoint(pi);
                        }
                    }
                    SLProps prop = new SLProps(splate, p2d.Coord(1), p2d.Coord(2), 2, 0.001);
                    cg2.ComputeAnalysis(prop, _linCont.Value(c).LPropSurf(u), G2);
                    d.ChangeValue(i) = cg2.C0Value();
                    an.ChangeValue(i) = cg2.G1Angle();
                    courb.ChangeValue(i) = cg2.G2CurvatureGap();
                }
                break;
        }
    }

    /// <summary>
    /// The UV points of the constraints (order other than -1) on the initial surface: the point constraints, then
    /// each curve's first point and, on each interval, its points at 1/4, 1/2, 3/4 and the end.
    /// </summary>
    public void Disc2dContour(int nbp, NCollection_Sequence<Xy> seq2d)
    {
        //  initialization
        seq2d.Clear();

        //  sampling in "cosine" + 3 points on each interval
        int ntCurve = _linCont.Length();
        int ntPntCont = _pntCont.Length();
        Point2D p2d;
        Xy uv = new Xy();
        Point3D pp;
        double u1, v1, u2, v2;
        int i;

        _surfInit!.ParameterBounds(out u1, out v1, out u2, out v2);
        SurfaceAdaptor surf = new SurfaceAdaptor(_surfInit);
        _proj.Initialize(surf, u1, v1, u2, v2, _tolU, _tolV);

        for (i = 1; i <= ntPntCont; i++)
        {
            if (_pntCont.Value(i).Order() != -1)
            {
                p2d = _pntCont.Value(i).Pnt2dOnSurf();
                uv.SetX(p2d.Coord(1));
                uv.SetY(p2d.Coord(2));
                seq2d.Append(uv);
            }
        }
        for (i = 1; i <= ntCurve; i++)
        {
            CurveConstraint linCont = _linCont.Value(i);
            if (linCont.Order() != -1)
            {
                int nbPt = _parCont!.Value(i).Length();
                // first point of constraint (j=0)
                if (linCont.ProjectedCurve() is not null)
                {
                    p2d = linCont.ProjectedCurve()!.Value(_parCont.Value(i).Value(1));
                }
                else
                {
                    if (linCont.Curve2dOnSurf() is not null)
                    {
                        p2d = linCont.Curve2dOnSurf()!.Value(_parCont.Value(i).Value(1));
                    }
                    else
                    {
                        linCont.D0(_parCont.Value(i).Value(1), out pp);
                        p2d = ProjectPoint(pp);
                    }
                }

                uv.SetX(p2d.Coord(1));
                uv.SetY(p2d.Coord(2));
                seq2d.Append(uv);
                for (int j = 2; j < nbPt; j++)
                {
                    double uj = _parCont.Value(i).Value(j), ujp1 = _parCont.Value(i).Value(j + 1);
                    if (linCont.ProjectedCurve() is not null)
                    {
                        p2d = linCont.ProjectedCurve()!.Value((ujp1 + 3 * uj) / 4);
                    }
                    else
                    {
                        if (linCont.Curve2dOnSurf() is not null)
                        {
                            p2d = linCont.Curve2dOnSurf()!.Value((ujp1 + 3 * uj) / 4);
                        }
                        else
                        {
                            linCont.D0((ujp1 + 3 * uj) / 4, out pp);
                            p2d = ProjectPoint(pp);
                        }
                    }
                    uv.SetX(p2d.Coord(1));
                    uv.SetY(p2d.Coord(2));
                    seq2d.Append(uv);
                    // point 1/2 previous
                    if (linCont.ProjectedCurve() is not null)
                    {
                        p2d = linCont.ProjectedCurve()!.Value((ujp1 + uj) / 2);
                    }
                    else
                    {
                        if (linCont.Curve2dOnSurf() is not null)
                        {
                            p2d = linCont.Curve2dOnSurf()!.Value((ujp1 + uj) / 2);
                        }
                        else
                        {
                            linCont.D0((ujp1 + uj) / 2, out pp);
                            p2d = ProjectPoint(pp);
                        }
                    }

                    uv.SetX(p2d.Coord(1));
                    uv.SetY(p2d.Coord(2));
                    seq2d.Append(uv);
                    //  point 3/4 previous
                    if (linCont.ProjectedCurve() is not null)
                    {
                        p2d = linCont.ProjectedCurve()!.Value((3 * ujp1 + uj) / 4);
                    }
                    else
                    {
                        if (linCont.Curve2dOnSurf() is not null)
                        {
                            p2d = linCont.Curve2dOnSurf()!.Value((3 * ujp1 + uj) / 4);
                        }
                        else
                        {
                            linCont.D0((3 * ujp1 + uj) / 4, out pp);
                            p2d = ProjectPoint(pp);
                        }
                    }

                    uv.SetX(p2d.Coord(1));
                    uv.SetY(p2d.Coord(2));
                    seq2d.Append(uv);
                    // current constraint point
                    if (linCont.ProjectedCurve() is not null)
                    {
                        p2d = linCont.ProjectedCurve()!.Value(ujp1);
                    }
                    else
                    {
                        if (linCont.Curve2dOnSurf() is not null)
                        {
                            p2d = linCont.Curve2dOnSurf()!.Value(ujp1);
                        }
                        else
                        {
                            linCont.D0(ujp1, out pp);
                            p2d = ProjectPoint(pp);
                        }
                    }

                    uv.SetX(p2d.Coord(1));
                    uv.SetY(p2d.Coord(2));
                    seq2d.Append(uv);
                }
            }
        }
    }

    /// <summary>
    /// The 3D points (<paramref name="iordre"/> 0) or normals (1) of the constraints at the points of
    /// <see cref="Disc2dContour"/>.
    /// </summary>
    public void Disc3dContour(int nbp, int iordre, NCollection_Sequence<Xyz> seq3d)
    {
        //  initialization
        seq3d.Clear();
        //  sampling in "cosine" + 3 points on each interval
        double u1, v1, u2, v2;
        _surfInit!.ParameterBounds(out u1, out v1, out u2, out v2);
        SurfaceAdaptor surf = new SurfaceAdaptor(_surfInit);
        _proj.Initialize(surf, u1, v1, u2, v2, surf.UResolution(_tol3d), surf.VResolution(_tol3d));
        int ntCurve = _linCont.Length();
        int ntPntCont = _pntCont.Length();
        Point3D p3d;
        Vector3D v1h, v2h, v3h;
        Xyz pos = new Xyz();
        int i;

        for (i = 1; i <= ntPntCont; i++)
        {
            if (_pntCont.Value(i).Order() != -1)
            {
                if (iordre == 0)
                {
                    _pntCont.Value(i).D0(out p3d);
                    pos.SetX(p3d.X);
                    pos.SetY(p3d.Y);
                    pos.SetZ(p3d.Z);
                    seq3d.Append(pos);
                }
                else
                {
                    _pntCont.Value(i).D1(out p3d, out v1h, out v2h);
                    v3h = v1h ^ v2h;
                    pos.SetX(v3h.X);
                    pos.SetY(v3h.Y);
                    pos.SetZ(v3h.Z);
                    seq3d.Append(pos);
                }
            }
        }

        for (i = 1; i <= ntCurve; i++)
        {
            if (_linCont.Value(i).Order() != -1)
            {
                int nbPt = _parCont!.Value(i).Length();
                // first constraint point (j=0)
                if (iordre == 0)
                {
                    _linCont.Value(i).D0(_parCont.Value(i).Value(1), out p3d);
                    pos.SetX(p3d.X);
                    pos.SetY(p3d.Y);
                    pos.SetZ(p3d.Z);
                    seq3d.Append(pos);
                }
                else
                {
                    _linCont.Value(i).D1(_parCont.Value(i).Value(1), out p3d, out v1h, out v2h);
                    v3h = v1h ^ v2h;
                    pos.SetX(v3h.X);
                    pos.SetY(v3h.Y);
                    pos.SetZ(v3h.Z);
                    seq3d.Append(pos);
                }

                for (int j = 2; j < nbPt; j++)
                {
                    double uj = _parCont.Value(i).Value(j), ujp1 = _parCont.Value(i).Value(j + 1);
                    if (iordre == 0)
                    {
                        // point 1/4 previous
                        _linCont.Value(i).D0((ujp1 + 3 * uj) / 4, out p3d);
                        pos.SetX(p3d.X);
                        pos.SetY(p3d.Y);
                        pos.SetZ(p3d.Z);
                        seq3d.Append(pos);
                        // point 1/2 previous
                        _linCont.Value(i).D0((ujp1 + uj) / 2, out p3d);
                        pos.SetX(p3d.X);
                        pos.SetY(p3d.Y);
                        pos.SetZ(p3d.Z);
                        seq3d.Append(pos);
                        // point 3/4 previous
                        _linCont.Value(i).D0((3 * ujp1 + uj) / 4, out p3d);
                        pos.SetX(p3d.X);
                        pos.SetY(p3d.Y);
                        pos.SetZ(p3d.Z);
                        seq3d.Append(pos);
                        // current constraint point
                        _linCont.Value(i).D0(ujp1, out p3d);
                        pos.SetX(p3d.X);
                        pos.SetY(p3d.Y);
                        pos.SetZ(p3d.Z);
                        seq3d.Append(pos);
                    }
                    else
                    {
                        // point 1/4 previous
                        _linCont.Value(i).D1((ujp1 + 3 * uj) / 4, out p3d, out v1h, out v2h);
                        v3h = v1h ^ v2h;
                        pos.SetX(v3h.X);
                        pos.SetY(v3h.Y);
                        pos.SetZ(v3h.Z);
                        seq3d.Append(pos);
                        // point 1/2 previous
                        _linCont.Value(i).D1((ujp1 + uj) / 2, out p3d, out v1h, out v2h);
                        v3h = v1h ^ v2h;
                        pos.SetX(v3h.X);
                        pos.SetY(v3h.Y);
                        pos.SetZ(v3h.Z);
                        seq3d.Append(pos);
                        // point 3/4 previous
                        _linCont.Value(i).D1((3 * ujp1 + uj) / 4, out p3d, out v1h, out v2h);
                        v3h = v1h ^ v2h;
                        pos.SetX(v3h.X);
                        pos.SetY(v3h.Y);
                        pos.SetZ(v3h.Z);
                        seq3d.Append(pos);
                        // current constraint point
                        _linCont.Value(i).D1(ujp1, out p3d, out v1h, out v2h);
                        v3h = v1h ^ v2h;
                        pos.SetX(v3h.X);
                        pos.SetY(v3h.Y);
                        pos.SetZ(v3h.Z);
                        seq3d.Append(pos);
                    }
                }
            }
        }
    }

    /// <summary>True if the plate was solved (OCCT's plate also reports done before any solve, occt-issues.md).</summary>
    public bool IsDone => _plate.IsDone;

    /// <summary>The plate surface (frozen), or null.</summary>
    public PlateSurface? Surface()
    {
        _geomPlateSurface?.Freeze();
        return _geomPlateSurface;
    }

    /// <summary>The initial surface (frozen), or null.</summary>
    public Surface? SurfInit()
    {
        _surfInit?.Freeze();
        return _surfInit;
    }

    /// <summary>For each curve constraint, in the order of loading, 1 if it is reversed in the contour.</summary>
    public NCollection_HArray1<int> Sense()
    {
        int ntCurve = _linCont.Length();
        NCollection_HArray1<int> sens = new NCollection_HArray1<int>(1, ntCurve);
        for (int i = 1; i <= ntCurve; i++)
        {
            sens.SetValue(i, _sense!.Value(_initOrder!.Value(i)));
        }
        return sens;
    }

    /// <summary>The 2D curves of the curve constraints on the initial surface, in the order of loading (frozen).</summary>
    public NCollection_HArray1<Curve2D?> Curves2d()
    {
        int ntCurve = _linCont.Length();
        NCollection_HArray1<Curve2D?> c2dfin = new NCollection_HArray1<Curve2D?>(1, ntCurve);

        for (int i = 1; i <= ntCurve; i++)
        {
            Curve2D? curve = _linCont.Value(_initOrder!.Value(i)).Curve2dOnSurf();
            curve?.Freeze();
            c2dfin.SetValue(i, curve);
        }
        return c2dfin;
    }

    /// <summary>For each curve constraint in the order of loading, its index in the reordered contour.</summary>
    public NCollection_HArray1<int> Order()
    {
        NCollection_HArray1<int> result = new NCollection_HArray1<int>(1, _linCont.Length());
        for (int i = 1; i <= _linCont.Length(); i++)
        {
            result.SetValue(_initOrder!.Value(i), i);
        }
        return result;
    }

    /// <summary>The largest distance between the surface and the curve constraints (0 before a curve is checked).</summary>
    public double G0Error() => _g0Error;

    /// <summary>The largest angle between the surface and the curve constraints.</summary>
    public double G1Error() => _g1Error;

    /// <summary>The largest curvature gap between the surface and the curve constraints.</summary>
    public double G2Error() => _g2Error;

    /// <summary>The largest distance between the surface and the curve constraint <paramref name="index"/>.</summary>
    /// <exception cref="OutOfRangeException">The constraint has more points than the builder's NbPtsOnCur (OCCT's bug).</exception>
    public double G0Error(int index)
    {
        // OCCT: the arrays have myNbPtsOnCur elements, but EcartContraintesMil writes one per interval of the curve's
        // discretisation, which has more points once Perform raised them: Standard_OutOfRange (occt-issues.md)
        NCollection_HArray1<double> tdistance = new NCollection_HArray1<double>(1, _nbPtsOnCur);
        NCollection_HArray1<double> tangle = new NCollection_HArray1<double>(1, _nbPtsOnCur);
        NCollection_HArray1<double> tcurvature = new NCollection_HArray1<double>(1, _nbPtsOnCur);
        EcartContraintesMil(index, tdistance, tangle, tcurvature);
        double maxDistance = 0.0;
        for (int i = 1; i <= _nbPtsOnCur; i++)
        {
            if (tdistance.Value(i) > maxDistance)
            {
                maxDistance = tdistance.Value(i);
            }
        }
        return maxDistance;
    }

    /// <summary>The largest angle between the surface and the curve constraint <paramref name="index"/>.</summary>
    public double G1Error(int index)
    {
        // OCCT: the arrays have myNbPtsOnCur elements, but EcartContraintesMil writes one per interval of the curve's
        // discretisation, which has more points once Perform raised them: Standard_OutOfRange (occt-issues.md)
        NCollection_HArray1<double> tdistance = new NCollection_HArray1<double>(1, _nbPtsOnCur);
        NCollection_HArray1<double> tangle = new NCollection_HArray1<double>(1, _nbPtsOnCur);
        NCollection_HArray1<double> tcurvature = new NCollection_HArray1<double>(1, _nbPtsOnCur);
        EcartContraintesMil(index, tdistance, tangle, tcurvature);
        double maxAngle = 0.0;
        for (int i = 1; i <= _nbPtsOnCur; i++)
        {
            if (tangle.Value(i) > maxAngle)
            {
                maxAngle = tangle.Value(i);
            }
        }
        return maxAngle;
    }

    /// <summary>The largest curvature gap between the surface and the curve constraint <paramref name="index"/>.</summary>
    public double G2Error(int index)
    {
        // OCCT: the arrays have myNbPtsOnCur elements, but EcartContraintesMil writes one per interval of the curve's
        // discretisation, which has more points once Perform raised them: Standard_OutOfRange (occt-issues.md)
        NCollection_HArray1<double> tdistance = new NCollection_HArray1<double>(1, _nbPtsOnCur);
        NCollection_HArray1<double> tangle = new NCollection_HArray1<double>(1, _nbPtsOnCur);
        NCollection_HArray1<double> tcurvature = new NCollection_HArray1<double>(1, _nbPtsOnCur);
        EcartContraintesMil(index, tdistance, tangle, tcurvature);
        double maxCurvature = 0.0;
        for (int i = 1; i <= _nbPtsOnCur; i++)
        {
            if (tcurvature.Value(i) > maxCurvature)
            {
                maxCurvature = tcurvature.Value(i);
            }
        }
        return maxCurvature;
    }

    /// <summary>
    /// The number of intervals of the last discretisation of the curve constraint <paramref name="index"/>, which
    /// G0Error(Index) to G2Error(Index) fill (Kerf, for the oracle's attribution of OCCT's uninitialised reads).
    /// </summary>
    internal int NbIntervalsChecked(int index)
    {
        int nbPt = _parCont!.Value(index).Length();
        return (nbPt < 3 ? 4 : nbPt) - 1;
    }

    /// <summary>The number of points per curve the builder was given (NbPtsOnCur).</summary>
    internal int PtsOnCurCount => _nbPtsOnCur;

    /// <summary>The curve constraint at <paramref name="order"/> in the (reordered) list.</summary>
    public CurveConstraint CurveConstraint(int order) => _linCont.Value(order);

    /// <summary>The point constraint at <paramref name="order"/>.</summary>
    public PointConstraint PointConstraint(int order) => _pntCont.Value(order);

    // Create a chain of curves to calculate the initial surface with the method of max flow.
    // Return true if it is a closed contour.
    private bool CourbeJointive(double tolerance)
    {
        int nbf = _linCont.Length();
        double ufinal1, uinit1, ufinal2, uinit2;
        _sense = new NCollection_HArray1<int>(1, nbf, 0);
        bool result = true;
        int j = 1, i;
        Point3D p1, p2;

        while (j <= (_nbBounds - 1))
        {
            int a = 0;
            i = j + 1;
            if (i > _nbBounds)
            {
                result = false;
                a = 2;
            }
            while (a < 1)
            {
                if (i > _nbBounds)
                {
                    result = false;
                    a = 2;
                }
                else
                {
                    uinit1 = _linCont.Value(j).FirstParameter();
                    ufinal1 = _linCont.Value(j).LastParameter();
                    uinit2 = _linCont.Value(i).FirstParameter();
                    ufinal2 = _linCont.Value(i).LastParameter();
                    if (_sense.Value(j) == 1)
                    {
                        ufinal1 = uinit1;
                    }
                    _linCont.Value(j).D0(ufinal1, out p1);
                    _linCont.Value(i).D0(uinit2, out p2);
                    if (p1.DistanceTo(p2) < tolerance)
                    {
                        if (i != j + 1)
                        {
                            CurveConstraint tampon = _linCont.Value(j + 1);
                            _linCont.SetValue(j + 1, _linCont.Value(i));
                            _linCont.SetValue(i, tampon);
                            int tmp = _initOrder!.Value(j + 1);
                            // See function TrierTab for the functioning of myInitOrder
                            _initOrder.SetValue(j + 1, _initOrder.Value(i));
                            _initOrder.SetValue(i, tmp);
                        }
                        a = 2;
                        _sense.SetValue(j + 1, 0);
                    }
                    else
                    {
                        _linCont.Value(i).D0(ufinal2, out p2);

                        if (p1.DistanceTo(p2) < tolerance)
                        {
                            if (i != j + 1)
                            {
                                CurveConstraint tampon = _linCont.Value(j + 1);
                                _linCont.SetValue(j + 1, _linCont.Value(i));
                                _linCont.SetValue(i, tampon);
                                int tmp = _initOrder!.Value(j + 1);
                                // See function TrierTab for the functioning of myInitOrder
                                _initOrder.SetValue(j + 1, _initOrder.Value(i));
                                _initOrder.SetValue(i, tmp);
                            }
                            a = 2;
                            _sense.SetValue(j + 1, 1);
                        }
                    }
                }
                i++;
            }
            j++;
        }
        uinit1 = _linCont.Value(_nbBounds).FirstParameter();
        ufinal1 = _linCont.Value(_nbBounds).LastParameter();
        uinit2 = _linCont.Value(1).FirstParameter();
        ufinal2 = _linCont.Value(1).LastParameter();
        _linCont.Value(_nbBounds).D0(ufinal1, out p1);
        _linCont.Value(1).D0(uinit2, out p2);
        if ((_sense.Value(_nbBounds) == 0) && (p1.DistanceTo(p2) < tolerance))
        {
            return result;
        }
        _linCont.Value(_nbBounds).D0(uinit1, out p1);
        if ((_sense.Value(_nbBounds) == 1) && (p1.DistanceTo(p2) < tolerance))
        {
            return result;
        }
        else
        {
            return false;
        }
    }

    // Calculate the initial surface either by the method of max flow or by
    // the method of the plane of inertia if the contour is not closed or if
    // there are point constraints.
    private void ComputeSurfInit(ProgressRange progress)
    {
        int nopt = 2, popt = 2, np = 1;
        bool isHalfSpace = true;
        double linTol = 0.001, angTol = 0.001; // AngTol = 0.0001; //LinTol = 0.0001

        // Option to calculate the initial plane
        int ntLinCont = _linCont.Length(), ntPntCont = _pntCont.Length();
        // Table of transformation to preserve the initial order see TrierTab
        if (ntLinCont != 0)
        {
            _initOrder = new NCollection_HArray1<int>(1, ntLinCont);
            for (int i = 1; i <= ntLinCont; i++)
            {
                _initOrder.SetValue(i, i);
            }
        }

        bool courbeJoint = (ntLinCont != 0) && CourbeJointive(_tol3d);
        if (courbeJoint && IsOrderG1())
        {
            nopt = 3;
            // Table contains the cloud of points for calculation of the plane
            int nbPoint = 20, discr = nbPoint / 4, pnum = 0;
            NCollection_HArray1<Point3D> pts = new NCollection_HArray1<Point3D>(1, (nbPoint + 1) * ntLinCont + ntPntCont);
            NCollection_Sequence<Vector3D> vecs = new NCollection_Sequence<Vector3D>(), newVecs = new NCollection_Sequence<Vector3D>();
            NCollection_Sequence<Aij> aset = new NCollection_Sequence<Aij>();
            double uinit, ufinal, uif;
            Vector3D lastVec = new Vector3D();
            int i;
            for (i = 1; i <= ntLinCont; i++)
            {
                int order = _linCont.Value(i).Order();

                newVecs.Clear();

                uinit = _linCont.Value(i).FirstParameter();
                ufinal = _linCont.Value(i).LastParameter();
                uif = ufinal - uinit;
                if (_sense!.Value(i) == 1)
                {
                    uinit = ufinal;
                    uif = -uif;
                }

                Vector3D vec1, vec2, normal = new Vector3D();
                bool toReverse = false;
                if (i > 1 && order >= (int)G1)
                {
                    Point3D p;
                    _linCont.Value(i).D1(uinit, out p, out vec1, out vec2);
                    normal = vec1 ^ vec2;
                    if (lastVec.IsOpposite(normal, angTol))
                    {
                        toReverse = true;
                    }
                }

                for (int j = 0; j <= nbPoint; j++)
                { // Number of points per curve = 20
                  // Linear distribution
                    double inter = j * uif / (nbPoint);
                    if (order < (int)G1 || j % discr != 0)
                    {
                        _linCont.Value(i).D0(uinit + inter, out pts.ChangeValue(++pnum));
                    }
                    else
                    {
                        _linCont.Value(i).D1(uinit + inter, out pts.ChangeValue(++pnum), out vec1, out vec2);
                        normal = vec1 ^ vec2;
                        normal = normal.Normalized();
                        if (toReverse)
                        {
                            normal = normal.Reversed();
                        }
                        bool isNew = true;
                        int k;
                        for (k = 1; k <= vecs.Length(); k++)
                        {
                            if (vecs.Value(k).IsEqual(normal, linTol, angTol))
                            {
                                isNew = false;
                                break;
                            }
                        }
                        if (isNew)
                        {
                            for (k = 1; k <= newVecs.Length(); k++)
                            {
                                if (newVecs.Value(k).IsEqual(normal, linTol, angTol))
                                {
                                    isNew = false;
                                    break;
                                }
                            }
                        }
                        if (isNew)
                        {
                            newVecs.Append(normal);
                        }
                    }
                }
                if (order >= (int)G1)
                {
                    isHalfSpace = BuildAveragePlane.HalfSpace(newVecs, vecs, aset, linTol, angTol);
                    if (!isHalfSpace)
                    {
                        break;
                    }
                    lastVec = normal;
                }
            } // for (i = 1; i <= NTLinCont; i++)

            if (isHalfSpace)
            {
                for (i = 1; i <= ntPntCont; i++)
                {
                    int order = _pntCont.Value(i).Order();

                    newVecs.Clear();
                    Vector3D vec1, vec2, normal;
                    if (order < (int)G1)
                    {
                        _pntCont.Value(i).D0(out pts.ChangeValue(++pnum));
                    }
                    else
                    {
                        _pntCont.Value(i).D1(out pts.ChangeValue(++pnum), out vec1, out vec2);
                        normal = vec1 ^ vec2;
                        normal = normal.Normalized();
                        bool isNew = true;
                        for (int k = 1; k <= vecs.Length(); k++)
                        {
                            if (vecs.Value(k).IsEqual(normal, linTol, angTol))
                            {
                                isNew = false;
                                break;
                            }
                        }
                        if (isNew)
                        {
                            newVecs.Append(normal);
                            isHalfSpace = BuildAveragePlane.HalfSpace(newVecs, vecs, aset, linTol, angTol);
                            if (!isHalfSpace)
                            {
                                newVecs.ChangeValue(1) = newVecs.ChangeValue(1).Reversed();
                                isHalfSpace = BuildAveragePlane.HalfSpace(newVecs, vecs, aset, linTol, angTol);
                            }
                            if (!isHalfSpace)
                            {
                                break;
                            }
                        }
                    }
                } // for (i = 1; i <= NTPntCont; i++)

                if (isHalfSpace)
                {
                    bool nullExist = true;
                    while (nullExist)
                    {
                        nullExist = false;
                        for (i = 1; i <= vecs.Length(); i++)
                        {
                            if (vecs.Value(i).SquareMagnitude == 0.0)
                            {
                                nullExist = true;
                                vecs.Remove(i);
                                break;
                            }
                        }
                    }
                    BuildAveragePlane bap = new BuildAveragePlane(vecs, pts);
                    double u1, u2, v1, v2;
                    bap.MinMaxBox(out u1, out u2, out v1, out v2);
                    // The space is greater for projections
                    double du = u2 - u1;
                    double dv = v2 - v1;
                    u1 -= du;
                    u2 += du;
                    v1 -= dv;
                    v2 += dv;
                    _surfInit = new TrimmedSurface(bap.Plane()!, u1, u2, v1, v2);
                }
            } // if (isHalfSpace)
            if (!isHalfSpace)
            {
                _isLinear = false;
                nopt = 2;
            }
        } // if (NTLinCont != 0 && (CourbeJoint = CourbeJointive( myTol3d )) && IsOrderG1())

        if (ntLinCont != 0)
        {
            TrierTab(_initOrder!); // Reorder the table of transformations
        }

        if (nopt != 3)
        {
            if (ntPntCont != 0)
            {
                nopt = 1; // Calculate by the method of plane of inertia
            }
            else if (!courbeJoint || ntLinCont != _nbBounds)
            { //    throw Standard_Failure("Curves are not joined");
                nopt = 1;
            }

            double lenT = 0;
            int npt = 0;
            int ntPoint = 20 * ntLinCont;
            int i;
            for (i = 1; i <= ntLinCont; i++)
            {
                lenT += _linCont.Value(i).Length();
            }
            for (i = 1; i <= ntLinCont; i++)
            {
                int nbPoint1 = (int)(ntPoint * (_linCont.Value(i).Length()) / lenT);
                if (nbPoint1 < 10)
                {
                    nbPoint1 = 10;
                }

                npt += nbPoint1; // unused but set for debug
            }
            _ = npt;
            // Table containing a cloud of points for calculation of the plane
            NCollection_HArray1<Point3D> pts = new NCollection_HArray1<Point3D>(1, 20 * ntLinCont + ntPntCont);
            int nbPoint = 20;
            double uinit, ufinal, uif;
            for (i = 1; i <= ntLinCont; i++)
            {
                uinit = _linCont.Value(i).FirstParameter();
                ufinal = _linCont.Value(i).LastParameter();
                uif = ufinal - uinit;
                if (_sense!.Value(i) == 1)
                {
                    uinit = ufinal;
                    uif = -uif;
                }
                for (int j = 0; j < nbPoint; j++)
                { // Number of points per curve = 20
                  // Linear distribution
                    double inter = j * uif / (nbPoint);
                    Point3D p;
                    _linCont.Value(i).D0(uinit + inter, out p);
                    pts.SetValue(np++, p);
                }
            }
            for (i = 1; i <= ntPntCont; i++)
            {
                Point3D p;
                _pntCont.Value(i).D0(out p);
                pts.SetValue(np++, p);
            }
            if (!courbeJoint)
            {
                _nbBounds = 0;
            }
            BuildAveragePlane bap = new BuildAveragePlane(pts, nbPoint * _nbBounds, _tol3d / 1000, popt, nopt);
            if (!bap.IsPlane())
            {
                return;
            }
            double u1, u2, v1, v2;
            bap.MinMaxBox(out u1, out u2, out v1, out v2);
            // The space is greater for projections
            double du = u2 - u1;
            double dv = v2 - v1;
            u1 -= du;
            u2 += du;
            v1 -= dv;
            v2 += dv;
            _surfInit = new TrimmedSurface(bap.Plane()!, u1, u2, v1, v2);
        } // if (nopt != 3)

        // Comparing metrics of curves and projected curves
        if (ntLinCont != 0 && _isLinear)
        {
            Surface initPlane = ((TrimmedSurface)_surfInit!).BasisSurface;

            double ratio = 0.0, r1 = 2.0, r2 = 0.6; //R1 = 3, R2 = 0.5;//R1 = 1.4, R2 = 0.8; //R1 = 5., R2 = 0.2;
            SurfaceAdaptor hsur = new SurfaceAdaptor(initPlane);
            int nbPoint = 20;

            for (int i = 1; i <= ntLinCont && _isLinear; i++)
            {
                double firstPar = _linCont.Value(i).FirstParameter();
                double lastPar = _linCont.Value(i).LastParameter();
                double uif = (lastPar - firstPar) / (nbPoint);

                CurveAdaptorBase curve = _linCont.Value(i).Curve3d();
                CompProjectedCurve projCurve = new CompProjectedCurve(hsur, curve, _tol3d, _tol3d);
                CurveOnSurface aProj = new CurveOnSurface(projCurve, hsur);

                Point3D p;
                Vector3D derC, derCproj;
                for (int j = 1; j < nbPoint && _isLinear; j++)
                {
                    double inter = firstPar + j * uif;
                    curve.D1(inter, out p, out derC);
                    aProj.D1(inter, out p, out derCproj);

                    double a1 = derC.Magnitude;
                    double a2 = derCproj.Magnitude;
                    if (a2 <= 1.0e-20)
                    {
                        ratio = 1.0e20;
                    }
                    else
                    {
                        ratio = a1 / a2;
                    }
                    if (ratio > r1 || ratio < r2)
                    {
                        _isLinear = false;
                        break;
                    }
                }
            }
        } // comparing metrics of curves and projected curves

        if (!_isLinear)
        {
            _planarSurfInit = _surfInit;
            double u1, v1, u2, v2;
            _surfInit!.ParameterBounds(out u1, out v1, out u2, out v2);
            SurfaceAdaptor surf = new SurfaceAdaptor(_surfInit);
            _tolU = surf.UResolution(_tol3d);
            _tolV = surf.VResolution(_tol3d);
            _proj.Initialize(surf, u1, v1, u2, v2, _tolU, _tolV);

            //======================================================================
            // Projection of curves
            //======================================================================
            int i;
            for (i = 1; i <= ntLinCont; i++)
            {
                if (_linCont.Value(i).Curve2dOnSurf() is null)
                {
                    _linCont.ChangeValue(i).SetCurve2dOnSurf(ProjectCurve(_linCont.Value(i).Curve3d()));
                }
            }

            //======================================================================
            // Projection of points
            //======================================================================
            for (i = 1; i <= ntPntCont; i++)
            {
                Point3D p;
                _pntCont.Value(i).D0(out p);
                if (!_pntCont.Value(i).HasPnt2dOnSurf)
                {
                    _pntCont.ChangeValue(i).SetPnt2dOnSurf(ProjectPoint(p));
                }
            }

            //======================================================================
            // Number of points by curve
            //======================================================================
            if ((ntLinCont != 0) && (_nbPtsOnCur != 0))
            {
                CalculNbPtsInit();
            }

            //======================================================================
            // Management of incompatibilities between curves
            //======================================================================
            NCollection_HArray1<NCollection_Sequence<double>>? pntInter = null;
            NCollection_HArray1<NCollection_Sequence<double>>? pntG1G1 = null;
            if (ntLinCont != 0)
            {
                pntInter = NewSequences(ntLinCont);
                pntG1G1 = NewSequences(ntLinCont);
                Intersect(pntInter, pntG1G1);
            }

            //====================================================================
            // Discretization of curves
            //====================================================================
            Discretise(pntInter!, pntG1G1!);
            //====================================================================
            // Preparation of points of constraint for plate
            //====================================================================
            LoadCurve(0, 0);
            if (_pntCont.Length() != 0)
            {
                LoadPoint(0, 0);
            }
            //====================================================================
            // Construction of the surface
            //====================================================================
            using ProgressScope ps = new ProgressScope(progress, "ComputeSurfInit", 1);
            _plate.SolveTI(2, ComputeAnisotropie(), ps.Next());
            if (progress.UserBreak())
            {
                return;
            }

            if (!_plate.IsDone)
            {
                return;
            }

            _geomPlateSurface = new PlateSurface(_surfInit, _plate);

            MakeApprox app = new MakeApprox(_geomPlateSurface, _tol3d, 1, 3, 15 * _tol3d, -1, C0);
            _surfInit = app.Surface();

            _surfInitIsGive = true;
            _plate.Init(); // Reset

            for (i = 1; i <= ntLinCont; i++)
            {
                Curve2D? nullCurve = null;
                _linCont.ChangeValue(i).SetCurve2dOnSurf(nullCurve);
            }
        }
    }

    // Find intersections between 2d curves
    // If the intersection is compatible (in cases G1-G1)
    // remove the point on one of two curves
    private void Intersect(NCollection_HArray1<NCollection_Sequence<double>> pntInter,
                           NCollection_HArray1<NCollection_Sequence<double>> pntG1G1)
    {
        int ntLinCont = _linCont.Length();
        GInter intersection = new GInter();
        CurveAdaptor2D ci = new CurveAdaptor2D(), cj = new CurveAdaptor2D();
        IntersectionPoint2D int2d;
        Point3D p1, p2;
        Point2D p2d;
        Vector2D v2d;
        for (int i = 1; i <= ntLinCont; i++)
        {
            //  Find the intersection with each curve including the curve itself
            ci.Load(_linCont.Value(i).Curve2dOnSurf()!);
            for (int j = i; j <= ntLinCont; j++)
            {
                cj.Load(_linCont.Value(j).Curve2dOnSurf()!);
                if (i == j)
                {
                    intersection.Perform(ci, _tol2d * 10, _tol2d * 10);
                }
                else
                {
                    intersection.Perform(ci, cj, _tol2d * 10, _tol2d * 10);
                }

                if (!intersection.IsEmpty)
                { // there is one intersection
                    int nbpt = intersection.PointCount;
                    // number of points of intersection
                    for (int k = 1; k <= nbpt; k++)
                    {
                        int2d = intersection.Point(k);
                        _linCont.Value(i).D0(int2d.ParamOnFirst(), out p1);
                        _linCont.Value(j).D0(int2d.ParamOnSecond(), out p2);
                        if (p1.DistanceTo(p2) < _tol3d)
                        { // 2D intersection corresponds to close 3D points.
                          // Note the interval, in which the point needs to be removed
                          // to avoid duplications, which cause
                          // error in plate. The point on curve i is removed;
                          // the point on curve j is preserved;
                          // the length of interval is a length 2d
                          // corresponding in 3d to myTol3d
                            double tolint = ci.Resolution(_tol3d);
                            ci.D1(int2d.ParamOnFirst(), out p2d, out v2d);
                            double aux = v2d.Magnitude;
                            if (aux > 1.0e-7)
                            {
                                aux = _tol3d / aux;
                                if (aux > 100 * tolint)
                                {
                                    tolint *= 100;
                                }
                                else
                                {
                                    tolint = aux;
                                }
                            }
                            else
                            {
                                tolint *= 100;
                            }

                            pntInter[i].Append(int2d.ParamOnFirst() - tolint);
                            pntInter[i].Append(int2d.ParamOnFirst() + tolint);
                            // If G1-G1
                            if ((_linCont.Value(i).Order() == 1) && (_linCont.Value(j).Order() == 1))
                            {
                                Vector3D v11, v12, v13, v14, v15, v16, v21, v22, v23, v24, v25, v26;
                                _linCont.Value(i).D2(int2d.ParamOnFirst(), out p1, out v11, out v12, out v13, out v14, out v15);
                                _linCont.Value(j).D2(int2d.ParamOnSecond(), out p2, out v21, out v22, out v23, out v24, out v25);
                                v16 = v11 ^ v12;
                                v26 = v21 ^ v22;
                                double ant = v16.AngleTo(v26);
                                if (ant > (M_PI / 2))
                                {
                                    ant = M_PI - ant;
                                }
                                if ((Math.Abs(v16 * v15 - v16 * v25) > (_tol3d / 1000))
                                    || (Math.Abs(ant) > _tol3d / 1000))
                                // Non-compatible ==> remove zone in constraint G1
                                // corresponding to 3D tolerance of 0.01
                                {
                                    double coin;
                                    double tol = 100 * _tol3d;
                                    double a1;
                                    Point2D p1temp, p2temp;
                                    Vector2D v1, v2;
                                    _linCont.Value(i).Curve2dOnSurf()!.D1(int2d.ParamOnFirst(), out p1temp, out v1);
                                    _linCont.Value(j).Curve2dOnSurf()!.D1(int2d.ParamOnSecond(), out p2temp, out v2);
                                    a1 = v1.AngleTo(v2);
                                    if (a1 > (M_PI / 2))
                                    {
                                        a1 = M_PI - a1;
                                    }
                                    if (Math.Abs(Math.Abs(a1) - M_PI) < _tolAng)
                                    {
                                        tol = 100000 * _tol3d;
                                    }

                                    coin = ci.Resolution(tol);
                                    double par1 = int2d.ParamOnFirst() - coin, par2 = int2d.ParamOnFirst() + coin;
                                    // Storage of the interval for curve i
                                    pntG1G1[i].Append(par1);
                                    pntG1G1[i].Append(par2);
                                    coin = cj.Resolution(tol);
                                    par1 = int2d.ParamOnSecond() - coin;
                                    par2 = int2d.ParamOnSecond() + coin;
                                    // Storage of the interval for curve j
                                    pntG1G1[j].Append(par1);
                                    pntG1G1[j].Append(par2);
                                }
                            }
                            // If G0-G1
                            if ((_linCont.Value(i).Order() == 0 && _linCont.Value(j).Order() == 1)
                                || (_linCont.Value(i).Order() == 1 && _linCont.Value(j).Order() == 0))
                            {
                                Vector3D vec, vecU, vecV, n;
                                if (_linCont.Value(i).Order() == 0)
                                {
                                    CurveAdaptorBase theCurve = _linCont.Value(i).Curve3d();
                                    theCurve.D1(int2d.ParamOnFirst(), out p1, out vec);
                                    _linCont.Value(j).D1(int2d.ParamOnSecond(), out p2, out vecU, out vecV);
                                }
                                else
                                {
                                    CurveAdaptorBase theCurve = _linCont.Value(j).Curve3d();
                                    theCurve.D1(int2d.ParamOnSecond(), out p2, out vec);
                                    _linCont.Value(i).D1(int2d.ParamOnFirst(), out p1, out vecU, out vecV);
                                }
                                n = vecU ^ vecV;
                                double angle = vec.AngleTo(n);
                                angle = Math.Abs(M_PI / 2 - angle);
                                if (angle > _tolAng / 10.0) //????????? //if (std::abs( scal ) > myTol3d/100)
                                {
                                    // Non-compatible ==> one removes zone in constraint G0 and G1
                                    // corresponding to 3D tolerance of 0.01
                                    double coin;
                                    double tol = 100 * _tol3d;
                                    double a1;
                                    Point2D p1temp, p2temp;
                                    Vector2D v1, v2;
                                    _linCont.Value(i).Curve2dOnSurf()!.D1(int2d.ParamOnFirst(), out p1temp, out v1);
                                    _linCont.Value(j).Curve2dOnSurf()!.D1(int2d.ParamOnSecond(), out p2temp, out v2);
                                    a1 = v1.AngleTo(v2);
                                    if (a1 > M_PI / 2)
                                    {
                                        a1 = M_PI - a1;
                                    }
                                    if (Math.Abs(Math.Abs(a1) - M_PI) < _tolAng)
                                    {
                                        tol = 100000 * _tol3d;
                                    }
                                    if (_linCont.Value(i).Order() == 1)
                                    {
                                        coin = ci.Resolution(tol);
                                        coin *= angle / _tolAng * 10.0;
                                        double par1 = int2d.ParamOnFirst() - coin;
                                        double par2 = int2d.ParamOnFirst() + coin;
                                        // Storage of the interval for curve i
                                        pntG1G1[i].Append(par1);
                                        pntG1G1[i].Append(par2);
                                    }
                                    else
                                    {
                                        coin = cj.Resolution(tol);
                                        coin *= angle / _tolAng * 10.0;
                                        double par1 = int2d.ParamOnSecond() - coin;
                                        double par2 = int2d.ParamOnSecond() + coin;
                                        // Storage of the interval for curve j
                                        pntG1G1[j].Append(par1);
                                        pntG1G1[j].Append(par2);
                                    }
                                }
                            }
                        } // if (P1.Distance( P2 ) < myTol3d)
                        else
                        {
                            // 2D intersection corresponds to extended 3D points.
                            // Note the interval where it is necessary to remove
                            // the points to avoid duplications causing
                            // error in plate. The point on curve i is removed,
                            // the point on curve j is preserved.
                            // The length of interval is 2D length
                            // corresponding to the distance of points in 3D to myTol3d
                            double tolint, dist;
                            dist = p1.DistanceTo(p2);
                            tolint = ci.Resolution(dist);
                            pntInter[i].Append(int2d.ParamOnFirst() - tolint);
                            pntInter[i].Append(int2d.ParamOnFirst() + tolint);
                            if (j != i)
                            {
                                tolint = cj.Resolution(dist);
                                pntInter[j].Append(int2d.ParamOnSecond() - tolint);
                                pntInter[j].Append(int2d.ParamOnSecond() + tolint);
                            }
                        }
                    }
                }
            }
        }
    }

    // Discretize curves according to parameters
    // the table of sequences Parcont contains all
    // parameter of points on curves
    // Field myPlateCont contains parameter of points on a plate;
    // it excludes duplicate points and incompatible zones.
    // The first part corresponds to verification of compatibility
    // and to removal of duplicate points.
    private void Discretise(NCollection_HArray1<NCollection_Sequence<double>> pntInter,
                            NCollection_HArray1<NCollection_Sequence<double>> pntG1G1)
    {
        int ntLinCont = _linCont.Length();
        bool acr;
        Curve2D? c2d;
        CurveAdaptor2D ac2d = new CurveAdaptor2D();
        Interpol acrlaw = new Interpol();
        _plateCont = NewSequences(ntLinCont);
        _parCont = NewSequences(ntLinCont);

        //===========================================================================
        // Construction of the table containing parameters of constraint points
        //===========================================================================
        double uinit, ufinal, length2d = 0, inter;
        double curLength;
        int NbPnt_i, nbPtInter, nbPtG1G1;
        CurveConstraint linCont;

        for (int i = 1; i <= ntLinCont; i++)
        {
            linCont = _linCont.Value(i);
            uinit = linCont.FirstParameter();
            ufinal = linCont.LastParameter();
            c2d = linCont.Curve2dOnSurf();
            acr = (c2d is not null);
            if (acr)
            {
                // Construct a law close to curvilinear abscissa
                if (c2d is not null)
                {
                    ac2d.Load(c2d);
                }
                int ii, nbint = 20;
                double u;
                NCollection_Array1<Point2D> tabP2d = new NCollection_Array1<Point2D>(1, nbint + 1);
                tabP2d[1] = tabP2d[1].WithY(uinit);
                tabP2d[1] = tabP2d[1].WithX(0.0);
                tabP2d[nbint + 1] = tabP2d[nbint + 1].WithY(ufinal);
                length2d = AbscissaPoint.Length(ac2d, uinit, ufinal);

                tabP2d[nbint + 1] = tabP2d[nbint + 1].WithX(length2d);
                for (ii = 2; ii <= nbint; ii++)
                {
                    u = uinit + (ufinal - uinit) * ((1 - Math.Cos((ii - 1) * M_PI / (nbint))) / 2);
                    tabP2d[ii] = tabP2d[ii].WithY(u);
                    tabP2d[ii] = tabP2d[ii].WithX(AbscissaPoint.Length(ac2d, uinit, u));
                }
                acrlaw.Set(tabP2d);
            }

            NbPnt_i = (int)(linCont.PointCount);
            nbPtInter = pntInter.Value(i).Length();
            nbPtG1G1 = pntG1G1.Value(i).Length();

            for (int j = 1; j <= NbPnt_i; j++)
            {
                // Distribution of points in cosine following ACR 2D
                // To avoid points of accumulation in 2D
                if (j == NbPnt_i)
                {
                    inter = ufinal; // to avoid bug on Sun
                }
                else if (acr)
                {
                    curLength = length2d * (1 - Math.Cos((j - 1) * M_PI / (NbPnt_i - 1))) / 2;
                    inter = acrlaw.Value(curLength);
                }
                else
                {
                    inter = uinit + (ufinal - uinit) * ((1 - Math.Cos((j - 1) * M_PI / (NbPnt_i - 1))) / 2);
                }
                _parCont[i].Append(inter); // add a point
                if (nbPtInter != 0)
                {
                    for (int l = 1; l <= nbPtInter; l += 2)
                    {
                        // check if the point Inter is in the interval
                        // PntInter[i] PntInter[i+1]
                        // in which case it is not necessary to store it (problem with duplicates)
                        if ((inter > pntInter.Value(i).Value(l)) && (inter < pntInter.Value(i).Value(l + 1)))
                        {
                            l = nbPtInter + 2;
                            // leave the loop without storing the point
                        }
                        else
                        {
                            if (l + 1 >= nbPtInter)
                            {
                                // one has parsed the entire table : the point
                                // does not belong to a common point interval
                                if (nbPtG1G1 != 0)
                                {
                                    // if there exists an incompatible interval
                                    for (int k = 1; k <= nbPtG1G1; k += 2)
                                    {
                                        if ((inter > pntG1G1.Value(i).Value(k)) && (inter < pntG1G1.Value(i).Value(k + 1)))
                                        {
                                            k = nbPtG1G1 + 2; // to leave the loop
                                            // Add points of constraint G0
                                            Point3D p3d, pp, pdif = new Point3D();
                                            Point2D p2d;

                                            ac2d.D0(inter, out p2d);
                                            linCont.D0(inter, out p3d);
                                            _surfInit!.D0(p2d.Coord(1), p2d.Coord(2), out pp);
                                            pdif = pdif.WithCoord(-pp.Coord(1) + p3d.Coord(1),
                                                          -pp.Coord(2) + p3d.Coord(2),
                                                          -pp.Coord(3) + p3d.Coord(3));
                                            PinpointConstraint pc = new PinpointConstraint(p2d.XY(), pdif.XYZ(), 0, 0);
                                            _plate.Load(pc);
                                        }
                                        else // the point does not belong to interval G1
                                        {
                                            if (k + 1 >= nbPtG1G1)
                                            {
                                                _plateCont[i].Append(inter);
                                                // add the point
                                            }
                                        }
                                    }
                                }
                                else
                                {
                                    _plateCont[i].Append(inter);
                                    // add the point
                                }
                            }
                        }
                    }
                }
                else
                {
                    if (nbPtG1G1 != 0) // there exist an incompatible interval
                    {
                        for (int k = 1; k <= nbPtG1G1; k += 2)
                        {
                            if ((inter > pntG1G1.Value(i).Value(k)) && (inter < pntG1G1.Value(i).Value(k + 1)))
                            {
                                k = nbPtG1G1 + 2; // to leave the loop
                                // Add points of constraint G0
                                Point3D p3d, pp, pdif = new Point3D();
                                Point2D p2d;

                                ac2d.D0(inter, out p2d);
                                linCont.D0(inter, out p3d);
                                _surfInit!.D0(p2d.Coord(1), p2d.Coord(2), out pp);
                                pdif = pdif.WithCoord(-pp.Coord(1) + p3d.Coord(1),
                                              -pp.Coord(2) + p3d.Coord(2),
                                              -pp.Coord(3) + p3d.Coord(3));
                                PinpointConstraint pc = new PinpointConstraint(p2d.XY(), pdif.XYZ(), 0, 0);
                                _plate.Load(pc);
                            }
                            else // the point does not belong to interval G1
                            {
                                if (k + 1 >= nbPtG1G1)
                                {
                                    _plateCont[i].Append(inter);
                                    // add the point
                                }
                            }
                        }
                    }
                    else
                    {
                        if (((!_surfInitIsGive)
                             && (new CurveAdaptor2D(linCont.Curve2dOnSurf()!).GetType() != CurveKind.Circle))
                            || ((j > 1) && (j < NbPnt_i)))
                        { // exclude extremities
                            _plateCont[i].Append(inter); // add the point
                        }
                    }
                }
            }
        }
    }

    // Calculate the number of points by curve depending on the
    // length for the first iteration
    private void CalculNbPtsInit()
    {
        double lenT = 0;
        int ntLinCont = _linCont.Length();
        int ntPoint = (int)(_nbPtsOnCur * ntLinCont);
        int i;

        for (i = 1; i <= ntLinCont; i++)
        {
            lenT += _linCont.Value(i).Length();
        }
        for (i = 1; i <= ntLinCont; i++)
        {
            int cont = _linCont.Value(i).Order();
            switch (cont)
            {
                case 0: // Case G0 *1.2
                    _linCont.ChangeValue(i).SetNbPoints((int)(1.2 * ntPoint * (_linCont.Value(i).Length()) / lenT));
                    break;
                case 1: // Case G1 *1
                    _linCont.ChangeValue(i).SetNbPoints((int)(ntPoint * (_linCont.Value(i).Length()) / lenT));
                    break;
                case 2: // Case G2 *0.7
                    _linCont.ChangeValue(i).SetNbPoints((int)(0.7 * ntPoint * (_linCont.Value(i).Length()) / lenT));
                    break;
            }
            if (_linCont.Value(i).PointCount < 3)
            {
                _linCont.ChangeValue(i).SetNbPoints(3);
            }
        }
    }

    // Starting from table myParCont load all the points noted in plate
    private void LoadCurve(int nbBoucle, int orderMax = 2)
    {
        Point3D p3d, pdif = new Point3D(), pp;
        Point2D p2d;
        int ntLinCont = _linCont.Length(), i, j;
        int tang, nt;

        for (i = 1; i <= ntLinCont; i++)
        {
            CurveConstraint cc = _linCont.Value(i);
            if (cc.Order() != -1)
            {
                tang = Min(cc.Order(), orderMax);
                nt = _plateCont!.Value(i).Length();
                if (tang != -1)
                {
                    for (j = 1; j <= nt; j++)
                    { // Loading of points G0 on boundaries
                        cc.D0(_plateCont.Value(i).Value(j), out p3d);
                        if (cc.ProjectedCurve() is not null)
                        {
                            p2d = cc.ProjectedCurve()!.Value(_plateCont.Value(i).Value(j));
                        }
                        else
                        {
                            if (cc.Curve2dOnSurf() is not null)
                            {
                                p2d = cc.Curve2dOnSurf()!.Value(_plateCont.Value(i).Value(j));
                            }
                            else
                            {
                                p2d = ProjectPoint(p3d);
                            }
                        }
                        _surfInit!.D0(p2d.Coord(1), p2d.Coord(2), out pp);
                        pdif = pdif.WithCoord(-pp.Coord(1) + p3d.Coord(1),
                                      -pp.Coord(2) + p3d.Coord(2),
                                      -pp.Coord(3) + p3d.Coord(3));
                        PinpointConstraint pc = new PinpointConstraint(p2d.XY(), pdif.XYZ(), 0, 0);
                        _plate.Load(pc);

                        // Loading of points G1
                        if (tang == 1)
                        { // ==1
                            Vector3D v1, v2, v3, v4;
                            cc.D1(_plateCont.Value(i).Value(j), out pp, out v1, out v2);
                            _surfInit.D1(p2d.Coord(1), p2d.Coord(2), out pp, out v3, out v4);

                            D1 d1final = new D1(v1.XYZ(), v2.XYZ());
                            D1 d1init = new D1(v3.XYZ(), v4.XYZ());
                            if (!_free)
                            {
                                GtoCConstraint gcc = new GtoCConstraint(p2d.XY(), d1init, d1final);
                                _plate.Load(gcc);
                            }
                            else if (nbBoucle == 1)
                            {
                                FreeGtoCConstraint freeGCC = new FreeGtoCConstraint(p2d.XY(), d1init, d1final);
                                _plate.Load(freeGCC);
                            }
                            else
                            {
                                Vector3D du = new Vector3D(), dv = new Vector3D(), normal, derPlateU, derPlateV;

                                normal = v1 ^ v2;
                                // Normal.Normalize();
                                double norm = normal.Magnitude;
                                if (norm > 1.0e-12)
                                {
                                    normal /= norm;
                                }
                                derPlateU = new Vector3D(_prevPlate.EvaluateDerivative(p2d.XY(), 1, 0));
                                derPlateV = new Vector3D(_prevPlate.EvaluateDerivative(p2d.XY(), 0, 1));

                                du = Vector3D.LinearForm(-(v3 + derPlateU).Dot(normal), normal, derPlateU);
                                dv = Vector3D.LinearForm(-(v4 + derPlateV).Dot(normal), normal, derPlateV);
                                PinpointConstraint pinU = new PinpointConstraint(p2d.XY(), du.XYZ(), 1, 0);
                                PinpointConstraint pinV = new PinpointConstraint(p2d.XY(), dv.XYZ(), 0, 1);
                                _plate.Load(pinU);
                                _plate.Load(pinV);
                            }
                        }
                        // Loading of points G2
                        if (tang == 2) // ==2
                        {
                            Vector3D v1, v2, v3, v4, v5, v6, v7, v8, v9, v10;
                            cc.D2(_plateCont.Value(i).Value(j), out pp, out v1, out v2, out v5, out v6, out v7);
                            _surfInit.D2(p2d.Coord(1), p2d.Coord(2), out pp, out v3, out v4, out v8, out v9, out v10);

                            D1 d1final = new D1(v1.XYZ(), v2.XYZ());
                            D1 d1init = new D1(v3.XYZ(), v4.XYZ());
                            D2 d2final = new D2(v5.XYZ(), v6.XYZ(), v7.XYZ());
                            D2 d2init = new D2(v8.XYZ(), v9.XYZ(), v10.XYZ());
                            GtoCConstraint gcc = new GtoCConstraint(p2d.XY(), d1init, d1final, d2init, d2final);
                            _plate.Load(gcc);
                        }
                    }
                }
            }
        }
    }

    // Loading of points of point constraints (OCCT doesn't use NbBoucle)
    private void LoadPoint(int nbBoucle, int orderMax = 2)
    {
        Point3D p3d, pdif = new Point3D(), pp;
        Point2D p2d;
        int ntPntCont = _pntCont.Length();
        int tang, i;

        // Loading of points of point constraints
        for (i = 1; i <= ntPntCont; i++)
        {
            _pntCont.Value(i).D0(out p3d);
            p2d = _pntCont.Value(i).Pnt2dOnSurf();
            _surfInit!.D0(p2d.Coord(1), p2d.Coord(2), out pp);
            pdif = pdif.WithCoord(-pp.Coord(1) + p3d.Coord(1),
                          -pp.Coord(2) + p3d.Coord(2),
                          -pp.Coord(3) + p3d.Coord(3));
            PinpointConstraint pc = new PinpointConstraint(p2d.XY(), pdif.XYZ(), 0, 0);
            _plate.Load(pc);
            tang = Min(_pntCont.Value(i).Order(), orderMax);
            if (tang == 1)
            { // ==1
                Vector3D v1, v2, v3, v4;
                _pntCont.Value(i).D1(out pp, out v1, out v2);
                _surfInit.D1(p2d.Coord(1), p2d.Coord(2), out pp, out v3, out v4);
                D1 d1final = new D1(v1.XYZ(), v2.XYZ());
                D1 d1init = new D1(v3.XYZ(), v4.XYZ());
                if (!_free)
                {
                    GtoCConstraint gcc = new GtoCConstraint(p2d.XY(), d1init, d1final);
                    _plate.Load(gcc);
                }
                else
                {
                    FreeGtoCConstraint freeGCC = new FreeGtoCConstraint(p2d.XY(), d1init, d1final);
                    _plate.Load(freeGCC);
                }
            }
            // Loading of points G2 GeomPlate_PlateG0Criterion
            if (tang == 2) // ==2
            {
                Vector3D v1, v2, v3, v4, v5, v6, v7, v8, v9, v10;
                _pntCont.Value(i).D2(out pp, out v1, out v2, out v5, out v6, out v7);
                _surfInit.D2(p2d.Coord(1), p2d.Coord(2), out pp, out v3, out v4, out v8, out v9, out v10);
                D1 d1final = new D1(v1.XYZ(), v2.XYZ());
                D1 d1init = new D1(v3.XYZ(), v4.XYZ());
                D2 d2final = new D2(v5.XYZ(), v6.XYZ(), v7.XYZ());
                D2 d2init = new D2(v8.XYZ(), v9.XYZ(), v10.XYZ());
                GtoCConstraint gcc = new GtoCConstraint(p2d.XY(), d1init, d1final, d2init, d2final);
                _plate.Load(gcc);
            }
        }
    }

    private bool VerifSurface(int nbBoucle)
    {
        //======================================================================
        //    Calculate errors
        //======================================================================
        int ntLinCont = _linCont.Length();
        bool result = true;

        // variable for error calculation
        _g0Error = 0;
        _g1Error = 0;
        _g2Error = 0;

        for (int i = 1; i <= ntLinCont; i++)
        {
            CurveConstraint linCont;
            linCont = _linCont.Value(i);
            if (linCont.Order() != -1)
            {
                int NbPts_i = _parCont!.Value(i).Length();
                if (NbPts_i < 3)
                {
                    NbPts_i = 4;
                }
                NCollection_HArray1<double> tdist = new NCollection_HArray1<double>(1, NbPts_i - 1);
                NCollection_HArray1<double> tang = new NCollection_HArray1<double>(1, NbPts_i - 1);
                NCollection_HArray1<double> tcourb = new NCollection_HArray1<double>(1, NbPts_i - 1);

                EcartContraintesMil(i, tdist, tang, tcourb);

                double diffDistMax = 0, diffAngMax = 0;
                int ndiffDist = 0, ndiffAng = 0;

                for (int j = 1; j < NbPts_i; j++)
                {
                    if (tdist.Value(j) > _g0Error)
                    {
                        _g0Error = tdist.Value(j);
                    }
                    if (tang.Value(j) > _g1Error)
                    {
                        _g1Error = tang.Value(j);
                    }
                    if (tcourb.Value(j) > _g2Error)
                    {
                        _g2Error = tcourb.Value(j);
                    }
                    double u;
                    if (_parCont.Value(i).Length() > 3)
                    {
                        u = (_parCont.Value(i).Value(j) + _parCont.Value(i).Value(j + 1)) / 2;
                    }
                    else
                    {
                        u = linCont.FirstParameter()
                            + (linCont.LastParameter() - linCont.FirstParameter()) * (j - 1) / (NbPts_i - 2);
                    }
                    double diffDist = tdist.Value(j) - linCont.G0Criterion(u), diffAng;
                    if (linCont.Order() > 0)
                    {
                        diffAng = tang.Value(j) - linCont.G1Criterion(u);
                    }
                    else
                    {
                        diffAng = 0;
                    }
                    // find the maximum variation of error and calculate the average
                    if (diffDist > 0)
                    {
                        diffDist = diffDist / linCont.G0Criterion(u);
                        if (diffDist > diffDistMax)
                        {
                            diffDistMax = diffDist;
                        }
                        ndiffDist++;
                    }
                    else if ((diffAng > 0) && (linCont.Order() == 1))
                    {
                        diffAng = diffAng / _linCont.Value(i).G1Criterion(u);
                        if (diffAng > diffAngMax)
                        {
                            diffAngMax = diffAng;
                        }
                        ndiffAng++;
                    }
                }

                if (ndiffDist > 0)
                { // at least one point is not acceptable in G0
                    double coef;
                    if (linCont.Order() == 0)
                    {
                        coef = 0.6 * Math.Log(diffDistMax + 7.4);
                        // 7.4 corresponds to the calculation of min. coefficient = 1.2 is e^1.2/0.6
                    }
                    else
                    {
                        coef = Math.Log(diffDistMax + 3.3);
                    }
                    // 3.3 corresponds to calculation of min. coefficient = 1.2 donc e^1.2
                    if (coef > 3)
                    {
                        coef = 3;
                    }
                    // experimentally after the coefficient becomes bad for L cases
                    if ((nbBoucle > 1) && (diffDistMax > 2))
                    {
                        coef = 1.6;
                    }

                    if (linCont.PointCount >= Math.Floor(linCont.PointCount * coef))
                    {
                        coef = 2; // to provide increase of the number of points
                    }

                    linCont.SetNbPoints((int)(linCont.PointCount * coef));
                    result = false;
                }
                else if (ndiffAng > 0) // at least 1 point is not acceptable in G1
                {
                    double coef = 1.5;
                    if ((linCont.PointCount + 1) >= Math.Floor(linCont.PointCount * coef))
                    {
                        coef = 2;
                    }

                    linCont.SetNbPoints((int)(linCont.PointCount * coef));
                    result = false;
                }
            }
        }
        if (!result)
        {
            if (_free && nbBoucle == 1)
            {
                _prevPlate.Copy(_plate);
            }
            _plate.Init();
        }
        return result;
    }

    // The distance, angle and curvature gap at the last point constraint (OCCT's callers discard them)
    private void VerifPoints(out double dist, out double ang, out double curv)
    {
        int ntPntCont = _pntCont.Length();
        Point3D pi, pf;
        Point2D p2d;
        Vector3D v1i, v1f, v2i, v2f, v3i, v3f;
        ang = 0;
        dist = 0;
        curv = 0;
        PointConstraint pntCont;
        for (int i = 1; i <= ntPntCont; i++)
        {
            pntCont = _pntCont.Value(i);
            switch (pntCont.Order())
            {
                case 0:
                    p2d = pntCont.Pnt2dOnSurf();
                    pntCont.D0(out pi);
                    _geomPlateSurface!.D0(p2d.Coord(1), p2d.Coord(2), out pf);
                    dist = pf.DistanceTo(pi);
                    break;
                case 1:
                    pntCont.D1(out pi, out v1i, out v2i);
                    p2d = pntCont.Pnt2dOnSurf();
                    _geomPlateSurface!.D1(p2d.Coord(1), p2d.Coord(2), out pf, out v1f, out v2f);
                    dist = pf.DistanceTo(pi);
                    v3i = v1i ^ v2i;
                    v3f = v1f ^ v2f;
                    ang = v3f.AngleTo(v3i);
                    if (ang > (M_PI / 2))
                    {
                        ang = M_PI - ang;
                    }
                    break;
                case 2:
                    Surface splate = _geomPlateSurface!;
                    SurfaceContinuity cg2 = new SurfaceContinuity();
                    p2d = pntCont.Pnt2dOnSurf();
                    SLProps prop = new SLProps(splate, p2d.Coord(1), p2d.Coord(2), 2, 0.001);
                    cg2.ComputeAnalysis(prop, pntCont.LPropSurf(), G2);
                    dist = cg2.C0Value();
                    ang = cg2.G1Angle();
                    curv = cg2.G2CurvatureGap();
                    break;
            }
        }
    }

    private double ComputeAnisotropie()
    {
        if (_anisotropie)
        {
            // Temporary
            return 1.0;
        }
        else
        {
            return 1.0;
        }
    }

    private bool IsOrderG1()
    {
        bool result = true;
        for (int i = 1; i <= _linCont.Length(); i++)
        {
            if (_linCont.Value(i).Order() < 1)
            {
                result = false;
                break;
            }
        }
        return result;
    }
}
