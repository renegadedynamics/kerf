// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtCS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtCS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Thu Jul  7 12:29:34 2005 OCC9134

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extremum distances between a 3D curve and a surface on parameter ranges. A line against a plane, cylinder or
/// sphere, a circle against a plane, cylinder or sphere and a hyperbola against a plane are solved analytically
/// (<see cref="ExtElCS"/>); other pairs by sampling and local search (Extrema_GenExtCS), with the curve's
/// sharp points projected on the surface. A curve parallel to the surface reports <see cref="IsParallel"/> with one
/// distance.
/// </summary>
/// <remarks>
/// An algorithm object (decision 117): construct it with the curve and the surface, or <c>Initialize</c> with the
/// surface and <c>Perform</c> with the curve. Keeps a reference to the surface, as OCCT keeps the pointer: the
/// caller's adaptor must outlive the algorithm. OCCT disallows copies.
/// </remarks>
internal class ExtCS
{
    private SurfaceAdaptorBase? _s;
    private bool _done;
    private bool _isPar;
    private readonly ExtElCS _extElCS = new();
    private readonly NCollection_Sequence<POnSurf> _pOnS = new();
    private readonly NCollection_Sequence<POnCurv> _pOnC = new();
    private double _myuinf;
    private double _myusup;
    private double _myvinf;
    private double _myvsup;
    private double _mytolC;
    private double _mytolS;
    private double _myucinf;
    private double _myucsup;
    private readonly NCollection_Sequence<double> _sqDist = new();
    private SurfaceKind _stype;

    /// <summary>An empty algorithm.</summary>
    public ExtCS()
    {
        _s = null;
        _done = false;
        _isPar = false;
        _myuinf = 0.0;
        _myusup = 0.0;
        _myvinf = 0.0;
        _myvsup = 0.0;
        _mytolC = 0.0;
        _mytolS = 0.0;
        _myucinf = 0.0;
        _myucsup = 0.0;
        _stype = OtherSurface;
    }

    /// <summary>It calculates all the distances between C and S.</summary>
    public ExtCS(CurveAdaptorBase c, SurfaceAdaptorBase s, double tolC, double tolS)
    {
        Initialize(s, tolC, tolS);
        Perform(c, c.FirstParameter(), c.LastParameter());
    }

    /// <summary>
    /// It calculates all the distances between C and S. UCinf and UCmax are the start and end parameters of the
    /// curve.
    /// </summary>
    public ExtCS(CurveAdaptorBase c,
                         SurfaceAdaptorBase s,
                         double uCinf,
                         double uCsup,
                         double uinf,
                         double usup,
                         double vinf,
                         double vsup,
                         double tolC,
                         double tolS)
    {
        Initialize(s, uinf, usup, vinf, vsup, tolC, tolS);
        Perform(c, uCinf, uCsup);
    }

    /// <summary>Initializes the fields of the algorithm.</summary>
    public void Initialize(SurfaceAdaptorBase s, double tolC, double tolS)
    {
        Initialize(s, s.FirstUParameter(), s.LastUParameter(), s.FirstVParameter(), s.LastVParameter(), tolC, tolS);
    }

    /// <summary>Initializes the fields of the algorithm.</summary>
    public void Initialize(SurfaceAdaptorBase s, double uinf, double usup, double vinf, double vsup, double tolC, double tolS)
    {
        _s = s;
        _isPar = false;
        _myuinf = uinf;
        _myusup = usup;
        _myvinf = vinf;
        _myvsup = vsup;
        _mytolC = tolC;
        _mytolS = tolS;
        _stype = _s.GetType();
    }

    /// <summary>Computes the distances. An exception is raised if the fields have not been initialized.</summary>
    public void Perform(CurveAdaptorBase c, double uinf, double usup)
    {
        // OCCT dereferences the null surface pointer; the port throws.
        SurfaceAdaptorBase s = _s ?? throw new NotDoneException("Extrema_ExtCS::Perform: not initialized");

        _myucinf = uinf;
        _myucsup = usup;
        _pOnS.Clear();
        _pOnC.Clear();
        _sqDist.Clear();
        int i, j;
        int nbT, nbU, nbV;
        nbT = 12;
        nbU = nbV = 10;
        CurveKind myCtype = c.GetType();

        _done = false;
        // Try analytic computation of extrema
        bool isComputeAnalytic = true;

        // OCCT's switch with [[fallthrough]]: the cases that reach the generic line/surface block, and the circle
        // cases falling through to the hyperbola test and to `default`, are written out.
        switch (myCtype)
        {
            case Line:
            {
                if (_stype == SurfaceKind.Sphere)
                {
                    _extElCS.Perform(c.Line(), s.Sphere());
                    break;
                }
                if (_stype == SurfaceKind.Cylinder)
                {
                    _extElCS.Perform(c.Line(), s.Cylinder());
                    break;
                }
                if (_stype == Plane)
                {
                    _extElCS.Perform(c.Line(), s.Plane());
                    if (_extElCS.IsParallel)
                    {
                        break;
                    }
                    // [[fallthrough]]
                }

                // GeomAbs_Plane (not parallel), Torus, Cone, BezierSurface, BSplineSurface, SurfaceOfRevolution,
                // SurfaceOfExtrusion, OffsetSurface, OtherSurface
                {
                    double cfirst = _myucinf, clast = _myucsup;
                    double ufirst = s.FirstUParameter(), ulast = s.LastUParameter(),
                           vfirst = s.FirstVParameter(), vlast = s.LastVParameter();

                    if (!(global::Kerf.Engine.Foundation.Precision.IsInfinite(ufirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(ulast)
                          || global::Kerf.Engine.Foundation.Precision.IsInfinite(vfirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(vlast)))
                    {
                        double tmin = global::Kerf.Engine.Foundation.Precision.Infinite(), tmax = -tmin;
                        Box surfBox = new();
                        AddSurface.Add(s, ufirst, ulast, vfirst, vlast, global::Kerf.Engine.Foundation.Precision.Confusion(), ref surfBox);
                        surfBox.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
                        Line3D lin = c.Line();
                        double parOnLin;
                        Span<Point3D> limPntArray =
                        [
                            new Point3D(xmin, ymin, zmin),
                            new Point3D(xmax, ymin, zmin),
                            new Point3D(xmin, ymax, zmin),
                            new Point3D(xmax, ymax, zmin),
                            new Point3D(xmin, ymin, zmax),
                            new Point3D(xmax, ymin, zmax),
                            new Point3D(xmin, ymax, zmax),
                            new Point3D(xmax, ymax, zmax),
                        ];

                        for (i = 0; i <= 7; i++)
                        {
                            parOnLin = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, limPntArray[i]);
                            tmin = Min(parOnLin, tmin);
                            tmax = Max(parOnLin, tmax);
                        }
                        cfirst = Max(cfirst, tmin);
                        clast = Min(clast, tmax);
                    }

                    if (s.IsUPeriodic)
                    {
                        nbU = 13;
                    }
                    if (s.IsVPeriodic)
                    {
                        nbV = 13;
                    }

                    if (clast - cfirst <= global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        double cPar = (cfirst + clast) / 2.0;
                        Point3D pm = c.Value(cPar);
                        ExtPS extPS = new(pm, s, ufirst, ulast, vfirst, vlast, _mytolS, _mytolS,
                                                    ExtFlag.ExtFlag_MIN);
                        _done = extPS.IsDone;
                        if (_done)
                        {
                            int nbExt = extPS.ExtCount;
                            double T = cPar;
                            POnSurf ps;
                            for (i = 1; i <= nbExt; i++)
                            {
                                ps = extPS.Point(i);
                                ps.Parameter(out double u, out double v);
                                // the middle point, where OCCT passed the origin (decision 1430)
                                AddSolution(c, T, u, v, pm, ps.Value(), extPS.SquareDistance(i));
                            }
                        }
                        return;
                    }

                    GenExtCS ext = new(c, s, nbT, nbU, nbV, cfirst, clast, ufirst, ulast, vfirst, vlast,
                                               _mytolC, _mytolS);

                    _done = ext.IsDone;
                    if (_done)
                    {
                        int nbExt = ext.ExtCount;
                        double T;
                        POnCurv pc;
                        POnSurf ps;
                        for (i = 1; i <= nbExt; i++)
                        {
                            pc = ext.PointOnCurve(i);
                            ps = ext.PointOnSurface(i);
                            T = pc.Parameter();
                            ps.Parameter(out double u, out double v);
                            AddSolution(c, T, u, v, pc.Value(), ps.Value(), ext.SquareDistance(i));
                        }
                    }
                    return;
                }
            }
            //  Modified by skv - Thu Jul  7 12:29:34 2005 OCC9134 Begin
            case Circle:
            {
                if (_stype == SurfaceKind.Cylinder)
                {
                    _extElCS.Perform(c.Circle(), s.Cylinder());
                    break;
                }
                else if (_stype == Plane)
                {
                    _extElCS.Perform(c.Circle(), s.Plane());
                    break;
                }
                else if (_stype == SurfaceKind.Sphere)
                {
                    _extElCS.Perform(c.Circle(), s.Sphere());
                    break;
                }
                // [[fallthrough]] to the hyperbola case, whose test fails for a circle, then to default
                isComputeAnalytic = false;
                break;
            }
            case Hyperbola:
            {
                if (myCtype == Hyperbola && _stype == Plane)
                {
                    //  Modified by skv - Thu Jul  7 12:29:34 2005 OCC9134 End
                    _extElCS.Perform(c.Hyperbola(), s.Plane());
                    break;
                }
                // [[fallthrough]]
                isComputeAnalytic = false;
                break;
            }
            default:
            {
                isComputeAnalytic = false;
                break;
            }
        }

        if (isComputeAnalytic)
        {
            if (_extElCS.IsDone)
            {
                _done = true;
                _isPar = _extElCS.IsParallel;
                if (_isPar)
                {
                    _sqDist.Append(_extElCS.SquareDistance(1));
                }
                else
                {
                    int nbExt = _extElCS.ExtCount;
                    for (i = 1; i <= nbExt; i++)
                    {
                        _extElCS.Points(i, out POnCurv pc, out POnSurf ps);
                        double ucurve = pc.Parameter();
                        ps.Parameter(out double u, out double v);
                        AddSolution(c, ucurve, u, v, pc.Value(), ps.Value(), _extElCS.SquareDistance(i));
                    }

                    if (_sqDist.Length() == 0 && nbExt > 0)
                    {
                        // Analytical extrema seem to be out of curve/surface boundaries.
                        // Try extremity points of curve.
                        Span<Point3D> pOnC = [new Point3D(), new Point3D()];
                        Span<Point3D> pOnS = [new Point3D(), new Point3D()];
                        Span<double> aT = [_myucinf, _myucsup];
                        Span<double> u = [0.0, 0.0];
                        Span<double> v = [0.0, 0.0];
                        Span<double> dist = [-1, -1];
                        for (i = 0; i < 2; ++i)
                        {
                            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aT[i]))
                            {
                                continue;
                            }

                            pOnC[i] = c.Value(aT[i]);
                            switch (_stype)
                            {
                                case Plane:
                                {
                                    global::Kerf.Engine.Geometry.ElSLib.Parameters(s.Plane(), pOnC[i], out u[i], out v[i]);
                                    pOnS[i] = global::Kerf.Engine.Geometry.ElSLib.Value(u[i], v[i], s.Plane());
                                    break;
                                }
                                case SurfaceKind.Sphere:
                                {
                                    global::Kerf.Engine.Geometry.ElSLib.Parameters(s.Sphere(), pOnC[i], out u[i], out v[i]);
                                    pOnS[i] = global::Kerf.Engine.Geometry.ElSLib.Value(u[i], v[i], s.Sphere());
                                    break;
                                }
                                case SurfaceKind.Cylinder:
                                {
                                    global::Kerf.Engine.Geometry.ElSLib.Parameters(s.Cylinder(), pOnC[i], out u[i], out v[i]);
                                    pOnS[i] = global::Kerf.Engine.Geometry.ElSLib.Value(u[i], v[i], s.Cylinder());
                                    break;
                                }
                                case SurfaceKind.Torus:
                                {
                                    global::Kerf.Engine.Geometry.ElSLib.Parameters(s.Torus(), pOnC[i], out u[i], out v[i]);
                                    pOnS[i] = global::Kerf.Engine.Geometry.ElSLib.Value(u[i], v[i], s.Torus());
                                    break;
                                }
                                case SurfaceKind.Cone:
                                {
                                    global::Kerf.Engine.Geometry.ElSLib.Parameters(s.Cone(), pOnC[i], out u[i], out v[i]);
                                    pOnS[i] = global::Kerf.Engine.Geometry.ElSLib.Value(u[i], v[i], s.Cone());
                                    break;
                                }
                                default:
                                    continue;
                            }

                            dist[i] = pOnC[i].SquareDistanceTo(pOnS[i]);
                        }

                        Span<bool> bAdd = [false, false];

                        // Choose solution to add
                        if (dist[0] >= 0.0 && dist[1] >= 0.0)
                        {
                            double diff = dist[0] - dist[1];
                            // Both computed -> take only minimal
                            if (Math.Abs(diff) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                // Add both
                                bAdd[0] = bAdd[1] = true;
                            }
                            else if (diff < 0)
                            {
                                // Add first
                                bAdd[0] = true;
                            }
                            else
                            {
                                // Add second
                                bAdd[1] = true;
                            }
                        }
                        else if (dist[0] >= 0.0)
                        {
                            // Add first
                            bAdd[0] = true;
                        }
                        else if (dist[1] >= 0.0)
                        {
                            // Add second
                            bAdd[1] = true;
                        }

                        for (i = 0; i < 2; ++i)
                        {
                            if (bAdd[i])
                            {
                                AddSolution(c, aT[i], u[i], v[i], pOnC[i], pOnS[i], dist[i]);
                            }
                        }
                    }
                }
                return;
            }
        }

        // Elementary extrema is not done, try generic solution
        GenExtCS ext2 = new();
        ext2.Initialize(s, nbU, nbV, _mytolS);
        if (myCtype == Hyperbola)
        {
            double tmin = Max(-20.0, c.FirstParameter());
            double tmax = Min(20.0, c.LastParameter());
            ext2.Perform(c, nbT, tmin, tmax, _mytolC); // to avoid overflow
        }
        else
        {
            if ((myCtype == Circle && nbT < 13) || (myCtype == BSplineCurve && nbT < 13))
            {
                nbT = 13;
            }
            ext2.Perform(c, nbT, _mytolC);
        }

        _done = ext2.IsDone;
        if (_done)
        {
            int nbExt = ext2.ExtCount;
            double t, u, v;
            POnCurv pc;
            POnSurf ps;
            for (i = 1; i <= nbExt; i++)
            {
                pc = ext2.PointOnCurve(i);
                ps = ext2.PointOnSurface(i);
                t = pc.Parameter();
                ps.Parameter(out u, out v);
                AddSolution(c, t, u, v, pc.Value(), ps.Value(), ext2.SquareDistance(i));
            }

            // Add sharp points
            int solNumber = _sqDist.Length();
            // OCCT casts the const curve to non-const (void* CopyC) to call NbIntervals and Intervals.
            CurveAdaptorBase aC = c;
            int nbIntervals = aC.NbIntervals(C1);
            NCollection_Array1<double> sharpPoints = new(1, nbIntervals + 1);
            aC.Intervals(sharpPoints, C1);

            ExtPS projPS = new();
            projPS.Initialize(s,
                               s.FirstUParameter(),
                               s.LastUParameter(),
                               s.FirstVParameter(),
                               s.LastVParameter(),
                               _mytolS,
                               _mytolS);

            for (i = 2; i < sharpPoints.Upper(); ++i)
            {
                t = sharpPoints[i];
                Point3D pnt = c.Value(t);
                projPS.Perform(pnt);
                if (!projPS.IsDone)
                {
                    continue;
                }
                int nbProj = projPS.ExtCount, jmin = 0;
                double minSqDist = RealLast();
                for (j = 1; j <= nbProj; j++)
                {
                    double sqDist = projPS.SquareDistance(j);
                    if (sqDist < minSqDist)
                    {
                        minSqDist = sqDist;
                        jmin = j;
                    }
                }
                if (jmin != 0)
                {
                    projPS.Point(jmin).Parameter(out u, out v);
                    AddSolution(c, t, u, v, pnt, projPS.Point(jmin).Value(), minSqDist);
                }
            }
            // Cut sharp solutions to keep only minimum and maximum
            int imin = solNumber + 1, imax = _sqDist.Length();
            for (i = solNumber + 1; i <= _sqDist.Length(); i++)
            {
                if (_sqDist[i] < _sqDist[imin])
                {
                    imin = i;
                }
                if (_sqDist[i] > _sqDist[imax])
                {
                    imax = i;
                }
            }
            if (_sqDist.Length() > solNumber + 2)
            {
                double minSqDist = _sqDist[imin];
                double maxSqDist = _sqDist[imax];
                POnCurv minPC = _pOnC[imin];
                POnCurv maxPC = _pOnC[imax];
                POnSurf minPS = _pOnS[imin];
                POnSurf maxPS = _pOnS[imax];

                _sqDist.Remove(solNumber + 1, _sqDist.Length());
                _pOnC.Remove(solNumber + 1, _pOnC.Length());
                _pOnS.Remove(solNumber + 1, _pOnS.Length());

                _sqDist.Append(minSqDist);
                _pOnC.Append(minPC);
                _pOnS.Append(minPS);
                _sqDist.Append(maxSqDist);
                _pOnC.Append(maxPC);
                _pOnS.Append(maxPS);
            }
        }
    }

    /// <summary>Returns True if the distances are found.</summary>
    public bool IsDone => _done;

    /// <summary>Returns True if the curve is on a parallel surface.</summary>
    public bool IsParallel
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }

            return _isPar;
        }
    }

    /// <summary>Returns the value of the Nth resulting square distance.</summary>
    public double SquareDistance(int n)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        return _sqDist.Value(n);
    }

    /// <summary>Returns the number of extremum distances.</summary>
    public int ExtCount
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }

            return _sqDist.Length();
        }
    }

    /// <summary>Returns the points of the Nth resulting distance.</summary>
    /// <remarks>
    /// When the curve is parallel to the surface, OCCT stores one distance and no points, and reading them fails;
    /// the port throws Standard_OutOfRange from the sequence there.
    /// </remarks>
    public void Points(int n, out POnCurv p1, out POnSurf p2)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        p1 = _pOnC.Value(n);
        p2 = _pOnS.Value(n);
    }

    private bool AddSolution(CurveAdaptorBase curve,
                             double aT,
                             double aU,
                             double aV,
                             in Point3D pointOnCurve,
                             in Point3D pointOnSurf,
                             double squareDist)
    {
        bool added = false;

        double t = aT, u = aU, v = aV;

        if (curve.IsPeriodic)
        {
            t = global::Kerf.Engine.Geometry.ElCLib.InPeriod(t, _myucinf, _myucinf + curve.Period());
        }
        if (_s!.IsUPeriodic)
        {
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, _myuinf, _myuinf + _s.UPeriod());
        }
        if (_s.IsVPeriodic)
        {
            v = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v, _myvinf, _myvinf + _s.VPeriod());
        }

        POnCurv pc = new();
        POnSurf ps;
        if ((_myucinf - t) <= _mytolC && (t - _myucsup) <= _mytolC && (_myuinf - u) <= _mytolS && (u - _myusup) <= _mytolS
            && (_myvinf - v) <= _mytolS && (v - _myvsup) <= _mytolS)
        {
            bool isNewSolution = true;
            for (int j = 1; j <= _sqDist.Length(); j++)
            {
                pc = _pOnC[j];
                ps = _pOnS[j];
                double tj = pc.Parameter();
                ps.Parameter(out double uj, out double vj);
                if (Math.Abs(t - tj) <= _mytolC && Math.Abs(u - uj) <= _mytolS && Math.Abs(v - vj) <= _mytolS)
                {
                    isNewSolution = false;
                    break;
                }
            }
            if (isNewSolution)
            {
                _sqDist.Append(squareDist);
                pc.SetValues(t, pointOnCurve);
                _pOnC.Append(pc);
                _pOnS.Append(new POnSurf(u, v, pointOnSurf));
                added = true;
            }
        }
        return added;
    }
}
