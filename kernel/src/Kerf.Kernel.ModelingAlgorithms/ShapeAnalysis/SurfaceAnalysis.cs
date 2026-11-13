// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Surface.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Surface.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>
/// Complements Geom_Surface with the detection of its singularities, the check of its spatial closure and the
/// projection of 3D points onto it.
/// </summary>
/// <remarks>
/// <para>
/// Singularities: each stores the precision with which the corresponding iso-line is considered degenerated. Their
/// number, for a given precision, is never more than 4.
/// </para>
/// <para>
/// Closure: checked with a given precision (Precision::Confusion() by default). A surface Geom_Surface says is closed
/// is closed; otherwise more analysis is done.
/// </para>
/// <para>
/// Projection: done with a given precision, taking the singularities into account; a variant searches from an
/// already computed solution.
/// </para>
/// <para>
/// A working object: it computes most of this once and caches it. The bound iso-curves it computes are frozen
/// before they are cached or handed out.
/// </para>
/// </remarks>
internal class SurfaceAnalysis : Transient
{
    protected Surface?         _surf;
    protected SurfaceAdaptor?  _adSur;
    protected ExtPS         _extPS = new ExtPS();
    protected bool                  _extOK;
    protected int                   _nbDeg;
    protected double[]              _preci    = new double[4];
    protected Point3D[]              _p3d      = new Point3D[4];
    protected Point2D[]            _firstP2d = new Point2D[4];
    protected Point2D[]            _lastP2d  = new Point2D[4];
    protected double[]              _firstPar = new double[4];
    protected double[]              _lastPar  = new double[4];
    protected bool[]                _uIsoDeg  = new bool[4];
    protected bool                  _isos;
    protected double                _uf;
    protected double                _ul;
    protected double                _vf;
    protected double                _vl;
    protected Curve3D?           _isoUF;
    protected Curve3D?           _isoUL;
    protected Curve3D?           _isoVF;
    protected Curve3D?           _isoVL;
    protected bool                  _isoBoxes;
    protected Box               _bndUF = new Box();
    protected Box               _bndUL = new Box();
    protected Box               _bndVF = new Box();
    protected Box               _bndVL = new Box();
    protected double                _gap;
    protected double                _uDelt;
    protected double                _vDelt;
    protected double                _uCloseVal;
    protected double                _vCloseVal;

    private static void RestrictBounds(ref double first, ref double last)
    {
        bool isFInf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(first);
        bool isLInf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(last);
        if (isFInf || isLInf)
        {
            if (isFInf && isLInf)
            {
                first = -1000;
                last  = 1000;
            }
            else if (isFInf)
            {
                first = last - 2000;
            }
            else
            {
                last = first + 2000;
            }
        }
    }

    private static void RestrictBounds(ref double uf, ref double ul, ref double vf, ref double vl)
    {
        RestrictBounds(ref uf, ref ul);
        RestrictBounds(ref vf, ref vl);
    }

    /// <summary>Creates an analyzer of an existing surface.</summary>
    /// <remarks>
    /// A null surface leaves the analyzer empty, as in OCCT (which sends a warning through Message, whose default
    /// messenger sends nothing in the port).
    /// </remarks>
    public SurfaceAnalysis(Surface? s)
    {
        _surf      = s;
        _extOK     = false; //: 30
        _nbDeg     = -1;
        _isos      = false;
        _isoBoxes  = false;
        _gap       = 0.0;
        _uDelt     = 0.01;
        _vDelt     = 0.01;
        _uCloseVal = -1;
        _vCloseVal = -1;
        // Bug 33895: Prevent crash when surface is null
        if (_surf is null)
        {
            // OCCT warns here: "Cannot create with null surface"
            return;
        }

        _surf.ParameterBounds(out _uf, out _ul, out _vf, out _vl);
        _adSur = new SurfaceAdaptor(_surf);
    }

    /// <summary>Loads an existing surface.</summary>
    public void Init(Surface? surface)
    {
        if (ReferenceEquals(_surf, surface))
        {
            return;
        }

        // Bug 33895: Prevent crash when surface is null
        if (surface is null)
        {
            // OCCT warns here: "Cannot initialize with null surface"
            return;
        }

        _extOK     = false; //: 30
        _surf      = surface;
        _nbDeg     = -1;
        _uCloseVal = _vCloseVal = -1;
        _gap       = 0.0;
        _surf.ParameterBounds(out _uf, out _ul, out _vf, out _vl);
        _adSur    = new SurfaceAdaptor(_surf);
        _isos     = false;
        _isoBoxes = false;
        _isoUF    = null;
        _isoUL    = null;
        _isoVF    = null;
        _isoVL    = null;
    }

    /// <summary>Reads all the data from another analyzer, without recomputing (it shares the other's adaptor).</summary>
    public void Init(SurfaceAnalysis other)
    {
        Init(other.Surface());
        _adSur = other.TrueAdaptor3d();
        _nbDeg = other._nbDeg; // rln S4135 direct transmission (to avoid computation in <other>)
        for (int i = 0; i < _nbDeg; i++)
        {
            other.Singularity(i + 1, ref _preci[i], ref _p3d[i], ref _firstP2d[i], ref _lastP2d[i],
                              ref _firstPar[i], ref _lastPar[i], ref _uIsoDeg[i]);
        }
    }

    /// <summary>Sets the parametric domain analysed (by default the surface's bounds).</summary>
    public void SetDomain(double u1, double u2, double v1, double v2)
    {
        _uf = u1;
        _ul = u2;
        _vf = v1;
        _vl = v2;
    }

    /// <summary>Returns the surface being analysed.</summary>
    public Surface? Surface() => _surf;

    /// <summary>Returns the adaptor of the surface.</summary>
    public SurfaceAdaptor? Adaptor3d() => _adSur;

    /// <summary>Returns the adaptor (null when the surface is null).</summary>
    public SurfaceAdaptor? TrueAdaptor3d() => _adSur;

    /// <summary>
    /// Returns the 3D distance found by the last of IsDegenerated, DegeneratedValues, ProjectDegenerated (to the found
    /// or last singularity), IsUClosed, IsVClosed (the smallest precision for which the surface is closed) and
    /// ValueOfUV (to the solution).
    /// </summary>
    public double Gap() => _gap;

    /// <summary>Returns the point of parameters (u, v).</summary>
    public Point3D Value(double u, double v) => _adSur!.Value(u, v);

    /// <summary>Returns the point of parameters p2d.</summary>
    public Point3D Value(in Point2D p2d) => _adSur!.Value(p2d.X, p2d.Y);

    // Kerf: whether the surface has a value at each point ComputeSingularities asks of its bounds (decision 2293)
    private bool HasValuesAtBounds(double su1, double su2, double sv1, double sv2)
    {
        try
        {
            foreach (double u in (ReadOnlySpan<double>)[su1, 0.5 * (su1 + su2), su2])
            {
                foreach (double v in (ReadOnlySpan<double>)[sv1, 0.5 * (sv1 + sv2), sv2])
                {
                    _adSur!.Value(u, v);
                }
            }

            return true;
        }
        catch (global::Kerf.Engine.Geometry.Objects.UndefinedValue3DException)
        {
            return false;
        }
    }

    private void ComputeSingularities()
    {
        // rln S4135
        if (_nbDeg >= 0)
        {
            return;
        }

        //: 51 abv 22 Dec 97: allow forcing:  if (myNbDeg >= 0) return;
        // CKY 27-FEV-98 : en appel direct on peut forcer. En appel interne on optimise
        if (_surf is null)
        {
            return;
        }

        Bounds(out double su1, out double su2, out double sv1, out double sv2); // modified by rln on 12/11/97 mySurf-> is deleted

        _nbDeg = 0; //: r3

        if (_surf is ConicalSurface conicS)
        {
            double vApex = -conicS.RefRadius / Math.Sin(conicS.SemiAngle);
            _preci[0] = 0;
            _p3d[0]   = conicS.Apex;
            _firstP2d[0] = _firstP2d[0].WithCoord(su1, vApex);
            _lastP2d[0] = _lastP2d[0].WithCoord(su2, vApex);
            _firstPar[0] = su1;
            _lastPar[0]  = su2;
            _uIsoDeg[0]  = false;
            _nbDeg       = 1;
        }
        else if (_surf is ToroidalSurface toroidS)
        {
            double minorR = toroidS.MinorRadius;
            double majorR = toroidS.MajorRadius;
            // szv#4:S4163:12Mar99 warning - possible div by zero
            double ang = Math.Acos(Min(1.0, majorR / minorR));
            _preci[0] = _preci[1] = Max(0.0, majorR - minorR);
            _p3d[0]   = _surf.Value(0.0, M_PI - ang);
            _firstP2d[0] = _firstP2d[0].WithCoord(su1, M_PI - ang);
            _lastP2d[0] = _lastP2d[0].WithCoord(su2, M_PI - ang);
            _p3d[1] = _surf.Value(0.0, M_PI + ang);
            _firstP2d[1] = _firstP2d[1].WithCoord(su2, M_PI + ang);
            _lastP2d[1] = _lastP2d[1].WithCoord(su1, M_PI + ang);
            _firstPar[0] = _firstPar[1] = su1;
            _lastPar[0]  = _lastPar[1]  = su2;
            _uIsoDeg[0]  = _uIsoDeg[1]  = false;
            _nbDeg       = (majorR > minorR ? 1 : 2);
        }
        else if (_surf is SphericalSurface)
        {
            _preci[0] = _preci[1] = 0;
            _p3d[0]   = _surf.Value(su1, sv2); // Northern pole is first
            _p3d[1]   = _surf.Value(su1, sv1);
            _firstP2d[0] = _firstP2d[0].WithCoord(su2, sv2);
            _lastP2d[0] = _lastP2d[0].WithCoord(su1, sv2);
            _firstP2d[1] = _firstP2d[1].WithCoord(su1, sv1);
            _lastP2d[1] = _lastP2d[1].WithCoord(su2, sv1);
            _firstPar[0] = _firstPar[1] = su1;
            _lastPar[0]  = _lastPar[1]  = su2;
            _uIsoDeg[0]  = _uIsoDeg[1]  = false;
            _nbDeg       = 2;
        }
        else if (_surf is BoundedSurface || _surf is RevolutionSurface //: b2 abv 18 Feb 98
                 || _surf is global::Kerf.Geometry.OffsetSurface)
        { // rln S4135

            // Kerf: a surface with no bound on a side has no boundary there to shrink to a point, but OCCT evaluates it
            // at the bound all the same, 2e100. Where the surface can't be evaluated there, as an offset of a line's
            // surface of revolution, which has no normal, OCCT throws from here and fixshape ends with it (#1365,
            // decision 2293): such a surface has no singular bound. One that can be evaluated there is read as OCCT
            // reads it, whatever that's worth, as the recordings from native have it.
            if ((global::Kerf.Engine.Foundation.Precision.IsInfinite(su1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(su2)
                 || global::Kerf.Engine.Foundation.Precision.IsInfinite(sv1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(sv2))
                && !HasValuesAtBounds(su1, su2, sv1, sv2))
            {
                return;
            }

            // rln S4135 //:r3
            _p3d[0] = _adSur!.Value(su1, 0.5 * (sv1 + sv2));
            _firstP2d[0] = _firstP2d[0].WithCoord(su1, sv2);
            _lastP2d[0] = _lastP2d[0].WithCoord(su1, sv1);

            _p3d[1] = _adSur.Value(su2, 0.5 * (sv1 + sv2));
            _firstP2d[1] = _firstP2d[1].WithCoord(su2, sv1);
            _lastP2d[1] = _lastP2d[1].WithCoord(su2, sv2);

            _p3d[2] = _adSur.Value(0.5 * (su1 + su2), sv1);
            _firstP2d[2] = _firstP2d[2].WithCoord(su1, sv1);
            _lastP2d[2] = _lastP2d[2].WithCoord(su2, sv1);

            _p3d[3] = _adSur.Value(0.5 * (su1 + su2), sv2);
            _firstP2d[3] = _firstP2d[3].WithCoord(su2, sv2);
            _lastP2d[3] = _lastP2d[3].WithCoord(su1, sv2);

            _firstPar[0] = _firstPar[1] = sv1;
            _lastPar[0]  = _lastPar[1]  = sv2;
            _uIsoDeg[0]  = _uIsoDeg[1]  = true;

            _firstPar[2] = _firstPar[3] = su1;
            _lastPar[2]  = _lastPar[3]  = su2;
            _uIsoDeg[2]  = _uIsoDeg[3]  = false;

            Point3D corner1 = _adSur.Value(su1, sv1);
            Point3D corner2 = _adSur.Value(su1, sv2);
            Point3D corner3 = _adSur.Value(su2, sv1);
            Point3D corner4 = _adSur.Value(su2, sv2);

            _preci[0] = Max(corner1.DistanceTo(corner2), Max(_p3d[0].DistanceTo(corner1), _p3d[0].DistanceTo(corner2)));
            _preci[1] = Max(corner3.DistanceTo(corner4), Max(_p3d[1].DistanceTo(corner3), _p3d[1].DistanceTo(corner4)));
            _preci[2] = Max(corner1.DistanceTo(corner3), Max(_p3d[2].DistanceTo(corner1), _p3d[2].DistanceTo(corner3)));
            _preci[3] = Max(corner2.DistanceTo(corner4), Max(_p3d[3].DistanceTo(corner2), _p3d[3].DistanceTo(corner4)));

            _nbDeg = 4;
        }

        SortSingularities();
    }

    /// <summary>
    /// Returns true if the surface has singularities for the given precision (singularities whose sizes are not
    /// greater than it).
    /// </summary>
    public bool HasSingularities(double preci)
    {
        return NbSingularities(preci) > 0;
    }

    /// <summary>
    /// Returns the number of singularities for the given precision (singularities whose sizes are not greater than
    /// it).
    /// </summary>
    public int NbSingularities(double preci)
    {
        if (_nbDeg < 0)
        {
            ComputeSingularities();
        }

        int nb = 0;
        for (int i = 1; i <= _nbDeg; i++)
        {
            if (_preci[i - 1] <= preci)
            {
                nb++;
            }
        }

        return nb;
    }

    /// <summary>
    /// Returns the singularity of rank <paramref name="num"/> (at most the number of possible singularities, not
    /// only those of a precision): the smallest precision with which its iso-line is degenerated, its 3D point (the
    /// middle of the iso-line), the first and last 2D points and parameters of the iso-line, and whether it is a U
    /// iso-line. Returns false, leaving the outputs unchanged, if <paramref name="num"/> is out of range.
    /// </summary>
    public bool Singularity(int num, ref double preci, ref Point3D p3d, ref Point2D firstP2d, ref Point2D lastP2d,
                            ref double firstpar, ref double lastpar, ref bool uisodeg)
    {
        // The fields are C arrays numbered from 0; num starts at 1
        if (_nbDeg < 0)
        {
            ComputeSingularities();
        }

        if (num < 1 || num > _nbDeg)
        {
            return false;
        }

        p3d      = _p3d[num - 1];
        preci    = _preci[num - 1];
        firstP2d = _firstP2d[num - 1];
        lastP2d  = _lastP2d[num - 1];
        firstpar = _firstPar[num - 1];
        lastpar  = _lastPar[num - 1];
        uisodeg  = _uIsoDeg[num - 1];
        return true;
    }

    /// <summary>
    /// Returns true if a surface boundary is degenerated with <paramref name="preci"/> and the point is within
    /// <paramref name="preci"/> of its singular point.
    /// </summary>
    public bool IsDegenerated(in Point3D p3d, double preci)
    {
        if (_nbDeg < 0)
        {
            ComputeSingularities();
        }

        for (int i = 0; i < _nbDeg && _preci[i] <= preci; i++)
        {
            _gap = _p3d[i].DistanceTo(p3d);
            // rln S4135
            if (_gap <= preci)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// As <see cref="IsDegenerated(in Point3D, double)"/>, and returns the characteristics of the singularity nearest
    /// to the point among those matching (the outputs are unchanged when there is none).
    /// </summary>
    public bool DegeneratedValues(in Point3D p3d, double preci, ref Point2D firstP2d, ref Point2D lastP2d,
                                  ref double firstPar, ref double lastPar, bool forward = true)
    {
        _ = forward;
        if (_nbDeg < 0)
        {
            ComputeSingularities();
        }

        // #77 rln S4135: returning singularity which has minimum gap between singular point and input 3D point
        int    indMin = -1;
        double gapMin = RealLast();
        for (int i = 0; i < _nbDeg && _preci[i] <= preci; i++)
        {
            _gap = _p3d[i].DistanceTo(p3d);
            // rln S4135
            if (_gap <= preci)
            {
                if (gapMin > _gap)
                {
                    gapMin = _gap;
                    indMin = i;
                }
            }
        }

        if (indMin >= 0)
        {
            _gap    = gapMin;
            firstP2d = _firstP2d[indMin];
            lastP2d  = _lastP2d[indMin];
            firstPar = _firstPar[indMin];
            lastPar  = _lastPar[indMin];
            return true;
        }

        return false;
    }

    /// <summary>
    /// Projects a point on a singularity by completing <paramref name="result"/>: finds the iso-line degenerated with
    /// <paramref name="preci"/> whose singular point is within <paramref name="preci"/> of the point (or of the 3D
    /// point of <paramref name="result"/>), and sets the coordinate of <paramref name="result"/> along it from
    /// <paramref name="neighbour"/>.
    /// </summary>
    public bool ProjectDegenerated(in Point3D p3d, double preci, in Point2D neighbour, ref Point2D result)
    {
        if (_nbDeg < 0)
        {
            ComputeSingularities();
        }

        // added by rln on 03/12/97
        //: c1 abv 23 Feb 98: preci (3d) -> Resolution (2d)
        // #77 rln S4135
        int    indMin = -1;
        double gapMin = RealLast();
        for (int i = 0; i < _nbDeg && _preci[i] <= preci; i++)
        {
            double gap2 = _p3d[i].SquareDistanceTo(p3d);
            if (gap2 > preci * preci)
            {
                gap2 = Min(gap2, _p3d[i].SquareDistanceTo(Value(result)));
            }

            // rln S4135
            if (gap2 <= preci * preci && gapMin > gap2)
            {
                gapMin = gap2;
                indMin = i;
            }
        }

        if (indMin < 0)
        {
            return false;
        }

        _gap = Math.Sqrt(gapMin);
        if (!_uIsoDeg[indMin])
        {
            result = result.WithX(neighbour.X);
        }
        else
        {
            result = result.WithY(neighbour.Y);
        }

        return true;
    }

    // pdn %12 11.02.99 PRO9234 entity 15402
    /// <summary>
    /// Checks whether the points at the start (<paramref name="direct"/>) or end of <paramref name="points"/> lie in
    /// a singularity, and if so sets their undetermined 2D coordinate from the nearest point that doesn't (evenly
    /// distributed if all do). Returns true if some points were adjusted.
    /// </summary>
    public bool ProjectDegenerated(int nbrPnt, NCollection_Sequence<Point3D> points, NCollection_Sequence<Point2D> pnt2d,
                                   double preci, bool direct)
    {
        if (_nbDeg < 0)
        {
            ComputeSingularities();
        }

        int step = (direct ? 1 : -1);
        // #77 rln S4135
        int    indMin = -1;
        double gapMin = RealLast(), prec2 = preci * preci;
        int    j      = (direct ? 1 : nbrPnt);
        for (int i = 0; i < _nbDeg && _preci[i] <= preci; i++)
        {
            double gap2 = _p3d[i].SquareDistanceTo(points[j]);
            if (gap2 > prec2)
            {
                gap2 = Min(gap2, _p3d[i].SquareDistanceTo(Value(pnt2d[j])));
            }

            if (gap2 <= prec2 && gapMin > gap2)
            {
                gapMin = gap2;
                indMin = i;
            }
        }

        if (indMin < 0)
        {
            return false;
        }

        _gap = Math.Sqrt(gapMin);
        Point2D pk = new Point2D();

        int k;
        for (k = j + step; k <= nbrPnt && k >= 1; k += step)
        {
            pk = pnt2d[k];
            Point3D p1 = points[k];
            if (_p3d[indMin].SquareDistanceTo(p1) > prec2 && _p3d[indMin].SquareDistanceTo(Value(pk)) > prec2)
            {
                break;
            }
        }

        //: p8 abv 11 Mar 99: PRO7226 #489490: if whole pcurve is degenerate, distribute evenly
        if (k < 1 || k > nbrPnt)
        {
            double x1 = (_uIsoDeg[indMin] ? pnt2d[1].Y : pnt2d[1].X);
            double x2 = (_uIsoDeg[indMin] ? pnt2d[nbrPnt].Y : pnt2d[nbrPnt].X);
            for (j = 1; j <= nbrPnt; j++)
            {
                // szv#4:S4163:12Mar99 warning - possible div by zero
                double x = (x1 * (nbrPnt - j) + x2 * (j - 1)) / (nbrPnt - 1);
                if (!_uIsoDeg[indMin])
                {
                    pnt2d[j] = pnt2d[j].WithX(x);
                }
                else
                {
                    pnt2d[j] = pnt2d[j].WithY(x);
                }
            }

            return true;
        }

        for (j = k - step; j <= nbrPnt && j >= 1; j -= step)
        {
            if (!_uIsoDeg[indMin])
            {
                pnt2d[j] = pnt2d[j].WithX(pk.X);
            }
            else
            {
                pnt2d[j] = pnt2d[j].WithY(pk.Y);
            }
        }

        return true;
    }

    /// <summary>
    /// Returns true if the straight pcurve from p2d1 to p2d2 is degenerate: its 3D extent (ends and middle) is within
    /// <paramref name="tol"/>, and its 2D extent is more than <paramref name="ratio"/> times the resolution of that
    /// 3D extent. This detects singularities otherwise than ComputeSingularities.
    /// </summary>
    public bool IsDegenerated(in Point2D p2d1, in Point2D p2d2, double tol, double ratio)
    {
        Point3D p1    = Value(p2d1);
        Point3D p2    = Value(p2d2);
        Point3D pm    = Value(new Point2D(0.5 * (p2d1.XY() + p2d2.XY())));
        double max3d = Max(p1.DistanceTo(p2), Max(pm.DistanceTo(p1), pm.DistanceTo(p2)));
        if (max3d > tol)
        {
            return false;
        }

        SurfaceAdaptor sa = Adaptor3d()!;
        double              ru = sa.UResolution(1.0);
        double              rv = sa.VResolution(1.0);

        if (ru < global::Kerf.Engine.Foundation.Precision.PConfusion() || rv < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        double du = Math.Abs(p2d1.X - p2d2.X) / ru;
        double dv = Math.Abs(p2d1.Y - p2d2.Y) / rv;
        max3d *= ratio;
        return du * du + dv * dv > max3d * max3d;
    }

    // The iso-curve, frozen (it is cached and handed out), or null where the surface can't make it
    private static Curve3D? ComputeIso(Surface surf, bool utype, double par)
    {
        Curve3D? iso;
        try
        {
            if (utype)
            {
                iso = surf.UIso(par);
            }
            else
            {
                iso = surf.VIso(par);
            }
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            iso = null;
        }

        iso?.Freeze(); // Kerf: a curve handed out (and cached) is frozen
        return iso;
    }

    /// <summary>Computes the bound iso-curves (protected against exceptions).</summary>
    public void ComputeBoundIsos()
    {
        if (_isos)
        {
            return;
        }

        _isos  = true;
        _isoUF = ComputeIso(_surf!, true, _uf);
        _isoUL = ComputeIso(_surf!, true, _ul);
        _isoVF = ComputeIso(_surf!, false, _vf);
        _isoVL = ComputeIso(_surf!, false, _vl);
    }

    /// <summary>Returns a U iso-curve (frozen; the bound ones are cached), or null if it can't be made.</summary>
    public Curve3D? UIso(double u)
    {
        if (u == _uf)
        {
            ComputeBoundIsos();
            return _isoUF;
        }

        if (u == _ul)
        {
            ComputeBoundIsos();
            return _isoUL;
        }

        return ComputeIso(_surf!, true, u);
    }

    /// <summary>Returns a V iso-curve (frozen; the bound ones are cached), or null if it can't be made.</summary>
    public Curve3D? VIso(double v)
    {
        if (v == _vf)
        {
            ComputeBoundIsos();
            return _isoVF;
        }

        if (v == _vl)
        {
            ComputeBoundIsos();
            return _isoVL;
        }

        return ComputeIso(_surf!, false, v);
    }

    /// <summary>
    /// Tells if the surface is spatially closed in U within <paramref name="preci"/> (Precision::Confusion() if
    /// negative): if Geom_Surface says so, else by the distance between its U bounds (poles of polynomial B-splines
    /// with full end multiplicities and of Bezier surfaces; values at the knots and their middles of the other
    /// B-splines; the ends of the basis curve of an extrusion; 101 points otherwise).
    /// </summary>
    public bool IsUClosed(double preci = -1)
    {
        double prec      = Max(preci, global::Kerf.Engine.Foundation.Precision.Confusion());
        double umidVal = -1.0;
        if (_uCloseVal < 0)
        {
            // To compute: minimal computations
            Bounds(out double uf, out double ul, out double vf, out double vl); // modified by rln on 12/11/97 mySurf-> is deleted
            RestrictBounds(ref uf, ref ul, ref vf, ref vl);
            _uDelt = Math.Abs(ul - uf) / 20; // modified by rln 11/11/97 instead of 10
                                              // because of the example when 10 was not enough
            if (_surf!.IsUClosed)
            {
                _uCloseVal = 0.0;
                _uDelt     = 0.0;
                _gap       = 0.0;
                return true;
            }

            // Adapted computations
            // #67 rln S4135
            SurfaceAdaptor surfAdapt = Adaptor3d()!;
            SurfaceKind surftype  = surfAdapt.GetType();
            if (_surf is TrimmedSurface)
            {
                surftype = OtherSurface;
            }

            switch (surftype)
            {
                case Plane:
                {
                    _uCloseVal = RealLast();
                    break;
                }
                case SurfaceOfExtrusion:
                { //: c8 abv 03 Mar 98: UKI60094 #753: process Geom_SurfaceOfLinearExtrusion
                    var        extr = (ExtrusionSurface)_surf;
                    Curve3D crv  = extr.BasisCurve;
                    double     f    = crv.FirstParameter;
                    double     l    = crv.LastParameter;
                    //: r3 abv (smh) 30 Mar 99: protect against unexpected signals
                    if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(f) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(l))
                    {
                        Point3D p1 = crv.Value(f);
                        Point3D p2 = crv.Value(l);
                        _uCloseVal = p1.SquareDistanceTo(p2);
                        Point3D pm = crv.Value((f + l) / 2.0);
                        umidVal = p1.SquareDistanceTo(pm);
                    }
                    else
                    {
                        _uCloseVal = RealLast();
                    }

                    break;
                }
                case SurfaceKind.BSplineSurface:
                {
                    var    bs      = (global::Kerf.Geometry.BSplineSurface)_surf;
                    int    nbup    = bs.UPoleCount;
                    double distmin = RealLast();
                    if (bs.IsUPeriodic)
                    {
                        _uCloseVal = 0;
                        _uDelt     = 0;
                    }
                    else if (nbup < 3)
                    { // modified by rln on 12/11/97
                        _uCloseVal = RealLast();
                    }
                    else if (bs.IsURational ||
                             // #6 rln 20/02/98 ProSTEP ug_exhaust-A.stp entity #18360 (Uclosed BSpline,
                             // but multiplicity of boundary knots != degree + 1)
                             bs.UMultiplicity(1) != bs.UDegree + 1 || // #6 //:h4: #6 moved
                             bs.UMultiplicity(bs.UKnotCount) != bs.UDegree + 1)
                    { // #6 //:h4
                        int    nbvk = bs.VKnotCount;
                        double v    = bs.VKnot(1);
                        Point3D p1   = surfAdapt.Value(uf, v);
                        Point3D p2   = surfAdapt.Value(ul, v);
                        _uCloseVal = p1.SquareDistanceTo(p2);
                        Point3D pm = surfAdapt.Value((uf + ul) / 2.0, v);
                        umidVal = p1.SquareDistanceTo(pm);
                        distmin   = _uCloseVal;
                        for (int i = 2; i <= nbvk; i++)
                        {
                            v  = 0.5 * (bs.VKnot(i - 1) + bs.VKnot(i));
                            p1 = bs.Value(uf, v);
                            p2 = bs.Value(ul, v);
                            double dist = p1.SquareDistanceTo(p2);
                            if (dist > _uCloseVal)
                            {
                                _uCloseVal = dist;
                                pm          = bs.Value((uf + ul) / 2.0, v);
                                umidVal   = p1.SquareDistanceTo(pm);
                            }
                            else
                            {
                                distmin = Min(distmin, dist);
                            }
                        }

                        distmin = Math.Sqrt(distmin);
                        _uDelt = Min(_uDelt, 0.5 * surfAdapt.UResolution(distmin)); // #4 smh
                    }
                    else
                    {
                        int nbvp = bs.VPoleCount;
                        _uCloseVal = bs.Pole(1, 1).SquareDistanceTo(bs.Pole(nbup, 1));
                        umidVal   = bs.Pole(1, 1).SquareDistanceTo(bs.Pole(nbup / 2 + 1, 1));
                        distmin     = _uCloseVal;
                        for (int i = 2; i <= nbvp; i++)
                        {
                            double dist = bs.Pole(1, i).SquareDistanceTo(bs.Pole(nbup, i));
                            if (dist > _uCloseVal)
                            {
                                _uCloseVal = dist;
                                umidVal   = bs.Pole(1, i).SquareDistanceTo(bs.Pole(nbup / 2 + 1, i));
                            }
                            else
                            {
                                distmin = Min(distmin, dist);
                            }
                        }

                        distmin = Math.Sqrt(distmin);
                        _uDelt = Min(_uDelt, 0.5 * surfAdapt.UResolution(distmin)); // #4 smh
                    }

                    break;
                }
                case SurfaceKind.BezierSurface:
                {
                    var    bz      = (global::Kerf.Geometry.BezierSurface)_surf;
                    int    nbup    = bz.UPoleCount;
                    double distmin = RealLast();
                    if (nbup < 3)
                    {
                        _uCloseVal = RealLast();
                    }
                    else
                    {
                        int nbvp = bz.VPoleCount;
                        _uCloseVal = bz.Pole(1, 1).SquareDistanceTo(bz.Pole(nbup, 1));
                        umidVal   = bz.Pole(1, 1).SquareDistanceTo(bz.Pole(nbup / 2 + 1, 1));
                        distmin     = _uCloseVal;
                        for (int i = 1; i <= nbvp; i++)
                        {
                            double dist = bz.Pole(1, i).SquareDistanceTo(bz.Pole(nbup, i));
                            if (dist > _uCloseVal)
                            {
                                _uCloseVal = dist;
                                umidVal   = bz.Pole(1, i).SquareDistanceTo(bz.Pole(nbup / 2 + 1, i));
                            }
                            else
                            {
                                distmin = Min(distmin, dist);
                            }
                        }

                        distmin = Math.Sqrt(distmin);
                        _uDelt = Min(_uDelt, 0.5 * surfAdapt.UResolution(distmin)); // #4 smh
                    }

                    break;
                }
                default:
                { // Geom_RectangularTrimmedSurface and Geom_OffsetSurface
                    double distmin  = RealLast();
                    int    nbpoints = 101; // can be revised
                    Point3D p1       = surfAdapt.Value(uf, vf);
                    Point3D p2       = surfAdapt.Value(ul, vf);
                    _uCloseVal = p1.SquareDistanceTo(p2);
                    Point3D pm = surfAdapt.Value((uf + ul) / 2, vf);
                    umidVal = p1.SquareDistanceTo(pm);
                    distmin   = _uCloseVal;
                    for (int i = 1; i < nbpoints; i++)
                    {
                        double vparam = vf + (vl - vf) * i / (nbpoints - 1);
                        p1 = surfAdapt.Value(uf, vparam);
                        p2 = surfAdapt.Value(ul, vparam);
                        double dist = p1.SquareDistanceTo(p2);
                        if (dist > _uCloseVal)
                        {
                            _uCloseVal = dist;
                            pm          = surfAdapt.Value((uf + ul) / 2, vparam);
                            umidVal   = p1.SquareDistanceTo(pm);
                        }
                        else
                        {
                            distmin = Min(distmin, dist);
                        }
                    }

                    distmin = Math.Sqrt(distmin);
                    _uDelt = Min(_uDelt, 0.5 * surfAdapt.UResolution(distmin)); // #4 smh
                    break;
                }
            } // switch

            _gap       = Math.Sqrt(_uCloseVal);
            _uCloseVal = _gap;
        }

        if (umidVal > 0.0 && _uCloseVal > Math.Sqrt(umidVal))
        {
            _uCloseVal = RealLast();
            return false;
        }

        return (_uCloseVal <= prec);
    }

    /// <summary>
    /// Tells if the surface is spatially closed in V within <paramref name="preci"/> (see
    /// <see cref="IsUClosed"/>; a surface of revolution by the ends of its basis curve).
    /// </summary>
    public bool IsVClosed(double preci = -1)
    {
        double prec     = Max(preci, global::Kerf.Engine.Foundation.Precision.Confusion());
        double vmidVal = -1.0;
        if (_vCloseVal < 0)
        {
            // To compute: minimal computations
            Bounds(out double uf, out double ul, out double vf, out double vl); // modified by rln on 12/11/97 mySurf-> is deleted
            RestrictBounds(ref uf, ref ul, ref vf, ref vl);
            _vDelt = Math.Abs(vl - vf) / 20; // 2; rln S4135
                                              // because of the example when 10 was not enough
            if (_surf!.IsVClosed)
            {
                _vCloseVal = 0.0;
                _vDelt     = 0.0;
                _gap       = 0.0;
                return true;
            }

            // Adapted computations
            // #67 rln S4135
            SurfaceAdaptor surfAdapt = Adaptor3d()!;
            SurfaceKind surftype  = surfAdapt.GetType();
            if (_surf is TrimmedSurface)
            {
                surftype = OtherSurface;
            }

            switch (surftype)
            {
                case Plane:
                case SurfaceKind.Cone:
                case SurfaceKind.Cylinder:
                case SurfaceKind.Sphere:
                case SurfaceOfExtrusion:
                {
                    _vCloseVal = RealLast();
                    break;
                }
                case SurfaceOfRevolution:
                {
                    var        revol = (RevolutionSurface)_surf;
                    Curve3D crv   = revol.BasisCurve;
                    Point3D     p1    = crv.Value(crv.FirstParameter);
                    Point3D     p2    = crv.Value(crv.LastParameter);
                    _vCloseVal = p1.SquareDistanceTo(p2);
                    break;
                }
                case SurfaceKind.BSplineSurface:
                {
                    var    bs      = (global::Kerf.Geometry.BSplineSurface)_surf;
                    int    nbvp    = bs.VPoleCount;
                    double distmin = RealLast();
                    if (bs.IsVPeriodic)
                    {
                        _vCloseVal = 0;
                        _vDelt     = 0;
                    }
                    else if (nbvp < 3)
                    { // modified by rln on 12/11/97
                        _vCloseVal = RealLast();
                    }
                    else if (bs.IsVRational || bs.VMultiplicity(1) != bs.VDegree + 1 || // #6 //:h4
                             bs.VMultiplicity(bs.VKnotCount) != bs.VDegree + 1)
                    { // #6 //:h4
                        int    nbuk = bs.UKnotCount;
                        double u    = bs.UKnot(1);
                        Point3D p1   = surfAdapt.Value(u, vf);
                        Point3D p2   = surfAdapt.Value(u, vl);
                        _vCloseVal = p1.SquareDistanceTo(p2);
                        Point3D pm = surfAdapt.Value(u, (vf + vl) / 2.0);
                        vmidVal = p1.SquareDistanceTo(pm);
                        distmin  = _vCloseVal;
                        for (int i = 2; i <= nbuk; i++)
                        {
                            u  = 0.5 * (bs.UKnot(i - 1) + bs.UKnot(i));
                            p1 = surfAdapt.Value(u, vf);
                            p2 = surfAdapt.Value(u, vl);
                            double dist = p1.SquareDistanceTo(p2);
                            if (dist > _vCloseVal)
                            {
                                _vCloseVal = dist;
                                pm          = surfAdapt.Value(u, (vf + vl) / 2);
                                vmidVal    = p1.SquareDistanceTo(pm);
                            }
                            else
                            {
                                distmin = Min(distmin, dist);
                            }
                        }

                        distmin = Math.Sqrt(distmin);
                        _vDelt = Min(_vDelt, 0.5 * surfAdapt.VResolution(distmin)); // #4 smh
                    }
                    else
                    {
                        int nbup = bs.UPoleCount;
                        _vCloseVal = bs.Pole(1, 1).SquareDistanceTo(bs.Pole(1, nbvp));
                        vmidVal    = bs.Pole(1, 1).SquareDistanceTo(bs.Pole(1, nbvp / 2 + 1));
                        distmin     = _vCloseVal;
                        for (int i = 2; i <= nbup; i++)
                        {
                            double dist = bs.Pole(i, 1).SquareDistanceTo(bs.Pole(i, nbvp));
                            if (dist > _vCloseVal)
                            {
                                _vCloseVal = dist;
                                vmidVal    = bs.Pole(i, 1).SquareDistanceTo(bs.Pole(i, nbvp / 2 + 1));
                            }
                            else
                            {
                                distmin = Min(distmin, dist);
                            }
                        }

                        distmin = Math.Sqrt(distmin);
                        _vDelt = Min(_vDelt, 0.5 * surfAdapt.VResolution(distmin)); // #4 smh
                    }

                    break;
                }
                case SurfaceKind.BezierSurface:
                {
                    var    bz      = (global::Kerf.Geometry.BezierSurface)_surf;
                    int    nbvp    = bz.VPoleCount;
                    double distmin = RealLast();
                    if (nbvp < 3)
                    {
                        _vCloseVal = RealLast();
                    }
                    else
                    {
                        int nbup = bz.UPoleCount;
                        _vCloseVal = bz.Pole(1, 1).SquareDistanceTo(bz.Pole(1, nbvp));
                        vmidVal    = bz.Pole(1, 1).SquareDistanceTo(bz.Pole(1, nbvp / 2 + 1));
                        distmin     = _vCloseVal;
                        for (int i = 2; i <= nbup; i++)
                        {
                            double dist = bz.Pole(i, 1).SquareDistanceTo(bz.Pole(i, nbvp));
                            if (dist > _vCloseVal)
                            {
                                _vCloseVal = dist;
                                vmidVal    = bz.Pole(i, 1).SquareDistanceTo(bz.Pole(i, nbvp / 2 + 1));
                            }
                            else
                            {
                                distmin = Min(distmin, dist);
                            }
                        }

                        distmin = Math.Sqrt(distmin);
                        _vDelt = Min(_vDelt, 0.5 * surfAdapt.VResolution(distmin)); // #4 smh
                    }

                    break;
                }
                default:
                { // Geom_RectangularTrimmedSurface and Geom_OffsetSurface
                    double distmin  = RealLast();
                    int    nbpoints = 101; // can be revised
                    Point3D p1       = surfAdapt.Value(uf, vf);
                    Point3D p2       = surfAdapt.Value(uf, vl);
                    Point3D pm       = surfAdapt.Value(uf, (vf + vl) / 2);
                    _vCloseVal = p1.SquareDistanceTo(p2);
                    vmidVal    = p1.SquareDistanceTo(pm);
                    distmin     = _vCloseVal;
                    for (int i = 1; i < nbpoints; i++)
                    {
                        double uparam = uf + (ul - uf) * i / (nbpoints - 1);
                        p1 = surfAdapt.Value(uparam, vf);
                        p2 = surfAdapt.Value(uparam, vl);
                        double dist = p1.SquareDistanceTo(p2);
                        if (dist > _vCloseVal)
                        {
                            _vCloseVal = dist;
                            pm          = surfAdapt.Value(uparam, (vf + vl) / 2);
                            vmidVal    = p1.SquareDistanceTo(pm);
                        }
                        else
                        {
                            distmin = Min(distmin, dist);
                        }
                    }

                    distmin = Math.Sqrt(distmin);
                    _vDelt = Min(_vDelt, 0.5 * surfAdapt.VResolution(distmin)); // #4 smh
                    break;
                }
            } // switch

            _gap       = Math.Sqrt(_vCloseVal);
            _vCloseVal = _gap;
        }

        if (vmidVal > 0.0 && _vCloseVal > Math.Sqrt(vmidVal))
        {
            _vCloseVal = RealLast();
            return false;
        }

        return (_vCloseVal <= prec);
    }

    // Newton algo (S4030); returns 0 (failed), 1 or 2 (found, near a singular point)
    private int SurfaceNewton(in Point2D p2dPrev, in Point3D p3D, double preci, ref Point2D sol)
    {
        SurfaceAdaptor surfAdapt = Adaptor3d()!;
        Bounds(out double uf, out double ul, out double vf, out double vl);
        double du = surfAdapt.UResolution(preci);
        double dv = surfAdapt.VResolution(preci);
        double UF = uf - du, UL = ul + du;
        double VF = vf - dv, VL = vl + dv;

        double tol     = global::Kerf.Engine.Foundation.Precision.Confusion();
        double tol2    = tol * tol; //, rs2p=1e10;
        double u       = p2dPrev.X, v = p2dPrev.Y;
        var    rsfirst = new Vector3D(p3D.XYZ() - Value(u, v).XYZ()); // pdn
        // Upstream ec0167b409 (decision 2424): a hint already at the point is kept, at a singular point too, where 8.0.1
        // broke off at the zero normal and the fallback moved the hint along the singular edge
        if (!NativeHealingAtPoles.IsNative && rsfirst.SquareMagnitude <= Math.Min(preci * preci, tol2))
        {
            sol = p2dPrev;
            return 1;
        }

        for (int i = 0; i < 25; i++)
        {
            surfAdapt.D2(u, v, out Point3D pnt, out Vector3D ru, out Vector3D rv, out Vector3D ruu, out Vector3D rvv, out Vector3D ruv);

            // normal
            double ru2  = ru * ru, rv2 = rv * rv;
            Vector3D n    = ru ^ rv;
            double nrm2 = n.SquareMagnitude;
            if (nrm2 < 1e-10 || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(nrm2))
            {
                break; // n == 0, use standard
            }

            // discriminant
            var    rs   = new Vector3D(p3D.XYZ() - Value(u, v).XYZ());
            double rSuu = (rs * ruu);
            double rSvv = (rs * rvv);
            double rSuv = (rs * ruv);
            double d    = -nrm2 + rv2 * rSuu + ru2 * rSvv - 2 * rSuv * (ru * rv) + rSuv * rSuv - rSuu * rSvv;
            if (Math.Abs(d) < 1e-10)
            {
                break; // bad case; use standard
            }

            // compute step
            double fract = 1.0 / d;
            du = (rs * ((n ^ rv) + ru * rSvv - rv * rSuv)) * fract;
            dv = (rs * ((ru ^ n) + rv * rSuu - ru * rSuv)) * fract;
            u += du;
            v += dv;
            if (u < UF || u > UL || v < VF || v > VL)
            {
                break;
            }

            // test the step by uv and deviation from the solution
            double resolution = Max(1e-12, (u + v) * 10e-16);
            if (Math.Abs(du) + Math.Abs(dv) > resolution)
            {
                continue; // Precision::PConfusion()  continue;
            }

            // pdn PRO10109 4517: protect against wrong result
            double rs2 = rs.SquareMagnitude;
            if (rs2 > rsfirst.SquareMagnitude)
            {
                break;
            }

            double rsn = rs * n;
            if (rs2 - rsn * rsn / nrm2 > tol2)
            {
                break;
            }

            // OK, return the result
            sol = sol.WithCoord(u, v);

            return (nrm2 < 0.01 * ru2 * rv2 ? 2 : 1); //: q6
        }

        return 0;
    }

    /// <summary>
    /// Projects a point on the surface as <see cref="ValueOfUV"/>, starting from the previous solution
    /// <paramref name="p2dPrev"/> by Newton's method. A solution farther than <paramref name="maxpreci"/> (when
    /// positive) from the point is bad, and <see cref="UVFromIso"/> is tried; if Newton fails, ValueOfUV is used.
    /// </summary>
    public Point2D NextValueOfUV(in Point2D p2dPrev, in Point3D p3D, double preci, double maxpreci = -1.0)
    {
        SurfaceAdaptor surfAdapt = Adaptor3d()!;
        SurfaceKind surftype  = surfAdapt.GetType();

        switch (surftype)
        {
            case SurfaceKind.BezierSurface:
            case SurfaceKind.BSplineSurface:
            case SurfaceOfExtrusion:
            case SurfaceOfRevolution:
            case SurfaceKind.OffsetSurface:
            {
                if (surftype == SurfaceKind.BSplineSurface)
                {
                    global::Kerf.Geometry.BSplineSurface bSpline = surfAdapt.BSpline();

                    // Check near to knot position ~ near to C0 points on U isoline.
                    if (surfAdapt.UContinuity() == C0)
                    {
                        int minIndex = bSpline.FirstUKnotIndex();
                        int maxIndex = bSpline.LastUKnotIndex();
                        for (int idx = minIndex; idx <= maxIndex; ++idx)
                        {
                            double knot = bSpline.UKnot(idx);
                            if (Math.Abs(knot - p2dPrev.X) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                return ValueOfUV(p3D, preci);
                            }
                        }
                    }

                    // Check near to knot position ~ near to C0 points on U isoline.
                    if (surfAdapt.VContinuity() == C0)
                    {
                        int minIndex = bSpline.FirstVKnotIndex();
                        int maxIndex = bSpline.LastVKnotIndex();
                        for (int idx = minIndex; idx <= maxIndex; ++idx)
                        {
                            double knot = bSpline.VKnot(idx);
                            if (Math.Abs(knot - p2dPrev.Y) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                return ValueOfUV(p3D, preci);
                            }
                        }
                    }
                }

                Point2D sol = new Point2D();
                int      res = SurfaceNewton(p2dPrev, p3D, preci, ref sol);
                if (res != 0)
                {
                    double gap = p3D.DistanceTo(Value(sol));
                    if (res == 2 || //: q6 abv 19 Mar 99: protect against strange attractors
                        (maxpreci > 0.0 && gap - maxpreci > global::Kerf.Engine.Foundation.Precision.Confusion()))
                    { //: q1: check with maxpreci
                        double u = sol.X, v = sol.Y;
                        _gap = UVFromIso(p3D, preci, ref u, ref v);
                        if (gap >= _gap)
                        {
                            return new Point2D(u, v);
                        }
                    }

                    _gap = gap;
                    return sol;
                }

                break;
            }
            default:
                break;
        }

        return ValueOfUV(p3D, preci);
    }

    /// <summary>
    /// Computes the parameters on the surface of the projection of a 3D point: analytically on elementary surfaces,
    /// else by Extrema_ExtPS improved by Newton and by projection on iso-lines (<see cref="UVFromIso"/>), which also
    /// handles points near the surface's boundaries and failures of Extrema. Sets <see cref="Gap"/> to the distance
    /// from the point to the solution (-1 and (0, 0) on surfaces it doesn't handle).
    /// </summary>
    public Point2D ValueOfUV(in Point3D p3D, double preci)
    {
        SurfaceAdaptor surfAdapt = Adaptor3d()!;
        double              s         = 0.0, t = 0.0;
        _gap = -1.0;          // to be computed
        bool computed = true; // a priori

        Bounds(out double uf, out double ul, out double vf, out double vl); // modified by rln on 12/11/97 mySurf-> is deleted

        { //: c9 abv 3 Mar 98: UKI60107-1 #350: to prevent 'catch' from catching exception raising below it
            try
            { // ajout CKY 30-DEC-1997 (cf ProStep TR6 r_89-ug)
                SurfaceKind surftype = surfAdapt.GetType();
                switch (surftype)
                {
                    case Plane:
                    {
                        Plane3D plane = surfAdapt.Plane();
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(plane, p3D, out s, out t);
                        break;
                    }
                    case SurfaceKind.Cylinder:
                    {
                        Cylinder3D cylinder = surfAdapt.Cylinder();
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cylinder, p3D, out s, out t);
                        s += ShapeAnalysisAnalysis.AdjustByPeriod(s, 0.5 * (uf + ul), 2 * M_PI);
                        break;
                    }
                    case SurfaceKind.Cone:
                    {
                        Cone3D cone = surfAdapt.Cone();
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cone, p3D, out s, out t);
                        s += ShapeAnalysisAnalysis.AdjustByPeriod(s, 0.5 * (uf + ul), 2 * M_PI);
                        break;
                    }
                    case SurfaceKind.Sphere:
                    {
                        Sphere3D sphere = surfAdapt.Sphere();
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(sphere, p3D, out s, out t);
                        s += ShapeAnalysisAnalysis.AdjustByPeriod(s, 0.5 * (uf + ul), 2 * M_PI);
                        break;
                    }
                    case SurfaceKind.Torus:
                    {
                        Torus3D torus = surfAdapt.Torus();
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(torus, p3D, out s, out t);
                        s += ShapeAnalysisAnalysis.AdjustByPeriod(s, 0.5 * (uf + ul), 2 * M_PI);
                        t += ShapeAnalysisAnalysis.AdjustByPeriod(t, 0.5 * (vf + vl), 2 * M_PI);
                        break;
                    }
                    case SurfaceKind.BezierSurface:
                    case SurfaceKind.BSplineSurface:
                    case SurfaceOfExtrusion:
                    case SurfaceOfRevolution:
                    case SurfaceKind.OffsetSurface: //: d0 abv 3 Mar 98: UKI60107-1 #350
                    {
                        s = (uf + ul) / 2;
                        t = (vf + vl) / 2; // there will be at least something
                                           // pdn to fix hangs PRO17015
                        if ((surftype == SurfaceOfExtrusion) && global::Kerf.Engine.Foundation.Precision.IsInfinite(uf)
                            && global::Kerf.Engine.Foundation.Precision.IsInfinite(ul))
                        {
                            // conic case
                            var      prev     = new Point2D(s, t);
                            Point2D solution = new Point2D();
                            if (SurfaceNewton(prev, p3D, preci, ref solution) != 0)
                            {
                                return solution;
                            }

                            uf = -500;
                            ul = 500;
                        }

                        RestrictBounds(ref uf, ref ul, ref vf, ref vl);

                        //: 30 by abv 2.12.97: speed optimization
                        // code is taken from GeomAPI_ProjectPointOnSurf
                        if (!_extOK)
                        {
                            //  Force the call to IsU-VClosed
                            if (_uCloseVal < 0)
                            {
                                IsUClosed();
                            }

                            if (_vCloseVal < 0)
                            {
                                IsVClosed();
                            }

                            double du = 0.0, dv = 0.0;
                            // extension of the surface range is limited to non-offset surfaces as the latter
                            // can throw exception (e.g. Geom_UndefinedValue) when computing value - see id23943
                            if (_surf is not global::Kerf.Geometry.OffsetSurface)
                            {
                                // modified by rln during fixing CSR # BUC60035 entity #D231
                                du = Min(_uDelt, surfAdapt.UResolution(preci));
                                dv = Min(_vDelt, surfAdapt.VResolution(preci));
                            }

                            double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
                            _extPS.SetFlag(ExtFlag.ExtFlag_MIN);
                            _extPS.Initialize(surfAdapt, uf - du, ul + du, vf - dv, vl + dv, tol, tol);
                            _extOK = true;
                        }

                        _extPS.Perform(p3D);
                        int nPSurf = (_extPS.IsDone ? _extPS.ExtCount : 0);

                        if (nPSurf > 0)
                        {
                            double dist2Min = _extPS.SquareDistance(1);
                            int    indMin   = 1;
                            for (int sol = 2; sol <= nPSurf; sol++)
                            {
                                double dist2 = _extPS.SquareDistance(sol);
                                if (dist2Min > dist2)
                                {
                                    dist2Min = dist2;
                                    indMin   = sol;
                                }
                            }

                            _extPS.Point(indMin).Parameter(out s, out t);
                            // PTV 26.06.2002 WORKAROUND protect OCC486. Remove after fix bug.
                            // file CEA_cuve-V5.igs Entityes 244, 259, 847, 925
                            // if project point3D on SurfaceOfRevolution Extreme recompute 2d point, but
                            // returns an old distance from 3d to solution :-(
                            Point3D checkPnt = surfAdapt.Value(s, t);
                            dist2Min = p3D.SquareDistanceTo(checkPnt);
                            // end of WORKAROUND
                            double disSurf = Math.Sqrt(dist2Min); //, disCurv =1.e10;

                            // Test of bad projections on the boundaries:
                            double uu = s, vv = t, distMinOnIso = RealLast(); // myGap;

                            // test added by rln on 08/12/97
                            bool possLockal = false; //: study S4030 (optimizing)
                            if (disSurf > preci)
                            {
                                var pp = new Point2D(uu, vv);
                                if (SurfaceNewton(pp, p3D, preci, ref pp) != 0)
                                { //: q2 abv 16 Mar 99: PRO7226 #412920
                                    double dist = p3D.DistanceTo(Value(pp));
                                    if (dist < disSurf)
                                    {
                                        disSurf = dist;
                                        s       = uu = pp.X;
                                        t       = vv = pp.Y;
                                    }
                                }

                                if (disSurf < 10 * preci)
                                {
                                    if (_surf!.Continuity != C0)
                                    {
                                        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
                                        surfAdapt.D1(uu, vv, out Point3D pnt, out Vector3D d1U, out Vector3D d1V);
                                        Vector3D b    = d1U.Crossed(d1V);
                                        var    a    = new Vector3D(pnt, p3D);
                                        double ab   = a.Dot(b);
                                        double nrm2 = b.SquareMagnitude;
                                        if (nrm2 > 1e-10)
                                        {
                                            double dist = a.SquareMagnitude - (ab * ab) / nrm2;
                                            possLockal = (dist < tol * tol);
                                        }
                                    }
                                }

                                if (!possLockal)
                                {
                                    distMinOnIso = UVFromIso(p3D, preci, ref uu, ref vv);
                                }
                            }

                            if (disSurf > distMinOnIso)
                            {
                                // take the parameters UU and VV
                                s     = uu;
                                t     = vv;
                                _gap = distMinOnIso;
                            }
                            else
                            {
                                _gap = disSurf;
                            }
                        }
                        else
                        {
                            // try on the boundaries
                            double uu = s, vv = t;
                            _gap = UVFromIso(p3D, preci, ref uu, ref vv);
                            s     = uu;
                            t     = vv;
                        }
                    }
                        break;

                    default:
                        computed = false;
                        break;
                }
            } // end Try ValueOfUV (CKY 30-DEC-1997)
            catch (Exception caught) when (Refusal.IsFailure(caught))
            {
                // No reason, but who knows. Better to return something wrong than to stop.
                s = (global::Kerf.Engine.Foundation.Precision.IsInfinite(uf)) ? 0 : (uf + ul) / 2.0;
                t = (global::Kerf.Engine.Foundation.Precision.IsInfinite(vf)) ? 0 : (vf + vl) / 2.0;
            }
        } //: c9

        // szv#4:S4163:12Mar99 waste raise
        if (computed)
        {
            if (_gap <= 0)
            {
                _gap = p3D.DistanceTo(surfAdapt.Value(s, t));
            }
        }
        else
        {
            _gap = -1.0;
            s     = 0.0;
            t     = 0.0;
        }

        return new Point2D(s, t);
    }

    /// <summary>
    /// Refines the parameters (U, V) of a point by projecting it on iso-lines: the surface's boundaries, the
    /// iso-lines through (U, V), then up to 5 alternating iso-lines through the new U and V. Returns the best distance
    /// between the point and the surface at the resulting (U, V), or a very large value on failure.
    /// </summary>
    public double UVFromIso(in Point3D p3d, double preci, ref double u, ref double v)
    {
        //  Projection that considers the isos as follows:
        //  the 4 boundaries, plus the isos in U and in V
        //  Often, one of the two is good...
        double theMin = RealLast();

        Point3D pntres;
        double cf, cl, uu, vv;

        //  Starting the search: point already found (?)
        uu = u;
        vv = v;
        Point3D depart = _adSur!.Value(u, v);
        theMin = depart.DistanceTo(p3d);

        if (theMin < preci / 10)
        {
            return theMin; // it was already OK
        }

        ComputeBoxes();
        if (_isoUF is null || _isoUL is null || _isoVF is null || _isoVL is null)
        {
            // no isolines
            // no more precise computation
            return theMin;
        }

        try
        { // RAJOUT
            // pdn Create BndBox containing point;
            var pBox = new Box();
            pBox.Set(p3d);

            // modified by rln on 04/12/97 in order to use these variables later
            bool        uv  = true;
            double      par = 0.0, other = 0.0, dist = 0.0;
            Curve3D? iso;
            var         isoCurve = new IsoCurve(Adaptor3d()!);
            for (int num = 0; num < 6; num++)
            {
                uv = (num < 3); // 0-1-2 : iso-U  3-4-5 : iso-V
                if (!(Adaptor3d()!.GetType() == SurfaceKind.OffsetSurface))
                {
                    Box? isoBox = null;
                    iso = null;
                    switch (num)
                    {
                        case 0:
                            par      = _uf;
                            iso      = _isoUF;
                            isoBox = _bndUF;
                            break;
                        case 1:
                            par      = _ul;
                            iso      = _isoUL;
                            isoBox = _bndUL;
                            break;
                        case 2:
                            par = u;
                            iso = UIso(u);
                            break;
                        case 3:
                            par      = _vf;
                            iso      = _isoVF;
                            isoBox = _bndVF;
                            break;
                        case 4:
                            par      = _vl;
                            iso      = _isoVL;
                            isoBox = _bndVL;
                            break;
                        case 5:
                            par = v;
                            iso = VIso(v);
                            break;
                        default:
                            break;
                    }

                    //    Go on it
                    if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(par) && iso is not null)
                    {
                        if (isoBox.HasValue && isoBox.Value.Distance(pBox) > theMin)
                        {
                            continue;
                        }

                        cf = iso.FirstParameter;
                        cl = iso.LastParameter;

                        RestrictBounds(ref cf, ref cl);
                        dist = new CurveAnalysis().Project(iso, p3d, preci, out pntres, out other, cf, cl, false);
                        if (dist < theMin)
                        {
                            theMin = dist;
                            //: q6	if (UV) VV = other;  else UU = other;
                            //  Along an iso-U, compute the best V
                            uu = (uv ? par : other);
                            vv = (uv ? other : par); //: q6: uncommented
                        }
                    }
                }
                else
                {
                    CurveAdaptorBase? adaptor  = null;
                    var              geomCurve = new CurveAdaptor();

                    Box? isoBox = null;
                    switch (num)
                    {
                        case 0:
                            par = _uf;
                            geomCurve.Load(_isoUF);
                            adaptor = geomCurve;
                            isoBox  = _bndUF;
                            break;
                        case 1:
                            par = _ul;
                            geomCurve.Load(_isoUL);
                            adaptor = geomCurve;
                            isoBox  = _bndUL;
                            break;
                        case 2:
                            par = u;
                            isoCurve.Load(IsoU, u);
                            adaptor = isoCurve;
                            break;
                        case 3:
                            par = _vf;
                            geomCurve.Load(_isoVF);
                            adaptor = geomCurve;
                            isoBox  = _bndVF;
                            break;
                        case 4:
                            par = _vl;
                            geomCurve.Load(_isoVL);
                            adaptor = geomCurve;
                            isoBox  = _bndVL;
                            break;
                        case 5:
                            par = v;
                            isoCurve.Load(IsoV, v);
                            adaptor = isoCurve;
                            break;
                        default:
                            break;
                    }

                    if (isoBox.HasValue && isoBox.Value.Distance(pBox) > theMin)
                    {
                        continue;
                    }

                    dist = new CurveAnalysis().Project(adaptor!, p3d, preci, out pntres, out other);
                    if (dist < theMin)
                    {
                        theMin = dist;
                        uu     = (uv ? par : other);
                        vv     = (uv ? other : par); //: q6: uncommented
                    }
                }
            }

            // added by rln on 04/12/97 iterational process
            double prevU    = u, prevV = v;
            int    maxIters = 5, iters = 0;
            if (!(Adaptor3d()!.GetType() == SurfaceKind.OffsetSurface))
            {
                while (((prevU != uu) || (prevV != vv)) && (iters < maxIters) && (theMin > preci))
                {
                    prevU = uu;
                    prevV = vv;
                    if (uv)
                    {
                        par = uu;
                        iso = UIso(uu);
                    }
                    else
                    {
                        par = vv;
                        iso = VIso(vv);
                    }

                    if (iso is not null)
                    {
                        cf = iso.FirstParameter;
                        cl = iso.LastParameter;
                        RestrictBounds(ref cf, ref cl);
                        dist = new CurveAnalysis().Project(iso, p3d, preci, out pntres, out other, cf, cl, false);
                        if (dist < theMin)
                        {
                            theMin = dist;
                            if (uv)
                            {
                                vv = other;
                            }
                            else
                            {
                                uu = other;
                            }
                        }
                    }

                    uv = !uv;
                    if (uv)
                    {
                        par = uu;
                        iso = UIso(uu);
                    }
                    else
                    {
                        par = vv;
                        iso = VIso(vv);
                    }

                    if (iso is not null)
                    {
                        cf = iso.FirstParameter;
                        cl = iso.LastParameter;
                        RestrictBounds(ref cf, ref cl);
                        dist = new CurveAnalysis().Project(iso, p3d, preci, out pntres, out other, cf, cl, false);
                        if (dist < theMin)
                        {
                            theMin = dist;
                            if (uv)
                            {
                                vv = other;
                            }
                            else
                            {
                                uu = other;
                            }
                        }
                    }

                    uv = !uv;
                    iters++;
                }
            }
            else
            {
                while (((prevU != uu) || (prevV != vv)) && (iters < maxIters) && (theMin > preci))
                {
                    prevU = uu;
                    prevV = vv;
                    if (uv)
                    {
                        par = uu;
                        isoCurve.Load(IsoU, uu);
                    }
                    else
                    {
                        par = vv;
                        isoCurve.Load(IsoV, vv);
                    }

                    cf = isoCurve.FirstParameter();
                    cl = isoCurve.LastParameter();
                    RestrictBounds(ref cf, ref cl);
                    dist = new CurveAnalysis().Project(isoCurve, p3d, preci, out pntres, out other);
                    if (dist < theMin)
                    {
                        theMin = dist;
                        if (uv)
                        {
                            vv = other;
                        }
                        else
                        {
                            uu = other;
                        }
                    }

                    uv = !uv;
                    if (uv)
                    {
                        par = uu;
                        isoCurve.Load(IsoU, uu);
                    }
                    else
                    {
                        par = vv;
                        isoCurve.Load(IsoV, vv);
                    }

                    cf = isoCurve.FirstParameter();
                    cl = isoCurve.LastParameter();
                    RestrictBounds(ref cf, ref cl);
                    dist = new CurveAnalysis().ProjectAct(isoCurve, p3d, preci, out pntres, out other);
                    if (dist < theMin)
                    {
                        theMin = dist;
                        if (uv)
                        {
                            vv = other;
                        }
                        else
                        {
                            uu = other;
                        }
                    }

                    uv = !uv;
                    iters++;
                }
            }

            u = uu;
            v = vv;
        } // end try RAJOUT
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            theMin = RealLast(); // the starting theMin
        }

        return theMin;
    }

    private void SortSingularities()
    {
        for (int i = 0; i < _nbDeg - 1; i++)
        {
            double minPreci = _preci[i];
            int    minIndex = i;
            for (int j = i + 1; j < _nbDeg; j++)
            {
                if (minPreci > _preci[j])
                {
                    minPreci = _preci[j];
                    minIndex = j;
                }
            }

            if (minIndex != i)
            {
                _preci[minIndex] = _preci[i];
                _preci[i]        = minPreci;
                Point3D tmpP3d = _p3d[minIndex];
                _p3d[minIndex] = _p3d[i];
                _p3d[i]        = tmpP3d;
                Point2D tmpP2d = _firstP2d[minIndex];
                _firstP2d[minIndex] = _firstP2d[i];
                _firstP2d[i]        = tmpP2d;
                tmpP2d               = _lastP2d[minIndex];
                _lastP2d[minIndex]  = _lastP2d[i];
                _lastP2d[i]         = tmpP2d;
                double tmpPar = _firstPar[minIndex];
                _firstPar[minIndex] = _firstPar[i];
                _firstPar[i]        = tmpPar;
                tmpPar               = _lastPar[minIndex];
                _lastPar[minIndex]  = _lastPar[i];
                _lastPar[i]         = tmpPar;
                bool tmpUIsoDeg = _uIsoDeg[minIndex];
                _uIsoDeg[minIndex] = _uIsoDeg[i];
                _uIsoDeg[i]        = tmpUIsoDeg;
            }
        }
    }

    /// <summary>Returns the bounds of the surface (Geom_Surface's, buffered, or those set by SetDomain).</summary>
    public void Bounds(out double ufirst, out double ulast, out double vfirst, out double vlast)
    {
        ufirst = _uf;
        ulast  = _ul;
        vfirst = _vf;
        vlast  = _vl;
    }

    /// <summary>Returns the smallest precision for which the surface is U-closed.</summary>
    public double UCloseVal() => _uCloseVal;

    /// <summary>Returns the smallest precision for which the surface is V-closed.</summary>
    public double VCloseVal() => _vCloseVal;

    private void ComputeBoxes()
    {
        if (_isoBoxes)
        {
            return;
        }

        _isoBoxes = true;
        ComputeBoundIsos();
        if (_isoUF is not null)
        {
            Add3dCurve.Add(new CurveAdaptor(_isoUF), global::Kerf.Engine.Foundation.Precision.Confusion(), ref _bndUF);
        }

        if (_isoUL is not null)
        {
            Add3dCurve.Add(new CurveAdaptor(_isoUL), global::Kerf.Engine.Foundation.Precision.Confusion(), ref _bndUL);
        }

        if (_isoVF is not null)
        {
            Add3dCurve.Add(new CurveAdaptor(_isoVF), global::Kerf.Engine.Foundation.Precision.Confusion(), ref _bndVF);
        }

        if (_isoVL is not null)
        {
            Add3dCurve.Add(new CurveAdaptor(_isoVL), global::Kerf.Engine.Foundation.Precision.Confusion(), ref _bndVL);
        }
    }

    /// <summary>The bounding box of the iso-curve at the first U bound.</summary>
    public Box GetBoxUF()
    {
        ComputeBoxes();
        return _bndUF;
    }

    /// <summary>The bounding box of the iso-curve at the last U bound.</summary>
    public Box GetBoxUL()
    {
        ComputeBoxes();
        return _bndUL;
    }

    /// <summary>The bounding box of the iso-curve at the first V bound.</summary>
    public Box GetBoxVF()
    {
        ComputeBoxes();
        return _bndVF;
    }

    /// <summary>The bounding box of the iso-curve at the last V bound.</summary>
    public Box GetBoxVL()
    {
        ComputeBoxes();
        return _bndVL;
    }
}
