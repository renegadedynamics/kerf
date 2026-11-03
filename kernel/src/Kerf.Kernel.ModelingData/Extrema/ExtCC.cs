// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtCC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtCC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Modified by MPS (june 96) : fix for the trap in the case line/Bezier
// Modified by MPS (may 97) : PRO 7598
//                            sort solutions to avoid returning the same
//                            solution multiple times

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extremum distances between two 3D curves on parameter ranges. A line against a line or conic, and two
/// circles, are solved analytically (<see cref="ExtElC"/>); other pairs by global optimisation
/// (<see cref="Ecc"/>). Parallel curves (lines, circles) report <see cref="IsParallel"/> with one distance.
/// </summary>
/// <remarks>
/// An algorithm object (decision 117): construct it with the curves, or <c>Initialize</c> (or <c>SetCurve</c>,
/// <c>SetRange</c>, <c>SetTolerance</c>) and <c>Perform</c>. Keeps references to the curves, as OCCT does. OCCT
/// disallows copies.
/// </remarks>
public class ExtCC
{
    private bool _isFindSingleSolution; // Default value is false.
    private readonly Ecc _ecc;
    private bool _done;
    private bool _isParallel;
    private readonly NCollection_Sequence<POnCurv> _mypoints = new();
    private readonly NCollection_Sequence<double> _sqDist = new();
    private readonly CurveAdaptorBase?[] _c = new CurveAdaptorBase?[2];
    private readonly double[] _inf = new double[2];
    private readonly double[] _sup = new double[2];
    private readonly double[] _tol = new double[2];
    private Point3D _p1f = new();
    private Point3D _p1l = new();
    private Point3D _p2f = new();
    private Point3D _p2l = new();
    private double _mydist11;
    private double _mydist12;
    private double _mydist21;
    private double _mydist22;

    /// <summary>An empty algorithm with the tolerances of the two curves.</summary>
    public ExtCC(double tolC1 = 1.0e-10, double tolC2 = 1.0e-10)
    {
        _isFindSingleSolution = false;
        _ecc = new Ecc();
        _done = false;
        _isParallel = false;
        _c[0] = null;
        _c[1] = null;
        _inf[0] = _inf[1] = -global::Kerf.Engine.Foundation.Precision.Infinite();
        _sup[0] = _sup[1] = global::Kerf.Engine.Foundation.Precision.Infinite();
        _tol[0] = tolC1;
        _tol[1] = tolC2;
        _mydist11 = _mydist12 = _mydist21 = _mydist22 = RealFirst();
    }

    /// <summary>The extremum distances between C1 on [U1, U2] and C2 on [V1, V2].</summary>
    public ExtCC(CurveAdaptorBase c1, CurveAdaptorBase c2, double u1, double u2, double v1, double v2,
        double tolC1 = 1.0e-10, double tolC2 = 1.0e-10)
    {
        _isFindSingleSolution = false;
        _ecc = new Ecc(c1, c2, u1, u2, v1, v2);
        _done = false;
        SetCurve(1, c1, u1, u2);
        SetCurve(2, c2, v1, v2);
        SetTolerance(1, tolC1);
        SetTolerance(2, tolC2);
        _mydist11 = _mydist12 = _mydist21 = _mydist22 = RealFirst();
        Perform();
    }

    /// <summary>The extremum distances between C1 and C2 on their whole ranges.</summary>
    public ExtCC(CurveAdaptorBase c1, CurveAdaptorBase c2, double tolC1 = 1.0e-10, double tolC2 = 1.0e-10)
    {
        _isFindSingleSolution = false;
        _ecc = new Ecc(c1, c2);
        _done = false;
        SetCurve(1, c1, c1.FirstParameter(), c1.LastParameter());
        SetCurve(2, c2, c2.FirstParameter(), c2.LastParameter());
        SetTolerance(1, tolC1);
        SetTolerance(2, tolC2);
        _mydist11 = _mydist12 = _mydist21 = _mydist22 = RealFirst();
        Perform();
    }

    /// <summary>Sets the curves on their whole ranges and the tolerances; <c>Perform</c> then computes.</summary>
    public void Initialize(CurveAdaptorBase c1, CurveAdaptorBase c2, double tolC1 = 1.0e-10, double tolC2 = 1.0e-10)
    {
        // myECC will be re-initialized by Perform()
        _done = false;
        SetCurve(1, c1, c1.FirstParameter(), c1.LastParameter());
        SetCurve(2, c2, c2.FirstParameter(), c2.LastParameter());
        SetTolerance(1, tolC1);
        SetTolerance(2, tolC2);
        _mydist11 = _mydist12 = _mydist21 = _mydist22 = RealFirst();
    }

    /// <summary>Sets C1 on [U1, U2], C2 on [V1, V2] and the tolerances; <c>Perform</c> then computes.</summary>
    public void Initialize(CurveAdaptorBase c1, CurveAdaptorBase c2, double u1, double u2, double v1, double v2,
        double tolC1 = 1.0e-10, double tolC2 = 1.0e-10)
    {
        // myECC will be re-initialized by Perform()
        _done = false;
        SetCurve(1, c1, u1, u2);
        SetCurve(2, c2, v1, v2);
        SetTolerance(1, tolC1);
        SetTolerance(2, tolC2);
        _mydist11 = _mydist12 = _mydist21 = _mydist22 = RealFirst();
    }

    /// <summary>Sets the curve of rank theRank (1 or 2), keeping its range.</summary>
    /// <exception cref="OutOfRangeException">theRank is not 1 or 2.</exception>
    public void SetCurve(int rank, CurveAdaptorBase c)
    {
        Standard_OutOfRange_Raise_if(rank < 1 || rank > 2, "Extrema_ExtCC::SetCurve()");
        int ind = rank - 1;
        _c[ind] = c;
    }

    /// <summary>Sets the curve of rank theRank (1 or 2) on [Uinf, Usup].</summary>
    /// <exception cref="OutOfRangeException">theRank is not 1 or 2.</exception>
    public void SetCurve(int rank, CurveAdaptorBase c, double uinf, double usup)
    {
        SetCurve(rank, c);
        SetRange(rank, uinf, usup);
    }

    /// <summary>Sets the range of the curve of rank theRank (1 or 2).</summary>
    /// <exception cref="OutOfRangeException">theRank is not 1 or 2.</exception>
    public void SetRange(int rank, double uinf, double usup)
    {
        Standard_OutOfRange_Raise_if(rank < 1 || rank > 2, "Extrema_ExtCC::SetRange()");
        int ind = rank - 1;
        _inf[ind] = uinf;
        _sup[ind] = usup;
    }

    /// <summary>Sets the tolerance of the curve of rank theRank (1 or 2).</summary>
    /// <exception cref="OutOfRangeException">theRank is not 1 or 2.</exception>
    public void SetTolerance(int rank, double tol)
    {
        Standard_OutOfRange_Raise_if(rank < 1 || rank > 2, "Extrema_ExtCC::SetTolerance()");
        int ind = rank - 1;
        _tol[ind] = tol;
    }

    /// <summary>Computes the extremum distances.</summary>
    /// <exception cref="NullObjectException">A curve is not set.</exception>
    public void Perform()
    {
        Standard_NullObject_Raise_if(_c[0] is null || _c[1] is null, "Extrema_ExtCC::Perform()");
        _ecc.SetParams(_c[0]!, _c[1]!, _inf[0], _sup[0], _inf[1], _sup[1]);
        _ecc.SetTolerance(Min(_tol[0], _tol[1]));
        _ecc.SetSingleSolutionFlag(GetSingleSolutionFlag());
        _done = false;
        _mypoints.Clear();
        _sqDist.Clear();
        _isParallel = false;

        CurveKind type1 = _c[0]!.GetType();
        CurveKind type2 = _c[1]!.GetType();
        double u11, u12, u21, u22, tol = Min(_tol[0], _tol[1]);

        u11 = _inf[0];
        u12 = _sup[0];
        u21 = _inf[1];
        u22 = _sup[1];

        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u11))
        {
            _p1f = CurveTool.Value(_c[0]!, u11);
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u12))
        {
            _p1l = CurveTool.Value(_c[0]!, u12);
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u21))
        {
            _p2f = CurveTool.Value(_c[1]!, u21);
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u22))
        {
            _p2l = CurveTool.Value(_c[1]!, u22);
        }

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u11) || global::Kerf.Engine.Foundation.Precision.IsInfinite(u21))
        {
            _mydist11 = RealLast();
        }
        else
        {
            _mydist11 = _p1f.SquareDistanceTo(_p2f);
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u11) || global::Kerf.Engine.Foundation.Precision.IsInfinite(u22))
        {
            _mydist12 = RealLast();
        }
        else
        {
            _mydist12 = _p1f.SquareDistanceTo(_p2l);
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u12) || global::Kerf.Engine.Foundation.Precision.IsInfinite(u21))
        {
            _mydist21 = RealLast();
        }
        else
        {
            _mydist21 = _p1l.SquareDistanceTo(_p2f);
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u12) || global::Kerf.Engine.Foundation.Precision.IsInfinite(u22))
        {
            _mydist22 = RealLast();
        }
        else
        {
            _mydist22 = _p1l.SquareDistanceTo(_p2l);
        }

        // Depending on the types of curves, the algorithm is chosen:
        //- _ExtElC, when one of the curves is a line and the other is elementary,
        //    or there are two circles;
        //- _GenExtCC, in all other cases
        if ((type1 == Line && type2 <= Parabola)
            || (type2 == Line && type1 <= Parabola))
        {
            // analytical case - one curve is always a line
            int ind1 = 0, ind2 = 1;
            CurveKind aType2 = type2;
            bool isInverse = (type1 > type2);
            if (isInverse)
            {
                // algorithm uses inverse order of arguments
                ind1 = 1;
                ind2 = 0;
                aType2 = type1;
            }
            switch (aType2)
            {
                case Line:
                {
                    ExtElC xtrem = new(_c[ind1]!.Line(), _c[ind2]!.Line(), tol);
                    PrepareResults(xtrem, isInverse, u11, u12, u21, u22);
                    break;
                }
                case Circle:
                {
                    ExtElC xtrem = new(_c[ind1]!.Line(), _c[ind2]!.Circle(), tol);
                    PrepareResults(xtrem, isInverse, u11, u12, u21, u22);
                    break;
                }
                case Ellipse:
                {
                    ExtElC xtrem = new(_c[ind1]!.Line(), _c[ind2]!.Ellipse());
                    PrepareResults(xtrem, isInverse, u11, u12, u21, u22);
                    break;
                }
                case Hyperbola:
                {
                    ExtElC xtrem = new(_c[ind1]!.Line(), _c[ind2]!.Hyperbola());
                    PrepareResults(xtrem, isInverse, u11, u12, u21, u22);
                    break;
                }
                case Parabola:
                {
                    ExtElC xtrem = new(_c[ind1]!.Line(), _c[ind2]!.Parabola());
                    PrepareResults(xtrem, isInverse, u11, u12, u21, u22);
                    break;
                }
                default:
                    break;
            }
        }
        else if (type1 == Circle && type2 == Circle)
        {
            // analytical case - two circles
            bool bIsDone;
            ExtElC ccXtrem = new(_c[0]!.Circle(), _c[1]!.Circle());
            bIsDone = ccXtrem.IsDone;
            if (bIsDone)
            {
                PrepareResults(ccXtrem, false, u11, u12, u21, u22);
            }
            else
            {
                _ecc.Perform();
                PrepareResults(_ecc, u11, u12, u21, u22);
            }
        }
        else
        {
            _ecc.Perform();
            PrepareResults(_ecc, u11, u12, u21, u22);
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _done;

    /// <summary>True if the curves are parallel (lines, circles); then there is one distance and no points, unless
    /// the trimmed ranges made the algorithm choose points.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public bool IsParallel
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }

            return _isParallel;
        }
    }

    /// <summary>The Nth square distance.</summary>
    /// <exception cref="OutOfRangeException">N is not in 1..NbExt().</exception>
    public double SquareDistance(int n = 1)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        return _sqDist.Value(n);
    }

    /// <summary>The number of extremum distances.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public int ExtCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _sqDist.Length();
        }
    }

    /// <summary>The points of the Nth extremum, on the first and on the second curve.</summary>
    /// <exception cref="OutOfRangeException">N is not in 1..NbExt().</exception>
    public void Points(int n, out POnCurv p1, out POnCurv p2)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        p1 = _mypoints.Value(2 * n - 1);
        p2 = _mypoints.Value(2 * n);
    }

    /// <summary>The square distances between the ends of the ranges and the end points (RealLast for an infinite
    /// end).</summary>
    public void TrimmedSquareDistances(out double dist11, out double dist12, out double dist21, out double dist22,
        out Point3D p11, out Point3D p12, out Point3D p21, out Point3D p22)
    {
        dist11 = _mydist11;
        dist12 = _mydist12;
        dist21 = _mydist21;
        dist22 = _mydist22;
        p11 = _p1f;
        p12 = _p1l;
        p21 = _p2f;
        p22 = _p2l;
    }

    /// <summary>Asks for one solution only (faster, for the global optimisation).</summary>
    public void SetSingleSolutionFlag(bool flag) => _isFindSingleSolution = flag;

    public bool GetSingleSolutionFlag() => _isFindSingleSolution;

    // Clears all found extremas.
    // This method does not change any flags (e.g. Done or IsParallel)
    protected void ClearSolutions()
    {
        _sqDist.Clear();
        _mypoints.Clear();
    }

    protected void PrepareParallelResult(double ut11, double ut12, double ut21, double ut22,
        double sqDist)
    {
        if (!_isParallel)
        {
            return;
        }

        CurveKind type1 = CurveTool.GetType(_c[0]!);
        CurveKind type2 = CurveTool.GetType(_c[1]!);
        if (((type1 != Line) && (type1 != Circle))
            || ((type2 != Line) && (type2 != Circle)))
        {
            _sqDist.Append(sqDist);
            _done = true;
            _isParallel = true;
            return;
        }

        // Parallel case is only for line-line, circle-circle and circle-line!!!
        // But really for trimmed curves extremas can not exist!
        if (type1 != type2)
        {
            // The projection of the circle's location to the trimmed line must exist.
            bool isReversed = (type1 != Circle);
            Point3D ponC = !isReversed ? CurveTool.Value(_c[0]!, ut11)
                                       : CurveTool.Value(_c[1]!, ut21);

            Line3D l = _c[!isReversed ? 1 : 0]!.Line();
            ExtPElC extPLin = new(ponC,
                                          l,
                                          global::Kerf.Engine.Foundation.Precision.Confusion(),
                                          !isReversed ? ut21 : ut11,
                                          !isReversed ? ut22 : ut12);

            if (extPLin.IsDone)
            {
                _sqDist.Append(sqDist);
            }
            else
            {
                _isParallel = false;
            }

            return;
        }

        if (type1 == Line)
        {
            // Line - Line

            // OCCT declares these two flags as double
            bool isFirstInfinite = (global::Kerf.Engine.Foundation.Precision.IsInfinite(ut11) && global::Kerf.Engine.Foundation.Precision.IsInfinite(ut12));
            bool isLastInfinite = (global::Kerf.Engine.Foundation.Precision.IsInfinite(ut21) && global::Kerf.Engine.Foundation.Precision.IsInfinite(ut22));

            if (isFirstInfinite || isLastInfinite)
            {
                // Infinite number of solution

                _sqDist.Append(sqDist);
            }
            else
            {
                // The range created by projection of both ends of the 1st line
                // to the 2nd one must intersect the (native) trimmed range of
                // the 2nd line.

                _isParallel = false;

                Line3D lin1 = _c[0]!.Line();
                Line3D lin2 = _c[1]!.Line();
                bool isOpposite = (lin1.Direction.Dot(lin2.Direction) < 0.0);

                BoundRange range2 = new(ut21, ut22);
                BoundRange projRng12 = new();

                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(ut11))
                {
                    if (isOpposite)
                    {
                        projRng12.Add(global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                    else
                    {
                        projRng12.Add(-global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                }
                else
                {
                    Point3D ponC1 = global::Kerf.Engine.Geometry.ElCLib.Value(ut11, lin1);
                    double par = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2, ponC1);
                    projRng12.Add(par);
                }

                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(ut12))
                {
                    if (isOpposite)
                    {
                        projRng12.Add(-global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                    else
                    {
                        projRng12.Add(global::Kerf.Engine.Foundation.Precision.Infinite());
                    }
                }
                else
                {
                    Point3D ponC1 = global::Kerf.Engine.Geometry.ElCLib.Value(ut12, lin1);
                    double par = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2, ponC1);
                    projRng12.Add(par);
                }

                range2.Common(projRng12);
                if (range2.Delta() > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    ClearSolutions();
                    _sqDist.Append(sqDist);
                    _isParallel = true;
                }
                else if (!range2.IsVoid)
                {
                    // Case like this:

                    //  **************     aLin1
                    //               o
                    //               o
                    //               ***************  aLin2

                    ClearSolutions();
                    double par1 = 0.0, par2 = 0.0;
                    range2.GetBounds(ref par1, ref par2);
                    par2 = 0.5 * (par1 + par2);
                    Point3D p = global::Kerf.Engine.Geometry.ElCLib.Value(par2, lin2);
                    POnCurv p2 = new(par2, p);
                    par1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin1, p);
                    p = global::Kerf.Engine.Geometry.ElCLib.Value(par1, lin1);
                    POnCurv p1 = new(par1, p);
                    _mypoints.Append(p1);
                    _mypoints.Append(p2);
                    _sqDist.Append(sqDist);
                }
                else
                {
                    // Case like this:

                    //  **************     aLin1
                    //                 o
                    //                  o
                    //                   ***********  aLin2
                    //
                    // Take minimal trimmed distance
                    double dmin;
                    ReadOnlySpan<double> dists = [_mydist11, _mydist12, _mydist21, _mydist22];
                    POnCurv p1 = new(), p2 = new();
                    dmin = dists[0];
                    int i, imin = 0;
                    for (i = 1; i < 4; ++i)
                    {
                        if (dmin > dists[i])
                        {
                            dmin = dists[i];
                            imin = i;
                        }
                    }
                    if (imin == 0)
                    {
                        p1.SetValues(_inf[0], _p1f);
                        p2.SetValues(_inf[1], _p2f);
                    }
                    else if (imin == 1)
                    {
                        p1.SetValues(_inf[0], _p1f);
                        p2.SetValues(_sup[1], _p2l);
                    }
                    else if (imin == 2)
                    {
                        p1.SetValues(_sup[0], _p1l);
                        p2.SetValues(_inf[1], _p2f);
                    }
                    else
                    {
                        p1.SetValues(_sup[0], _p1l);
                        p2.SetValues(_sup[1], _p2l);
                    }
                    ClearSolutions();
                    _mypoints.Append(p1);
                    _mypoints.Append(p2);
                    _sqDist.Append(dmin);
                }
            }
        }
        else
        {
            // Circle - Circle
            _isParallel = false;

            // Two arcs with ranges [U1, U2] and [V1, V2] correspondingly are
            // considered to be parallel in the following case:
            //   The range created by projection both points U1 and U2 of the
            //   1st circle to the 2nd one intersects either the range [V1, V2] or
            //   the range [V1-PI, V2-PI]. All ranges must be adjusted to correspond
            //   periodic range before checking of intersection.

            Circle3D workCirc = _c[1]!.Circle();
            double period = M_PI + M_PI;
            Vector3D vTg1;
            Point3D p11;
            Point3D p12 = CurveTool.Value(_c[0]!, ut12);
            CurveTool.D1(_c[0]!, ut11, out p11, out vTg1);

            BoundRange range = new(ut21, ut22);
            BoundRange projRng1 = new();

            // Precision of the calculation depends on circles radii
            double precision = Max(Epsilon(_c[0]!.Circle().Radius), Epsilon(_c[1]!.Circle().Radius));

            // Project arc of the 1st circle between points theUt11 and theUt12 to the
            // 2nd circle. It is necessary to chose correct arc from two possible ones.

            double par1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(global::Kerf.Engine.Geometry.ElCLib.Parameter(workCirc, p11), ut21, ut21 + period);
            Vector3D vTg2 = CurveTool.DN(_c[1]!, par1, 1);

            // Check if circles have same/opposite directions
            bool isOpposite = (vTg1.Dot(vTg2) < 0.0);

            double par2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(global::Kerf.Engine.Geometry.ElCLib.Parameter(workCirc, p12), ut21, ut21 + period);

            if (isOpposite)
            {
                // Must be aPar2 < aPar1
                if ((range.Delta() > global::Kerf.Engine.Foundation.Precision.Angular()) && ((par1 - par2) < global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    par2 -= period;
                }
            }
            else
            {
                // Must be aPar2 > aPar1
                if ((range.Delta() > global::Kerf.Engine.Foundation.Precision.Angular()) && ((par2 - par1) < global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    par2 += period;
                }
            }

            // Now the projection result is the range [aPar1, aPar2]
            // if aPar1 < aPar2 or the range [aPar2, aPar1], otherwise.

            double minSquareDist = RealLast();

            projRng1.Add(par1 - period);
            projRng1.Add(par2 - period);
            for (int i = 0; i < 3; i++)
            {
                // Repeat computation three times, shifting the range to PI on each step,
                // to be able to find if the concentric arcs ranges are intersected in just one parameter
                // (lower or upper boundary).

                BoundRange rng = projRng1;
                rng.Common(range);

                // Cases are possible and processed below:
                // 1. Extrema does not exist. In this case all common ranges are VOID.
                // 2. Arcs are parallel and distance between them is equal to sqrt(theSqDist).
                //     In this case myIsParallel = TRUE definitely.
                // 3. Arcs are parallel and distance between them is equal to (sqrt(theSqDist) + R),
                //     where R is the least radius of the both circles. In this case myIsParallel flag
                //     will temporary be set to TRUE but check will be continued until less
                //     distance will be found. At that, region with the least distance can be
                //     either a local point or continuous range. In 1st case myIsParallel = FALSE and
                //     several (or single) extremas will be returned. In the 2nd one
                //     myIsParallel = TRUE and only the least distance will be returned.
                // 4. Arcs are not parallel. Then several (or single) extremas will be returned.

                if (rng.Delta() > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    double par = 0.0;
                    rng.GetIntermediatePoint(0.5, ref par);
                    Point3D pCirc2 = global::Kerf.Engine.Geometry.ElCLib.Value(par, workCirc);
                    ExtPElC extPCir = new(pCirc2,
                                                  CurveTool.Circle(_c[0]!),
                                                  global::Kerf.Engine.Foundation.Precision.Confusion(),
                                                  ut11,
                                                  ut12);
                    if (extPCir.ExtCount < 1)
                    {
                        // OCCT: this skips the shift of the projected range at the end of the loop, so the next
                        // two iterations test the same range again
                        continue;
                    }
                    double minSqD = extPCir.SquareDistance(1);
                    for (int extID = 2; extID <= extPCir.ExtCount; extID++)
                    {
                        minSqD = Min(minSqD, extPCir.SquareDistance(extID));
                    }

                    if (minSqD <= minSquareDist + (1.0 + minSqD) * precision)
                    {
                        ClearSolutions();
                        _sqDist.Append(minSqD);
                        _isParallel = true;

                        double deltaSqDist = minSqD - sqDist;
                        double sqD = Max(minSqD, sqDist);

                        //  0 <= Dist1-Dist2 <= Eps
                        //  0 <= Dist1^2 - Dist2^2 < Eps*(Dist1+Dist2)

                        // If Dist1 ~= Dist2 ==> Dist1+Dist2 ~= 2*Dist2.
                        // Consequently,
                        //   0 <= Dist1^2 - Dist2^2 <= 2*Dist2*Eps

                        // Or
                        //   (Dist1^2 - Dist2^2)^2 <= 4*Dist2^2*Eps^2

                        if (deltaSqDist * deltaSqDist < 4.0 * sqD * global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                        {
                            // New solution is found
                            break;
                        }
                    }
                    // Nearer solution can be found
                }
                else if (!rng.IsVoid)
                {
                    // Check cases like this:

                    //  **************     aCirc1
                    //               o
                    //               o
                    //               ***************  aCirc2

                    double par = 0.0;
                    rng.GetIntermediatePoint(0.5, ref par);
                    Point3D pCirc2 = global::Kerf.Engine.Geometry.ElCLib.Value(par, workCirc);
                    POnCurv p2 = new(par, pCirc2);

                    ExtPElC extPCir = new(pCirc2,
                                                  CurveTool.Circle(_c[0]!),
                                                  global::Kerf.Engine.Foundation.Precision.Confusion(),
                                                  ut11,
                                                  ut12);

                    bool isFound = !_isParallel;

                    if (!isFound)
                    {
                        // If the flag myIsParallel was set earlier then it does not mean that
                        // we have found the minimal distance. Here we check it. If there is
                        // a pair of points, which are in less distance then myIsParallel flag
                        // was unset and the algorithm will return these nearest points.

                        for (int extID = 1; extID <= extPCir.ExtCount; extID++)
                        {
                            if (extPCir.SquareDistance(extID) < minSquareDist)
                            {
                                isFound = true;
                                break;
                            }
                        }
                    }

                    if (isFound)
                    {
                        ClearSolutions();
                        _isParallel = false;
                        for (int extID = 1; extID <= extPCir.ExtCount; extID++)
                        {
                            _mypoints.Append(extPCir.Point(extID));
                            _mypoints.Append(p2);
                            _sqDist.Append(extPCir.SquareDistance(extID));
                            minSquareDist = Min(minSquareDist, extPCir.SquareDistance(extID));
                        }
                    }
                }
                else
                {
                    // Case like this:

                    //  **************     Cir1
                    //                 o
                    //                  o
                    //                   ***********  Cir2
                    //
                    // Take minimal trimmed distance
                    _isParallel = false;
                    double dmin;
                    ReadOnlySpan<double> dists = [_mydist11, _mydist12, _mydist21, _mydist22];
                    POnCurv p1 = new(), p2 = new();
                    dmin = dists[0];
                    int k, imin = 0;
                    for (k = 1; k < 4; ++k)
                    {
                        if (dmin > dists[k])
                        {
                            dmin = dists[k];
                            imin = k;
                        }
                    }
                    if (dmin <= minSquareDist + (1.0 + dmin) * precision)
                    {
                        if (imin == 0)
                        {
                            p1.SetValues(_inf[0], _p1f);
                            p2.SetValues(_inf[1], _p2f);
                        }
                        else if (imin == 1)
                        {
                            p1.SetValues(_inf[0], _p1f);
                            p2.SetValues(_sup[1], _p2l);
                        }
                        else if (imin == 2)
                        {
                            p1.SetValues(_sup[0], _p1l);
                            p2.SetValues(_inf[1], _p2f);
                        }
                        else
                        {
                            p1.SetValues(_sup[0], _p1l);
                            p2.SetValues(_sup[1], _p2l);
                        }
                        ClearSolutions();
                        _mypoints.Append(p1);
                        _mypoints.Append(p2);
                        _sqDist.Append(dmin);
                        minSquareDist = Min(minSquareDist, dmin);
                    }
                }
                projRng1.Shift(M_PI);
            }
        }
    }

    protected void PrepareResults(ExtElC algExt, bool isInverse, double ut11, double ut12, double ut21,
        double ut22)
    {
        int i, nbExt;
        double val, u, u2;
        POnCurv p1, p2;

        _done = algExt.IsDone;
        if (_done)
        {
            _isParallel = algExt.IsParallel;
            if (_isParallel)
            {
                PrepareParallelResult(ut11, ut12, ut21, ut22, algExt.SquareDistance());
            }
            else
            {
                nbExt = algExt.ExtCount;
                for (i = 1; i <= nbExt; i++)
                {
                    // Verification of the validity of parameters
                    algExt.Points(i, out p1, out p2);
                    if (!isInverse)
                    {
                        u = p1.Parameter();
                        u2 = p2.Parameter();
                    }
                    else
                    {
                        u2 = p1.Parameter();
                        u = p2.Parameter();
                    }

                    if (CurveTool.IsPeriodic(_c[0]!))
                    {
                        u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, ut11, ut11 + CurveTool.Period(_c[0]!));
                    }
                    if (CurveTool.IsPeriodic(_c[1]!))
                    {
                        u2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u2, ut21, ut21 + CurveTool.Period(_c[1]!));
                    }

                    if ((u >= ut11 - RealEpsilon()) && (u <= ut12 + RealEpsilon())
                        && (u2 >= ut21 - RealEpsilon()) && (u2 <= ut22 + RealEpsilon()))
                    {
                        val = algExt.SquareDistance(i);
                        _sqDist.Append(val);
                        if (!isInverse)
                        {
                            p1.SetValues(u, p1.Value());
                            p2.SetValues(u2, p2.Value());
                            _mypoints.Append(p1);
                            _mypoints.Append(p2);
                        }
                        else
                        {
                            p1.SetValues(u2, p1.Value());
                            p2.SetValues(u, p2.Value());
                            _mypoints.Append(p2);
                            _mypoints.Append(p1);
                        }
                    }
                }
            }
        }
    }

    protected void PrepareResults(Ecc algExt, double ut11, double ut12, double ut21, double ut22)
    {
        int i, nbExt;
        double val, u, u2;
        POnCurv p1, p2;

        _done = algExt.IsDone;
        if (_done)
        {
            _isParallel = algExt.IsParallel;
            if (_isParallel)
            {
                PrepareParallelResult(ut11, ut12, ut21, ut22, algExt.SquareDistance());
            }
            else
            {
                nbExt = algExt.ExtCount;
                for (i = 1; i <= nbExt; i++)
                {
                    algExt.Points(i, out p1, out p2);
                    u = p1.Parameter();
                    u2 = p2.Parameter();

                    // Check points to be into param space.
                    if (CurveTool.IsPeriodic(_c[0]!))
                    {
                        u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, ut11, ut11 + CurveTool.Period(_c[0]!));
                    }
                    if (CurveTool.IsPeriodic(_c[1]!))
                    {
                        u2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u2, ut21, ut21 + CurveTool.Period(_c[1]!));
                    }

                    if ((u >= ut11 - RealEpsilon()) && (u <= ut12 + RealEpsilon())
                        && (u2 >= ut21 - RealEpsilon()) && (u2 <= ut22 + RealEpsilon()))
                    {
                        val = algExt.SquareDistance(i);
                        _sqDist.Append(val);
                        p1.SetValues(u, p1.Value());
                        p2.SetValues(u2, p2.Value());
                        _mypoints.Append(p1);
                        _mypoints.Append(p2);
                    }
                }
            }
        }
    }
}
