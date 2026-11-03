// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_EdgeEdge.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_EdgeEdge.lxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_EdgeEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>
/// The edge/edge intersection algorithm, based on the intersection of the edges' bounding boxes: it finds the common
/// parts (vertices and edges) of two edges on their ranges.
/// </summary>
/// <remarks>
/// A working object (plan section 6), derived from by the pave filler's <c>BOPAlgo_EdgeEdge</c> (unit 6E), which
/// reads and restores its protected members. The common parts are the algorithm's own
/// <see cref="CommonPrt"/> objects (a class, decision 620): callers copy one with
/// <c>new IntTools_CommonPrt(x)</c> to keep it.
/// </remarks>
public class EdgeEdge
{
    protected Edge _edge1 = new();
    protected Edge _edge2 = new();
    protected Curve3D? _geom1;
    protected Curve3D? _geom2;
    protected EdgeCurveAdaptor _curve1 = new();
    protected EdgeCurveAdaptor _curve2 = new();
    protected double _tol1;
    protected double _tol2;
    protected double _tol;
    protected double _fuzzyValue;
    protected double _res1;
    protected double _res2;
    protected double _resCoeff1;
    protected double _resCoeff2;
    protected double _pTol1;
    protected double _pTol2;
    protected ParameterRange _range1;
    protected ParameterRange _range2;
    protected bool _swap;
    protected int _errorStatus;
    protected readonly NCollection_Sequence<CommonPrt> _commonParts = new();

    /// <summary>
    /// Allows avoiding the use of the edge/edge intersection algorithm (speeding up the Boolean algorithm) if the edges
    /// really coincide. If it is not evident, the flag should not be set (the Boolean algorithm would be slower).
    /// </summary>
    protected bool _quickCoincidenceCheck;

    /// <summary>An empty algorithm.</summary>
    public EdgeEdge()
    {
        _tol1 = 0.0;
        _tol2 = 0.0;
        _tol = 0.0;
        _fuzzyValue = global::Kerf.Engine.Foundation.Precision.Confusion();
        _res1 = 0.0;
        _res2 = 0.0;
        _resCoeff1 = 0.0;
        _resCoeff2 = 0.0;
        _pTol1 = 0.0;
        _pTol2 = 0.0;
        _range1 = new ParameterRange(0.0, 0.0);
        _range2 = new ParameterRange(0.0, 0.0);
        _swap = false;
        _errorStatus = 0;
        _quickCoincidenceCheck = false;
    }

    /// <summary>The intersection of the edges on their whole ranges.</summary>
    public EdgeEdge(in Edge edge1, in Edge edge2)
        : this()
    {
        _edge1 = edge1;
        _edge2 = edge2;
    }

    /// <summary>The intersection of the edges on the given ranges.</summary>
    public EdgeEdge(in Edge edge1, double aT11, double aT12, in Edge edge2, double aT21,
                             double aT22)
        : this()
    {
        _edge1 = edge1;
        _edge2 = edge2;
        _range1 = new ParameterRange(aT11, aT12);
        _range2 = new ParameterRange(aT21, aT22);
    }

    /// <summary>Sets the first edge.</summary>
    public void SetEdge1(in Edge edge) => _edge1 = edge;

    /// <summary>Sets the first edge and its range.</summary>
    public void SetEdge1(in Edge edge, double aT1, double aT2)
    {
        SetEdge1(edge);
        SetRange1(aT1, aT2);
    }

    /// <summary>Sets the range of the first edge.</summary>
    public void SetRange1(in ParameterRange range) => _range1 = range;

    /// <summary>Sets the range of the first edge.</summary>
    public void SetRange1(double aT1, double aT2)
    {
        _range1.SetFirst(aT1);
        _range1.SetLast(aT2);
    }

    /// <summary>Sets the second edge.</summary>
    public void SetEdge2(in Edge edge) => _edge2 = edge;

    /// <summary>Sets the second edge and its range.</summary>
    public void SetEdge2(in Edge edge, double aT1, double aT2)
    {
        SetEdge2(edge);
        SetRange2(aT1, aT2);
    }

    /// <summary>Sets the range of the second edge.</summary>
    public void SetRange2(in ParameterRange range) => _range2 = range;

    /// <summary>Sets the range of the second edge.</summary>
    public void SetRange2(double aT1, double aT2)
    {
        _range2.SetFirst(aT1);
        _range2.SetLast(aT2);
    }

    /// <summary>Sets the fuzzy value (at least <see cref="Precision.Confusion"/>).</summary>
    public void SetFuzzyValue(double fuzz) => _fuzzyValue = Max(fuzz, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <summary>True if the computation succeeded (common parts may still be none).</summary>
    public bool IsDone => _errorStatus == 0;

    /// <summary>The fuzzy value.</summary>
    public double FuzzyValue() => _fuzzyValue;

    /// <summary>The common parts (the algorithm's own sequence; ported code only reads it).</summary>
    public NCollection_Sequence<CommonPrt> CommonParts() => _commonParts;

    /// <summary>Sets the flag of the quick coincidence check.</summary>
    public void UseQuickCoincidenceCheck(bool bFlag) => _quickCoincidenceCheck = bFlag;

    /// <summary>The flag of the quick coincidence check.</summary>
    public bool IsCoincidenceCheckedQuickly => _quickCoincidenceCheck;

    /// <summary>Checks the data.</summary>
    protected void CheckData()
    {
        if (_edge1.IsNull || _edge2.IsNull)
        {
            _errorStatus = 1;
            return;
        }
        //
        if (Tool.Degenerated(_edge1) || Tool.Degenerated(_edge2))
        {
            _errorStatus = 2;
            return;
        }
        //
        if (!Tool.IsGeometric(_edge1) || !Tool.IsGeometric(_edge2))
        {
            _errorStatus = 3;
            return;
        }
    }

    /// <summary>Prepares the data.</summary>
    protected void Prepare()
    {
        CurveKind ct1, ct2;
        int iCT1, iCT2;
        //
        _curve1.Initialize(_edge1);
        _curve2.Initialize(_edge2);
        //
        if (_range1.First() == 0.0 && _range1.Last() == 0.0)
        {
            _range1.SetFirst(_curve1.FirstParameter());
            _range1.SetLast(_curve1.LastParameter());
        }
        //
        if (_range2.First() == 0.0 && _range2.Last() == 0.0)
        {
            _range2.SetFirst(_curve2.FirstParameter());
            _range2.SetLast(_curve2.LastParameter());
        }
        //
        ct1 = _curve1.GetType();
        ct2 = _curve2.GetType();
        //
        iCT1 = TypeToInteger(ct1);
        iCT2 = TypeToInteger(ct2);
        //
        if (iCT1 == iCT2)
        {
            if (iCT1 != 0)
            {
                // compute deflection
                double c1, c2;
                //
                c2 = CurveDeflection(_curve2, _range2);
                c1 = (c2 > global::Kerf.Engine.Foundation.Precision.Confusion()) ? CurveDeflection(_curve1, _range1) : 1.0;
                //
                if (c1 < c2)
                {
                    --iCT1;
                }
            }
        }
        //
        if (iCT1 < iCT2)
        {
            Edge tmpE = _edge1;
            _edge1 = _edge2;
            _edge2 = tmpE;
            //
            EdgeCurveAdaptor tmpC = _curve1;
            _curve1 = _curve2;
            _curve2 = tmpC;
            //
            ParameterRange tmpR = _range1;
            _range1 = _range2;
            _range2 = tmpR;
            //
            _swap = true;
        }
        //
        double tolAdd = _fuzzyValue / 2.0;
        _tol1 = _curve1.Tolerance() + tolAdd;
        _tol2 = _curve2.Tolerance() + tolAdd;
        _tol = _tol1 + _tol2;
        //
        if (iCT1 != 0 || iCT2 != 0)
        {
            double tm;
            //
            _geom1 = Tool.Curve(_edge1, out _, out _);
            _geom2 = Tool.Curve(_edge2, out _, out _);
            //
            _resCoeff1 = ResolutionCoeff(_curve1, _range1);
            _resCoeff2 = ResolutionCoeff(_curve2, _range2);
            //
            _res1 = Resolution(_curve1.Curve().Curve()!, _curve1.GetType(), _resCoeff1, _tol1);
            _res2 = Resolution(_curve2.Curve().Curve()!, _curve2.GetType(), _resCoeff2, _tol2);
            //
            _pTol1 = 5.0e-13;
            tm = Max(Math.Abs(_range1.First()), Math.Abs(_range1.Last()));
            if (tm > 999.0)
            {
                _pTol1 = 5.0e-16 * tm;
            }
            //
            _pTol2 = 5.0e-13;
            tm = Max(Math.Abs(_range2.First()), Math.Abs(_range2.Last()));
            if (tm > 999.0)
            {
                _pTol2 = 5.0e-16 * tm;
            }
        }
    }

    /// <summary>Performs the intersection of the edges.</summary>
    public void Perform()
    {
        // 1. Check data
        CheckData();
        if (_errorStatus != 0)
        {
            return;
        }
        //
        // 2. Prepare Data
        Prepare();
        //
        // 3.1. Check Line/Line case
        if (_curve1.GetType() == Line && _curve2.GetType() == Line)
        {
            ComputeLineLine();
            return;
        }
        //
        if (_quickCoincidenceCheck)
        {
            if (IsCoincident())
            {
                double t11, t12, t21, t22;
                //
                _range1.Range(out t11, out t12);
                _range2.Range(out t21, out t22);
                AddSolution(t11, t12, t21, t22, ShapeType.Edge);
                return;
            }
        }
        //
        if ((_curve1.GetType() <= Parabola && _curve2.GetType() <= Parabola)
            && (_curve1.GetType() == Line || _curve2.GetType() == Line))
        {
            // Improvement of performance for cases of searching common parts between line
            // and analytical curve. This code allows to define that edges have no
            // common parts more fast, then regular algorithm (FindSolution(...))
            // Check minimal distance between edges
            DistanceShapeShape minDist = new(_edge1, _edge2, ExtFlag_MIN);
            if (minDist.IsDone)
            {
                double d = minDist.Value();
                if (d > 1.1 * _tol)
                {
                    return;
                }
            }
        }

        NCollection_Sequence<ParameterRange> ranges1 = new(), ranges2 = new();
        //
        // 3.2. Find ranges containing solutions
        bool bSplit2;
        FindSolutions(ranges1, ranges2, out bSplit2);
        //
        // 4. Merge solutions and save common parts
        MergeSolutions(ranges1, ranges2, bSplit2);
    }

    /// <summary>Checks if the edges really coincide.</summary>
    protected bool IsCoincident()
    {
        int i, iCnt, nbSeg, nbP2;
        double dT, t1, coeff, tresh, d;
        double t11, t12, t21, t22;
        ProjectPointOnCurve projPC = new();
        Point3D p1;
        //
        tresh = 0.5;
        nbSeg = 23;
        _range1.Range(out t11, out t12);
        _range2.Range(out t21, out t22);
        //
        projPC.Init(_geom2!, t21, t22);
        //
        dT = (t12 - t11) / nbSeg;
        //
        iCnt = 0;
        for (i = 0; i <= nbSeg; ++i)
        {
            t1 = t11 + i * dT;
            _geom1!.D0(t1, out p1);
            //
            projPC.Perform(p1);
            nbP2 = projPC.NbPoints();
            if (nbP2 == 0)
            {
                continue;
            }
            //
            d = projPC.LowerDistance();
            if (d < _tol)
            {
                ++iCnt;
            }
        }
        //
        coeff = (double)iCnt / ((double)nbSeg + 1);
        return coeff > tresh;
    }

    /// <summary>Intermediate function: finds the ranges containing solutions.</summary>
    protected void FindSolutions(NCollection_Sequence<ParameterRange> ranges1,
                                 NCollection_Sequence<ParameterRange> ranges2,
                                 out bool bSplit2)
    {
        bool bIsClosed2;
        double t11, t12, t21, t22;
        Box b1 = new(), b2 = new();
        //
        bSplit2 = false;
        _range1.Range(out t11, out t12);
        _range2.Range(out t21, out t22);
        //
        bIsClosed2 = IsClosed(_geom2!, t21, t22, _tol2, _res2);
        //
        if (bIsClosed2)
        {
            BndBuildBox(_curve1, t11, t12, _tol1, out b1);
            //
            Point3D p = _geom2!.Value(t21);
            bIsClosed2 = !b1.IsOut(p);
        }
        //
        if (!bIsClosed2)
        {
            BndBuildBox(_curve1, t11, t12, _tol1, out b1);
            BndBuildBox(_curve2, t21, t22, _tol2, out b2);
            FindSolutions(_range1, b1, _range2, b2, ranges1, ranges2);
            return;
        }
        //
        if (CheckCoincidence(t11, t12, t21, t22, _tol, _res1) == 0)
        {
            ranges1.Append(_range1);
            ranges2.Append(_range2);
            return;
        }
        //
        int i, j, nb1, nb2;
        NCollection_Sequence<ParameterRange> segments1 = new(), segments2 = new();
        //
        nb1 = IsClosed(_geom1!, t11, t12, _tol1, _res1) ? 2 : 1;
        nb2 = 2;
        //
        nb1 = SplitRangeOnSegments(t11, t12, _res1, nb1, segments1);
        nb2 = SplitRangeOnSegments(t21, t22, _res2, nb2, segments2);
        //
        for (i = 1; i <= nb1; ++i)
        {
            ParameterRange r1 = segments1.Value(i);
            BndBuildBox(_curve1, r1.First(), r1.Last(), _tol1, out b1);
            for (j = 1; j <= nb2; ++j)
            {
                ParameterRange r2 = segments2.Value(j);
                BndBuildBox(_curve2, r2.First(), r2.Last(), _tol2, out b2);
                FindSolutions(r1, b1, r2, b2, ranges1, ranges2);
            }
        }
        //
        bSplit2 = nb2 > 1;
    }

    /// <summary>Looks for the exact intersection ranges.</summary>
    protected void FindSolutions(in ParameterRange r1,
                                 in Box box1,
                                 in ParameterRange r2,
                                 in Box box2,
                                 NCollection_Sequence<ParameterRange> ranges1,
                                 NCollection_Sequence<ParameterRange> ranges2)
    {
        bool bOut, bStop, bThin;
        double t11, t12, t21, t22;
        double tb11, tb12, tb21, tb22;
        double smallStep1, smallStep2;
        int iCom;
        Box b1, b2;
        //
        r1.Range(out t11, out t12);
        r2.Range(out t21, out t22);
        //
        b1 = box1;
        b2 = box2;
        //
        bThin = false;
        bStop = false;
        iCom = 1;
        //
        do
        {
            tb11 = t11;
            tb12 = t12;
            tb21 = t21;
            tb22 = t22;
            //
            // 1. Find parameters of the second edge in the box of first one
            bOut = b1.IsOut(b2);
            if (bOut)
            {
                break;
            }
            //
            bThin = ((t12 - t11) < _res1) || (b1.IsXThin(_tol) && b1.IsYThin(_tol) && b1.IsZThin(_tol));
            //
            bOut = !FindParameters(_curve2, tb21, tb22, _tol2, _res2, _pTol2, _resCoeff2, b1, ref t21, ref t22);
            if (bOut || bThin)
            {
                break;
            }
            //
            // 2. Build box for second edge and find parameters
            //   of the first one in that box
            BndBuildBox(_curve2, t21, t22, _tol2, out b2);
            bOut = b1.IsOut(b2);
            if (bOut)
            {
                break;
            }
            //
            bThin = ((t22 - t21) < _res2) || (b2.IsXThin(_tol) && b2.IsYThin(_tol) && b2.IsZThin(_tol));
            //
            bOut = !FindParameters(_curve1, tb11, tb12, _tol1, _res1, _pTol1, _resCoeff1, b2, ref t11, ref t12);
            //
            if (bOut || bThin)
            {
                break;
            }
            //
            // 3. Check if it makes sense to continue
            smallStep1 = (tb12 - tb11) / 250.0;
            smallStep2 = (tb22 - tb21) / 250.0;
            //
            if (smallStep1 < _res1)
            {
                smallStep1 = _res1;
            }
            if (smallStep2 < _res2)
            {
                smallStep2 = _res2;
            }
            //
            if (((t11 - tb11) < smallStep1) && ((tb12 - t12) < smallStep1)
                && ((t21 - tb21) < smallStep2) && ((tb22 - t22) < smallStep2))
            {
                bStop = true;
            }
            else
            {
                BndBuildBox(_curve1, t11, t12, _tol1, out b1);
            }
        } while (!bStop);
        //
        if (bOut)
        {
            // no intersection;
            return;
        }
        //
        if (!bThin)
        {
            // check curves for coincidence on the ranges
            iCom = CheckCoincidence(t11, t12, t21, t22, _tol, _res1);
            if (iCom == 0)
            {
                bThin = true;
            }
        }
        //
        if (bThin)
        {
            if (iCom != 0)
            {
                // check intermediate points
                bool bSol;
                double t1;
                Point3D p1;
                ProjectPointOnCurve projPC = new();
                //
                t1 = (t11 + t12) * 0.5;
                _geom1!.D0(t1, out p1);
                //
                projPC.Init(_geom2!, t21, t22);
                projPC.Perform(p1);
                //
                if (projPC.NbPoints() != 0)
                {
                    bSol = projPC.LowerDistance() <= _tol;
                }
                else
                {
                    double t2;
                    Point3D p2;
                    //
                    t2 = (t21 + t22) * 0.5;
                    _geom2!.D0(t2, out p2);
                    //
                    bSol = p1.IsEqual(p2, _tol);
                }
                //
                if (!bSol)
                {
                    return;
                }
            }
            // add common part
            ParameterRange aR1 = new(t11, t12), aR2 = new(t21, t22);
            //
            ranges1.Append(aR1);
            ranges2.Append(aR2);
            return;
        }
        //
        if (!IsIntersection(t11, t12, t21, t22))
        {
            return;
        }
        //
        // split ranges on segments and repeat
        int i, nb1;
        NCollection_Sequence<ParameterRange> segments1 = new();
        //
        // Build box for first curve to compare
        // the boxes of the splits with this one
        BndBuildBox(_curve1, t11, t12, _tol1, out b1);
        double b1SqExtent = b1.SquareExtent();
        //
        ParameterRange r2r = new(t21, t22);
        BndBuildBox(_curve2, t21, t22, _tol2, out b2);
        //
        nb1 = SplitRangeOnSegments(t11, t12, _res1, 3, segments1);
        for (i = 1; i <= nb1; ++i)
        {
            ParameterRange aR1 = segments1.Value(i);
            BndBuildBox(_curve1, aR1.First(), aR1.Last(), _tol1, out b1);
            if (!b1.IsOut(b2) && (nb1 == 1 || b1.SquareExtent() < b1SqExtent))
            {
                FindSolutions(aR1, b1, r2r, b2, ranges1, ranges2);
            }
        }
    }

    /// <summary>Looks for the range of the edge which is in the box.</summary>
    protected static bool FindParameters(EdgeCurveAdaptor bac,
                                         double aT1,
                                         double aT2,
                                         double tol,
                                         double res,
                                         double pTol,
                                         double resCoeff,
                                         in Box cBox,
                                         ref double aTB1,
                                         ref double aTB2)
    {
        bool bRet;
        int c, i;
        double cf, diff, dt, t, tb, tOut, tIn;
        double dist, distP;
        Point3D p;
        Box cBx;
        //
        bRet = false;
        cf = 0.6180339887498948482045868343656; // =0.5*(1.+sqrt(5.))/2.;
        cBx = cBox;
        cBx.SetGap(cBx.GetGap() + tol);
        //
        Curve3D curve = bac.Curve().Curve()!;
        CurveKind curveType = bac.GetType();
        double maxDt = (aT2 - aT1) * 0.01;
        //
        for (i = 0; i < 2; ++i)
        {
            tb = i == 0 ? aT1 : aT2;
            t = i == 0 ? aT2 : aTB1;
            c = i == 0 ? 1 : -1;
            dt = res;
            distP = 0.0;
            bRet = false;
            double k = 1;
            // looking for the point on the edge which is in the box;
            while (c * (t - tb) >= 0)
            {
                bac.D0(tb, out p);
                dist = PointBoxDistance(cBox, p);
                if (dist > tol)
                {
                    if (distP > 0.0)
                    {
                        bool toGrow = false;
                        if (Math.Abs(distP - dist) / distP < 0.1)
                        {
                            dt = Resolution(curve, curveType, resCoeff, k * dist);
                            if (dt < maxDt)
                            {
                                toGrow = true;
                                k *= 2;
                            }
                        }
                        if (!toGrow)
                        {
                            k = 1;
                            dt = Resolution(curve, curveType, resCoeff, dist);
                        }
                    }
                    tb += c * dt;
                }
                else
                {
                    bRet = true;
                    break;
                }
                distP = dist;
            }
            //
            if (!bRet)
            {
                if (i == 0)
                {
                    // edge is out of the box;
                    return bRet;
                }
                else
                {
                    bRet = !bRet;
                    tb = aTB1;
                    dt = aT2 - aTB1;
                }
            }
            //
            t = i == 0 ? aT1 : aT2;
            if (tb != t)
            {
                // one point IN, one point OUT; looking for the bounding point;
                tIn = tb;
                tOut = tb - c * dt;
                diff = tIn - tOut;
                while (Math.Abs(diff) > pTol)
                {
                    tb = tOut + diff * cf;
                    bac.D0(tb, out p);
                    if (cBx.IsOut(p))
                    {
                        tOut = tb;
                    }
                    else
                    {
                        tIn = tb;
                    }
                    diff = tIn - tOut;
                }
            }
            if (i == 0)
            {
                aTB1 = tb;
            }
            else
            {
                aTB2 = tb;
            }
        }
        return bRet;
    }

    /// <summary>Merges the found solutions into common parts.</summary>
    protected void MergeSolutions(NCollection_Sequence<ParameterRange> ranges1,
                                  NCollection_Sequence<ParameterRange> ranges2,
                                  bool bSplit2)
    {
        int nbCP = ranges1.Length();
        if (nbCP == 0)
        {
            return;
        }
        //
        ParameterRange ri1, ri2, rj1, rj2;
        bool bCond;
        int i, j;
        ShapeType type;
        double t11, t12, t21, t22;
        double ti11, ti12, ti21, ti22;
        double tj11, tj12, tj21, tj22;
        double res1, res2, dTR1, dTR2;
        NCollection_Map<int> mi = new();
        //
        res1 = Resolution(_curve1.Curve().Curve()!, _curve1.GetType(), _resCoeff1, _tol);
        res2 = Resolution(_curve2.Curve().Curve()!, _curve2.GetType(), _resCoeff2, _tol);
        //
        _range1.Range(out t11, out t12);
        _range2.Range(out t21, out t22);
        dTR1 = 20 * res1;
        dTR2 = 20 * res2;
        type = ShapeType.Vertex;
        //
        for (i = 1; i <= nbCP;)
        {
            if (mi.Contains(i))
            {
                ++i;
                continue;
            }
            //
            ri1 = ranges1.Value(i);
            ri2 = ranges2.Value(i);
            //
            ri1.Range(out ti11, out ti12);
            ri2.Range(out ti21, out ti22);
            //
            mi.Add(i);
            //
            for (j = i + 1; j <= nbCP; ++j)
            {
                if (mi.Contains(j))
                {
                    continue;
                }
                //
                rj1 = ranges1.Value(j);
                rj2 = ranges2.Value(j);
                //
                rj1.Range(out tj11, out tj12);
                rj2.Range(out tj21, out tj22);
                //
                bCond = (Math.Abs(ti12 - tj11) < dTR1) || (tj11 > ti11 && tj11 < ti12)
                        || (ti11 > tj11 && ti11 < tj12) || (bSplit2 && (Math.Abs(tj12 - ti11) < dTR1));
                if (bCond && bSplit2)
                {
                    bCond = (Math.Abs((Max(ti22, tj22) - Min(ti21, tj21)) - ((ti22 - ti21) + (tj22 - tj21)))
                             < dTR2)
                            || (tj21 > ti21 && tj21 < ti22) || (ti21 > tj21 && ti21 < tj22);
                }
                //
                if (bCond)
                {
                    ti11 = Min(ti11, tj11);
                    ti12 = Max(ti12, tj12);
                    ti21 = Min(ti21, tj21);
                    ti22 = Max(ti22, tj22);
                    mi.Add(j);
                }
                else if (!bSplit2)
                {
                    i = j;
                    break;
                }
            }
            //
            if (((Math.Abs(t11 - ti11) < _res1) && (Math.Abs(t12 - ti12) < _res1))
                || ((Math.Abs(t21 - ti21) < _res2) && (Math.Abs(t22 - ti22) < _res2)))
            {
                type = ShapeType.Edge;
                _commonParts.Clear();
            }
            //
            AddSolution(ti11, ti12, ti21, ti22, type);
            if (type == ShapeType.Edge)
            {
                break;
            }
            //
            if (bSplit2)
            {
                ++i;
            }
        }
    }

    /// <summary>Adds a common part of the given type.</summary>
    protected void AddSolution(double aT11, double aT12, double aT21, double aT22, ShapeType type)
    {
        CommonPrt cPart = new();
        //
        cPart.SetType(type);
        if (!_swap)
        {
            cPart.SetEdge1(_edge1);
            cPart.SetEdge2(_edge2);
            cPart.SetRange1(aT11, aT12);
            cPart.AppendRange2(aT21, aT22);
        }
        else
        {
            cPart.SetEdge1(_edge2);
            cPart.SetEdge2(_edge1);
            cPart.SetRange1(aT21, aT22);
            cPart.AppendRange2(aT11, aT12);
        }
        //
        if (type == ShapeType.Vertex)
        {
            double t1, t2;
            //
            FindBestSolution(aT11, aT12, aT21, aT22, out t1, out t2);
            //
            if (!_swap)
            {
                cPart.SetVertexParameter1(t1);
                cPart.SetVertexParameter2(t2);
            }
            else
            {
                cPart.SetVertexParameter1(t2);
                cPart.SetVertexParameter2(t1);
            }
        }
        _commonParts.Append(cPart);
    }

    /// <summary>Looks for the minimal distance between the edges on the ranges.</summary>
    protected void FindBestSolution(double aT11, double aT12, double aT21, double aT22, out double aT1, out double aT2)
    {
        int i, nbS, iErr;
        double dMin, d = 0.0, res1, solCriteria, touchCriteria;
        double t1A, t1B, t1Min = 0.0, t2Min = 0.0;
        ProjectPointOnCurve projPC = new();
        NCollection_Sequence<ParameterRange> ranges = new();
        //
        // Both are written on every path below; C++ leaves them unset until then.
        aT1 = 0.0;
        aT2 = 0.0;
        dMin = global::Kerf.Engine.Foundation.Precision.Infinite();
        solCriteria = 5.0e-16;
        touchCriteria = 5.0e-13;
        bool bTouch = false;
        bool bTouchConfirm = false;
        //
        res1 = Resolution(_curve1.Curve().Curve()!, _curve1.GetType(), _resCoeff1, _tol);
        nbS = 10;
        nbS = SplitRangeOnSegments(aT11, aT12, 3 * res1, nbS, ranges);
        //
        projPC.Init(_geom2!, aT21, aT22);
        //
        double t11Touch = aT11, t12Touch = aT12;
        double t21Touch = aT21, t22Touch = aT22;
        bool isSolFound = false;
        for (i = 1; i <= nbS; ++i)
        {
            ParameterRange r1 = ranges.Value(i);
            r1.Range(out t1A, out t1B);
            //
            d = _tol;
            iErr = FindDistPC(t1A, t1B, _geom1!, solCriteria, _pTol1, projPC, ref d, ref t1Min, ref t2Min, false);
            if (iErr != 1)
            {
                if (d < dMin)
                {
                    aT1 = t1Min;
                    aT2 = t2Min;
                    dMin = d;
                    isSolFound = true;
                }
                //
                if (d < touchCriteria)
                {
                    if (bTouch)
                    {
                        t12Touch = t1Min;
                        t22Touch = t2Min;
                        bTouchConfirm = true;
                    }
                    else
                    {
                        t11Touch = t1Min;
                        t21Touch = t2Min;
                        bTouch = true;
                    }
                }
            }
        }
        if (!isSolFound || bTouchConfirm)
        {
            aT1 = (t11Touch + t12Touch) * 0.5;
            iErr = DistPC(aT1, _geom1!, solCriteria, projPC, ref d, ref aT2, -1);
            if (iErr == 1)
            {
                aT2 = (t21Touch + t22Touch) * 0.5;
            }
        }
    }

    /// <summary>Computes the Line/Line intersection.</summary>
    protected void ComputeLineLine()
    {
        double tol = _tol * _tol;

        Line3D l1 = _curve1.Line();
        Line3D l2 = _curve2.Line();

        Direction3D d1 = l1.Direction();
        Direction3D d2 = l2.Direction();

        double angle = d1.Angle(d2);
        bool isCoincide = angle < global::Kerf.Engine.Foundation.Precision.Angular();
        if (isCoincide)
        {
            if (l1.SquareDistance(l2.Location()) > tol)
            {
                return;
            }
        }

        double t11, t12, aT21, aT22;
        _range1.Range(out t11, out t12);
        _range2.Range(out aT21, out aT22);

        Point3D p11 = global::Kerf.Engine.Geometry.ElCLib.Value(t11, l1);
        Point3D p12 = global::Kerf.Engine.Geometry.ElCLib.Value(t12, l1);

        if (!isCoincide)
        {
            Point3D o2 = l2.Location();
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(aT21) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(aT22))
            {
                o2 = global::Kerf.Engine.Geometry.ElCLib.Value((aT21 + aT22) / 2.0, l2);
            }

            Vector3D vec1 = new Vector3D(o2, p11).Crossed(new Vector3D(d2));
            Vector3D vec2 = new Vector3D(o2, p12).Crossed(new Vector3D(d2));

            double sqDist1 = vec1.SquareMagnitude();
            double sqDist2 = vec2.SquareMagnitude();

            isCoincide = (sqDist1 <= tol && sqDist2 <= tol);

            if (!isCoincide && vec1.Dot(vec2) > 0)
            {
                // the lines do not intersect
                return;
            }
        }

        CommonPrt commonPrt = new();
        commonPrt.SetEdge1(_edge1);
        commonPrt.SetEdge2(_edge2);

        if (isCoincide)
        {
            double t21 = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, p11);
            double t22 = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, p12);

            if ((t21 > aT22 && t22 > aT22) || (t21 < aT21 && t22 < aT21))
            {
                // projections are out of range
                return;
            }

            if (t21 > t22)
            {
                (t21, t22) = (t22, t21);
            }

            if (t21 >= aT21)
            {
                if (t22 <= aT22)
                {
                    commonPrt.SetRange1(t11, t12);
                    commonPrt.SetAllNullFlag(true);
                    commonPrt.AppendRange2(t21, t22);
                }
                else
                {
                    commonPrt.SetRange1(t11, t12 - (t22 - aT22));
                    commonPrt.AppendRange2(t21, aT22);
                }
            }
            else
            {
                commonPrt.SetRange1(t11 + (aT21 - t21), t12);
                commonPrt.AppendRange2(aT21, t22);
            }
            commonPrt.SetType(ShapeType.Edge);
            _commonParts.Append(commonPrt);
            return;
        }

        Vector3D o1O2 = new(l1.Location(), l2.Location());
        Xyz cross = d1.XYZ().Crossed(d2.XYZ());
        double distLL = o1O2.Dot(new Vector3D(cross.Normalized()));
        if (Math.Abs(distLL) > _tol)
        {
            return;
        }

        {
            // Fast check that no intersection needs to be added
            for (Iterator it1 = new(_edge1); it1.More(); it1.Next())
            {
                for (Iterator it2 = new(_edge2); it2.More(); it2.Next())
                {
                    if (it1.Value().IsSame(it2.Value()))
                    {
                        return;
                    }
                }
            }
        }

        double sqSin = cross.SquareModulus();
        double t2 = (d1.XYZ() * (o1O2.Dot(new Vector3D(d1))) - (o1O2.XYZ())).Dot(d2.XYZ());
        t2 /= sqSin;

        if (t2 < aT21 || t2 > aT22)
        {
            // out of range
            return;
        }

        Point3D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(t2, l2);
        double t1 = new Vector3D(l1.Location(), p2).Dot(new Vector3D(d1));

        if (t1 < t11 || t1 > t12)
        {
            // out of range
            return;
        }

        Point3D p1 = global::Kerf.Engine.Geometry.ElCLib.Value(t1, l1);
        double dist = p1.SquareDistance(p2);

        if (dist > tol)
        {
            // no intersection
            return;
        }

        // compute correct range on the edges
        double dt1 = IntersectionTools.ComputeIntRange(_tol1, _tol2, angle);
        double dt2 = IntersectionTools.ComputeIntRange(_tol2, _tol1, angle);

        commonPrt.SetRange1(t1 - dt1, t1 + dt1);
        commonPrt.AppendRange2(t2 - dt2, t2 + dt2);
        commonPrt.SetType(ShapeType.Vertex);
        commonPrt.SetVertexParameter1(t1);
        commonPrt.SetVertexParameter2(t2);
        _commonParts.Append(commonPrt);
    }

    /// <summary>Checks if there is an intersection between the edges on the ranges (for nearly coincident edges).</summary>
    protected bool IsIntersection(double aT11, double aT12, double aT21, double aT22)
    {
        bool bRet;
        Point3D p11, p12, p21, p22;
        Vector3D v11, v12, v21, v22;
        double d11_21, d11_22, d12_21, d12_22, criteria, coef;
        bool bSmall_11_21, bSmall_11_22, bSmall_12_21, bSmall_12_22;
        //
        bRet = true;
        coef = 1.0e+5;
        if (((aT12 - aT11) > coef * _res1) && ((aT22 - aT21) > coef * _res2))
        {
            coef = 5000;
        }
        else
        {
            double trMin = Min((aT12 - aT11) / _res1, (aT22 - aT21) / _res2);
            coef = trMin / 100.0;
            if (coef < 1.0)
            {
                coef = 1.0;
            }
        }
        criteria = coef * _tol;
        criteria *= criteria;
        //
        _geom1!.D1(aT11, out p11, out v11);
        _geom1.D1(aT12, out p12, out v12);
        _geom2!.D1(aT21, out p21, out v21);
        _geom2.D1(aT22, out p22, out v22);
        //
        d11_21 = p11.SquareDistance(p21);
        d11_22 = p11.SquareDistance(p22);
        d12_21 = p12.SquareDistance(p21);
        d12_22 = p12.SquareDistance(p22);
        //
        bSmall_11_21 = d11_21 < criteria;
        bSmall_11_22 = d11_22 < criteria;
        bSmall_12_21 = d12_21 < criteria;
        bSmall_12_22 = d12_22 < criteria;
        //
        if ((bSmall_11_21 && bSmall_12_22) || (bSmall_11_22 && bSmall_12_21))
        {
            if (coef == 1.0)
            {
                return bRet;
            }
            //
            double angleCriteria;
            double angle1 = 0.0, angle2 = 0.0;
            //
            angleCriteria = 5.0e-3;
            if (v11.SquareMagnitude() > global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                && v12.SquareMagnitude() > global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                && v21.SquareMagnitude() > global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                && v22.SquareMagnitude() > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                if (bSmall_11_21 && bSmall_12_22)
                {
                    angle1 = v11.Angle(v21);
                    angle2 = v12.Angle(v22);
                }
                else
                {
                    angle1 = v11.Angle(v22);
                    angle2 = v12.Angle(v21);
                }
            }
            //
            if (((angle1 < angleCriteria) || ((M_PI - angle1) < angleCriteria))
                || ((angle2 < angleCriteria) || ((M_PI - angle2) < angleCriteria)))
            {
                ProjectPointOnCurve projPC = new();
                int iErr;
                double d, t1Min = 0.0, t2Min = 0.0;
                //
                d = global::Kerf.Engine.Foundation.Precision.Infinite();
                projPC.Init(_geom2, aT21, aT22);
                iErr = FindDistPC(aT11, aT12, _geom1, _tol, _res1, projPC, ref d, ref t1Min, ref t2Min, false);
                bRet = (iErr == 2);
            }
        }
        return bRet;
    }

    /// <summary>
    /// Checks if the edges coincide on the ranges: 0 if they do, 1 if a point of the first curve can't be projected
    /// on the second, 2 if the distance is too big.
    /// </summary>
    protected int CheckCoincidence(double aT11, double aT12, double aT21, double aT22, double criteria,
                                   double curveRes1)
    {
        int iErr, nb, nb1, i;
        double t1A, t1B, t1max = 0.0, t2max = 0.0, dmax;
        ProjectPointOnCurve projPC = new();
        NCollection_Sequence<ParameterRange> ranges = new();
        //
        iErr = 0;
        dmax = -1.0;
        projPC.Init(_geom2!, aT21, aT22);
        //
        // 1. Express evaluation
        nb = 10; // Number of intervals on the curve #1
        nb1 = SplitRangeOnSegments(aT11, aT12, curveRes1, nb, ranges);
        for (i = 1; i < nb1; ++i)
        {
            ParameterRange r1 = ranges.Value(i);
            r1.Range(out t1A, out t1B);
            //
            iErr = DistPC(t1B, _geom1!, criteria, projPC, ref dmax, ref t2max);
            if (iErr != 0)
            {
                return iErr;
            }
        }
        //
        // if the ranges in aRanges are less than theCurveRes1,
        // there is no need to do step 2 (deep evaluation)
        if (nb1 < nb)
        {
            return iErr;
        }
        //
        // 2. Deep evaluation
        for (i = 2; i < nb1; ++i)
        {
            ParameterRange r1 = ranges.Value(i);
            r1.Range(out t1A, out t1B);
            //
            iErr = FindDistPC(t1A, t1B, _geom1!, criteria, curveRes1, projPC, ref dmax, ref t1max, ref t2max);
            if (iErr != 0)
            {
                return iErr;
            }
        }
        // Possible values:
        // iErr == 0 - the patches are coincided
        // iErr == 1 - a point from aC1 can not be projected on aC2
        // iErr == 2 - the distance is too big
        return iErr;
    }

    // ---- file-static functions of IntTools_EdgeEdge.cxx

    private static int FindDistPC(double aT1A,
                                  double aT1B,
                                  Curve3D c1,
                                  double criteria,
                                  double eps,
                                  ProjectPointOnCurve projPC,
                                  ref double aDmax,
                                  ref double aT1max,
                                  ref double aT2max,
                                  bool bMaxDist = true)
    {
        int iErr, iC;
        double gs, xp, a, b, xl, yp = 0.0, yl = 0.0, t2P = 0.0, t2L = 0.0;
        //
        iC = bMaxDist ? 1 : -1;
        iErr = 0;
        aT1max = aT2max = 0.0; // silence GCC warning
        //
        gs = 0.6180339887498948482045868343656; // =0.5*(1.+sqrt(5.))-1.;
        a = aT1A;
        b = aT1B;
        //
        // check bounds
        iErr = DistPC(a, c1, criteria, projPC, ref yp, ref t2P, ref aDmax, ref aT1max, ref aT2max, iC);
        if (iErr == 2)
        {
            return iErr;
        }
        //
        iErr = DistPC(b, c1, criteria, projPC, ref yl, ref t2L, ref aDmax, ref aT1max, ref aT2max, iC);
        if (iErr == 2)
        {
            return iErr;
        }
        //
        xp = a + (b - a) * gs;
        xl = b - (b - a) * gs;
        //
        iErr = DistPC(xp, c1, criteria, projPC, ref yp, ref t2P, ref aDmax, ref aT1max, ref aT2max, iC);
        if (iErr != 0)
        {
            return iErr;
        }
        //
        iErr = DistPC(xl, c1, criteria, projPC, ref yl, ref t2L, ref aDmax, ref aT1max, ref aT2max, iC);
        if (iErr != 0)
        {
            return iErr;
        }
        //
        double anEps = Max(eps, Epsilon(Max(Math.Abs(a), Math.Abs(b))) * 10.0);
        for (;;)
        {
            if (iC * (yp - yl) > 0)
            {
                a = xl;
                xl = xp;
                yl = yp;
                xp = a + (b - a) * gs;
                iErr = DistPC(xp, c1, criteria, projPC, ref yp, ref t2P, ref aDmax, ref aT1max, ref aT2max, iC);
            }
            else
            {
                b = xp;
                xp = xl;
                yp = yl;
                xl = b - (b - a) * gs;
                iErr = DistPC(xl, c1, criteria, projPC, ref yl, ref t2L, ref aDmax, ref aT1max, ref aT2max, iC);
            }
            //
            if (iErr != 0)
            {
                if ((iErr == 2) && !bMaxDist)
                {
                    xp = (a + b) * 0.5;
                    DistPC(xp, c1, criteria, projPC, ref yp, ref t2P, ref aDmax, ref aT1max, ref aT2max, iC);
                }
                return iErr;
            }
            //
            if ((b - a) < anEps)
            {
                break;
            }
        } // for (;;) {
        //
        return iErr;
    }

    private static int DistPC(double aT1,
                              Curve3D c1,
                              double criteria,
                              ProjectPointOnCurve projPC,
                              ref double aD,
                              ref double aT2,
                              ref double aDmax,
                              ref double aT1max,
                              ref double aT2max,
                              int iC)
    {
        int iErr;
        //
        iErr = DistPC(aT1, c1, criteria, projPC, ref aD, ref aT2, iC);
        if (iErr == 1)
        {
            return iErr;
        }
        //
        if (iC * (aD - aDmax) > 0)
        {
            aDmax = aD;
            aT1max = aT1;
            aT2max = aT2;
        }
        //
        return iErr;
    }

    private static int DistPC(double aT1,
                              Curve3D c1,
                              double criteria,
                              ProjectPointOnCurve projPC,
                              ref double aD,
                              ref double aT2,
                              int iC = 1)
    {
        int iErr, nbP2;
        Point3D p1;
        //
        iErr = 0;
        c1.D0(aT1, out p1);
        //
        projPC.Perform(p1);
        nbP2 = projPC.NbPoints();
        if (nbP2 == 0)
        {
            iErr = 1; // the point from aC1 can not be projected on aC2
            return iErr;
        }
        //
        aD = projPC.LowerDistance();
        aT2 = projPC.LowerDistanceParameter();
        if (iC * (aD - criteria) > 0)
        {
            iErr = 2; // the distance is too big or small
        }
        //
        return iErr;
    }

    private static int SplitRangeOnSegments(double aT1, double aT2, double resolution, int nbSeg,
                                            NCollection_Sequence<ParameterRange> segments)
    {
        double diff = aT2 - aT1;
        if (diff < resolution || nbSeg == 1)
        {
            segments.Append(new ParameterRange(aT1, aT2));
            return 1;
        }
        //
        double dt, t1x, t2x, seg;
        int nbSegments, i;
        //
        nbSegments = nbSeg;
        dt = diff / nbSegments;
        if (dt < resolution)
        {
            seg = diff / resolution;
            nbSegments = (int)seg + 1;
            dt = diff / nbSegments;
        }
        //
        t1x = aT1;
        for (i = 1; i < nbSegments; ++i)
        {
            t2x = t1x + dt;
            //
            ParameterRange r = new(t1x, t2x);
            segments.Append(r);
            //
            t1x = t2x;
        }
        //
        ParameterRange rLast = new(t1x, aT2);
        segments.Append(rLast);
        //
        return nbSegments;
    }

    private static void BndBuildBox(EdgeCurveAdaptor bac, double aT1, double aT2, double tol, out Box box)
    {
        Box b = new();
        Add3dCurve.Add(bac, aT1, aT2, tol, ref b);
        box = b;
    }

    private static double PointBoxDistance(in Box aB, in Point3D aP)
    {
        Span<double> pCoord = stackalloc double[3];
        Span<double> bMinCoord = stackalloc double[3];
        Span<double> bMaxCoord = stackalloc double[3];
        double dist, r1, r2;
        int i;
        //
        aP.Coord(out pCoord[0], out pCoord[1], out pCoord[2]);
        aB.Get(out bMinCoord[0], out bMinCoord[1], out bMinCoord[2], out bMaxCoord[0], out bMaxCoord[1],
               out bMaxCoord[2]);
        //
        dist = 0.0;
        for (i = 0; i < 3; ++i)
        {
            r1 = bMinCoord[i] - pCoord[i];
            if (r1 > 0.0)
            {
                dist += r1 * r1;
                continue;
            }
            //
            r2 = pCoord[i] - bMaxCoord[i];
            if (r2 > 0.0)
            {
                dist += r2 * r2;
            }
        }
        //
        dist = Math.Sqrt(dist);
        return dist;
    }

    private static int TypeToInteger(CurveKind cType)
    {
        int iRet;
        //
        switch (cType)
        {
            case Line:
                iRet = 0;
                break;
            case Hyperbola:
            case Parabola:
                iRet = 1;
                break;
            case Circle:
            case Ellipse:
                iRet = 2;
                break;
            case BezierCurve:
            case BSplineCurve:
                iRet = 3;
                break;
            default:
                iRet = 4;
                break;
        }
        return iRet;
    }

    private static double ResolutionCoeff(EdgeCurveAdaptor bac, in ParameterRange range)
    {
        double resCoeff = 0.0;
        //
        Curve3D curve = bac.Curve().Curve()!;
        CurveKind curveType = bac.GetType();
        //
        // OCCT's switch with the offset case falling through to the sampled one
        bool sample = false;
        switch (curveType)
        {
            case Circle:
                resCoeff = 1.0 / (2 * ((CircleCurve3D)curve).Circ().Radius());
                break;
            case Ellipse:
                resCoeff = 1.0 / ((EllipseCurve3D)curve).MajorRadius();
                break;
            case OffsetCurve:
                {
                    OffsetCurve3D offsetCurve = (OffsetCurve3D)curve;
                    Curve3D basisCurve = offsetCurve.BasisCurve();
                    CurveAdaptor gBasisCurve = new(basisCurve);
                    CurveKind bcType = gBasisCurve.GetType();
                    if (bcType == Line)
                    {
                        break;
                    }
                    else if (bcType == Circle)
                    {
                        resCoeff = 1.0 / (2 * (offsetCurve.Offset() + gBasisCurve.Circle().Radius()));
                        break;
                    }
                    else if (bcType == Ellipse)
                    {
                        resCoeff = 1.0 / (offsetCurve.Offset() + gBasisCurve.Ellipse().MajorRadius());
                        break;
                    }
                    sample = true;
                    break;
                }
            case Hyperbola:
            case Parabola:
            case OtherCurve:
                sample = true;
                break;
            default:
                break;
        }
        if (sample)
        {
            double k, kMin, dist, dt, t1, t2, t;
            int nbP, i;
            Point3D p1, p2;
            //
            nbP = 30;
            range.Range(out t1, out t2);
            dt = (t2 - t1) / nbP;
            t = t1;
            kMin = 10.0;
            //
            bac.D0(t1, out p1);
            for (i = 1; i <= nbP; ++i)
            {
                t += dt;
                bac.D0(t, out p2);
                dist = p1.Distance(p2);
                k = dt / dist;
                if (k < kMin)
                {
                    kMin = k;
                }
                p1 = p2;
            }
            //
            resCoeff = kMin;
        }
        //
        return resCoeff;
    }

    private static double Resolution(Curve3D curve, CurveKind curveType, double resCoeff, double r3D)
    {
        double res;
        //
        switch (curveType)
        {
            case Line:
                res = r3D;
                break;
            case Circle:
                {
                    double dt = resCoeff * r3D;
                    res = (dt <= 1.0) ? 2 * Math.Asin(dt) : 2 * M_PI;
                    break;
                }
            case BezierCurve:
                ((BezierCurve3D)curve).Resolution(r3D, out res);
                break;
            case BSplineCurve:
                ((BSplineCurve3D)curve).Resolution(r3D, out res);
                break;
            case OffsetCurve:
                {
                    Curve3D basisCurve = ((OffsetCurve3D)curve).BasisCurve();
                    CurveKind bcType = new CurveAdaptor(basisCurve).GetType();
                    if (bcType == Line)
                    {
                        res = r3D;
                        break;
                    }
                    else if (bcType == Circle)
                    {
                        double dt = resCoeff * r3D;
                        res = (dt <= 1.0) ? 2 * Math.Asin(dt) : 2 * M_PI;
                        break;
                    }
                    // falls through to the default case, as OCCT's [[fallthrough]]
                    res = resCoeff * r3D;
                    break;
                }
            default:
                res = resCoeff * r3D;
                break;
        }
        //
        return res;
    }

    private static double CurveDeflection(EdgeCurveAdaptor bac, in ParameterRange range)
    {
        double dt, t, t1, t2, defl;
        int i, nbP;
        Vector3D v1, v2;
        //
        defl = 0;
        nbP = 10;
        range.Range(out t1, out t2);
        dt = (t2 - t1) / nbP;
        t = t1;
        //
        bac.D1(t1, out _, out v1);
        for (i = 1; i <= nbP; ++i)
        {
            t += dt;
            bac.D1(t, out _, out v2);
            if (v1.Magnitude() > Constants.Resolution() && v2.Magnitude() > Constants.Resolution())
            {
                Direction3D d1 = new(v1), d2 = new(v2);
                defl += d1.Angle(d2);
            }
            v1 = v2;
        }
        //
        return defl;
    }

    private static bool IsClosed(Curve3D curve, double aT1, double aT2, double tol, double res)
    {
        if (Math.Abs(aT1 - aT2) < res)
        {
            return false;
        }

        Point3D p1, p2;
        curve.D0(aT1, out p1);
        curve.D0(aT2, out p2);
        //
        double d = p1.Distance(p2);
        return d < tol;
    }
}
