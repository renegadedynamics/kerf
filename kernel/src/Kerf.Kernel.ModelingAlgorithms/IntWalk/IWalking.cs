// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_IWalking.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_IWalking.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Intersection.Walking.StatusDeflection;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>What IntWalk_IWalking reads from its function, beyond math_FunctionSetWithDerivatives (decision 607).</summary>
/// <typeparam name="TSelf">The function's class.</typeparam>
internal interface IWFunction<TSelf> : ISurfFunction<SurfaceAdaptorBase>
    where TSelf : IWFunction<TSelf>
{
    /// <summary>The parametric surface.</summary>
    SurfaceAdaptorBase PSurface();

    /// <summary>A copy of <paramref name="other"/>: OCCT's copy constructor (decision 686).</summary>
    static abstract TSelf Copy(TSelf other);
}

/// <summary>
/// The walking of an implicit/parametric intersection: the polylines on a parametric surface where a function
/// F(u, v) vanishes, from start points on the surface's restrictions (open lines) and interior points (closed lines).
/// </summary>
/// <remarks>
/// OCCT's generic file IntWalk_IWalking.gxx, ported once as a generic class (decisions 607 and 686) over the
/// function (TheIWFunction: IntPatch_TheSurfFunction, later Contap_SurfFunction) and the line it builds (TheIWLine:
/// IntPatch_TheIWLineOfTheIWalking, Contap_TheIWLineOfTheIWalking). The other macros are the same types in both of
/// OCCT's instantiations and are named in the body: the path and interior points (IntSurf_PathPoint,
/// IntSurf_InteriorPoint and their tools), the surface (Adaptor3d_Surface and Adaptor3d_HSurfaceTool).
/// </remarks>
internal class IWalking<TheIWFunction, TheIWLine>
    where TheIWFunction : FunctionSetWithDerivatives, IWFunction<TheIWFunction>
    where TheIWLine : global::Kerf.Engine.Intersection.Walking.TheIWLine, new()
{
    private bool _done;
    private readonly NCollection_Sequence<PathPoint> _seqSingle = new();
    private double _fleche;
    private double _pas;
    private readonly math_Vector _tolerance = new(1, 2);
    private double _epsilon;
    private bool _reversed;
    private readonly NCollection_LinearVector<WalkingData> _wd1 = new();
    private NCollection_LinearVector<WalkingData> _wd2 = new();
    private readonly NCollection_LinearVector<int> _nbMultiplicities = new();
    private BoundRange _sRangeU = new(); // Estimated U-range for section curve
    private BoundRange _sRangeV = new(); // Estimated V-range for section curve
    private double _uMin;
    private double _uMax;
    private double _vMin;
    private double _vMax;
    private PntOn2S _previousPoint = new();
    private Vector3D _previousd3d = new();
    private Direction2D _previousd2d = new();
    private readonly NCollection_Sequence<int> _seqAjout = new();
    private readonly NCollection_Sequence<int> _seqAlone = new();
    private readonly Dictionary<int, NCollection_List<int>> _pointLineLine = new();
    private readonly NCollection_Sequence<TheIWLine> _lines = new();
    private readonly bool _toFillHoles;

    /// <summary>
    /// Deflection is the largest deflection admitted between two consecutive points of a line, Step the largest
    /// increment between two consecutive points (in the parametric space, relative to the surface's range), Epsilon
    /// the distance below which two points are confused. theToFillHoles fills possible holes between the lines
    /// (false for IntPatch's walking).
    /// </summary>
    public IWalking(double epsilon, double deflection, double step, bool toFillHoles = false)
    {
        _done = false;
        _fleche = deflection;
        _pas = step;
        _epsilon = epsilon * epsilon;
        _reversed = false;
        _uMin = 0.0;
        _uMax = 0.0;
        _vMin = 0.0;
        _vMax = 0.0;
        _toFillHoles = toFillHoles;
    }

    public void SetTolerance(double epsilon, double deflection, double step)
    {
        _fleche = deflection;
        _pas = step;
        _epsilon = epsilon * epsilon;
    }

    /// <summary>True if the computation succeeded.</summary>
    public bool IsDone => _done;

    /// <summary>The number of lines. Throws StdFail_NotDone if not done.</summary>
    public int LineCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _lines.Length();
        }
    }

    /// <summary>The line <paramref name="index"/>. Throws StdFail_NotDone if not done.</summary>
    public TheIWLine Value(int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _lines.Value(index);
    }

    /// <summary>The number of the start points on which no line starts or ends. Throws StdFail_NotDone if not done.</summary>
    public int SinglePntCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _seqSingle.Length();
        }
    }

    /// <summary>The single point <paramref name="index"/>. Throws StdFail_NotDone if not done.</summary>
    public PathPoint SinglePnt(int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _seqSingle.Value(index);
    }


    // IntWalk_IWalking_1.gxx

    private const double CosRef3D = 0.98; // rule by tests in U4, corresponds to 11.478 d
    private const double CosRef2D = 0.88; // corresponds to 25 d
    private const int MaxDivision = 60; // max number of step divisions because the angle is too great in 2d (U4)

    /// <summary>
    /// Test hook (decision 687): called each time <see cref="TestDeflection"/> returns, with its status and the point
    /// it tested (the function's point and the parameters on the parametric surface), so that a walk can be compared
    /// step by step with a native recording. It changes nothing in the computation.
    /// </summary>
    internal Action<StatusDeflection, PntOn2S>? _stepTrace;

    /// <summary>
    /// The same hook for the walkings that other algorithms create (IntPatch_ImpPrmIntersection's), on the current
    /// thread only; <see cref="_stepTrace"/> takes precedence.
    /// </summary>
    [ThreadStatic]
    internal static Action<StatusDeflection, PntOn2S>? s_threadStepTrace;

    // OCCT's stack buffers of the stop tests (written by the function, never read).
    private readonly math_Vector _bidF = new(1, 1);
    private readonly Matrix _bidD = new(1, 1, 1, 2);

    /// <summary>
    /// Additional check if the point (theU, theV) is a tangent point. If it is, any direction leads to the next
    /// intersection point, which is difficult to process; the walking then returns an empty line.
    /// </summary>
    private static bool IsTangentExtCheck(TheIWFunction func, double u, double v, double stepU,
                                          double stepV, double uinf, double usup, double vinf,
                                          double vsup)
    {
        double tol = func.Tolerance();
        const int nbItems = 4;
        ReadOnlySpan<double> parU = [Math.Min(u + stepU, usup), Math.Max(u - stepU, uinf), u, u];
        ReadOnlySpan<double> parV = [v, v, Math.Min(v + stepV, vsup), Math.Max(v - stepV, vinf)];

        math_Vector x = new(1, 2), val = new(1, 1);

        for (int i = 0; i < nbItems; i++)
        {
            x.Value(1) = parU[i];
            x.Value(2) = parV[i];

            if (!func.Value(x, val))
            {
                continue;
            }

            if (Math.Abs(func.Root()) > tol)
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// Clears the containers and puts a dummy item at index 0 of the vectors, so that start points are indexed from 1.
    /// </summary>
    protected void Clear()
    {
        _wd1.Clear();
        _wd2.Clear();
        WalkingData dummy;
        dummy.etat = -10;
        dummy.ustart = dummy.vstart = 0.0;
        _wd1.Append(dummy);
        _wd2.Append(dummy);
        _nbMultiplicities.Clear();
        _nbMultiplicities.Append(-1);

        _done = false;
        _seqAjout.Clear();
        _lines.Clear();
    }

    // ***************************************************************************
    //  etat1=12 not tangent, not passes
    //  etat1=11 tangent, not passes
    //  etat1=2  not tangent, passes
    //  etat1=1  tangent, passes
    //  after a point is processed its state becomes negative.
    // ***************************************************************************
    //  etat2=13  interior start point on closed line
    //  etat2=12  interior start point on open line
    //            (line initially closed -> la line s is open)
    //  after a point is processed (or if it is passed over during
    //  routing) its state becomes negative.
    // ****************************************************************************

    /// <summary>
    /// Searches the lines starting on a point of <paramref name="pnts1"/> (open lines) or of <paramref name="pnts2"/>
    /// (closed lines). Each point of a line verifies F(u, v) = 0.
    /// </summary>
    public void Perform(NCollection_Sequence<PathPoint> pnts1, NCollection_Sequence<InteriorPoint> pnts2,
                        TheIWFunction func, SurfaceAdaptorBase caro, bool reversed = false)
    {
        int i;
        bool rajout = false;
        int nbPnts1 = pnts1.Length();
        int nbPnts2 = pnts2.Length();
        double u, v;

        Clear();
        _reversed = reversed;

        _uMin = HSurfaceTool.FirstUParameter(caro);
        _vMin = HSurfaceTool.FirstVParameter(caro);
        _uMax = HSurfaceTool.LastUParameter(caro);
        _vMax = HSurfaceTool.LastVParameter(caro);

        if (_uMax < _uMin)
        {
            double utemp = _uMax;
            _uMax = _uMin;
            _uMin = utemp;
        }
        if (_vMax < _vMin)
        {
            double vtemp = _vMax;
            _vMax = _vMin;
            _vMin = vtemp;
        }

        double stepU = _pas * (_uMax - _uMin), stepV = _pas * (_vMax - _vMin);

        // Loading of etat1 and etat2  as well as  ustart and vstart.

        NCollection_Sequence<double> umult = new();
        NCollection_Sequence<double> vmult = new();

        int decal = 0;
        _wd1.Reserve(nbPnts1 + decal);
        _nbMultiplicities.Reserve(nbPnts1 + decal);
        for (i = 1; i <= nbPnts1 + decal; i++)
        {
            PathPoint pathPnt = pnts1.Value(i - decal);
            WalkingData wd1;
            wd1.etat = 1;
            if (!PathPointTool.IsPassingPnt(pathPnt))
            {
                wd1.etat = 11;
            }
            if (!PathPointTool.IsTangent(pathPnt))
            {
                ++wd1.etat;
            }

            if (wd1.etat == 2)
            {
                wd1.etat = 11;
            }

            PathPointTool.Value2d(pathPnt, out wd1.ustart, out wd1.vstart);
            _sRangeU.Add(wd1.ustart);
            _sRangeV.Add(wd1.vstart);

            _wd1.Append(wd1);
            int nbMult = PathPointTool.Multiplicity(pathPnt);
            _nbMultiplicities.Append(nbMult);

            for (int j = 1; j <= nbMult; j++)
            {
                PathPointTool.Parameters(pathPnt, j, out u, out v);
                umult.Append(u);
                vmult.Append(v);
            }
        }

        _wd2.Reserve(nbPnts2);
        for (i = 1; i <= nbPnts2; i++)
        {
            WalkingData wd2;
            wd2.etat = 1;
            InteriorPoint ip = pnts2.Value(i);
            InteriorPointTool.Value2d(ip, out wd2.ustart, out wd2.vstart);
            _sRangeU.Add(wd2.ustart);
            _sRangeV.Add(wd2.vstart);

            if (!IsTangentExtCheck(func, wd2.ustart, wd2.vstart, stepU, stepV, _uMin, _uMax, _vMin, _vMax))
            {
                wd2.etat = 13;
            }

            _wd2.Append(wd2);
        }

        _tolerance[1] = HSurfaceTool.UResolution(caro, global::Kerf.Engine.Foundation.Precision.Confusion());
        _tolerance[2] = HSurfaceTool.VResolution(caro, global::Kerf.Engine.Foundation.Precision.Confusion());

        func.Set(caro);

        if (_sRangeU.Delta() > Math.Max(_tolerance[1], global::Kerf.Engine.Foundation.Precision.PConfusion()))
        {
            _sRangeU.Enlarge(_sRangeU.Delta());
            _sRangeU.Common(new BoundRange(_uMin, _uMax));
        }
        else
        {
            _sRangeU = new BoundRange(_uMin, _uMax);
        }

        if (_sRangeV.Delta() > Math.Max(_tolerance[2], global::Kerf.Engine.Foundation.Precision.PConfusion()))
        {
            _sRangeV.Enlarge(_sRangeV.Delta());
            _sRangeV.Common(new BoundRange(_vMin, _vMax));
        }
        else
        {
            _sRangeV = new BoundRange(_vMin, _vMax);
        }

        // calculation of all open lines
        if (nbPnts1 != 0)
        {
            ComputeOpenLine(umult, vmult, pnts1, func, ref rajout);
        }

        // calculation of all closed lines
        if (nbPnts2 != 0)
        {
            ComputeCloseLine(umult, vmult, pnts1, pnts2, func, ref rajout);
        }

        if (_toFillHoles)
        {
            int maxNbIter = 10, nb_iter = 0;
            while (_seqAlone.Length() > 1 && nb_iter < maxNbIter)
            {
                nb_iter++;
                NCollection_Sequence<InteriorPoint> pntsInHoles = new();
                NCollection_Sequence<int> copySeqAlone = new(_seqAlone);
                FillPntsInHoles(func, copySeqAlone, pntsInHoles);
                _wd2.Clear();
                WalkingData dummy;
                dummy.etat = -10;
                dummy.ustart = dummy.vstart = 0.0;
                _wd2.Append(dummy);
                int nbHoles = pntsInHoles.Length();
                _wd2.Reserve(nbHoles);
                for (i = 1; i <= nbHoles; i++)
                {
                    WalkingData wd2;
                    wd2.etat = 13;
                    InteriorPoint ip = pntsInHoles.Value(i);
                    InteriorPointTool.Value2d(ip, out wd2.ustart, out wd2.vstart);
                    _wd2.Append(wd2);
                }
                ComputeCloseLine(umult, vmult, pnts1, pntsInHoles, func, ref rajout);
            }
        }

        for (i = 1; i <= nbPnts1; i++)
        {
            if (_wd1[i].etat > 0)
            {
                _seqSingle.Append(pnts1.Value(i));
            }
        }
        _done = true;
    }

    /// <summary>Searches the lines starting on a point of <paramref name="pnts1"/>.</summary>
    /// <remarks>
    /// As OCCT's, this overload doesn't call <see cref="Clear"/> (occt-issues.md, unit 5C): it keeps the lines of a
    /// previous Perform, and on a new object the vectors lack their dummy item 0, so the walking reads start point
    /// I + 1's data as point I's and, for the last one, past the vectors' end (out of bounds in OCCT; the port throws
    /// Standard_OutOfRange). No OCCT caller uses it (ImpPrm and Contap give interior points).
    /// </remarks>
    public void Perform(NCollection_Sequence<PathPoint> pnts1, TheIWFunction func, SurfaceAdaptorBase caro,
                        bool reversed = false)
    {
        int i;
        bool rajout = false;
        int nbPnts1 = pnts1.Length();
        double u, v;

        // OCCT: no Clear() here, unlike the other overload (see the remarks).
        _reversed = reversed;

        // Loading of etat1 as well as ustart1 and vstart1.

        NCollection_Sequence<double> umult = new();
        NCollection_Sequence<double> vmult = new();

        _wd1.Reserve(nbPnts1);
        for (i = 1; i <= nbPnts1; i++)
        {
            PathPoint pathPnt = pnts1.Value(i);
            WalkingData wd1;
            wd1.etat = 1;
            if (!PathPointTool.IsPassingPnt(pathPnt))
            {
                wd1.etat = 11;
            }
            if (!PathPointTool.IsTangent(pathPnt))
            {
                ++wd1.etat;
            }
            PathPointTool.Value2d(pathPnt, out wd1.ustart, out wd1.vstart);
            _wd1.Append(wd1);
            int nbMult = PathPointTool.Multiplicity(pathPnt);
            _nbMultiplicities.Append(nbMult);

            for (int j = 1; j <= nbMult; j++)
            {
                PathPointTool.Parameters(pathPnt, j, out u, out v);
                umult.Append(u);
                vmult.Append(v);
            }
        }

        _tolerance[1] = HSurfaceTool.UResolution(caro, global::Kerf.Engine.Foundation.Precision.Confusion());
        _tolerance[2] = HSurfaceTool.VResolution(caro, global::Kerf.Engine.Foundation.Precision.Confusion());

        _uMin = HSurfaceTool.FirstUParameter(caro);
        _vMin = HSurfaceTool.FirstVParameter(caro);
        _uMax = HSurfaceTool.LastUParameter(caro);
        _vMax = HSurfaceTool.LastVParameter(caro);

        if (_uMax < _uMin)
        {
            double utemp = _uMax;
            _uMax = _uMin;
            _uMin = utemp;
        }
        if (_vMax < _vMin)
        {
            double vtemp = _vMax;
            _vMax = _vMin;
            _vMin = vtemp;
        }

        func.Set(caro);

        // calcul de toutes les lignes ouvertes
        if (nbPnts1 != 0)
        {
            ComputeOpenLine(umult, vmult, pnts1, func, ref rajout);
        }

        for (i = 1; i <= nbPnts1; i++)
        {
            if (_wd1[i].etat > 0)
            {
                _seqSingle.Append(pnts1.Value(i));
            }
        }
        _done = true;
    }

    // IntWalk_IWalking_2.gxx

    private static void CutVectorByTolerances(ref Vector2D vector, math_Vector tolerance)
    {
        if (Math.Abs(vector.X) < tolerance[1])
        {
            vector = vector.WithX(0.0);
        }
        if (Math.Abs(vector.Y) < tolerance[2])
        {
            vector = vector.WithY(0.0);
        }
    }

    /// <summary>
    /// Locates the point (u, v) in the natural domain of the surface and updates (u, v) for the next point. If the
    /// approached point is out of the domain, it is put on the border in the best direction, the step is changed and a
    /// bound blocks one parameter for the next FunctionSetRoot. Returns true if the point was put on the border.
    /// </summary>
    protected bool Cadrage(math_Vector bornInf, math_Vector bornSup, math_Vector uVap, ref double step, int stepSign)
    {
        // there are always :
        // BorInf(1) <= UVap(1) <= BornSup(1) et BorInf(2) <= UVap(2) <= BornSup(2)
        double duvx = _previousd2d.X;
        double duvy = _previousd2d.Y;

        if (!_reversed)
        {
            _previousPoint.ParametersOnS2(out uVap[1], out uVap[2]);
        }
        else
        {
            _previousPoint.ParametersOnS1(out uVap[1], out uVap[2]);
        }

        double u1 = uVap[1] + step * duvx * stepSign;
        double v1 = uVap[2] + step * duvy * stepSign;

        bool infu = (u1 <= bornInf[1] + global::Kerf.Engine.Foundation.Precision.PConfusion());
        bool supu = (u1 >= bornSup[1] - global::Kerf.Engine.Foundation.Precision.PConfusion());
        bool infv = (v1 <= bornInf[2] + global::Kerf.Engine.Foundation.Precision.PConfusion());
        bool supv = (v1 >= bornSup[2] - global::Kerf.Engine.Foundation.Precision.PConfusion());

        double theStepU, theStepV;

        if (!infu && !supu && !infv && !supv)
        {
            uVap[1] = u1;
            uVap[2] = v1;
            return false;
        }

        if ((infu || supu) && (infv || supv))
        {
            if (infu)
            { // jag 940616
                if (duvx != 0.0)
                {
                    theStepU = Math.Abs((bornInf[1] - uVap[1]) / duvx); // iso U =BornInf(1)
                }
                else
                {
                    theStepU = step;
                }
            }
            else
            {
                if (duvx != 0.0)
                {
                    theStepU = Math.Abs((bornSup[1] - uVap[1]) / duvx); // iso U =BornSup(1)
                }
                else
                {
                    theStepU = step;
                }
            }
            if (infv)
            { // jag 940616
                if (duvy != 0.0)
                {
                    theStepV = Math.Abs((bornInf[2] - uVap[2]) / duvy); // iso V =BornInf(2)
                }
                else
                {
                    theStepV = step;
                }
            }
            else
            {
                if (duvy != 0.0)
                {
                    theStepV = Math.Abs((bornSup[2] - uVap[2]) / duvy); // iso V =BornSup(2)
                }
                else
                {
                    theStepV = step;
                }
            }

            if (theStepU <= theStepV)
            {
                step = theStepU;
                if (infu)
                {
                    uVap[1] = bornInf[1];
                    bornSup[1] = bornInf[1];
                }
                else
                {
                    uVap[1] = bornSup[1];
                    bornInf[1] = bornSup[1];
                }
                uVap[2] += step * duvy * stepSign;
            }
            else
            {
                step = theStepV;
                if (infv)
                {
                    uVap[2] = bornInf[2];
                    bornSup[2] = bornInf[2];
                }
                else
                {
                    uVap[2] = bornSup[2];
                    bornInf[2] = bornSup[2];
                }
                uVap[1] += step * duvx * stepSign;
            }
            return true;
        }

        else if (infu)
        { // jag 940616
            if (duvx != 0.0)
            {
                double aStep = Math.Abs((bornInf[1] - uVap[1]) / duvx); // iso U =BornInf(1)
                if (aStep < step)
                {
                    step = aStep;
                }
            }
            bornSup[1] = bornInf[1]; // limit the parameter
            uVap[1] = bornInf[1];
            uVap[2] += step * duvy * stepSign;
            return true;
        }
        else if (supu)
        { // jag 940616
            if (duvx != 0.0)
            {
                double aStep = Math.Abs((bornSup[1] - uVap[1]) / duvx); // iso U =BornSup(1)
                if (aStep < step)
                {
                    step = aStep;
                }
            }
            bornInf[1] = bornSup[1]; // limit the parameter
            uVap[1] = bornSup[1];
            uVap[2] += step * duvy * stepSign;
            return true;
        }
        else if (infv)
        { // jag 940616
            if (duvy != 0.0)
            {
                double aStep = Math.Abs((bornInf[2] - uVap[2]) / duvy); // iso V =BornInf(2)
                if (aStep < step)
                {
                    step = aStep;
                }
            }
            bornSup[2] = bornInf[2];
            uVap[1] += step * duvx * stepSign;
            uVap[2] = bornInf[2];
            return true;
        }
        else if (supv)
        { // jag 940616
            if (duvy != 0.0)
            {
                double aStep = Math.Abs((bornSup[2] - uVap[2]) / duvy); // iso V =BornSup(2)
                if (aStep < step)
                {
                    step = aStep;
                }
            }
            bornInf[2] = bornSup[2];
            uVap[1] += step * duvx * stepSign;
            uVap[2] = bornSup[2];
            return true;
        }
        return true;
    }

    /// <summary>
    /// Stop test for an OPEN line: crossing test on all interior points, then stop test on all start points. If a stop
    /// is detected, <paramref name="irang"/> is the index of the stop point and <paramref name="uv"/> its parameters.
    /// </summary>
    protected bool TestArretPassage(NCollection_Sequence<double> umult, NCollection_Sequence<double> vmult,
                                    TheIWFunction sp, math_Vector uv, ref int irang)
    {
        // Umult et Vmult : table of stop (or crossing) points on border,
        //         here only the crossing points are taken into account.
        // UV     : the current point.
        // Irang  : at exit : give index of the stop point in uvstart1 or 0.
        //          consider that there is no risk of crossing only if there is one crossing point.
        double up, vp, du, dv, dup, dvp, utest, vtest;
        int j, n, ind;
        double tolu = _tolerance[1];
        double tolv = _tolerance[2];
        double tolu2 = 10.0 * _tolerance[1];
        double tolv2 = 10.0 * _tolerance[2];

        bool arrive = false;

        // crossing test  on point that can start a loop; mark
        // as processed if passes through an open line

        if (!_reversed)
        {
            _previousPoint.ParametersOnS2(out up, out vp);
        }
        else
        {
            _previousPoint.ParametersOnS1(out up, out vp);
        }

        for (int i = 1; i < _wd2.Size(); i++)
        {
            if (_wd2[i].etat > 0)
            {
                // debug jag 05.04.94

                //      if ((Up-wd2[i].ustart)*(UV(1)-wd2[i].ustart) +
                //	  (Vp-wd2[i].vstart)*(UV(2)-wd2[i].vstart) <= 0)
                utest = _wd2[i].ustart;
                vtest = _wd2[i].vstart;

                du = uv[1] - utest;
                dv = uv[2] - vtest;
                dup = up - utest;
                dvp = vp - vtest;

                //-- lbr le 30 oct 97

                // IFV for OCC20285

                if ((Math.Abs(du) < tolu2 && Math.Abs(dv) < tolv2) || (Math.Abs(dup) < tolu2 && Math.Abs(dvp) < tolv2))
                {
                    _wd2[i].etat = -_wd2[i].etat;
                }
                else
                {
                    double dDu = (uv[1] - up);
                    double dDv = (uv[2] - vp);
                    double ddd = dDu * dDu + dDv * dDv;
                    double dd1 = du * du + dv * dv;
                    if (dd1 <= ddd)
                    {
                        double dd2 = dup * dup + dvp * dvp;
                        if (dd2 <= ddd && ((du * dup) + (dv * dvp * tolu / tolv) <= 0.0))
                        {
                            _wd2[i].etat = -_wd2[i].etat;
                        }
                    }
                }
            }
        }

        // stop test on point given at input and not yet processed

        //  Modified by Sergey KHROMOV - Tue Nov 20 10:55:01 2001 Begin
        // Check of all path points in the following order:
        //   * First check all not treated points;
        //   * After that check of already treated ones.
        int l;

        //// Modified by jgv, 28.07.2010 for OCC21914 ////
        // There are several path points between (Up,Vp) and UV
        // So several path points satisfy the condition
        // Dup*UV1mUtest + Dvp*UV2mVtest) < 0
        // We choose from them the path point with
        // minimum distance to (Up,Vp)
        NCollection_Sequence<int> i_candidates = new();
        NCollection_Sequence<double> SqDist_candidates = new();

        for (l = 1; l <= 2 && !arrive; l++)
        {
            bool isToCheck;

            for (int i = 1; i < _wd1.Size(); i++)
            {
                if (l == 1)
                {
                    isToCheck = (_wd1[i].etat > 0);
                }
                else
                {
                    isToCheck = (_wd1[i].etat < 0);
                }

                if (isToCheck)
                {
                    //  Modified by Sergey KHROMOV - Tue Nov 20 11:03:16 2001 End

                    // debug jag voir avec isg

                    utest = _wd1[i].ustart;
                    vtest = _wd1[i].vstart;
                    dup = up - utest;
                    dvp = vp - vtest;
                    if (Math.Abs(dup) >= tolu || Math.Abs(dvp) >= tolv)
                    {
                        double uv1mUtest = uv[1] - utest;
                        double uv2mVtest = uv[2] - vtest;
                        if (((dup * uv1mUtest + dvp * uv2mVtest) < 0)
                            || (Math.Abs(uv1mUtest) < tolu && Math.Abs(uv2mVtest) < tolv))
                        {
                            i_candidates.Append(i);
                            SqDist_candidates.Append(dup * dup + dvp * dvp);
                        }
                        else if (_nbMultiplicities[i] > 0 && i_candidates.IsEmpty())
                        {
                            n = 0;
                            for (int k = 1; k < i; k++)
                            {
                                n += _nbMultiplicities[k];
                            }
                            for (j = n + 1; j <= n + _nbMultiplicities[i]; j++)
                            {
                                if (((up - umult.Value(j)) * (uv[1] - umult.Value(j))
                                         + (vp - vmult.Value(j)) * (uv[2] - vmult.Value(j))
                                     < 0)
                                    || (Math.Abs(uv[1] - umult.Value(j)) < tolu && Math.Abs(uv[2] - vmult.Value(j)) < tolv))
                                {
                                    irang = i;
                                    arrive = true;
                                    uv[1] = utest;
                                    uv[2] = vtest;
                                    break;
                                }
                            }
                        }
                        if (arrive)
                        {
                            sp.Values(uv, _bidF, _bidD);
                            break;
                        }
                    }
                }
            } // end of for (i = 1; i < wd1.Size(); i++)
            if (!i_candidates.IsEmpty())
            {
                double minSqDist = RealLast();
                for (ind = 1; ind <= i_candidates.Length(); ind++)
                {
                    if (SqDist_candidates.Value(ind) < minSqDist)
                    {
                        minSqDist = SqDist_candidates.Value(ind);
                        irang = i_candidates.Value(ind);
                    }
                }
                arrive = true;
                uv[1] = _wd1[irang].ustart;
                uv[2] = _wd1[irang].vstart;
            }
        } // end of for (l = 1; l <= 2 && !Arrive; l++)
        return arrive;
    }

    /// <summary>
    /// Stop test for a CLOSED line started at the interior point <paramref name="index"/>: crossing test on all
    /// interior points, then on all passing points (<paramref name="irang"/>: the passing point crossed, or 0).
    /// </summary>
    protected bool TestArretPassage(NCollection_Sequence<double> umult, NCollection_Sequence<double> vmult,
                                    math_Vector uv, int index, ref int irang)
    {
        double up, vp, scal;
        bool arrive = false;
        int n, k, i;
        double utest, vtest;
        double tolu = _tolerance[1];
        double tolv = _tolerance[2];

        // tests of stop and of crossing on all interior points.

        if (!_reversed)
        {
            _previousPoint.ParametersOnS2(out up, out vp);
        }
        else
        {
            _previousPoint.ParametersOnS1(out up, out vp);
        }

        double uv1 = uv[1];
        double uv2 = uv[2];

        // Normalizing factor. If it is less than 1.0 then the range will be expanded.
        // This is no good for computation. Therefore, it is limited.
        // Do not limit this factor in case of highly anisotropic parametrization
        //(parametric space is considerably larger in one direction than another).
        bool isHighlyAnisotropic = Math.Max(tolu, tolv) > 1000.0 * Math.Min(tolu, tolv);
        double deltau = _sRangeU.IsVoid
                            ? _uMax - _uMin
                            : (isHighlyAnisotropic ? _sRangeU.Delta() : Math.Max(_sRangeU.Delta(), 1.0));
        double deltav = _sRangeV.IsVoid
                            ? _vMax - _vMin
                            : (isHighlyAnisotropic ? _sRangeV.Delta() : Math.Max(_sRangeV.Delta(), 1.0));

        up /= deltau;
        uv1 /= deltau;
        vp /= deltav;
        uv2 /= deltav;

        tolu /= deltau;
        tolv /= deltav;

        double tolu2 = tolu + tolu;
        double tolv2 = tolv + tolv;

        double dPreviousCurrent = (up - uv1) * (up - uv1) + (vp - uv2) * (vp - uv2);
        for (k = 1; k < _wd2.Size(); k++)
        {
            if (_wd2[k].etat > 0)
            {
                utest = _wd2[k].ustart;
                vtest = _wd2[k].vstart;

                utest /= deltau;
                vtest /= deltav;

                double uv1mUtest = uv1 - utest;
                double uv2mVtest = uv2 - vtest;
                if ((uv1mUtest < tolu2 && uv1mUtest > -tolu2) && (uv2mVtest < tolv2 && uv2mVtest > -tolv2))
                {
                    if (index != k)
                    {
                        _wd2[k].etat = -_wd2[k].etat; //-- mark the point as a crossing point
                    }
                    else
                    { //-- Index == k
                        arrive = true;
                    }
                }
                else
                {
                    double upmUtest = (up - utest);
                    double vpmVtest = (vp - vtest);
                    double dPreviousStart = (upmUtest) * (upmUtest) + (vpmVtest) * (vpmVtest);
                    double dCurrentStart = uv1mUtest * uv1mUtest + uv2mVtest * uv2mVtest;

                    scal = (upmUtest) * (uv1mUtest) + (vpmVtest) * (uv2mVtest);
                    if ((Math.Abs(upmUtest) < tolu && Math.Abs(vpmVtest) < tolv))
                    {
                        if (index != k)
                        {
                            _wd2[k].etat = -_wd2[k].etat;
                        }
                    }
                    else if (scal < 0 && (dPreviousStart + dCurrentStart < dPreviousCurrent))
                    {
                        if (index == k)
                        { // on a boucle.
                            arrive = true;
                        }
                        else
                        {
                            _wd2[k].etat = -_wd2[k].etat; // mark the point as a crossing point
                        }
                    }
                    else if (k != index)
                    {
                        if (dPreviousStart < dPreviousCurrent * 0.25)
                        {
                            _wd2[k].etat = -_wd2[k].etat; // mark the point as a crossing point
                        }
                        else
                        {
                            if (dCurrentStart < dPreviousCurrent * 0.25)
                            {
                                _wd2[k].etat = -_wd2[k].etat; // mark the point as a crossing point
                            }
                            else
                            {
                                double uMidUtest = 0.5 * (uv1 + up) - utest;
                                double vMidVtest = 0.5 * (uv2 + vp) - vtest;
                                double dMiddleStart = uMidUtest * uMidUtest + vMidVtest * vMidVtest;

                                if (dMiddleStart < dPreviousCurrent * 0.5)
                                {
                                    _wd2[k].etat = -_wd2[k].etat; // mark the point as a crossing point
                                }
                            }
                        }
                    }
                }
            }
        }

        // crossing test on crossing points.

        irang = 0;
        for (i = 1; i < _wd1.Size(); i++)
        {
            if (_wd1[i].etat > 0 && _wd1[i].etat < 11)
            { // test of crossing points
                utest = _wd1[i].ustart;
                vtest = _wd1[i].vstart;
                utest /= deltau;
                vtest /= deltav;

                if (((up - utest) * (uv1 - utest) + (vp - vtest) * (uv2 - vtest) < 0)
                    || (Math.Abs(uv1 - utest) < tolu && Math.Abs(uv2 - vtest) < tolv))
                {
                    irang = i;
                }
                else if (_nbMultiplicities[i] > 0)
                {
                    n = 0;
                    for (k = 1; k < i; k++)
                    {
                        n = n + _nbMultiplicities[k];
                    }
                    for (int j = n + 1; j <= n + _nbMultiplicities[i]; j++)
                    {
                        double umultj = umult.Value(j) / deltau;
                        double vmultj = vmult.Value(j) / deltav;
                        if (((up - umultj) * (uv1 - umultj) + (vp - vmultj) * (uv2 - vmultj) < 0)
                            || (Math.Abs(uv1 - umultj) < tolu && Math.Abs(uv2 - vmultj) < tolv))
                        {
                            irang = i;
                            break;
                        }
                    }
                }
            }
        }
        return arrive;
    }

    /// <summary>
    /// Stop test on the added points (the points on the natural border that were not given at input): returns the
    /// added point in <paramref name="psol"/>, its index among the added points in <paramref name="irang"/> and its
    /// parameters in <paramref name="uv"/>.
    /// </summary>
    protected bool TestArretAjout(TheIWFunction sp, math_Vector uv, ref int irang, ref PntOn2S psol)
    {
        bool arrive = false;
        double u1, v1;
        double up, vp;

        if (!_reversed)
        {
            _previousPoint.ParametersOnS2(out up, out vp);
        }
        else
        {
            _previousPoint.ParametersOnS1(out up, out vp);
        }

        int nbAjout = _seqAjout.Length();
        for (int i = 1; i <= nbAjout; i++)
        {
            irang = _seqAjout.Value(i);

            // add test std::abs(Irang) <= lines.Length() for the case when
            // a closed line is opened by adding a  1 point on this same
            // line. Anyway there is a huge problem as 2 points will be
            // added on this line...

            if (Math.Abs(irang) <= _lines.Length())
            {
                TheIWLine line = _lines.Value(Math.Abs(irang));
                if (irang > 0)
                {
                    psol = line.Value(line.PointCount);
                }
                else
                {
                    psol = line.Value(1);
                }
                if (!_reversed)
                {
                    psol.ParametersOnS2(out u1, out v1);
                }
                else
                {
                    psol.ParametersOnS1(out u1, out v1);
                }
                if (((up - u1) * (uv[1] - u1) + (vp - v1) * (uv[2] - v1)) < 0
                    || (Math.Abs(uv[1] - u1) < _tolerance[1] && Math.Abs(uv[2] - v1) < _tolerance[2]))
                {
                    // jag 940615	Irang= -std::abs(Irang);
                    arrive = true;
                    uv[1] = u1;
                    uv[2] = v1;
                    sp.Values(uv, _bidF, _bidD);
                    break;
                }
            }
        }
        return arrive;
    }

    /// <summary>
    /// Finds interior start points in the holes between the lines' lone ends (<paramref name="copySeqAlone"/>): the
    /// root of the function near the middle of the two nearest ends, when it is between them and on no line.
    /// </summary>
    protected void FillPntsInHoles(TheIWFunction sp, NCollection_Sequence<int> copySeqAlone,
                                   NCollection_Sequence<InteriorPoint> pntsInHoles)
    {
        math_Vector bornInf = new(1, 2), bornSup = new(1, 2);
        bornInf[1] = _uMin;
        bornSup[1] = _uMax;
        bornInf[2] = _vMin;
        bornSup[2] = _vMax;
        _pointLineLine.Clear();
        NCollection_Sequence<int> seqToRemove = new();
        HashSet<int> badSolutions = new();

        for (int i = 1; i < copySeqAlone.Length(); i++)
        {
            int irang1 = copySeqAlone.Value(i);
            if (irang1 == 0)
            {
                continue;
            }
            bool toRemove = false;
            PntOn2S pointAlone1, pointAlone2 = new();
            TheIWLine line1 = _lines.Value(Math.Abs(irang1));
            if (irang1 > 0)
            {
                pointAlone1 = line1.Value(line1.PointCount);
            }
            else
            {
                pointAlone1 = line1.Value(1);
            }
            Point2D p2d1 = pointAlone1.ValueOnSurface(_reversed), p2d2;
            double minSqDist = RealLast();
            int minRang = 0, minIndex = 0;
            for (int j = i + 1; j <= copySeqAlone.Length(); j++)
            {
                int irang2 = copySeqAlone.Value(j);
                if (irang2 == 0 || badSolutions.Contains(irang2))
                {
                    continue;
                }
                TheIWLine line2a = _lines.Value(Math.Abs(irang2));
                if (irang2 > 0)
                {
                    pointAlone2 = line2a.Value(line2a.PointCount);
                }
                else
                {
                    pointAlone2 = line2a.Value(1);
                }
                p2d2 = pointAlone2.ValueOnSurface(_reversed);
                double sqDist = p2d1.SquareDistanceTo(p2d2);
                if (sqDist < minSqDist)
                {
                    minSqDist = sqDist;
                    minRang = irang2;
                    minIndex = j;
                }
            }
            // processing points from Irang1 and MinRang
            if (minRang == 0)
            {
                seqToRemove.Append(irang1);
                badSolutions.Clear();
                continue;
            }
            // Ends of same line
            if (Math.Abs(irang1) == Math.Abs(minRang) && _lines.Value(Math.Abs(irang1)).PointCount == 2)
            {
                seqToRemove.Append(irang1);
                seqToRemove.Append(minRang);
                copySeqAlone[i] = 0;
                copySeqAlone[minIndex] = 0;
                badSolutions.Clear();
                continue;
            }
            ///////////////////
            TheIWLine line2 = _lines.Value(Math.Abs(minRang));
            if (minRang > 0)
            {
                pointAlone2 = line2.Value(line2.PointCount);
            }
            else
            {
                pointAlone2 = line2.Value(1);
            }
            Point3D pnt1 = pointAlone1.Value();
            Point3D pnt2 = pointAlone2.Value();
            p2d2 = pointAlone2.ValueOnSurface(_reversed);
            double minSqDist3d = pnt1.SquareDistanceTo(pnt2);
            if (minSqDist3d <= _epsilon
                || (Math.Abs(p2d1.X - p2d2.X) <= _tolerance[1] && Math.Abs(p2d1.Y - p2d2.Y) <= _tolerance[2])) // close points
            {
                toRemove = true;
            }
            else // real curve
            {
                math_Vector uVap = new(1, 2), uv = new(1, 2);
                uVap[1] = (p2d1.X + p2d2.X) / 2;
                uVap[2] = (p2d1.Y + p2d2.Y) / 2;
                FunctionSetRoot rsnld = new(sp, _tolerance);
                rsnld.Perform(sp, uVap, bornInf, bornSup);
                if (rsnld.IsDone && Math.Abs(sp.Root()) <= sp.Tolerance() && !sp.IsTangent())
                {
                    rsnld.Root(uv);
                    Point2D pmid = new(uv[1], uv[2]);
                    Vector2D p1P2 = new(p2d1, p2d2);
                    Vector2D p1Pmid = new(p2d1, pmid);
                    Vector2D p2Pmid = new(p2d2, pmid);
                    double scalProd1 = p1P2 * p1Pmid;
                    double scalProd2 = p1P2 * p2Pmid;
                    bool isPmidValid = (scalProd1 > 0.0 && scalProd2 < 0); // Pmid is in the middle
                    if (isPmidValid)
                    {
                        for (int iline = 1; iline <= _lines.Length(); iline++)
                        {
                            if (IsPointOnLine(pmid, iline))
                            {
                                isPmidValid = false;
                                break;
                            }
                        }
                    }
                    if (isPmidValid)
                    {
                        InteriorPoint point = new(sp.Point(), uv[1], uv[2], sp.Direction3d(),
                                                           new Vector2D(sp.Direction2d()));
                        pntsInHoles.Append(point);
                        NCollection_List<int> lineLine = new();
                        lineLine.Append(irang1);
                        lineLine.Append(minRang);
                        _pointLineLine[pntsInHoles.Length()] = lineLine;
                    }
                    else
                    {
                        badSolutions.Add(minRang);
                        i--;
                        continue;
                    }
                }
                else
                {
                    badSolutions.Add(minRang);
                    i--;
                    continue;
                }
            }
            copySeqAlone[i] = 0;
            copySeqAlone[minIndex] = 0;
            if (toRemove)
            {
                seqToRemove.Append(irang1);
                seqToRemove.Append(minRang);
            }
            badSolutions.Clear();
        }

        for (int i = 1; i <= seqToRemove.Length(); i++)
        {
            for (int j = 1; j <= _seqAlone.Length(); j++)
            {
                if (_seqAlone.Value(j) == seqToRemove.Value(i))
                {
                    _seqAlone.Remove(j);
                    break;
                }
            }
        }
    }

    /// <summary>
    /// Stop test on a border: on all the start points already marked and on the whole current line, which is cut at
    /// the stop point if one is found. |Irang| is the stop point's index or 0; if Irang &lt; 0 the point must be added to
    /// the line (no cut). UV is the stop point's parameters.
    /// </summary>
    protected void TestArretCadre(NCollection_Sequence<double> umult, NCollection_Sequence<double> vmult, TheIWLine line,
                                  TheIWFunction sp, math_Vector uv, ref int irang)
    {
        double scal, up, vp, uc, vc;
        int n;
        bool found = false;

        irang = 0;
        for (int i = 1; i < _wd1.Size(); i++)
        {
            if (_wd1[i].etat < 0)
            {
                n = 0; // range in UVMult.
                if (_nbMultiplicities[i] > 0)
                {
                    for (int k = 1; k < i; k++)
                    {
                        n += _nbMultiplicities[k];
                    }
                }
                if (!_reversed)
                {
                    line.Value(1).ParametersOnS2(out up, out vp);
                }
                else
                {
                    line.Value(1).ParametersOnS1(out up, out vp);
                }
                int nbp = line.PointCount;
                Vector2D vec1, vec2;
                for (int j = 2; j <= nbp; j++)
                {
                    if (!_reversed)
                    {
                        line.Value(j).ParametersOnS2(out uc, out vc);
                    }
                    else
                    {
                        line.Value(j).ParametersOnS1(out uc, out vc);
                    }

                    vec1 = new Vector2D(up - _wd1[i].ustart, vp - _wd1[i].vstart);
                    vec2 = new Vector2D(uc - _wd1[i].ustart, vc - _wd1[i].vstart);
                    CutVectorByTolerances(ref vec1, _tolerance);
                    CutVectorByTolerances(ref vec2, _tolerance);

                    scal = vec1 * vec2;

                    // if a stop point is found: stop the line on this point.
                    if (scal < 0)
                    {
                        line.Cut(j);
                        nbp = line.PointCount;
                        irang = i;
                        uv[1] = _wd1[irang].ustart;
                        uv[2] = _wd1[irang].vstart;
                        found = true;
                    }
                    else if (Math.Abs(uc - _wd1[i].ustart) < _tolerance[1] && Math.Abs(vc - _wd1[i].vstart) < _tolerance[2])
                    {
                        line.Cut(j);
                        nbp = line.PointCount;
                        irang = i;
                        uv[1] = _wd1[irang].ustart;
                        uv[2] = _wd1[irang].vstart;
                        found = true;
                    }
                    else if (_nbMultiplicities[i] > 0)
                    {
                        for (int k = n + 1; k <= n + _nbMultiplicities[i]; k++)
                        {
                            vec1 = vec1.WithCoord(up - umult.Value(k), vp - vmult.Value(k));
                            vec2 = vec2.WithCoord(uc - umult.Value(k), vc - vmult.Value(k));
                            CutVectorByTolerances(ref vec1, _tolerance);
                            CutVectorByTolerances(ref vec2, _tolerance);

                            scal = vec1 * vec2;

                            if (scal < 0)
                            {
                                line.Cut(j);
                                nbp = line.PointCount;
                                irang = i;
                                uv[1] = _wd1[irang].ustart;
                                uv[2] = _wd1[irang].vstart;
                                found = true;
                                break;
                            }
                            else if (Math.Abs(uc - umult.Value(k)) < _tolerance[1] && Math.Abs(vc - vmult.Value(k)) < _tolerance[2])
                            {
                                line.Cut(j);
                                nbp = line.PointCount;
                                irang = i;
                                uv[1] = _wd1[irang].ustart;
                                uv[2] = _wd1[irang].vstart;
                                found = true;
                                break;
                            }
                        }
                    }
                    if (found)
                    {
                        sp.Values(uv, _bidF, _bidD);
                        int NBP = line.PointCount;
                        line.TangentVector(out int indextg);
                        if (indextg > NBP)
                        {
                            if (j > 3 && j <= NBP + 1)
                            {
                                // OCCT computes and orients this direction, then sets the previous tangent (kept:
                                // Direction3d throws at a tangent point, as OCCT's).
                                Vector3D dir3d = sp.Direction3d();
                                Vector3D dir3d1 = new(line.Value(j - 2).Value(), line.Value(j - 1).Value());
                                double dot = dir3d.Dot(dir3d1);
                                if (dot < 0.0)
                                { // Normally this Function should not be used often !!!
                                    dir3d = dir3d.Reversed();
                                }
                                line.SetTangentVector(_previousd3d, j - 1);
                            }
                        }

                        return;
                    }
                    up = uc;
                    vp = vc;
                }

                // now the last point of the line and the last calculated point are compated.
                // there will be no need to "Cut"

                vec1 = new Vector2D(up - _wd1[i].ustart, vp - _wd1[i].vstart);
                vec2 = new Vector2D(uv[1] - _wd1[i].ustart, uv[2] - _wd1[i].vstart);
                CutVectorByTolerances(ref vec1, _tolerance);
                CutVectorByTolerances(ref vec2, _tolerance);

                scal = vec1 * vec2;

                if (scal < 0)
                {
                    irang = i;
                    uv[1] = _wd1[irang].ustart;
                    uv[2] = _wd1[irang].vstart;
                    found = true;
                }
                else if (Math.Abs(uv[1] - _wd1[i].ustart) < _tolerance[1] && Math.Abs(uv[2] - _wd1[i].vstart) < _tolerance[2])
                {
                    irang = i;
                    uv[1] = _wd1[irang].ustart;
                    uv[2] = _wd1[irang].vstart;
                    found = true;
                }
                else if (_nbMultiplicities[i] > 0)
                {
                    for (int j = n + 1; j <= n + _nbMultiplicities[i]; j++)
                    {
                        vec1 = vec1.WithCoord(up - umult.Value(j), vp - vmult.Value(j));
                        vec2 = vec2.WithCoord(uv[1] - umult.Value(j), uv[2] - vmult.Value(j));
                        CutVectorByTolerances(ref vec1, _tolerance);
                        CutVectorByTolerances(ref vec2, _tolerance);

                        scal = vec1 * vec2;

                        if (scal < 0)
                        {
                            irang = i;
                            uv[1] = _wd1[irang].ustart;
                            uv[2] = _wd1[irang].vstart;
                            found = true;
                            break;
                        }
                        else if (Math.Abs(uv[1] - umult.Value(j)) < _tolerance[1] && Math.Abs(uv[2] - vmult.Value(j)) < _tolerance[2])
                        {
                            irang = i;
                            uv[1] = _wd1[irang].ustart;
                            uv[2] = _wd1[irang].vstart;
                            found = true;
                            break;
                        }
                    }
                }
                if (found)
                {
                    irang = -irang; // jag 941017
                    sp.Values(uv, _bidF, _bidD);
                    return;
                }
            }
        }
    }

    // IntWalk_IWalking_3.gxx

    /// <summary>
    /// Processing of the open lines: from each start point that is neither passing nor tangent and not yet processed,
    /// walk with a step from the deflection and the largest step, framing the approached point on the borders,
    /// dividing the step when no point is found, until a stop test passes (start point, added point, border).
    /// </summary>
    protected void ComputeOpenLine(NCollection_Sequence<double> umult, NCollection_Sequence<double> vmult,
                                   NCollection_Sequence<PathPoint> pnts1, TheIWFunction func, ref bool rajout)
    {
        int i = 0, n = 0, saveN = 0;
        math_Vector bornInf = new(1, 2), bornSup = new(1, 2), uVap = new(1, 2);
        double pasC = 0.0, pasCu = 0.0, pasCv = 0.0;
        bool arrive = false; // shows if the line ends
        bool cadre = false; // shows if one is on border of the domain
        bool arretAjout = false; // shows if one is on added point
        PntOn2S psol = new();
        TheIWLine currentLine; // line under construction
        bool tgtend = false;

        StatusDeflection status = StatusDeflection.OK, statusPrecedent = StatusDeflection.OK;

        int nbDivision = 0;
        // number of divisions of step for each section

        int stepSign = 0;

        PathPoint pathPnt = new();

        bornInf[1] = _uMin;
        bornSup[1] = _uMax;
        bornInf[2] = _vMin;
        bornSup[2] = _vMax;

        FunctionSetRoot rsnld = new(func, _tolerance);
        int nbPath = pnts1.Length();

        // modified by NIZHNY-MKK  Fri Oct 27 12:32:34 2000.BEGIN
        int[] movementdirectioninfo = new int[nbPath + 1];
        for (i = 0; i <= nbPath; i++)
        {
            movementdirectioninfo[i] = 0;
        }
        // modified by NIZHNY-MKK  Fri Oct 27 12:32:38 2000.END

        TheIWFunction funcForDuplicate = TheIWFunction.Copy(func);

        for (i = 1; i <= nbPath; i++)
        {
            // start point of the progression
            //      if (wd1[I].etat > 11) {
            //  modified by NIZHNY-MKK  Fri Oct 27 12:33:37 2000.BEGIN
            if ((_wd1[i].etat > 11) || ((_wd1[i].etat < -11) && (movementdirectioninfo[i] != 0)))
            {
                // modified by NIZHNY-MKK  Fri Oct 27 12:33:43 2000.END
                pathPnt = pnts1.Value(i);
                uVap[1] = _wd1[i].ustart;
                uVap[2] = _wd1[i].vstart;
                MakeWalkingPoint(11, uVap[1], uVap[2], func, ref _previousPoint);

                if (IsPointOnLine(_previousPoint, bornInf, bornSup, rsnld, funcForDuplicate))
                {
                    _wd1[i].etat = -Math.Abs(_wd1[i].etat); // mark point as processed
                    continue;
                }

                currentLine = new TheIWLine();
                currentLine.SetTangencyAtBegining(false);
                tgtend = false;
                currentLine.AddStatusFirst(false, true, i, pathPnt);
                _previousd3d = func.Direction3d();
                _previousd2d = func.Direction2d();
                currentLine.AddPoint(_previousPoint);
                // modified by NIZHNY-MKK  Fri Oct 27 12:34:32 2000.BEGIN
                if (movementdirectioninfo[i] != 0)
                {
                    if (movementdirectioninfo[i] < 0)
                    {
                        stepSign = -1;
                        currentLine.SetTangentVector(_previousd3d.Reversed(), 1);
                    }
                    else
                    {
                        stepSign = 1;
                        currentLine.SetTangentVector(_previousd3d, 1);
                    }
                }
                else
                {
                    double tyutuyt = PathPointTool.Direction3d(pathPnt) * _previousd3d;
                    if (tyutuyt < 0)
                    {
                        stepSign = -1;
                        currentLine.SetTangentVector(_previousd3d.Reversed(), 1);
                    }
                    else
                    {
                        stepSign = 1;
                        currentLine.SetTangentVector(_previousd3d, 1);
                    }
                }
                // modified by NIZHNY-MKK  Fri Oct 27 12:34:37 2000.END

                //  Modified by Sergey KHROMOV - Tue Nov 20 10:41:45 2001 Begin
                _wd1[i].etat = -Math.Abs(_wd1[i].etat);
                movementdirectioninfo[i] = (movementdirectioninfo[i] == 0) ? stepSign : 0;
                //  Modified by Sergey KHROMOV - Tue Nov 20 10:41:56 2001 End
                // first step of advancement
                double d2dx = Math.Abs(_previousd2d.X);
                double d2dy = Math.Abs(_previousd2d.Y);
                if (d2dx < _tolerance[1])
                {
                    pasC = _pas * (_vMax - _vMin) / d2dy;
                }
                else if (d2dy < _tolerance[2])
                {
                    pasC = _pas * (_uMax - _uMin) / d2dx;
                }
                else
                {
                    pasC = _pas * Math.Min((_uMax - _uMin) / d2dx, (_vMax - _vMin) / d2dy);
                }

                arrive = false;
                arretAjout = false;
                nbDivision = 0;
                statusPrecedent = StatusDeflection.OK;
                // modified by NIZHNY-MKK  Fri Oct 27 12:39:37 2000
                int indexOfPathPointDoNotCheck = 0;
                int nbIter = 10;
                int nbBadRootIter = 0; // consecutive iterations where Root > Tolerance
                while (!arrive)
                {
                    cadre = Cadrage(bornInf, bornSup, uVap, ref pasC, stepSign);
                    //  Border?

                    rsnld.Perform(func, uVap, bornInf, bornSup);

                    if (cadre)
                    {
                        bornInf[1] = _uMin;
                        bornSup[1] = _uMax;
                        bornInf[2] = _vMin;
                        bornSup[2] = _vMax;
                    }
                    if (rsnld.IsDone)
                    {
                        if (Math.Abs(func.Root()) > func.Tolerance())
                        {
                            pasC = pasC / 2.0;
                            pasCu = Math.Abs(pasC * _previousd2d.X);
                            pasCv = Math.Abs(pasC * _previousd2d.Y);
                            ++nbBadRootIter;
                            if ((pasCu <= _tolerance[1] && pasCv <= _tolerance[2]) || nbBadRootIter > MaxDivision)
                            {
                                // Step collapsed below tolerance, or Cadrage keeps resetting
                                // the step (preventing convergence) after too many attempts.
                                if (currentLine.PointCount == 1)
                                {
                                    break;
                                }
                                arrive = true;
                                currentLine.AddStatusLast(false);
                                tgtend = true; // check
                                rajout = true;
                                _seqAlone.Append(_lines.Length() + 1);
                                _seqAjout.Append(_lines.Length() + 1);
                            }
                        }
                        else
                        { // test stop
                            nbBadRootIter = 0;
                            rsnld.Root(uVap);
                            arrive = TestArretPassage(umult, vmult, func, uVap, ref n);
                            if (arrive)
                            {
                                cadre = false;
                                // in case if there is a frame and arrive at the same time
                            }
                            else
                            {
                                if (rajout)
                                {
                                    arretAjout = TestArretAjout(func, uVap, ref n, ref psol);
                                    saveN = n;
                                    if (arretAjout)
                                    {
                                        // jag 940615
                                        tgtend = _lines.Value(n).IsTangentAtEnd;
                                        n = -n;
                                    }
                                }
                                // modified by NIZHNY-MKK  Thu Nov  2 15:09:08 2000.BEGIN
                                if (!(rajout && arretAjout))
                                {
                                    double prevUp, prevVp;
                                    if (!_reversed)
                                    {
                                        _previousPoint.ParametersOnS2(out prevUp, out prevVp);
                                    }
                                    else
                                    {
                                        _previousPoint.ParametersOnS1(out prevUp, out prevVp);
                                    }
                                    arrive = TestPassedSolutionWithNegativeState(_wd1, umult, vmult, prevUp, prevVp,
                                                                                 _nbMultiplicities, _tolerance, func,
                                                                                 uVap, ref n, _bidF, _bidD);
                                    if (arrive)
                                    {
                                        cadre = false;
                                    }
                                }
                                // modified by NIZHNY-MKK  Thu Nov  2 15:09:13 2000.END
                                if (!arretAjout && cadre)
                                {
                                    if (currentLine.PointCount == 1)
                                    {
                                        break; // cancel the line
                                    }
                                    TestArretCadre(umult, vmult, currentLine, func, uVap, ref n);
                                    //		if (N == 0) {
                                    if (n <= 0)
                                    { // jag 941017
                                        MakeWalkingPoint(2, uVap[1], uVap[2], func, ref psol);
                                        tgtend = func.IsTangent();
                                        n = -n;
                                    }
                                }
                            }
                            status = TestDeflection(func, arrive, uVap, statusPrecedent, ref nbDivision, ref pasC,
                                                     stepSign);
                            statusPrecedent = status;
                            if (status == StatusDeflection.PasTropGrand)
                            {
                                arrive = false;
                                arretAjout = false;
                                tgtend = false; // jag 940615
                                if (!_reversed)
                                {
                                    _previousPoint.ParametersOnS2(out uVap[1], out uVap[2]);
                                }
                                else
                                {
                                    _previousPoint.ParametersOnS1(out uVap[1], out uVap[2]);
                                }
                            }
                            else if (arretAjout || cadre)
                            {
                                arrive = true;
                                currentLine.AddStatusLast(false);
                                // if (aStatus != IntWalk_ArretSurPointPrecedent)
                                currentLine.AddPoint(psol);
                                // Remove <SaveN> from <seqAlone>
                                for (int iseq = 1; iseq <= _seqAlone.Length(); iseq++)
                                {
                                    if (_seqAlone.Value(iseq) == saveN)
                                    {
                                        _seqAlone.Remove(iseq);
                                        break;
                                    }
                                }

                                if (cadre && n == 0)
                                {
                                    rajout = true;
                                    _seqAjout.Append(_lines.Length() + 1);
                                }
                            }
                            else if (status == StatusDeflection.ArretSurPointPrecedent)
                            {
                                if (currentLine.PointCount == 1)
                                { // cancel the line
                                    arrive = false;
                                    break;
                                }
                                arrive = true;
                                rajout = true;
                                // AddAlonePoint();
                                _seqAlone.Append(_lines.Length() + 1);
                                _seqAjout.Append(_lines.Length() + 1);
                                currentLine.AddStatusLast(false);
                                tgtend = true; // check
                            }
                            else if (arrive)
                            {
                                if (currentLine.PointCount == 1 && // cancel the line
                                    (n == i || status == StatusDeflection.PointConfondu))
                                {
                                    // if N == I the main uv is probably lost
                                    // or the point is a point of accumulation
                                    // if point is confused the start data is bad
                                    arrive = false;
                                    break;
                                }
                                // necessarily N > 0 jag 940617
                                // point of stop given at input
                                pathPnt = pnts1.Value(n);

                                int etat1N = _wd1[n].etat;
                                // modified by NIZHNY-MKK  Thu Nov  2 15:09:51 2000.BEGIN
                                // 	      if (etat1N < 11) { // passing point that is a stop
                                if (Math.Abs(etat1N) < 11)
                                { // passing point that is a stop
                                    // modified by NIZHNY-MKK  Thu Nov  2 15:12:11 2000.END
                                    if (status == StatusDeflection.ArretSurPoint)
                                    {
                                        currentLine.AddStatusLast(false);
                                        tgtend = true; // need check
                                    }
                                    else
                                    {
                                        arrive = false;
                                    }
                                    currentLine.AddIndexPassing(n);
                                }
                                else
                                { // point of stop given at input
                                    if (etat1N == 11)
                                    {
                                        tgtend = true;
                                    }
                                    currentLine.AddStatusLast(true, n, pathPnt);
                                }
                                AddPointInCurrentLine(n, pathPnt, currentLine);
                                if ((etat1N != 1 && etat1N != 11))
                                {
                                    // modified by NIZHNY-MKK  Fri Oct 27 12:43:05 2000.BEGIN
                                    // 		wd1[N].etat= - wd1[N].etat;
                                    _wd1[n].etat = -Math.Abs(etat1N);
                                    movementdirectioninfo[n] = (movementdirectioninfo[n] == 0) ? stepSign : 0;
                                    if (arrive && movementdirectioninfo[n] != 0)
                                    {
                                        indexOfPathPointDoNotCheck = n;
                                    }

                                    if (arrive)
                                    {
                                        rajout = true;
                                        _seqAjout.Append(_lines.Length() + 1);
                                    }
                                    // modified by NIZHNY-MKK  Fri Oct 27 12:45:33 2000.END
                                }
                            }
                            else if (status == StatusDeflection.ArretSurPoint)
                            {
                                arrive = true;
                                currentLine.AddStatusLast(false);
                                tgtend = true;
                                MakeWalkingPoint(1, uVap[1], uVap[2], func, ref psol);
                                currentLine.AddPoint(psol);
                                rajout = true;
                                _seqAlone.Append(_lines.Length() + 1);
                                _seqAjout.Append(_lines.Length() + 1);
                            }
                            else if (status == StatusDeflection.OK)
                            {
                                MakeWalkingPoint(2, uVap[1], uVap[2], func, ref _previousPoint);
                                _previousd3d = func.Direction3d();
                                _previousd2d = func.Direction2d();
                                currentLine.AddPoint(_previousPoint);
                            }
                            else if (status == StatusDeflection.PointConfondu)
                            {
                                nbIter--;
                            }
                        }
                    }
                    else
                    { // no numerical solution
                        pasC = pasC / 2.0;
                        pasCu = Math.Abs(pasC * _previousd2d.X);
                        pasCv = Math.Abs(pasC * _previousd2d.Y);
                        if (pasCu <= _tolerance[1] && pasCv <= _tolerance[2])
                        {
                            if (currentLine.PointCount == 1)
                            {
                                break;
                            }
                            arrive = true;
                            currentLine.AddStatusLast(false);
                            tgtend = true; // need check
                            rajout = true;
                            _seqAlone.Append(_lines.Length() + 1);
                            _seqAjout.Append(_lines.Length() + 1);
                        }
                    }

                    if (nbIter < 0)
                    {
                        break;
                    }
                } // end of started line

                if (arrive)
                {
                    currentLine.SetTangencyAtEnd(tgtend);
                    _lines.Append(currentLine);
                    // modified by NIZHNY-MKK  Fri Oct 27 12:59:29 2000.BEGIN
                    movementdirectioninfo[i] = 0;
                    if (_wd1[i].etat > 0)
                    {
                        // modified by NIZHNY-MKK  Fri Oct 27 12:59:42 2000.END
                        _wd1[i].etat = -_wd1[i].etat;
                    }

                    //-- lbr le 5 juin 97 (Pb ds Contap)
                    for (int av = 1; av <= nbPath; av++)
                    {
                        // modified by NIZHNY-MKK  Fri Oct 27 13:00:22 2000.BEGIN
                        // 	  if (wd1[av].etat > 11) {
                        if ((_wd1[av].etat > 11)
                            || ((av != i) && (av != indexOfPathPointDoNotCheck) && (_wd1[av].etat < -11)
                                && (movementdirectioninfo[av] != 0)))
                        {
                            // modified by NIZHNY-MKK  Fri Oct 27 13:00:26 2000.END
                            double uav = _wd1[av].ustart;
                            double vav = _wd1[av].vstart;
                            double uavp, vavp;
                            PntOn2S avP = currentLine.Value(currentLine.PointCount);
                            if (!_reversed)
                            {
                                avP.ParametersOnS2(out uavp, out vavp);
                            }
                            else
                            {
                                avP.ParametersOnS1(out uavp, out vavp);
                            }
                            uav -= uavp;
                            vav -= vavp;
                            uav *= 0.001;
                            vav *= 0.001;
                            if (Math.Abs(uav) < _tolerance[1] && Math.Abs(vav) < _tolerance[2])
                            {
                                // modified by NIZHNY-MKK  Fri Oct 27 13:01:38 2000.BEGIN
                                // 	      wd1[av].etat=-wd1[av].etat;
                                if (_wd1[av].etat < 0)
                                {
                                    movementdirectioninfo[av] = 0;
                                }
                                else
                                {
                                    _wd1[av].etat = -_wd1[av].etat;
                                    movementdirectioninfo[av] = stepSign;
                                }
                                // modified by NIZHNY-MKK  Fri Oct 27 13:01:42 2000.END
                                currentLine.AddStatusLast(true, av, pnts1.Value(av));
                            }

                            PntOn2S avPP = currentLine.Value(1);
                            if (!_reversed)
                            {
                                avPP.ParametersOnS2(out uavp, out vavp);
                            }
                            else
                            {
                                avPP.ParametersOnS1(out uavp, out vavp);
                            }
                            uav = _wd1[av].ustart;
                            vav = _wd1[av].vstart;
                            uav -= uavp;
                            vav -= vavp;
                            uav *= 0.001;
                            vav *= 0.001;
                            if (Math.Abs(uav) < _tolerance[1] && Math.Abs(vav) < _tolerance[2])
                            {
                                // modified by NIZHNY-MKK  Fri Oct 27 13:02:49 2000.BEGIN
                                // 	      wd1[av].etat=-wd1[av].etat;
                                if (_wd1[av].etat < 0)
                                {
                                    movementdirectioninfo[av] = 0;
                                }
                                else
                                {
                                    _wd1[av].etat = -_wd1[av].etat;
                                    movementdirectioninfo[av] = -stepSign;
                                }
                                // modified by NIZHNY-MKK  Fri Oct 27 13:02:52 2000.END
                                currentLine.AddStatusFirst(false, true, av, pnts1.Value(av));
                            }
                        }
                    }
                }
            } // end of point processing
        } // end of all points
    }

    // modified by NIZHNY-MKK  Thu Nov  2 15:07:53 2000.BEGIN
    private static bool TestPassedSolutionWithNegativeState(NCollection_LinearVector<WalkingData> wd,
                                                            NCollection_Sequence<double> umult,
                                                            NCollection_Sequence<double> vmult, double prevUp,
                                                            double prevVp, NCollection_LinearVector<int> nbMultiplicities,
                                                            math_Vector tolerance, TheIWFunction sp, math_Vector uv,
                                                            ref int irang, math_Vector bidF, Matrix bidD)
    {
        bool arrive = false;
        double dup, dvp, utest, vtest;
        double tolu = tolerance[1];
        double tolv = tolerance[2];
        int i, j, k, n;
        for (i = 1; i < wd.Size(); i++)
        {
            if (wd[i].etat < -11)
            {
                // debug jag see with isg

                utest = wd[i].ustart;
                vtest = wd[i].vstart;
                dup = prevUp - utest;
                dvp = prevVp - vtest;
                if (Math.Abs(dup) >= tolu || Math.Abs(dvp) >= tolv)
                {
                    double uv1mUtest = uv[1] - utest;
                    double uv2mVtest = uv[2] - vtest;
                    if (((dup * uv1mUtest + dvp * uv2mVtest) < 0)
                        || (Math.Abs(uv1mUtest) < tolu && Math.Abs(uv2mVtest) < tolv))
                    {
                        irang = i;
                        arrive = true;
                        uv[1] = utest;
                        uv[2] = vtest;
                    }
                    else if (nbMultiplicities[i] > 0)
                    {
                        n = 0;
                        for (k = 1; k < i; k++)
                        {
                            n += nbMultiplicities[k];
                        }
                        for (j = n + 1; j <= n + nbMultiplicities[i]; j++)
                        {
                            if (((prevUp - umult.Value(j)) * (uv[1] - umult.Value(j))
                                     + (prevVp - vmult.Value(j)) * (uv[2] - vmult.Value(j))
                                 < 0)
                                || (Math.Abs(uv[1] - umult.Value(j)) < tolu && Math.Abs(uv[2] - vmult.Value(j)) < tolv))
                            {
                                irang = i;
                                arrive = true;
                                uv[1] = utest;
                                uv[2] = vtest;
                                break;
                            }
                        }
                    }
                    if (arrive)
                    {
                        sp.Values(uv, bidF, bidD);
                        break;
                    }
                }
            }
        }
        return arrive;
    }
    // modified by NIZHNY-MKK  Thu Nov  2 15:07:58 2000.END

    // IntWalk_IWalking_4.gxx

    /// <summary>
    /// Processing of the closed lines: from each interior point not yet processed, walk until the line closes, or
    /// until it stops (then the line is opened and walked in the other direction).
    /// </summary>
    protected void ComputeCloseLine(NCollection_Sequence<double> umult, NCollection_Sequence<double> vmult,
                                    NCollection_Sequence<PathPoint> pnts1,
                                    NCollection_Sequence<InteriorPoint> pnts2, TheIWFunction func,
                                    ref bool rajout)
    {
        int i = 0, n = 0, saveN = 0;
        math_Vector bornInf = new(1, 2), bornSup = new(1, 2);
        math_Vector uvap = new(1, 2); // parameters of current approach
        double pasC = 0.0; // step of advancement on the tangent
        double pasCu = 0.0; // step of advancement current by U
        double pasCv = 0.0; // step of advancement current by V
        double pasSav = 0.0; // save first step of advancement
        bool arrive = false; // show if line ends
        bool cadre = false; // show if on border of the  domains
        bool arretAjout = false; // show if on the added point
        PntOn2S psol = new();
        TheIWLine currentLine; // line under construction
        PathPoint pathPnt = new();
        InteriorPoint loopPnt = new();

        bool tgtbeg = false, tgtend = false;

        int stepSign = 0;

        StatusDeflection status = StatusDeflection.OK, statusPrecedent;
        int nbDivision = 0; // number of divisions of step
        // during calculation of  1 section

        int ipass = 0;
        // index in the iterator of points on edge of point of passage

        bornInf[1] = _uMin;
        bornSup[1] = _uMax;
        bornInf[2] = _vMin;
        bornSup[2] = _vMax;

        FunctionSetRoot rsnld = new(func, _tolerance);
        int nbLoop = pnts2.Length();

        // Check borders for degeneracy:
        Matrix d = new(1, 1, 1, 2);
        const int nbSamplePnt = 10;
        Span<bool> isLeftDegeneratedBorder = [true, true];
        Span<bool> isRightDegeneratedBorder = [true, true];
        math_Vector step = new(1, 2);
        // OCCT: aStep = (BornSup - BornInf) / (aNbSamplePnt - 1), item by item.
        for (int idx = 1; idx <= 2; idx++)
        {
            step[idx] = (bornSup[idx] - bornInf[idx]) / (nbSamplePnt - 1);
        }
        for (int borderIdx = 1; borderIdx <= 2; borderIdx++)
        {
            int changeIdx = borderIdx == 2 ? 1 : 2;
            math_Vector UV = new(1, 2);

            // Left border.
            UV[borderIdx] = bornInf[borderIdx];
            for (int paramIdx = 0; paramIdx < nbSamplePnt; paramIdx++)
            {
                double param = bornInf[changeIdx] + paramIdx * step[changeIdx];
                UV[changeIdx] = param;
                func.Derivatives(UV, d);
                if (Math.Abs(d[1, changeIdx]) > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    isLeftDegeneratedBorder[borderIdx - 1] = false;
                    break;
                }
            }

            // Right border.
            UV[borderIdx] = bornSup[borderIdx];
            for (int paramIdx = 0; paramIdx < nbSamplePnt; paramIdx++)
            {
                double param = bornInf[changeIdx] + paramIdx * step[changeIdx];
                UV[changeIdx] = param;
                func.Derivatives(UV, d);
                if (Math.Abs(d[1, changeIdx]) > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    isRightDegeneratedBorder[borderIdx - 1] = false;
                    break;
                }
            }
        }

        TheIWFunction funcForDuplicate = TheIWFunction.Copy(func);

        // OCCT's uvprev[2] and uv[2] of the degenerated-border fix, hoisted out of the loops.
        Span<double> uvprev = stackalloc double[2];
        Span<double> uv = stackalloc double[2];

        for (i = 1; i <= nbLoop; i++)
        {
            if (_wd2[i].etat > 12)
            { // start point of closed line
                loopPnt = pnts2.Value(i);
                _previousPoint.SetValue(InteriorPointTool.Value3d(loopPnt), _reversed, _wd2[i].ustart,
                                       _wd2[i].vstart);

                if (IsPointOnLine(_previousPoint, bornInf, bornSup, rsnld, funcForDuplicate))
                {
                    _wd2[i].etat = -_wd2[i].etat; // mark point as processed
                    continue;
                }

                _previousd3d = InteriorPointTool.Direction3d(loopPnt);
                _previousd2d = InteriorPointTool.Direction2d(loopPnt);

                currentLine = new TheIWLine();
                currentLine.AddPoint(_previousPoint);
                currentLine.SetTangentVector(_previousd3d, 1);
                tgtbeg = false;
                tgtend = false;
                uvap[1] = _wd2[i].ustart;
                uvap[2] = _wd2[i].vstart;

                stepSign = 1;

                // first step of advancement

                double d2dx = Math.Abs(_previousd2d.X);
                double d2dy = Math.Abs(_previousd2d.Y);
                if (d2dx < _tolerance[1])
                {
                    pasC = _pas * (_vMax - _vMin) / d2dy;
                }
                else if (d2dy < _tolerance[2])
                {
                    pasC = _pas * (_uMax - _uMin) / d2dx;
                }
                else
                {
                    pasC = _pas * Math.Min((_uMax - _uMin) / d2dx, (_vMax - _vMin) / d2dy);
                }

                pasSav = pasC;

                arrive = false;
                arretAjout = false;
                nbDivision = 0;
                statusPrecedent = StatusDeflection.OK;
                int nbIter = 10;
                int nbBadRootIter = 0; // consecutive iterations where Root > Tolerance
                while (!arrive)
                { // as no test of stop is passed
                    cadre = Cadrage(bornInf, bornSup, uvap, ref pasC, stepSign); // border?

                    rsnld.Perform(func, uvap, bornInf, bornSup);

                    bool isOnDegeneratedBorder = false;

                    if (cadre)
                    { // update of limits.
                        bornInf[1] = _uMin;
                        bornSup[1] = _uMax;
                        bornInf[2] = _vMin;
                        bornSup[2] = _vMax;
                    }
                    if (rsnld.IsDone)
                    {
                        if (Math.Abs(func.Root()) > func.Tolerance())
                        { // no solution for the tolerance
                            pasC = pasC / 2.0;
                            pasCu = Math.Abs(pasC * _previousd2d.X);
                            pasCv = Math.Abs(pasC * _previousd2d.Y);
                            ++nbBadRootIter;

                            if ((pasCu <= _tolerance[1] && pasCv <= _tolerance[2]) || nbBadRootIter > MaxDivision)
                            {
                                if (currentLine.PointCount == 1)
                                {
                                    RemoveTwoEndPoints(i);
                                    break; // cancel the line
                                }
                                if (_wd2[i].etat > 12)
                                { // the line should become open
                                    _wd2[i].etat = 12; // declare it open
                                    arretAjout = false;
                                    OpenLine(0, psol, pnts1, func, currentLine);
                                    stepSign = -1;
                                    statusPrecedent = StatusDeflection.OK;
                                    arrive = false;
                                    pasC = pasSav;
                                    nbBadRootIter = 0;
                                    rajout = true;
                                    _seqAlone.Append(-_lines.Length() - 1);
                                    _seqAjout.Append(-_lines.Length() - 1);
                                }
                                else
                                { // line s is open
                                    arrive = true;
                                    currentLine.AddStatusLast(false);
                                    rajout = true;
                                    _seqAlone.Append(_lines.Length() + 1);
                                    _seqAjout.Append(_lines.Length() + 1);
                                    tgtend = true;
                                }
                            }
                        }
                        else
                        { // there is a solution
                            nbBadRootIter = 0;
                            rsnld.Root(uvap);

                            // Avoid uninitialized memory access.
                            if (currentLine.PointCount > 2)
                            {
                                for (int coordIdx = 1; coordIdx <= 2; coordIdx++)
                                {
                                    // Check degenerated cases and fix if possible.
                                    if ((isLeftDegeneratedBorder[coordIdx - 1]
                                         && Math.Abs(uvap[coordIdx] - bornInf[coordIdx]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                                        || (isRightDegeneratedBorder[coordIdx - 1]
                                            && Math.Abs(uvap[coordIdx] - bornSup[coordIdx]) < global::Kerf.Engine.Foundation.Precision.PConfusion()))
                                    {
                                        if (!_reversed)
                                        {
                                            currentLine.Value(currentLine.PointCount - 1).ParametersOnS2(out uvprev[0], out uvprev[1]);
                                            currentLine.Value(currentLine.PointCount).ParametersOnS2(out uv[0], out uv[1]);
                                        }
                                        else
                                        {
                                            currentLine.Value(currentLine.PointCount - 1).ParametersOnS1(out uvprev[0], out uvprev[1]);
                                            currentLine.Value(currentLine.PointCount).ParametersOnS1(out uv[0], out uv[1]);
                                        }

                                        double scaleCoeff = 0.0;

                                        // Avoid finite cycle which lead to stop computing iline.
                                        if (status != StatusDeflection.PasTropGrand)
                                        {
                                            // Make linear extrapolation.
                                            if (Math.Abs(uv[coordIdx - 1] - uvprev[coordIdx - 1]) > Constants.Resolution())
                                            {
                                                scaleCoeff = Math.Abs((uvap[coordIdx] - uv[coordIdx - 1])
                                                                       / (uv[coordIdx - 1] - uvprev[coordIdx - 1]));
                                            }
                                            int fixIdx = coordIdx == 1 ? 2 : 1; // Fixing index;
                                            uvap[fixIdx] = uv[fixIdx - 1] + (uv[fixIdx - 1] - uvprev[fixIdx - 1]) * scaleCoeff;
                                            isOnDegeneratedBorder = true;
                                        }
                                    }
                                }
                            }

                            arrive = TestArretPassage(umult, vmult, uvap, i, ref ipass);
                            if (arrive)
                            { // reset proper parameter to test the arrow.
                                psol = currentLine.Value(1);
                                if (!_reversed)
                                {
                                    psol.ParametersOnS2(out uvap[1], out uvap[2]);
                                }
                                else
                                {
                                    psol.ParametersOnS1(out uvap[1], out uvap[2]);
                                }
                                cadre = false;
                                // in case if there is a frame and arrival at the same time
                            }
                            else
                            { // modif jag 940615
                                if (rajout)
                                { // test on added points
                                    arretAjout = TestArretAjout(func, uvap, ref n, ref psol);
                                    saveN = n;
                                    if (arretAjout)
                                    {
                                        if (n > 0)
                                        {
                                            tgtend = _lines.Value(n).IsTangentAtEnd;
                                            n = -n;
                                        }
                                        else
                                        {
                                            tgtend = _lines.Value(-n).IsTangentAtBegining;
                                        }
                                        arrive = (_wd2[i].etat == 12);
                                    }
                                }

                                if (!arretAjout && cadre)
                                { // test on already marked points
                                    if (currentLine.PointCount == 1)
                                    {
                                        RemoveTwoEndPoints(i);
                                        break; // cancel the line
                                    }
                                    TestArretCadre(umult, vmult, currentLine, func, uvap, ref n);
                                    saveN = n;
                                    //		if (N==0) {
                                    if (n <= 0)
                                    { // jag 941017
                                        MakeWalkingPoint(2, uvap[1], uvap[2], func, ref psol);
                                        tgtend = func.IsTangent(); // jag 940616
                                        if (isOnDegeneratedBorder)
                                        {
                                            tgtend = true;
                                        }
                                        n = -n;
                                    }
                                    arrive = (_wd2[i].etat == 12); // the line is open
                                }
                            }
                            status = TestDeflection(func, arrive, uvap, statusPrecedent, ref nbDivision, ref pasC,
                                                     stepSign);

                            if (isOnDegeneratedBorder && tgtend)
                            {
                                status = StatusDeflection.ArretSurPoint;
                            }

                            statusPrecedent = status;
                            if (status == StatusDeflection.PasTropGrand)
                            { // division of the step
                                arrive = false;
                                arretAjout = false;
                                tgtend = false; // jag 940616
                                if (!_reversed)
                                {
                                    _previousPoint.ParametersOnS2(out uvap[1], out uvap[2]);
                                }
                                else
                                {
                                    _previousPoint.ParametersOnS1(out uvap[1], out uvap[2]);
                                }
                            }
                            else if (arretAjout || cadre)
                            {
                                if (arrive)
                                { // line s is open
                                    currentLine.AddStatusLast(false);
                                    // if (aStatus != IntWalk_ArretSurPointPrecedent)
                                    currentLine.AddPoint(psol);

                                    // Remove <SaveN> from <seqAlone> and, if it is first found point,
                                    // from <seqAjout> too
                                    if (IsValidEndPoint(i, saveN))
                                    {
                                        for (int iseq = 1; iseq <= _seqAlone.Length(); iseq++)
                                        {
                                            if (_seqAlone.Value(iseq) == saveN)
                                            {
                                                _seqAlone.Remove(iseq);
                                                break;
                                            }
                                        }
                                        if (currentLine.PointCount <= 3)
                                        {
                                            for (int iseq = 1; iseq <= _seqAjout.Length(); iseq++)
                                            {
                                                if (_seqAjout.Value(iseq) == saveN)
                                                {
                                                    _seqAjout.Remove(iseq);
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if (_seqAlone.Last() == -_lines.Length() - 1)
                                        {
                                            _seqAlone.Remove(_seqAlone.Length());
                                            _seqAjout.Remove(_seqAjout.Length());
                                        }
                                        RemoveTwoEndPoints(i);
                                        arrive = false;
                                        break; // cancel the line
                                    }

                                    if (cadre && n == 0)
                                    {
                                        rajout = true;
                                        // seqAlone.Append(lines.Length()+1);
                                        _seqAjout.Append(_lines.Length() + 1);
                                    }
                                }
                                else
                                { // open
                                    _wd2[i].etat = 12; // declare it open
                                    tgtbeg = tgtend;
                                    tgtend = false;
                                    arretAjout = false;
                                    stepSign = -1;
                                    statusPrecedent = StatusDeflection.OK;
                                    pasC = pasSav;
                                    // Check if <Psol> has been really updated
                                    if (arrive || rajout || (!arretAjout && cadre && saveN <= 0))
                                    {
                                        if (status == StatusDeflection.ArretSurPointPrecedent)
                                        {
                                            currentLine.AddPoint(psol);
                                            OpenLine(0, psol, pnts1, func, currentLine);
                                        }
                                        else
                                        {
                                            OpenLine(-_lines.Length() - 1, psol, pnts1, func, currentLine);
                                        }
                                    }
                                    // Remove <SaveN> from <seqAlone> and, if it is first found point,
                                    // from <seqAjout> too
                                    if (IsValidEndPoint(i, saveN))
                                    {
                                        for (int iseq = 1; iseq <= _seqAlone.Length(); iseq++)
                                        {
                                            if (_seqAlone.Value(iseq) == saveN)
                                            {
                                                _seqAlone.Remove(iseq);
                                                break;
                                            }
                                        }
                                        if (currentLine.PointCount <= 2)
                                        {
                                            for (int iseq = 1; iseq <= _seqAjout.Length(); iseq++)
                                            {
                                                if (_seqAjout.Value(iseq) == saveN)
                                                {
                                                    _seqAjout.Remove(iseq);
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    else
                                    {
                                        RemoveTwoEndPoints(i);
                                        break; // cancel the line
                                    }

                                    if (cadre && n == 0)
                                    {
                                        rajout = true;
                                        _seqAjout.Append(-_lines.Length() - 1);
                                    }
                                }
                            }
                            else if (status == StatusDeflection.ArretSurPointPrecedent)
                            {
                                if (currentLine.PointCount == 1)
                                { // cancel the line
                                    arrive = false;
                                    RemoveTwoEndPoints(i);
                                    break;
                                }
                                if (_wd2[i].etat > 12)
                                { // the line should become open
                                    _wd2[i].etat = 12; // declare it open
                                    arretAjout = false;
                                    OpenLine(0, psol, pnts1, func, currentLine);
                                    stepSign = -1;
                                    statusPrecedent = StatusDeflection.OK;
                                    arrive = false;
                                    pasC = pasSav;
                                    nbBadRootIter = 0;
                                    rajout = true;
                                    _seqAlone.Append(-_lines.Length() - 1);
                                    _seqAjout.Append(-_lines.Length() - 1);
                                }
                                else
                                { // line s is open
                                    arrive = true;
                                    currentLine.AddStatusLast(false);
                                    rajout = true;
                                    _seqAlone.Append(_lines.Length() + 1);
                                    _seqAjout.Append(_lines.Length() + 1);
                                }
                            }
                            else if (arrive)
                            {
                                if (_wd2[i].etat > 12)
                                { // line closed good case
                                    currentLine.AddStatusFirstLast(true, false, false);
                                    currentLine.AddPoint(currentLine.Value(1));
                                }
                                else if ((n > 0) && (pnts1.Length() >= n))
                                {
                                    // point of stop given at input
                                    pathPnt = pnts1.Value(n);
                                    currentLine.AddStatusLast(true, n, pathPnt);
                                    AddPointInCurrentLine(n, pathPnt, currentLine);
                                }
                            }
                            else if (status == StatusDeflection.ArretSurPoint)
                            {
                                if (_wd2[i].etat > 12)
                                { // line should become open
                                    _wd2[i].etat = 12; // declare it open
                                    tgtbeg = true;
                                    tgtend = false;
                                    n = -_lines.Length() - 1;
                                    psol.SetValue(func.Point(), _reversed, uvap[1], uvap[2]);
                                    OpenLine(n, psol, pnts1, func, currentLine);
                                    stepSign = -1;
                                    rajout = true;
                                    _seqAlone.Append(n);
                                    _seqAjout.Append(n);
                                    statusPrecedent = StatusDeflection.OK;
                                    arrive = false;
                                    pasC = pasSav;
                                    nbBadRootIter = 0;
                                }
                                else
                                {
                                    arrive = true;
                                    if (ipass != 0)
                                    { // point of passage, point of stop
                                        pathPnt = pnts1.Value(ipass);
                                        currentLine.AddStatusLast(true, ipass, pathPnt);
                                        AddPointInCurrentLine(ipass, pathPnt, currentLine);
                                    }
                                    else
                                    {
                                        currentLine.AddStatusLast(false);
                                        PntOn2S newP = new();
                                        newP.SetValue(func.Point(), _reversed, uvap[1], uvap[2]);
                                        currentLine.AddPoint(newP);
                                        rajout = true;
                                        _seqAlone.Append(_lines.Length() + 1);
                                        _seqAjout.Append(_lines.Length() + 1);
                                    }
                                }
                            }
                            else if (status == StatusDeflection.OK)
                            {
                                if (ipass != 0)
                                {
                                    currentLine.AddIndexPassing(ipass);
                                }
                                _previousPoint.SetValue(func.Point(), _reversed, uvap[1], uvap[2]);
                                _previousd3d = func.Direction3d();
                                _previousd2d = func.Direction2d();
                                currentLine.AddPoint(_previousPoint);
                            }
                            else if (status == StatusDeflection.PointConfondu)
                            {
                                nbIter--;
                            }
                        }
                    }
                    else
                    { // no numerical solution NotDone
                        pasC = pasC / 2.0;
                        pasCu = Math.Abs(pasC * _previousd2d.X);
                        pasCv = Math.Abs(pasC * _previousd2d.Y);

                        if (pasCu <= _tolerance[1] && pasCv <= _tolerance[2])
                        {
                            if (currentLine.PointCount == 1)
                            {
                                RemoveTwoEndPoints(i);
                                break; // cancel the line
                            }
                            if (_wd2[i].etat > 12)
                            { // the line should become open
                                _wd2[i].etat = 12; // declare it open
                                arretAjout = false;
                                OpenLine(0, psol, pnts1, func, currentLine);
                                stepSign = -1;
                                statusPrecedent = StatusDeflection.OK;
                                arrive = false;
                                pasC = pasSav;
                                nbBadRootIter = 0;
                                rajout = true;
                                _seqAlone.Append(-_lines.Length() - 1);
                                _seqAjout.Append(-_lines.Length() - 1);
                            }
                            else
                            { // line s is open
                                arrive = true;
                                currentLine.AddStatusLast(false);
                                tgtend = true;
                                rajout = true;
                                _seqAlone.Append(_lines.Length() + 1);
                                _seqAjout.Append(_lines.Length() + 1);
                            }
                        }
                    }

                    if (nbIter < 0)
                    {
                        break;
                    }
                } // end of started line
                if (arrive)
                {
                    currentLine.SetTangencyAtBegining(tgtbeg);
                    currentLine.SetTangencyAtEnd(tgtend);

                    _lines.Append(currentLine);
                    _wd2[i].etat = -_wd2[i].etat; // mark point as processed
                }
            } // end of processing of start point
        } // end of all start points
    }

    // IntWalk_IWalking_5.gxx

    /// <summary>
    /// Checks the step of advancement and computes the next one: confused points, 3D and 2D angles too great, tangency,
    /// change of side, then the step from the deflection, adjusted by the previous steps.
    /// </summary>
    protected StatusDeflection TestDeflection(TheIWFunction sp, bool finished, math_Vector uv,
                                                      StatusDeflection statusPrecedent, ref int nbDivision,
                                                      ref double step, int stepSign)
    {
        StatusDeflection status = TestDeflectionImpl(sp, finished, uv, statusPrecedent, ref nbDivision,
                                                              ref step, stepSign);
        if ((_stepTrace ?? s_threadStepTrace) is { } trace)
        {
            PntOn2S point = new();
            point.SetValue(sp.Point(), _reversed, uv[1], uv[2]);
            trace(status, point);
        }
        return status;
    }

    private StatusDeflection TestDeflectionImpl(TheIWFunction sp, bool finished, math_Vector uv,
                                                        StatusDeflection statusPrecedent, ref int nbDivision,
                                                        ref double step, int stepSign)
    {
        // Check the step of advancement, AND recalculate this step :
        //
        // 1) test point confused
        //     if yes other tests are not done
        // 2) test angle 3d too great
        //     if yes divide the step and leave
        //     angle3d = angle ((previous point, calculated point),
        //                       previous tangent)
        // 3) check step of advancement in 2d
        // 4) test point confused
        // 5) test angle 2d too great
        // 6) test point of tangency
        //     if yes leave
        // 7) calculate the tangent by u,v of the section
        // 8) test angle 3d too great
        //     angle3d = angle ((previous point, calculated point),
        //                       new tangent)
        // 9) test angle 2d too great
        // 10) test change of side (pass the tangent point not knowing it)
        // 11) calculate the step of advancement depending on the vector
        // 12) adjust the step depending on the previous steps

        StatusDeflection status = StatusDeflection.OK;

        //---------------------------------------------------------------------------------
        //-- lbr le 4 Avril 95 : it is possible that the status returns points confused
        //-- if epsilon is great enough (1e-11). In this case one loops
        //-- without ever changing the values sent to Rsnld.
        //---------------------------------------------------------------------------------
        double paramu = 0.0, paramv = 0.0;

        if (!_reversed)
        {
            _previousPoint.ParametersOnS2(out paramu, out paramv);
        }
        else
        {
            _previousPoint.ParametersOnS1(out paramu, out paramv);
        }

        double du = uv[1] - paramu;
        double dv = uv[2] - paramv;
        double duv = du * du + dv * dv;

        Vector3D corde = new(_previousPoint.Value(), sp.Point());

        double norme = corde.SquareMagnitude;

        if ((norme <= 4.0 * global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            && ((duv <= global::Kerf.Engine.Foundation.Precision.SquarePConfusion()) || (statusPrecedent != StatusDeflection.OK)))
        { // the square is already taken in the constructor
            status = StatusDeflection.PointConfondu;
            if (statusPrecedent == StatusDeflection.PasTropGrand)
            {
                return StatusDeflection.ArretSurPointPrecedent;
            }
        }
        else
        {
            double cosi = corde * _previousd3d;
            double cosi2 = 0.0;

            if (cosi * stepSign >= 0.0)
            { // angle 3d <= pi/2 !!!!
                double div = _previousd3d.SquareMagnitude * norme;
                if (div == 0)
                {
                    return status;
                }
                cosi2 = cosi * cosi / div;
            }
            if (cosi2 < CosRef3D)
            { // angle 3d too great
                step = step / 2.0;
                double stepU = Math.Abs(step * _previousd2d.X), stepV = Math.Abs(step * _previousd2d.Y);
                if (stepU < _tolerance[1] && stepV < _tolerance[2])
                {
                    status = StatusDeflection.ArretSurPointPrecedent;
                }
                else
                {
                    status = StatusDeflection.PasTropGrand;
                }
                return status;
            }
        }

        double minTolU = 0.1 * Math.Abs(step * _previousd2d.X), minTolV = 0.1 * Math.Abs(step * _previousd2d.Y);
        double tolU = (minTolU > 0.0) ? Math.Min(_tolerance[1], minTolU) : _tolerance[1],
               tolV = (minTolV > 0.0) ? Math.Min(_tolerance[2], minTolV) : _tolerance[2];

        // If aMinTolU==0.0 then (std::abs(Du) < aMinTolU) is equivalent of (std::abs(Du) < 0.0).
        // It is impossible. Therefore, this case should be processed separately.
        // Analogically for aMinTolV.

        if ((Math.Abs(du) < tolU) && (Math.Abs(dv) < tolV))
        {
            // Thin shapes (for which Ulast-Ufirst or/and Vlast-Vfirst is quite small)
            // exists (see bug #25820). In this case, step is quite small too.
            // Nevertheless, it not always means that we mark time. Therefore, Du and Dv
            // must consider step (aMinTolU and aMinTolV parameters).

            return StatusDeflection.ArretSurPointPrecedent; // confused point 2d
        }

        {
            double cosi = stepSign * (du * _previousd2d.X + dv * _previousd2d.Y);

            if (cosi < 0 && status == StatusDeflection.PointConfondu)
            {
                return StatusDeflection.ArretSurPointPrecedent; // leave as step back with confused point
            }

            if (sp.IsTangent())
            {
                return StatusDeflection.ArretSurPoint;
            }

            // if during routing one has subdivided more than  MaxDivision for each
            // previous step, bug on the square; do nothing (experience U4)

            if ((nbDivision < MaxDivision) && (status != StatusDeflection.PointConfondu)
                && (statusPrecedent != StatusDeflection.PointConfondu))
            {
                double cosi2 = cosi * cosi / duv;
                if (cosi2 < CosRef2D || cosi < 0)
                {
                    step = step / 2.0;
                    double stepU = Math.Abs(step * _previousd2d.X), stepV = Math.Abs(step * _previousd2d.Y);

                    if (stepU < _tolerance[1] && stepV < _tolerance[2])
                    {
                        status = StatusDeflection.ArretSurPointPrecedent;
                    }
                    else
                    {
                        status = StatusDeflection.PasTropGrand;
                    }
                    nbDivision = nbDivision + 1;
                    return status;
                }

                cosi = corde * sp.Direction3d();
                cosi2 = cosi * cosi / sp.Direction3d().SquareMagnitude / norme;
                if (cosi2 < CosRef3D)
                { // angle 3d too great
                    step = step / 2.0;
                    double stepU = Math.Abs(step * _previousd2d.X), stepV = Math.Abs(step * _previousd2d.Y);
                    if (stepU < _tolerance[1] && stepV < _tolerance[2])
                    {
                        status = StatusDeflection.ArretSurPoint;
                    }
                    else
                    {
                        status = StatusDeflection.PasTropGrand;
                    }
                    return status;
                }
                cosi = du * sp.Direction2d().X + dv * sp.Direction2d().Y;
                cosi2 = cosi * cosi / duv;
                if (cosi2 < CosRef2D || sp.Direction2d() * _previousd2d < 0)
                {
                    // angle 2d too great or change the side
                    step = step / 2.0;
                    double stepU = Math.Abs(step * _previousd2d.X), stepV = Math.Abs(step * _previousd2d.Y);
                    if (stepU < _tolerance[1] && stepV < _tolerance[2])
                    {
                        status = StatusDeflection.ArretSurPointPrecedent;
                    }
                    else
                    {
                        status = StatusDeflection.PasTropGrand;
                    }
                    return status;
                }
            }
        }

        if (!finished)
        {
            if (status == StatusDeflection.PointConfondu)
            {
                double stepU = Math.Min(Math.Abs(1.5 * du), _pas * (_uMax - _uMin)),
                       stepV = Math.Min(Math.Abs(1.5 * dv), _pas * (_vMax - _vMin));

                double d2dx = Math.Abs(_previousd2d.X);
                double d2dy = Math.Abs(_previousd2d.Y);

                if (d2dx < _tolerance[1])
                {
                    step = stepV / d2dy;
                }
                else if (d2dy < _tolerance[2])
                {
                    step = stepU / d2dx;
                }
                else
                {
                    step = Math.Min(stepU / d2dx, stepV / d2dy);
                }
            }
            else
            {
                //   estimate the current vector.
                //   if vector/2<=current vector<= vector it is considered that the criterion
                //   is observed.
                //   otherwise adjust the step depending on the previous step

                // Direct calculation :
                // POnCurv=(((p1+p2)/2.+(p2+p3)/2.)/2. + ((p2+p3)/2.+(p3+P4)/2.)/2.)/2.
                // either POnCurv = p1/8. + 3.p2/8. + 3.p3/8. + p4/8.
                // Or p2 = p1 + lambda*d1 et p3 = p4 - lambda*d4
                // So POnCurv = (p1 + p4)/2. + 3.*(lambda d1 - lambda d4)/8.
                // Calculate the deviation with (p1+p4)/2. . So it is just necessary to calculate
                // the norm (square) of 3.*lambda (d1 - d4)/8.
                // either the norm of :
                //    3.*(std::sqrt(Norme)/3.)*StepSign*(d1-d4)/8.
                // which produces, taking the square :
                //         Norme * (d1-d4).SquareMagnitude()/64.

                double flecheCourante =
                    (_previousd3d.Normalized().XYZ() - sp.Direction3d().Normalized().XYZ()).SquareModulus() * norme / 64.0;

                //      if (FlecheCourante <= 0.5*fleche) {
                if (flecheCourante <= 0.25 * _fleche * _fleche)
                {
                    double d2dx = Math.Abs(sp.Direction2d().X);
                    double d2dy = Math.Abs(sp.Direction2d().Y);

                    double stepU = Math.Min(Math.Abs(1.5 * du), _pas * (_uMax - _uMin)),
                           stepV = Math.Min(Math.Abs(1.5 * dv), _pas * (_vMax - _vMin));

                    if (d2dx < _tolerance[1])
                    {
                        step = stepV / d2dy;
                    }
                    else if (d2dy < _tolerance[2])
                    {
                        step = stepU / d2dx;
                    }
                    else
                    {
                        step = Math.Min(stepU / d2dx, stepV / d2dy);
                    }
                }
                else
                {
                    // if (FlecheCourante > fleche) {  // step too great
                    if (flecheCourante > _fleche * _fleche)
                    { // step too great
                        step = step / 2.0;
                        double stepU = Math.Abs(step * _previousd2d.X), stepV = Math.Abs(step * _previousd2d.Y);

                        if (stepU < _tolerance[1] && stepV < _tolerance[2])
                        {
                            status = StatusDeflection.ArretSurPointPrecedent;
                        }
                        else
                        {
                            status = StatusDeflection.PasTropGrand;
                        }
                    }
                    else
                    {
                        double d2dx = Math.Abs(sp.Direction2d().X);
                        double d2dy = Math.Abs(sp.Direction2d().Y);

                        double stepU = Math.Min(Math.Abs(1.5 * du), _pas * (_uMax - _uMin)),
                               stepV = Math.Min(Math.Abs(1.5 * dv), _pas * (_vMax - _vMin));

                        if (d2dx < _tolerance[1])
                        {
                            step = Math.Min(step, stepV / d2dy);
                        }
                        else if (d2dy < _tolerance[2])
                        {
                            step = Math.Min(step, stepU / d2dx);
                        }
                        else
                        {
                            step = Math.Min(step, Math.Min(stepU / d2dx, stepV / d2dy));
                        }
                    }
                }
            }
        }
        return status;
    }

    // IntWalk_IWalking_6.gxx

    protected void AddPointInCurrentLine(int n, in PathPoint pathPnt, TheIWLine currentLine)
    {
        PntOn2S psol = new();
        psol.SetValue(PathPointTool.Value3d(pathPnt), _reversed, _wd1[n].ustart, _wd1[n].vstart);
        currentLine.AddPoint(psol);
    }

    /// <summary>
    /// Makes a walking point: Case 1 or 2 from the function's last point; Case 10 + i the same after evaluating the
    /// function at (U, V). Any other case throws Standard_ConstructionError.
    /// </summary>
    protected void MakeWalkingPoint(int Case, double u, double v, TheIWFunction sp, ref PntOn2S psol)
    {
        // Case == 1      : make a WalkinkPoint.
        // Case == 2      : make a WalkinkPoint.
        //                  The computation of the tangency on is done
        // Case == 10 + i : make a WalkinkPoint according to i.
        //                  but F is updated according to U and V
        // Case == other  : the exception Standard_Failure is raised.

        if ((Case == 1) || (Case == 2))
        {
            psol.SetValue(sp.Point(), _reversed, u, v);
        }
        else if (Case == 11 || Case == 12)
        {
            math_Vector uv = new(1, 2);
            math_Vector ff = new(1, 1);
            Matrix dd = new(1, 1, 1, 2);
            uv[1] = u;
            uv[2] = v;
            sp.Values(uv, ff, dd);
            MakeWalkingPoint(Case - 10, u, v, sp, ref psol);
        }
        else
        {
            throw new ConstructionErrorException();
        }
    }

    /// <summary>Opens the line and restarts it in the other direction.</summary>
    protected void OpenLine(int n, in PntOn2S psol, NCollection_Sequence<PathPoint> pnts1,
                            TheIWFunction sp, TheIWLine line)
    {
        PathPoint pathPnt;

        math_Vector uv = new(1, 2);
        math_Vector ff = new(1, 1);
        Matrix dd = new(1, 1, 1, 2);

        _previousPoint = line.Value(1);
        if (!_reversed)
        {
            _previousPoint.ParametersOnS2(out uv[1], out uv[2]);
        }
        else
        {
            _previousPoint.ParametersOnS1(out uv[1], out uv[2]);
        }
        sp.Values(uv, ff, dd);
        _previousd3d = sp.Direction3d();
        _previousd2d = sp.Direction2d();

        if (n > 0)
        { // departure point given at input
            pathPnt = pnts1.Value(n);
            // mark the line as open with a given stop point
            line.AddStatusFirst(false, true, n, pathPnt);

            AddPointInCurrentLine(n, pathPnt, line);
        }
        else
        {
            if (n < 0)
            {
                line.AddPoint(psol);
            }
            line.AddStatusFirst(false, false);
            // mark the line as open without given stop point
        }
        line.Reverse(); // inverser la ligne
        line.SetTangentVector(_previousd3d.Reversed(), line.PointCount);
    }

    protected bool IsValidEndPoint(int indOfPoint, int indOfLine)
    {
        if (_pointLineLine.Count == 0)
        {
            return true;
        }

        NCollection_List<int> list = _pointLineLine.GetValue(indOfPoint);
        NCollection_List<int>.Iterator itl = new(list);
        for (; itl.More(); itl.Next())
        {
            if (itl.Value() == indOfLine)
            {
                list.Remove(itl);
                return true;
            }
        }
        return false;
    }

    protected void RemoveTwoEndPoints(int indOfPoint)
    {
        if (_pointLineLine.ContainsKey(indOfPoint))
        {
            int line1 = _pointLineLine.GetValue(indOfPoint).First();
            int line2 = _pointLineLine.GetValue(indOfPoint).Last();
            for (int iseq = 1; iseq <= _seqAlone.Length(); iseq++)
            {
                if (_seqAlone.Value(iseq) == line1 || _seqAlone.Value(iseq) == line2)
                {
                    _seqAlone.Remove(iseq--);
                }
            }
        }
    }

    protected bool IsPointOnLine(in Point2D p2d, int irang)
    {
        TheIWLine line = _lines.Value(Math.Abs(irang));
        for (int i = 1; i <= line.PointCount; i++)
        {
            Point2D p2d1 = line.Value(i).ValueOnSurface(_reversed);
            if (Math.Abs(p2d1.X - p2d.X) <= _tolerance[1] && Math.Abs(p2d1.Y - p2d.Y) <= _tolerance[2])
            {
                return true;
            }
            if (i < line.PointCount)
            {
                Point2D p2d2 = line.Value(i + 1).ValueOnSurface(_reversed);
                Vector2D pp1 = new(p2d, p2d1);
                Vector2D pp2 = new(p2d, p2d2);
                if (pp1 * pp2 < 0)
                {
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>
    /// True if <paramref name="pOn2S"/> is on one of the lines already computed: it is projected on the nearest
    /// segment of each line, the projection refined with <paramref name="solver"/>, and compared.
    /// </summary>
    protected bool IsPointOnLine(in PntOn2S pOn2S, math_Vector infBounds, math_Vector supBounds,
                                 FunctionSetRoot solver, TheIWFunction func)
    {
        double eps = Epsilon(1.0);
        Point3D p3d = pOn2S.Value();

        for (int lIdx = 1; lIdx <= _lines.Length(); lIdx++)
        {
            LineOn2S l = _lines.Value(lIdx).Line();

            if (l.IsOutBox(p3d))
            {
                continue;
            }

            // Look for the nearest segment
            double uMin = 0.0, vMin = 0.0;
            double minSqDist = RealLast();
            for (int ptIdx = 1; ptIdx < l.PointCount; ptIdx++)
            {
                Point3D p1 = l.Value(ptIdx).Value();
                Point3D p2 = l.Value(ptIdx + 1).Value();

                Xyz p1P = p3d.XYZ() - p1.XYZ();
                Xyz p1P2 = p2.XYZ() - p1.XYZ();

                double sq12 = p1P2.SquareModulus();

                if (sq12 < Constants.Resolution())
                {
                    continue;
                }

                double dp = p1P.Dot(p1P2);

                double sqD = RealLast();
                if (dp < 0.0)
                {
                    continue;
                }
                else if (dp > sq12)
                {
                    continue;
                }
                else
                {
                    sqD = p1P.CrossSquareMagnitude(p1P2) / sq12;
                }

                if (sqD < minSqDist)
                {
                    minSqDist = sqD;

                    double l1 = dp / sq12;
                    double l2 = 1.0 - l1;

                    if (l1 < eps || l2 < eps)
                    {
                        return true;
                    }

                    l.Value(ptIdx).ParametersOnSurface(_reversed, out double u1, out double v1);
                    l.Value(ptIdx + 1).ParametersOnSurface(_reversed, out double u2, out double v2);

                    uMin = l1 * u2 + l2 * u1;
                    vMin = l1 * v2 + l2 * v1;
                }
            }

            if (minSqDist > global::Kerf.Engine.Foundation.Precision.Infinite())
            {
                continue;
            }

            math_Vector vecPrms = new(1, 2);
            vecPrms[1] = uMin;
            vecPrms[2] = vMin;
            solver.Perform(func, vecPrms, infBounds, supBounds);
            if (!solver.IsDone)
            {
                continue;
            }

            solver.Root(vecPrms);

            Point3D pa = func.PSurface().Value(uMin, vMin), pb = func.PSurface().Value(vecPrms[1], vecPrms[2]);
            double sqD1 = pb.SquareDistanceTo(p3d);
            double sqD2 = pa.SquareDistanceTo(pb);

            if (sqD1 < 4.0 * sqD2)
            {
                return true;
            }
        }

        return false;
    }
}
