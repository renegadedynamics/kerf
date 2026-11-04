// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CorrectedFrenet.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CorrectedFrenet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Construction.Surfaces.Trihedron;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The "corrected Frenet" trihedron law: the Frenet trihedron turned about the tangent by an angle law that removes
/// its rotation (and its flips at inflexions), so that the trihedron has minimal torsion.
/// </summary>
/// <remarks>
/// A working object (decision 21), as <see cref="TrihedronLaw"/>. <see cref="SetCurve"/> computes the angle
/// law (a <see cref="Composite"/> of one law per C0 interval of the Frenet law).
/// </remarks>
internal class CorrectedFrenet : TrihedronLaw
{
    private readonly Frenet _frenet;
    private LawsFunction? _evolAroundT;
    private LawsFunction? _tLaw;
    private Vector3D _at;
    private Vector3D _an;
    private bool _isFrenet;
    private readonly bool _forEvaluation;
    private NCollection_HArray1<double>? _hArrPoles;
    private NCollection_HArray1<double>? _hArrAngle;
    private NCollection_HArray1<Vector3D>? _hArrTangent;
    private NCollection_HArray1<Vector3D>? _hArrNormal;

    private static double ComputeTorsion(double param, CurveAdaptorBase aCurve)
    {
        double torsion;

        aCurve.D3(param, out Point3D point, out Vector3D dc1, out Vector3D dc2, out Vector3D dc3);
        Vector3D dc1crossDC2 = dc1 ^ dc2;
        double Norm_DC1crossDC2 = dc1crossDC2.Magnitude;

        double dc1DC2DC3 = dc1crossDC2 * dc3; // mixed product

        double tol = Constants.Resolution();
        double SquareNorm_DC1crossDC2 = Norm_DC1crossDC2 * Norm_DC1crossDC2;
        if (SquareNorm_DC1crossDC2 <= tol)
        {
            torsion = 0.0;
        }
        else
        {
            torsion = dc1DC2DC3 / SquareNorm_DC1crossDC2;
        }

        return torsion;
    }

    // Smooths a law: reduces the number of knots.
    private static void Smoothlaw(ref BSpline law, NCollection_HArray1<double> points,
                                  NCollection_HArray1<double> param, double Tol)
    {
        double tol, d;
        int ii, nbk;
        bool b, ok;
        BSpline bs = law.Copy();

        nbk = bs.KnotCount;
        tol = Tol / 10;
        ok = false;

        for (ii = nbk - 1; ii > 1; ii--)
        { // Une premiere passe tolerance serres
            b = bs.RemoveKnot(ii, 0, tol);
            if (b)
            {
                ok = true;
            }
        }

        if (ok)
        { // controle
            tol = 0.0;
            for (ii = 1; ii <= param.Length() && ok; ii++)
            {
                d = Math.Abs(bs.Value(param.Value(ii)) - points.Value(ii));
                if (d > tol)
                {
                    tol = d;
                }
                ok = (tol <= Tol);
            }
            if (ok)
            {
                tol = (Tol - tol) / 2;
            }
            else
            {
                return; // Echec
            }
        }
        else
        {
            tol = Tol / 2;
        }

        if (ok)
        {
            // OCCT: Law now shares BS, so the second pass below changes the returned law even when its control
            // fails (the result then is the second pass's, not the first's).
            law = bs;
        }

        ok = false; // Une deuxieme passe tolerance desserre
        nbk = bs.KnotCount;
        for (ii = nbk - 1; ii > 1; ii--)
        {
            b = bs.RemoveKnot(ii, 0, tol);
            if (b)
            {
                ok = true;
            }
        }

        if (ok)
        { // controle
            tol = 0.0;
            for (ii = 1; ii <= param.Length() && ok; ii++)
            {
                d = Math.Abs(bs.Value(param.Value(ii)) - points.Value(ii));
                if (d > tol)
                {
                    tol = d;
                }
                ok = (tol <= Tol);
            }
        }
        if (ok)
        {
            law = bs;
        }
    }

    private static bool FindPlane(CurveAdaptorBase theC, ref PlaneSurface? p)
    {
        bool found = true;
        NCollection_HArray1<Point3D>? tabP = null;

        switch (theC.GetType())
        {
            case Line:
            {
                found = false;
            }
            break;

            case Circle:
                p = new PlaneSurface(new CoordinateSystem(theC.Circle().Position));
                break;

            case Ellipse:
                p = new PlaneSurface(new CoordinateSystem(theC.Ellipse().Position));
                break;

            case Hyperbola:
                p = new PlaneSurface(new CoordinateSystem(theC.Hyperbola().Position));
                break;

            case Parabola:
                p = new PlaneSurface(new CoordinateSystem(theC.Parabola().Position));
                break;

            case BezierCurve:
            {
                BezierCurve3D gc = theC.Bezier();
                int nbp = gc.PoleCount;
                if (nbp < 2)
                {
                    found = false;
                }
                else if (nbp == 2)
                {
                    found = false;
                }
                else
                {
                    tabP = new NCollection_HArray1<Point3D>(gc.InternalPoles());
                }
            }
            break;

            case BSplineCurve:
            {
                BSplineCurve3D gc = theC.BSpline();
                int nbp = gc.PoleCount;
                if (nbp < 2)
                {
                    found = false;
                }
                else if (nbp == 2)
                {
                    found = false;
                }
                else
                {
                    tabP = new NCollection_HArray1<Point3D>(gc.InternalPoles());
                }
            }
            break;

            default:
            { // On utilise un echantillonage
                int nbp = 15 + theC.NbIntervals(C3);
                double f, l, t, inv;
                int ii;
                f = theC.FirstParameter();
                l = theC.LastParameter();
                inv = 1.0 / (nbp - 1);
                tabP = new NCollection_HArray1<Point3D>(1, nbp);
                for (ii = 1; ii <= nbp; ii++)
                {
                    t = (f * (nbp - ii) + l * (ii - 1));
                    t *= inv;
                    tabP.SetValue(ii, theC.Value(t));
                }
            }
            break;
        }

        if (tabP is not null)
        { // Recherche d'un plan moyen et controle
            global::Kerf.Engine.Geometry.Library.GeomLib.AxeOfInertia(tabP, out Frame3D inertia, out bool issingular);
            if (issingular)
            {
                found = false;
            }
            else
            {
                p = new PlaneSurface(new CoordinateSystem(inertia));
            }
            if (found)
            {
                double a, b, c, d, dist;
                int ii;
                p!.Coefficients(out a, out b, out c, out d);
                for (ii = 1; ii <= tabP.Length() && found; ii++)
                {
                    Xyz xyz = tabP.Value(ii).XYZ();
                    dist = a * xyz.X() + b * xyz.Y() + c * xyz.Z() + d;
                    found = (Math.Abs(dist) <= global::Kerf.Engine.Foundation.Precision.Confusion());
                }
                return found;
            }
        }

        return found;
    }

    public CorrectedFrenet()
    {
        _isFrenet = false;
        _frenet = new Frenet();
        _forEvaluation = false;
    }

    /// <summary>
    /// A corrected Frenet law; ForEvaluation skips the search for a mean plane of the curve (the law is then only
    /// evaluated, not used to sweep).
    /// </summary>
    public CorrectedFrenet(bool forEvaluation)
    {
        _isFrenet = false;
        _frenet = new Frenet();
        _forEvaluation = forEvaluation;
    }

    public override TrihedronLaw Copy()
    {
        // OCCT: the copy is made with the default constructor, so it loses ForEvaluation.
        CorrectedFrenet copy = new CorrectedFrenet();
        if (_curve is not null)
        {
            copy.SetCurve(_curve);
        }
        return copy;
    }

    public override bool SetCurve(CurveAdaptorBase c)
    {
        base.SetCurve(c);
        if (c is not null)
        {
            _frenet.SetCurve(c);

            CurveKind type;
            type = c.GetType();
            switch (type)
            {
                case Circle:
                case Ellipse:
                case Hyperbola:
                case Parabola:
                case Line:
                {
                    // No probleme isFrenet
                    _isFrenet = true;
                    break;
                }
                default:
                {
                    // We have to search singularities
                    _isFrenet = true;
                    Init();
                    break;
                }
            }
        }
        return _isFrenet;
    }

    private void Init()
    {
        _evolAroundT = new Composite();
        int nbI = _frenet.NbIntervals(C0), i;
        NCollection_Array1<double> t = new(1, nbI + 1);
        _frenet.Intervals(t, C0);
        LawsFunction? func = null;
        // OCC78
        NCollection_Sequence<double> seqPoles = new(), seqAngle = new();
        NCollection_Sequence<Vector3D> seqTangent = new(), seqNormal = new();

        Vector3D tangent = new(), normal = new(), bn = new();
        _frenet.D0(_trimmed!.FirstParameter(), ref tangent, ref normal, ref bn);
        int nbStep;
        double startAng = 0, avStep, step;

        nbStep = 10;
        avStep = (_trimmed.LastParameter() - _trimmed.FirstParameter()) / nbStep;
        for (i = 1; i <= nbI; i++)
        {
            nbStep = Math.Max((int)((t[i + 1] - t[i]) / avStep), 3);
            step = (t[i + 1] - t[i]) / nbStep;
            if (!InitInterval(t[i], t[i + 1], step, ref startAng, ref tangent, ref normal, ref _at, ref _an, ref func,
                              seqPoles, seqAngle, seqTangent, seqNormal))
            {
                if (_isFrenet)
                {
                    _isFrenet = false;
                }
            }
            ((Composite)_evolAroundT).ChangeLaws().Append(func!);
        }
        if (_trimmed.IsPeriodic)
        {
            ((Composite)_evolAroundT).SetPeriodic();
        }

        _tLaw = _evolAroundT;
        // OCC78
        int iEnd = seqPoles.Length();
        if (iEnd != 0)
        {
            _hArrPoles = new NCollection_HArray1<double>(1, iEnd);
            _hArrAngle = new NCollection_HArray1<double>(1, iEnd);
            _hArrTangent = new NCollection_HArray1<Vector3D>(1, iEnd);
            _hArrNormal = new NCollection_HArray1<Vector3D>(1, iEnd);
            for (i = 1; i <= iEnd; i++)
            {
                _hArrPoles[i] = seqPoles[i];
                _hArrAngle[i] = seqAngle[i];
                _hArrTangent[i] = seqTangent[i];
                _hArrNormal[i] = seqNormal[i];
            }
        }
    }

    // Computes the angle law on a span.
    /// <summary>The number of points at which SetCurve sampled the curve (for tests; decision 1440).</summary>
    internal int SampleCount => _hArrPoles?.Length() ?? 0;

    /// <summary>
    /// The samples InitInterval takes on an interval at OCCT's steps; beyond them its step is at least the interval's
    /// length divided by this (decision 1440).
    /// </summary>
    internal const int SampleBudget = 2000;

    private bool InitInterval(double first, double last, double step, ref double startAng, ref Vector3D prevTangent,
                              ref Vector3D prevNormal, ref Vector3D aT, ref Vector3D aN, ref LawsFunction? funcInt,
                              NCollection_Sequence<double> seqPoles, NCollection_Sequence<double> seqAngle,
                              NCollection_Sequence<Vector3D> seqTangent, NCollection_Sequence<Vector3D> seqNormal)
    {
        Vector3D tangent = new(), normal = new(), bn = new(), cross;
        NCollection_Sequence<double> parameters = new();
        NCollection_Sequence<double> evolAT = new();
        double param = first;
        bool isZero = true, isConst = true;


        _frenet.SetInterval(first, last); // To have right evaluation at bounds
        SnglrFunc cs = new(_curve!);
        Box curveBoundBox = new();
        Add3dCurve.Add(cs, first, last, 1.0e-2, ref curveBoundBox);
        double lengthMin = curveBoundBox.GetGap() * 1.0e-4;

        aT = new Vector3D(0, 0, 0);
        aN = new Vector3D(0, 0, 0);

        double angleAT = 0.0;
        double currStep = step;

        PlaneSurface? plane = null;
        bool isPlanar = false;
        if (!_forEvaluation)
        {
            isPlanar = FindPlane(_curve!, ref plane);
        }

        int i = 1;
        double currParam = param;
        double dLast = last - global::Kerf.Engine.Foundation.Precision.PConfusion();

        while (param < last)
        {
            if (currParam > dLast)
            {
                if (Math.Abs(dLast - param) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    param = currParam;
                }

                currStep = dLast - param;
                currParam = last;
            }
            if (isPlanar)
            {
                currParam = last;
            }

            _frenet.D0(currParam, ref tangent, ref normal, ref bn);
            if (prevTangent.AngleTo(tangent) < M_PI / 3 || i == 1)
            {
                parameters.Append(currParam);
                // OCC78
                seqPoles.Append(param);
                seqAngle.Append(i > 1 ? evolAT[i - 1] : startAng);
                seqTangent.Append(prevTangent);
                seqNormal.Append(prevNormal);
                angleAT = CalcAngleAT(tangent, normal, prevTangent, prevNormal);

                if (isConst && i > 1)
                {
                    if (Math.Abs(angleAT) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        isConst = false;
                    }
                }

                angleAT += (i > 1) ? evolAT[i - 1] : startAng;
                evolAT.Append(angleAT);
                prevNormal = normal;

                if (isZero)
                {
                    if (Math.Abs(angleAT) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        isZero = false;
                    }
                }

                aT += tangent;
                cross = tangent.Crossed(normal);
                aN = Vector3D.LinearForm(Math.Sin(angleAT), cross, 1 - Math.Cos(angleAT), tangent.Crossed(cross), normal + aN);
                prevTangent = tangent;
                param = currParam;
                i++;

                // Evaluate the Next step
                cs.D1(param, out Point3D ponC, out Vector3D d1);
                double l = Math.Max(ponC.XYZ().Modulus() / 2, lengthMin);
                double norm = d1.Magnitude;
                if (norm < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    norm = global::Kerf.Engine.Foundation.Precision.Confusion();
                }
                currStep = l / norm;
                if (currStep > step)
                {
                    currStep = step; // default value
                }
                if (i > SampleBudget && currStep < (last - first) / SampleBudget)
                {
                    // past the budget of samples, the step is bounded below, where OCCT's could fall near 1e-6 on
                    // offset curves: hundreds of thousands of Frenet evaluations, seconds to minutes (decision 1440).
                    // A turn too sharp still halves it.
                    currStep = (last - first) / SampleBudget;
                }
            }
            else
            {
                currStep /= 2; // Step too long !
            }

            currParam = param + currStep;
        }

        if (!isPlanar)
        {
            aT /= parameters.Length() - 1;
            aN /= parameters.Length() - 1;
        }
        startAng = angleAT;

        // Interpolation
        if (isConst || isPlanar)
        {
            funcInt = new Constant();
            ((Constant)funcInt).Set(angleAT, first, last);
        }

        else
        {
            int length = parameters.Length();
            NCollection_HArray1<double> pararr = new(1, length);
            NCollection_HArray1<double> angleATarr = new(1, length);

            for (int paramIndex = 1; paramIndex <= length; ++paramIndex)
            {
                pararr[paramIndex] = parameters[paramIndex];
                angleATarr[paramIndex] = evolAT[paramIndex];
            }

            LawsInterpolate lawAT = new(angleATarr, pararr, false, global::Kerf.Engine.Foundation.Precision.PConfusion());
            lawAT.Perform();
            // Curve() freezes the interpolated law; smoothlaw works on a copy of it.
            BSpline bs = lawAT.Curve();
            Smoothlaw(ref bs, angleATarr, pararr, 0.1);

            funcInt = new BSpFunc(bs, first, last);
        }
        return isZero;
    }

    // Calculates the angle of rotation of the trihedron's normal relative to the previous one (OCC78).
    private double CalcAngleAT(in Vector3D tangent, in Vector3D normal, in Vector3D prevTangent, in Vector3D prevNormal)
    {
        double angle;
        Vector3D Normal_rot, cross;
        angle = tangent.AngleTo(prevTangent);
        if (Math.Abs(angle) > global::Kerf.Engine.Foundation.Precision.Angular() && Math.Abs(angle) < M_PI - global::Kerf.Engine.Foundation.Precision.Angular())
        {
            cross = tangent.Crossed(prevTangent).Normalized();
            Normal_rot = normal + Math.Sin(angle) * cross.Crossed(normal)
                         + (1 - Math.Cos(angle)) * cross.Crossed(cross.Crossed(normal));
        }
        else
        {
            Normal_rot = normal;
        }
        double angleAT = Normal_rot.AngleTo(prevNormal);
        if (angleAT > global::Kerf.Engine.Foundation.Precision.Angular() && M_PI - angleAT > global::Kerf.Engine.Foundation.Precision.Angular())
        {
            if (Normal_rot.Crossed(prevNormal).IsOpposite(prevTangent, global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                angleAT = -angleAT;
            }
        }
        return angleAT;
    }

    // Conversions of angles (OCC78).
    private static double Corr2PIPI(double ang)
    {
        return ang = (ang < M_PI ? ang : ang - 2 * M_PI);
    }

    private static double DiffAng(double a, double ao)
    {
        double dA = (a - ao) - Math.Floor((a - ao) / 2.0 / M_PI) * 2.0 * M_PI;
        return dA = dA >= 0 ? Corr2PIPI(dA) : -Corr2PIPI(-dA);
    }

    // The angle law at Param, corrected where the interpolated law strays from the local angle (OCC78).
    private double GetAngleAT(double param)
    {
        // Search index of low margin from poles of TLaw by bisection method
        int iB = 1, iE = _hArrPoles!.Length(), iC = (iE + iB) / 2;
        if (param == _hArrPoles.Value(iB))
        {
            return _tLaw!.Value(param);
        }
        if (param > _hArrPoles.Value(iE))
        {
            iC = iE;
        }
        if (iC < iE)
        {
            while (_hArrPoles.Value(iC) > param || param > _hArrPoles.Value(iC + 1))
            {
                if (_hArrPoles.Value(iC) < param)
                {
                    iB = iC;
                }
                else
                {
                    iE = iC;
                }
                iC = (iE + iB) / 2;
            }
            if (_hArrPoles.Value(iC) == param || param == _hArrPoles.Value(iC + 1))
            {
                return _tLaw!.Value(param);
            }
        }
        //  Calculate differentiation between approximated and local values of AngleAT
        double angP = _tLaw!.Value(param), angPo = _hArrAngle!.Value(iC), dAng = angP - angPo;
        Vector3D tangent = new(), normal = new(), bn = new();
        _frenet.D0(param, ref tangent, ref normal, ref bn);
        double DAng = CalcAngleAT(tangent, normal, _hArrTangent!.Value(iC), _hArrNormal!.Value(iC));
        double da = DiffAng(DAng, dAng);
        // The correction (there is core of OCC78 bug)
        if (Math.Abs(da) > M_PI / 2.0)
        {
            angP = angPo + DAng;
        }
        return angP;
    }

    public override bool D0(double param, ref Vector3D tangent, ref Vector3D normal, ref Vector3D biNormal)
    {
        _frenet.D0(param, ref tangent, ref normal, ref biNormal);
        if (_isFrenet)
        {
            return true;
        }

        double angleAT;
        angleAT = GetAngleAT(param); // OCC78

        // rotation around Tangent
        Vector3D cross;
        cross = tangent.Crossed(normal);
        normal = Vector3D.LinearForm(Math.Sin(angleAT), cross, (1 - Math.Cos(angleAT)), tangent.Crossed(cross), normal);
        biNormal = tangent.Crossed(normal);

        return true;
    }

    public override bool D1(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D normal, ref Vector3D dNormal,
                            ref Vector3D biNormal, ref Vector3D dBiNormal)
    {
        _frenet.D1(param, ref tangent, ref dTangent, ref normal, ref dNormal, ref biNormal, ref dBiNormal);
        if (_isFrenet)
        {
            return true;
        }

        double angleAT, d_angleAT;
        double sina, cosa;

        _tLaw!.D1(param, out angleAT, out d_angleAT);
        angleAT = GetAngleAT(param); // OCC78

        Vector3D cross, dcross = new(), tcross, dtcross = new(), aux = new();
        sina = Math.Sin(angleAT);
        cosa = Math.Cos(angleAT);

        cross = tangent.Crossed(normal);
        dcross = Vector3D.LinearForm(1, dTangent.Crossed(normal), tangent.Crossed(dNormal));

        tcross = tangent.Crossed(cross);
        dtcross = Vector3D.LinearForm(1, dTangent.Crossed(cross), tangent.Crossed(dcross));

        aux = Vector3D.LinearForm(sina, dcross, cosa * d_angleAT, cross);
        aux = Vector3D.LinearForm(1 - cosa, dtcross, sina * d_angleAT, tcross, aux);
        dNormal += aux;

        normal = Vector3D.LinearForm(sina, cross, (1 - cosa), tcross, normal);

        biNormal = tangent.Crossed(normal);

        dBiNormal = Vector3D.LinearForm(1, dTangent.Crossed(normal), tangent.Crossed(dNormal));

        return true;
    }

    public override bool D2(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D d2Tangent, ref Vector3D normal,
                            ref Vector3D dNormal, ref Vector3D d2Normal, ref Vector3D biNormal, ref Vector3D dBiNormal,
                            ref Vector3D d2BiNormal)
    {
        _frenet.D2(param, ref tangent, ref dTangent, ref d2Tangent, ref normal, ref dNormal, ref d2Normal, ref biNormal,
                  ref dBiNormal, ref d2BiNormal);
        if (_isFrenet)
        {
            return true;
        }

        double angleAT, d_angleAT, d2_angleAT;
        double sina, cosa;
        _tLaw!.D2(param, out angleAT, out d_angleAT, out d2_angleAT);
        angleAT = GetAngleAT(param); // OCC78

        Vector3D cross, dcross = new(), d2cross = new(), tcross, dtcross = new(), d2tcross = new(), aux = new();
        sina = Math.Sin(angleAT);
        cosa = Math.Cos(angleAT);
        cross = tangent.Crossed(normal);
        dcross = Vector3D.LinearForm(1, dTangent.Crossed(normal), tangent.Crossed(dNormal));
        d2cross = Vector3D.LinearForm(1, d2Tangent.Crossed(normal), 2, dTangent.Crossed(dNormal), tangent.Crossed(d2Normal));

        tcross = tangent.Crossed(cross);
        dtcross = Vector3D.LinearForm(1, dTangent.Crossed(cross), tangent.Crossed(dcross));
        d2tcross = Vector3D.LinearForm(1, d2Tangent.Crossed(cross), 2, dTangent.Crossed(dcross), tangent.Crossed(d2cross));

        aux = Vector3D.LinearForm(sina, d2cross, 2 * cosa * d_angleAT, dcross, cosa * d2_angleAT - sina * d_angleAT * d_angleAT,
                          cross);

        aux = Vector3D.LinearForm(1 - cosa, d2tcross, 2 * sina * d_angleAT, dtcross,
                          cosa * d_angleAT * d_angleAT + sina * d2_angleAT, tcross, aux);
        d2Normal += aux;

        aux = Vector3D.LinearForm(sina, dcross, cosa * d_angleAT, cross);
        aux = Vector3D.LinearForm(1 - cosa, dtcross, sina * d_angleAT, tcross, aux);
        dNormal += aux;

        normal = Vector3D.LinearForm(sina, cross, (1 - cosa), tcross, normal);

        biNormal = tangent.Crossed(normal);

        dBiNormal = Vector3D.LinearForm(1, dTangent.Crossed(normal), tangent.Crossed(dNormal));

        d2BiNormal = Vector3D.LinearForm(1, d2Tangent.Crossed(normal), 2, dTangent.Crossed(dNormal), tangent.Crossed(d2Normal));

        return true;
    }

    public override int NbIntervals(Continuity s)
    {
        int nbFrenet, nbLaw;
        nbFrenet = _frenet.NbIntervals(s);
        if (_isFrenet)
        {
            return nbFrenet;
        }

        nbLaw = _evolAroundT!.NbIntervals(s);
        if (nbFrenet == 1)
        {
            return nbLaw;
        }

        NCollection_Array1<double> frenetInt = new(1, nbFrenet + 1);
        NCollection_Array1<double> lawInt = new(1, nbLaw + 1);
        NCollection_Sequence<double> fusion = new();

        _frenet.Intervals(frenetInt, s);
        _evolAroundT.Intervals(lawInt, s);
        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(frenetInt, lawInt, fusion, global::Kerf.Engine.Foundation.Precision.PConfusion(), true);

        return fusion.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        int nbFrenet, nbLaw;
        if (_isFrenet)
        {
            _frenet.Intervals(t, s);
            return;
        }

        nbFrenet = _frenet.NbIntervals(s);
        if (nbFrenet == 1)
        {
            // OCCT: no return here, so the fusion below overwrites T (with the same values).
            _evolAroundT!.Intervals(t, s);
        }

        nbLaw = _evolAroundT!.NbIntervals(s);

        NCollection_Array1<double> frenetInt = new(1, nbFrenet + 1);
        NCollection_Array1<double> lawInt = new(1, nbLaw + 1);
        NCollection_Sequence<double> fusion = new();

        _frenet.Intervals(frenetInt, s);
        _evolAroundT.Intervals(lawInt, s);
        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(frenetInt, lawInt, fusion, global::Kerf.Engine.Foundation.Precision.PConfusion(), true);

        for (int i = 1; i <= fusion.Length(); i++)
        {
            t[i] = fusion.Value(i);
        }
    }

    public override void SetInterval(double first, double last)
    {
        base.SetInterval(first, last);
        _frenet.SetInterval(first, last);
        if (!_isFrenet)
        {
            _tLaw = _evolAroundT!.Trim(first, last, global::Kerf.Engine.Foundation.Precision.PConfusion() / 2);
        }
    }

    /// <summary>
    /// The trihedron mode best suited to the curve: Frenet if no angle law was needed, DiscreteTrihedron if the
    /// torsion or the angle law turns too sharply, else CorrectedFrenet.
    /// </summary>
    public Trihedron EvaluateBestMode()
    {
        if (_evolAroundT is null)
        {
            return Trihedron.IsFrenet; // Frenet
        }

        const double maxAngle = 3.0 * M_PI / 4.0;
        const double maxTorsion = 100.0;

        double step, u, v, tmin, tmax;
        int nbInt, i, j, k = 1;
        nbInt = _evolAroundT.NbIntervals(CN);
        NCollection_Array1<double> Int = new(1, nbInt + 1);
        _evolAroundT.Intervals(Int, CN);
        Point2D old = new();
        Vector2D vec = new(), prevVec = new();

        int nbSamples = 10;
        for (i = 1; i <= nbInt; i++)
        {
            tmin = Int[i];
            tmax = Int[i + 1];
            double torsion = ComputeTorsion(tmin, _trimmed!);
            if (Math.Abs(torsion) > maxTorsion)
            {
                return Trihedron.IsDiscreteTrihedron; // DiscreteTrihedron
            }

            LawsFunction trimmedlaw = _evolAroundT.Trim(tmin, tmax, global::Kerf.Engine.Foundation.Precision.PConfusion() / 2);
            step = (Int[i + 1] - Int[i]) / nbSamples;
            for (j = 0; j <= nbSamples; j++)
            {
                u = tmin + j * step;
                v = trimmedlaw.Value(u);
                Point2D point2d = new(u, v);
                if (j != 0)
                {
                    vec = vec.WithXY(point2d.XY() - old.XY());
                    if (k > 2)
                    {
                        double theAngle = prevVec.AngleTo(vec);
                        if (Math.Abs(theAngle) > maxAngle)
                        {
                            return Trihedron.IsDiscreteTrihedron; // DiscreteTrihedron
                        }
                    }
                    prevVec = vec;
                }
                old = point2d;
                k++;
            }
        }

        return Trihedron.IsCorrectedFrenet; // CorrectedFrenet
    }

    public override void GetAverageLaw(ref Vector3D aTangent, ref Vector3D aNormal, ref Vector3D aBiNormal)
    {
        if (_isFrenet)
        {
            _frenet.GetAverageLaw(ref aTangent, ref aNormal, ref aBiNormal);
        }
        else
        {
            aTangent = _at;
            aNormal = _an;
            aBiNormal = aTangent;
            aBiNormal = aBiNormal.Crossed(aNormal);
        }
    }

    public override bool IsConstant => (_curve!.GetType() == Line);

    public override bool IsOnlyBy3dCurve => true;
}
