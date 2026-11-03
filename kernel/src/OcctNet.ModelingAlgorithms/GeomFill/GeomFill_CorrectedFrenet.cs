// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CorrectedFrenet.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CorrectedFrenet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Occt.GeomFill_Trihedron;

namespace Occt;

/// <summary>
/// The "corrected Frenet" trihedron law: the Frenet trihedron turned about the tangent by an angle law that removes
/// its rotation (and its flips at inflexions), so that the trihedron has minimal torsion.
/// </summary>
/// <remarks>
/// A working object (decision 21), as <see cref="GeomFill_TrihedronLaw"/>. <see cref="SetCurve"/> computes the angle
/// law (a <see cref="Law_Composite"/> of one law per C0 interval of the Frenet law).
/// </remarks>
public class GeomFill_CorrectedFrenet : GeomFill_TrihedronLaw
{
    private readonly GeomFill_Frenet frenet;
    private Law_Function? EvolAroundT;
    private Law_Function? TLaw;
    private Vector3D AT;
    private Vector3D AN;
    private bool isFrenet;
    private readonly bool myForEvaluation;
    private NCollection_HArray1<double>? HArrPoles;
    private NCollection_HArray1<double>? HArrAngle;
    private NCollection_HArray1<Vector3D>? HArrTangent;
    private NCollection_HArray1<Vector3D>? HArrNormal;

    private static double ComputeTorsion(double Param, CurveAdaptorBase aCurve)
    {
        double Torsion;

        aCurve.D3(Param, out Point3D aPoint, out Vector3D DC1, out Vector3D DC2, out Vector3D DC3);
        Vector3D DC1crossDC2 = DC1 ^ DC2;
        double Norm_DC1crossDC2 = DC1crossDC2.Magnitude();

        double DC1DC2DC3 = DC1crossDC2 * DC3; // mixed product

        double Tol = Constants.Resolution();
        double SquareNorm_DC1crossDC2 = Norm_DC1crossDC2 * Norm_DC1crossDC2;
        if (SquareNorm_DC1crossDC2 <= Tol)
        {
            Torsion = 0.0;
        }
        else
        {
            Torsion = DC1DC2DC3 / SquareNorm_DC1crossDC2;
        }

        return Torsion;
    }

    // Smooths a law: reduces the number of knots.
    private static void smoothlaw(ref Law_BSpline Law, NCollection_HArray1<double> Points,
                                  NCollection_HArray1<double> Param, double Tol)
    {
        double tol, d;
        int ii, Nbk;
        bool B, Ok;
        Law_BSpline BS = Law.Copy();

        Nbk = BS.NbKnots();
        tol = Tol / 10;
        Ok = false;

        for (ii = Nbk - 1; ii > 1; ii--)
        { // Une premiere passe tolerance serres
            B = BS.RemoveKnot(ii, 0, tol);
            if (B)
            {
                Ok = true;
            }
        }

        if (Ok)
        { // controle
            tol = 0.0;
            for (ii = 1; ii <= Param.Length() && Ok; ii++)
            {
                d = Math.Abs(BS.Value(Param.Value(ii)) - Points.Value(ii));
                if (d > tol)
                {
                    tol = d;
                }
                Ok = (tol <= Tol);
            }
            if (Ok)
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

        if (Ok)
        {
            // OCCT: Law now shares BS, so the second pass below changes the returned law even when its control
            // fails (the result then is the second pass's, not the first's).
            Law = BS;
        }

        Ok = false; // Une deuxieme passe tolerance desserre
        Nbk = BS.NbKnots();
        for (ii = Nbk - 1; ii > 1; ii--)
        {
            B = BS.RemoveKnot(ii, 0, tol);
            if (B)
            {
                Ok = true;
            }
        }

        if (Ok)
        { // controle
            tol = 0.0;
            for (ii = 1; ii <= Param.Length() && Ok; ii++)
            {
                d = Math.Abs(BS.Value(Param.Value(ii)) - Points.Value(ii));
                if (d > tol)
                {
                    tol = d;
                }
                Ok = (tol <= Tol);
            }
        }
        if (Ok)
        {
            Law = BS;
        }
    }

    private static bool FindPlane(CurveAdaptorBase theC, ref PlaneSurface? theP)
    {
        bool found = true;
        NCollection_HArray1<Point3D>? TabP = null;

        switch (theC.GetType())
        {
            case Line:
            {
                found = false;
            }
            break;

            case Circle:
                theP = new PlaneSurface(new CoordinateSystem(theC.Circle().Position()));
                break;

            case Ellipse:
                theP = new PlaneSurface(new CoordinateSystem(theC.Ellipse().Position()));
                break;

            case Hyperbola:
                theP = new PlaneSurface(new CoordinateSystem(theC.Hyperbola().Position()));
                break;

            case Parabola:
                theP = new PlaneSurface(new CoordinateSystem(theC.Parabola().Position()));
                break;

            case BezierCurve:
            {
                BezierCurve3D GC = theC.Bezier();
                int nbp = GC.PoleCount;
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
                    TabP = new NCollection_HArray1<Point3D>(GC.InternalPoles());
                }
            }
            break;

            case BSplineCurve:
            {
                BSplineCurve3D GC = theC.BSpline();
                int nbp = GC.PoleCount;
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
                    TabP = new NCollection_HArray1<Point3D>(GC.InternalPoles());
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
                TabP = new NCollection_HArray1<Point3D>(1, nbp);
                for (ii = 1; ii <= nbp; ii++)
                {
                    t = (f * (nbp - ii) + l * (ii - 1));
                    t *= inv;
                    TabP.SetValue(ii, theC.Value(t));
                }
            }
            break;
        }

        if (TabP is not null)
        { // Recherche d'un plan moyen et controle
            global::Kerf.Engine.Geometry.Library.GeomLib.AxeOfInertia(TabP, out Frame3D inertia, out bool issingular);
            if (issingular)
            {
                found = false;
            }
            else
            {
                theP = new PlaneSurface(new CoordinateSystem(inertia));
            }
            if (found)
            {
                double a, b, c, d, dist;
                int ii;
                theP!.Coefficients(out a, out b, out c, out d);
                for (ii = 1; ii <= TabP.Length() && found; ii++)
                {
                    Xyz xyz = TabP.Value(ii).XYZ();
                    dist = a * xyz.X() + b * xyz.Y() + c * xyz.Z() + d;
                    found = (Math.Abs(dist) <= global::Kerf.Engine.Foundation.Precision.Confusion());
                }
                return found;
            }
        }

        return found;
    }

    public GeomFill_CorrectedFrenet()
    {
        isFrenet = false;
        frenet = new GeomFill_Frenet();
        myForEvaluation = false;
    }

    /// <summary>
    /// A corrected Frenet law; ForEvaluation skips the search for a mean plane of the curve (the law is then only
    /// evaluated, not used to sweep).
    /// </summary>
    public GeomFill_CorrectedFrenet(bool ForEvaluation)
    {
        isFrenet = false;
        frenet = new GeomFill_Frenet();
        myForEvaluation = ForEvaluation;
    }

    public override GeomFill_TrihedronLaw Copy()
    {
        // OCCT: the copy is made with the default constructor, so it loses ForEvaluation.
        GeomFill_CorrectedFrenet copy = new GeomFill_CorrectedFrenet();
        if (myCurve is not null)
        {
            copy.SetCurve(myCurve);
        }
        return copy;
    }

    public override bool SetCurve(CurveAdaptorBase C)
    {
        base.SetCurve(C);
        if (C is not null)
        {
            frenet.SetCurve(C);

            CurveKind type;
            type = C.GetType();
            switch (type)
            {
                case Circle:
                case Ellipse:
                case Hyperbola:
                case Parabola:
                case Line:
                {
                    // No probleme isFrenet
                    isFrenet = true;
                    break;
                }
                default:
                {
                    // We have to search singularities
                    isFrenet = true;
                    Init();
                    break;
                }
            }
        }
        return isFrenet;
    }

    private void Init()
    {
        EvolAroundT = new Law_Composite();
        int NbI = frenet.NbIntervals(C0), i;
        NCollection_Array1<double> T = new(1, NbI + 1);
        frenet.Intervals(T, C0);
        Law_Function? Func = null;
        // OCC78
        NCollection_Sequence<double> SeqPoles = new(), SeqAngle = new();
        NCollection_Sequence<Vector3D> SeqTangent = new(), SeqNormal = new();

        Vector3D Tangent = new(), Normal = new(), BN = new();
        frenet.D0(myTrimmed!.FirstParameter(), ref Tangent, ref Normal, ref BN);
        int NbStep;
        double StartAng = 0, AvStep, Step;

        NbStep = 10;
        AvStep = (myTrimmed.LastParameter() - myTrimmed.FirstParameter()) / NbStep;
        for (i = 1; i <= NbI; i++)
        {
            NbStep = Math.Max((int)((T[i + 1] - T[i]) / AvStep), 3);
            Step = (T[i + 1] - T[i]) / NbStep;
            if (!InitInterval(T[i], T[i + 1], Step, ref StartAng, ref Tangent, ref Normal, ref AT, ref AN, ref Func,
                              SeqPoles, SeqAngle, SeqTangent, SeqNormal))
            {
                if (isFrenet)
                {
                    isFrenet = false;
                }
            }
            ((Law_Composite)EvolAroundT).ChangeLaws().Append(Func!);
        }
        if (myTrimmed.IsPeriodic)
        {
            ((Law_Composite)EvolAroundT).SetPeriodic();
        }

        TLaw = EvolAroundT;
        // OCC78
        int iEnd = SeqPoles.Length();
        if (iEnd != 0)
        {
            HArrPoles = new NCollection_HArray1<double>(1, iEnd);
            HArrAngle = new NCollection_HArray1<double>(1, iEnd);
            HArrTangent = new NCollection_HArray1<Vector3D>(1, iEnd);
            HArrNormal = new NCollection_HArray1<Vector3D>(1, iEnd);
            for (i = 1; i <= iEnd; i++)
            {
                HArrPoles[i] = SeqPoles[i];
                HArrAngle[i] = SeqAngle[i];
                HArrTangent[i] = SeqTangent[i];
                HArrNormal[i] = SeqNormal[i];
            }
        }
    }

    // Computes the angle law on a span.
    private bool InitInterval(double First, double Last, double Step, ref double startAng, ref Vector3D prevTangent,
                              ref Vector3D prevNormal, ref Vector3D aT, ref Vector3D aN, ref Law_Function? FuncInt,
                              NCollection_Sequence<double> SeqPoles, NCollection_Sequence<double> SeqAngle,
                              NCollection_Sequence<Vector3D> SeqTangent, NCollection_Sequence<Vector3D> SeqNormal)
    {
        Vector3D Tangent = new(), Normal = new(), BN = new(), cross;
        NCollection_Sequence<double> parameters = new();
        NCollection_Sequence<double> EvolAT = new();
        double Param = First;
        bool isZero = true, isConst = true;


        frenet.SetInterval(First, Last); // To have right evaluation at bounds
        GeomFill_SnglrFunc CS = new(myCurve!);
        Box aCurveBoundBox = new();
        Add3dCurve.Add(CS, First, Last, 1.0e-2, ref aCurveBoundBox);
        double LengthMin = aCurveBoundBox.GetGap() * 1.0e-4;

        aT = new Vector3D(0, 0, 0);
        aN = new Vector3D(0, 0, 0);

        double angleAT = 0.0;
        double currStep = Step;

        PlaneSurface? aPlane = null;
        bool isPlanar = false;
        if (!myForEvaluation)
        {
            isPlanar = FindPlane(myCurve!, ref aPlane);
        }

        int i = 1;
        double currParam = Param;
        double DLast = Last - global::Kerf.Engine.Foundation.Precision.PConfusion();

        while (Param < Last)
        {
            if (currParam > DLast)
            {
                if (Math.Abs(DLast - Param) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    Param = currParam;
                }

                currStep = DLast - Param;
                currParam = Last;
            }
            if (isPlanar)
            {
                currParam = Last;
            }

            frenet.D0(currParam, ref Tangent, ref Normal, ref BN);
            if (prevTangent.Angle(Tangent) < M_PI / 3 || i == 1)
            {
                parameters.Append(currParam);
                // OCC78
                SeqPoles.Append(Param);
                SeqAngle.Append(i > 1 ? EvolAT[i - 1] : startAng);
                SeqTangent.Append(prevTangent);
                SeqNormal.Append(prevNormal);
                angleAT = CalcAngleAT(Tangent, Normal, prevTangent, prevNormal);

                if (isConst && i > 1)
                {
                    if (Math.Abs(angleAT) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        isConst = false;
                    }
                }

                angleAT += (i > 1) ? EvolAT[i - 1] : startAng;
                EvolAT.Append(angleAT);
                prevNormal = Normal;

                if (isZero)
                {
                    if (Math.Abs(angleAT) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        isZero = false;
                    }
                }

                aT += Tangent;
                cross = Tangent.Crossed(Normal);
                aN.SetLinearForm(Math.Sin(angleAT), cross, 1 - Math.Cos(angleAT), Tangent.Crossed(cross), Normal + aN);
                prevTangent = Tangent;
                Param = currParam;
                i++;

                // Evaluate the Next step
                CS.D1(Param, out Point3D PonC, out Vector3D D1);
                double L = Math.Max(PonC.XYZ().Modulus() / 2, LengthMin);
                double norm = D1.Magnitude();
                if (norm < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    norm = global::Kerf.Engine.Foundation.Precision.Confusion();
                }
                currStep = L / norm;
                if (currStep > Step)
                {
                    currStep = Step; // default value
                }
            }
            else
            {
                currStep /= 2; // Step too long !
            }

            currParam = Param + currStep;
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
            FuncInt = new Law_Constant();
            ((Law_Constant)FuncInt).Set(angleAT, First, Last);
        }

        else
        {
            int Length = parameters.Length();
            NCollection_HArray1<double> pararr = new(1, Length);
            NCollection_HArray1<double> angleATarr = new(1, Length);

            for (int aParamIndex = 1; aParamIndex <= Length; ++aParamIndex)
            {
                pararr[aParamIndex] = parameters[aParamIndex];
                angleATarr[aParamIndex] = EvolAT[aParamIndex];
            }

            Law_Interpolate lawAT = new(angleATarr, pararr, false, global::Kerf.Engine.Foundation.Precision.PConfusion());
            lawAT.Perform();
            // Curve() freezes the interpolated law; smoothlaw works on a copy of it.
            Law_BSpline BS = lawAT.Curve();
            smoothlaw(ref BS, angleATarr, pararr, 0.1);

            FuncInt = new Law_BSpFunc(BS, First, Last);
        }
        return isZero;
    }

    // Calculates the angle of rotation of the trihedron's normal relative to the previous one (OCC78).
    private double CalcAngleAT(in Vector3D Tangent, in Vector3D Normal, in Vector3D prevTangent, in Vector3D prevNormal)
    {
        double angle;
        Vector3D Normal_rot, cross;
        angle = Tangent.Angle(prevTangent);
        if (Math.Abs(angle) > global::Kerf.Engine.Foundation.Precision.Angular() && Math.Abs(angle) < M_PI - global::Kerf.Engine.Foundation.Precision.Angular())
        {
            cross = Tangent.Crossed(prevTangent).Normalized();
            Normal_rot = Normal + Math.Sin(angle) * cross.Crossed(Normal)
                         + (1 - Math.Cos(angle)) * cross.Crossed(cross.Crossed(Normal));
        }
        else
        {
            Normal_rot = Normal;
        }
        double angleAT = Normal_rot.Angle(prevNormal);
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
    private static double corr2PI_PI(double Ang)
    {
        return Ang = (Ang < M_PI ? Ang : Ang - 2 * M_PI);
    }

    private static double diffAng(double A, double Ao)
    {
        double dA = (A - Ao) - Math.Floor((A - Ao) / 2.0 / M_PI) * 2.0 * M_PI;
        return dA = dA >= 0 ? corr2PI_PI(dA) : -corr2PI_PI(-dA);
    }

    // The angle law at Param, corrected where the interpolated law strays from the local angle (OCC78).
    private double GetAngleAT(double Param)
    {
        // Search index of low margin from poles of TLaw by bisection method
        int iB = 1, iE = HArrPoles!.Length(), iC = (iE + iB) / 2;
        if (Param == HArrPoles.Value(iB))
        {
            return TLaw!.Value(Param);
        }
        if (Param > HArrPoles.Value(iE))
        {
            iC = iE;
        }
        if (iC < iE)
        {
            while (HArrPoles.Value(iC) > Param || Param > HArrPoles.Value(iC + 1))
            {
                if (HArrPoles.Value(iC) < Param)
                {
                    iB = iC;
                }
                else
                {
                    iE = iC;
                }
                iC = (iE + iB) / 2;
            }
            if (HArrPoles.Value(iC) == Param || Param == HArrPoles.Value(iC + 1))
            {
                return TLaw!.Value(Param);
            }
        }
        //  Calculate differentiation between approximated and local values of AngleAT
        double AngP = TLaw!.Value(Param), AngPo = HArrAngle!.Value(iC), dAng = AngP - AngPo;
        Vector3D Tangent = new(), Normal = new(), BN = new();
        frenet.D0(Param, ref Tangent, ref Normal, ref BN);
        double DAng = CalcAngleAT(Tangent, Normal, HArrTangent!.Value(iC), HArrNormal!.Value(iC));
        double DA = diffAng(DAng, dAng);
        // The correction (there is core of OCC78 bug)
        if (Math.Abs(DA) > M_PI / 2.0)
        {
            AngP = AngPo + DAng;
        }
        return AngP;
    }

    public override bool D0(double Param, ref Vector3D Tangent, ref Vector3D Normal, ref Vector3D BiNormal)
    {
        frenet.D0(Param, ref Tangent, ref Normal, ref BiNormal);
        if (isFrenet)
        {
            return true;
        }

        double angleAT;
        angleAT = GetAngleAT(Param); // OCC78

        // rotation around Tangent
        Vector3D cross;
        cross = Tangent.Crossed(Normal);
        Normal.SetLinearForm(Math.Sin(angleAT), cross, (1 - Math.Cos(angleAT)), Tangent.Crossed(cross), Normal);
        BiNormal = Tangent.Crossed(Normal);

        return true;
    }

    public override bool D1(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D Normal, ref Vector3D DNormal,
                            ref Vector3D BiNormal, ref Vector3D DBiNormal)
    {
        frenet.D1(Param, ref Tangent, ref DTangent, ref Normal, ref DNormal, ref BiNormal, ref DBiNormal);
        if (isFrenet)
        {
            return true;
        }

        double angleAT, d_angleAT;
        double sina, cosa;

        TLaw!.D1(Param, out angleAT, out d_angleAT);
        angleAT = GetAngleAT(Param); // OCC78

        Vector3D cross, dcross = new(), tcross, dtcross = new(), aux = new();
        sina = Math.Sin(angleAT);
        cosa = Math.Cos(angleAT);

        cross = Tangent.Crossed(Normal);
        dcross.SetLinearForm(1, DTangent.Crossed(Normal), Tangent.Crossed(DNormal));

        tcross = Tangent.Crossed(cross);
        dtcross.SetLinearForm(1, DTangent.Crossed(cross), Tangent.Crossed(dcross));

        aux.SetLinearForm(sina, dcross, cosa * d_angleAT, cross);
        aux.SetLinearForm(1 - cosa, dtcross, sina * d_angleAT, tcross, aux);
        DNormal += aux;

        Normal.SetLinearForm(sina, cross, (1 - cosa), tcross, Normal);

        BiNormal = Tangent.Crossed(Normal);

        DBiNormal.SetLinearForm(1, DTangent.Crossed(Normal), Tangent.Crossed(DNormal));

        return true;
    }

    public override bool D2(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D D2Tangent, ref Vector3D Normal,
                            ref Vector3D DNormal, ref Vector3D D2Normal, ref Vector3D BiNormal, ref Vector3D DBiNormal,
                            ref Vector3D D2BiNormal)
    {
        frenet.D2(Param, ref Tangent, ref DTangent, ref D2Tangent, ref Normal, ref DNormal, ref D2Normal, ref BiNormal,
                  ref DBiNormal, ref D2BiNormal);
        if (isFrenet)
        {
            return true;
        }

        double angleAT, d_angleAT, d2_angleAT;
        double sina, cosa;
        TLaw!.D2(Param, out angleAT, out d_angleAT, out d2_angleAT);
        angleAT = GetAngleAT(Param); // OCC78

        Vector3D cross, dcross = new(), d2cross = new(), tcross, dtcross = new(), d2tcross = new(), aux = new();
        sina = Math.Sin(angleAT);
        cosa = Math.Cos(angleAT);
        cross = Tangent.Crossed(Normal);
        dcross.SetLinearForm(1, DTangent.Crossed(Normal), Tangent.Crossed(DNormal));
        d2cross.SetLinearForm(1, D2Tangent.Crossed(Normal), 2, DTangent.Crossed(DNormal), Tangent.Crossed(D2Normal));

        tcross = Tangent.Crossed(cross);
        dtcross.SetLinearForm(1, DTangent.Crossed(cross), Tangent.Crossed(dcross));
        d2tcross.SetLinearForm(1, D2Tangent.Crossed(cross), 2, DTangent.Crossed(dcross), Tangent.Crossed(d2cross));

        aux.SetLinearForm(sina, d2cross, 2 * cosa * d_angleAT, dcross, cosa * d2_angleAT - sina * d_angleAT * d_angleAT,
                          cross);

        aux.SetLinearForm(1 - cosa, d2tcross, 2 * sina * d_angleAT, dtcross,
                          cosa * d_angleAT * d_angleAT + sina * d2_angleAT, tcross, aux);
        D2Normal += aux;

        aux.SetLinearForm(sina, dcross, cosa * d_angleAT, cross);
        aux.SetLinearForm(1 - cosa, dtcross, sina * d_angleAT, tcross, aux);
        DNormal += aux;

        Normal.SetLinearForm(sina, cross, (1 - cosa), tcross, Normal);

        BiNormal = Tangent.Crossed(Normal);

        DBiNormal.SetLinearForm(1, DTangent.Crossed(Normal), Tangent.Crossed(DNormal));

        D2BiNormal.SetLinearForm(1, D2Tangent.Crossed(Normal), 2, DTangent.Crossed(DNormal), Tangent.Crossed(D2Normal));

        return true;
    }

    public override int NbIntervals(Continuity S)
    {
        int NbFrenet, NbLaw;
        NbFrenet = frenet.NbIntervals(S);
        if (isFrenet)
        {
            return NbFrenet;
        }

        NbLaw = EvolAroundT!.NbIntervals(S);
        if (NbFrenet == 1)
        {
            return NbLaw;
        }

        NCollection_Array1<double> FrenetInt = new(1, NbFrenet + 1);
        NCollection_Array1<double> LawInt = new(1, NbLaw + 1);
        NCollection_Sequence<double> Fusion = new();

        frenet.Intervals(FrenetInt, S);
        EvolAroundT.Intervals(LawInt, S);
        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(FrenetInt, LawInt, Fusion, global::Kerf.Engine.Foundation.Precision.PConfusion(), true);

        return Fusion.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        int NbFrenet, NbLaw;
        if (isFrenet)
        {
            frenet.Intervals(T, S);
            return;
        }

        NbFrenet = frenet.NbIntervals(S);
        if (NbFrenet == 1)
        {
            // OCCT: no return here, so the fusion below overwrites T (with the same values).
            EvolAroundT!.Intervals(T, S);
        }

        NbLaw = EvolAroundT!.NbIntervals(S);

        NCollection_Array1<double> FrenetInt = new(1, NbFrenet + 1);
        NCollection_Array1<double> LawInt = new(1, NbLaw + 1);
        NCollection_Sequence<double> Fusion = new();

        frenet.Intervals(FrenetInt, S);
        EvolAroundT.Intervals(LawInt, S);
        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(FrenetInt, LawInt, Fusion, global::Kerf.Engine.Foundation.Precision.PConfusion(), true);

        for (int i = 1; i <= Fusion.Length(); i++)
        {
            T[i] = Fusion.Value(i);
        }
    }

    public override void SetInterval(double First, double Last)
    {
        base.SetInterval(First, Last);
        frenet.SetInterval(First, Last);
        if (!isFrenet)
        {
            TLaw = EvolAroundT!.Trim(First, Last, global::Kerf.Engine.Foundation.Precision.PConfusion() / 2);
        }
    }

    /// <summary>
    /// The trihedron mode best suited to the curve: Frenet if no angle law was needed, DiscreteTrihedron if the
    /// torsion or the angle law turns too sharply, else CorrectedFrenet.
    /// </summary>
    public GeomFill_Trihedron EvaluateBestMode()
    {
        if (EvolAroundT is null)
        {
            return GeomFill_IsFrenet; // Frenet
        }

        const double MaxAngle = 3.0 * M_PI / 4.0;
        const double MaxTorsion = 100.0;

        double Step, u, v, tmin, tmax;
        int NbInt, i, j, k = 1;
        NbInt = EvolAroundT.NbIntervals(CN);
        NCollection_Array1<double> Int = new(1, NbInt + 1);
        EvolAroundT.Intervals(Int, CN);
        Point2D old = new();
        Vector2D aVec = new(), PrevVec = new();

        int NbSamples = 10;
        for (i = 1; i <= NbInt; i++)
        {
            tmin = Int[i];
            tmax = Int[i + 1];
            double Torsion = ComputeTorsion(tmin, myTrimmed!);
            if (Math.Abs(Torsion) > MaxTorsion)
            {
                return GeomFill_IsDiscreteTrihedron; // DiscreteTrihedron
            }

            Law_Function trimmedlaw = EvolAroundT.Trim(tmin, tmax, global::Kerf.Engine.Foundation.Precision.PConfusion() / 2);
            Step = (Int[i + 1] - Int[i]) / NbSamples;
            for (j = 0; j <= NbSamples; j++)
            {
                u = tmin + j * Step;
                v = trimmedlaw.Value(u);
                Point2D point2d = new(u, v);
                if (j != 0)
                {
                    aVec.SetXY(point2d.XY() - old.XY());
                    if (k > 2)
                    {
                        double theAngle = PrevVec.Angle(aVec);
                        if (Math.Abs(theAngle) > MaxAngle)
                        {
                            return GeomFill_IsDiscreteTrihedron; // DiscreteTrihedron
                        }
                    }
                    PrevVec = aVec;
                }
                old = point2d;
                k++;
            }
        }

        return GeomFill_IsCorrectedFrenet; // CorrectedFrenet
    }

    public override void GetAverageLaw(ref Vector3D ATangent, ref Vector3D ANormal, ref Vector3D ABiNormal)
    {
        if (isFrenet)
        {
            frenet.GetAverageLaw(ref ATangent, ref ANormal, ref ABiNormal);
        }
        else
        {
            ATangent = AT;
            ANormal = AN;
            ABiNormal = ATangent;
            ABiNormal.Cross(ANormal);
        }
    }

    public override bool IsConstant() => (myCurve!.GetType() == Line);

    public override bool IsOnlyBy3dCurve() => true;
}
