// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_LocationGuide.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_LocationGuide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.GeomFill_PipeError;

namespace Occt;

/// <summary>
/// The location law of a sweep along a guide: a trihedron law with a guide and, with a section law, a rotation
/// about the tangent that keeps the section in contact with the guide (the "contact" modes of the pipe shell).
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="Set"/>, <see cref="EraseRotation"/>, <see cref="SetCurve"/>,
/// <see cref="SetTrsf"/>, <see cref="SetInterval"/>, <see cref="SetTolerance"/> and <see cref="SetOrigine"/> are
/// public. It keeps the caller's trihedron law and section law, as OCCT keeps the handles, and sets the trihedron
/// law's curve and origin. OCCT's debug-only TraceRevol and the commented-out derivatives of the rotation (D1 and
/// D2 return false with a rotation, as in OCCT) are not ported.
/// </remarks>
public class GeomFill_LocationGuide : GeomFill_LocationLaw
{
    protected NCollection_HArray2<Point2D> myPoles2d;

    private readonly GeomFill_TrihedronWithGuide myLaw;
    private GeomFill_SectionLaw? mySec;
    private CurveAdaptorBase? myCurve;
    private readonly CurveAdaptorBase myGuide;
    private CurveAdaptorBase? myTrimmed;
    private readonly int myNbPts;
    private bool rotation;
    private double OrigParam1;
    private double OrigParam2;
    private double Uf;
    private double Ul;
    private double myFirstS;
    private double myLastS;
    private double ratio;
    private bool WithTrans;
    private Matrix3x3 Trans;
    private readonly math_Vector TolRes;
    private readonly math_Vector Inf;
    private readonly math_Vector Sup;
    private readonly math_Vector X;
    private readonly math_Vector R;
    private GeomFill_PipeError myStatus;

    private static void InGoodPeriod(double Prec, double Period, ref double Current)
    {
        double Diff = Current - Prec;
        int nb = (int)Math.Truncate(Diff / Period);
        Current -= nb * Period;
        Diff = Current - Prec;
        if (Diff > Period / 2)
        {
            Current -= Period;
        }
        else if (Diff < -Period / 2)
        {
            Current += Period;
        }
    }

    public GeomFill_LocationGuide(GeomFill_TrihedronWithGuide Triedre)
    {
        TolRes = new math_Vector(1, 3);
        Inf = new math_Vector(1, 3, 0.0);
        Sup = new math_Vector(1, 3, 0.0);
        X = new math_Vector(1, 3);
        R = new math_Vector(1, 3);
        myStatus = GeomFill_PipeOk;

        TolRes.Init(1.0e-6);
        myLaw = Triedre; // loi de triedre
        mySec = null;    // loi de section
        myCurve = null;
        myFirstS = myLastS = -505e77;

        myNbPts = 21;              // nb points pour les calculs
        CurveAdaptorBase aGuide = myLaw.Guide(); // courbe guide
        if (!aGuide.IsPeriodic)
        {
            double f, l, delta;
            f = aGuide.FirstParameter();
            l = aGuide.LastParameter();
            delta = (l - f) / 100;
            f -= delta;
            l += delta;
            aGuide = aGuide.Trim(f, l, delta * 1.0e-7); // courbe guide
        } // if
        myGuide = aGuide;

        myPoles2d = new NCollection_HArray2<Point2D>(1, 2, 1, myNbPts);
        rotation = false;  // contact ou non
        OrigParam1 = 0;    // param pour ACR quand trajectoire
        OrigParam2 = 1;    // et guide pas meme sens de parcourt
        Trans = new Matrix3x3();
        Trans.SetIdentity();
        WithTrans = false;
    }

    private void SetRotation(double PrecAngle, ref double LastAngle)
    {
        if (myCurve is null)
        {
            throw new ConstructionErrorException("GeomFill_LocationGuide::The path is not set !!");
        }

        // repere fixe
        CoordinateSystem Rep = new CoordinateSystem(Constants.Origin(), Constants.DZ(), Constants.DX());
        Vector3D T = new Vector3D(), N = new Vector3D(), B = new Vector3D();
        int ii, Deg;
        bool isconst, israt = false;
        double t, v, w, OldAngle = 0, Angle, DeltaG, Diff;
        double CurAngle = PrecAngle, a1;
        Point2D p1 = new Point2D(), p2 = new Point2D();
        RevolutionSurface Revol; // surface de revolution
        TrimmedCurve3D S;
        IntCurveSurface_IntersectionPoint PInt = new IntCurveSurface_IntersectionPoint(); // intersection guide/Revol
        NCollection_HArray1<int>? Mult = null;
        NCollection_HArray1<double>? Knots = null, Weights = null;
        NCollection_HArray1<Point3D>? Poles = null;

        double U = 0, UPeriod = 0;
        double f = myCurve.FirstParameter();
        double l = myCurve.LastParameter();
        bool Ok, uperiodic = mySec!.IsUPeriodic();

        DeltaG = (myGuide.LastParameter() - myGuide.FirstParameter()) / 5;
        Curve3D? mySection = null;
        double Tol = 1.0e-9;

        mySec.SectionShape(out int NbPoles, out int NbKnots, out Deg);

        if (mySec.IsConstant(ref Tol))
        {
            mySection = mySec.ConstantSection();
            Uf = mySection.FirstParameter();
            Ul = mySection.LastParameter();

            isconst = true;
        }
        else
        {
            isconst = false;
            israt = mySec.IsRational();
            Mult = new NCollection_HArray1<int>(1, NbKnots);
            mySec.Mults(Mult);
            Knots = new NCollection_HArray1<double>(1, NbKnots);
            mySec.Knots(Knots);
            Poles = new NCollection_HArray1<Point3D>(1, NbPoles);
            Weights = new NCollection_HArray1<double>(1, NbPoles);
            Uf = Knots.Value(1);
            Ul = Knots.Value(NbKnots);
        }

        // Bornes de calculs
        double Delta;
        Delta = myGuide.LastParameter() - myGuide.FirstParameter();
        Inf[1] = myGuide.FirstParameter() - Delta / 10;
        Sup[1] = myGuide.LastParameter() + Delta / 10;

        Inf[2] = -M_PI;
        Sup[2] = 3 * M_PI;

        Delta = Ul - Uf;
        Inf[3] = Uf - Delta / 10;
        Sup[3] = Ul + Delta / 10;

        // JALONNEMENT
        if (uperiodic)
        {
            UPeriod = Ul - Uf;
        }

        for (ii = 1; ii <= myNbPts; ii++)
        {
            t = (double)(myNbPts - ii) * f + (double)(ii - 1) * l;
            t /= myNbPts - 1;
            myCurve.D0(t, out Point3D P);
            Ok = myLaw.D0(t, ref T, ref N, ref B);
            if (!Ok)
            {
                myStatus = myLaw.ErrorStatus();
                return; // Y a rien a faire.
            }
            Direction3D D = new Direction3D(T);
            if (WithTrans)
            {
                Matrix3x3 M = new Matrix3x3(N.XYZ(), B.XYZ(), T.XYZ());
                M *= Trans;
                D = new Direction3D(M.Column(3));
            }
            Axis3D Ax = new Axis3D(P, D); // axe pour la surface de revoltuion

            // calculer transfo entre triedre et Oxyz
            Direction3D N2 = new Direction3D(N);
            CoordinateSystem N3 = new CoordinateSystem(P, D, N2);
            Transform3D Transfo = new Transform3D();
            Transfo.SetTransformation(N3, Rep);

            // transformer la section
            if (!isconst)
            {
                U = myFirstS + (t - myCurve.FirstParameter()) * ratio;
                mySec.D0(U, Poles!, Weights!);

                if (israt)
                {
                    mySection = new BSplineCurve3D(Poles!, Weights!, Knots!, Mult!, Deg, mySec.IsUPeriodic());
                }
                else
                {
                    mySection = new BSplineCurve3D(Poles!, Knots!, Mult!, Deg, mySec.IsUPeriodic());
                }
                S = new TrimmedCurve3D(mySection, Uf, Ul);
            }
            else
            {
                S = new TrimmedCurve3D((Curve3D)mySection!.Copy(), Uf, Ul);
            }
            S.Transform(Transfo);

            // Surface de revolution
            Revol = new RevolutionSurface(S, Ax);

            SurfaceAdaptor GArevol = new SurfaceAdaptor(Revol);
            ExtCS DistMini = new ExtCS(myGuide, GArevol, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion());
            POnCurv Pc = new POnCurv();
            POnSurf Ps;
            double theU = 0.0, theV = 0.0;

            if (!DistMini.IsDone || DistMini.ExtCount == 0)
            {
                bool SOS = false;
                if (ii > 1)
                {
                    // Intersection de secour entre surf revol et guide
                    // equation
                    X[1] = myPoles2d.Value(1, ii - 1).Y();
                    X[2] = myPoles2d.Value(2, ii - 1).X();
                    X[3] = myPoles2d.Value(2, ii - 1).Y();
                    GeomFill_FunctionGuide E = new GeomFill_FunctionGuide(mySec, myGuide, U);
                    E.SetParam(U, P, T.XYZ(), N.XYZ());
                    // resolution   =>  angle
                    FunctionSetRoot Result = new FunctionSetRoot(E, TolRes);
                    Result.Perform(E, X, Inf, Sup);

                    if (Result.IsDone && (Result.FunctionSetErrors().Norm() < TolRes[1] * TolRes[1]))
                    {
                        SOS = true;
                        math_Vector RR = new math_Vector(1, 3);
                        Result.Root(RR);
                        PInt.SetValues(P, RR[2], RR[3], RR[1], IntCurveSurface_TransitionOnCurve.IntCurveSurface_Out);
                        theU = PInt.U();
                        theV = PInt.V();
                    }
                }
                if (!SOS)
                {
                    myStatus = GeomFill_ImpossibleContact;
                    return;
                }
                // OCCT: after the rescue, Pc is still the default point (parameter 0), which gives w below.
            }
            else
            { // on prend le point d'intersection
                // d'angle le plus proche de P
                double MinDist = RealLast();
                int jref = 0;
                for (int j = 1; j <= DistMini.ExtCount; j++)
                {
                    double aDist = DistMini.SquareDistance(j);
                    if (aDist < MinDist)
                    {
                        MinDist = aDist;
                        jref = j;
                    }
                }
                MinDist = Math.Sqrt(MinDist);
                DistMini.Points(jref, out Pc, out Ps);

                Ps.Parameter(out theU, out theV);
                a1 = theU;

                InGoodPeriod(CurAngle, 2 * M_PI, ref a1);
            } // else

            // Controle de w
            w = Pc.Parameter();

            if (ii > 1)
            {
                Diff = w - myPoles2d.Value(1, ii - 1).Y();
                if (Math.Abs(Diff) > DeltaG)
                {
                    if (myGuide.IsPeriodic)
                    {
                        InGoodPeriod(myPoles2d.Value(1, ii - 1).Y(), myGuide.Period(), ref w);
                        Diff = w - myPoles2d.Value(1, ii - 1).Y();
                    }
                }
            }

            // Recadrage de l'angle.
            Angle = theU;

            if (ii > 1)
            {
                Diff = Angle - OldAngle;
                if (Math.Abs(Diff) > M_PI)
                {
                    InGoodPeriod(OldAngle, 2 * M_PI, ref Angle);
                    Diff = Angle - OldAngle;
                }
            }

            // Recadrage du V
            v = theV;

            if (ii > 1)
            {
                if (uperiodic)
                {
                    InGoodPeriod(myPoles2d.Value(2, ii - 1).Y(), UPeriod, ref v);
                }
                Diff = v - myPoles2d.Value(2, ii - 1).Y();
            }

            p1.SetCoord(t, w); // on stocke les parametres
            p2.SetCoord(Angle, v);
            CurAngle = Angle;
            myPoles2d.SetValue(1, ii, p1);
            myPoles2d.SetValue(2, ii, p2);
            OldAngle = Angle;
        }

        LastAngle = CurAngle;
        rotation = true; // C'est pret !
    }

    /// <summary>
    /// Sets the section law (parameters SFirst to SLast along the path) and, if rotat, computes the rotation that
    /// keeps the section in contact with the guide, starting from PrecAngle; LastAngle is the angle at the end.
    /// </summary>
    public void Set(GeomFill_SectionLaw Section, bool rotat, double SFirst, double SLast, double PrecAngle,
                    ref double LastAngle)
    {
        myStatus = GeomFill_PipeOk;
        myFirstS = SFirst;
        myLastS = SLast;
        LastAngle = PrecAngle;
        if (myCurve is null)
        {
            ratio = 0.0;
        }
        else
        {
            ratio = (SLast - SFirst) / (myCurve.LastParameter() - myCurve.FirstParameter());
        }
        mySec = Section;

        if (rotat)
        {
            SetRotation(PrecAngle, ref LastAngle);
        }
        else
        {
            rotation = false;
        }
    }

    /// <summary>Removes the rotation (and an impossible-contact status).</summary>
    public void EraseRotation()
    {
        rotation = false;
        if (myStatus == GeomFill_ImpossibleContact)
        {
            myStatus = GeomFill_PipeOk;
        }
    }

    /// <remarks>
    /// OCCT: the copy has no curve when <see cref="Set"/> computes its rotation, so copying a law with a rotation
    /// throws Standard_ConstructionError; and the start angle it passes is the first path parameter.
    /// </remarks>
    public override GeomFill_LocationLaw Copy()
    {
        double la = 0;
        GeomFill_TrihedronWithGuide L;
        L = (GeomFill_TrihedronWithGuide)myLaw.Copy();
        GeomFill_LocationGuide copy = new GeomFill_LocationGuide(L);
        copy.SetOrigine(OrigParam1, OrigParam2);
        copy.Set(mySec!, rotation, myFirstS, myLastS, myPoles2d.Value(1, 1).X(), ref la);
        copy.SetTrsf(Trans);

        return copy;
    }

    /// <summary>
    /// Sets the path and, with a rotation, computes the poles on the guide (the intersections of the guide with
    /// the surface of revolution of the section at myNbPts points).
    /// </summary>
    public override bool SetCurve(CurveAdaptorBase C)
    {
        double LastAngle = 0;
        myCurve = C;
        myTrimmed = C;

        if (myCurve is not null)
        {
            myLaw.SetCurve(C);
            myLaw.Origine(OrigParam1, OrigParam2);
            myStatus = myLaw.ErrorStatus();

            if (rotation)
            {
                // OCCT: the start angle passed is the first path parameter, not the first angle.
                SetRotation(myPoles2d.Value(1, 1).X(), ref LastAngle);
            }
        }
        return myStatus == GeomFill_PipeOk;
    }

    public override CurveAdaptorBase? GetCurve() => myCurve;

    public override void SetTrsf(in Matrix3x3 Transfo)
    {
        Trans = Transfo;
        Matrix3x3 Aux = new Matrix3x3();
        Aux.SetIdentity();
        Aux -= Trans;
        WithTrans = false; // Au cas ou Trans = I
        for (int ii = 1; ii <= 3 && !WithTrans; ii++)
        {
            for (int jj = 1; jj <= 3 && !WithTrans; jj++)
            {
                if (Math.Abs(Aux.Value(ii, jj)) > 1.0e-14)
                {
                    WithTrans = true;
                }
            }
        }
    }

    public override bool D0(double Param, ref Matrix3x3 M, ref Vector3D V)
    {
        bool Ok;
        Vector3D T = new Vector3D(), N = new Vector3D(), B = new Vector3D();

        myCurve!.D0(Param, out Point3D P);
        V.SetXYZ(P.XYZ());
        Ok = myLaw.D0(Param, ref T, ref N, ref B);
        if (!Ok)
        {
            myStatus = myLaw.ErrorStatus();
            return Ok;
        }
        M.SetCols(N.XYZ(), B.XYZ(), T.XYZ());

        if (WithTrans)
        {
            M *= Trans;
        }

        if (rotation)
        {
            double U = myFirstS + (Param - myCurve.FirstParameter()) * ratio;
            return Rotate(Param, U, P, ref M);
        }

        return true;
    }

    // The rotation of D0 and D0 with poles (OCCT writes it twice): the angle about the tangent that puts the
    // section on the guide, applied to the normal and binormal columns of M.
    private bool Rotate(double Param, double U, in Point3D P, ref Matrix3x3 M)
    {
        // initialisations germe
        InitX(Param);

        int Iter = 100;
        Xyz t, b, n;
        t = M.Column(3);
        b = M.Column(2);
        n = M.Column(1);

        // Intersection entre surf revol et guide
        // equation
        GeomFill_FunctionGuide E = new GeomFill_FunctionGuide(mySec!, myGuide, U);
        E.SetParam(Param, P, t, n);

        // resolution   =>  angle
        FunctionSetRoot Result = new FunctionSetRoot(E, TolRes, Iter);
        Result.Perform(E, X, Inf, Sup);

        if (Result.IsDone)
        {
            // solution
            Result.Root(R);

            // rotation
            Matrix3x3 Rot = new Matrix3x3();
            Rot.SetRotation(t, R[2]);
            b *= Rot;
            n *= Rot;

            M.SetCols(n, b, t);
        }
        else
        {
            myStatus = GeomFill_ImpossibleContact;
            return false;
        }
        return true;
    }

    /// <summary>The location; with a rotation, the intersection (C0) of the surface of revolution and the guide.</summary>
    public override bool D0(double Param, ref Matrix3x3 M, ref Vector3D V, NCollection_Array1<Point2D> Poles2d)
    {
        Vector3D T = new Vector3D(), N = new Vector3D(), B = new Vector3D();
        bool Ok;

        myCurve!.D0(Param, out Point3D P);
        V.SetXYZ(P.XYZ());
        Ok = myLaw.D0(Param, ref T, ref N, ref B);
        if (!Ok)
        {
            myStatus = myLaw.ErrorStatus();
            return Ok;
        }
        M.SetCols(N.XYZ(), B.XYZ(), T.XYZ());

        if (WithTrans)
        {
            M *= Trans;
        }

        if (rotation)
        {
            return Rotate(Param, myFirstS + (Param - myCurve.FirstParameter()) * ratio, P, ref M);
        }

        return true;
    }

    /// <summary>The location and its first derivative; false with a rotation (OCCT's computation is commented out).</summary>
    public override bool D1(double Param, ref Matrix3x3 M, ref Vector3D V, ref Matrix3x3 DM, ref Vector3D DV,
                            NCollection_Array1<Point2D> Poles2d, NCollection_Array1<Vector2D> DPoles2d)
    {
        Vector3D T = new Vector3D(), N = new Vector3D(), B = new Vector3D(), DT = new Vector3D(), DN = new Vector3D(),
               DB = new Vector3D();
        bool Ok;

        myCurve!.D1(Param, out Point3D P, out DV);
        V.SetXYZ(P.XYZ());
        Ok = myLaw.D1(Param, ref T, ref DT, ref N, ref DN, ref B, ref DB);
        if (!Ok)
        {
            myStatus = myLaw.ErrorStatus();
            return Ok;
        }

        M.SetCols(N.XYZ(), B.XYZ(), T.XYZ());
        DM.SetCols(DN.XYZ(), DB.XYZ(), DT.XYZ());

        if (WithTrans)
        {
            M *= Trans;
            DM *= Trans;
        }

        if (rotation)
        {
            return false;
        } // if_rotation

        return true;
    }

    /// <summary>The location and its first two derivatives; false with a rotation (OCCT's computation is commented out).</summary>
    public override bool D2(double Param, ref Matrix3x3 M, ref Vector3D V, ref Matrix3x3 DM, ref Vector3D DV, ref Matrix3x3 D2M,
                            ref Vector3D D2V, NCollection_Array1<Point2D> Poles2d, NCollection_Array1<Vector2D> DPoles2d,
                            NCollection_Array1<Vector2D> D2Poles2d)
    {
        Vector3D T = new Vector3D(), N = new Vector3D(), B = new Vector3D(), DT = new Vector3D(), DN = new Vector3D(),
               DB = new Vector3D(), D2T = new Vector3D(), D2N = new Vector3D(), D2B = new Vector3D();
        bool Ok;

        myCurve!.D2(Param, out Point3D P, out DV, out D2V);
        V.SetXYZ(P.XYZ());
        Ok = myLaw.D2(Param, ref T, ref DT, ref D2T, ref N, ref DN, ref D2N, ref B, ref DB, ref D2B);
        if (!Ok)
        {
            myStatus = myLaw.ErrorStatus();
            return Ok;
        }

        // OCCT: the caller's incoming matrices are multiplied here, before they are set from the trihedron below,
        // so D2 never applies the transformation (D0 and D1 do).
        if (WithTrans)
        {
            M *= Trans;
            DM *= Trans;
            D2M *= Trans;
        }

        if (rotation)
        {
            return false;
        } // if_rotation
        else
        {
            M.SetCols(N.XYZ(), B.XYZ(), T.XYZ());
            DM.SetCols(DN.XYZ(), DB.XYZ(), DT.XYZ());
            D2M.SetCols(D2N.XYZ(), D2B.XYZ(), D2T.XYZ());
        }

        return true;
    }

    public override bool HasFirstRestriction() => false;

    public override bool HasLastRestriction() => false;

    public override int TraceNumber() => 0;

    public override GeomFill_PipeError ErrorStatus() => myStatus;

    public override int NbIntervals(Continuity S)
    {
        int Nb_Sec, Nb_Law;
        Nb_Sec = myTrimmed!.NbIntervals(S);
        Nb_Law = myLaw.NbIntervals(S);

        if (Nb_Sec == 1)
        {
            return Nb_Law;
        }
        else if (Nb_Law == 1)
        {
            return Nb_Sec;
        }

        NCollection_Array1<double> IntC = new(1, Nb_Sec + 1);
        NCollection_Array1<double> IntL = new(1, Nb_Law + 1);
        NCollection_Sequence<double> Inter = new();
        myTrimmed.Intervals(IntC, S);
        myLaw.Intervals(IntL, S);

        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(IntC, IntL, Inter, global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.99);
        return Inter.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        int Nb_Sec, Nb_Law;
        Nb_Sec = myTrimmed!.NbIntervals(S);
        Nb_Law = myLaw.NbIntervals(S);

        if (Nb_Sec == 1)
        {
            myLaw.Intervals(T, S);
            return;
        }
        else if (Nb_Law == 1)
        {
            myTrimmed.Intervals(T, S);
            return;
        }

        NCollection_Array1<double> IntC = new(1, Nb_Sec + 1);
        NCollection_Array1<double> IntL = new(1, Nb_Law + 1);
        NCollection_Sequence<double> Inter = new();
        myTrimmed.Intervals(IntC, S);
        myLaw.Intervals(IntL, S);

        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(IntC, IntL, Inter, global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.99);
        for (int ii = 1; ii <= Inter.Length(); ii++)
        {
            T[ii] = Inter[ii];
        }
    }

    public override void SetInterval(double First, double Last)
    {
        myLaw.SetInterval(First, Last);
        myTrimmed = myCurve!.Trim(First, Last, 0);
    }

    public override void GetInterval(out double First, out double Last)
    {
        First = myTrimmed!.FirstParameter();
        Last = myTrimmed.LastParameter();
    }

    public override void GetDomain(out double First, out double Last)
    {
        First = myCurve!.FirstParameter();
        Last = myCurve.LastParameter();
    }

    public override void SetTolerance(double Tol3d, double Tol2d)
    {
        TolRes[1] = myGuide.Resolution(Tol3d);
        Resolution(1, Tol3d, out TolRes[2], out TolRes[3]);
    }

    public override void Resolution(int Index, double Tol, out double TolU, out double TolV)
    {
        TolU = Tol / 100;
        TolV = Tol / 100;
    }

    /// <summary>1: the trihedrons are normed.</summary>
    public override double GetMaximalNorm() => 1.0;

    public override void GetAverageLaw(ref Matrix3x3 AM, ref Vector3D AV)
    {
        int ii;
        double U, delta;
        Vector3D V = new Vector3D(), V1 = new Vector3D(), V2 = new Vector3D(), V3 = new Vector3D();

        myLaw.GetAverageLaw(ref V1, ref V2, ref V3);
        // OCCT: the columns are (tangent, normal, binormal) here, but (normal, binormal, tangent) in D0, D1 and D2.
        AM.SetCols(V1.XYZ(), V2.XYZ(), V3.XYZ());

        AV.SetCoord(0.0, 0.0, 0.0);
        delta = (myTrimmed!.LastParameter() - myTrimmed.FirstParameter()) / 10;
        U = myTrimmed.FirstParameter();
        // OCCT: myNbPts + 1 (22) points with a tenth of the interval as step, so most lie past its end.
        for (ii = 0; ii <= myNbPts; ii++, U += delta)
        {
            V.SetXYZ(myTrimmed.Value(U).XYZ());
            AV += V;
        }
        AV = AV / (myNbPts + 1);
    }

    /// <summary>A new copy of the constant section of the section law.</summary>
    public Curve3D Section() => mySec!.ConstantSection();

    /// <summary>The guide (trimmed a little beyond the trihedron's guide when that is not periodic).</summary>
    public CurveAdaptorBase Guide() => myGuide;

    public override bool IsRotation(ref double Error) => false;

    public override void Rotation(ref Point3D Center)
    {
        throw new MethodNotImplementedException("GeomFill_LocationGuide::Rotation");
    }

    public override bool IsTranslation(ref double Error) => false;

    // Finds an initial value by interpolation.
    private void InitX(double Param)
    {
        int Ideb = 1, Ifin = myPoles2d.RowLength(), Idemi;
        double Valeur, t1, t2;

        Valeur = myPoles2d.Value(1, Ideb).X();
        if (Param == Valeur)
        {
            Ifin = Ideb + 1;
        }

        Valeur = myPoles2d.Value(1, Ifin).X();
        if (Param == Valeur)
        {
            Ideb = Ifin - 1;
        }

        while (Ideb + 1 != Ifin)
        {
            Idemi = (Ideb + Ifin) / 2;
            Valeur = myPoles2d.Value(1, Idemi).X();
            if (Valeur < Param)
            {
                Ideb = Idemi;
            }
            else
            {
                if (Valeur > Param)
                {
                    Ifin = Idemi;
                }
                else
                {
                    Ideb = Idemi;
                    Ifin = Ideb + 1;
                }
            }
        }

        t1 = myPoles2d.Value(1, Ideb).X();
        t2 = myPoles2d.Value(1, Ifin).X();
        double diff = t2 - t1;

        double W1, W2;
        W1 = myPoles2d.Value(1, Ideb).Coord(2);
        W2 = myPoles2d.Value(1, Ifin).Coord(2);
        Point2D P1 = myPoles2d.Value(2, Ideb);
        Point2D P2 = myPoles2d.Value(2, Ifin);

        if (diff > 1.0e-7)
        {
            double b = (Param - t1) / diff, a = (t2 - Param) / diff;
            X[1] = a * W1 + b * W2;
            X[2] = a * P1.Coord(1) + b * P2.Coord(1); // angle
            X[3] = a * P1.Coord(2) + b * P2.Coord(2); // param isov
        }
        else
        {
            X[1] = (W1 + W2) / 2;
            X[2] = (P1.Coord(1) + P2.Coord(1)) / 2;
            X[3] = (P1.Coord(2) + P2.Coord(2)) / 2;
        }

        if (myGuide.IsPeriodic)
        {
            X[1] = global::Kerf.Engine.Geometry.ElCLib.InPeriod(X[1], myGuide.FirstParameter(), myGuide.LastParameter());
        }
        X[2] = global::Kerf.Engine.Geometry.ElCLib.InPeriod(X[2], 0, 2 * M_PI);
        if (mySec!.IsUPeriodic())
        {
            X[3] = global::Kerf.Engine.Geometry.ElCLib.InPeriod(X[3], Uf, Ul);
        }
    }

    /// <summary>Sets the origins of the curvilinear abscissas, for a multi-edge path (ACR).</summary>
    public void SetOrigine(double Param1, double Param2)
    {
        OrigParam1 = Param1;
        OrigParam2 = Param2;
    }

    /// <summary>The distance from the path to the guide (the current point on it) at myNbPts parameters.</summary>
    public GeomFill_PipeError ComputeAutomaticLaw(out NCollection_HArray1<Point2D> ParAndRad)
    {
        Vector3D T = new Vector3D(), N = new Vector3D(), B = new Vector3D();
        int ii;
        double t;

        GeomFill_PipeError theStatus = GeomFill_PipeOk;

        double f = myCurve!.FirstParameter();
        double l = myCurve.LastParameter();

        ParAndRad = new NCollection_HArray1<Point2D>(1, myNbPts);
        for (ii = 1; ii <= myNbPts; ii++)
        {
            t = (double)(myNbPts - ii) * f + (double)(ii - 1) * l;
            t /= myNbPts - 1;
            myCurve.D0(t, out Point3D P);
            bool Ok = myLaw.D0(t, ref T, ref N, ref B);
            if (!Ok)
            {
                theStatus = myLaw.ErrorStatus();
                return theStatus;
            }
            Point3D PointOnGuide = myLaw.CurrentPointOnGuide();
            double CurWidth = P.Distance(PointOnGuide);

            Point2D aParamWithRadius = new Point2D(t, CurWidth);
            ParAndRad.SetValue(ii, aParamWithRadius);
        }

        return theStatus;
    }
}
