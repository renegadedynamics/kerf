// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_LocationDraft.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_LocationDraft.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// The location law of a draft sweep: a <see cref="GeomFill_DraftTrihedron"/> along the curve and, with a stop
/// surface, the trace of the generatrix on that surface as a 2D restriction.
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="SetCurve"/>, <see cref="SetStopSurf"/>, <see cref="SetAngle"/>,
/// <see cref="SetTrsf"/> and <see cref="SetInterval"/> are public. It keeps the caller's curve and surface
/// adaptors and only evaluates them. The seed search, which OCCT repeats in D0, D1 and D2, is a private helper.
/// </remarks>
public class GeomFill_LocationDraft : GeomFill_LocationLaw
{
    protected NCollection_HArray1<Point2D> myPoles2d;

    private Matrix3x3 Trans;
    private readonly GeomFill_DraftTrihedron myLaw;
    private Adaptor3d_Surface? mySurf;
    private Adaptor3d_Curve? myCurve;
    private Adaptor3d_Curve? myTrimmed;
    private readonly Direction3D myDir;
    private double myAngle;
    private readonly int myNbPts;
    private bool Intersec;
    private bool WithTrans;

    public GeomFill_LocationDraft(in Direction3D Direction, double Angle)
    {
        myDir = Direction; // direction de depouille

        myAngle = Angle; // angle de depouille (teta prime)

        mySurf = null;
        myLaw = new GeomFill_DraftTrihedron(new Vector3D(myDir), Angle); // triedre
        myNbPts = 41;                                                    // nb de points utilises pour les calculs
        myPoles2d = new NCollection_HArray1<Point2D>(1, 2 * myNbPts);
        Intersec = false; // intersection avec surface d'arret ?
        WithTrans = false;
        Trans = new Matrix3x3();
    }

    public override GeomFill_LocationLaw Copy()
    {
        GeomFill_TrihedronLaw law;
        law = myLaw.Copy();
        GeomFill_LocationDraft copy = new GeomFill_LocationDraft(myDir, myAngle);
        copy.SetCurve(myCurve!);
        copy.SetStopSurf(mySurf);
        if (WithTrans)
        {
            copy.SetTrsf(Trans);
        }

        return copy;
    }

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

    /// <summary>
    /// Sets the curve and computes the poles on the stop surface (the intersections of the generatrix with the
    /// surface at myNbPts points of the curve).
    /// </summary>
    public override bool SetCurve(Adaptor3d_Curve C)
    {
        myCurve = C;
        myTrimmed = C;
        bool isOK = myLaw.SetCurve(C);

        Prepare();
        return isOK;
    }

    /// <summary>Sets the stop surface (null for none).</summary>
    public void SetStopSurf(Adaptor3d_Surface? Surf)
    {
        mySurf = Surf;
        Prepare();
    }

    public void SetAngle(double Angle)
    {
        myAngle = Angle;
        myLaw.SetAngle(myAngle);
        Prepare();
    }

    // Lays the milestones of the intersection of the draft with the surface.
    private void Prepare()
    {
        if (mySurf is null)
        {
            Intersec = false;
            return;
        }

        Intersec = true;

        int ii, jj;
        double f, l, t;
        Vector3D D, T = new Vector3D(), N = new Vector3D(), B = new Vector3D();
        Geom_Line L;
        IntCurveSurface_IntersectionPoint P1, P2;
        f = myCurve!.FirstParameter();
        l = myCurve.LastParameter();

        for (ii = 1; ii <= myNbPts; ii++)
        {
            t = (double)(myNbPts - ii) * f + (double)(ii - 1) * l;
            t /= myNbPts - 1;
            myCurve.D0(t, out Point3D P);
            myLaw.D0(t, ref T, ref N, ref B);

            // Generatrice
            D = Math.Cos(myAngle) * B + Math.Sin(myAngle) * N;

            L = new Geom_Line(P, new Direction3D(D));

            IntCurveSurface_HInter Int = new IntCurveSurface_HInter(); // intersection surface / generatrice
            GeomAdaptor_Curve AC = new GeomAdaptor_Curve(L);
            Int.Perform(AC, mySurf); // calcul de l'intersection

            if (Int.NbPoints() > 0) // il y a au moins 1 intersection
            {
                P1 = Int.Point(1); // 1er  point d'intersection

                for (jj = 2; jj <= Int.NbPoints(); jj++)
                {
                    P2 = Int.Point(jj);
                    if (P1.W() > P2.W())
                    {
                        P1 = P2; // point le plus proche
                    }
                } // for_jj

                Point2D p = new Point2D(P1.W(), t);      // point de la courbe
                Point2D q = new Point2D(P1.U(), P1.V()); // point sur la surface
                myPoles2d.SetValue(2 * ii - 1, p);          // point de la courbe (indice impair)
                myPoles2d.SetValue(2 * ii, q);              // point sur la surface (indice pair)
            }
            else
            { // au moins un point ou il n'y a pas intersection
                Intersec = false;
            }
        } // for_ii
    }

    public override Adaptor3d_Curve? GetCurve() => myCurve;

    public override bool D0(double Param, ref Matrix3x3 M, ref Vector3D V)
    {
        bool Ok;
        Vector3D T = new Vector3D(), N = new Vector3D(), B = new Vector3D();

        myTrimmed!.D0(Param, out Point3D P);
        V.SetXYZ(P.XYZ());

        Ok = myLaw.D0(Param, ref T, ref N, ref B);
        if (!Ok)
        {
            return Ok;
        }
        M.SetCols(N.XYZ(), B.XYZ(), T.XYZ());

        if (WithTrans)
        {
            M *= Trans;
        }

        return true;
    }

    // The seed of the search: OCCT's block repeated in D0, D1 and D2. Finds the interval of Param among the
    // milestones and interpolates linearly, or takes the milestone; leaves zeros past the last one.
    private void Seed(double Param, out double W0, out double U0, out double V0)
    {
        double t1, t2, Paramt1, t2Param;
        U0 = 0;
        V0 = 0;
        W0 = 0;

        int ii = 1;
        // on recherche l'intervalle auquel appartient Param
        while (ii < 2 * myNbPts && myPoles2d.Value(ii).Coord(2) < Param)
        {
            ii = ii + 2;
        }

        if (ii < 2 * myNbPts && !IsEqual(myPoles2d.Value(ii).Coord(2), Param))
        {
            // interpolation lineaire pour initialiser le germe de la recherche
            // OCCT: for Param before the first milestone, ii is 1 and OCCT reads the poles at -1 and 0 out of
            // bounds; the port's array throws.
            t1 = myPoles2d.Value(ii).Coord(2);
            t2 = myPoles2d.Value(ii - 2).Coord(2);

            Paramt1 = (Param - t1) / (t2 - t1);
            t2Param = (t2 - Param) / (t2 - t1);

            W0 = myPoles2d.Value(ii - 2).Coord(1) * Paramt1 + myPoles2d.Value(ii).Coord(1) * t2Param;
            U0 = myPoles2d.Value(ii - 1).Coord(1) * Paramt1 + myPoles2d.Value(ii + 1).Coord(1) * t2Param;
            V0 = myPoles2d.Value(ii - 1).Coord(2) * Paramt1 + myPoles2d.Value(ii + 1).Coord(2) * t2Param;
        } // if

        // on est sur un param ou les points ont deja ete calcules
        else if (ii < 2 * myNbPts && IsEqual(myPoles2d.Value(ii).Coord(2), Param))
        {
            W0 = myPoles2d.Value(ii).Coord(1);
            U0 = myPoles2d.Value(ii + 1).Coord(1);
            V0 = myPoles2d.Value(ii + 1).Coord(2);
        } // else if
    }

    /// <summary>The location and the intersection (C0) on the stop surface.</summary>
    public override bool D0(double Param, ref Matrix3x3 M, ref Vector3D V, NCollection_Array1<Point2D> Poles2d)
    {
        bool Ok;
        Vector3D D, T = new Vector3D(), N = new Vector3D(), B = new Vector3D();

        myCurve!.D0(Param, out Point3D P);
        V.SetXYZ(P.XYZ());

        Ok = myLaw.D0(Param, ref T, ref N, ref B);
        if (!Ok)
        {
            return Ok;
        }
        M.SetCols(N.XYZ(), B.XYZ(), T.XYZ());

        if (WithTrans)
        {
            M *= Trans;
        }

        if (Intersec)
        {
            // la generatrice intersecte la surface d'arret
            // la generatrice
            D = Math.Cos(myAngle) * B + Math.Sin(myAngle) * N;

            Geom_Line L = new Geom_Line(P, new Direction3D(D));
            GeomAdaptor_Curve G = new GeomAdaptor_Curve(L);

            Seed(Param, out double W0, out double U0, out double V0);

            // recherche de la solution (pt d'intersection generatrice / surface)
            // point initial
            math_Vector X = new math_Vector(1, 3);
            X[1] = W0;
            X[2] = U0;
            X[3] = V0;

            // tolerance sur X
            math_Vector XTol = new math_Vector(1, 3);
            XTol.Init(0.00001);

            // tolerance sur F
            double FTol = 0.0000001;
            int Iter = 100;

            // fonction dont il faut trouver la racine : G(W)-S(U,V)=0
            GeomFill_FunctionDraft E = new GeomFill_FunctionDraft(mySurf!, G);

            // resolution
            NewtonFunctionSetRoot Result = new NewtonFunctionSetRoot(E, XTol, FTol, Iter);
            Result.Perform(E, X);

            if (Result.IsDone)
            {
                math_Vector R = new math_Vector(1, 3);
                Result.Root(R); // solution

                Point2D p = new Point2D(R[2], R[3]);  // point sur la surface
                Point2D q = new Point2D(R[1], Param); // point de la courbe
                Poles2d.SetValue(1, p);
                Poles2d.SetValue(2, q);
            }
            else
            {
                return false;
            }
        } // if_Intersec

        // la generatrice n'intersecte pas la surface d'arret
        return true;
    }

    /// <summary>The location, the intersection (C1) on the stop surface and their first derivatives.</summary>
    public override bool D1(double Param, ref Matrix3x3 M, ref Vector3D V, ref Matrix3x3 DM, ref Vector3D DV,
                            NCollection_Array1<Point2D> Poles2d, NCollection_Array1<Vector2D> DPoles2d)
    {
        bool Ok;
        Vector3D D, T = new Vector3D(), N = new Vector3D(), B = new Vector3D(), DT = new Vector3D(), DN = new Vector3D(),
               DB = new Vector3D();

        myCurve!.D1(Param, out Point3D P, out DV);
        V.SetXYZ(P.XYZ());

        Ok = myLaw.D1(Param, ref T, ref DT, ref N, ref DN, ref B, ref DB);
        if (!Ok)
        {
            return false;
        }

        M.SetCols(N.XYZ(), B.XYZ(), T.XYZ());
        DM.SetCols(DN.XYZ(), DB.XYZ(), DT.XYZ());

        if (WithTrans)
        {
            M *= Trans;
            DM *= Trans;
        }

        if (Intersec)
        { // la generatrice intersecte la surface d'arret
            // la generatrice
            D = Math.Cos(myAngle) * B + Math.Sin(myAngle) * N;

            Geom_Line L = new Geom_Line(P, new Direction3D(D));
            GeomAdaptor_Curve G = new GeomAdaptor_Curve(L);

            // recherche de la solution (pt d'intersection  generatrice / surface)
            Seed(Param, out double W0, out double U0, out double V0);

            // germe
            math_Vector X = new math_Vector(1, 3);
            X[1] = W0;
            X[2] = U0;
            X[3] = V0;

            // tolerance sur X
            math_Vector XTol = new math_Vector(1, 3);
            XTol.Init(0.0001);

            // tolerance sur F
            double FTol = 0.000001;
            int Iter = 100;

            // fonction dont il faut trouver la racine : G(W)-S(U,V)=0
            GeomFill_FunctionDraft E = new GeomFill_FunctionDraft(mySurf!, G);

            // resolution
            NewtonFunctionSetRoot Result = new NewtonFunctionSetRoot(E, XTol, FTol, Iter);
            Result.Perform(E, X);

            if (Result.IsDone)
            {
                math_Vector R = new math_Vector(1, 3);
                Result.Root(R); // solution

                Point2D p = new Point2D(R[2], R[3]);  // point sur la surface
                Point2D q = new Point2D(R[1], Param); // point de la courbe
                Poles2d.SetValue(1, p);
                Poles2d.SetValue(2, q);

                // derivee de la fonction par rapport a Param
                math_Vector DEDT = new math_Vector(1, 3, 0);
                E.DerivT(myTrimmed!, Param, R[1], DN, myAngle, DEDT); // dE/dt => DEDT

                math_Vector DSDT = new math_Vector(1, 3, 0);

                Matrix DEDX = new Matrix(1, 3, 1, 3, 0);
                E.Derivatives(R, DEDX); // dE/dx au point R => DEDX

                // resolution du syst. lin. : DEDX*DSDT = -DEDT
                Gauss Ga = new Gauss(DEDX);
                if (Ga.IsDone)
                {
                    Ga.Solve(DEDT.Opposite(), DSDT);              // resolution du syst. lin.
                    Vector2D dp = new Vector2D(DSDT[2], DSDT[3]); // surface
                    Vector2D dq = new Vector2D(DSDT[1], 1);       //  courbe
                    DPoles2d.SetValue(1, dp);
                    DPoles2d.SetValue(2, dq);
                } // if
            } // if_Result
            else
            { // la generatrice n'intersecte pas la surface d'arret
                return false;
            }
        } // if_Intersec
        return true;
    }

    /// <summary>The location, the intersection (C2) on the stop surface and their first and second derivatives.</summary>
    public override bool D2(double Param, ref Matrix3x3 M, ref Vector3D V, ref Matrix3x3 DM, ref Vector3D DV, ref Matrix3x3 D2M,
                            ref Vector3D D2V, NCollection_Array1<Point2D> Poles2d, NCollection_Array1<Vector2D> DPoles2d,
                            NCollection_Array1<Vector2D> D2Poles2d)
    {
        bool Ok;
        Vector3D D, T = new Vector3D(), N = new Vector3D(), B = new Vector3D(), DT = new Vector3D(), DN = new Vector3D(),
               DB = new Vector3D(), D2T = new Vector3D(), D2N = new Vector3D(), D2B = new Vector3D();

        myCurve!.D2(Param, out Point3D P, out DV, out D2V);
        V.SetXYZ(P.XYZ());

        Ok = myLaw.D2(Param, ref T, ref DT, ref D2T, ref N, ref DN, ref D2N, ref B, ref DB, ref D2B);
        if (!Ok)
        {
            return Ok;
        }

        M.SetCols(N.XYZ(), B.XYZ(), T.XYZ());
        DM.SetCols(DN.XYZ(), DB.XYZ(), DT.XYZ());
        D2M.SetCols(D2N.XYZ(), D2B.XYZ(), D2T.XYZ());

        if (WithTrans)
        {
            M *= Trans;
            DM *= Trans;
            D2M *= Trans;
        }
        if (Intersec)
        { // la generatrice intersecte la surface d'arret
            // la generatrice
            D = Math.Cos(myAngle) * B + Math.Sin(myAngle) * N;

            Geom_Line L = new Geom_Line(P, new Direction3D(D));
            GeomAdaptor_Curve G = new GeomAdaptor_Curve(L);

            Seed(Param, out double W0, out double U0, out double V0);

            // recherche de la solution (pt d'intersection generatrice / surface)
            // germe
            math_Vector X = new math_Vector(1, 3);
            X[1] = W0;
            X[2] = U0;
            X[3] = V0;

            // tolerance sur X
            math_Vector XTol = new math_Vector(1, 3);
            XTol.Init(0.0001);

            // tolerance sur F
            double FTol = 0.000001;
            int Iter = 150;

            // fonction dont il faut trouver la racine : G(W)-S(U,V)=0
            GeomFill_FunctionDraft E = new GeomFill_FunctionDraft(mySurf!, G);

            // resolution
            NewtonFunctionSetRoot Result = new NewtonFunctionSetRoot(E, XTol, FTol, Iter);
            Result.Perform(E, X);

            if (Result.IsDone)
            {
                math_Vector R = new math_Vector(1, 3);
                Result.Root(R); // solution

                // solution
                Point2D p = new Point2D(R[2], R[3]);  // point sur la surface
                Point2D q = new Point2D(R[1], Param); // point de la courbe
                Poles2d.SetValue(1, p);
                Poles2d.SetValue(2, q);

                // premiere derivee de la fonction
                math_Vector DEDT = new math_Vector(1, 3, 0);
                E.DerivT(myTrimmed!, Param, R[1], DN, myAngle, DEDT); // dE/dt => DEDT

                math_Vector DSDT = new math_Vector(1, 3, 0);

                Matrix DEDX = new Matrix(1, 3, 1, 3, 0);
                E.Derivatives(R, DEDX); // dE/dx => DEDX

                // resolution du syst. lin.
                Gauss Ga = new Gauss(DEDX);
                if (Ga.IsDone)
                {
                    Ga.Solve(DEDT.Opposite(), DSDT);
                    Vector2D dp = new Vector2D(DSDT[2], DSDT[3]); // surface
                    Vector2D dq = new Vector2D(DSDT[1], 1);       //  courbe
                    DPoles2d.SetValue(1, dp);
                    DPoles2d.SetValue(2, dq);
                } // if

                // deuxieme derivee
                GeomFill_Tensor D2EDX2 = new GeomFill_Tensor(3, 3, 3);
                E.Deriv2X(R, D2EDX2); // d2E/dx2

                math_Vector D2EDT2 = new math_Vector(1, 3, 0);

                E.Deriv2T(myTrimmed!, Param, R[1], D2N, myAngle, D2EDT2); // d2E/dt2

                Matrix D2EDTDX = new Matrix(1, 3, 1, 3, 0);
                E.DerivTX(DN, myAngle, D2EDTDX); // d2E/dtdx

                math_Vector D2SDT2 = new math_Vector(1, 3, 0); // d2s/dt2
                Matrix aT = new Matrix(1, 3, 1, 3, 0);
                D2EDX2.Multiply(DSDT, aT);

                // resolution du syst. lin.
                Gauss Ga1 = new Gauss(DEDX);
                if (Ga1.IsDone)
                {
                    Ga1.Solve(-aT * DSDT - 2 * D2EDTDX * DSDT - D2EDT2, D2SDT2);
                    Vector2D d2p = new Vector2D(D2SDT2[2], D2SDT2[3]); // surface
                    Vector2D d2q = new Vector2D(D2SDT2[1], 0);         // courbe
                    D2Poles2d.SetValue(1, d2p);
                    D2Poles2d.SetValue(2, d2q);
                } // if
                else
                { // la generatrice n'intersecte pas la surface d'arret
                    return false;
                }
            } // if_Result
        } // if_Intersec
        return true;
    }

    public override bool HasFirstRestriction() => false;

    public override bool HasLastRestriction() => Intersec;

    public override int TraceNumber()
    {
        if (Intersec)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    public override int NbIntervals(Continuity S) => myLaw.NbIntervals(S);

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        myLaw.Intervals(T, S);
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

    public override void Resolution(int Index, double Tol, out double TolU, out double TolV)
    {
        if (Index == 1)
        {
            TolU = mySurf!.UResolution(Tol);
            TolV = mySurf.VResolution(Tol);
        }
        else
        {
            TolU = Tol;
            TolV = Tol;
        }
    }

    /// <summary>1: the trihedrons are normed.</summary>
    public override double GetMaximalNorm() => 1.0;

    public override void GetAverageLaw(ref Matrix3x3 AM, ref Vector3D AV)
    {
        int ii;
        double U, delta;
        Vector3D V1 = new Vector3D(), V2 = new Vector3D(), V3 = new Vector3D(), V = new Vector3D();

        myLaw.GetAverageLaw(ref V1, ref V2, ref V3);
        // OCCT: the columns are (tangent, normal, binormal) here, but (normal, binormal, tangent) in D0, D1 and D2.
        AM.SetCols(V1.XYZ(), V2.XYZ(), V3.XYZ());

        AV.SetCoord(0.0, 0.0, 0.0);
        delta = (myTrimmed!.LastParameter() - myTrimmed.FirstParameter()) / 10;
        U = myTrimmed.FirstParameter();
        for (ii = 0; ii <= 10; ii++, U += delta)
        {
            V.SetXYZ(myTrimmed.Value(U).XYZ());
            AV += V;
        }
        AV /= 11;
    }

    public override bool IsTranslation(ref double Error) => myLaw.IsConstant();

    public override bool IsRotation(ref double Error)
    {
        CurveKind Type;
        Error = 0;
        Type = myCurve!.GetType();
        if (Type == Circle)
        {
            return myLaw.IsOnlyBy3dCurve();
        }
        return false;
    }

    public override void Rotation(ref Point3D Centre)
    {
        Centre = myCurve!.Circle().Location();
    }

    /// <summary>Whether the generatrix meets the stop surface all along the curve.</summary>
    public bool IsIntersec() => Intersec;

    /// <summary>The draft direction.</summary>
    public Direction3D Direction() => myDir;
}
