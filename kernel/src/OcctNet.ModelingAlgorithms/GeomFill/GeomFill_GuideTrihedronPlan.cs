// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_GuideTrihedronPlan.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_GuideTrihedronPlan.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// A trihedron law with a guide, by planes: the normal points from the path to where the guide crosses the plane
/// normal to the path's (Frenet) tangent.
/// </summary>
/// <remarks>
/// As in OCCT, the class has its own <c>myCurve</c> and <c>myTrimmed</c> (hiding the base's, which stay null), so
/// the base's <see cref="GeomFill_TrihedronLaw.GetInterval"/> throws (OCCT dereferences a null handle there;
/// occt-issues.md). <see cref="D2"/> computes only the Frenet trihedron of the path and returns false, as in OCCT.
/// </remarks>
public class GeomFill_GuideTrihedronPlan : GeomFill_TrihedronWithGuide
{
    private new CurveAdaptorBase? myTrimmed;
    private new CurveAdaptorBase? myCurve;
    private readonly NCollection_HArray2<Point2D> Pole;
    private readonly math_Vector X;
    private readonly math_Vector XTol;
    private readonly math_Vector Inf;
    private readonly math_Vector Sup;
    private readonly GeomFill_Frenet frenet;
    private readonly int myNbPts;
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

    public GeomFill_GuideTrihedronPlan(CurveAdaptorBase theGuide)
    {
        X = new math_Vector(1, 1);
        XTol = new math_Vector(1, 1);
        Inf = new math_Vector(1, 1);
        Sup = new math_Vector(1, 1);
        myStatus = GeomFill_PipeError.GeomFill_PipeOk;
        myCurve = null;
        myGuide = theGuide; // guide
        myTrimG = theGuide;
        myNbPts = 20; // nb points pour calculs
        Pole = new NCollection_HArray2<Point2D>(1, 1, 1, myNbPts); // tab pr stocker Pprime (pt sur guide)
        frenet = new GeomFill_Frenet();
        XTol.Init(1.0e-6);
        XTol[1] = myGuide.Resolution(1.0e-6);
    }

    /// <summary>Computes myNbPts points on the guide (its intersections with the normal planes of the path).</summary>
    private void Init()
    {
        myStatus = GeomFill_PipeError.GeomFill_PipeOk;
        Vector3D Tangent = new Vector3D(), Normal = new Vector3D(), BiNormal = new Vector3D();
        int ii;
        double t, DeltaG, w = 0.0;
        double f = myCurve!.FirstParameter();
        double l = myCurve.LastParameter();

        PlaneSurface Plan;
        SurfaceAdaptor Pl;
        IntCurveSurface_IntersectionPoint PInt;
        IntCurveSurface_HInter Int = new IntCurveSurface_HInter();
        frenet.SetCurve(myCurve);
        DeltaG = (myGuide!.LastParameter() - myGuide.FirstParameter()) / 2;

        Inf[1] = myGuide.FirstParameter() - DeltaG;
        Sup[1] = myGuide.LastParameter() + DeltaG;

        if (!myGuide.IsPeriodic)
        {
            myTrimG = myGuide.Trim(myGuide.FirstParameter() - DeltaG / 100, myGuide.LastParameter() + DeltaG / 100,
                                   DeltaG * 1.0e-7);
        }
        else
        {
            myTrimG = myGuide;
        }

        DeltaG /= 3;
        for (ii = 1; ii <= myNbPts; ii++)
        {
            t = (double)(myNbPts - ii) * f + (double)(ii - 1) * l;
            t /= (myNbPts - 1);
            myCurve.D0(t, out Point3D P);
            frenet.D0(t, ref Tangent, ref Normal, ref BiNormal);
            Plan = new PlaneSurface(P, new Direction3D(Tangent));
            Pl = new SurfaceAdaptor(Plan);

            Int.Perform(myTrimG, Pl); // intersection plan / guide
            if (Int.NbPoints() == 0)
            {
                w = (Math.Abs(myGuide.LastParameter() - w) > Math.Abs(myGuide.FirstParameter() - w)
                         ? myGuide.FirstParameter()
                         : myGuide.LastParameter());

                myStatus = GeomFill_PipeError.GeomFill_PlaneNotIntersectGuide;
            }
            else
            {
                Point3D Pmin;
                PInt = Int.Point(1);
                Pmin = PInt.Pnt();
                double Dmin = P.Distance(Pmin);
                for (int jj = 2; jj <= Int.NbPoints(); jj++)
                {
                    Pmin = Int.Point(jj).Pnt();
                    if (P.Distance(Pmin) < Dmin)
                    {
                        PInt = Int.Point(jj);
                        Dmin = P.Distance(Pmin);
                    }
                } // for_jj

                w = PInt.W();
            }
            if (ii > 1)
            {
                double Diff = w - Pole.Value(1, ii - 1).Y();
                if (Math.Abs(Diff) > DeltaG)
                {
                    if (myGuide.IsPeriodic)
                    {
                        InGoodPeriod(Pole.Value(1, ii - 1).Y(), myGuide.Period(), ref w);
                    }
                }
            }

            Point2D p1 = new Point2D(t, w); // on stocke les parametres
            Pole.SetValue(1, ii, p1);
        } // for_ii
    }

    public override bool SetCurve(CurveAdaptorBase C)
    {
        myCurve = C;
        if (myCurve is not null)
        {
            Init();
        }
        return true;
    }

    public override CurveAdaptorBase Guide() => myGuide!;

    public override bool D0(double Param, ref Vector3D Tangent, ref Vector3D Normal, ref Vector3D BiNormal)
    {
        Point3D Pprime;
        myCurve!.D0(Param, out Point3D P);
        frenet.D0(Param, ref Tangent, ref Normal, ref BiNormal);

        // initialisation de la recherche
        InitX(Param);

        int Iter = 50;

        // fonction dont il faut trouver la racine : G(W)-Pl(U,V)=0
        GeomFill_PlanFunc E = new GeomFill_PlanFunc(P, Tangent, myGuide!);

        // resolution
        FunctionRoot Result = new FunctionRoot(E, X[1], XTol[1], Inf[1], Sup[1], Iter);

        if (Result.IsDone)
        {
            double Res = Result.Root();
            Pprime = myTrimG!.Value(Res); // pt sur courbe guide
            Vector3D n = new Vector3D(P, Pprime); // vecteur definissant la normale du triedre

            Normal = n.Normalized();
            BiNormal = Tangent.Crossed(Normal);
            BiNormal.Normalize();
        }
        else
        { // Erreur...
            myStatus = GeomFill_PipeError.GeomFill_PlaneNotIntersectGuide;
            return false;
        }

        return true;
    }

    public override bool D1(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D Normal, ref Vector3D DNormal,
                            ref Vector3D BiNormal, ref Vector3D DBiNormal)
    {
        // triedre de frenet sur la trajectoire
        myCurve!.D1(Param, out Point3D P, out Vector3D To);
        frenet.D1(Param, ref Tangent, ref DTangent, ref Normal, ref DNormal, ref BiNormal, ref DBiNormal);

        // tolerance sur E
        int Iter = 50;

        // fonction dont il faut trouver la racine : G(W)-Pl(U,V)=0
        InitX(Param);
        GeomFill_PlanFunc E = new GeomFill_PlanFunc(P, Tangent, myGuide!);

        // resolution
        FunctionRoot Result = new FunctionRoot(E, X[1], XTol[1], Inf[1], Sup[1], Iter);

        if (Result.IsDone)
        {
            double Res = Result.Root();
            myTrimG!.D1(Res, out Point3D PG, out Vector3D TG);
            Vector3D n = new Vector3D(P, PG), dn = new Vector3D(); // vecteur definissant la normale du triedre
            double Norm = n.Magnitude();
            if (Norm < 1.0e-12)
            {
                Norm = 1.0;
            }
            n /= Norm;

            Normal = n;
            BiNormal = Tangent.Crossed(Normal);

            // derivee premiere du triedre
            double dtg_dt;
            E.Derivative(Res, out double dedx);
            E.DEDT(Res, To, DTangent, out double dedt);
            dtg_dt = -dedt / dedx;

            dn.SetLinearForm(dtg_dt, TG, -1, To);
            DNormal.SetLinearForm(-(n * dn), n, dn);
            DNormal /= Norm;
            DBiNormal.SetLinearForm(Tangent.Crossed(DNormal), DTangent.Crossed(Normal));
        }
        else
        { // Erreur...
            myStatus = GeomFill_PipeError.GeomFill_PlaneNotIntersectGuide;
            return false;
        }

        return true;
    }

    /// <summary>Computes the Frenet trihedron of the path and its derivatives, and returns false (as OCCT).</summary>
    public override bool D2(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D D2Tangent, ref Vector3D Normal,
                            ref Vector3D DNormal, ref Vector3D D2Normal, ref Vector3D BiNormal, ref Vector3D DBiNormal,
                            ref Vector3D D2BiNormal)
    {
        myCurve!.D2(Param, out _, out _, out _);
        // triedre de Frenet sur la trajectoire
        frenet.D2(Param, ref Tangent, ref DTangent, ref D2Tangent, ref Normal, ref DNormal, ref D2Normal, ref BiNormal,
                  ref DBiNormal, ref D2BiNormal);
        return false;
    }

    public override GeomFill_TrihedronLaw Copy()
    {
        GeomFill_GuideTrihedronPlan copy = new GeomFill_GuideTrihedronPlan(myGuide!);
        // OCCT sets the curve even when it is null
        copy.SetCurve(myCurve!);
        return copy;
    }

    public override GeomFill_PipeError ErrorStatus() => myStatus;

    private static Continuity CurveShape(Continuity S)
    {
        switch (S)
        {
            case C0:
                return C1;
            case C1:
                return C2;
            case C2:
                return C3;
            default:
                return CN;
        }
    }

    /// <summary>The intervals of the path (a provisional version in OCCT: the guide is not taken into account).</summary>
    public override int NbIntervals(Continuity S) => myCurve!.NbIntervals(CurveShape(S));

    public override void Intervals(NCollection_Array1<double> TT, Continuity S)
    {
        myCurve!.Intervals(TT, CurveShape(S));
    }

    public override void SetInterval(double First, double Last)
    {
        myTrimmed = myCurve!.Trim(First, Last, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    public override void GetAverageLaw(ref Vector3D ATangent, ref Vector3D ANormal, ref Vector3D ABiNormal)
    {
        int ii;
        double t, Delta = (myCurve!.LastParameter() - myCurve.FirstParameter()) / 20.001;

        ATangent.SetCoord(0.0, 0.0, 0.0);
        ANormal.SetCoord(0.0, 0.0, 0.0);
        ABiNormal.SetCoord(0.0, 0.0, 0.0);
        Vector3D T = new Vector3D(), N = new Vector3D(), B = new Vector3D();

        for (ii = 1; ii <= 20; ii++)
        {
            t = myCurve.FirstParameter() + (ii - 1) * Delta;
            D0(t, ref T, ref N, ref B);
            ATangent += T;
            ANormal += N;
            ABiNormal += B;
        }
        ATangent /= 20;
        ANormal /= 20;
        ABiNormal /= 20;
    }

    public override bool IsConstant()
    {
        if ((myCurve!.GetType() == Line) && (myGuide!.GetType() == Line))
        {
            double Angle;
            Angle = myCurve.Line().Angle(myGuide.Line());
            if ((Angle < 1.0e-12) || ((2 * M_PI - Angle) < 1.0e-12))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>Returns false.</summary>
    public override bool IsOnlyBy3dCurve() => false;

    /// <summary>Does nothing.</summary>
    public override void Origine(double OrACR1, double OrACR2)
    {
    }

    /// <summary>Interpolates a starting value for the root search.</summary>
    private void InitX(double Param)
    {
        int Ideb = 1, Ifin = Pole.RowLength(), Idemi;
        double Valeur, t1, t2;

        Valeur = Pole.Value(1, Ideb).X();
        if (Param == Valeur)
        {
            Ifin = Ideb + 1;
        }

        Valeur = Pole.Value(1, Ifin).X();
        if (Param == Valeur)
        {
            Ideb = Ifin - 1;
        }

        while (Ideb + 1 != Ifin)
        {
            Idemi = (Ideb + Ifin) / 2;
            Valeur = Pole.Value(1, Idemi).X();
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

        t1 = Pole.Value(1, Ideb).X();
        t2 = Pole.Value(1, Ifin).X();
        double diff = t2 - t1;
        if (diff > 1.0e-7)
        {
            double b = (Param - t1) / diff, a = (t2 - Param) / diff;
            X[1] = Pole.Value(1, Ideb).Coord(2) * a + Pole.Value(1, Ifin).Coord(2) * b; // param guide
        }
        else
        {
            X[1] = (Pole.Value(1, Ideb).Coord(2) + Pole.Value(1, Ifin).Coord(2)) / 2;
        }

        if (myGuide!.IsPeriodic)
        {
            X[1] = global::Kerf.Engine.Geometry.ElCLib.InPeriod(X[1], myGuide.FirstParameter(), myGuide.LastParameter());
        }
    }
}
