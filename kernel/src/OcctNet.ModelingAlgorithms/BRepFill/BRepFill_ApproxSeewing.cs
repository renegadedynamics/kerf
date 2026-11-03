// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ApproxSeewing.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ApproxSeewing.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Approximates a <see cref="BRepFill_MultiLine"/> by a 3D B-spline curve and its two pcurves, through 50 points
/// spread along it (AppDef_Compute), or by polygons when the approximation fails.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT copies the MultiLine; the port shares it, which is the same, because a
/// BRepFill_MultiLine never changes after its construction. The curves handed out are new and unfrozen, as OCCT's.
/// </remarks>
public sealed class BRepFill_ApproxSeewing
{
    private BRepFill_MultiLine myML = new();
    private bool myIsDone;
    private Geom_Curve? myCurve;
    private Geom2d_Curve? myPCurve1;
    private Geom2d_Curve? myPCurve2;

    /// <summary>An empty approximation.</summary>
    public BRepFill_ApproxSeewing()
    {
        myIsDone = false;
    }

    /// <summary>Approximates <paramref name="ML"/>.</summary>
    public BRepFill_ApproxSeewing(BRepFill_MultiLine ML)
    {
        myIsDone = false;
        Perform(ML);
    }

    /// <summary>Approximates <paramref name="ML"/>.</summary>
    public void Perform(BRepFill_MultiLine ML)
    {
        myML = ML;

        // evaluate the approximative length of the 3dCurve
        int i;
        double Length = 0.0;
        double U1 = myML.FirstParameter();
        double U2 = myML.LastParameter();
        int NbPoints = 50;
        double Dist, dU = (U2 - U1) / (2 * NbPoints - 1);

        NCollection_Array1<Point2D> LP = new(1, 2 * NbPoints); // tableau Longueur <-> Param
        Point3D aPnt1, aPnt2;
        aPnt1 = myML.Value(U1);

        for (i = 0; i < 2 * NbPoints; i++)
        {
            aPnt2 = myML.Value(U1 + i * dU);
            Dist = aPnt1.Distance(aPnt2);
            Length += Dist;
            LP[i + 1] = new Point2D(Length, U1 + (i * dU));
            aPnt1 = aPnt2;
        }

        // On cherche a mettre NbPoints dans la curve.
        // on met les points environ a Length/NbPoints.

        AppDef_MultiLine MLS = new(NbPoints);
        AppDef_MultiPointConstraint MP = new(1, 2);
        Point3D P3d;
        Point2D PF1, PF2;

        ML.Value3dOnF1OnF2(U1, out P3d, out PF1, out PF2);
        MP.SetPoint(1, P3d);
        MP.SetPoint2d(2, PF1);
        MP.SetPoint2d(3, PF2);
        MLS.SetValue(1, MP);

        double DCorde = Length / (NbPoints - 1);
        double Corde = DCorde;
        int Index = 1;
        double U, Alpha;
        for (i = 2; i < NbPoints; i++)
        {
            while (LP[Index].X() < Corde)
            {
                Index++;
            }
            Alpha = (Corde - LP[Index - 1].X()) / (LP[Index].X() - LP[Index - 1].X());
            U = LP[Index - 1].Y() + Alpha * (LP[Index].Y() - LP[Index - 1].Y());
            AppDef_MultiPointConstraint MPC = new(1, 2);
            ML.Value3dOnF1OnF2(U, out P3d, out PF1, out PF2);
            MPC.SetPoint(1, P3d);
            MPC.SetPoint2d(2, PF1);
            MPC.SetPoint2d(3, PF2);
            MLS.SetValue(i, MPC);
            Corde = i * DCorde;
        }
        AppDef_MultiPointConstraint MPE = new(1, 2);
        ML.Value3dOnF1OnF2(U2, out P3d, out PF1, out PF2);
        MPE.SetPoint(1, P3d);
        MPE.SetPoint2d(2, PF1);
        MPE.SetPoint2d(3, PF2);
        MLS.SetValue(NbPoints, MPE);

        AppDef_Compute Fit = new(MLS);

        int NbCurves = Fit.NbMultiCurves();

        if (NbCurves == 0)
        {
            NCollection_Array1<double> Knots = new(1, NbPoints);
            NCollection_Array1<int> Mults = new(1, NbPoints);
            Mults.Init(1);
            Mults[1] = Mults[NbPoints] = 2;
            NCollection_Array1<Point3D> P = new(1, NbPoints);
            NCollection_Array1<Point2D> P1 = new(1, NbPoints);
            NCollection_Array1<Point2D> P2 = new(1, NbPoints);

            double Uf = ML.FirstParameter();
            double Ul = ML.LastParameter();
            double dUlf = (Ul - Uf) / (NbPoints - 1);
            AppDef_MultiPointConstraint MPC;
            for (i = 1; i <= NbPoints - 1; i++)
            {
                MPC = MLS.Value(i);
                U = Uf + (i - 1) * dUlf;
                P[i] = MPC.Point(1);
                P1[i] = MPC.Point2d(2);
                P2[i] = MPC.Point2d(3);
                Knots[i] = U;
            }
            // eval the last point on Ul
            MPC = MLS.Value(NbPoints);
            P[NbPoints] = MPC.Point(1);
            P1[NbPoints] = MPC.Point2d(2);
            P2[NbPoints] = MPC.Point2d(3);
            Knots[NbPoints] = Ul;

            myCurve = new Geom_BSplineCurve(P, Knots, Mults, 1);
            myPCurve1 = new Geom2d_BSplineCurve(P1, Knots, Mults, 1);
            myPCurve2 = new Geom2d_BSplineCurve(P2, Knots, Mults, 1);

            myIsDone = true;

            return;
        }

        // Les approx sont a priori OK.

        AppParCurves_MultiBSpCurve MBSp = Fit.SplineValue();
        int NbPoles = MBSp.NbPoles();
        NCollection_Array1<Point3D> Poles = new(1, NbPoles);
        NCollection_Array1<Point2D> Poles2d1 = new(1, NbPoles);
        NCollection_Array1<Point2D> Poles2d2 = new(1, NbPoles);

        MBSp.Curve(1, Poles);
        MBSp.Curve(2, Poles2d1);
        MBSp.Curve(3, Poles2d2);

        NCollection_Array1<double> aKnots = MBSp.Knots();
        NCollection_Array1<int> aMults = MBSp.Multiplicities();
        int Degree = MBSp.Degree();

        myCurve = new Geom_BSplineCurve(Poles, aKnots, aMults, Degree);
        myPCurve1 = new Geom2d_BSplineCurve(Poles2d1, aKnots, aMults, Degree);
        myPCurve2 = new Geom2d_BSplineCurve(Poles2d2, aKnots, aMults, Degree);

        myIsDone = true;
    }

    /// <summary>True once <see cref="Perform"/> has run.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>The 3D curve.</summary>
    /// <exception cref="NotDoneException">Not done.</exception>
    public Geom_Curve Curve()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "BRepFill_ApproxSeewing::Curve");
        return myCurve!;
    }

    /// <summary>The pcurve on the first face.</summary>
    /// <exception cref="NotDoneException">Not done.</exception>
    public Geom2d_Curve CurveOnF1()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "BRepFill_ApproxSeewing::CurveOnF1");
        return myPCurve1!;
    }

    /// <summary>The pcurve on the second face.</summary>
    /// <exception cref="NotDoneException">Not done.</exception>
    public Geom2d_Curve CurveOnF2()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "BRepFill_ApproxSeewing::CurveOnF2");
        return myPCurve2!;
    }
}
