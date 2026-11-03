// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_MakeApprox.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_MakeApprox.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// Approximates a plate surface by a B-spline surface (AdvApp2Var), with the G0 or G1 criterion of the plate's
/// constraints, a criterion given by the caller, or none.
/// </summary>
/// <remarks>
/// The approximation runs in the constructor. <see cref="Surface"/> freezes the surface it returns (the same object
/// each time); ported code that goes on changing it reads <see cref="InternalSurface"/>.
/// </remarks>
public class GeomPlate_MakeApprox
{
    private readonly GeomPlate_Surface myPlate;
    private readonly BSplineSurface? mySurface;
    private readonly double myAppError;
    private readonly double myCritError;

    private sealed class GeomPlate_MakeApprox_Eval : EvaluatorFunc2Var
    {
        private readonly Surface mySurf;

        public GeomPlate_MakeApprox_Eval(Surface theSurf)
        {
            mySurf = theSurf;
        }

        public override void Evaluate(int Dimension, ReadOnlySpan<double> UStartEnd, ReadOnlySpan<double> VStartEnd,
                                      int FavorIso, double ConstParam, int NbParams, ReadOnlySpan<double> Parameters,
                                      int UOrder, int VOrder, Span<double> Result, out int ErrorCode)
        {
            ErrorCode = 0;
            int idim, jpar;
            double Upar, Vpar;

            // Dimension incorrecte
            if (Dimension != 3)
            {
                ErrorCode = 1;
            }

            // Parametres incorrects
            if (FavorIso == 1)
            {
                Upar = ConstParam;
                if ((Upar < UStartEnd[0]) || (Upar > UStartEnd[1]))
                {
                    ErrorCode = 2;
                }
                for (jpar = 1; jpar <= NbParams; jpar++)
                {
                    Vpar = Parameters[jpar - 1];
                    if ((Vpar < VStartEnd[0]) || (Vpar > VStartEnd[1]))
                    {
                        ErrorCode = 2;
                    }
                }
            }
            else
            {
                Vpar = ConstParam;
                if ((Vpar < VStartEnd[0]) || (Vpar > VStartEnd[1]))
                {
                    ErrorCode = 2;
                }
                for (jpar = 1; jpar <= NbParams; jpar++)
                {
                    Upar = Parameters[jpar - 1];
                    if ((Upar < UStartEnd[0]) || (Upar > UStartEnd[1]))
                    {
                        ErrorCode = 2;
                    }
                }
            }

            // Initialisation
            for (idim = 1; idim <= Dimension; idim++)
            {
                for (jpar = 1; jpar <= NbParams; jpar++)
                {
                    Result[idim - 1 + (jpar - 1) * Dimension] = 0.0;
                }
            }

            int Order = UOrder + VOrder;
            Point3D pnt;
            Vector3D v1, v2, v3, v4, v5;

            if (FavorIso == 1)
            {
                Upar = ConstParam;
                switch (Order)
                {
                    case 0:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Vpar = Parameters[jpar - 1];
                            pnt = mySurf.Value(Upar, Vpar);
                            Result[(jpar - 1) * Dimension] = pnt.X();
                            Result[1 + (jpar - 1) * Dimension] = pnt.Y();
                            Result[2 + (jpar - 1) * Dimension] = pnt.Z();
                        }
                        break;
                    case 1:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Vpar = Parameters[jpar - 1];
                            mySurf.D1(Upar, Vpar, out pnt, out v1, out v2);
                            if (UOrder == 1)
                            {
                                Result[(jpar - 1) * Dimension] = v1.X();
                                Result[1 + (jpar - 1) * Dimension] = v1.Y();
                                Result[2 + (jpar - 1) * Dimension] = v1.Z();
                            }
                            else
                            {
                                Result[(jpar - 1) * Dimension] = v2.X();
                                Result[1 + (jpar - 1) * Dimension] = v2.Y();
                                Result[2 + (jpar - 1) * Dimension] = v2.Z();
                            }
                        }
                        break;
                    case 2:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Vpar = Parameters[jpar - 1];
                            mySurf.D2(Upar, Vpar, out pnt, out v1, out v2, out v3, out v4, out v5);
                            if (UOrder == 2)
                            {
                                Result[(jpar - 1) * Dimension] = v3.X();
                                Result[1 + (jpar - 1) * Dimension] = v3.Y();
                                Result[2 + (jpar - 1) * Dimension] = v3.Z();
                            }
                            else if (UOrder == 1)
                            {
                                Result[(jpar - 1) * Dimension] = v5.X();
                                Result[1 + (jpar - 1) * Dimension] = v5.Y();
                                Result[2 + (jpar - 1) * Dimension] = v5.Z();
                            }
                            else if (UOrder == 0)
                            {
                                Result[(jpar - 1) * Dimension] = v4.X();
                                Result[1 + (jpar - 1) * Dimension] = v4.Y();
                                Result[2 + (jpar - 1) * Dimension] = v4.Z();
                            }
                        }
                        break;
                }
            }
            else
            {
                Vpar = ConstParam;
                switch (Order)
                {
                    case 0:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Upar = Parameters[jpar - 1];
                            pnt = mySurf.Value(Upar, Vpar);
                            Result[(jpar - 1) * Dimension] = pnt.X();
                            Result[1 + (jpar - 1) * Dimension] = pnt.Y();
                            Result[2 + (jpar - 1) * Dimension] = pnt.Z();
                        }
                        break;
                    case 1:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Upar = Parameters[jpar - 1];
                            mySurf.D1(Upar, Vpar, out pnt, out v1, out v2);
                            if (UOrder == 1)
                            {
                                Result[(jpar - 1) * Dimension] = v1.X();
                                Result[1 + (jpar - 1) * Dimension] = v1.Y();
                                Result[2 + (jpar - 1) * Dimension] = v1.Z();
                            }
                            else
                            {
                                Result[(jpar - 1) * Dimension] = v2.X();
                                Result[1 + (jpar - 1) * Dimension] = v2.Y();
                                Result[2 + (jpar - 1) * Dimension] = v2.Z();
                            }
                        }
                        break;
                    case 2:
                        for (jpar = 1; jpar <= NbParams; jpar++)
                        {
                            Upar = Parameters[jpar - 1];
                            mySurf.D2(Upar, Vpar, out pnt, out v1, out v2, out v3, out v4, out v5);
                            if (UOrder == 2)
                            {
                                Result[(jpar - 1) * Dimension] = v3.X();
                                Result[1 + (jpar - 1) * Dimension] = v3.Y();
                                Result[2 + (jpar - 1) * Dimension] = v3.Z();
                            }
                            else if (UOrder == 1)
                            {
                                Result[(jpar - 1) * Dimension] = v5.X();
                                Result[1 + (jpar - 1) * Dimension] = v5.Y();
                                Result[2 + (jpar - 1) * Dimension] = v5.Z();
                            }
                            else if (UOrder == 0)
                            {
                                Result[(jpar - 1) * Dimension] = v4.X();
                                Result[1 + (jpar - 1) * Dimension] = v4.Y();
                                Result[2 + (jpar - 1) * Dimension] = v4.Z();
                            }
                        }
                        break;
                }
            }
        }
    }

    /// <summary>Approximates <paramref name="SurfPlate"/> under the criterion <paramref name="PlateCrit"/>.</summary>
    /// <exception cref="ConstructionErrorException">The approximation fails.</exception>
    public GeomPlate_MakeApprox(GeomPlate_Surface SurfPlate, Criterion PlateCrit, double Tol3d, int Nbmax,
                                int dgmax, Continuity Continuity = C1, double EnlargeCoeff = 1.1)
    {
        myPlate = SurfPlate;

        double U0 = 0.0, U1 = 0.0, V0 = 0.0, V1 = 0.0;
        myPlate.RealBounds(out U0, out U1, out V0, out V1);
        U0 = EnlargeCoeff * U0;
        U1 = EnlargeCoeff * U1;
        V0 = EnlargeCoeff * V0;
        V1 = EnlargeCoeff * V1;

        int nb1 = 0, nb2 = 0, nb3 = 1;
        NCollection_HArray1<double> nul1 = new NCollection_HArray1<double>(1, 1);
        nul1.Init(0.0);
        NCollection_HArray2<double> nul2 = new NCollection_HArray2<double>(1, 1, 1, 4);
        nul2.Init(0.0);
        NCollection_HArray1<double> eps3D = new NCollection_HArray1<double>(1, 1);
        eps3D.Init(Tol3d);
        NCollection_HArray2<double> epsfr = new NCollection_HArray2<double>(1, 1, 1, 4);
        epsfr.Init(Tol3d);
        IsoType myType = IsoType.IsoV;
        int myPrec = 0;

        DichoCutting myDec = new DichoCutting();

        GeomPlate_MakeApprox_Eval ev = new GeomPlate_MakeApprox_Eval(myPlate);
        ApproxAFunc2Var AppPlate = new ApproxAFunc2Var(nb1, nb2, nb3, nul1, nul1, eps3D, nul2, nul2,
                                                                            epsfr, U0, U1, V0, V1, myType, Continuity,
                                                                            Continuity, myPrec, dgmax, dgmax, Nbmax, ev,
                                                                            PlateCrit, myDec, myDec);
        mySurface = AppPlate.Surface(1);
        myAppError = AppPlate.MaxError(3, 1);
        myCritError = AppPlate.CritError(3, 1);
    }

    /// <summary>
    /// Approximates <paramref name="SurfPlate"/> under the G0 (<paramref name="CritOrder"/> 0) or G1 (1) criterion of
    /// the plate's constraints, or none (-1). <paramref name="dmax"/> is the largest distance to the constraints:
    /// the criterion's threshold is at least ten times it.
    /// </summary>
    /// <exception cref="ConstructionErrorException">The approximation fails.</exception>
    public GeomPlate_MakeApprox(GeomPlate_Surface SurfPlate, double Tol3d, int Nbmax, int dgmax, double dmax,
                                int CritOrder = 0, Continuity Continuity = C1, double EnlargeCoeff = 1.1)
    {
        myPlate = SurfPlate;

        NCollection_Sequence<Xy> Seq2d = new NCollection_Sequence<Xy>();
        NCollection_Sequence<Xyz> Seq3d = new NCollection_Sequence<Xyz>();

        if (CritOrder >= 0)
        {
            //    contraintes 2d d'ordre 0
            myPlate.Constraints(Seq2d);

            //    contraintes 3d correspondantes sur plate
            int i, nbp = Seq2d.Length();
            for (i = 1; i <= nbp; i++)
            {
                Xy P2d = Seq2d.Value(i);
                Point3D PP;
                Vector3D v1h, v2h, v3h;
                if (CritOrder == 0)
                {
                    //    a l'ordre 0
                    myPlate.D0(P2d.X(), P2d.Y(), out PP);
                    Xyz P3d = new Xyz(PP.X(), PP.Y(), PP.Z());
                    Seq3d.Append(P3d);
                }
                else
                {
                    //    a l'ordre 1
                    myPlate.D1(P2d.X(), P2d.Y(), out PP, out v1h, out v2h);
                    v3h = v1h ^ v2h;
                    Xyz P3d = new Xyz(v3h.X(), v3h.Y(), v3h.Z());
                    Seq3d.Append(P3d);
                }
            }
        }

        double U0 = 0.0, U1 = 0.0, V0 = 0.0, V1 = 0.0;
        myPlate.RealBounds(out U0, out U1, out V0, out V1);
        U0 = EnlargeCoeff * U0;
        U1 = EnlargeCoeff * U1;
        V0 = EnlargeCoeff * V0;
        V1 = EnlargeCoeff * V1;

        double seuil = Tol3d;
        if (CritOrder == 0 && Tol3d < 10 * dmax)
        {
            seuil = 10 * dmax;
        }
        if (CritOrder == 1 && Tol3d < 10 * dmax)
        {
            seuil = 10 * dmax;
        }
        int nb1 = 0, nb2 = 0, nb3 = 1;
        NCollection_HArray1<double> nul1 = new NCollection_HArray1<double>(1, 1);
        nul1.Init(0.0);
        NCollection_HArray2<double> nul2 = new NCollection_HArray2<double>(1, 1, 1, 4);
        nul2.Init(0.0);
        NCollection_HArray1<double> eps3D = new NCollection_HArray1<double>(1, 1);
        eps3D.Init(Tol3d);
        NCollection_HArray2<double> epsfr = new NCollection_HArray2<double>(1, 1, 1, 4);
        epsfr.Init(Tol3d);

        IsoType myType = IsoType.IsoV;
        int myPrec = 0;

        DichoCutting myDec = new DichoCutting();

        if (CritOrder == -1)
        {
            myPrec = 1;
            GeomPlate_MakeApprox_Eval ev = new GeomPlate_MakeApprox_Eval(myPlate);
            ApproxAFunc2Var AppPlate = new ApproxAFunc2Var(nb1, nb2, nb3, nul1, nul1, eps3D, nul2,
                                                                                nul2, epsfr, U0, U1, V0, V1, myType,
                                                                                Continuity, Continuity, myPrec, dgmax,
                                                                                dgmax, Nbmax, ev, myDec, myDec);
            mySurface = AppPlate.Surface(1);
            myAppError = AppPlate.MaxError(3, 1);
            myCritError = 0.0;
        }
        else if (CritOrder == 0)
        {
            GeomPlate_PlateG0Criterion Crit0 = new GeomPlate_PlateG0Criterion(Seq2d, Seq3d, seuil);
            GeomPlate_MakeApprox_Eval ev = new GeomPlate_MakeApprox_Eval(myPlate);
            ApproxAFunc2Var AppPlate = new ApproxAFunc2Var(nb1, nb2, nb3, nul1, nul1, eps3D, nul2,
                                                                                nul2, epsfr, U0, U1, V0, V1, myType,
                                                                                Continuity, Continuity, myPrec, dgmax,
                                                                                dgmax, Nbmax, ev, Crit0, myDec, myDec);
            mySurface = AppPlate.Surface(1);
            myAppError = AppPlate.MaxError(3, 1);
            myCritError = AppPlate.CritError(3, 1);
        }
        else if (CritOrder == 1)
        {
            GeomPlate_PlateG1Criterion Crit1 = new GeomPlate_PlateG1Criterion(Seq2d, Seq3d, seuil);
            GeomPlate_MakeApprox_Eval ev = new GeomPlate_MakeApprox_Eval(myPlate);
            ApproxAFunc2Var AppPlate = new ApproxAFunc2Var(nb1, nb2, nb3, nul1, nul1, eps3D, nul2,
                                                                                nul2, epsfr, U0, U1, V0, V1, myType,
                                                                                Continuity, Continuity, myPrec, dgmax,
                                                                                dgmax, Nbmax, ev, Crit1, myDec, myDec);
            mySurface = AppPlate.Surface(1);
            myAppError = AppPlate.MaxError(3, 1);
            myCritError = AppPlate.CritError(3, 1);
        }
        // OCCT: another CritOrder leaves the surface null and the errors uninitialised; the port's are 0
    }

    /// <summary>The approximating surface (frozen; null for a criterion order other than -1, 0 or 1).</summary>
    public BSplineSurface? Surface()
    {
        mySurface?.Freeze();
        return mySurface;
    }

    /// <summary>The approximating surface, not frozen, for ported code.</summary>
    internal BSplineSurface? InternalSurface() => mySurface;

    public double ApproxError() => myAppError;

    public double CriterionError() => myCritError;
}
