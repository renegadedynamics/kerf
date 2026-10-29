// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_ResolConstraint.gxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_TheResol.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// lpa, le 11/09/91

// Approximation of a constrained set of points (MultiLine) with an
// approximate solution (MultiCurve). The algorithm used is the
// Uzawa algorithm from the mathematical package.

using static Occt.AppParCurves_Constraint;

namespace Occt;

/// <summary>
/// Corrects an approximating MultiCurve so that it satisfies the passing and tangency constraints of a
/// MultiLine, by the Uzawa algorithm.
/// </summary>
/// <remarks>
/// The generic form of OCCT's <c>AppParCurves_ResolConstraint.gxx</c> (decision 142), instantiated by OCCT as
/// AppDef_TheResol, AppDef_ResConstraintOf... and the GeomInt and BRepApprox equivalents. A working object; the
/// matrices returned are its own (decision 33; shared: don't modify). OCCT declares an <c>Error()</c> accessor
/// but never defines it; it is left out.
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="ToolLine">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
public class AppParCurves_ResolConstraint<MultiLine, ToolLine>
    where ToolLine : struct, IAppParCurves_LineTool<MultiLine>
{
    private bool Done;
    private readonly math_Matrix Cont;
    private readonly math_Matrix DeCont;
    private readonly math_Vector Secont;
    private readonly math_Matrix CTCinv;
    private readonly math_Vector Vardua;
    private int IncPass;
    private int IncTan;
    private int IncCurv;
    private readonly NCollection_Array1<int> IPas;
    private readonly NCollection_Array1<int> ITan;
    private readonly NCollection_Array1<int> ICurv;

    /// <summary>
    /// Corrects the poles of <paramref name="SCurv"/> (changed in place) for the constraints
    /// <paramref name="TheConstraints"/> on the points <paramref name="FirstPoint"/> to
    /// <paramref name="LastPoint"/> of <paramref name="SSP"/>, given the Bernstein matrix and its derivative at the
    /// points' parameters.
    /// </summary>
    public AppParCurves_ResolConstraint(MultiLine SSP, AppParCurves_MultiCurve SCurv, int FirstPoint, int LastPoint,
                                        NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints, math_Matrix Bern,
                                        math_Matrix DerivativeBern, double Tolerance = 1.0e-10)
    {
        Cont = new math_Matrix(1,
                               NbConstraints(SSP, FirstPoint, LastPoint, TheConstraints),
                               1,
                               NbColumns(SSP, SCurv.NbPoles() - 1),
                               0.0);
        DeCont = new math_Matrix(1,
                                 NbConstraints(SSP, FirstPoint, LastPoint, TheConstraints),
                                 1,
                                 NbColumns(SSP, SCurv.NbPoles() - 1),
                                 0.0);
        Secont = new math_Vector(1, NbConstraints(SSP, FirstPoint, LastPoint, TheConstraints), 0.0);
        CTCinv = new math_Matrix(1,
                                 NbConstraints(SSP, FirstPoint, LastPoint, TheConstraints),
                                 1,
                                 NbConstraints(SSP, FirstPoint, LastPoint, TheConstraints));
        Vardua = new math_Vector(1, NbConstraints(SSP, FirstPoint, LastPoint, TheConstraints));
        IPas = new NCollection_Array1<int>(1, LastPoint - FirstPoint + 1);
        ITan = new NCollection_Array1<int>(1, LastPoint - FirstPoint + 1);
        ICurv = new NCollection_Array1<int>(1, LastPoint - FirstPoint + 1);

        int i, j, k, NbCu = SCurv.NbCurves();
        int Npt;
        int Inc3, IncSec, IncCol, IP = 0, CCol;
        int myindex, Def = SCurv.NbPoles() - 1;
        int nb3d, nb2d, Npol = Def + 1, Npol2 = 2 * Npol;
        double T1 = 0.0, T2 = 0.0, T3, Tmax, Daij;
        gp_Vec V;
        gp_Vec2d V2d;
        gp_Pnt Poi;
        gp_Pnt2d Poi2d;
        AppParCurves_ConstraintCouple mycouple;
        AppParCurves_Constraint FC = AppParCurves_NoConstraint, LC = AppParCurves_NoConstraint, Cons;

        // Boucle de calcul du nombre de points de passage afin de dimensionner
        // les matrices.
        IncPass = 0;
        IncTan = 0;
        IncCurv = 0;
        for (i = TheConstraints.Lower(); i <= TheConstraints.Upper(); i++)
        {
            mycouple = TheConstraints.Value(i);
            myindex = mycouple.Index();
            Cons = mycouple.Constraint();
            if (myindex == FirstPoint)
            {
                FC = Cons;
            }
            if (myindex == LastPoint)
            {
                LC = Cons;
            }
            if ((int)Cons >= 1)
            {
                IncPass++; // IncPass = nbre de points de passage.
                IPas[IncPass] = myindex;
            }
            if ((int)Cons >= 2)
            {
                IncTan++; // IncTan= nbre de points de tangence.
                ITan[IncTan] = myindex;
            }
            if ((int)Cons == 3)
            {
                IncCurv++; // IncCurv = nbre de pts de courbure.
                ICurv[IncCurv] = myindex;
            }
        }

        if (IncPass == 0)
        {
            Done = true;
            return;
        }

        nb3d = ToolLine.NbP3d(SSP);
        nb2d = ToolLine.NbP2d(SSP);
        int mynb3d = nb3d, mynb2d = nb2d;
        if (nb3d == 0)
        {
            mynb3d = 1;
        }
        if (nb2d == 0)
        {
            mynb2d = 1;
        }
        CCol = nb3d * 3 + nb2d * 2;

        // Declaration et initialisation des matrices et vecteurs de contraintes:
        math_Matrix ContInit = new(1, IncPass, 1, Npol);
        math_Vector Start = new(1, CCol * Npol);
        NCollection_Array2<int> Ibont = new(1, NbCu, 1, IncTan);

        // Filling Cont for the passing points:
        // =================================================
        for (i = 1; i <= IncPass; i++)
        { // Cette partie ne depend que de Bernstein
            Npt = IPas[i];
            for (j = 1; j <= Npol; j++)
            {
                ContInit[i, j] = Bern[Npt, j];
            }
        }
        for (i = 1; i <= CCol; i++)
        {
            Cont.Set(IncPass * (i - 1) + 1, IncPass * i, Npol * (i - 1) + 1, Npol * i, ContInit);
        }

        // Retrieval of starting vectors for Uzawa. This vector represents the
        // poles of SCurv.
        // Filling of secont and resolution.

        NCollection_Array1<gp_Vec> tabV = new(1, mynb3d);
        NCollection_Array1<gp_Vec2d> tabV2d = new(1, mynb2d);
        NCollection_Array1<gp_Pnt> tabP = new(1, mynb3d);
        NCollection_Array1<gp_Pnt2d> tabP2d = new(1, mynb2d);

        Inc3 = CCol * IncPass + 1;
        IncCol = 0;
        IncSec = 0;
        for (k = 1; k <= NbCu; k++)
        {
            if (k <= nb3d)
            {
                for (i = 1; i <= IncTan; i++)
                {
                    Npt = ITan[i];
                    // choix du maximum de tangence pour exprimer la colinearite:
                    ToolLine.Tangency(SSP, Npt, tabV);
                    V = tabV[k];
                    V.Coord(out T1, out T2, out T3);
                    Tmax = Math.Abs(T1);
                    Ibont[k, i] = 1;
                    if (Math.Abs(T2) > Tmax)
                    {
                        Tmax = Math.Abs(T2);
                        Ibont[k, i] = 2;
                    }
                    if (Math.Abs(T3) > Tmax)
                    {
                        Tmax = Math.Abs(T3);
                        Ibont[k, i] = 3;
                    }
                    if (Ibont[k, i] == 3)
                    {
                        for (j = 1; j <= Npol; j++)
                        {
                            Daij = DerivativeBern[Npt, j];
                            Cont[Inc3, j + Npol + IncCol] = Daij * T3 / Tmax;
                            Cont[Inc3, j + Npol2 + IncCol] = -Daij * T2 / Tmax;
                            Cont[Inc3 + 1, j + IncCol] = Daij * T3 / Tmax;
                            Cont[Inc3 + 1, j + Npol2 + IncCol] = -Daij * T1 / Tmax;
                        }
                    }
                    else if (Ibont[k, i] == 1)
                    {
                        for (j = 1; j <= Npol; j++)
                        {
                            Daij = DerivativeBern[Npt, j];
                            Cont[Inc3, j + IncCol] = Daij * T3 / Tmax;
                            Cont[Inc3, j + Npol2 + IncCol] = -Daij * T1 / Tmax;
                            Cont[Inc3 + 1, j + IncCol] = Daij * T2 / Tmax;
                            Cont[Inc3 + 1, j + Npol + IncCol] = -Daij * T1 / Tmax;
                        }
                    }
                    else if (Ibont[k, i] == 2)
                    {
                        for (j = 1; j <= Def + 1; j++)
                        {
                            Daij = DerivativeBern[Npt, j];
                            Cont[Inc3, j + Npol + IncCol] = Daij * T3 / Tmax;
                            Cont[Inc3, j + Npol2 + IncCol] = -Daij * T2 / Tmax;
                            Cont[Inc3 + 1, j + IncCol] = Daij * T2 / Tmax;
                            Cont[Inc3 + 1, j + Npol + IncCol] = -Daij * T1 / Tmax;
                        }
                    }
                    Inc3 = Inc3 + 2;
                }

                // Remplissage du second membre:
                for (i = 1; i <= IncPass; i++)
                {
                    ToolLine.Value(SSP, IPas[i], tabP);
                    Poi = tabP[k];
                    Secont[i + IncSec] = Poi.X();
                    Secont[i + IncPass + IncSec] = Poi.Y();
                    Secont[i + 2 * IncPass + IncSec] = Poi.Z();
                }
                IncSec = IncSec + 3 * IncPass;

                // Vecteur de depart:
                for (j = 1; j <= Npol; j++)
                {
                    Poi = SCurv.Value(j).Point(k);
                    Start[j + IncCol] = Poi.X();
                    Start[j + Npol + IncCol] = Poi.Y();
                    Start[j + Npol2 + IncCol] = Poi.Z();
                }
                IncCol = IncCol + 3 * Npol;
            }

            else
            {
                for (i = 1; i <= IncTan; i++)
                {
                    Npt = ITan[i];
                    ToolLine.Tangency(SSP, Npt, tabV2d);
                    V2d = tabV2d[k - nb3d];
                    T1 = V2d.X();
                    T2 = V2d.Y();
                    Tmax = Math.Abs(T1);
                    Ibont[k, i] = 1;
                    if (Math.Abs(T2) > Tmax)
                    {
                        Tmax = Math.Abs(T2);
                        Ibont[k, i] = 2;
                    }
                    for (j = 1; j <= Npol; j++)
                    {
                        Daij = DerivativeBern[Npt, j];
                        Cont[Inc3, j + IncCol] = Daij * T2;
                        Cont[Inc3, j + Npol + IncCol] = -Daij * T1;
                    }
                    Inc3 = Inc3 + 1;
                }

                // Remplissage du second membre:
                for (i = 1; i <= IncPass; i++)
                {
                    ToolLine.Value(SSP, IPas[i], tabP2d);
                    // OCCT: indexes the 2d points with the passing point i where the curve k - nb3d is meant; out of
                    // range unless i - nb3d happens to be a 2d curve (unchecked in OCCT; the port throws)
                    Poi2d = tabP2d[i - nb3d];
                    Secont[i + IncSec] = Poi2d.X();
                    Secont[i + IncPass + IncSec] = Poi2d.Y();
                }
                IncSec = IncSec + 2 * IncPass;

                // Remplissage du vecteur de depart:
                for (j = 1; j <= Npol; j++)
                {
                    Poi2d = SCurv.Value(j).Point2d(k);
                    Start[j + IncCol] = Poi2d.X();
                    Start[j + Npol + IncCol] = Poi2d.Y();
                }
                IncCol = IncCol + Npol2;
            }
        }

        // Equations exprimant le meme rapport de tangence sur chaque courbe:
        // On prend les coordonnees les plus significatives.

        --Inc3;
        for (i = 1; i <= IncTan; ++i)
        {
            IncCol = 0;
            Npt = ITan[i];
            for (k = 1; k <= NbCu - 1; ++k)
            {
                ++Inc3;
                // Initialize first relation variable (T1)
                int addIndex_1 = 0, aVal = Ibont[k, i];
                switch (aVal)
                {
                    case 1: // T1 ~ T1x
                    {
                        if (k <= nb3d)
                        {
                            ToolLine.Tangency(SSP, Npt, tabV);
                            V = tabV[k];
                            T1 = V.X();
                            IP = 3 * Npol;
                        }
                        else
                        {
                            ToolLine.Tangency(SSP, Npt, tabV2d);
                            V2d = tabV2d[k - nb3d];
                            T1 = V2d.X();
                            IP = 2 * Npol;
                        }
                        addIndex_1 = 0;
                        break;
                    }
                    case 2: // T1 ~ T1y
                    {
                        if (k <= nb3d)
                        {
                            ToolLine.Tangency(SSP, Npt, tabV);
                            V = tabV[k];
                            // OCCT: T1 = V.Y() is missing, so T1 keeps its previous value for a 3d curve
                            IP = 3 * Npol;
                        }
                        else
                        {
                            ToolLine.Tangency(SSP, Npt, tabV2d);
                            V2d = tabV2d[k - nb3d];
                            T1 = V2d.Y();
                            IP = 2 * Npol;
                        }
                        addIndex_1 = Npol;
                        break;
                    }
                    case 3: // T1 ~ T1z
                    {
                        ToolLine.Tangency(SSP, Npt, tabV);
                        V = tabV[k];
                        T1 = V.Z();
                        IP = 3 * Npol;
                        addIndex_1 = 2 * Npol;
                        break;
                    }
                }
                // Initialize second relation variable (T2)
                int addIndex_2 = 0, aNextVal = Ibont[k + 1, i];
                switch (aNextVal)
                {
                    case 1: // T2 ~ T2x
                    {
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(SSP, Npt, tabV);
                            V = tabV[k + 1];
                            T2 = V.X();
                        }
                        else
                        {
                            ToolLine.Tangency(SSP, Npt, tabV2d);
                            V2d = tabV2d[k + 1 - nb3d];
                            T2 = V2d.X();
                        }
                        addIndex_2 = 0;
                        break;
                    }
                    case 2: // T2 ~ T2y
                    {
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(SSP, Npt, tabV);
                            V = tabV[k + 1];
                            T2 = V.Y();
                        }
                        else
                        {
                            ToolLine.Tangency(SSP, Npt, tabV2d);
                            V2d = tabV2d[k + 1 - nb3d];
                            T2 = V2d.Y();
                        }
                        addIndex_2 = Npol;
                        break;
                    }
                    case 3: // T2 ~ T2z
                    {
                        ToolLine.Tangency(SSP, Npt, tabV);
                        V = tabV[k + 1];
                        T2 = V.Z();
                        addIndex_2 = 2 * Npol;
                        break;
                    }
                }

                // Relations between T1 and T2:
                for (j = 1; j <= Npol; j++)
                {
                    Daij = DerivativeBern[Npt, j];
                    Cont[Inc3, j + IncCol + addIndex_1] = Daij * T2;
                    Cont[Inc3, j + IP + IncCol + addIndex_2] = -Daij * T1;
                }
                IncCol += IP;
            }
        }

        // Equations concernant la courbure: (commented out in OCCT)

        // Resolution par Uzawa:

        math_Uzawa UzaResol = new(Cont, Secont, Start, Tolerance);
        if (!(UzaResol.IsDone()))
        {
            Done = false;
            return;
        }
        CTCinv.Initialized(UzaResol.InverseCont());
        UzaResol.Duale(Vardua);
        for (i = 1; i <= CTCinv.RowNumber(); i++)
        {
            for (j = i; j <= CTCinv.RowNumber(); j++)
            {
                CTCinv[i, j] = CTCinv[j, i];
            }
        }
        Done = true;
        math_Vector VecPoles = new(1, CCol * Npol);
        VecPoles.Initialized(UzaResol.Value());

        int polinit = 1, polfin = Npol;
        if ((int)FC >= 1)
        {
            polinit = 2;
        }
        if ((int)LC >= 1)
        {
            polfin = Npol - 1;
        }

        for (i = polinit; i <= polfin; i++)
        {
            IncCol = 0;
            AppParCurves_MultiPoint MPol = new(nb3d, nb2d);
            for (k = 1; k <= NbCu; k++)
            {
                if (k <= nb3d)
                {
                    gp_Pnt Pol = new(VecPoles[IncCol + i], VecPoles[IncCol + Npol + i], VecPoles[IncCol + 2 * Npol + i]);
                    MPol.SetPoint(k, Pol);
                    IncCol = IncCol + 3 * Npol;
                }
                else
                {
                    gp_Pnt2d Pol2d = new(VecPoles[IncCol + i], VecPoles[IncCol + Npol + i]);
                    MPol.SetPoint2d(k, Pol2d);
                    IncCol = IncCol + 2 * Npol;
                }
            }
            SCurv.SetValue(i, MPol);
        }
    }

    /// <summary>True if the constraints were resolved.</summary>
    public bool IsDone() => Done;

    /// <summary>The number of constraint equations.</summary>
    protected int NbConstraints(MultiLine SSP, int FirstPoint, int LastPoint,
                                NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints)
    {
        // Boucle de calcul du nombre de points de passage afin de dimensionner
        // les matrices.
        int aIncPass, aIncTan, aIncCurv, aCCol;
        int i;
        AppParCurves_Constraint Cons;

        aIncPass = 0;
        aIncTan = 0;
        aIncCurv = 0;

        for (i = TheConstraints.Lower(); i <= TheConstraints.Upper(); i++)
        {
            Cons = (TheConstraints.Value(i)).Constraint();
            if ((int)Cons >= 1)
            {
                aIncPass++; // IncPass = nbre de points de passage.
            }
            if ((int)Cons >= 2)
            {
                aIncTan++; // IncTan= nbre de points de tangence.
            }
            if ((int)Cons == 3)
            {
                aIncCurv++; // IncCurv = nbre de pts de courbure.
            }
        }
        int nb3d = ToolLine.NbP3d(SSP);
        int nb2d = ToolLine.NbP2d(SSP);
        aCCol = nb3d * 3 + nb2d * 2;

        return aCCol * aIncPass + aIncTan * (aCCol - 1) + 3 * aIncCurv;
    }

    /// <summary>The number of unknowns: the coordinates of every pole.</summary>
    protected int NbColumns(MultiLine SSP, int Deg)
    {
        int nb3d = ToolLine.NbP3d(SSP);
        int nb2d = ToolLine.NbP2d(SSP);
        int CCol = nb3d * 3 + nb2d * 2;

        return CCol * (Deg + 1);
    }

    /// <summary>The constraint matrix (shared: don't modify).</summary>
    public math_Matrix ConstraintMatrix() => Cont;

    /// <summary>The inverse of C tC, symmetrised (shared: don't modify).</summary>
    public math_Matrix InverseMatrix() => CTCinv;

    /// <summary>The dual variables of the Uzawa resolution (shared: don't modify).</summary>
    public math_Vector Duale() => Vardua;

    /// <summary>
    /// The derivative of the constraint matrix with respect to the parameters (the object's own matrix,
    /// recomputed by each call; shared: don't modify).
    /// </summary>
    public math_Matrix ConstraintDerivative(MultiLine SSP, math_Vector Parameters, int Deg, math_Matrix DA)
    {
        int i, j, k, nb2d, nb3d, CCol, Inc3, IncCol, Npt;
        int Npol, Npol2, NbCu = ToolLine.NbP3d(SSP) + ToolLine.NbP2d(SSP);
        int IP;
        double Daij;
        Npol = Deg + 1;
        Npol2 = 2 * Npol;
        NCollection_Array2<int> Ibont = new(1, NbCu, 1, IncTan);
        math_Matrix DeCInit = new(1, IncPass, 1, Npol);
        math_Vector DDA = new(1, Npol);
        gp_Vec V;
        gp_Vec2d V2d;
        // T1 and T2 are uninitialised in OCCT; the relations below may read the values the loop above left
        double T1 = 0.0, T2 = 0.0, T3, Tmax, DDaij;
        nb3d = ToolLine.NbP3d(SSP);
        nb2d = ToolLine.NbP2d(SSP);
        int mynb3d = nb3d, mynb2d = nb2d;
        if (nb3d == 0)
        {
            mynb3d = 1;
        }
        if (nb2d == 0)
        {
            mynb2d = 1;
        }
        CCol = nb3d * 3 + nb2d * 2;

        NCollection_Array1<gp_Vec> tabV = new(1, mynb3d);
        NCollection_Array1<gp_Vec2d> tabV2d = new(1, mynb2d);

        for (i = 1; i <= DeCont.RowNumber(); i++)
        {
            for (j = 1; j <= DeCont.ColNumber(); j++)
            {
                DeCont[i, j] = 0.0;
            }
        }

        //  Remplissage de DK pour les points de passages:

        for (i = 1; i <= IncPass; i++)
        {
            Npt = IPas[i];
            for (j = 1; j <= Npol; j++)
            {
                DeCInit[i, j] = DA[Npt, j];
            }
        }
        for (i = 1; i <= CCol; i++)
        {
            DeCont.Set(IncPass * (i - 1) + 1, IncPass * i, Npol * (i - 1) + 1, Npol * i, DeCInit);
        }

        // Pour les points de tangence:

        Inc3 = CCol * IncPass + 1;
        IncCol = 0;

        for (k = 1; k <= NbCu; k++)
        {
            if (k <= nb3d)
            {
                for (i = 1; i <= IncTan; i++)
                {
                    Npt = ITan[i];
                    // choix du maximum de tangence pour exprimer la colinearite:
                    ToolLine.Tangency(SSP, Npt, tabV);
                    V = tabV[k];
                    V.Coord(out T1, out T2, out T3);
                    Tmax = Math.Abs(T1);
                    Ibont[k, i] = 1;
                    if (Math.Abs(T2) > Tmax)
                    {
                        Tmax = Math.Abs(T2);
                        Ibont[k, i] = 2;
                    }
                    if (Math.Abs(T3) > Tmax)
                    {
                        Tmax = Math.Abs(T3);
                        Ibont[k, i] = 3;
                    }
                    AppParCurves.SecondDerivativeBernstein(Parameters[Npt], DDA);
                    if (Ibont[k, i] == 3)
                    {
                        for (j = 1; j <= Npol; j++)
                        {
                            DDaij = DDA[j];
                            DeCont[Inc3, j + Npol + IncCol] = DDaij * T3 / Tmax;
                            DeCont[Inc3, j + Npol2 + IncCol] = -DDaij * T2 / Tmax;
                            DeCont[Inc3 + 1, j + IncCol] = DDaij * T3 / Tmax;
                            DeCont[Inc3 + 1, j + Npol2 + IncCol] = -DDaij * T1 / Tmax;
                        }
                    }
                    else if (Ibont[k, i] == 1)
                    {
                        for (j = 1; j <= Npol; j++)
                        {
                            DDaij = DDA[j];
                            DeCont[Inc3, j + IncCol] = DDaij * T3 / Tmax;
                            DeCont[Inc3, j + Npol2 + IncCol] = -DDaij * T1 / Tmax;
                            DeCont[Inc3 + 1, j + IncCol] = DDaij * T2 / Tmax;
                            DeCont[Inc3 + 1, j + Npol + IncCol] = -DDaij * T1 / Tmax;
                        }
                    }
                    else if (Ibont[k, i] == 2)
                    {
                        for (j = 1; j <= Npol; j++)
                        {
                            DDaij = DDA[j];
                            DeCont[Inc3, j + Npol + IncCol] = DDaij * T3 / Tmax;
                            DeCont[Inc3, j + Npol2 + IncCol] = -DDaij * T2 / Tmax;
                            DeCont[Inc3 + 1, j + IncCol] = DDaij * T2 / Tmax;
                            DeCont[Inc3 + 1, j + Npol + IncCol] = -DDaij * T1 / Tmax;
                        }
                    }
                    Inc3 = Inc3 + 2;
                }
                IncCol = IncCol + 3 * Npol;
            }
            else
            {
                for (i = 1; i <= IncTan; i++)
                {
                    Npt = ITan[i];
                    AppParCurves.SecondDerivativeBernstein(Parameters[Npt], DDA);
                    ToolLine.Tangency(SSP, Npt, tabV2d);
                    // OCCT: tabV2d(k) where k - nb3d is meant (out of range when there are 3d curves; the port throws)
                    V2d = tabV2d[k];
                    V2d.Coord(out T1, out T2);
                    Tmax = Math.Abs(T1);
                    Ibont[k, i] = 1;
                    if (Math.Abs(T2) > Tmax)
                    {
                        Tmax = Math.Abs(T2);
                        Ibont[k, i] = 2;
                    }
                    for (j = 1; j <= Npol; j++)
                    {
                        DDaij = DDA[j];
                        DeCont[Inc3, j + IncCol] = DDaij * T2;
                        DeCont[Inc3, j + Npol + IncCol] = -DDaij * T1;
                    }
                    Inc3 = Inc3 + 1;
                }
            }
        }

        // Equations exprimant le meme rapport de tangence sur chaque courbe:
        // On prend les coordonnees les plus significatives.
        // OCCT: the relations below are written into Cont, the constraint matrix, where DeCont is meant; and
        // tabV2d is indexed by k where k - nb3d is meant, as above

        Inc3 = Inc3 - 1;
        for (i = 1; i <= IncTan; i++)
        {
            IncCol = 0;
            Npt = ITan[i];
            AppParCurves.SecondDerivativeBernstein(Parameters[Npt], DDA);
            for (k = 1; k <= NbCu - 1; k++)
            {
                Inc3 = Inc3 + 1;
                if (Ibont[k, i] == 1)
                {
                    if (k <= nb3d)
                    {
                        ToolLine.Tangency(SSP, Npt, tabV);
                        V = tabV[k];
                        T1 = V.X();
                        IP = 3 * Npol;
                    }
                    else
                    {
                        ToolLine.Tangency(SSP, Npt, tabV2d);
                        V2d = tabV2d[k];
                        T1 = V2d.X();
                        IP = 2 * Npol;
                    }
                    if (Ibont[k + 1, i] == 1)
                    { // Relations entre T1x et T2x:
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(SSP, Npt, tabV);
                            V = tabV[k + 1];
                            T2 = V.X();
                        }
                        else
                        {
                            ToolLine.Tangency(SSP, Npt, tabV2d);
                            V2d = tabV2d[k + 1];
                            T2 = V2d.X();
                        }
                        for (j = 1; j <= Npol; j++)
                        {
                            Daij = DDA[j];
                            Cont[Inc3, j + IncCol] = Daij * T2;
                            Cont[Inc3, j + IP + IncCol] = -Daij * T1;
                        }
                        IncCol = IncCol + IP;
                    }
                    else if (Ibont[k + 1, i] == 2)
                    { // Relations entre T1x et T2y:
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(SSP, Npt, tabV);
                            V = tabV[k + 1];
                            T2 = V.Y();
                        }
                        else
                        {
                            ToolLine.Tangency(SSP, Npt, tabV2d);
                            V2d = tabV2d[k + 1];
                            T2 = V2d.Y();
                        }
                        for (j = 1; j <= Npol; j++)
                        {
                            Daij = DDA[j];
                            Cont[Inc3, j + IncCol] = Daij * T2;
                            Cont[Inc3, j + IP + Npol + IncCol] = -Daij * T1;
                        }
                        IncCol = IncCol + IP;
                    }
                    else if (Ibont[k + 1, i] == 3)
                    { // Relations entre T1x et T2z:
                        ToolLine.Tangency(SSP, Npt, tabV);
                        V = tabV[k + 1];
                        T2 = V.Z();
                        for (j = 1; j <= Npol; j++)
                        {
                            Daij = DDA[j];
                            Cont[Inc3, j + IncCol] = Daij * T2;
                            Cont[Inc3, j + IP + 2 * Npol + IncCol] = -Daij * T1;
                        }
                        IncCol = IncCol + IP;
                    }
                }
                else if (Ibont[k, i] == 2)
                {
                    if (k <= nb3d)
                    {
                        ToolLine.Tangency(SSP, Npt, tabV);
                        V = tabV[k];
                        T1 = V.Y();
                        IP = 3 * Npol;
                    }
                    else
                    {
                        ToolLine.Tangency(SSP, Npt, tabV2d);
                        V2d = tabV2d[k];
                        T1 = V2d.Y();
                        IP = 2 * Npol;
                    }
                    if (Ibont[k + 1, i] == 1)
                    { // Relations entre T1y et T2x:
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(SSP, Npt, tabV);
                            V = tabV[k + 1];
                            T2 = V.X();
                        }
                        else
                        {
                            ToolLine.Tangency(SSP, Npt, tabV2d);
                            V2d = tabV2d[k + 1];
                            T2 = V2d.X();
                        }
                        for (j = 1; j <= Npol; j++)
                        {
                            Daij = DDA[j];
                            Cont[Inc3, j + Npol + IncCol] = Daij * T2;
                            Cont[Inc3, j + IP + IncCol] = -Daij * T1;
                        }
                        IncCol = IncCol + IP;
                    }
                    else if (Ibont[k + 1, i] == 2)
                    { // Relations entre T1y et T2y:
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(SSP, Npt, tabV);
                            V = tabV[k + 1];
                            T2 = V.Y();
                        }
                        else
                        {
                            ToolLine.Tangency(SSP, Npt, tabV2d);
                            V2d = tabV2d[k + 1];
                            T2 = V2d.Y();
                        }
                        for (j = 1; j <= Npol; j++)
                        {
                            Daij = DDA[j];
                            Cont[Inc3, j + Npol + IncCol] = Daij * T2;
                            Cont[Inc3, j + IP + Npol + IncCol] = -Daij * T1;
                        }
                        IncCol = IncCol + IP;
                    }
                    else if (Ibont[k + 1, i] == 3)
                    { // Relations entre T1y et T2z:
                        ToolLine.Tangency(SSP, Npt, tabV);
                        V = tabV[k + 1];
                        T2 = V.Z();
                        for (j = 1; j <= Npol; j++)
                        {
                            Daij = DDA[j];
                            Cont[Inc3, j + Npol + IncCol] = Daij * T2;
                            Cont[Inc3, j + IP + 2 * Npol + IncCol] = -Daij * T1;
                        }
                        IncCol = IncCol + IP;
                    }
                }

                else
                {
                    ToolLine.Tangency(SSP, Npt, tabV);
                    V = tabV[k];
                    T1 = V.Z();
                    IP = 3 * Npol;
                    if (Ibont[k + 1, i] == 1)
                    { // Relations entre T1z et T2x:
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(SSP, Npt, tabV);
                            V = tabV[k + 1];
                            T2 = V.X();
                        }
                        else
                        {
                            ToolLine.Tangency(SSP, Npt, tabV2d);
                            V2d = tabV2d[k + 1];
                            T2 = V2d.X();
                        }
                        for (j = 1; j <= Npol; j++)
                        {
                            Daij = DDA[j];
                            Cont[Inc3, j + 2 * Npol + IncCol] = Daij * T2;
                            Cont[Inc3, j + IP + IncCol] = -Daij * T1;
                        }
                        IncCol = IncCol + IP;
                    }

                    else if (Ibont[k + 1, i] == 2)
                    { // Relations entre T1z et T2y:
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(SSP, Npt, tabV);
                            V = tabV[k + 1];
                            T2 = V.Y();
                        }
                        else
                        {
                            ToolLine.Tangency(SSP, Npt, tabV2d);
                            V2d = tabV2d[k + 1];
                            T2 = V2d.Y();
                        }
                        for (j = 1; j <= Npol; j++)
                        {
                            Daij = DDA[j];
                            Cont[Inc3, j + 2 * Npol + IncCol] = Daij * T2;
                            Cont[Inc3, j + IP + Npol + IncCol] = -Daij * T1;
                        }
                        IncCol = IncCol + IP;
                    }

                    else if (Ibont[k + 1, i] == 3)
                    { // Relations entre T1z et T2z:
                        ToolLine.Tangency(SSP, Npt, tabV);
                        V = tabV[k + 1];
                        T2 = V.Z();
                        for (j = 1; j <= Npol; j++)
                        {
                            Daij = DDA[j];
                            Cont[Inc3, j + 2 * Npol + IncCol] = Daij * T2;
                            Cont[Inc3, j + IP + 2 * Npol + IncCol] = -Daij * T1;
                        }
                        IncCol = IncCol + IP;
                    }
                }
            }
        }

        return DeCont;
    }
}
