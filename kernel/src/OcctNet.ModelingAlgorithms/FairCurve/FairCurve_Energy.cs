// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Energy.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Energy.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Energy.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The energy of a fair curve as a function of its free variables (the free poles, the lengths and slides that the
/// end constraints leave, and the sliding when it is free), with its gradient and hessian.
/// </summary>
/// <remarks>
/// A working object (decision 21). It keeps the caller's pole array as OCCT keeps the handle, and each evaluation
/// writes the poles that correspond to the variables into it (<see cref="Poles"/>), as OCCT does: FairCurve_Batten
/// relies on it to read the optimised poles back (decision 859).
/// </remarks>
public abstract class FairCurve_Energy : MultipleVarFunctionWithHessian
{
    protected NCollection_HArray1<Point2D> MyPoles;
    protected int MyContrOrder1;
    protected int MyContrOrder2;
    protected bool MyWithAuxValue;
    protected int MyNbVar;

    private readonly int MyNbValues;
    private readonly NCollection_Array1<Xy> MyLinearForm;
    private readonly NCollection_Array1<Xy> MyQuadForm;
    private readonly math_Vector MyGradient;
    private readonly math_Vector MyHessian;

    /// <summary>The number of variables of the energy.</summary>
    public override int VariableCount
    {
        get
        {
            return MyNbVar;
        }
    }

    /// <summary>The poles (the caller's array, which the evaluations write).</summary>
    public NCollection_HArray1<Point2D> Poles()
    {
        return MyPoles;
    }

    protected int Indice(int i, int j)
    {
        return MyNbValues + j + i * (i - 1) / 2;
    }

    /// <summary>
    /// Angles are measured from the Ox axis. <paramref name="ContrOrder1"/> and <paramref name="ContrOrder2"/> are 0, 1
    /// or 2.
    /// </summary>
    protected FairCurve_Energy(NCollection_HArray1<Point2D> Poles,
                               int ContrOrder1,
                               int ContrOrder2,
                               bool WithAuxValue = false,
                               double Angle1 = 0,
                               double Angle2 = 0,
                               int Degree = 2,
                               double Curvature1 = 0,
                               double Curvature2 = 0)
    {
        MyPoles = Poles;
        MyContrOrder1 = ContrOrder1;
        MyContrOrder2 = ContrOrder2;
        MyWithAuxValue = WithAuxValue;
        MyNbVar = 2 * (Poles.Length() - 2) - ContrOrder2 - ContrOrder1 + (WithAuxValue ? 1 : 0);
        MyNbValues = 2 * Poles.Length() + (WithAuxValue ? 1 : 0);
        MyLinearForm = new NCollection_Array1<Xy>(0, 1);
        MyQuadForm = new NCollection_Array1<Xy>(0, 1);
        MyGradient = new math_Vector(0, MyNbValues);
        MyHessian = new math_Vector(0, MyNbValues + MyNbValues * (MyNbValues + 1) / 2);

        // chesk angles in reference (Ox,Oy)
        Xy L0 = new(Math.Cos(Angle1), Math.Sin(Angle1)), L1 = new(-Math.Cos(Angle2), Math.Sin(Angle2));
        MyLinearForm.SetValue(0, L0);
        MyLinearForm.SetValue(1, L1);
        Xy Q0 = new(-Math.Sin(Angle1), Math.Cos(Angle1)), Q1 = new(Math.Sin(Angle2), Math.Cos(Angle2));
        MyQuadForm.SetValue(0, ((double)Degree) / (Degree - 1) * Curvature1 * Q0);
        MyQuadForm.SetValue(1, ((double)Degree) / (Degree - 1) * Curvature2 * Q1);
    }

    /// <summary>Computes the energy <paramref name="E"/> at <paramref name="X"/>; false if it could not be computed.</summary>
    public override bool Value(math_Vector X, out double E)
    {
        bool IsDone;
        math_Vector Energie = new(0, 0);
        ComputePoles(X);
        IsDone = Compute(0, Energie);
        // OCCT: Energie(0) is unwritten when Compute stops on an infinite sliding (uninitialised natively, 0 here)
        E = Energie[0];
        return IsDone;
    }

    /// <summary>Computes the gradient <paramref name="G"/> at <paramref name="X"/>; false if it could not be computed.</summary>
    public override bool Gradient(math_Vector X, math_Vector G)
    {
        bool IsDone;
        double E;

        IsDone = Values(X, out E, G);
        return IsDone;
    }

    /// <summary>Makes the gradient vector <paramref name="Grad"/> from the intermediate format.</summary>
    protected void Gradient1(math_Vector Vect, math_Vector Grad)
    {
        int ii, DebG = Grad.Lower(), FinG = Grad.Upper();
        int Vdeb = 3, Vfin = 2 * MyPoles.Length() - 2;

        // .... by calculation
        if (MyContrOrder1 >= 1)
        {
            Xy DPole = new(Vect[Vdeb], Vect[Vdeb + 1]);
            Grad[DebG] = MyLinearForm[0] * DPole;
            Vdeb += 2;
            DebG += 1;
        }
        if (MyContrOrder1 == 2)
        {
            double Lambda0 =
              MyPoles.Value(MyPoles.Lower()).Distance(MyPoles.Value(MyPoles.Lower() + 1));
            Xy DPole = new(Vect[Vdeb], Vect[Vdeb + 1]);
            Grad[DebG - 1] += (MyLinearForm[0] + 2 * Lambda0 * MyQuadForm[0]) * DPole;
            Grad[DebG] = MyLinearForm[0] * DPole;
            Vdeb += 2;
            DebG += 1;
        }
        if (MyWithAuxValue)
        {
            Grad[FinG] = Vect[2 * MyPoles.Length() + 1];
            FinG -= 1;
        }
        if (MyContrOrder2 >= 1)
        {
            Xy DPole = new(Vect[Vfin - 1], Vect[Vfin]);
            Grad[FinG] = MyLinearForm[1] * DPole;
            FinG -= 1;
        }
        if (MyContrOrder2 == 2)
        {
            double Lambda1 =
              MyPoles.Value(MyPoles.Upper()).Distance(MyPoles.Value(MyPoles.Upper() - 1));
            Xy DPole = new(Vect[Vfin - 3], Vect[Vfin - 2]);
            Grad[FinG] = Grad[FinG + 1] + (MyLinearForm[1] + 2 * Lambda1 * MyQuadForm[1]) * DPole;
            Grad[FinG + 1] = MyLinearForm[1] * DPole;
            FinG -= 1;
        }
        // ... by recopy
        for (ii = DebG; ii <= FinG; ii++)
        {
            Grad[ii] = Vect[Vdeb];
            Vdeb += 1;
        }
    }

    /// <summary>
    /// Computes the energy <paramref name="E"/> and the gradient <paramref name="G"/> at <paramref name="X"/>; false if
    /// they could not be computed (E is then 0; OCCT leaves the caller's variable unchanged).
    /// </summary>
    public override bool Values(math_Vector X, out double E, math_Vector G)
    {
        bool IsDone;

        E = 0;
        ComputePoles(X);
        IsDone = Compute(1, MyGradient);
        if (IsDone)
        {
            E = MyGradient[0];
            Gradient1(MyGradient, G);
        }
        return IsDone;
    }

    /// <summary>
    /// Computes the energy <paramref name="E"/>, the gradient <paramref name="G"/> and the hessian
    /// <paramref name="H"/> at <paramref name="X"/>; false if they could not be computed (E is then 0; OCCT leaves the
    /// caller's variable unchanged).
    /// </summary>
    public override bool Values(math_Vector X, out double E, math_Vector G, Matrix H)
    {
        bool IsDone;

        E = 0;
        ComputePoles(X);
        IsDone = Compute(2, MyHessian);
        if (IsDone)
        {
            E = MyHessian[0];
            Gradient1(MyHessian, G);
            Hessian1(MyHessian, H);
        }
        return IsDone;
    }

    /// <summary>Makes the hessian matrix <paramref name="H"/> from the intermediate format.</summary>
    protected void Hessian1(math_Vector Vect, Matrix H)
    {

        int ii, jj, kk, Vk;
        int Vdeb = 3 + 2 * MyContrOrder1, Vfin = 2 * MyPoles.Length() - 2 * (MyContrOrder2 + 1),
            Vup = 2 * MyPoles.Length() + (MyWithAuxValue ? 1 : 0);
        int DebH = 1 + MyContrOrder1, FinH = MyNbVar - (MyWithAuxValue ? 1 : 0) - MyContrOrder2;
        double Cos0 = Math.Pow(MyLinearForm[0].X(), 2), Sin0 = Math.Pow(MyLinearForm[0].Y(), 2),
               CosSin0 = 2 * MyLinearForm[0].X() * MyLinearForm[0].Y(),
               Cos1 = Math.Pow(MyLinearForm[1].X(), 2), Sin1 = Math.Pow(MyLinearForm[1].Y(), 2),
               CosSin1 = 2 * MyLinearForm[1].X() * MyLinearForm[1].Y();
        double Lambda0 = 0, Lambda1 = 0;

        if (MyContrOrder1 >= 1)
        {
            Lambda0 = MyPoles.Value(MyPoles.Lower()).Distance(MyPoles.Value(MyPoles.Lower() + 1));
        }

        if (MyContrOrder2 >= 1)
        {
            Lambda1 = MyPoles.Value(MyPoles.Upper()).Distance(MyPoles.Value(MyPoles.Upper() - 1));
        }

        if (MyContrOrder1 >= 1)
        {

            // calculate the left lambda column --------------------------------

            jj = Vdeb - 2 * MyContrOrder1;
            kk = Indice(jj, jj);     // X2X2
            ii = Indice(jj + 1, jj); // X2Y2
            H[1, 1] = Cos0 * Vect[kk] + CosSin0 * Vect[ii] + Sin0 * Vect[ii + 1];

            if (MyContrOrder1 >= 2)
            {
                Xy Laux = (MyLinearForm[0] + 2 * Lambda0 * MyQuadForm[0]);
                jj = Vdeb - 2 * (MyContrOrder1 - 1);
                kk = Indice(jj, jj - 2);     // X1X2
                ii = Indice(jj + 1, jj - 2); // X1Y2
                Xy Aux = new(Vect[kk + 2], Vect[ii + 3]);

                H[1, 1] +=
                  2
                    * ((MyQuadForm[0].X() * Vect[5] + MyQuadForm[0].Y() * Vect[6])
                       + (Laux.X() * (MyLinearForm[0].X() * Vect[kk] + MyLinearForm[0].Y() * Vect[kk + 1])
                          + Laux.Y() * (MyLinearForm[0].X() * Vect[ii] + MyLinearForm[0].Y() * Vect[ii + 1]))
                       + Laux.X() * Laux.Y() * Vect[ii + 2])
                  + (Math.Pow(Laux.X(), 2) * Vect[kk + 2] + Math.Pow(Laux.Y(), 2) * Vect[ii + 3]);

                H[2, 1] = (Cos0 * Vect[kk] + CosSin0 * (Vect[ii] + Vect[kk + 1]) / 2 + Sin0 * Vect[ii + 1])
                          + Laux * MyLinearForm[0].Multiplied(Aux)
                          + (Laux.X() * MyLinearForm[0].Y() + Laux.Y() * MyLinearForm[0].X()) * Vect[ii + 2];
            }

            if (MyWithAuxValue)
            {
                kk = Indice(Vup, Vdeb - 2 * MyContrOrder1);
                H[MyNbVar, 1] = MyLinearForm[0].X() * Vect[kk] + MyLinearForm[0].Y() * Vect[kk + 1];
            }

            if (MyContrOrder2 >= 1)
            {
                // OCCT: the terms between the two ends' constraint variables are not 0 when every pole interacts
                H[MyNbVar - (MyWithAuxValue ? 1 : 0), 1] = 0; // correct if there are less than 3 nodes
                if (MyContrOrder2 == 2)
                {
                    H[MyNbVar - (MyWithAuxValue ? 1 : 0) - 1, 1] = 0;
                }
            }

            Vk = Vdeb;
            kk = Indice(Vk, Vdeb - 2 * MyContrOrder1);
            for (ii = DebH; ii <= FinH; ii++)
            {
                H[ii, 1] = MyLinearForm[0].X() * Vect[kk] + MyLinearForm[0].Y() * Vect[kk + 1];
                kk += Vk;
                Vk++;
            }
        }

        // calculate the left line mu ----------------------
        if (MyContrOrder1 >= 2)
        {
            jj = Vdeb - 2 * (MyContrOrder1 - 1);
            kk = Indice(jj, jj);     // X3X3
            ii = Indice(jj + 1, jj); // X3Y3
            H[2, 2] = Cos0 * Vect[kk] + CosSin0 * Vect[ii] + Sin0 * Vect[ii + 1];

            if (MyWithAuxValue)
            {
                kk = Indice(Vup, Vdeb - 2 * (MyContrOrder1 - 1));
                Xy Pole = new(Vect[kk], Vect[kk + 1]);
                H[MyNbVar, 1] += (MyLinearForm[0] + 2 * Lambda0 * MyQuadForm[0]) * Pole;
                H[MyNbVar, 2] = MyLinearForm[0].X() * Vect[kk] + MyLinearForm[0].Y() * Vect[kk + 1];
            }

            if (MyContrOrder2 >= 1)
            {
                // OCCT: as for the first column
                H[MyNbVar - (MyWithAuxValue ? 1 : 0), 2] = 0; // correct if there are less than 3 nodes
                if (MyContrOrder2 == 2)
                {
                    H[MyNbVar - (MyWithAuxValue ? 1 : 0) - 1, 2] = 0;
                }
            }
            Vk = Vdeb;

            double Xaux = (MyLinearForm[0] + 2 * Lambda0 * MyQuadForm[0]).X(),
                   Yaux = (MyLinearForm[0] + 2 * Lambda0 * MyQuadForm[0]).Y();

            kk = Indice(Vk, Vdeb - 2 * MyContrOrder1 + 2);
            for (ii = DebH; ii <= FinH; ii++)
            {
                H[ii, 2] = MyLinearForm[0].X() * Vect[kk] + MyLinearForm[0].Y() * Vect[kk + 1];
                H[ii, 1] += Xaux * Vect[kk] + Yaux * Vect[kk + 1];
                kk += Vk;
                Vk++;
            }
        }

        // calculate the right lambda line -----------------------
        if (MyContrOrder2 >= 1)
        {

            jj = FinH + 1;
            Vk = Vfin + 2 * MyContrOrder2 - 1;
            kk = Indice(Vk, Vdeb);
            for (ii = DebH; ii <= FinH; ii++)
            {
                H[jj, ii] = MyLinearForm[1].X() * Vect[kk] + MyLinearForm[1].Y() * Vect[kk + Vk];
                kk++;
            }

            kk = Indice(Vk, Vk);
            H[jj, jj] = Cos1 * Vect[kk] + CosSin1 * Vect[kk + Vk] + Sin1 * Vect[kk + Vk + 1];

            if (MyContrOrder2 >= 2)
            {
                // H(jj,jj) +=
                Xy Laux = (MyLinearForm[1] + 2 * Lambda1 * MyQuadForm[1]);
                jj = Vfin + 2 * MyContrOrder2 - 3;
                kk = Indice(jj + 2, jj); // Xn-1Xn-2
                ii = Indice(jj + 3, jj); // Yn-1Xn-2
                int ll = Indice(jj, jj);

                H[FinH + 1, FinH + 1] +=
                  2
                    * ((MyQuadForm[1].X() * Vect[jj] + MyQuadForm[1].Y() * Vect[jj + 1])
                       + (Laux.X() * (MyLinearForm[1].X() * Vect[kk] + MyLinearForm[1].Y() * Vect[ii])
                          + Laux.Y()
                              * (MyLinearForm[1].X() * Vect[kk + 1] + MyLinearForm[1].Y() * Vect[ii + 1]))
                       + Laux.X() * Laux.Y() * Vect[ll + jj])
                  + (Math.Pow(Laux.X(), 2) * Vect[ll] + Math.Pow(Laux.Y(), 2) * Vect[ll + jj + 1]);

                H[FinH + 2, FinH + 1] =
                  Cos1 * Vect[kk] + CosSin1 * (Vect[ii] + Vect[kk + 1]) / 2 + Sin1 * Vect[ii + 1];
                Xy Aux = new(Vect[ll], Vect[ll + jj + 1]);
                H[FinH + 2, FinH + 1] +=
                  Laux * MyLinearForm[1].Multiplied(Aux)
                  + (Laux.X() * MyLinearForm[1].Y() + Laux.Y() * MyLinearForm[1].X()) * Vect[ll + jj];
                //       H(FinH+2, FinH+1) = 0; // No better alternative. Bug in the previous expression
            }
        }

        // calculate the right line mu  -----------------------
        if (MyContrOrder2 >= 2)
        {
            jj = FinH + 2;
            Vk = Vfin + 2 * MyContrOrder2 - 3;
            kk = Indice(Vk, Vdeb);

            double Xaux = (MyLinearForm[1] + 2 * Lambda1 * MyQuadForm[1]).X(),
                   Yaux = (MyLinearForm[1] + 2 * Lambda1 * MyQuadForm[1]).Y();

            for (ii = DebH; ii <= FinH; ii++)
            {
                H[jj, ii] = MyLinearForm[1].X() * Vect[kk] + MyLinearForm[1].Y() * Vect[kk + Vk];
                // update the right line Lambda
                H[jj - 1, ii] += Xaux * Vect[kk] + Yaux * Vect[kk + Vk];
                kk++;
            }
            kk = Indice(Vk, Vk);
            ii = Indice(Vk + 1, Vk);
            H[jj, jj] = Cos1 * Vect[kk] + CosSin1 * Vect[ii] + Sin1 * Vect[ii + 1];
        }

        // calculate the Auxiliary Variable line -----------------------
        if (MyWithAuxValue)
        {

            kk = Indice(Vup, Vdeb);
            for (ii = DebH; ii <= FinH; ii++)
            {
                H[MyNbVar, ii] = Vect[kk];
                kk++;
            }

            if (MyContrOrder2 >= 1)
            {
                kk = Indice(Vup, Vfin + 2 * MyContrOrder2 - 1);
                H[MyNbVar, FinH + 1] = MyLinearForm[1].X() * Vect[kk] + MyLinearForm[1].Y() * Vect[kk + 1];
            }
            if (MyContrOrder2 >= 2)
            {
                kk = Indice(Vup, Vfin + 2 * MyContrOrder2 - 3);
                Xy Pole = new(Vect[kk], Vect[kk + 1]);
                H[MyNbVar, FinH + 1] += (MyLinearForm[1] + 2 * Lambda1 * MyQuadForm[1]) * Pole;
                H[MyNbVar, FinH + 2] = MyLinearForm[1] * Pole;
            }
            kk = Indice(Vup, Vup);
            H[H.UpperRow(), H.UpperRow()] = Vect[kk];
        }

        // recopy the internal block -----------------------------------

        kk = Indice(Vdeb, Vdeb);
        for (ii = DebH; ii <= FinH; ii++)
        {
            for (jj = DebH; jj <= ii; jj++)
            {
                H[ii, jj] = Vect[kk];
                kk++;
            }
            kk += Vdeb - 1;
        }
        // symmetry
        for (ii = H.LowerRow(); ii <= H.UpperRow(); ii++)
        {
            for (jj = ii + 1; jj <= H.UpperRow(); jj++)
            {
                H[ii, jj] = H[jj, ii];
            }
        }
    }

    /// <summary>Computes the variables <paramref name="X"/> that correspond to the poles.</summary>
    public virtual bool Variable(math_Vector X)
    {
        int ii, IndexDeb1 = MyPoles.Lower() + 1, IndexDeb2 = X.Lower(), IndexFin1 = MyPoles.Upper() - 1,
            IndexFin2 = X.Upper() - (MyWithAuxValue ? 1 : 0); //  decrease by 1 if the sliding is
                                                               // free as the last value of X is reserved.

        // calculation of variables of constraints
        if (MyContrOrder1 >= 1)
        {
            X[IndexDeb2] = MyPoles.Value(MyPoles.Lower()).Distance(MyPoles.Value(MyPoles.Lower() + 1));
            IndexDeb1 += 1;
            IndexDeb2 += 1;
        }
        if (MyContrOrder1 == 2)
        {
            Vector2D b1b2 = new(MyPoles.Value(MyPoles.Lower() + 1), MyPoles.Value(MyPoles.Lower() + 2));
            X[IndexDeb2] = b1b2.XY() * MyLinearForm[0];
            IndexDeb1 += 1;
            IndexDeb2 += 1;
        }
        if (MyContrOrder2 == 2)
        {
            Vector2D bn2bn1 = new(MyPoles.Value(MyPoles.Upper() - 2), MyPoles.Value(MyPoles.Upper() - 1));
            X[IndexFin2] = -bn2bn1.XY() * MyLinearForm[1];
            IndexFin1 -= 1;
            IndexFin2 -= 1;
        }
        if (MyContrOrder2 >= 1)
        {
            X[IndexFin2] = MyPoles.Value(MyPoles.Upper()).Distance(MyPoles.Value(MyPoles.Upper() - 1));
            IndexFin1 -= 1;
        }

        //  Recopy of auxiliary variables is not done in the abstract class

        // copy poles to variables
        for (ii = IndexDeb1; ii <= IndexFin1; ii++)
        {
            X[IndexDeb2] = MyPoles.Value(ii).X();
            X[IndexDeb2 + 1] = MyPoles.Value(ii).Y();
            IndexDeb2 += 2;
        }
        return true;
    }

    /// <summary>Computes the poles that correspond to the variables <paramref name="X"/> (written into the pole array).</summary>
    protected virtual void ComputePoles(math_Vector X)
    {
        int ii,
          IndexDeb1 = MyPoles.Lower() + 1, IndexDeb2 = X.Lower(), IndexFin1 = MyPoles.Upper() - 1,
          IndexFin2 = X.Upper() - (MyWithAuxValue ? 1 : 0); // decrease by 1 if the sliding is
                                                             // is free as the last value of X is reserved.
                                                             // calculation of pole constraints

        if (MyContrOrder1 >= 1)
        {
            IndexDeb1 += 1;
            IndexDeb2 += 1;
            ComputePolesG1(0,
                           X[1],
                           MyPoles.Value(MyPoles.Lower()),
                           out MyPoles.ChangeValue(MyPoles.Lower() + 1));
        }
        if (MyContrOrder1 == 2)
        {
            IndexDeb1 += 1;
            IndexDeb2 += 1;
            ComputePolesG2(0,
                           X[1],
                           X[2],
                           MyPoles.Value(MyPoles.Lower()),
                           out MyPoles.ChangeValue(MyPoles.Lower() + 2));
        }
        if (MyContrOrder2 == 2)
        {
            IndexFin1 -= 1;
            IndexFin2 -= 1;
            ComputePolesG2(1,
                           X[IndexFin2],
                           X[IndexFin2 + 1],
                           MyPoles.Value(MyPoles.Upper()),
                           out MyPoles.ChangeValue(MyPoles.Upper() - 2));
        }
        if (MyContrOrder2 >= 1)
        {
            IndexFin1 -= 1;
            ComputePolesG1(1,
                           X[IndexFin2],
                           MyPoles.Value(MyPoles.Upper()),
                           out MyPoles.ChangeValue(MyPoles.Upper() - 1));
        }

        //  if (MyWithAuxValue) { MyLengthSliding = X(X.Upper()); }
        // recopy others
        for (ii = IndexDeb1; ii <= IndexFin1; ii++)
        {
            MyPoles.ChangeValue(ii).SetX(X[IndexDeb2]);
            MyPoles.ChangeValue(ii).SetY(X[IndexDeb2 + 1]);
            IndexDeb2 += 2;
        }
    }

    /// <summary>Computes the pole that depends on the variables and the G1 constraint.</summary>
    protected void ComputePolesG1(int Side, double Lambda, in Point2D P1, out Point2D P2)
    {
        P2 = new Point2D(P1.XY() + MyLinearForm[Side] * Lambda);
    }

    /// <summary>Computes the pole that depends on the variables and the G2 constraint.</summary>
    protected void ComputePolesG2(int Side, double Lambda, double Rho, in Point2D P1, out Point2D P2)
    {
        P2 = new Point2D(P1.XY() + MyLinearForm[Side] * (Lambda + Rho) + MyQuadForm[Side] * (Lambda * Lambda));
    }

    /// <summary>Computes the energy (and its derivatives up to <paramref name="DerivativeOrder"/>) in the intermediate format.</summary>
    protected abstract bool Compute(int DerivativeOrder, math_Vector Result);
}
