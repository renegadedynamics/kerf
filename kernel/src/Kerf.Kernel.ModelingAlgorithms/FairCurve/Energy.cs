// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Energy.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Energy.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Energy.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Fairing;

/// <summary>
/// The energy of a fair curve as a function of its free variables (the free poles, the lengths and slides that the
/// end constraints leave, and the sliding when it is free), with its gradient and hessian.
/// </summary>
/// <remarks>
/// A working object (decision 21). It keeps the caller's pole array as OCCT keeps the handle, and each evaluation
/// writes the poles that correspond to the variables into it (<see cref="Poles"/>), as OCCT does: FairCurve_Batten
/// relies on it to read the optimised poles back (decision 859).
/// </remarks>
public abstract class Energy : MultipleVarFunctionWithHessian
{
    protected NCollection_HArray1<Point2D> _myPoles;
    protected int _myContrOrder1;
    protected int _myContrOrder2;
    protected bool _myWithAuxValue;
    protected int _myNbVar;

    private readonly int _myNbValues;
    private readonly NCollection_Array1<Xy> _myLinearForm;
    private readonly NCollection_Array1<Xy> _myQuadForm;
    private readonly math_Vector _myGradient;
    private readonly math_Vector _myHessian;

    /// <summary>The number of variables of the energy.</summary>
    public override int VariableCount
    {
        get
        {
            return _myNbVar;
        }
    }

    /// <summary>The poles (the caller's array, which the evaluations write).</summary>
    public NCollection_HArray1<Point2D> Poles()
    {
        return _myPoles;
    }

    protected int Indice(int i, int j)
    {
        return _myNbValues + j + i * (i - 1) / 2;
    }

    /// <summary>
    /// Angles are measured from the Ox axis. <paramref name="contrOrder1"/> and <paramref name="contrOrder2"/> are 0, 1
    /// or 2.
    /// </summary>
    protected Energy(NCollection_HArray1<Point2D> poles,
                               int contrOrder1,
                               int contrOrder2,
                               bool withAuxValue = false,
                               double angle1 = 0,
                               double angle2 = 0,
                               int degree = 2,
                               double curvature1 = 0,
                               double curvature2 = 0)
    {
        _myPoles = poles;
        _myContrOrder1 = contrOrder1;
        _myContrOrder2 = contrOrder2;
        _myWithAuxValue = withAuxValue;
        _myNbVar = 2 * (poles.Length() - 2) - contrOrder2 - contrOrder1 + (withAuxValue ? 1 : 0);
        _myNbValues = 2 * poles.Length() + (withAuxValue ? 1 : 0);
        _myLinearForm = new NCollection_Array1<Xy>(0, 1);
        _myQuadForm = new NCollection_Array1<Xy>(0, 1);
        _myGradient = new math_Vector(0, _myNbValues);
        _myHessian = new math_Vector(0, _myNbValues + _myNbValues * (_myNbValues + 1) / 2);

        // chesk angles in reference (Ox,Oy)
        Xy l0 = new(Math.Cos(angle1), Math.Sin(angle1)), l1 = new(-Math.Cos(angle2), Math.Sin(angle2));
        _myLinearForm.SetValue(0, l0);
        _myLinearForm.SetValue(1, l1);
        Xy q0 = new(-Math.Sin(angle1), Math.Cos(angle1)), q1 = new(Math.Sin(angle2), Math.Cos(angle2));
        _myQuadForm.SetValue(0, ((double)degree) / (degree - 1) * curvature1 * q0);
        _myQuadForm.SetValue(1, ((double)degree) / (degree - 1) * curvature2 * q1);
    }

    /// <summary>Computes the energy <paramref name="e"/> at <paramref name="x"/>; false if it could not be computed.</summary>
    public override bool Value(math_Vector x, out double e)
    {
        bool isDone;
        math_Vector energie = new(0, 0);
        ComputePoles(x);
        isDone = Compute(0, energie);
        // OCCT: Energie(0) is unwritten when Compute stops on an infinite sliding (uninitialised natively, 0 here)
        e = energie[0];
        return isDone;
    }

    /// <summary>Computes the gradient <paramref name="g"/> at <paramref name="x"/>; false if it could not be computed.</summary>
    public override bool Gradient(math_Vector x, math_Vector g)
    {
        bool isDone;
        double e;

        isDone = Values(x, out e, g);
        return isDone;
    }

    /// <summary>Makes the gradient vector <paramref name="grad"/> from the intermediate format.</summary>
    protected void Gradient1(math_Vector vect, math_Vector grad)
    {
        int ii, debG = grad.Lower(), finG = grad.Upper();
        int vdeb = 3, vfin = 2 * _myPoles.Length() - 2;

        // .... by calculation
        if (_myContrOrder1 >= 1)
        {
            Xy dPole = new(vect[vdeb], vect[vdeb + 1]);
            grad[debG] = _myLinearForm[0] * dPole;
            vdeb += 2;
            debG += 1;
        }
        if (_myContrOrder1 == 2)
        {
            double lambda0 =
              _myPoles.Value(_myPoles.Lower()).Distance(_myPoles.Value(_myPoles.Lower() + 1));
            Xy dPole = new(vect[vdeb], vect[vdeb + 1]);
            grad[debG - 1] += (_myLinearForm[0] + 2 * lambda0 * _myQuadForm[0]) * dPole;
            grad[debG] = _myLinearForm[0] * dPole;
            vdeb += 2;
            debG += 1;
        }
        if (_myWithAuxValue)
        {
            grad[finG] = vect[2 * _myPoles.Length() + 1];
            finG -= 1;
        }
        if (_myContrOrder2 >= 1)
        {
            Xy dPole = new(vect[vfin - 1], vect[vfin]);
            grad[finG] = _myLinearForm[1] * dPole;
            finG -= 1;
        }
        if (_myContrOrder2 == 2)
        {
            double lambda1 =
              _myPoles.Value(_myPoles.Upper()).Distance(_myPoles.Value(_myPoles.Upper() - 1));
            Xy dPole = new(vect[vfin - 3], vect[vfin - 2]);
            grad[finG] = grad[finG + 1] + (_myLinearForm[1] + 2 * lambda1 * _myQuadForm[1]) * dPole;
            grad[finG + 1] = _myLinearForm[1] * dPole;
            finG -= 1;
        }
        // ... by recopy
        for (ii = debG; ii <= finG; ii++)
        {
            grad[ii] = vect[vdeb];
            vdeb += 1;
        }
    }

    /// <summary>
    /// Computes the energy <paramref name="e"/> and the gradient <paramref name="g"/> at <paramref name="x"/>; false if
    /// they could not be computed (E is then 0; OCCT leaves the caller's variable unchanged).
    /// </summary>
    public override bool Values(math_Vector x, out double e, math_Vector g)
    {
        bool isDone;

        e = 0;
        ComputePoles(x);
        isDone = Compute(1, _myGradient);
        if (isDone)
        {
            e = _myGradient[0];
            Gradient1(_myGradient, g);
        }
        return isDone;
    }

    /// <summary>
    /// Computes the energy <paramref name="e"/>, the gradient <paramref name="g"/> and the hessian
    /// <paramref name="h"/> at <paramref name="x"/>; false if they could not be computed (E is then 0; OCCT leaves the
    /// caller's variable unchanged).
    /// </summary>
    public override bool Values(math_Vector x, out double e, math_Vector g, Matrix h)
    {
        bool isDone;

        e = 0;
        ComputePoles(x);
        isDone = Compute(2, _myHessian);
        if (isDone)
        {
            e = _myHessian[0];
            Gradient1(_myHessian, g);
            Hessian1(_myHessian, h);
        }
        return isDone;
    }

    /// <summary>Makes the hessian matrix <paramref name="h"/> from the intermediate format.</summary>
    protected void Hessian1(math_Vector vect, Matrix h)
    {

        int ii, jj, kk, vk;
        int vdeb = 3 + 2 * _myContrOrder1, vfin = 2 * _myPoles.Length() - 2 * (_myContrOrder2 + 1),
            vup = 2 * _myPoles.Length() + (_myWithAuxValue ? 1 : 0);
        int debH = 1 + _myContrOrder1, finH = _myNbVar - (_myWithAuxValue ? 1 : 0) - _myContrOrder2;
        double cos0 = Math.Pow(_myLinearForm[0].X(), 2), sin0 = Math.Pow(_myLinearForm[0].Y(), 2),
               cosSin0 = 2 * _myLinearForm[0].X() * _myLinearForm[0].Y(),
               cos1 = Math.Pow(_myLinearForm[1].X(), 2), sin1 = Math.Pow(_myLinearForm[1].Y(), 2),
               cosSin1 = 2 * _myLinearForm[1].X() * _myLinearForm[1].Y();
        double lambda0 = 0, lambda1 = 0;

        if (_myContrOrder1 >= 1)
        {
            lambda0 = _myPoles.Value(_myPoles.Lower()).Distance(_myPoles.Value(_myPoles.Lower() + 1));
        }

        if (_myContrOrder2 >= 1)
        {
            lambda1 = _myPoles.Value(_myPoles.Upper()).Distance(_myPoles.Value(_myPoles.Upper() - 1));
        }

        if (_myContrOrder1 >= 1)
        {

            // calculate the left lambda column --------------------------------

            jj = vdeb - 2 * _myContrOrder1;
            kk = Indice(jj, jj);     // X2X2
            ii = Indice(jj + 1, jj); // X2Y2
            h[1, 1] = cos0 * vect[kk] + cosSin0 * vect[ii] + sin0 * vect[ii + 1];

            if (_myContrOrder1 >= 2)
            {
                Xy laux = (_myLinearForm[0] + 2 * lambda0 * _myQuadForm[0]);
                jj = vdeb - 2 * (_myContrOrder1 - 1);
                kk = Indice(jj, jj - 2);     // X1X2
                ii = Indice(jj + 1, jj - 2); // X1Y2
                Xy aux = new(vect[kk + 2], vect[ii + 3]);

                h[1, 1] +=
                  2
                    * ((_myQuadForm[0].X() * vect[5] + _myQuadForm[0].Y() * vect[6])
                       + (laux.X() * (_myLinearForm[0].X() * vect[kk] + _myLinearForm[0].Y() * vect[kk + 1])
                          + laux.Y() * (_myLinearForm[0].X() * vect[ii] + _myLinearForm[0].Y() * vect[ii + 1]))
                       + laux.X() * laux.Y() * vect[ii + 2])
                  + (Math.Pow(laux.X(), 2) * vect[kk + 2] + Math.Pow(laux.Y(), 2) * vect[ii + 3]);

                h[2, 1] = (cos0 * vect[kk] + cosSin0 * (vect[ii] + vect[kk + 1]) / 2 + sin0 * vect[ii + 1])
                          + laux * _myLinearForm[0].Multiplied(aux)
                          + (laux.X() * _myLinearForm[0].Y() + laux.Y() * _myLinearForm[0].X()) * vect[ii + 2];
            }

            if (_myWithAuxValue)
            {
                kk = Indice(vup, vdeb - 2 * _myContrOrder1);
                h[_myNbVar, 1] = _myLinearForm[0].X() * vect[kk] + _myLinearForm[0].Y() * vect[kk + 1];
            }

            if (_myContrOrder2 >= 1)
            {
                // OCCT: the terms between the two ends' constraint variables are not 0 when every pole interacts
                h[_myNbVar - (_myWithAuxValue ? 1 : 0), 1] = 0; // correct if there are less than 3 nodes
                if (_myContrOrder2 == 2)
                {
                    h[_myNbVar - (_myWithAuxValue ? 1 : 0) - 1, 1] = 0;
                }
            }

            vk = vdeb;
            kk = Indice(vk, vdeb - 2 * _myContrOrder1);
            for (ii = debH; ii <= finH; ii++)
            {
                h[ii, 1] = _myLinearForm[0].X() * vect[kk] + _myLinearForm[0].Y() * vect[kk + 1];
                kk += vk;
                vk++;
            }
        }

        // calculate the left line mu ----------------------
        if (_myContrOrder1 >= 2)
        {
            jj = vdeb - 2 * (_myContrOrder1 - 1);
            kk = Indice(jj, jj);     // X3X3
            ii = Indice(jj + 1, jj); // X3Y3
            h[2, 2] = cos0 * vect[kk] + cosSin0 * vect[ii] + sin0 * vect[ii + 1];

            if (_myWithAuxValue)
            {
                kk = Indice(vup, vdeb - 2 * (_myContrOrder1 - 1));
                Xy pole = new(vect[kk], vect[kk + 1]);
                h[_myNbVar, 1] += (_myLinearForm[0] + 2 * lambda0 * _myQuadForm[0]) * pole;
                h[_myNbVar, 2] = _myLinearForm[0].X() * vect[kk] + _myLinearForm[0].Y() * vect[kk + 1];
            }

            if (_myContrOrder2 >= 1)
            {
                // OCCT: as for the first column
                h[_myNbVar - (_myWithAuxValue ? 1 : 0), 2] = 0; // correct if there are less than 3 nodes
                if (_myContrOrder2 == 2)
                {
                    h[_myNbVar - (_myWithAuxValue ? 1 : 0) - 1, 2] = 0;
                }
            }
            vk = vdeb;

            double xaux = (_myLinearForm[0] + 2 * lambda0 * _myQuadForm[0]).X(),
                   yaux = (_myLinearForm[0] + 2 * lambda0 * _myQuadForm[0]).Y();

            kk = Indice(vk, vdeb - 2 * _myContrOrder1 + 2);
            for (ii = debH; ii <= finH; ii++)
            {
                h[ii, 2] = _myLinearForm[0].X() * vect[kk] + _myLinearForm[0].Y() * vect[kk + 1];
                h[ii, 1] += xaux * vect[kk] + yaux * vect[kk + 1];
                kk += vk;
                vk++;
            }
        }

        // calculate the right lambda line -----------------------
        if (_myContrOrder2 >= 1)
        {

            jj = finH + 1;
            vk = vfin + 2 * _myContrOrder2 - 1;
            kk = Indice(vk, vdeb);
            for (ii = debH; ii <= finH; ii++)
            {
                h[jj, ii] = _myLinearForm[1].X() * vect[kk] + _myLinearForm[1].Y() * vect[kk + vk];
                kk++;
            }

            kk = Indice(vk, vk);
            h[jj, jj] = cos1 * vect[kk] + cosSin1 * vect[kk + vk] + sin1 * vect[kk + vk + 1];

            if (_myContrOrder2 >= 2)
            {
                // H(jj,jj) +=
                Xy laux = (_myLinearForm[1] + 2 * lambda1 * _myQuadForm[1]);
                jj = vfin + 2 * _myContrOrder2 - 3;
                kk = Indice(jj + 2, jj); // Xn-1Xn-2
                ii = Indice(jj + 3, jj); // Yn-1Xn-2
                int ll = Indice(jj, jj);

                h[finH + 1, finH + 1] +=
                  2
                    * ((_myQuadForm[1].X() * vect[jj] + _myQuadForm[1].Y() * vect[jj + 1])
                       + (laux.X() * (_myLinearForm[1].X() * vect[kk] + _myLinearForm[1].Y() * vect[ii])
                          + laux.Y()
                              * (_myLinearForm[1].X() * vect[kk + 1] + _myLinearForm[1].Y() * vect[ii + 1]))
                       + laux.X() * laux.Y() * vect[ll + jj])
                  + (Math.Pow(laux.X(), 2) * vect[ll] + Math.Pow(laux.Y(), 2) * vect[ll + jj + 1]);

                h[finH + 2, finH + 1] =
                  cos1 * vect[kk] + cosSin1 * (vect[ii] + vect[kk + 1]) / 2 + sin1 * vect[ii + 1];
                Xy aux = new(vect[ll], vect[ll + jj + 1]);
                h[finH + 2, finH + 1] +=
                  laux * _myLinearForm[1].Multiplied(aux)
                  + (laux.X() * _myLinearForm[1].Y() + laux.Y() * _myLinearForm[1].X()) * vect[ll + jj];
                //       H(FinH+2, FinH+1) = 0; // No better alternative. Bug in the previous expression
            }
        }

        // calculate the right line mu  -----------------------
        if (_myContrOrder2 >= 2)
        {
            jj = finH + 2;
            vk = vfin + 2 * _myContrOrder2 - 3;
            kk = Indice(vk, vdeb);

            double xaux = (_myLinearForm[1] + 2 * lambda1 * _myQuadForm[1]).X(),
                   yaux = (_myLinearForm[1] + 2 * lambda1 * _myQuadForm[1]).Y();

            for (ii = debH; ii <= finH; ii++)
            {
                h[jj, ii] = _myLinearForm[1].X() * vect[kk] + _myLinearForm[1].Y() * vect[kk + vk];
                // update the right line Lambda
                h[jj - 1, ii] += xaux * vect[kk] + yaux * vect[kk + vk];
                kk++;
            }
            kk = Indice(vk, vk);
            ii = Indice(vk + 1, vk);
            h[jj, jj] = cos1 * vect[kk] + cosSin1 * vect[ii] + sin1 * vect[ii + 1];
        }

        // calculate the Auxiliary Variable line -----------------------
        if (_myWithAuxValue)
        {

            kk = Indice(vup, vdeb);
            for (ii = debH; ii <= finH; ii++)
            {
                h[_myNbVar, ii] = vect[kk];
                kk++;
            }

            if (_myContrOrder2 >= 1)
            {
                kk = Indice(vup, vfin + 2 * _myContrOrder2 - 1);
                h[_myNbVar, finH + 1] = _myLinearForm[1].X() * vect[kk] + _myLinearForm[1].Y() * vect[kk + 1];
            }
            if (_myContrOrder2 >= 2)
            {
                kk = Indice(vup, vfin + 2 * _myContrOrder2 - 3);
                Xy pole = new(vect[kk], vect[kk + 1]);
                h[_myNbVar, finH + 1] += (_myLinearForm[1] + 2 * lambda1 * _myQuadForm[1]) * pole;
                h[_myNbVar, finH + 2] = _myLinearForm[1] * pole;
            }
            kk = Indice(vup, vup);
            h[h.UpperRow(), h.UpperRow()] = vect[kk];
        }

        // recopy the internal block -----------------------------------

        kk = Indice(vdeb, vdeb);
        for (ii = debH; ii <= finH; ii++)
        {
            for (jj = debH; jj <= ii; jj++)
            {
                h[ii, jj] = vect[kk];
                kk++;
            }
            kk += vdeb - 1;
        }
        // symmetry
        for (ii = h.LowerRow(); ii <= h.UpperRow(); ii++)
        {
            for (jj = ii + 1; jj <= h.UpperRow(); jj++)
            {
                h[ii, jj] = h[jj, ii];
            }
        }
    }

    /// <summary>Computes the variables <paramref name="x"/> that correspond to the poles.</summary>
    public virtual bool Variable(math_Vector x)
    {
        int ii, indexDeb1 = _myPoles.Lower() + 1, indexDeb2 = x.Lower(), indexFin1 = _myPoles.Upper() - 1,
            indexFin2 = x.Upper() - (_myWithAuxValue ? 1 : 0); //  decrease by 1 if the sliding is
                                                               // free as the last value of X is reserved.

        // calculation of variables of constraints
        if (_myContrOrder1 >= 1)
        {
            x[indexDeb2] = _myPoles.Value(_myPoles.Lower()).Distance(_myPoles.Value(_myPoles.Lower() + 1));
            indexDeb1 += 1;
            indexDeb2 += 1;
        }
        if (_myContrOrder1 == 2)
        {
            Vector2D b1b2 = new(_myPoles.Value(_myPoles.Lower() + 1), _myPoles.Value(_myPoles.Lower() + 2));
            x[indexDeb2] = b1b2.XY() * _myLinearForm[0];
            indexDeb1 += 1;
            indexDeb2 += 1;
        }
        if (_myContrOrder2 == 2)
        {
            Vector2D bn2bn1 = new(_myPoles.Value(_myPoles.Upper() - 2), _myPoles.Value(_myPoles.Upper() - 1));
            x[indexFin2] = -bn2bn1.XY() * _myLinearForm[1];
            indexFin1 -= 1;
            indexFin2 -= 1;
        }
        if (_myContrOrder2 >= 1)
        {
            x[indexFin2] = _myPoles.Value(_myPoles.Upper()).Distance(_myPoles.Value(_myPoles.Upper() - 1));
            indexFin1 -= 1;
        }

        //  Recopy of auxiliary variables is not done in the abstract class

        // copy poles to variables
        for (ii = indexDeb1; ii <= indexFin1; ii++)
        {
            x[indexDeb2] = _myPoles.Value(ii).X();
            x[indexDeb2 + 1] = _myPoles.Value(ii).Y();
            indexDeb2 += 2;
        }
        return true;
    }

    /// <summary>Computes the poles that correspond to the variables <paramref name="x"/> (written into the pole array).</summary>
    protected virtual void ComputePoles(math_Vector x)
    {
        int ii,
          indexDeb1 = _myPoles.Lower() + 1, indexDeb2 = x.Lower(), indexFin1 = _myPoles.Upper() - 1,
          indexFin2 = x.Upper() - (_myWithAuxValue ? 1 : 0); // decrease by 1 if the sliding is
                                                             // is free as the last value of X is reserved.
                                                             // calculation of pole constraints

        if (_myContrOrder1 >= 1)
        {
            indexDeb1 += 1;
            indexDeb2 += 1;
            ComputePolesG1(0,
                           x[1],
                           _myPoles.Value(_myPoles.Lower()),
                           out _myPoles.ChangeValue(_myPoles.Lower() + 1));
        }
        if (_myContrOrder1 == 2)
        {
            indexDeb1 += 1;
            indexDeb2 += 1;
            ComputePolesG2(0,
                           x[1],
                           x[2],
                           _myPoles.Value(_myPoles.Lower()),
                           out _myPoles.ChangeValue(_myPoles.Lower() + 2));
        }
        if (_myContrOrder2 == 2)
        {
            indexFin1 -= 1;
            indexFin2 -= 1;
            ComputePolesG2(1,
                           x[indexFin2],
                           x[indexFin2 + 1],
                           _myPoles.Value(_myPoles.Upper()),
                           out _myPoles.ChangeValue(_myPoles.Upper() - 2));
        }
        if (_myContrOrder2 >= 1)
        {
            indexFin1 -= 1;
            ComputePolesG1(1,
                           x[indexFin2],
                           _myPoles.Value(_myPoles.Upper()),
                           out _myPoles.ChangeValue(_myPoles.Upper() - 1));
        }

        //  if (MyWithAuxValue) { MyLengthSliding = X(X.Upper()); }
        // recopy others
        for (ii = indexDeb1; ii <= indexFin1; ii++)
        {
            _myPoles.ChangeValue(ii).SetX(x[indexDeb2]);
            _myPoles.ChangeValue(ii).SetY(x[indexDeb2 + 1]);
            indexDeb2 += 2;
        }
    }

    /// <summary>Computes the pole that depends on the variables and the G1 constraint.</summary>
    protected void ComputePolesG1(int side, double lambda, in Point2D p1, out Point2D p2)
    {
        p2 = new Point2D(p1.XY() + _myLinearForm[side] * lambda);
    }

    /// <summary>Computes the pole that depends on the variables and the G2 constraint.</summary>
    protected void ComputePolesG2(int side, double lambda, double rho, in Point2D p1, out Point2D p2)
    {
        p2 = new Point2D(p1.XY() + _myLinearForm[side] * (lambda + rho) + _myQuadForm[side] * (lambda * lambda));
    }

    /// <summary>Computes the energy (and its derivatives up to <paramref name="derivativeOrder"/>) in the intermediate format.</summary>
    protected abstract bool Compute(int derivativeOrder, math_Vector result);
}
