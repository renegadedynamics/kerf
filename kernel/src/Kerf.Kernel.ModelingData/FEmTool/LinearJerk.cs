// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_LinearJerk.hxx
// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_LinearJerk.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Engine.Numerics.Polynomials;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Approximation.FiniteElements;

/// <summary>
/// The linear jerk criterion of Hermite-Jacobi elements: the integral of the squared third derivative.
/// </summary>
internal class LinearJerk : ElementaryCriterion
{
    // OCCT keeps the integrals of the last constraint order in function-static variables, recomputed when the
    // order changes; they are cached here per order, published without a lock (the values are the same).
    private const int WDeg = 14;
    private static readonly math_Vector?[] MatrixElemts = new math_Vector?[3];

    private readonly Matrix _refMatrix;
    private readonly int _order;

    /// <summary>The criterion for elements of degree at most <paramref name="workDegree"/> (at most 14).</summary>
    public LinearJerk(int workDegree, Continuity constraintOrder)
    {
        _refMatrix = new Matrix(0, workDegree, 0, workDegree);
        _order = global::Kerf.Engine.Numerics.Polynomials.PLib.NivConstr(constraintOrder);

        // OCCT checks the degree only when it computes the integrals, and otherwise reads past them
        if (workDegree > WDeg)
        {
            throw new ConstructionErrorException("Degree too high");
        }

        math_Vector matrixElemts;
        math_Vector? cached = Volatile.Read(ref MatrixElemts[_order]);
        if (cached is null)
        {
            // Calculating RefMatrix
            int derOrder = 3;
            HermitJacobi theBase = new(WDeg, constraintOrder);
            ElementsOfRefMatrix elem = new(theBase, derOrder);

            int maxDegree = WDeg + 1;
            math_IntegerVector order = new(1, 1, Min(4 * (maxDegree / 2 + 1), NumericTables.GaussPointsMax()));
            math_Vector lower = new(1, 1, -1.0), upper = new(1, 1, 1.0);

            GaussSetIntegration anInt = new(elem, lower, upper, order);

            cached = new math_Vector(0, ((WDeg + 2) * (WDeg + 1)) / 2 - 1);
            cached.Initialized(anInt.Value());
            // computed twice at worst, by threads racing on the first use; the values are the same
            cached = Interlocked.CompareExchange(ref MatrixElemts[_order], cached, null) ?? cached;
        }
        matrixElemts = cached;

        int i, j, ii, jj;
        for (ii = i = 0; i <= workDegree; i++)
        {
            _refMatrix[i, i] = matrixElemts[ii];
            for (j = i + 1, jj = ii + 1; j <= workDegree; j++, jj++)
            {
                _refMatrix[j, i] = _refMatrix[i, j] = matrixElemts[jj];
            }
            ii += WDeg + 1 - i;
        }
    }

    public override NCollection_HArray2<int> DependenceTable()
    {
        if (_coeff is null)
        {
            throw new DomainErrorException("FEmTool_LinearJerk::DependenceTable");
        }

        NCollection_HArray2<int> depTab =
            new(_coeff.LowerCol(), _coeff.UpperCol(), _coeff.LowerCol(), _coeff.UpperCol(), 0);
        int i;
        for (i = _coeff.LowerCol(); i <= _coeff.UpperCol(); i++)
        {
            depTab.SetValue(i, i, 1);
        }

        return depTab;
    }

    public override double Value()
    {
        NCollection_HArray2<double> aCoeff = _coeff!;
        int deg = Min(aCoeff.ColLength() - 1, _refMatrix.UpperRow()), i, j, j0 = aCoeff.LowerRow(),
            degH = Min(2 * _order + 1, deg), nbDim = aCoeff.RowLength(), dim;

        NCollection_Array2<double> newCoeff = new(1, nbDim, 0, deg);

        double coeff = (_last - _first) / 2.0, cteh3 = 2.0 / Math.Pow(coeff, 5), mfact, jline;

        int k1;

        double J = 0.0;

        for (i = 0; i <= degH; i++)
        {
            k1 = (i <= _order) ? i : i - _order - 1;
            mfact = Math.Pow(coeff, k1);
            for (dim = 1; dim <= nbDim; dim++)
            {
                newCoeff[dim, i] = aCoeff.Value(j0 + i, dim) * mfact;
            }
        }

        for (i = degH + 1; i <= deg; i++)
        {
            for (dim = 1; dim <= nbDim; dim++)
            {
                newCoeff[dim, i] = aCoeff.Value(j0 + i, dim);
            }
        }

        for (dim = 1; dim <= nbDim; dim++)
        {
            for (i = 0; i <= deg; i++)
            {
                jline = 0.5 * _refMatrix[i, i] * newCoeff[dim, i];

                for (j = 0; j < i; j++)
                {
                    jline += _refMatrix[i, j] * newCoeff[dim, j];
                }

                J += jline * newCoeff[dim, i];
                if (J < 0.0)
                {
                    J = 0.0;
                }
            }
        }

        return cteh3 * J;
    }

    public override void Hessian(int dimension1, int dimension2, Matrix h)
    {
        NCollection_HArray2<int> depTab = DependenceTable();

        if (dimension1 < depTab.LowerRow() || dimension1 > depTab.UpperRow() || dimension2 < depTab.LowerCol()
            || dimension2 > depTab.UpperCol())
        {
            throw new OutOfRangeException("FEmTool_LinearJerk::Hessian");
        }

        if (depTab.Value(dimension1, dimension2) == 0)
        {
            throw new DomainErrorException("FEmTool_LinearJerk::Hessian");
        }

        int deg = Min(_refMatrix.UpperRow(), h.RowNumber() - 1), degH = Min(2 * _order + 1, deg);

        double coeff = (_last - _first) / 2.0, cteh3 = 2.0 / Math.Pow(coeff, 5), mfact;
        int k1, k2, i, j, i0 = h.LowerRow(), j0 = h.LowerCol(), i1, j1;

        h.Init(0.0);

        i1 = i0;
        for (i = 0; i <= degH; i++)
        {
            k1 = (i <= _order) ? i : i - _order - 1;
            mfact = Math.Pow(coeff, k1) * cteh3;
            // Hermite*Hermite part of matrix
            j1 = j0 + i;
            for (j = i; j <= degH; j++)
            {
                k2 = (j <= _order) ? j : j - _order - 1;
                h[i1, j1] = mfact * Math.Pow(coeff, k2) * _refMatrix[i, j];
                if (i != j)
                {
                    h[j1, i1] = h[i1, j1];
                }
                j1++;
            }
            // Hermite*Jacobi part of matrix
            j1 = j0 + degH + 1;
            for (j = degH + 1; j <= deg; j++)
            {
                h[i1, j1] = mfact * _refMatrix[i, j];
                h[j1, i1] = h[i1, j1];
                j1++;
            }
            i1++;
        }

        // Jacoby*Jacobi part of matrix
        i1 = i0 + degH + 1;
        for (i = degH + 1; i <= deg; i++)
        {
            j1 = j0 + i;
            for (j = i; j <= deg; j++)
            {
                h[i1, j1] = cteh3 * _refMatrix[i, j];
                if (i != j)
                {
                    h[j1, i1] = h[i1, j1];
                }
                j1++;
            }
            i1++;
        }
    }

    public override void Gradient(int dimension, math_Vector g)
    {
        NCollection_HArray2<double> coeff = _coeff!;
        if (dimension < coeff.LowerCol() || dimension > coeff.UpperCol())
        {
            throw new OutOfRangeException("FEmTool_LinearJerk::Gradient");
        }

        int deg = Min(g.Length() - 1, coeff.ColLength() - 1);

        math_Vector x = new(0, deg);
        int i, i1 = coeff.LowerRow();
        for (i = 0; i <= deg; i++)
        {
            x[i] = coeff.Value(i1 + i, dimension);
        }

        Matrix h = new(0, deg, 0, deg);
        Hessian(dimension, dimension, h);

        g.Multiply(h, x);
    }
}
