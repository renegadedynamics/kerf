// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_LinearFlexion.hxx
// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_LinearFlexion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Engine.Numerics.Polynomials;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Approximation.FiniteElements;

/// <summary>
/// The linear flexion criterion of Hermite-Jacobi elements: the integral of the squared second derivative.
/// </summary>
public class LinearFlexion : ElementaryCriterion
{
    // OCCT keeps the integrals of the last constraint order in function-static variables, recomputed when the
    // order changes; they are cached here per order, under a lock (the values are the same).
    private const int WDeg = 14;
    private static readonly math_Vector?[] MatrixElemts = new math_Vector?[3];
    private static readonly Lock Lock = new();

    private readonly Matrix _refMatrix;
    private readonly int _order;

    /// <summary>The criterion for elements of degree at most <paramref name="workDegree"/> (at most 14).</summary>
    public LinearFlexion(int workDegree, Continuity constraintOrder)
    {
        _refMatrix = new Matrix(0, workDegree, 0, workDegree);
        _order = global::Kerf.Engine.Numerics.Polynomials.PLib.NivConstr(constraintOrder);

        // OCCT checks the degree only when it computes the integrals, and otherwise reads past them
        if (workDegree > WDeg)
        {
            throw new ConstructionErrorException("Degree too high");
        }

        math_Vector matrixElemts;
        lock (global::Kerf.Engine.Approximation.FiniteElements.LinearFlexion.Lock)
        {
            if (global::Kerf.Engine.Approximation.FiniteElements.LinearFlexion.MatrixElemts[_order] is not math_Vector cached)
            {
                // Calculating RefMatrix
                int derOrder = 2;
                HermitJacobi theBase = new(WDeg, constraintOrder);
                ElementsOfRefMatrix elem = new(theBase, derOrder);

                int maxDegree = WDeg + 1;
                math_IntegerVector order = new(1, 1, Min(4 * (maxDegree / 2 + 1), NumericTables.GaussPointsMax()));
                math_Vector lower = new(1, 1, -1.0), upper = new(1, 1, 1.0);

                GaussSetIntegration anInt = new(elem, lower, upper, order);

                cached = new math_Vector(0, ((WDeg + 2) * (WDeg + 1)) / 2 - 1);
                cached.Initialized(anInt.Value());
                global::Kerf.Engine.Approximation.FiniteElements.LinearFlexion.MatrixElemts[_order] = cached;
            }
            matrixElemts = cached;
        }

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
            throw new DomainErrorException("FEmTool_LinearFlexion::DependenceTable");
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

        double coeff = (_last - _first) / 2.0, cteh3 = 2.0 / Math.Pow(coeff, 3), mfact, jline;

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
            }
        }

        if (J < 0.0)
        {
            J = 0.0;
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

        double coeff = (_last - _first) / 2.0, cteh3 = 2.0 / Math.Pow(coeff, 3), mfact;
        int k1, k2, i, j;

        h.Init(0.0);

        // Unlike the tension and jerk criteria, H is indexed from 0 whatever its bounds (AppDef_LinearCriteria
        // always passes a 0-based matrix)
        for (i = 0; i <= degH; i++)
        {
            k1 = (i <= _order) ? i : i - _order - 1;
            mfact = Math.Pow(coeff, k1) * cteh3;
            // Hermite*Hermite part of matrix
            for (j = i; j <= degH; j++)
            {
                k2 = (j <= _order) ? j : j - _order - 1;
                h[i, j] = mfact * Math.Pow(coeff, k2) * _refMatrix[i, j];
                if (i != j)
                {
                    h[j, i] = h[i, j];
                }
            }
            // Hermite*Jacobi part of matrix
            for (j = degH + 1; j <= deg; j++)
            {
                h[i, j] = h[j, i] = mfact * _refMatrix[i, j];
            }
        }

        // Jacoby*Jacobi part of matrix
        for (i = degH + 1; i <= deg; i++)
        {
            for (j = i; j <= deg; j++)
            {
                h[i, j] = cteh3 * _refMatrix[i, j];
                if (i != j)
                {
                    h[j, i] = h[i, j];
                }
            }
        }
    }

    public override void Gradient(int dimension, math_Vector g)
    {
        NCollection_HArray2<double> coeff = _coeff!;
        if (dimension < coeff.LowerCol() || dimension > coeff.UpperCol())
        {
            throw new OutOfRangeException("FEmTool_LinearFlexion::Gradient");
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
