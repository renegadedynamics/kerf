// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_LinearTension.hxx
// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_LinearTension.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The linear tension criterion of Hermite-Jacobi elements: the integral of the squared first derivative.
/// </summary>
public class FEmTool_LinearTension : FEmTool_ElementaryCriterion
{
    // OCCT keeps the integrals of the last constraint order in function-static variables, recomputed when the
    // order changes; they are cached here per order, under a lock (the values are the same).
    private const int WDeg = 14;
    private static readonly math_Vector?[] theMatrixElemts = new math_Vector?[3];
    private static readonly Lock theLock = new();

    private readonly math_Matrix RefMatrix;
    private readonly int myOrder;

    /// <summary>The criterion for elements of degree at most <paramref name="WorkDegree"/> (at most 14).</summary>
    public FEmTool_LinearTension(int WorkDegree, GeomAbs_Shape ConstraintOrder)
    {
        RefMatrix = new math_Matrix(0, WorkDegree, 0, WorkDegree);
        myOrder = PLib.NivConstr(ConstraintOrder);

        // OCCT checks the degree only when it computes the integrals, and otherwise reads past them
        if (WorkDegree > WDeg)
        {
            throw new Standard_ConstructionError("Degree too high");
        }

        math_Vector MatrixElemts;
        lock (theLock)
        {
            if (theMatrixElemts[myOrder] is not math_Vector aCached)
            {
                // Calculating RefMatrix
                int DerOrder = 1;
                PLib_HermitJacobi theBase = new(WDeg, ConstraintOrder);
                FEmTool_ElementsOfRefMatrix Elem = new(theBase, DerOrder);

                int maxDegree = WDeg + 1;
                math_IntegerVector anOrder = new(1, 1, Min(4 * (maxDegree / 2 + 1), math.GaussPointsMax()));
                math_Vector Lower = new(1, 1, -1.0), Upper = new(1, 1, 1.0);

                math_GaussSetIntegration anInt = new(Elem, Lower, Upper, anOrder);

                aCached = new math_Vector(0, ((WDeg + 2) * (WDeg + 1)) / 2 - 1);
                aCached.Initialized(anInt.Value());
                theMatrixElemts[myOrder] = aCached;
            }
            MatrixElemts = aCached;
        }

        int i, j, ii, jj;
        for (ii = i = 0; i <= WorkDegree; i++)
        {
            RefMatrix[i, i] = MatrixElemts[ii];
            for (j = i + 1, jj = ii + 1; j <= WorkDegree; j++, jj++)
            {
                RefMatrix[j, i] = RefMatrix[i, j] = MatrixElemts[jj];
            }
            ii += WDeg + 1 - i;
        }
    }

    public override NCollection_HArray2<int> DependenceTable()
    {
        if (myCoeff is null)
        {
            throw new Standard_DomainError("FEmTool_LinearTension::DependenceTable");
        }

        NCollection_HArray2<int> DepTab =
            new(myCoeff.LowerCol(), myCoeff.UpperCol(), myCoeff.LowerCol(), myCoeff.UpperCol(), 0);
        int i;
        for (i = 1; i <= myCoeff.RowLength(); i++)
        {
            DepTab.SetValue(i, i, 1);
        }

        return DepTab;
    }

    public override double Value()
    {
        NCollection_HArray2<double> aCoeff = myCoeff!;
        int deg = Min(aCoeff.ColLength() - 1, RefMatrix.UpperRow()), i, j, j0 = aCoeff.LowerRow(),
            degH = Min(2 * myOrder + 1, deg), NbDim = aCoeff.RowLength(), dim;

        NCollection_Array2<double> NewCoeff = new(1, NbDim, 0, deg);

        double coeff = (myLast - myFirst) / 2.0, cteh3 = 2.0 / coeff, mfact, Jline;

        int k1;

        double J = 0.0;

        for (i = 0; i <= degH; i++)
        {
            k1 = (i <= myOrder) ? i : i - myOrder - 1;
            mfact = Math.Pow(coeff, k1);
            for (dim = 1; dim <= NbDim; dim++)
            {
                NewCoeff[dim, i] = aCoeff.Value(j0 + i, dim) * mfact;
            }
        }

        for (i = degH + 1; i <= deg; i++)
        {
            for (dim = 1; dim <= NbDim; dim++)
            {
                NewCoeff[dim, i] = aCoeff.Value(j0 + i, dim);
            }
        }

        for (dim = 1; dim <= NbDim; dim++)
        {
            for (i = 0; i <= deg; i++)
            {
                Jline = 0.5 * RefMatrix[i, i] * NewCoeff[dim, i];

                for (j = 0; j < i; j++)
                {
                    Jline += RefMatrix[i, j] * NewCoeff[dim, j];
                }

                J += Jline * NewCoeff[dim, i];
            }
        }

        return cteh3 * J;
    }

    public override void Hessian(int Dimension1, int Dimension2, math_Matrix H)
    {
        NCollection_HArray2<int> DepTab = DependenceTable();

        if (Dimension1 < DepTab.LowerRow() || Dimension1 > DepTab.UpperRow() || Dimension2 < DepTab.LowerCol()
            || Dimension2 > DepTab.UpperCol())
        {
            throw new Standard_OutOfRange("FEmTool_LinearTension::Hessian");
        }

        if (DepTab.Value(Dimension1, Dimension2) == 0)
        {
            throw new Standard_DomainError("FEmTool_LinearTension::Hessian");
        }

        int deg = Min(RefMatrix.UpperRow(), H.RowNumber() - 1), degH = Min(2 * myOrder + 1, deg);

        double coeff = (myLast - myFirst) / 2.0, cteh3 = 2.0 / coeff, mfact;
        int k1, k2, i, j, i0 = H.LowerRow(), j0 = H.LowerCol(), i1, j1;

        H.Init(0.0);

        i1 = i0;
        for (i = 0; i <= degH; i++)
        {
            k1 = (i <= myOrder) ? i : i - myOrder - 1;
            mfact = Math.Pow(coeff, k1) * cteh3;
            // Hermite*Hermite part of matrix
            j1 = j0 + i;
            for (j = i; j <= degH; j++)
            {
                k2 = (j <= myOrder) ? j : j - myOrder - 1;
                H[i1, j1] = mfact * Math.Pow(coeff, k2) * RefMatrix[i, j];
                if (i != j)
                {
                    H[j1, i1] = H[i1, j1];
                }
                j1++;
            }
            // Hermite*Jacobi part of matrix
            j1 = j0 + degH + 1;
            for (j = degH + 1; j <= deg; j++)
            {
                H[i1, j1] = mfact * RefMatrix[i, j];
                H[j1, i1] = H[i1, j1];
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
                H[i1, j1] = cteh3 * RefMatrix[i, j];
                if (i != j)
                {
                    H[j1, i1] = H[i1, j1];
                }
                j1++;
            }
            i1++;
        }
    }

    public override void Gradient(int Dimension, math_Vector G)
    {
        NCollection_HArray2<double> aCoeff = myCoeff!;
        if (Dimension < aCoeff.LowerCol() || Dimension > aCoeff.UpperCol())
        {
            throw new Standard_OutOfRange("FEmTool_LinearTension::Gradient");
        }

        int deg = Min(G.Length() - 1, aCoeff.ColLength() - 1);

        math_Vector X = new(0, deg);
        int i, i1 = aCoeff.LowerRow();
        for (i = 0; i <= deg; i++)
        {
            X[i] = aCoeff.Value(i1 + i, Dimension);
        }

        math_Matrix H = new(0, deg, 0, deg);
        Hessian(Dimension, Dimension, H);

        G.Multiply(H, X);
    }
}
