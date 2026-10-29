// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// FEmTool_Curve, the linear criteria and FEmTool_ElementsOfRefMatrix on cubics, which the Hermite part of a
/// C1 Hermite-Jacobi element represents exactly.
/// </summary>
public class FEmTool_CurveTests
{
    // P(U) = (U^3 - U, 2U^2 + 1) and its derivatives
    private static double[] P(double u) => [u * u * u - u, 2 * u * u + 1];

    private static double[] DP(double u) => [3 * u * u - 1, 4 * u];

    private static double[] D2P(double u) => [6 * u, 4];

    private const int WorkDegree = 7;

    /// <summary>The C1 Hermite-Jacobi coefficients of P on [a, b]: values and derivatives at the ends.</summary>
    private static NCollection_Array2<double> Element(double a, double b, int theLowerRow = 1, int theLowerCol = 1)
    {
        NCollection_Array2<double> aCoeffs = new(theLowerRow, theLowerRow + WorkDegree, theLowerCol, theLowerCol + 1);
        aCoeffs.Init(0.0);
        double[][] aRows = [P(a), DP(a), P(b), DP(b)];
        for (int i = 0; i < 4; i++)
        {
            for (int d = 0; d < 2; d++)
            {
                aCoeffs[theLowerRow + i, theLowerCol + d] = aRows[i][d];
            }
        }
        return aCoeffs;
    }

    private static FEmTool_Curve MakeCurve()
    {
        PLib_HermitJacobi aBase = new(WorkDegree, GeomAbs_C1);
        FEmTool_Curve aCurve = new(2, 2, aBase, 1e-7);
        NCollection_Array1<double> aKnots = aCurve.Knots();
        aKnots[1] = 0;
        aKnots[2] = 1;
        aKnots[3] = 3;
        aCurve.SetElement(1, Element(0, 1));
        aCurve.SetElement(2, Element(1, 3));
        return aCurve;
    }

    [Fact]
    public void Curve_evaluates_the_polynomial()
    {
        FEmTool_Curve aCurve = MakeCurve();
        Assert.Equal(2, aCurve.NbElements());
        Assert.Equal(2, aCurve.Dimension());
        Assert.Equal(WorkDegree, aCurve.Degree(1));
        Assert.Equal(WorkDegree, aCurve.Base().WorkDegree());

        NCollection_Array1<double> aV = new(1, 2);
        // Out of order on purpose: the span cache must follow
        foreach (double u in new[] { 0.0, 2.5, 0.3, 1.0, 3.0, 1.7, 0.99, 2.0 })
        {
            aCurve.D0(u, aV);
            Assert.Equal(P(u)[0], aV[1], 1e-12);
            Assert.Equal(P(u)[1], aV[2], 1e-12);
            aCurve.D1(u, aV);
            Assert.Equal(DP(u)[0], aV[1], 1e-11);
            Assert.Equal(DP(u)[1], aV[2], 1e-11);
            aCurve.D2(u, aV);
            Assert.Equal(D2P(u)[0], aV[1], 1e-10);
            Assert.Equal(D2P(u)[1], aV[2], 1e-10);
        }
    }

    [Fact]
    public void Curve_elements_degrees_and_polynomials()
    {
        FEmTool_Curve aCurve = MakeCurve();

        NCollection_Array2<double> anElement = new(1, WorkDegree + 1, 1, 2);
        aCurve.GetElement(2, anElement);
        NCollection_Array2<double> anExpected = Element(1, 3);
        for (int i = 1; i <= WorkDegree + 1; i++)
        {
            for (int j = 1; j <= 2; j++)
            {
                Assert.Equal(anExpected[i, j], anElement[i, j], 1e-13);
            }
        }

        // The canonical coefficients of element 1 on [-1, 1]: U = (t + 1) / 2
        NCollection_Array1<double> aPoly = new(1, 2 * 2 * (WorkDegree + 1));
        aCurve.GetPolynom(aPoly);
        // x = U^3 - U = (t^3 + 3t^2 + 3t + 1) / 8 - (t + 1) / 2
        Assert.Equal(1.0 / 8 - 0.5, aPoly[1], 1e-14);
        Assert.Equal(3.0 / 8 - 0.5, aPoly[3], 1e-14);
        Assert.Equal(3.0 / 8, aPoly[5], 1e-14);
        Assert.Equal(1.0 / 8, aPoly[7], 1e-14);
        for (int k = 4; k <= WorkDegree; k++)
        {
            Assert.Equal(0.0, aPoly[2 * k + 1], 1e-13);
        }

        // The Jacobi part is zero: the degree reduces to the Hermite degree
        aCurve.ReduceDegree(1, 1e-10, out int aNewDegree, out double aMaxError);
        Assert.Equal(3, aNewDegree);
        Assert.True(aMaxError < 1e-10);
        Assert.Equal(3, aCurve.Degree(1));
        NCollection_Array1<double> aV = new(1, 2);
        aCurve.D0(0.5, aV);
        Assert.Equal(P(0.5)[0], aV[1], 1e-13);

        aCurve.SetDegree(2, 5);
        Assert.Equal(5, aCurve.Degree(2));
        Assert.ThrowsAny<Standard_OutOfRange>(() => aCurve.SetDegree(2, WorkDegree + 1));
        Assert.ThrowsAny<Standard_OutOfRange>(() => aCurve.SetElement(3, anElement));
        Assert.ThrowsAny<Standard_OutOfRange>(() => aCurve.GetElement(0, anElement));
    }

    [Fact]
    public void Curve_GetElement_with_shifted_columns_reproduces_OCCT()
    {
        // OCCT rescales the derivative coefficients of columns j1 + 1 to Dimension only, where j1 = LowerCol - 1:
        // with LowerCol = 2, the second column keeps the coefficients relative to the [-1, 1] parametrisation
        FEmTool_Curve aCurve = MakeCurve();
        NCollection_Array2<double> anElement = new(1, WorkDegree + 1, 2, 3);
        aCurve.GetElement(2, anElement);
        NCollection_Array2<double> anExpected = Element(1, 3);
        Assert.Equal(anExpected[2, 1], anElement[2, 2], 1e-13);
        // The interval [1, 3] has half-length 1: the derivative scale factor happens to be 1 there; use [0, 1]
        aCurve.GetElement(1, anElement);
        anExpected = Element(0, 1);
        Assert.Equal(anExpected[2, 1], anElement[2, 2], 1e-13);
        Assert.Equal(anExpected[2, 2] * 0.5, anElement[2, 3], 1e-13);
    }

    [Fact]
    public void Curve_length()
    {
        // A straight line P(U) = (U, 2U), as cubic elements
        PLib_HermitJacobi aBase = new(WorkDegree, GeomAbs_C1);
        FEmTool_Curve aLine = new(2, 3, aBase, 1e-7);
        NCollection_Array1<double> aKnots = aLine.Knots();
        aKnots[1] = 0;
        aKnots[2] = 1;
        aKnots[3] = 2;
        aKnots[4] = 4;
        for (int el = 1; el <= 3; el++)
        {
            NCollection_Array2<double> aCoeffs = new(1, WorkDegree + 1, 1, 2);
            aCoeffs.Init(0.0);
            double a = aKnots[el], b = aKnots[el + 1];
            aCoeffs[1, 1] = a;
            aCoeffs[1, 2] = 2 * a;
            aCoeffs[2, 1] = 1;
            aCoeffs[2, 2] = 2;
            aCoeffs[3, 1] = b;
            aCoeffs[3, 2] = 2 * b;
            aCoeffs[4, 1] = 1;
            aCoeffs[4, 2] = 2;
            aLine.SetElement(el, aCoeffs);
        }
        double aSqrt5 = Math.Sqrt(5);
        aLine.Length(0.5, 0.75, out double aLength);
        Assert.Equal(0.25 * aSqrt5, aLength, 1e-12);
        aLine.Length(0.5, 3.5, out aLength);
        Assert.Equal(3 * aSqrt5, aLength, 1e-12);
        aLine.Length(0, 4, out aLength);
        Assert.Equal(4 * aSqrt5, aLength, 1e-12);
        Assert.ThrowsAny<Standard_OutOfRange>(() => aLine.Length(2, 1, out _));
    }

    private static NCollection_HArray2<double> CriterionCoeffs(double a, double b)
    {
        return new NCollection_HArray2<double>(Element(a, b, 0, 1));
    }

    [Fact]
    public void Linear_criteria_on_a_cubic()
    {
        // On [0, 2]: tension = int |P'|^2, flexion = int |P''|^2, jerk = int |P'''|^2
        // x' = 3U^2 - 1, y' = 4U: int (9U^4 - 6U^2 + 1 + 16U^2) = 9 * 32 / 5 + 10 * 8 / 3 + 2
        double aTension = 9.0 * 32 / 5 + 10.0 * 8 / 3 + 2;
        // x'' = 6U, y'' = 4: int (36U^2 + 16) = 96 + 32
        double aFlexion = 96.0 + 32;
        // x''' = 6: 36 * 2
        double aJerk = 72.0;

        FEmTool_ElementaryCriterion[] aCriteria =
        [
            new FEmTool_LinearTension(WorkDegree, GeomAbs_C1),
            new FEmTool_LinearFlexion(WorkDegree, GeomAbs_C1),
            new FEmTool_LinearJerk(WorkDegree, GeomAbs_C1),
        ];
        double[] anExpected = [aTension, aFlexion, aJerk];
        for (int c = 0; c < 3; c++)
        {
            FEmTool_ElementaryCriterion aCriterion = aCriteria[c];
            NCollection_HArray2<double> aCoeffs = CriterionCoeffs(0, 2);
            aCriterion.Set(aCoeffs);
            aCriterion.Set(0.0, 2.0);
            Assert.Equal(anExpected[c], aCriterion.Value(), 1e-9 * anExpected[c]);

            NCollection_HArray2<int> aDep = aCriterion.DependenceTable();
            Assert.Equal(1, aDep[1, 1]);
            Assert.Equal(0, aDep[1, 2]);
            Assert.Equal(1, aDep[2, 2]);
            Assert.ThrowsAny<Standard_DomainError>(() => aCriterion.Hessian(1, 2, new math_Matrix(0, WorkDegree, 0, WorkDegree)));

            // Value = 1/2 sum over the dimensions of tX H X, and G = H X
            double aSum = 0;
            for (int dim = 1; dim <= 2; dim++)
            {
                math_Matrix aH = new(0, WorkDegree, 0, WorkDegree);
                aCriterion.Hessian(dim, dim, aH);
                math_Vector aG = new(0, WorkDegree);
                aCriterion.Gradient(dim, aG);
                for (int i = 0; i <= WorkDegree; i++)
                {
                    double aHX = 0;
                    for (int j = 0; j <= WorkDegree; j++)
                    {
                        Assert.Equal(aH[i, j], aH[j, i]);
                        aHX += aH[i, j] * aCoeffs[j, dim];
                    }
                    Assert.Equal(aHX, aG[i], 1e-9 * (1 + Math.Abs(aHX)));
                    aSum += 0.5 * aCoeffs[i, dim] * aG[i];
                }
            }
            Assert.Equal(anExpected[c], aSum, 1e-9 * anExpected[c]);
        }
    }

    [Fact]
    public void Linear_criteria_on_a_line()
    {
        // P(U) = (U, 2U) on [1, 3]: tension 2 * 5, no flexion, no jerk
        NCollection_HArray2<double> aCoeffs = new(0, WorkDegree, 1, 2, 0.0);
        aCoeffs[0, 1] = 1;
        aCoeffs[0, 2] = 2;
        aCoeffs[1, 1] = 1;
        aCoeffs[1, 2] = 2;
        aCoeffs[2, 1] = 3;
        aCoeffs[2, 2] = 6;
        aCoeffs[3, 1] = 1;
        aCoeffs[3, 2] = 2;

        FEmTool_LinearTension aTension = new(WorkDegree, GeomAbs_C1);
        FEmTool_LinearFlexion aFlexion = new(WorkDegree, GeomAbs_C1);
        FEmTool_LinearJerk aJerk = new(WorkDegree, GeomAbs_C1);
        foreach (FEmTool_ElementaryCriterion aCriterion in new FEmTool_ElementaryCriterion[] { aTension, aFlexion, aJerk })
        {
            aCriterion.Set(aCoeffs);
            aCriterion.Set(1.0, 3.0);
        }
        Assert.Equal(10.0, aTension.Value(), 1e-11);
        Assert.Equal(0.0, aFlexion.Value(), 1e-11);
        Assert.Equal(0.0, aJerk.Value(), 1e-11);
    }

    [Fact]
    public void Linear_criteria_cache_each_constraint_order()
    {
        // OCCT keeps the reference integrals of the last order only; the port caches each order. The results are
        // the same whatever the order of construction.
        double Tension(GeomAbs_Shape theOrder)
        {
            // The line P(U) = U on [0, 1], in the Hermite part of the order's base
            int aNiv = PLib.NivConstr(theOrder);
            NCollection_HArray2<double> aCoeffs = new(0, 10, 1, 1, 0.0);
            aCoeffs[0, 1] = 0;
            aCoeffs[aNiv + 1, 1] = 1;
            if (aNiv >= 1)
            {
                aCoeffs[1, 1] = 1;
                aCoeffs[aNiv + 2, 1] = 1;
            }
            FEmTool_LinearTension aCriterion = new(10, theOrder);
            aCriterion.Set(aCoeffs);
            aCriterion.Set(0.0, 1.0);
            return aCriterion.Value();
        }
        Assert.Equal(1.0, Tension(GeomAbs_C0), 1e-12);
        Assert.Equal(1.0, Tension(GeomAbs_C2), 1e-12);
        Assert.Equal(1.0, Tension(GeomAbs_C1), 1e-12);
        Assert.Equal(1.0, Tension(GeomAbs_C0), 1e-12);

        Assert.ThrowsAny<Standard_ConstructionError>(() => new FEmTool_LinearJerk(15, GeomAbs_C1));
        FEmTool_LinearTension anUnset = new(5, GeomAbs_C0);
        Assert.ThrowsAny<Standard_DomainError>(() => anUnset.DependenceTable());
    }

    [Fact]
    public void ElementsOfRefMatrix_are_products_of_derivatives()
    {
        PLib_HermitJacobi aBase = new(6, GeomAbs_C1);
        for (int aDer = 0; aDer <= 3; aDer++)
        {
            FEmTool_ElementsOfRefMatrix anElem = new(aBase, aDer);
            Assert.Equal(1, anElem.NbVariables());
            Assert.Equal(28, anElem.NbEquations());
            math_Vector aX = new(1, 1, 0.3);
            math_Vector aF = new(1, 28);
            Assert.True(anElem.Value(aX, aF));

            NCollection_Array1<double> aV = new(0, 6), aD1 = new(0, 6), aD2 = new(0, 6), aD3 = new(0, 6);
            aBase.D3(0.3, aV, aD1, aD2, aD3);
            NCollection_Array1<double> aB = aDer switch { 0 => aV, 1 => aD1, 2 => aD2, _ => aD3 };
            int k = 1;
            for (int i = 0; i <= 6; i++)
            {
                for (int j = i; j <= 6; j++)
                {
                    Assert.Equal(aB[i] * aB[j], aF[k++], 1e-12);
                }
            }
            Assert.ThrowsAny<Standard_OutOfRange>(() => anElem.Value(aX, new math_Vector(1, 27)));
        }
        Assert.ThrowsAny<Standard_ConstructionError>(() => new FEmTool_ElementsOfRefMatrix(aBase, 4));
    }
}
