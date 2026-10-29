// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_Curve.hxx
// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A curve made of polynomial elements in a Hermite-Jacobi base, one per knot interval, each parametrised on
/// [-1, 1].
/// </summary>
/// <remarks>
/// A working object of AppDef_Variational (decision 21): <see cref="Knots"/> returns the curve's own knot array,
/// which callers fill or change in place as in OCCT (shared: changing it changes the curve). The evaluators cache
/// the canonical coefficients of each element, so they are not thread-safe.
/// </remarks>
public class FEmTool_Curve : Standard_Transient
{
    private readonly int myNbElements;
    private readonly int myDimension;
    private readonly PLib_HermitJacobi myBase;
    private readonly NCollection_HArray1<double> myKnots;
    private readonly NCollection_Array1<int> myDegree;
    private readonly NCollection_Array1<double> myCoeff;
    private readonly NCollection_Array1<double> myPoly;
    private readonly NCollection_Array1<double> myDeri;
    private readonly NCollection_Array1<double> myDsecn;
    private readonly NCollection_Array1<int> HasPoly;
    private readonly NCollection_Array1<int> HasDeri;
    private readonly NCollection_Array1<int> HasSecn;
    private readonly NCollection_Array1<double> myLength;
    private double Uf;
    private double Ul;
    private double Denom;
    private double USum;
    private int myIndex;
    private int myPtr;

    /// <summary>
    /// A curve of dimension <paramref name="Dimension"/> with <paramref name="NbElements"/> elements in the base
    /// <paramref name="TheBase"/>. The tolerance is not used (as in OCCT). The knots must be set through
    /// <see cref="Knots"/>.
    /// </summary>
    public FEmTool_Curve(int Dimension, int NbElements, PLib_HermitJacobi TheBase, double Tolerance)
    {
        myNbElements = NbElements;
        myDimension = Dimension;
        myBase = TheBase;
        myDegree = new NCollection_Array1<int>(1, myNbElements);
        myCoeff = new NCollection_Array1<double>(1, myDimension * myNbElements * (myBase.WorkDegree() + 1));
        myPoly = new NCollection_Array1<double>(1, myDimension * myNbElements * (myBase.WorkDegree() + 1));
        myDeri = new NCollection_Array1<double>(1, myDimension * myNbElements * (myBase.WorkDegree()));
        myDsecn = new NCollection_Array1<double>(1, myDimension * myNbElements * (myBase.WorkDegree() - 1));
        HasPoly = new NCollection_Array1<int>(1, myNbElements);
        HasDeri = new NCollection_Array1<int>(1, myNbElements);
        HasSecn = new NCollection_Array1<int>(1, myNbElements);
        myLength = new NCollection_Array1<double>(1, myNbElements);
        myIndex = 0;

        myKnots = new NCollection_HArray1<double>(1, myNbElements + 1);
        myDegree.Init(myBase.WorkDegree());
        HasPoly.Init(0);
        HasDeri.Init(0);
        HasSecn.Init(0);
        myLength.Init(-1);
    }

    /// <summary>The knots (1 .. NbElements + 1): the curve's own array (shared: changing it changes the curve).</summary>
    public NCollection_Array1<double> Knots() => myKnots.ChangeArray1();

    /// <summary>
    /// Sets the coefficients of element <paramref name="IndexOfElement"/> in the Hermite-Jacobi base:
    /// <paramref name="Coeffs"/>(degree, dimension), with the Hermite part relative to the knot interval.
    /// </summary>
    public void SetElement(int IndexOfElement, NCollection_Array2<double> Coeffs)
    {
        int i, j, degBase, deg;
        if (IndexOfElement > myNbElements || IndexOfElement < 1)
        {
            throw new Standard_OutOfRange();
        }
        degBase = myBase.WorkDegree();
        deg = myDegree[IndexOfElement];
        int iBase = (IndexOfElement - 1) * (degBase + 1) * myDimension, i1 = iBase - myDimension,
            i2 = Coeffs.LowerRow() - 1, j1 = Coeffs.LowerCol() - 1;
        for (i = 1; i <= deg + 1; i++)
        {
            i1 += myDimension;
            i2++;
            for (j = 1; j <= myDimension; j++)
            {
                myCoeff[i1 + j] = Coeffs[i2, j1 + j];
            }
        }

        double stenor = (myKnots.Value(IndexOfElement + 1) - myKnots.Value(IndexOfElement)) / 2.0, mfact;
        i1 = iBase;
        i2 = iBase + (myBase.NivConstr() + 1) * myDimension;
        for (i = 1; i <= myBase.NivConstr(); i++)
        {
            i1 += myDimension;
            i2 += myDimension;
            mfact = Math.Pow(stenor, i);
            for (j = 1; j <= myDimension; j++)
            {
                myCoeff[i1 + j] *= mfact;
                myCoeff[i2 + j] *= mfact;
            }
        }

        HasPoly[IndexOfElement] = HasDeri[IndexOfElement] = HasSecn[IndexOfElement] = 0;
        myLength[IndexOfElement] = -1;
    }

    /// <summary>The coefficients of element <paramref name="IndexOfElement"/>, as given to <see cref="SetElement"/>.</summary>
    public void GetElement(int IndexOfElement, NCollection_Array2<double> Coeffs)
    {
        int i, j, degBase, deg;
        if (IndexOfElement > myNbElements || IndexOfElement < 1)
        {
            throw new Standard_OutOfRange();
        }
        degBase = myBase.WorkDegree();
        deg = myDegree[IndexOfElement];
        int iBase = (IndexOfElement - 1) * (degBase + 1) * myDimension, i1 = iBase - myDimension,
            i2 = Coeffs.LowerRow() - 1, j1 = Coeffs.LowerCol() - 1;
        for (i = 1; i <= deg + 1; i++)
        {
            i1 += myDimension;
            i2++;
            for (j = 1; j <= myDimension; j++)
            {
                Coeffs[i2, j1 + j] = myCoeff.Value(i1 + j);
            }
        }

        double stenor = 2.0 / (myKnots.Value(IndexOfElement + 1) - myKnots.Value(IndexOfElement)), mfact;
        i2 = Coeffs.LowerRow();
        int i3 = i2 + myBase.NivConstr() + 1;

        for (i = 1; i <= myBase.NivConstr(); i++)
        {
            mfact = Math.Pow(stenor, i);
            // OCCT: the bound should be j1 + myDimension; with Coeffs.LowerCol() > 1 the last columns are not
            // rescaled (occt-issues.md)
            for (j = j1 + 1; j <= myDimension; j++)
            {
                Coeffs[i2 + i, j] *= mfact;
                Coeffs[i3 + i, j] *= mfact;
            }
        }
    }

    /// <summary>The coefficients of all elements in the canonical base, each on [-1, 1].</summary>
    public void GetPolynom(NCollection_Array1<double> Coeffs)
    {
        int IndexOfElement, i, di = Coeffs.Lower() - myPoly.Lower();

        for (IndexOfElement = 1; IndexOfElement <= myNbElements; IndexOfElement++)
        {
            if (HasPoly.Value(IndexOfElement) == 0)
            {
                Update(IndexOfElement, 0);
            }
        }

        for (i = myPoly.Lower(); i <= myPoly.Upper(); i++)
        {
            Coeffs[di + i] = myPoly[i];
        }
    }

    // The span search shared by D0, D1 and D2
    private void LocateSpan(double U)
    {
        if (myIndex == 0 || (U < Uf) || (U > Ul) || (myKnots.Value(myIndex) != Uf)
            || (myKnots.Value(myIndex + 1) != Ul))
        {
            // Search the span
            if (U <= myKnots.Value(2))
            {
                myIndex = 1;
            }
            else
            {
                for (myIndex = 2; myIndex <= myNbElements; myIndex++)
                {
                    if (U >= myKnots.Value(myIndex) && U <= myKnots.Value(myIndex + 1))
                    {
                        break;
                    }
                }
                if (myIndex > myNbElements)
                {
                    myIndex = myNbElements;
                }
            }

            Uf = myKnots.Value(myIndex);
            Ul = myKnots.Value(myIndex + 1);
            Denom = 1.0 / (Ul - Uf);
            USum = Uf + Ul;
            myPtr = (myIndex - 1) * (myBase.WorkDegree() + 1) * myDimension + 1;
        }
    }

    /// <summary>The point at <paramref name="U"/> into <paramref name="Pnt"/> (Dimension values).</summary>
    public void D0(double U, NCollection_Array1<double> Pnt)
    {
        int deg;
        double S;

        LocateSpan(U);

        deg = myDegree[myIndex];
        if (HasPoly.Value(myIndex) == 0)
        {
            Update(myIndex, 0);
        }

        // Parameter normalization: S [-1, 1]
        S = (2 * U - USum) * Denom;
        PLib.NoDerivativeEvalPolynomial(S, deg, myDimension, deg * myDimension, myPoly.AsSpan(myPtr),
                                        Pnt.AsSpan(Pnt.Lower()));
    }

    /// <summary>The first derivative at <paramref name="U"/> into <paramref name="Vec"/>.</summary>
    public void D1(double U, NCollection_Array1<double> Vec)
    {
        int deg, i;
        double S;

        LocateSpan(U);

        deg = myDegree[myIndex];
        if (HasDeri.Value(myIndex) == 0)
        {
            Update(myIndex, 1);
        }

        // Parameter normalization: S [-1, 1]
        S = (2 * U - USum) * Denom;
        PLib.NoDerivativeEvalPolynomial(S, deg - 1, myDimension, (deg - 1) * myDimension,
                                        myDeri.AsSpan(1 + (myIndex - 1) * myBase.WorkDegree() * myDimension),
                                        Vec.AsSpan(Vec.Lower()));

        S = 2 * Denom;
        for (i = Vec.Lower(); i <= Vec.Upper(); i++)
        {
            Vec[i] *= S;
        }
    }

    /// <summary>The second derivative at <paramref name="U"/> into <paramref name="Vec"/>.</summary>
    public void D2(double U, NCollection_Array1<double> Vec)
    {
        int deg, i;
        double S;

        LocateSpan(U);

        deg = myDegree[myIndex];
        if (HasSecn.Value(myIndex) == 0)
        {
            Update(myIndex, 2);
        }

        // Parameter normalization: S [-1, 1]
        S = (2 * U - USum) * Denom;
        if (deg < 2)
        {
            // OCCT: an element of degree 1 (ReduceDegree allows it at C0) has no second-derivative coefficients, and
            // OCCT evaluates a polynomial of degree -1, whose value is the Dimension coefficients just before the
            // element's block. For a later element they are the last coefficients of the previous element's block,
            // which the port reads as OCCT does; for the first element they lie before the array (uninitialised
            // memory in OCCT), and the port uses zeros (occt-issues.md, decision 1606).
            int aTop = 1 + (myIndex - 1) * (myBase.WorkDegree() - 1) * myDimension - myDimension;
            for (i = 0; i < myDimension; i++)
            {
                Vec[Vec.Lower() + i] = aTop >= 1 ? myDsecn[aTop + i] : 0.0;
            }
        }
        else
        {
            PLib.NoDerivativeEvalPolynomial(S, deg - 2, myDimension, (deg - 2) * myDimension,
                                            myDsecn.AsSpan(1 + (myIndex - 1) * (myBase.WorkDegree() - 1) * myDimension),
                                            Vec.AsSpan(Vec.Lower()));
        }

        S = 4 * Denom * Denom;
        for (i = Vec.Lower(); i <= Vec.Upper(); i++)
        {
            Vec[i] *= S;
        }
    }

    /// <summary>The length of the curve between <paramref name="FirstU"/> and <paramref name="LastU"/>.</summary>
    public void Length(double FirstU, double LastU, out double Length)
    {
        int Low, High, deg, degBase, i, Ptr;
        if (FirstU > LastU)
        {
            throw new Standard_OutOfRange("FEmTool_Curve::Length");
        }

        if (myKnots.Value(1) > FirstU)
        {
            Low = 1;
        }
        else
        {
            for (Low = 1; Low <= myNbElements; Low++)
            {
                if (FirstU >= myKnots.Value(Low) && FirstU <= myKnots.Value(Low + 1))
                {
                    break;
                }
            }
        }
        if (Low > myNbElements)
        {
            Low = myNbElements;
        }

        if (myKnots.Value(1) > LastU)
        {
            High = 1;
        }
        else
        {
            for (High = Low; High <= myNbElements; High++)
            {
                if (LastU >= myKnots.Value(High) && LastU <= myKnots.Value(High + 1))
                {
                    break;
                }
            }
        }
        if (myKnots.Value(myNbElements + 1) < LastU)
        {
            High = myNbElements;
        }

        double Li;
        degBase = myBase.WorkDegree();
        Length = 0;

        double FirstS, LastS;
        FirstS = (2 * FirstU - myKnots.Value(Low) - myKnots.Value(Low + 1))
                 / (myKnots.Value(Low + 1) - myKnots.Value(Low));
        LastS = (2 * LastU - myKnots.Value(High) - myKnots.Value(High + 1))
                / (myKnots.Value(High + 1) - myKnots.Value(High));

        if (Low == High)
        {
            Ptr = (Low - 1) * (degBase + 1) * myDimension + 1;
            deg = myDegree[Low];

            if (HasPoly[Low] == 0)
            {
                Update(Low, 0);
            }
            PLib.EvalLength(deg, myDimension, myPoly.AsSpan(Ptr), FirstS, LastS, out Length);
            return;
        }

        deg = myDegree[Low];
        Ptr = (Low - 1) * (degBase + 1) * myDimension + 1;

        if (HasPoly[Low] == 0)
        {
            Update(Low, 0);
        }
        if (FirstS < -1.0)
        {
            PLib.EvalLength(deg, myDimension, myPoly.AsSpan(Ptr), FirstS, -1.0, out Li);
            Length += Li;
            if (myLength[Low] < 0.0)
            {
                PLib.EvalLength(deg, myDimension, myPoly.AsSpan(Ptr), -1.0, 1.0, out Li);
                myLength[Low] = Li;
            }
            Length += myLength[Low];
        }
        else
        {
            PLib.EvalLength(deg, myDimension, myPoly.AsSpan(Ptr), FirstS, 1.0, out Li);
            Length += Li;
        }

        deg = myDegree[High];
        Ptr = (High - 1) * (degBase + 1) * myDimension + 1;

        if (HasPoly[High] == 0)
        {
            Update(High, 0);
        }
        if (LastS > 1.0)
        {
            PLib.EvalLength(deg, myDimension, myPoly.AsSpan(Ptr), 1.0, LastS, out Li);
            Length += Li;
            if (myLength[High] < 0.0)
            {
                PLib.EvalLength(deg, myDimension, myPoly.AsSpan(Ptr), -1.0, 1.0, out Li);
                myLength[High] = Li;
            }
            Length += myLength[High];
        }
        else
        {
            PLib.EvalLength(deg, myDimension, myPoly.AsSpan(Ptr), -1.0, LastS, out Li);
            Length += Li;
        }

        for (i = Low + 1; i < High; i++)
        {
            if (myLength.Value(i) < 0)
            {
                Ptr = (i - 1) * (degBase + 1) * myDimension + 1;
                deg = myDegree[i];
                if (HasPoly[i] == 0)
                {
                    Update(i, 0);
                }
                PLib.EvalLength(deg, myDimension, myPoly.AsSpan(Ptr), -1.0, 1.0, out Li);
                myLength[i] = Li;
            }
            Length += myLength[i];
        }
    }

    public int NbElements() => myNbElements;

    public int Dimension() => myDimension;

    public PLib_HermitJacobi Base() => myBase;

    public int Degree(int IndexOfElement) => myDegree.Value(IndexOfElement);

    /// <summary>Sets the degree of an element (at most the base's work degree).</summary>
    public void SetDegree(int IndexOfElement, int Degree)
    {
        if (Degree <= myBase.WorkDegree())
        {
            myDegree[IndexOfElement] = Degree;
            HasPoly[IndexOfElement] = HasDeri[IndexOfElement] = HasSecn[IndexOfElement] = 0;
            myLength[IndexOfElement] = -1;
        }
        else if (Degree > myBase.WorkDegree())
        {
            throw new Standard_OutOfRange("FEmTool_Curve::SetDegree");
        }
    }

    /// <summary>
    /// Lowers the degree of an element while the error stays below <paramref name="Tol"/> (not below
    /// 2 * NivConstr + 1).
    /// </summary>
    public void ReduceDegree(int IndexOfElement, double Tol, out int NewDegree, out double MaxError)
    {
        int deg = myDegree[IndexOfElement];

        int Ptr = (IndexOfElement - 1) * (myBase.WorkDegree() + 1) * myDimension + 1;

        myBase.ReduceDegree(myDimension, deg, Tol, myCoeff.AsSpan(Ptr), out NewDegree, out MaxError);
        NewDegree = Max(NewDegree, 2 * myBase.NivConstr() + 1);

        if (NewDegree < deg)
        {
            myDegree[IndexOfElement] = NewDegree;
            HasPoly[IndexOfElement] = HasDeri[IndexOfElement] = HasSecn[IndexOfElement] = 0;
            myLength[IndexOfElement] = -1;
        }
    }

    private void Update(int Index, int Order)
    {
        int degBase = myBase.WorkDegree(), deg = myDegree[Index];

        if (HasPoly[Index] == 0)
        {
            int Ptr = (Index - 1) * (degBase + 1) * myDimension + 1;

            NCollection_Array1<double> Coeff = new(myPoly, Ptr, 0, myDimension * (deg + 1) - 1);
            NCollection_Array1<double> BaseCoeff = new(myCoeff, Ptr, 0, myDimension * (deg + 1) - 1);

            myBase.ToCoefficients(myDimension, deg, BaseCoeff, Coeff);
            HasPoly[Index] = 1;
        }

        if (Order >= 1)
        {
            int i1 = (Index - 1) * (degBase) * myDimension - myDimension,
                i2 = (Index - 1) * (degBase + 1) * myDimension;
            int i, j;
            if (HasDeri.Value(Index) == 0)
            {
                for (i = 1; i <= deg; i++)
                {
                    i1 += myDimension;
                    i2 += myDimension;
                    for (j = 1; j <= myDimension; j++)
                    {
                        myDeri[i1 + j] = i * myPoly.Value(i2 + j);
                    }
                }
                HasDeri[Index] = 1;
            }
            if ((Order >= 2) && HasSecn.Value(Index) == 0)
            {
                i1 = (Index - 1) * (degBase - 1) * myDimension - myDimension;
                i2 = (Index - 1) * (degBase) * myDimension;
                for (i = 1; i < deg; i++)
                {
                    i1 += myDimension;
                    i2 += myDimension;
                    for (j = 1; j <= myDimension; j++)
                    {
                        myDsecn[i1 + j] = i * myDeri.Value(i2 + j);
                    }
                }
                HasSecn[Index] = 1;
            }
        }
    }
}
