// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_LinearCriteria.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_LinearCriteria.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Engine.Numerics.Polynomials;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// The linear criteria used by <see cref="AppDef_Variational"/> to smooth points: the least-squares distance to the
/// points plus the tension, flexion and jerk of the curve (FEmTool_LinearTension, LinearFlexion, LinearJerk).
/// </summary>
/// <remarks>A working object (decisions 21 and 207); the MultiLine, parameters and curve it is given are shared.</remarks>
public class AppDef_LinearCriteria : AppDef_SmoothCriterion
{
    private readonly AppDef_MultiLine mySSP;
    private NCollection_HArray1<double>? myParameters;
    private NCollection_HArray1<double>? myCache;
    private readonly FEmTool_ElementaryCriterion[] myCriteria = new FEmTool_ElementaryCriterion[3];
    private readonly double[] myEstimation = new double[3];
    private double myQuadraticWeight;
    private double myQualityWeight;
    private readonly double[] myPercent = new double[3];
    private readonly NCollection_Array1<double> myPntWeight;
    private FEmTool_Curve? myCurve;
    private double myLength;
    private int myE;
    private int IF;
    private int IL;

    private static int order(HermitJacobi B)
    {
        return B.NivConstr();
    }

    /// <summary>The criteria for the points <paramref name="FirstPoint"/> to <paramref name="LastPoint"/> of <paramref name="SSP"/>.</summary>
    public AppDef_LinearCriteria(AppDef_MultiLine SSP, int FirstPoint, int LastPoint)
    {
        mySSP = SSP;
        myQuadraticWeight = 0.0;
        myQualityWeight = 0.0;
        myPntWeight = new NCollection_Array1<double>(FirstPoint, LastPoint);
        myLength = 0.0;
        myE = 0;
        IF = 0;
        IL = 0;
        myPntWeight.Init(1.0);
    }

    public override void SetParameters(NCollection_HArray1<double> Parameters)
    {
        myParameters = Parameters;
        myE = 0; // Cache become invalid.
    }

    public override void SetCurve(FEmTool_Curve C)
    {
        if (myCurve is null)
        {
            myCurve = C;

            int MxDeg = myCurve.Base().WorkDegree(), NbDim = myCurve.Dimension(),
                Order = order(myCurve.Base());

            Continuity ConstraintOrder = C0;
            switch (Order)
            {
                case 0:
                    ConstraintOrder = C0;
                    break;
                case 1:
                    ConstraintOrder = C1;
                    break;
                case 2:
                    ConstraintOrder = C2;
                    break;
            }

            myCriteria[0] = new FEmTool_LinearTension(MxDeg, ConstraintOrder);
            myCriteria[1] = new FEmTool_LinearFlexion(MxDeg, ConstraintOrder);
            myCriteria[2] = new FEmTool_LinearJerk(MxDeg, ConstraintOrder);

            NCollection_HArray2<double> Coeff = new(0, 0, 1, NbDim);

            myCriteria[0].Set(Coeff);
            myCriteria[1].Set(Coeff);
            myCriteria[2].Set(Coeff);
        }
        else if (myCurve != C)
        {
            int OldMxDeg = myCurve.Base().WorkDegree(), OldNbDim = myCurve.Dimension(),
                OldOrder = order(myCurve.Base());

            myCurve = C;

            int MxDeg = myCurve.Base().WorkDegree(), NbDim = myCurve.Dimension(),
                Order = order(myCurve.Base());

            if (MxDeg != OldMxDeg || Order != OldOrder)
            {
                Continuity ConstraintOrder = C0;
                switch (Order)
                {
                    case 0:
                        ConstraintOrder = C0;
                        break;
                    case 1:
                        ConstraintOrder = C1;
                        break;
                    case 2:
                        ConstraintOrder = C2;
                        break;
                }

                myCriteria[0] = new FEmTool_LinearTension(MxDeg, ConstraintOrder);
                myCriteria[1] = new FEmTool_LinearFlexion(MxDeg, ConstraintOrder);
                myCriteria[2] = new FEmTool_LinearJerk(MxDeg, ConstraintOrder);

                NCollection_HArray2<double> Coeff = new(0, 0, 1, NbDim);

                myCriteria[0].Set(Coeff);
                myCriteria[1].Set(Coeff);
                myCriteria[2].Set(Coeff);
            }
            else if (NbDim != OldNbDim)
            {
                NCollection_HArray2<double> Coeff = new(0, 0, 1, NbDim);

                myCriteria[0].Set(Coeff);
                myCriteria[1].Set(Coeff);
                myCriteria[2].Set(Coeff);
            }
        }
    }

    public override void GetCurve(out FEmTool_Curve? C)
    {
        C = myCurve;
    }

    public override void SetEstimation(double E1, double E2, double E3)
    {
        myEstimation[0] = E1;
        myEstimation[1] = E2;
        myEstimation[2] = E3;
    }

    public override ref double EstLength()
    {
        return ref myLength;
    }

    public override void GetEstimation(out double E1, out double E2, out double E3)
    {
        E1 = myEstimation[0];
        E2 = myEstimation[1];
        E3 = myEstimation[2];
    }

    public override NCollection_HArray2<NCollection_HArray1<int>> AssemblyTable()
    {
        if (myCurve is null)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::AssemblyTable");
        }

        int NbDim = myCurve.Dimension(), NbElm = myCurve.NbElements(), nc1 = order(myCurve.Base()) + 1;
        int MxDeg = myCurve.Base().WorkDegree();

        NCollection_HArray2<NCollection_HArray1<int>> AssTable = new(1, NbDim, 1, NbElm);

        NCollection_HArray1<int> GlobIndex, Aux;

        int i, el = 1, dim = 1, NbGlobVar = 0, gi0;

        // For dim = 1
        // For first element (el = 1)
        GlobIndex = new NCollection_HArray1<int>(0, MxDeg);

        for (i = 0; i < nc1; i++)
        {
            NbGlobVar++;
            GlobIndex.SetValue(i, NbGlobVar);
        }
        gi0 = MxDeg - 2 * nc1 + 1;
        for (i = nc1; i < 2 * nc1; i++)
        {
            NbGlobVar++;
            GlobIndex.SetValue(i, NbGlobVar + gi0);
        }
        for (i = 2 * nc1; i <= MxDeg; i++)
        {
            NbGlobVar++;
            GlobIndex.SetValue(i, NbGlobVar - nc1);
        }
        gi0 = NbGlobVar - nc1 + 1;
        AssTable.SetValue(dim, el, GlobIndex);

        // For rest elements
        for (el = 2; el <= NbElm; el++)
        {
            GlobIndex = new NCollection_HArray1<int>(0, MxDeg);
            for (i = 0; i < nc1; i++)
            {
                GlobIndex.SetValue(i, gi0 + i);
            }

            gi0 = MxDeg - 2 * nc1 + 1;
            for (i = nc1; i < 2 * nc1; i++)
            {
                NbGlobVar++;
                GlobIndex.SetValue(i, NbGlobVar + gi0);
            }
            for (i = 2 * nc1; i <= MxDeg; i++)
            {
                NbGlobVar++;
                GlobIndex.SetValue(i, NbGlobVar - nc1);
            }
            gi0 = NbGlobVar - nc1 + 1;
            AssTable.SetValue(dim, el, GlobIndex);
        }

        // For other dimensions
        gi0 = NbGlobVar;
        for (dim = 2; dim <= NbDim; dim++)
        {
            for (el = 1; el <= NbElm; el++)
            {
                Aux = AssTable.Value(1, el);
                GlobIndex = new NCollection_HArray1<int>(0, MxDeg);
                for (i = 0; i <= MxDeg; i++)
                {
                    GlobIndex.SetValue(i, Aux.Value(i) + NbGlobVar);
                }
                AssTable.SetValue(dim, el, GlobIndex);
            }
            NbGlobVar += gi0;
        }

        return AssTable;
    }

    public override NCollection_HArray2<int> DependenceTable()
    {
        if (myCurve is null)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::DependenceTable");
        }

        int Dim = myCurve.Dimension();

        NCollection_HArray2<int> DepTab = new(1, Dim, 1, Dim, 0);
        int i;
        for (i = 1; i <= Dim; i++)
        {
            DepTab.SetValue(i, i, 1);
        }

        return DepTab;
    }

    public override int QualityValues(double J1min, double J2min, double J3min, out double J1, out double J2,
                                      out double J3)
    {
        if (myCurve is null)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::QualityValues");
        }

        int NbDim = myCurve.Dimension(), NbElm = myCurve.NbElements();

        NCollection_Array1<double> Knots = myCurve.Knots();
        NCollection_HArray2<double>? Coeff = null;

        int el, deg = 0, curdeg, i;
        double UFirst, ULast;

        J1 = J2 = J3 = 0.0;
        for (el = 1; el <= NbElm; el++)
        {
            curdeg = myCurve.Degree(el);
            if (deg != curdeg)
            {
                deg = curdeg;
                Coeff = new NCollection_HArray2<double>(0, deg, 1, NbDim);
            }

            myCurve.GetElement(el, Coeff!.ChangeArray2());

            UFirst = Knots[el];
            ULast = Knots[el + 1];

            myCriteria[0].Set(Coeff);
            myCriteria[0].Set(UFirst, ULast);
            J1 = J1 + myCriteria[0].Value();

            myCriteria[1].Set(Coeff);
            myCriteria[1].Set(UFirst, ULast);
            J2 = J2 + myCriteria[1].Value();

            myCriteria[2].Set(Coeff);
            myCriteria[2].Set(UFirst, ULast);
            J3 = J3 + myCriteria[2].Value();
        }

        // Calculation of ICDANA - see MOTEST.f
        //  double JEsMin[3] = {.01, .001, .001}; // from MOTLIS.f
        Span<double> JEsMin = stackalloc double[3];
        JEsMin[0] = J1min;
        JEsMin[1] = J2min;
        JEsMin[2] = J3min;
        Span<double> ValCri = stackalloc double[3];
        ValCri[0] = J1;
        ValCri[1] = J2;
        ValCri[2] = J3;

        int ICDANA = 0;

        //   (2) Test the improvement of estimates
        //       (overestimated criterion => No minimization)

        for (i = 0; i <= 2; i++)
        {
            if ((ValCri[i] < 0.8 * myEstimation[i]) && (myEstimation[i] > JEsMin[i]))
            {
                if (ICDANA < 1)
                {
                    ICDANA = 1;
                }
                if (ValCri[i] < 0.1 * myEstimation[i])
                {
                    ICDANA = 2;
                }
                myEstimation[i] = Max(1.05 * ValCri[i], JEsMin[i]);
            }
        }

        //  (3) Update the Estimates
        //     (underestimated criterion => poor conditioning)
        if (ValCri[0] > myEstimation[0] * 2)
        {
            myEstimation[0] += ValCri[0] * .1;
            if (ICDANA == 0)
            {
                if (ValCri[0] > myEstimation[0] * 10)
                {
                    ICDANA = 2;
                }
                else
                {
                    ICDANA = 1;
                }
            }
            else
            {
                ICDANA = 2;
            }
        }
        if (ValCri[1] > myEstimation[1] * 20)
        {
            myEstimation[1] += ValCri[1] * .1;
            if (ICDANA == 0)
            {
                if (ValCri[1] > myEstimation[1] * 100)
                {
                    ICDANA = 2;
                }
                else
                {
                    ICDANA = 1;
                }
            }
            else
            {
                ICDANA = 2;
            }
        }
        if (ValCri[2] > myEstimation[2] * 20)
        {
            myEstimation[2] += ValCri[2] * .05;
            if (ICDANA == 0)
            {
                if (ValCri[2] > myEstimation[2] * 100)
                {
                    ICDANA = 2;
                }
                else
                {
                    ICDANA = 1;
                }
            }
            else
            {
                ICDANA = 2;
            }
        }

        return ICDANA;
    }

    public override void ErrorValues(out double MaxError, out double QuadraticError, out double AverageError)
    {
        if (myCurve is null)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::ErrorValues");
        }

        int NbDim = myCurve.Dimension();

        int myNbP2d = AppDef_MyLineTool.NbP2d(mySSP), myNbP3d = AppDef_MyLineTool.NbP3d(mySSP);

        if (NbDim != (2 * myNbP2d + 3 * myNbP3d))
        {
            throw new DomainErrorException("AppDef_LinearCriteria::ErrorValues");
        }

        NCollection_Array1<Point3D> TabP3d = new(1, Max(1, myNbP3d));
        NCollection_Array1<Point2D> TabP2d = new(1, Max(1, myNbP2d));
        NCollection_Array1<double> BasePoint = new(1, NbDim);
        Point2D P2d = new();
        Point3D P3d = new();

        int i, ipnt, c0 = 0;
        double SqrDist, Dist;

        MaxError = QuadraticError = AverageError = 0.0;

        for (i = myParameters!.Lower(); i <= myParameters.Upper(); i++)
        {
            myCurve.D0(myParameters.Value(i), BasePoint);

            c0 = 0;
            AppDef_MyLineTool.Value(mySSP, i, TabP3d);
            for (ipnt = 1; ipnt <= myNbP3d; ipnt++)
            {
                P3d.SetCoord(BasePoint[c0 + 1], BasePoint[c0 + 2], BasePoint[c0 + 3]);
                SqrDist = P3d.SquareDistance(TabP3d[ipnt]);
                Dist = Math.Sqrt(SqrDist);
                MaxError = Max(MaxError, Dist);
                QuadraticError += SqrDist;
                AverageError += Dist;
                c0 += 3;
            }

            if (myNbP3d == 0)
            {
                AppDef_MyLineTool.Value(mySSP, i, TabP2d);
            }
            else
            {
                AppDef_MyLineTool.Value(mySSP, i, TabP3d, TabP2d);
            }
            for (ipnt = 1; ipnt <= myNbP2d; ipnt++)
            {
                P2d.SetCoord(BasePoint[c0 + 1], BasePoint[c0 + 2]);
                SqrDist = P2d.SquareDistance(TabP2d[ipnt]);
                Dist = Math.Sqrt(SqrDist);
                MaxError = Max(MaxError, Dist);
                QuadraticError += SqrDist;
                AverageError += Dist;
                c0 += 2;
            }
        }
    }

    public override void Hessian(int Element, int Dimension1, int Dimension2, Matrix H)
    {
        if (myCurve is null)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::Hessian");
        }

        if (DependenceTable().Value(Dimension1, Dimension2) == 0)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::Hessian");
        }

        int MxDeg = myCurve.Base().WorkDegree(),
            Order = order(myCurve.Base());

        Matrix AuxH = new(0, H.RowNumber() - 1, 0, H.ColNumber() - 1, 0.0);

        NCollection_Array1<double> Knots = myCurve.Knots();
        double UFirst, ULast;

        UFirst = Knots[Element];
        ULast = Knots[Element + 1];

        int icrit;

        // Quality criterion part of Hessian

        H.Init(0);

        for (icrit = 0; icrit <= 2; icrit++)
        {
            myCriteria[icrit].Set(UFirst, ULast);
            myCriteria[icrit].Hessian(Dimension1, Dimension2, AuxH);
            H += (myQualityWeight * myPercent[icrit] / myEstimation[icrit]) * AuxH;
        }

        // Least square part of Hessian

        AuxH.Init(0.0);

        double coeff = (ULast - UFirst) / 2.0, curcoeff, poid;
        int ipnt, ii, degH = 2 * Order + 1;

        int k1, k2, i, j, i0 = H.LowerRow(), j0 = H.LowerCol(), i1, j1,
            di = myPntWeight.Lower() - myParameters!.Lower();

        // BuilCache
        if (myE != Element)
        {
            BuildCache(Element);
        }

        // Compute the least square Hessian
        for (ii = 1, ipnt = IF; ipnt <= IL; ipnt++, ii += (MxDeg + 1))
        {
            poid = myPntWeight[di + ipnt] * 2.0;
            ReadOnlySpan<double> BV = myCache!.AsSpan(ii);

            // Hermite*Hermite part of matrix
            for (i = 0; i <= degH; i++)
            {
                k1 = (i <= Order) ? i : i - Order - 1;
                curcoeff = Math.Pow(coeff, k1) * poid * BV[i];

                // Hermite*Hermite part of matrix
                for (j = i; j <= degH; j++)
                {
                    k2 = (j <= Order) ? j : j - Order - 1;
                    AuxH[i, j] += curcoeff * Math.Pow(coeff, k2) * BV[j];
                }
                // Hermite*Jacobi part of matrix
                for (j = degH + 1; j <= MxDeg; j++)
                {
                    AuxH[i, j] += curcoeff * BV[j];
                }
            }

            // Jacoby*Jacobi part of matrix
            for (i = degH + 1; i <= MxDeg; i++)
            {
                curcoeff = BV[i] * poid;
                for (j = i; j <= MxDeg; j++)
                {
                    AuxH[i, j] += curcoeff * BV[j];
                }
            }
        }

        i1 = i0;
        for (i = 0; i <= MxDeg; i++)
        {
            j1 = j0 + i;
            for (j = i; j <= MxDeg; j++)
            {
                H[i1, j1] += myQuadraticWeight * AuxH[i, j];
                H[j1, i1] = H[i1, j1];
                j1++;
            }
            i1++;
        }
    }

    public override void Gradient(int Element, int Dimension, math_Vector G)
    {
        if (myCurve is null)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::ErrorValues");
        }

        int myNbP2d = AppDef_MyLineTool.NbP2d(mySSP), myNbP3d = AppDef_MyLineTool.NbP3d(mySSP);

        if (Dimension > (2 * myNbP2d + 3 * myNbP3d))
        {
            throw new DomainErrorException("AppDef_LinearCriteria::ErrorValues");
        }

        NCollection_Array1<Point3D> TabP3d = new(1, Max(1, myNbP3d));
        NCollection_Array1<Point2D> TabP2d = new(1, Max(1, myNbP2d));

        bool In3d;
        int IndPnt, IndCrd;

        if (Dimension <= 3 * myNbP3d)
        {
            In3d = true;
            IndCrd = Dimension % 3;
            IndPnt = Dimension / 3;
            if (IndCrd == 0)
            {
                IndCrd = 3;
            }
            else
            {
                IndPnt++;
            }
        }
        else
        {
            In3d = false;
            IndCrd = (Dimension - 3 * myNbP3d) % 2;
            IndPnt = (Dimension - 3 * myNbP3d) / 2;
            if (IndCrd == 0)
            {
                IndCrd = 2;
            }
            else
            {
                IndPnt++;
            }
        }

        NCollection_Array1<double> Knots = myCurve.Knots();
        double UFirst, ULast, Pnt;
        UFirst = Knots[Element];
        ULast = Knots[Element + 1];
        double coeff = (ULast - UFirst) / 2;

        int Order = order(myCurve.Base());

        HermitJacobi myBase = myCurve.Base();
        int MxDeg = myBase.WorkDegree();

        double curcoeff;
        int degH = 2 * Order + 1;
        int ipnt, k, i, ii, i0 = G.Lower(), di = myPntWeight.Lower() - myParameters!.Lower();

        if (myE != Element)
        {
            BuildCache(Element);
        }

        G.Init(0.0);

        // C++: BV = &myCache->Value(1) - 1, so BV[ii] is myCache(ii). The port indexes myCache directly, inside
        // the loop: OCCT takes the address before the loop even when the element has no points and no cache.
        for (ii = 1, ipnt = IF; ipnt <= IL; ipnt++)
        {
            if (In3d)
            {
                AppDef_MyLineTool.Value(mySSP, ipnt, TabP3d);
                Pnt = TabP3d[IndPnt].Coord(IndCrd);
            }
            else
            {
                if (myNbP3d == 0)
                {
                    AppDef_MyLineTool.Value(mySSP, ipnt, TabP2d);
                }
                else
                {
                    AppDef_MyLineTool.Value(mySSP, ipnt, TabP3d, TabP2d);
                }
                Pnt = TabP2d[IndPnt].Coord(IndCrd);
            }

            curcoeff = Pnt * myPntWeight[di + ipnt];
            for (i = 0; i <= MxDeg; i++, ii++)
            {
                G[i0 + i] += myCache![ii] * curcoeff;
            }
        }

        G *= 2.0 * myQuadraticWeight;

        for (i = 0; i <= degH; i++)
        {
            k = (i <= Order) ? i : i - Order - 1;
            curcoeff = Math.Pow(coeff, k);
            G[i0 + i] *= curcoeff;
        }
    }

    public override void InputVector(math_Vector X, NCollection_HArray2<NCollection_HArray1<int>> AssTable)
    {
        int NbDim = myCurve!.Dimension(), NbElm = myCurve.NbElements();
        int MxDeg = 0;
        MxDeg = myCurve.Base().WorkDegree();
        NCollection_Array2<double> CoeffEl = new(0, MxDeg, 1, NbDim);

        NCollection_HArray1<int> GlobIndex;

        int el, dim, i, i0 = X.Lower() - 1;

        for (el = 1; el <= NbElm; el++)
        {
            for (dim = 1; dim <= NbDim; dim++)
            {
                GlobIndex = AssTable.Value(dim, el);
                for (i = 0; i <= MxDeg; i++)
                {
                    CoeffEl[i, dim] = X[i0 + GlobIndex.Value(i)];
                }
            }
            myCurve.SetDegree(el, MxDeg);
            myCurve.SetElement(el, CoeffEl);
        }
    }

    public override void SetWeight(double QuadraticWeight, double QualityWeight, double percentJ1, double percentJ2,
                                   double percentJ3)
    {
        if (QuadraticWeight < 0.0 || QualityWeight < 0.0)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::SetWeight");
        }
        if (percentJ1 < 0.0 || percentJ2 < 0.0 || percentJ3 < 0.0)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::SetWeight");
        }

        myQuadraticWeight = QuadraticWeight;
        myQualityWeight = QualityWeight;

        double Total = percentJ1 + percentJ2 + percentJ3;
        myPercent[0] = percentJ1 / Total;
        myPercent[1] = percentJ2 / Total;
        myPercent[2] = percentJ3 / Total;
    }

    public override void GetWeight(out double QuadraticWeight, out double QualityWeight)
    {
        QuadraticWeight = myQuadraticWeight;
        QualityWeight = myQualityWeight;
    }

    public override void SetWeight(NCollection_Array1<double> Weight)
    {
        myPntWeight.Assign(Weight);
    }

    private void BuildCache(int Element)
    {
        double t;
        double UFirst, ULast;

        UFirst = myCurve!.Knots()[Element];
        ULast = myCurve.Knots()[Element + 1];

        IF = 0;
        for (int ipnt = myParameters!.Lower(); ipnt <= myParameters.Upper(); ipnt++)
        {
            t = myParameters.Value(ipnt);
            if ((t > UFirst && t <= ULast) || (Element == 1 && t == UFirst))
            {
                if (IF == 0)
                {
                    IF = ipnt;
                }
                IL = ipnt;
            }
            else if (t > ULast)
            {
                break;
            }
        }

        if (IF != 0)
        {
            HermitJacobi myBase = myCurve.Base();
            int order = myBase.WorkDegree() + 1;
            myCache = new NCollection_HArray1<double>(1, (IL - IF + 1) * (order));

            for (int ipnt = IF, ii = 1; ipnt <= IL; ipnt++, ii += order)
            {
                NCollection_Array1<double> BasicValue = new(myCache, ii, 0, order - 1);
                t = myParameters.Value(ipnt);
                double coeff = 2.0 / (ULast - UFirst), c0 = -(ULast + UFirst) / 2.0, s;
                s = (t + c0) * coeff;
                myBase.D0(s, BasicValue);
            }
        }
        else
        { // no points in the interval.
            IF = IL;
            IL--;
        }
        myE = Element;
    }
}
