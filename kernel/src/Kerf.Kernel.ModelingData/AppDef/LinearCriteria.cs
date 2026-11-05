// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_LinearCriteria.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_LinearCriteria.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.FiniteElements;
using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Engine.Numerics.Polynomials;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Approximation;

/// <summary>
/// The linear criteria used by <see cref="Variational"/> to smooth points: the least-squares distance to the
/// points plus the tension, flexion and jerk of the curve (FEmTool_LinearTension, LinearFlexion, LinearJerk).
/// </summary>
/// <remarks>A working object (decisions 21 and 207); the MultiLine, parameters and curve it is given are shared.</remarks>
internal class LinearCriteria : SmoothCriterion
{
    private readonly MultiLine _ssp;
    private NCollection_HArray1<double>? _parameters;
    private NCollection_HArray1<double>? _cache;
    private readonly ElementaryCriterion[] _criteria = new ElementaryCriterion[3];
    private readonly double[] _estimation = new double[3];
    private double _quadraticWeight;
    private double _qualityWeight;
    private readonly double[] _percent = new double[3];
    private readonly NCollection_Array1<double> _pntWeight;
    private Curve? _curve;
    private double _length;
    private int _e;
    private int _if;
    private int _il;

    private static int Order(HermitJacobi b)
    {
        return b.NivConstr();
    }

    /// <summary>The criteria for the points <paramref name="firstPoint"/> to <paramref name="lastPoint"/> of <paramref name="ssp"/>.</summary>
    public LinearCriteria(MultiLine ssp, int firstPoint, int lastPoint)
    {
        _ssp = ssp;
        _quadraticWeight = 0.0;
        _qualityWeight = 0.0;
        _pntWeight = new NCollection_Array1<double>(firstPoint, lastPoint);
        _length = 0.0;
        _e = 0;
        _if = 0;
        _il = 0;
        _pntWeight.Init(1.0);
    }

    public override void SetParameters(NCollection_HArray1<double> parameters)
    {
        _parameters = parameters;
        _e = 0; // Cache become invalid.
    }

    public override void SetCurve(Curve c)
    {
        if (_curve is null)
        {
            _curve = c;

            int mxDeg = _curve.Base().WorkDegree(), nbDim = _curve.Dimension(),
                order = global::Kerf.Engine.Approximation.LinearCriteria.Order(_curve.Base());

            Continuity constraintOrder = C0;
            switch (order)
            {
                case 0:
                    constraintOrder = C0;
                    break;
                case 1:
                    constraintOrder = C1;
                    break;
                case 2:
                    constraintOrder = C2;
                    break;
            }

            _criteria[0] = new LinearTension(mxDeg, constraintOrder);
            _criteria[1] = new LinearFlexion(mxDeg, constraintOrder);
            _criteria[2] = new LinearJerk(mxDeg, constraintOrder);

            NCollection_HArray2<double> coeff = new(0, 0, 1, nbDim);

            _criteria[0].Set(coeff);
            _criteria[1].Set(coeff);
            _criteria[2].Set(coeff);
        }
        else if (_curve != c)
        {
            int oldMxDeg = _curve.Base().WorkDegree(), oldNbDim = _curve.Dimension(),
                oldOrder = global::Kerf.Engine.Approximation.LinearCriteria.Order(_curve.Base());

            _curve = c;

            int mxDeg = _curve.Base().WorkDegree(), nbDim = _curve.Dimension(),
                order = global::Kerf.Engine.Approximation.LinearCriteria.Order(_curve.Base());

            if (mxDeg != oldMxDeg || order != oldOrder)
            {
                Continuity constraintOrder = C0;
                switch (order)
                {
                    case 0:
                        constraintOrder = C0;
                        break;
                    case 1:
                        constraintOrder = C1;
                        break;
                    case 2:
                        constraintOrder = C2;
                        break;
                }

                _criteria[0] = new LinearTension(mxDeg, constraintOrder);
                _criteria[1] = new LinearFlexion(mxDeg, constraintOrder);
                _criteria[2] = new LinearJerk(mxDeg, constraintOrder);

                NCollection_HArray2<double> coeff = new(0, 0, 1, nbDim);

                _criteria[0].Set(coeff);
                _criteria[1].Set(coeff);
                _criteria[2].Set(coeff);
            }
            else if (nbDim != oldNbDim)
            {
                NCollection_HArray2<double> coeff = new(0, 0, 1, nbDim);

                _criteria[0].Set(coeff);
                _criteria[1].Set(coeff);
                _criteria[2].Set(coeff);
            }
        }
    }

    public override void GetCurve(out Curve? c)
    {
        c = _curve;
    }

    public override void SetEstimation(double e1, double e2, double e3)
    {
        _estimation[0] = e1;
        _estimation[1] = e2;
        _estimation[2] = e3;
    }

    public override ref double EstLength()
    {
        return ref _length;
    }

    public override void GetEstimation(out double e1, out double e2, out double e3)
    {
        e1 = _estimation[0];
        e2 = _estimation[1];
        e3 = _estimation[2];
    }

    public override NCollection_HArray2<NCollection_HArray1<int>> AssemblyTable()
    {
        if (_curve is null)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::AssemblyTable");
        }

        int nbDim = _curve.Dimension(), nbElm = _curve.ElementCount, nc1 = Order(_curve.Base()) + 1;
        int mxDeg = _curve.Base().WorkDegree();

        NCollection_HArray2<NCollection_HArray1<int>> assTable = new(1, nbDim, 1, nbElm);

        NCollection_HArray1<int> globIndex, aux;

        int i, el = 1, dim = 1, nbGlobVar = 0, gi0;

        // For dim = 1
        // For first element (el = 1)
        globIndex = new NCollection_HArray1<int>(0, mxDeg);

        for (i = 0; i < nc1; i++)
        {
            nbGlobVar++;
            globIndex.SetValue(i, nbGlobVar);
        }
        gi0 = mxDeg - 2 * nc1 + 1;
        for (i = nc1; i < 2 * nc1; i++)
        {
            nbGlobVar++;
            globIndex.SetValue(i, nbGlobVar + gi0);
        }
        for (i = 2 * nc1; i <= mxDeg; i++)
        {
            nbGlobVar++;
            globIndex.SetValue(i, nbGlobVar - nc1);
        }
        gi0 = nbGlobVar - nc1 + 1;
        assTable.SetValue(dim, el, globIndex);

        // For rest elements
        for (el = 2; el <= nbElm; el++)
        {
            globIndex = new NCollection_HArray1<int>(0, mxDeg);
            for (i = 0; i < nc1; i++)
            {
                globIndex.SetValue(i, gi0 + i);
            }

            gi0 = mxDeg - 2 * nc1 + 1;
            for (i = nc1; i < 2 * nc1; i++)
            {
                nbGlobVar++;
                globIndex.SetValue(i, nbGlobVar + gi0);
            }
            for (i = 2 * nc1; i <= mxDeg; i++)
            {
                nbGlobVar++;
                globIndex.SetValue(i, nbGlobVar - nc1);
            }
            gi0 = nbGlobVar - nc1 + 1;
            assTable.SetValue(dim, el, globIndex);
        }

        // For other dimensions
        gi0 = nbGlobVar;
        for (dim = 2; dim <= nbDim; dim++)
        {
            for (el = 1; el <= nbElm; el++)
            {
                aux = assTable.Value(1, el);
                globIndex = new NCollection_HArray1<int>(0, mxDeg);
                for (i = 0; i <= mxDeg; i++)
                {
                    globIndex.SetValue(i, aux.Value(i) + nbGlobVar);
                }
                assTable.SetValue(dim, el, globIndex);
            }
            nbGlobVar += gi0;
        }

        return assTable;
    }

    public override NCollection_HArray2<int> DependenceTable()
    {
        if (_curve is null)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::DependenceTable");
        }

        int dim = _curve.Dimension();

        NCollection_HArray2<int> depTab = new(1, dim, 1, dim, 0);
        int i;
        for (i = 1; i <= dim; i++)
        {
            depTab.SetValue(i, i, 1);
        }

        return depTab;
    }

    public override int QualityValues(double j1min, double j2min, double j3min, out double j1, out double j2,
                                      out double j3)
    {
        if (_curve is null)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::QualityValues");
        }

        int nbDim = _curve.Dimension(), nbElm = _curve.ElementCount;

        NCollection_Array1<double> knots = _curve.Knots();
        NCollection_HArray2<double>? coeff = null;

        int el, deg = 0, curdeg, i;
        double uFirst, uLast;

        j1 = j2 = j3 = 0.0;
        for (el = 1; el <= nbElm; el++)
        {
            curdeg = _curve.Degree(el);
            if (deg != curdeg)
            {
                deg = curdeg;
                coeff = new NCollection_HArray2<double>(0, deg, 1, nbDim);
            }

            _curve.GetElement(el, coeff!.ChangeArray2());

            uFirst = knots[el];
            uLast = knots[el + 1];

            _criteria[0].Set(coeff);
            _criteria[0].Set(uFirst, uLast);
            j1 = j1 + _criteria[0].Value();

            _criteria[1].Set(coeff);
            _criteria[1].Set(uFirst, uLast);
            j2 = j2 + _criteria[1].Value();

            _criteria[2].Set(coeff);
            _criteria[2].Set(uFirst, uLast);
            j3 = j3 + _criteria[2].Value();
        }

        // Calculation of ICDANA - see MOTEST.f
        //  double JEsMin[3] = {.01, .001, .001}; // from MOTLIS.f
        Span<double> jEsMin = stackalloc double[3];
        jEsMin[0] = j1min;
        jEsMin[1] = j2min;
        jEsMin[2] = j3min;
        Span<double> valCri = stackalloc double[3];
        valCri[0] = j1;
        valCri[1] = j2;
        valCri[2] = j3;

        int icdana = 0;

        //   (2) Test the improvement of estimates
        //       (overestimated criterion => No minimization)

        for (i = 0; i <= 2; i++)
        {
            if ((valCri[i] < 0.8 * _estimation[i]) && (_estimation[i] > jEsMin[i]))
            {
                if (icdana < 1)
                {
                    icdana = 1;
                }
                if (valCri[i] < 0.1 * _estimation[i])
                {
                    icdana = 2;
                }
                _estimation[i] = Max(1.05 * valCri[i], jEsMin[i]);
            }
        }

        //  (3) Update the Estimates
        //     (underestimated criterion => poor conditioning)
        if (valCri[0] > _estimation[0] * 2)
        {
            _estimation[0] += valCri[0] * .1;
            if (icdana == 0)
            {
                if (valCri[0] > _estimation[0] * 10)
                {
                    icdana = 2;
                }
                else
                {
                    icdana = 1;
                }
            }
            else
            {
                icdana = 2;
            }
        }
        if (valCri[1] > _estimation[1] * 20)
        {
            _estimation[1] += valCri[1] * .1;
            if (icdana == 0)
            {
                if (valCri[1] > _estimation[1] * 100)
                {
                    icdana = 2;
                }
                else
                {
                    icdana = 1;
                }
            }
            else
            {
                icdana = 2;
            }
        }
        if (valCri[2] > _estimation[2] * 20)
        {
            _estimation[2] += valCri[2] * .05;
            if (icdana == 0)
            {
                if (valCri[2] > _estimation[2] * 100)
                {
                    icdana = 2;
                }
                else
                {
                    icdana = 1;
                }
            }
            else
            {
                icdana = 2;
            }
        }

        return icdana;
    }

    public override void ErrorValues(out double maxError, out double quadraticError, out double averageError)
    {
        if (_curve is null)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::ErrorValues");
        }

        int nbDim = _curve.Dimension();

        int myNbP2d = MyLineTool.NbP2d(_ssp), myNbP3d = MyLineTool.NbP3d(_ssp);

        if (nbDim != (2 * myNbP2d + 3 * myNbP3d))
        {
            throw new DomainErrorException("AppDef_LinearCriteria::ErrorValues");
        }

        NCollection_Array1<Point3D> tabP3d = new(1, Max(1, myNbP3d));
        NCollection_Array1<Point2D> tabP2d = new(1, Max(1, myNbP2d));
        NCollection_Array1<double> basePoint = new(1, nbDim);
        Point2D p2d = new();
        Point3D p3d = new();

        int i, ipnt, c0 = 0;
        double sqrDist, dist;

        maxError = quadraticError = averageError = 0.0;

        for (i = _parameters!.Lower(); i <= _parameters.Upper(); i++)
        {
            _curve.D0(_parameters.Value(i), basePoint);

            c0 = 0;
            MyLineTool.Value(_ssp, i, tabP3d);
            for (ipnt = 1; ipnt <= myNbP3d; ipnt++)
            {
                p3d = p3d.WithCoord(basePoint[c0 + 1], basePoint[c0 + 2], basePoint[c0 + 3]);
                sqrDist = p3d.SquareDistanceTo(tabP3d[ipnt]);
                dist = Math.Sqrt(sqrDist);
                maxError = Max(maxError, dist);
                quadraticError += sqrDist;
                averageError += dist;
                c0 += 3;
            }

            if (myNbP3d == 0)
            {
                MyLineTool.Value(_ssp, i, tabP2d);
            }
            else
            {
                MyLineTool.Value(_ssp, i, tabP3d, tabP2d);
            }
            for (ipnt = 1; ipnt <= myNbP2d; ipnt++)
            {
                p2d = p2d.WithCoord(basePoint[c0 + 1], basePoint[c0 + 2]);
                sqrDist = p2d.SquareDistanceTo(tabP2d[ipnt]);
                dist = Math.Sqrt(sqrDist);
                maxError = Max(maxError, dist);
                quadraticError += sqrDist;
                averageError += dist;
                c0 += 2;
            }
        }
    }

    public override void Hessian(int element, int dimension1, int dimension2, Matrix h)
    {
        if (_curve is null)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::Hessian");
        }

        if (DependenceTable().Value(dimension1, dimension2) == 0)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::Hessian");
        }

        int mxDeg = _curve.Base().WorkDegree(),
            order = global::Kerf.Engine.Approximation.LinearCriteria.Order(_curve.Base());

        Matrix auxH = new(0, h.RowNumber() - 1, 0, h.ColNumber() - 1, 0.0);

        NCollection_Array1<double> knots = _curve.Knots();
        double uFirst, uLast;

        uFirst = knots[element];
        uLast = knots[element + 1];

        int icrit;

        // Quality criterion part of Hessian

        h.Init(0);

        for (icrit = 0; icrit <= 2; icrit++)
        {
            _criteria[icrit].Set(uFirst, uLast);
            _criteria[icrit].Hessian(dimension1, dimension2, auxH);
            h += (_qualityWeight * _percent[icrit] / _estimation[icrit]) * auxH;
        }

        // Least square part of Hessian

        auxH.Init(0.0);

        double coeff = (uLast - uFirst) / 2.0, curcoeff, poid;
        int ipnt, ii, degH = 2 * order + 1;

        int k1, k2, i, j, i0 = h.LowerRow(), j0 = h.LowerCol(), i1, j1,
            di = _pntWeight.Lower() - _parameters!.Lower();

        // BuilCache
        if (_e != element)
        {
            BuildCache(element);
        }

        // Compute the least square Hessian
        for (ii = 1, ipnt = _if; ipnt <= _il; ipnt++, ii += (mxDeg + 1))
        {
            poid = _pntWeight[di + ipnt] * 2.0;
            ReadOnlySpan<double> bv = _cache!.AsSpan(ii);

            // Hermite*Hermite part of matrix
            for (i = 0; i <= degH; i++)
            {
                k1 = (i <= order) ? i : i - order - 1;
                curcoeff = Math.Pow(coeff, k1) * poid * bv[i];

                // Hermite*Hermite part of matrix
                for (j = i; j <= degH; j++)
                {
                    k2 = (j <= order) ? j : j - order - 1;
                    auxH[i, j] += curcoeff * Math.Pow(coeff, k2) * bv[j];
                }
                // Hermite*Jacobi part of matrix
                for (j = degH + 1; j <= mxDeg; j++)
                {
                    auxH[i, j] += curcoeff * bv[j];
                }
            }

            // Jacoby*Jacobi part of matrix
            for (i = degH + 1; i <= mxDeg; i++)
            {
                curcoeff = bv[i] * poid;
                for (j = i; j <= mxDeg; j++)
                {
                    auxH[i, j] += curcoeff * bv[j];
                }
            }
        }

        i1 = i0;
        for (i = 0; i <= mxDeg; i++)
        {
            j1 = j0 + i;
            for (j = i; j <= mxDeg; j++)
            {
                h[i1, j1] += _quadraticWeight * auxH[i, j];
                h[j1, i1] = h[i1, j1];
                j1++;
            }
            i1++;
        }
    }

    public override void Gradient(int element, int dimension, math_Vector g)
    {
        if (_curve is null)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::ErrorValues");
        }

        int myNbP2d = MyLineTool.NbP2d(_ssp), myNbP3d = MyLineTool.NbP3d(_ssp);

        if (dimension > (2 * myNbP2d + 3 * myNbP3d))
        {
            throw new DomainErrorException("AppDef_LinearCriteria::ErrorValues");
        }

        NCollection_Array1<Point3D> tabP3d = new(1, Max(1, myNbP3d));
        NCollection_Array1<Point2D> tabP2d = new(1, Max(1, myNbP2d));

        bool in3d;
        int indPnt, indCrd;

        if (dimension <= 3 * myNbP3d)
        {
            in3d = true;
            indCrd = dimension % 3;
            indPnt = dimension / 3;
            if (indCrd == 0)
            {
                indCrd = 3;
            }
            else
            {
                indPnt++;
            }
        }
        else
        {
            in3d = false;
            indCrd = (dimension - 3 * myNbP3d) % 2;
            indPnt = (dimension - 3 * myNbP3d) / 2;
            if (indCrd == 0)
            {
                indCrd = 2;
            }
            else
            {
                indPnt++;
            }
        }

        NCollection_Array1<double> knots = _curve.Knots();
        double uFirst, uLast, pnt;
        uFirst = knots[element];
        uLast = knots[element + 1];
        double coeff = (uLast - uFirst) / 2;

        int order = global::Kerf.Engine.Approximation.LinearCriteria.Order(_curve.Base());

        HermitJacobi myBase = _curve.Base();
        int mxDeg = myBase.WorkDegree();

        double curcoeff;
        int degH = 2 * order + 1;
        int ipnt, k, i, ii, i0 = g.Lower(), di = _pntWeight.Lower() - _parameters!.Lower();

        if (_e != element)
        {
            BuildCache(element);
        }

        g.Init(0.0);

        // C++: BV = &myCache->Value(1) - 1, so BV[ii] is myCache(ii). The port indexes myCache directly, inside
        // the loop: OCCT takes the address before the loop even when the element has no points and no cache.
        for (ii = 1, ipnt = _if; ipnt <= _il; ipnt++)
        {
            if (in3d)
            {
                MyLineTool.Value(_ssp, ipnt, tabP3d);
                pnt = tabP3d[indPnt].Coord(indCrd);
            }
            else
            {
                if (myNbP3d == 0)
                {
                    MyLineTool.Value(_ssp, ipnt, tabP2d);
                }
                else
                {
                    MyLineTool.Value(_ssp, ipnt, tabP3d, tabP2d);
                }
                pnt = tabP2d[indPnt].Coord(indCrd);
            }

            curcoeff = pnt * _pntWeight[di + ipnt];
            for (i = 0; i <= mxDeg; i++, ii++)
            {
                g[i0 + i] += _cache![ii] * curcoeff;
            }
        }

        g *= 2.0 * _quadraticWeight;

        for (i = 0; i <= degH; i++)
        {
            k = (i <= order) ? i : i - order - 1;
            curcoeff = Math.Pow(coeff, k);
            g[i0 + i] *= curcoeff;
        }
    }

    public override void InputVector(math_Vector x, NCollection_HArray2<NCollection_HArray1<int>> assTable)
    {
        int nbDim = _curve!.Dimension(), nbElm = _curve.ElementCount;
        int mxDeg = 0;
        mxDeg = _curve.Base().WorkDegree();
        NCollection_Array2<double> coeffEl = new(0, mxDeg, 1, nbDim);

        NCollection_HArray1<int> globIndex;

        int el, dim, i, i0 = x.Lower() - 1;

        for (el = 1; el <= nbElm; el++)
        {
            for (dim = 1; dim <= nbDim; dim++)
            {
                globIndex = assTable.Value(dim, el);
                for (i = 0; i <= mxDeg; i++)
                {
                    coeffEl[i, dim] = x[i0 + globIndex.Value(i)];
                }
            }
            _curve.SetDegree(el, mxDeg);
            _curve.SetElement(el, coeffEl);
        }
    }

    public override void SetWeight(double quadraticWeight, double qualityWeight, double percentJ1, double percentJ2,
                                   double percentJ3)
    {
        if (quadraticWeight < 0.0 || qualityWeight < 0.0)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::SetWeight");
        }
        if (percentJ1 < 0.0 || percentJ2 < 0.0 || percentJ3 < 0.0)
        {
            throw new DomainErrorException("AppDef_LinearCriteria::SetWeight");
        }

        _quadraticWeight = quadraticWeight;
        _qualityWeight = qualityWeight;

        double total = percentJ1 + percentJ2 + percentJ3;
        _percent[0] = percentJ1 / total;
        _percent[1] = percentJ2 / total;
        _percent[2] = percentJ3 / total;
    }

    public override void GetWeight(out double quadraticWeight, out double qualityWeight)
    {
        quadraticWeight = _quadraticWeight;
        qualityWeight = _qualityWeight;
    }

    public override void SetWeight(NCollection_Array1<double> weight)
    {
        _pntWeight.Assign(weight);
    }

    private void BuildCache(int element)
    {
        double t;
        double uFirst, uLast;

        uFirst = _curve!.Knots()[element];
        uLast = _curve.Knots()[element + 1];

        _if = 0;
        for (int ipnt = _parameters!.Lower(); ipnt <= _parameters.Upper(); ipnt++)
        {
            t = _parameters.Value(ipnt);
            if ((t > uFirst && t <= uLast) || (element == 1 && t == uFirst))
            {
                if (_if == 0)
                {
                    _if = ipnt;
                }
                _il = ipnt;
            }
            else if (t > uLast)
            {
                break;
            }
        }

        if (_if != 0)
        {
            HermitJacobi myBase = _curve.Base();
            int order = myBase.WorkDegree() + 1;
            _cache = new NCollection_HArray1<double>(1, (_il - _if + 1) * (order));

            for (int ipnt = _if, ii = 1; ipnt <= _il; ipnt++, ii += order)
            {
                NCollection_Array1<double> basicValue = new(_cache, ii, 0, order - 1);
                t = _parameters.Value(ipnt);
                double coeff = 2.0 / (uLast - uFirst), c0 = -(uLast + uFirst) / 2.0, s;
                s = (t + c0) * coeff;
                myBase.D0(s, basicValue);
            }
        }
        else
        { // no points in the interval.
            _if = _il;
            _il--;
        }
        _e = element;
    }
}
