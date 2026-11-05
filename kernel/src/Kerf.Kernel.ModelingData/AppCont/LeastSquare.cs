// Port of: src/ModelingData/TKGeomBase/AppCont/AppCont_LeastSquare.hxx
// Port of: src/ModelingData/TKGeomBase/AppCont/AppCont_LeastSquare.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Approximation.Constraint;

namespace Kerf.Engine.Approximation;

/// <summary>
/// Approximates a continuous function (<see cref="Function"/>) on [U0, U1] by Bezier curves of one degree,
/// minimising the squared distance integrated with Gauss-Legendre quadrature, with optional pass-point or tangency
/// constraints at the ends. The computation runs in the constructor.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Value"/> returns the object's own MultiCurve (shared: don't modify).
/// As in OCCT, <see cref="IsDone"/> is true only when a constrained system was solved: it stays false when both ends
/// are free (the poles are still computed) or when the constraints fix every pole (occt-issues.md).
/// </remarks>
internal class LeastSquare
{
    // OCCT's struct PeriodicityInfo, declared in AppCont_LeastSquare.hxx and used only here.
    private struct PeriodicityInfo
    {
        public bool isPeriodic;
        public double myPeriod;
    }

    private readonly MultiCurve _scu;
    private readonly Matrix _points;
    private readonly Matrix _poles;
    private readonly math_Vector _param;
    private readonly Matrix _vb;
    private readonly NCollection_Array1<PeriodicityInfo> _perInfo;
    private bool _done;
    private int _degre;
    private int _nbdiscret, _nbP, _nbP2d;

    // Fix border point evaluation.
    private void FixSingleBorderPoint(global::Kerf.Engine.Approximation.Function ssp, double u, double u0, double u1,
                                      NCollection_Array1<Point2D> fix2d, NCollection_Array1<Point3D> fix)
    {
        int maxIter = 15;
        NCollection_Array1<Point3D> tabP = new(1, Max(_nbP, 1)), prevP = new(1, Max(_nbP, 1));
        NCollection_Array1<Point2D> tabP2d = new(1, Max(_nbP2d, 1)), prevP2d = new(1, Max(_nbP2d, 1));
        double mult = ((u - u0) > (u1 - u)) ? 1.0 : -1.0;
        double startParam = u, currParam, prevDist = 1.0, currDist = 1.0;

        double du = -(u1 - u0) / 2.0 * mult;
        double eps = Epsilon(1.0);
        double dd = du, dec = .1;
        for (int iter = 1; iter < maxIter; iter++)
        {
            dd *= dec;
            currParam = startParam + dd;
            ssp.Value(currParam, tabP2d, tabP);

            // from second iteration
            if (iter > 1)
            {
                currDist = 0.0;

                for (int j = 1; j <= _nbP; j++)
                {
                    currDist += tabP[j].DistanceTo(prevP[j]);
                }
                for (int j = 1; j <= _nbP2d; j++)
                {
                    currDist += tabP2d[j].DistanceTo(prevP2d[j]);
                }

                // from the third iteration
                if (iter > 2 && currDist / prevDist > 10.0)
                {
                    break;
                }
            }
            prevP.Assign(tabP);
            prevP2d.Assign(tabP2d);
            prevDist = currDist;
            if (prevDist <= eps)
            {
                break;
            }
        }
        fix2d.Assign(prevP2d);
        fix.Assign(prevP);
    }

    /// <summary>
    /// Approximates <paramref name="ssp"/> on [<paramref name="u0"/>, <paramref name="u1"/>] by Bezier curves of
    /// degree <paramref name="deg"/> (at most 23), with the constraints <paramref name="firstCons"/> and
    /// <paramref name="lastCons"/> at the ends (NoConstraint, PassPoint or TangencyPoint; a tangency the function
    /// can't give becomes a pass point), from <paramref name="myNbPoints"/> Gauss points (at most 24).
    /// </summary>
    public LeastSquare(global::Kerf.Engine.Approximation.Function ssp, double u0, double u1, Constraint firstCons,
                               Constraint lastCons, int deg, int myNbPoints)
    {
        _scu = new MultiCurve(deg + 1);
        _points = new Matrix(1, myNbPoints, 1, 3 * ssp.GetNbOf3dPoints() + 2 * ssp.GetNbOf2dPoints());
        _poles = new Matrix(1, deg + 1, 1, 3 * ssp.GetNbOf3dPoints() + 2 * ssp.GetNbOf2dPoints(), 0.0);
        _param = new math_Vector(1, myNbPoints);
        _vb = new Matrix(1, deg + 1, 1, myNbPoints);
        _perInfo = new NCollection_Array1<PeriodicityInfo>(1, 3 * ssp.GetNbOf3dPoints() + 2 * ssp.GetNbOf2dPoints());

        _done = false;
        _degre = deg;
        int i, j, k, c, i2;
        int classe = deg + 1, cl1 = deg;
        double u, dU, coeff, coeff2;
        double iBij, ibPij;

        int firstP = 1, lastP = myNbPoints;
        int nbcol = 3 * ssp.GetNbOf3dPoints() + 2 * ssp.GetNbOf2dPoints();
        Matrix b = new(1, classe, 1, nbcol, 0.0);
        int bdeb = 1, bfin = classe;
        Constraint myFirstC = firstCons, myLastC = lastCons;
        ssp.GetNumberOfPoints(out _nbP, out _nbP2d);

        int i2plus1, i2plus2;
        _nbdiscret = myNbPoints;
        NCollection_Array1<Point3D> tabP = new(1, Max(_nbP, 1));
        NCollection_Array1<Point2D> tabP2d = new(1, Max(_nbP2d, 1));
        NCollection_Array1<Vector3D> tabV = new(1, Max(_nbP, 1));
        NCollection_Array1<Vector2D> tabV2d = new(1, Max(_nbP2d, 1));

        for (int dimIdx = 1; dimIdx <= _nbP * 3 + _nbP2d * 2; dimIdx++)
        {
            ssp.PeriodInformation(dimIdx, out _perInfo[dimIdx].isPeriodic, out _perInfo[dimIdx].myPeriod);
        }

        bool ok;
        if (myFirstC == TangencyPoint)
        {
            ok = ssp.D1(u0, tabV2d, tabV);
            if (!ok)
            {
                myFirstC = PassPoint;
            }
        }

        if (myLastC == TangencyPoint)
        {
            ok = ssp.D1(u1, tabV2d, tabV);
            if (!ok)
            {
                myLastC = PassPoint;
            }
        }

        // Compute control points params on which approximation will be built.
        math_Vector gaussP = new(1, myNbPoints), gaussW = new(1, myNbPoints);
        NumericTables.GaussPoints(myNbPoints, gaussP);
        NumericTables.GaussWeights(myNbPoints, gaussW);
        math_Vector theWeights = new(1, myNbPoints), vbParam = new(1, myNbPoints);
        dU = 0.5 * (u1 - u0);
        for (i = firstP; i <= lastP; i++)
        {
            u = 0.5 * (u1 + u0) + dU * gaussP[i];
            if (i <= (myNbPoints + 1) / 2)
            {
                _param[lastP - i + 1] = u;
                vbParam[lastP - i + 1] = 0.5 * (1 + gaussP[i]);
                theWeights[lastP - i + 1] = 0.5 * gaussW[i];
            }
            else
            {
                vbParam[i - (myNbPoints + 1) / 2] = 0.5 * (1 + gaussP[i]);
                _param[i - (myNbPoints + 1) / 2] = u;
                theWeights[i - (myNbPoints + 1) / 2] = 0.5 * gaussW[i];
            }
        }

        // Compute control points.
        for (i = firstP; i <= lastP; i++)
        {
            u = _param[i];
            ssp.Value(u, tabP2d, tabP);

            i2 = 1;
            for (j = 1; j <= _nbP; j++)
            {
                tabP[j].Coord(out _points[i, i2], out _points[i, i2 + 1], out _points[i, i2 + 2]);
                i2 += 3;
            }
            for (j = 1; j <= _nbP2d; j++)
            {
                tabP2d[j].Coord(out _points[i, i2], out _points[i, i2 + 1]);
                i2 += 2;
            }
        }

        // Fix possible "period jump".
        int maxDim = 3 * _nbP + 2 * _nbP2d;
        for (int dimIdx = 1; dimIdx <= maxDim; dimIdx++)
        {
            if (_perInfo[dimIdx].isPeriodic
                && Math.Abs(_points[1, dimIdx] - _points[2, dimIdx]) > _perInfo[dimIdx].myPeriod / 2.01
                && Math.Abs(_points[2, dimIdx] - _points[3, dimIdx]) < _perInfo[dimIdx].myPeriod / 2.01)
            {
                double periodMult = (_points[1, dimIdx] < _points[2, dimIdx]) ? 1.0 : -1.0;
                double newParam = _points[1, dimIdx] + periodMult * _perInfo[dimIdx].myPeriod;
                _points[1, dimIdx] = newParam;
            }
        }
        for (int pntIdx = 1; pntIdx < myNbPoints; pntIdx++)
        {
            for (int dimIdx = 1; dimIdx <= maxDim; dimIdx++)
            {
                if (_perInfo[dimIdx].isPeriodic
                    && Math.Abs(_points[pntIdx, dimIdx] - _points[pntIdx + 1, dimIdx]) > _perInfo[dimIdx].myPeriod / 2.01)
                {
                    double periodMult = (_points[pntIdx, dimIdx] > _points[pntIdx + 1, dimIdx]) ? 1.0 : -1.0;
                    double newParam = _points[pntIdx + 1, dimIdx] + periodMult * _perInfo[dimIdx].myPeriod;
                    _points[pntIdx + 1, dimIdx] = newParam;
                }
            }
        }

        ContMatrices.VBernstein(classe, myNbPoints, _vb);

        // Processing of the right-hand side:
        NCollection_Array1<double> tmppoints = new(1, nbcol);

        for (c = 1; c <= classe; c++)
        {
            tmppoints.Init(0.0);
            for (i = 1; i <= myNbPoints; i++)
            {
                coeff = theWeights[i] * _vb[c, i];
                for (j = 1; j <= nbcol; j++)
                {
                    tmppoints[j] += _points[i, j] * coeff;
                }
            }
            for (k = 1; k <= nbcol; k++)
            {
                b[c, k] += tmppoints[k];
            }
        }

        if (myFirstC == NoConstraint && myLastC == NoConstraint)
        {
            Matrix invM = new(1, classe, 1, classe);
            ContMatrices.InvMMatrix(classe, invM);
            // Direct computation of poles:

            for (i = 1; i <= classe; i++)
            {
                for (j = 1; j <= classe; j++)
                {
                    iBij = invM[i, j];
                    for (k = 1; k <= nbcol; k++)
                    {
                        _poles[i, k] += iBij * b[j, k];
                    }
                }
            }
            // the poles are computed (OCCT left myDone false here: decision 1470)
            _done = true;
        }
        else
        {
            Matrix m = new(1, classe, 1, classe);
            ContMatrices.MMatrix(classe, m);
            NCollection_Array1<Point2D> fixP2d = new(1, Max(_nbP2d, 1));
            NCollection_Array1<Point3D> fixP = new(1, Max(_nbP, 1));

            if (myFirstC == PassPoint || myFirstC == TangencyPoint)
            {
                ssp.Value(u0, tabP2d, tabP);
                FixSingleBorderPoint(ssp, u0, u0, u1, fixP2d, fixP);

                i2 = 1;
                for (k = 1; k <= _nbP; k++)
                {
                    if (fixP[k].DistanceTo(tabP[k]) > 0.1)
                    {
                        fixP[k].Coord(out _poles[1, i2], out _poles[1, i2 + 1], out _poles[1, i2 + 2]);
                    }
                    else
                    {
                        tabP[k].Coord(out _poles[1, i2], out _poles[1, i2 + 1], out _poles[1, i2 + 2]);
                    }
                    i2 += 3;
                }
                for (k = 1; k <= _nbP2d; k++)
                {
                    if (fixP2d[k].DistanceTo(tabP2d[k]) > 0.1)
                    {
                        fixP2d[k].Coord(out _poles[1, i2], out _poles[1, i2 + 1]);
                    }
                    else
                    {
                        tabP2d[k].Coord(out _poles[1, i2], out _poles[1, i2 + 1]);
                    }
                    i2 += 2;
                }

                for (int dimIdx = 1; dimIdx <= maxDim; dimIdx++)
                {
                    if (_perInfo[dimIdx].isPeriodic
                        && Math.Abs(_poles[1, dimIdx] - _points[1, dimIdx]) > _perInfo[dimIdx].myPeriod / 2.01)
                    {
                        double mult = _poles[1, dimIdx] < _points[1, dimIdx] ? 1.0 : -1.0;
                        _poles[1, dimIdx] += mult * _perInfo[dimIdx].myPeriod;
                    }
                }
            }

            if (myLastC == PassPoint || myLastC == TangencyPoint)
            {
                ssp.Value(u1, tabP2d, tabP);
                FixSingleBorderPoint(ssp, u1, u0, u1, fixP2d, fixP);

                i2 = 1;
                for (k = 1; k <= _nbP; k++)
                {
                    if (fixP[k].DistanceTo(tabP[k]) > 0.1)
                    {
                        fixP[k].Coord(out _poles[classe, i2], out _poles[classe, i2 + 1], out _poles[classe, i2 + 2]);
                    }
                    else
                    {
                        tabP[k].Coord(out _poles[classe, i2], out _poles[classe, i2 + 1], out _poles[classe, i2 + 2]);
                    }
                    i2 += 3;
                }
                for (k = 1; k <= _nbP2d; k++)
                {
                    if (fixP2d[k].DistanceTo(tabP2d[k]) > 0.1)
                    {
                        fixP2d[k].Coord(out _poles[classe, i2], out _poles[classe, i2 + 1]);
                    }
                    else
                    {
                        tabP2d[k].Coord(out _poles[classe, i2], out _poles[classe, i2 + 1]);
                    }
                    i2 += 2;
                }

                // OCCT: only the first two coordinates are checked for a period jump here, where the first point
                // checks all aMaxDim (occt-issues.md).
                for (int dimIdx = 1; dimIdx <= 2; dimIdx++)
                {
                    if (_perInfo[dimIdx].isPeriodic
                        && Math.Abs(_poles[classe, dimIdx] - _points[myNbPoints, dimIdx]) > _perInfo[dimIdx].myPeriod / 2.01)
                    {
                        double mult = _poles[classe, dimIdx] < _points[myNbPoints, dimIdx] ? 1.0 : -1.0;
                        _poles[classe, dimIdx] += mult * _perInfo[dimIdx].myPeriod;
                    }
                }
            }

            if (myFirstC == PassPoint)
            {
                bdeb = 2;
                // Update the right-hand side:
                for (i = 1; i <= classe; i++)
                {
                    coeff = m[i, 1];
                    for (k = 1; k <= nbcol; k++)
                    {
                        b[i, k] -= _poles[1, k] * coeff;
                    }
                }
            }

            if (myLastC == PassPoint)
            {
                bfin = cl1;
                for (i = 1; i <= classe; i++)
                {
                    coeff = m[i, classe];
                    for (k = 1; k <= nbcol; k++)
                    {
                        b[i, k] -= _poles[classe, k] * coeff;
                    }
                }
            }

            if (myFirstC == TangencyPoint)
            {
                // Fix the second pole:
                bdeb = 3;
                ssp.D1(u0, tabV2d, tabV);

                i2 = 1;
                coeff = (u1 - u0) / _degre;
                for (k = 1; k <= _nbP; k++)
                {
                    i2plus1 = i2 + 1;
                    i2plus2 = i2 + 2;
                    _poles[2, i2] = _poles[1, i2] + tabV[k].X * coeff;
                    _poles[2, i2plus1] = _poles[1, i2plus1] + tabV[k].Y * coeff;
                    _poles[2, i2plus2] = _poles[1, i2plus2] + tabV[k].Z * coeff;
                    i2 += 3;
                }
                for (k = 1; k <= _nbP2d; k++)
                {
                    i2plus1 = i2 + 1;
                    _poles[2, i2] = _poles[1, i2] + tabV2d[k].X * coeff;
                    _poles[2, i2plus1] = _poles[1, i2plus1] + tabV2d[k].Y * coeff;
                    i2 += 2;
                }

                for (i = 1; i <= classe; i++)
                {
                    coeff = m[i, 1];
                    coeff2 = m[i, 2];
                    for (k = 1; k <= nbcol; k++)
                    {
                        b[i, k] -= _poles[1, k] * coeff + _poles[2, k] * coeff2;
                    }
                }
            }

            if (myLastC == TangencyPoint)
            {
                bfin = classe - 2;
                ssp.D1(u1, tabV2d, tabV);
                i2 = 1;
                coeff = (u1 - u0) / _degre;
                for (k = 1; k <= _nbP; k++)
                {
                    i2plus1 = i2 + 1;
                    i2plus2 = i2 + 2;
                    _poles[cl1, i2] = _poles[classe, i2] - tabV[k].X * coeff;
                    _poles[cl1, i2plus1] = _poles[classe, i2plus1] - tabV[k].Y * coeff;
                    _poles[cl1, i2plus2] = _poles[classe, i2plus2] - tabV[k].Z * coeff;
                    i2 += 3;
                }
                for (k = 1; k <= _nbP2d; k++)
                {
                    i2plus1 = i2 + 1;
                    _poles[cl1, i2] = _poles[classe, i2] - tabV2d[k].X * coeff;
                    _poles[cl1, i2plus1] = _poles[classe, i2plus1] - tabV2d[k].Y * coeff;
                    i2 += 2;
                }

                for (i = 1; i <= classe; i++)
                {
                    coeff = m[i, classe];
                    coeff2 = m[i, cl1];
                    for (k = 1; k <= nbcol; k++)
                    {
                        b[i, k] -= _poles[classe, k] * coeff + _poles[cl1, k] * coeff2;
                    }
                }
            }

            if (bdeb <= bfin)
            {
                Matrix b2 = new(bdeb, bfin, 1, b.UpperCol(), 0.0);

                for (i = bdeb; i <= bfin; i++)
                {
                    for (j = 1; j <= classe; j++)
                    {
                        coeff = m[i, j];
                        for (k = 1; k <= nbcol; k++)
                        {
                            b2[i, k] += b[j, k] * coeff;
                        }
                    }
                }

                // Resolution:
                // ===========
                Matrix ibp = new(bdeb, bfin, bdeb, bfin);

                // In IBPMatrix and IBTMatrix only results for a degree
                // less than or equal to 26 are stored (for now at least).

                if (bdeb == 2 && bfin == classe - 1 && classe <= 26)
                {
                    ContMatrices.IBPMatrix(classe, ibp);
                }
                else if (bdeb == 3 && bfin == classe - 2 && classe <= 26)
                {
                    ContMatrices.IBTMatrix(classe, ibp);
                }
                else
                {
                    Matrix mp = new(1, classe, bdeb, bfin);
                    for (i = 1; i <= classe; i++)
                    {
                        for (j = bdeb; j <= bfin; j++)
                        {
                            mp[i, j] = m[i, j];
                        }
                    }
                    Matrix ibp1 = new(bdeb, bfin, bdeb, bfin);
                    ibp1.Initialized(mp.Transposed() * mp);
                    ibp.Initialized(ibp1.Inverse());
                }

                _done = true;
                for (i = bdeb; i <= bfin; i++)
                {
                    for (j = bdeb; j <= bfin; j++)
                    {
                        ibPij = ibp[i, j];
                        for (k = 1; k <= nbcol; k++)
                        {
                            _poles[i, k] += ibPij * b2[j, k];
                        }
                    }
                }
            }
        }
    }

    /// <summary>The approximation: Degree + 1 MultiPoints, rebuilt from the poles at each call (shared: don't modify).</summary>
    public MultiCurve Value()
    {
        int i, j, j2;
        Point3D pt = new();
        Point2D pt2d = new();
        int ideb = 1, ifin = _degre + 1;

        // Store the result in the corresponding curves
        for (i = ideb; i <= ifin; i++)
        {
            j2 = 1;
            MultiPoint mPole = new(_nbP, _nbP2d);
            for (j = 1; j <= _nbP; j++)
            {
                pt = pt.WithCoord(_poles[i, j2], _poles[i, j2 + 1], _poles[i, j2 + 2]);
                mPole.SetPoint(j, pt);
                j2 += 3;
            }
            for (j = _nbP + 1; j <= _nbP + _nbP2d; j++)
            {
                pt2d = pt2d.WithCoord(_poles[i, j2], _poles[i, j2 + 1]);
                mPole.SetPoint2d(j, pt2d);
                j2 += 2;
            }
            _scu.SetValue(i, mPole);
        }
        return _scu;
    }

    /// <summary>
    /// The errors at the Gauss points: <paramref name="f"/> the sum of the squared distances,
    /// <paramref name="maxE3d"/> and <paramref name="maxE2d"/> the largest 3d and 2d distances.
    /// </summary>
    public void Error(out double f, out double maxE3d, out double maxE2d)
    {
        int i, j, k, c, i2, classe = _degre + 1;
        double coeff, err3d = 0.0, err2d = 0.0;
        int ncol = _points.UpperCol() - _points.LowerCol() + 1;

        Matrix myPoints = new(1, _nbdiscret, 1, ncol);
        myPoints.Initialized(_points);

        maxE3d = maxE2d = f = 0.0;

        NCollection_Array1<double> tmppoles = new(1, ncol);

        for (c = 1; c <= classe; c++)
        {
            for (k = 1; k <= ncol; k++)
            {
                tmppoles[k] = _poles[c, k];
            }
            for (i = 1; i <= _nbdiscret; i++)
            {
                coeff = _vb[c, i];
                for (j = 1; j <= ncol; j++)
                {
                    myPoints[i, j] -= tmppoles[j] * coeff;
                }
            }
        }

        double e1, e2, e3;
        for (i = 1; i <= _nbdiscret; i++)
        {
            i2 = 1;
            for (j = 1; j <= _nbP; j++)
            {
                e1 = myPoints[i, i2];
                e2 = myPoints[i, i2 + 1];
                e3 = myPoints[i, i2 + 2];
                err3d = e1 * e1 + e2 * e2 + e3 * e3;
                maxE3d = Max(maxE3d, err3d);
                f += err3d;
                i2 += 3;
            }
            for (j = 1; j <= _nbP2d; j++)
            {
                e1 = myPoints[i, i2];
                e2 = myPoints[i, i2 + 1];
                err2d = e1 * e1 + e2 * e2;
                maxE2d = Max(maxE2d, err2d);
                f += err2d;
                i2 += 2;
            }
        }

        maxE3d = Math.Sqrt(maxE3d);
        maxE2d = Math.Sqrt(maxE2d);
    }

    /// <summary>True if a constrained system was solved (see the remarks).</summary>
    public bool IsDone => _done;
}
