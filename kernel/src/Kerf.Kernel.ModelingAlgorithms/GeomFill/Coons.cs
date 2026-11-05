// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Coons.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Coons.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The Coons filling of a grid of poles: the boundary rows are kept and the inner poles blend them with cubic
/// Hermite functions (the bicubically blended Coons patch).
/// </summary>
/// <remarks>
/// P1 and P3 are the rows j = 1 and j = NbVPoles, P2 the column i = 1 and P4 the column i = NbUPoles (for the
/// poles; OCCT's weights put W4 in column 1 and W2 in the last column, occt-issues.md).
/// </remarks>
internal class Coons : Filling
{
    public Coons()
    {
    }

    public Coons(NCollection_Array1<Point3D> p1, NCollection_Array1<Point3D> p2, NCollection_Array1<Point3D> p3,
                          NCollection_Array1<Point3D> p4)
    {
        Init(p1, p2, p3, p4);
    }

    public Coons(NCollection_Array1<Point3D> p1, NCollection_Array1<Point3D> p2, NCollection_Array1<Point3D> p3,
                          NCollection_Array1<Point3D> p4, NCollection_Array1<double> w1, NCollection_Array1<double> w2,
                          NCollection_Array1<double> w3, NCollection_Array1<double> w4)
    {
        Init(p1, p2, p3, p4, w1, w2, w3, w4);
    }

    public void Init(NCollection_Array1<Point3D> p1, NCollection_Array1<Point3D> p2, NCollection_Array1<Point3D> p3,
                     NCollection_Array1<Point3D> p4)
    {
        Standard_DomainError_Raise_if(p1.Length() != p3.Length() || p2.Length() != p4.Length(), " ");

        int nPolU = p1.Length();
        int nPolV = p2.Length();

        _isRational = false;
        _poles = new NCollection_HArray2<Point3D>(1, nPolU, 1, nPolV);

        // The boundaries are not modified
        int i, j, k;
        for (i = 1; i <= nPolU; i++)
        {
            _poles[i, 1] = p1[i];
            _poles[i, nPolV] = p3[i];
        }
        for (i = 1; i <= nPolV; i++)
        {
            _poles[1, i] = p2[i];
            _poles[nPolU, i] = p4[i];
        }

        // Calcul des coefficients multiplicateurs
        NCollection_Array1<Point3D> coef = new(1, 4);
        NCollection_Array1<Point3D> pole = new(1, 4);
        NCollection_Array1<Point3D> coefU = new(1, nPolU);
        NCollection_Array1<Point3D> coefV = new(1, nPolV);
        coef[4] = new Point3D(2.0, -2.0, 0.0);
        coef[3] = new Point3D(-3.0, 3.0, 0.0);
        coef[2] = new Point3D(0.0, 0.0, 0.0);
        coef[1] = new Point3D(1.0, 0.0, 0.0);
        global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(coef, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), pole, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());

        if (nPolU > 4)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(nPolU - 1, pole, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), coefU, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
        }
        else
        {
            coefU.Assign(pole);
        }

        if (nPolV > 4)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(nPolV - 1, pole, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), coefV, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
        }
        else
        {
            coefV.Assign(pole);
        }

        NCollection_Array1<double> fu = new(2, nPolU - 1);
        NCollection_Array1<double> gu = new(2, nPolU - 1);
        NCollection_Array1<double> fv = new(2, nPolV - 1);
        NCollection_Array1<double> gv = new(2, nPolV - 1);

        double dummy;
        for (i = 2; i < nPolU; i++)
        {
            coefU[i].Coord(out fu[i], out gu[i], out dummy);
        }
        for (i = 2; i < nPolV; i++)
        {
            coefV[i].Coord(out fv[i], out gv[i], out dummy);
        }

        // Clacul des poles interieurs
        Point3D p = new();
        for (j = 2; j < nPolV; j++)
        {
            for (i = 2; i < nPolU; i++)
            {
                for (k = 1; k <= 3; k++)
                {
                    p = p.WithCoord(k,
                               fv[j] * (_poles.Value(i, 1)).Coord(k) + gv[j] * (_poles.Value(i, nPolV)).Coord(k)
                               + fu[i] * (_poles.Value(1, j)).Coord(k) + gu[i] * (_poles.Value(nPolU, j)).Coord(k)
                               - fu[i] * fv[j] * (_poles.Value(1, 1)).Coord(k)
                               - fu[i] * gv[j] * (_poles.Value(1, nPolV)).Coord(k)
                               - gu[i] * fv[j] * (_poles.Value(nPolU, 1)).Coord(k)
                               - gu[i] * gv[j] * (_poles.Value(nPolU, nPolV)).Coord(k));
                }
                _poles[i, j] = p;
            }
        }
    }

    public void Init(NCollection_Array1<Point3D> p1, NCollection_Array1<Point3D> p2, NCollection_Array1<Point3D> p3,
                     NCollection_Array1<Point3D> p4, NCollection_Array1<double> w1, NCollection_Array1<double> w2,
                     NCollection_Array1<double> w3, NCollection_Array1<double> w4)
    {
        Standard_DomainError_Raise_if(w1.Length() != w3.Length() || w2.Length() != w4.Length(), " ");
        Standard_DomainError_Raise_if(w1.Length() != p1.Length() || w2.Length() != p2.Length()
                                      || w3.Length() != p3.Length() || w4.Length() != p4.Length(), " ");

        Init(p1, p2, p3, p4);
        _isRational = true;

        int nPolU = w1.Length();
        int nPolV = w2.Length();

        double nu = nPolU - 1;
        double nv = nPolV - 1;
        _weights = new NCollection_HArray2<double>(1, nPolU, 1, nPolV);
        // The boundaries are not modified
        int i, j;
        for (i = 1; i <= nPolU; i++)
        {
            _weights[i, 1] = w1[i];
            _weights[i, nPolV] = w3[i];
        }
        double pu, pu1, pv, pv1;

        for (j = 2; j <= nPolV - 1; j++)
        {
            pv = (j - 1) / nv;
            pv1 = 1 - pv;
            // W2 in the first column and W4 in the last, as the poles have P2 and P4 (decision 1479; OCCT put W4
            // first and W2 last, and blended them so)
            _weights[1, j] = w2[j];
            _weights[nPolU, j] = w4[j];
            for (i = 2; i <= nPolU - 1; i++)
            {
                pu = (i - 1) / nu;
                pu1 = 1 - pu;
                double w = pv1 * w1[i] + pv * w3[i] + pu1 * w2[j] + pu * w4[j]
                           - (pu1 * pv1 * w1[1] + pu * pv1 * w4[1] + pu * pv * w3[nPolU] + pu1 * pv * w2[nPolV]);
                _weights[i, j] = w;
            }
        }
    }
}
