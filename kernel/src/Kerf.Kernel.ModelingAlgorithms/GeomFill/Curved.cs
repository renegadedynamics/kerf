// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Curved.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Curved.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The Curved filling of a grid of poles: the inner poles are the average of the linear interpolations between
/// opposite boundaries (the most rounded style). With two rows, the first row is translated along the second.
/// </summary>
/// <remarks>P1 and P3 are the rows j = 1 and j = NbVPoles, P4 the column i = 1 and P2 the column i = NbUPoles.</remarks>
internal class Curved : Filling
{
    public Curved()
    {
    }

    public Curved(NCollection_Array1<Point3D> p1, NCollection_Array1<Point3D> p2, NCollection_Array1<Point3D> p3,
                           NCollection_Array1<Point3D> p4)
    {
        Init(p1, p2, p3, p4);
    }

    public Curved(NCollection_Array1<Point3D> p1, NCollection_Array1<Point3D> p2, NCollection_Array1<Point3D> p3,
                           NCollection_Array1<Point3D> p4, NCollection_Array1<double> w1, NCollection_Array1<double> w2,
                           NCollection_Array1<double> w3, NCollection_Array1<double> w4)
    {
        Init(p1, p2, p3, p4, w1, w2, w3, w4);
    }

    public Curved(NCollection_Array1<Point3D> p1, NCollection_Array1<Point3D> p2)
    {
        Init(p1, p2);
    }

    public Curved(NCollection_Array1<Point3D> p1, NCollection_Array1<Point3D> p2, NCollection_Array1<double> w1,
                           NCollection_Array1<double> w2)
    {
        Init(p1, p2, w1, w2);
    }

    public void Init(NCollection_Array1<Point3D> p1, NCollection_Array1<Point3D> p2, NCollection_Array1<Point3D> p3,
                     NCollection_Array1<Point3D> p4)
    {
        Standard_DomainError_Raise_if(p1.Length() != p3.Length() || p2.Length() != p4.Length(), " ");

        int nPolU = p1.Length();
        int nPolV = p2.Length();

        _isRational = false;

        double nu = nPolU - 1;
        double nv = nPolV - 1;
        _poles = new NCollection_HArray2<Point3D>(1, nPolU, 1, nPolV);

        // The boundaries are not modified
        int i, j, k;
        for (i = 1; i <= nPolU; i++)
        {
            _poles[i, 1] = p1[i];
            _poles[i, nPolV] = p3[i];
        }
        double pu, pu1, pv, pv1;

        for (j = 2; j <= nPolV - 1; j++)
        {
            pv = (j - 1) / nv;
            pv1 = 1 - pv;
            pv /= 2.0;
            pv1 /= 2.0;
            _poles[1, j] = p4[j];
            _poles[nPolU, j] = p2[j];
            for (i = 2; i <= nPolU - 1; i++)
            {
                pu = (i - 1) / nu;
                pu1 = 1 - pu;
                pu /= 2.0;
                pu1 /= 2.0;
                Point3D p = new();
                for (k = 1; k <= 3; k++)
                {
                    p = p.WithCoord(k, pv1 * p1[i].Coord(k) + pv * p3[i].Coord(k) + pu * p2[j].Coord(k) + pu1 * p4[j].Coord(k));
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
            pv /= 2.0;
            pv1 /= 2.0;
            _weights[1, j] = w4[j];
            _weights[nPolU, j] = w2[j];
            for (i = 2; i <= nPolU - 1; i++)
            {
                pu = (i - 1) / nu;
                pu1 = 1 - pu;
                pu /= 2.0;
                pu1 /= 2.0;
                double w = pv1 * w1[i] + pv * w3[i] + pu * w2[j] + pu1 * w4[j];
                _weights[i, j] = w;
            }
        }
    }

    public void Init(NCollection_Array1<Point3D> p1, NCollection_Array1<Point3D> p2)
    {
        int nPolU = p1.Length();
        int nPolV = p2.Length();

        _isRational = false;

        _poles = new NCollection_HArray2<Point3D>(1, nPolU, 1, nPolV);

        int i, j;

        for (j = 1; j <= nPolV; j++)
        {
            Vector3D tra = new(p2[1], p2[j]);
            for (i = 1; i <= nPolU; i++)
            {
                _poles[i, j] = p1[i].Translated(tra);
            }
        }
    }

    public void Init(NCollection_Array1<Point3D> p1, NCollection_Array1<Point3D> p2, NCollection_Array1<double> w1,
                     NCollection_Array1<double> w2)
    {
        Init(p1, p2);
        _isRational = true;

        // Initialisation des poids.
        int nPolU = w1.Length();
        int nPolV = w2.Length();

        _weights = new NCollection_HArray2<double>(1, nPolU, 1, nPolV);
        for (int j = 1; j <= nPolV; j++)
        {
            double factor = w2[j] / w1[1];
            for (int i = 1; i <= nPolU; i++)
            {
                _weights[i, j] = w1[i] * factor;
            }
        }
    }
}
