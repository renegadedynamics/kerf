// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntLinTorus.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntLinTorus.lxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntLinTorus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//-- IntAna_IntLinTorus.cxx
//-- lbr : the method using coefficients is catastrophic.
//--       Implementation of a proper solution.

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection;

/// <summary>Intersection between a line and a torus.</summary>
/// <remarks>An algorithm object: construct it with the line and the torus, or empty and call <c>Perform</c>.</remarks>
public class IntLinTorus
{
    private bool _done;
    private int _nbpt;
    private readonly Point3D[] _point = new Point3D[4];
    private readonly double[] _param = new double[4];
    private readonly double[] _fi = new double[4];
    private readonly double[] _theta = new double[4];

    public IntLinTorus()
    {
        _done = false;
        _nbpt = 0;
    }

    /// <summary>Creates the intersection between a line and a torus.</summary>
    public IntLinTorus(in Line3D l, in Torus t)
    {
        Perform(l, t);
    }

    /// <summary>Intersects a line and a torus.</summary>
    public void Perform(in Line3D l, in Torus T)
    {
        Point3D pl = l.Location();
        Direction3D dl = l.Direction();

        // Reparametrize the line:
        // set its location as nearest to the location of torus
        Point3D torLoc = T.Location();
        double paramOfNewPL = new Vector3D(pl, torLoc).Dot(new Vector3D(dl));
        Point3D newPL = new(pl.XYZ() + paramOfNewPL * dl.XYZ());

        //--------------------------------------------------------------
        //-- Coefficients of the line in the torus reference frame
        //--
        Transform3D trsf = new();
        trsf.SetTransformation(T.Position());
        newPL.Transform(trsf);
        dl.Transform(trsf);

        double a, b, c, x1, y1, z1, x0, y0, z0;
        double a0, a1, a2, a3, a4;
        double R, r, R2, r2;

        x1 = dl.X();
        y1 = dl.Y();
        z1 = dl.Z();
        x0 = newPL.X();
        y0 = newPL.Y();
        z0 = newPL.Z();
        R = T.MajorRadius();
        R2 = R * R;
        r = T.MinorRadius();
        r2 = r * r;

        a = x1 * x1 + y1 * y1 + z1 * z1;
        b = 2.0 * (x1 * x0 + y1 * y0 + z1 * z0);
        c = x0 * x0 + y0 * y0 + z0 * z0 - (R2 + r2);

        a4 = a * a;
        a3 = 2.0 * a * b;
        a2 = 2.0 * a * c + 4.0 * R2 * z1 * z1 + b * b;
        a1 = 2.0 * b * c + 8.0 * R2 * z1 * z0;
        a0 = c * c + 4.0 * R2 * (z0 * z0 - r2);

        double u, v;
        DirectPolynomialRoots mdpr = new(a4, a3, a2, a1, a0);
        if (mdpr.IsDone)
        {
            int nbsolvalid = 0;
            int n = mdpr.SolutionCount;
            int nbBadSol = 0;
            for (int i = 1; i <= n; i++)
            {
                double t = mdpr.Value(i);
                t += paramOfNewPL;
                Point3D pSolL = global::Kerf.Engine.Geometry.ElCLib.Value(t, l);
                global::Kerf.Engine.Geometry.ElSLib.Parameters(T, pSolL, out u, out v);
                Point3D pSolT = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, T);
                a0 = pSolT.SquareDistance(pSolL);

                if (a0 > 0.0000000001)
                {
                    nbBadSol++;
                }
                else
                {
                    _param[nbsolvalid] = t;
                    _fi[nbsolvalid] = u;
                    _theta[nbsolvalid] = v;
                    _point[nbsolvalid] = pSolL;
                    nbsolvalid++;
                }
            }
            if (n > 0 && nbsolvalid == 0 && nbBadSol == n)
            {
                _nbpt = 0;
                _done = false;
            }
            else
            {
                _nbpt = nbsolvalid;
                _done = true;
            }
        }
        else
        {
            _nbpt = 0;
            _done = false;
        }
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone => _done;

    /// <summary>The number of intersection points.</summary>
    /// <exception cref="NotDoneException">The computation was not done.</exception>
    public int PointCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _nbpt;
        }
    }

    /// <summary>The intersection point of range Index.</summary>
    /// <exception cref="NotDoneException">The computation was not done.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoints].</exception>
    public Point3D Value(int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbpt)
        {
            throw new OutOfRangeException();
        }
        return _point[index - 1];
    }

    /// <summary>The parameter on the line of the intersection point of range Index.</summary>
    /// <exception cref="NotDoneException">The computation was not done.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoints].</exception>
    public double ParamOnLine(int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbpt)
        {
            throw new OutOfRangeException();
        }
        return _param[index - 1];
    }

    /// <summary>The parameters on the torus of the intersection point of range Index.</summary>
    /// <exception cref="NotDoneException">The computation was not done.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoints].</exception>
    public void ParamOnTorus(int index, out double fi, out double theta)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbpt)
        {
            throw new OutOfRangeException();
        }
        fi = _fi[index - 1];
        theta = _theta[index - 1];
    }
}
