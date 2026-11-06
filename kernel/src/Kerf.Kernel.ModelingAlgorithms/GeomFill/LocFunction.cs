// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_LocFunction.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_LocFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A location law as a function of 12 reals (the vector and the three columns of the matrix), for the
/// approximation of the law (AdvApprox).
/// </summary>
/// <remarks>A working object: evaluation stores its results in the function. It keeps the caller's law.</remarks>
internal sealed class LocFunction
{
    private readonly SurfaceLocationLaw _law;
    private readonly NCollection_Array1<Vector3D> _v;
    private readonly NCollection_Array1<Vector3D> _dv;
    private readonly NCollection_Array1<Vector3D> _d2V;
    // OCCT's members M, DM and D2M are never used and are left out.

    public LocFunction(SurfaceLocationLaw law)
    {
        _v = new NCollection_Array1<Vector3D>(1, 4);
        _dv = new NCollection_Array1<Vector3D>(1, 4);
        _d2V = new NCollection_Array1<Vector3D>(1, 4);
        _law = law;
    }

    /// <summary>Computes the location for v = Param.</summary>
    public bool D0(double param, double first, double last)
    {
        Matrix3x3 m = new Matrix3x3();
        bool b;
        b = _law.D0(param, ref m, ref _v[1]);
        _v[2] = _v[2].WithXYZ(m.Column(1));
        _v[3] = _v[3].WithXYZ(m.Column(2));
        _v[4] = _v[4].WithXYZ(m.Column(3));
        return b;
    }

    /// <summary>Computes the location and its first derivative for v = Param.</summary>
    public bool D1(double param, double first, double last)
    {
        NCollection_Array1<Point2D> t1 = new(1, 1);
        NCollection_Array1<Vector2D> t2 = new(1, 1);
        Matrix3x3 m = new Matrix3x3(), dm = new Matrix3x3();
        bool b;
        b = _law.D1(param, ref m, ref _v[1], ref dm, ref _dv[1], t1, t2);
        _v[2] = _v[2].WithXYZ(m.Column(1));
        _v[3] = _v[3].WithXYZ(m.Column(2));
        _v[4] = _v[4].WithXYZ(m.Column(3));
        _dv[2] = _dv[2].WithXYZ(dm.Column(1));
        _dv[3] = _dv[3].WithXYZ(dm.Column(2));
        _dv[4] = _dv[4].WithXYZ(dm.Column(3));
        return b;
    }

    /// <summary>Computes the location and its first and second derivatives for v = Param.</summary>
    public bool D2(double param, double first, double last)
    {
        NCollection_Array1<Point2D> t1 = new(1, 1);
        NCollection_Array1<Vector2D> t2 = new(1, 1), t3 = new(1, 1);
        Matrix3x3 m = new Matrix3x3(), dm = new Matrix3x3(), d2M = new Matrix3x3();
        bool b;
        b = _law.D2(param, ref m, ref _v[1], ref dm, ref _dv[1], ref d2M, ref _d2V[1], t1, t2, t3);
        _v[2] = _v[2].WithXYZ(m.Column(1));
        _v[3] = _v[3].WithXYZ(m.Column(2));
        _v[4] = _v[4].WithXYZ(m.Column(3));
        _dv[2] = _dv[2].WithXYZ(dm.Column(1));
        _dv[3] = _dv[3].WithXYZ(dm.Column(2));
        _dv[4] = _dv[4].WithXYZ(dm.Column(3));
        _d2V[2] = _d2V[2].WithXYZ(d2M.Column(1));
        _d2V[3] = _d2V[3].WithXYZ(d2M.Column(2));
        _d2V[4] = _d2V[4].WithXYZ(d2M.Column(3));
        return b;
    }

    /// <summary>
    /// The derivative of order Order (0 to 2) as 12 reals into Result (OCCT's <c>double&amp;</c> start of an
    /// array); Ier is Order + 1 when the law fails.
    /// </summary>
    public void DN(double param, double first, double last, int order, Span<double> result, ref int ier)
    {
        bool b;
        NCollection_Array1<Vector3D>? localResult = null;
        ier = 0;

        switch (order)
        {
            case 0:
                b = D0(param, first, last);
                localResult = _v;
                break;
            case 1:
                b = D1(param, first, last);
                localResult = _dv;
                break;
            case 2:
                b = D2(param, first, last);
                localResult = _d2V;
                break;
            default:
                b = false;
                break;
        }
        if (!b)
        {
            ier = order + 1;
        }

        // OCCT: another order leaves LocalResult null and reads through it; the port throws.
        if (localResult is null)
        {
            throw new OutOfRangeException("GeomFill_LocFunction::DN");
        }
        for (int ii = 0; ii <= 11; ii++)
        {
            Vector3D v = localResult[1 + ii / 3];
            result[ii] = v.Coord(1 + ii % 3);
        }
    }
}
