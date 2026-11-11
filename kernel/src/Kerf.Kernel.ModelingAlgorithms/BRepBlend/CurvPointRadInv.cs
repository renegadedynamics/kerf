// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_CurvPointRadInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_CurvPointRadInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Fillets.BlendFunctions;
using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The inverse of the restriction/restriction function at an extremity: the section plane of the first curve (the
/// guide) through a given point and a point of the second curve, and for a constant radius the ball through both. The
/// variables are the parameters on the two curves.
/// </summary>
internal class CurvPointRadInv : CurvPointFuncInv
{
    private readonly CurveAdaptorBase _curv1;
    private readonly CurveAdaptorBase _curv2;
    private Point3D _point;

    public CurvPointRadInv(CurveAdaptorBase c1, CurveAdaptorBase c2)
    {
        _curv1 = c1;
        _curv2 = c2;
    }

    /// <summary>Sets the "quadrant" <paramref name="choix"/> (unused by the equations, as in OCCT).</summary>
    public void Set(int choix)
    {
    }

    // Kerf (decision 2162): the rolling ball's equations, as RstRstConstRad's at a restriction's end, once a radius is
    // set (and, decision 31204, an evolving radius's law, as RstRstEvolRad's): the guide's plane through the two
    // points' midpoint, and the centre at the radius from both, its chord to the point square to that restriction's
    // tangent there (SetTangent) and its chord to the curve's point square to the curve.
    private bool _isBall;
    private double _ray;
    private LawsFunction? _law;
    private int _choix;
    private bool _pointOnFirst;
    private Vector3D _tangent;

    /// <summary>
    /// Sets the radius and the "quadrant" <paramref name="choix"/> of a constant-radius fillet, and whether the point
    /// is on the first restriction (the curve then the second's) or on the second, which places the ball's centre.
    /// </summary>
    public void Set(double radius, int choix, bool pointOnFirst)
    {
        _isBall = true;
        _ray = Math.Abs(radius);
        _law = null;
        _choix = choix;
        _pointOnFirst = pointOnFirst;
    }

    /// <summary>
    /// As <see cref="Set(double, int, bool)"/> for an evolving-radius fillet: the radius the law gives at the guide's
    /// parameter, the first variable.
    /// </summary>
    public void Set(LawsFunction law, int choix, bool pointOnFirst)
    {
        _isBall = true;
        _law = law;
        _choix = choix;
        _pointOnFirst = pointOnFirst;
    }

    /// <summary>The restriction's tangent at the point <see cref="Set(in Point3D)"/> gives, which the ball is square to.</summary>
    public void SetTangent(in Vector3D tangent)
    {
        _tangent = tangent.Magnitude > 0 ? tangent.Normalized() : tangent;
    }

    private void Ball(math_Vector x, math_Vector f, Matrix? d)
    {
        _curv1.D2(x[1], out Point3D ptcur1, out Vector3D d1cur1, out Vector3D d2cur1);
        double unsurnormd1cur1 = 1.0 / d1cur1.Magnitude;
        Vector3D nplan = unsurnormd1cur1 * d1cur1;
        double theD = -(nplan.XYZ().Dot(ptcur1.XYZ()));
        _curv2.D2(x[2], out Point3D ptcur2, out Vector3D d1cur2, out Vector3D d2cur2);
        double dray = 0.0;
        if (_law is not null)
        {
            _law.D1(x[1], out _ray, out dray);
        }
        double inv2 = ConstRad.InverseNorm(d1cur2);
        Vector3D t2 = inv2 * d1cur2;

        // The side OCCT's CenterCircleRst1Rst2 puts the centre on, from the first restriction's point to the second's.
        Vector3D side = (_pointOnFirst ? new Vector3D(_point, ptcur2) : new Vector3D(ptcur2, _point)).Crossed(nplan);
        if (_choix > 2)
        {
            side = side.Reversed();
        }
        RstRstConstRad.BallCentre(_point, _tangent, ptcur2, _ray, side, out Point3D center);
        Vector3D c2 = new(ptcur2, center);

        f[1] = (nplan.XYZ().Dot(_point.XYZ()) + nplan.XYZ().Dot(ptcur2.XYZ())) / 2 + theD;
        f[2] = c2.Dot(t2);
        if (d is null)
        {
            return;
        }

        Vector3D dnplan = Vector3D.LinearForm(-nplan.Dot(d2cur1), nplan, d2cur1).Multiplied(unsurnormd1cur1);
        double dtheD = -nplan.XYZ().Dot(d1cur1.XYZ()) - dnplan.XYZ().Dot(ptcur1.XYZ());
        Vector3D dc = RstRstConstRad.BallCentreD1(center, _point, _tangent, ptcur2, 0.0, 0.0, -c2.Dot(d1cur2));
        d[1, 1] = (dnplan.XYZ().Dot(_point.XYZ()) + dnplan.XYZ().Dot(ptcur2.XYZ())) / 2 + dtheD;
        d[1, 2] = nplan.Dot(d1cur2) / 2;
        // In the guide's parameter the centre moves only with the radius.
        d[2, 1] = dray == 0.0 ? 0.0
            : RstRstConstRad.BallCentreD1(center, _point, _tangent, ptcur2, 0.0, -_ray * dray, -_ray * dray).Dot(t2);
        d[2, 2] = (dc - d1cur2).Dot(t2) + c2.Dot(ConstRad.UnitNormalD1(t2, inv2, d2cur2));
    }

    public override int NbEquations()
    {
        return 2;
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        if (_isBall)
        {
            Ball(x, f, null);
            return true;
        }
        double theD;
        Point3D ptcur1, ptcur2;
        Vector3D d1cur1, d1cur2;
        Xyz nplan;
        _curv1.D1(x[1], out ptcur1, out d1cur1);
        nplan = d1cur1.Normalized().XYZ();
        theD = -(nplan.Dot(ptcur1.XYZ()));
        _curv2.D1(x[2], out ptcur2, out d1cur2);
        f[1] = nplan.Dot(_point.XYZ()) + theD;
        f[2] = nplan.Dot(ptcur2.XYZ()) + theD;
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        if (_isBall)
        {
            Ball(x, new math_Vector(1, 2), d);
            return true;
        }
        Point3D ptcur1, ptcur2;
        Vector3D d1cur1, d2cur1, d1cur2, nplan, dnplan = new();
        double dtheD, normd1cur1, unsurnormd1cur1;

        _curv1.D2(x[1], out ptcur1, out d1cur1, out d2cur1);

        normd1cur1 = d1cur1.Magnitude;
        unsurnormd1cur1 = 1.0 / normd1cur1;
        nplan = unsurnormd1cur1 * d1cur1;
        dnplan = Vector3D.LinearForm(-nplan.Dot(d2cur1), nplan, d2cur1);
        dnplan = dnplan.Multiplied(unsurnormd1cur1);
        dtheD = -nplan.XYZ().Dot(d1cur1.XYZ()) - dnplan.XYZ().Dot(ptcur1.XYZ());
        d[1, 1] = dnplan.XYZ().Dot(_point.XYZ()) + dtheD;
        d[1, 2] = 0.0;
        _curv2.D1(x[2], out ptcur2, out d1cur2);
        d[2, 1] = dnplan.XYZ().Dot(ptcur2.XYZ()) + dtheD;
        d[2, 2] = nplan.Dot(d1cur2);

        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        Value(x, f);
        Derivatives(x, d);

        return true;
    }

    public override void Set(in Point3D p)
    {
        _point = p;
    }

    public override void GetTolerance(math_Vector tolerance, double tol)
    {
        tolerance[1] = _curv1.Resolution(tol);
        tolerance[2] = _curv2.Resolution(tol);
    }

    public override void GetBounds(math_Vector infBound, math_Vector supBound)
    {
        infBound[1] = _curv1.FirstParameter();
        supBound[1] = _curv1.LastParameter();
        infBound[2] = _curv2.FirstParameter();
        supBound[2] = _curv2.LastParameter();
    }

    public override bool IsSolution(math_Vector sol, double tol)
    {
        math_Vector valsol = new(1, 2);
        Value(sol, valsol);
        return Math.Abs(valsol[1]) <= tol && Math.Abs(valsol[2]) <= tol;
    }
}
