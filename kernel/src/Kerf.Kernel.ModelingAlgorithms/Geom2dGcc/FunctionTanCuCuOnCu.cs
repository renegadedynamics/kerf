// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuCuOnCu.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuCuOnCu.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Type2;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// The function whose roots are the parameters (u1, u2, u3) and the radius of a circle tangent to two curves (or a
/// circle, a line or a point and a curve) whose centre lies on a third curve (a line, a circle or a curve).
/// </summary>
/// <remarks>OCCT copies the adaptors; the port shares the solver's, which nothing writes (decision 797).</remarks>
public sealed class FunctionTanCuCuOnCu : FunctionSetWithDerivatives
{
    private readonly CurveAdaptor2D _curv1 = new();
    private readonly CurveAdaptor2D _curv2 = new();
    private readonly Circle2D _circ1 = new();
    private readonly Line2D _lin1 = new();
    private readonly Point2D _pnt2 = new();
    private readonly Circle2D _circon = new();
    private readonly Line2D _linon = new();
    private readonly CurveAdaptor2D _curvon = new();
    private readonly double _firstRad;
    private readonly Type2 _theType;

    public void InitDerivative(math_Vector x,
                               out Point2D point1,
                               out Point2D point2,
                               out Point2D point3,
                               out Vector2D tan1,
                               out Vector2D tan2,
                               out Vector2D tan3,
                               out Vector2D d21,
                               out Vector2D d22,
                               out Vector2D d23)
    {
        switch (_theType)
        {
            case Type2.CuCuOnCu:
            {
                ConstraintCurveTool.D2(_curv1, x[1], out point1, out tan1, out d21);
                ConstraintCurveTool.D2(_curv2, x[2], out point2, out tan2, out d22);
                ConstraintCurveTool.D2(_curvon, x[3], out point3, out tan3, out d23);
            }
            break;
            case Type2.CiCuOnCu:
            {
                global::Kerf.Engine.Geometry.ElCLib.D2(x[1], _circ1, out point1, out tan1, out d21);
                ConstraintCurveTool.D2(_curv2, x[2], out point2, out tan2, out d22);
                ConstraintCurveTool.D2(_curvon, x[3], out point3, out tan3, out d23);
            }
            break;
            case Type2.LiCuOnCu:
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(x[1], _lin1, out point1, out tan1);
                d21 = new Vector2D(0.0, 0.0);
                ConstraintCurveTool.D2(_curv2, x[2], out point2, out tan2, out d22);
                ConstraintCurveTool.D2(_curvon, x[3], out point3, out tan3, out d23);
            }
            break;
            case Type2.CuPtOnCu:
            {
                ConstraintCurveTool.D2(_curv1, x[1], out point1, out tan1, out d21);
                ConstraintCurveTool.D2(_curvon, x[3], out point3, out tan3, out d23);
                point2 = _pnt2;
                tan2   = new Vector2D(0.0, 0.0);
                d22    = new Vector2D(0.0, 0.0);
            }
            break;
            case Type2.CuCuOnCi:
            {
                ConstraintCurveTool.D2(_curv1, x[1], out point1, out tan1, out d21);
                ConstraintCurveTool.D2(_curv2, x[2], out point2, out tan2, out d22);
                global::Kerf.Engine.Geometry.ElCLib.D2(x[3], _circon, out point3, out tan3, out d23);
            }
            break;
            case Type2.CiCuOnCi:
            {
                global::Kerf.Engine.Geometry.ElCLib.D2(x[1], _circ1, out point1, out tan1, out d21);
                ConstraintCurveTool.D2(_curv2, x[2], out point2, out tan2, out d22);
                global::Kerf.Engine.Geometry.ElCLib.D2(x[3], _circon, out point3, out tan3, out d23);
            }
            break;
            case Type2.LiCuOnCi:
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(x[1], _lin1, out point1, out tan1);
                d21 = new Vector2D(0.0, 0.0);
                ConstraintCurveTool.D2(_curv2, x[2], out point2, out tan2, out d22);
                global::Kerf.Engine.Geometry.ElCLib.D2(x[3], _circon, out point3, out tan3, out d23);
            }
            break;
            case Type2.CuPtOnCi:
            {
                ConstraintCurveTool.D2(_curv1, x[1], out point1, out tan1, out d21);
                point2 = _pnt2;
                tan2   = new Vector2D(0.0, 0.0);
                d22    = new Vector2D(0.0, 0.0);
                global::Kerf.Engine.Geometry.ElCLib.D2(x[3], _circon, out point3, out tan3, out d23);
            }
            break;
            case Type2.CuCuOnLi:
            {
                ConstraintCurveTool.D2(_curv1, x[1], out point1, out tan1, out d21);
                ConstraintCurveTool.D2(_curv2, x[2], out point2, out tan2, out d22);
                global::Kerf.Engine.Geometry.ElCLib.D1(x[3], _linon, out point3, out tan3);
                d23 = new Vector2D(0.0, 0.0);
            }
            break;
            case Type2.CiCuOnLi:
            {
                global::Kerf.Engine.Geometry.ElCLib.D2(x[1], _circ1, out point1, out tan1, out d21);
                ConstraintCurveTool.D2(_curv2, x[2], out point2, out tan2, out d22);
                global::Kerf.Engine.Geometry.ElCLib.D1(x[3], _linon, out point3, out tan3);
                d23 = new Vector2D(0.0, 0.0);
            }
            break;
            case Type2.LiCuOnLi:
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(x[1], _lin1, out point1, out tan1);
                ConstraintCurveTool.D2(_curv2, x[2], out point2, out tan2, out d22);
                d21 = new Vector2D(0.0, 0.0);
                global::Kerf.Engine.Geometry.ElCLib.D1(x[3], _linon, out point3, out tan3);
                d23 = new Vector2D(0.0, 0.0);
            }
            break;
            case Type2.CuPtOnLi:
            {
                ConstraintCurveTool.D2(_curv1, x[1], out point1, out tan1, out d21);
                point2 = _pnt2;
                tan2   = new Vector2D(0.0, 0.0);
                d22    = new Vector2D(0.0, 0.0);
                global::Kerf.Engine.Geometry.ElCLib.D1(x[3], _linon, out point3, out tan3);
                d23 = new Vector2D(0.0, 0.0);
            }
            break;
            default:
            {
                throw new ConstructionErrorException();
            }
        }
    }

    public FunctionTanCuCuOnCu(CurveAdaptor2D c1, CurveAdaptor2D c2, CurveAdaptor2D c3, double rad)
    {
        _curv1 = c1;
        _curv2 = c2;
        _curvon = c3;
        _firstRad = rad;
        _theType = Type2.CuCuOnCu;
    }

    public FunctionTanCuCuOnCu(in Circle2D c1, CurveAdaptor2D c2, CurveAdaptor2D c3, double rad)
    {
        _circ1 = c1;
        _curv2 = c2;
        _curvon = c3;
        _firstRad = rad;
        _theType = Type2.CiCuOnCu;
    }

    public FunctionTanCuCuOnCu(in Line2D l1, CurveAdaptor2D c2, CurveAdaptor2D c3, double rad)
    {
        _lin1 = l1;
        _curv2 = c2;
        _curvon = c3;
        _firstRad = rad;
        _theType = Type2.LiCuOnCu;
    }

    public FunctionTanCuCuOnCu(CurveAdaptor2D c1, in Point2D p2, CurveAdaptor2D c3, double rad)
    {
        _curv1 = c1;
        _pnt2 = p2;
        _curvon = c3;
        _firstRad = rad;
        _theType = Type2.CuPtOnCu;
    }

    public FunctionTanCuCuOnCu(CurveAdaptor2D c1, CurveAdaptor2D c2, in Line2D onLi, double rad)
    {
        _curv1 = c1;
        _curv2 = c2;
        _linon = onLi;
        _firstRad = rad;
        _theType = Type2.CuCuOnLi;
    }

    public FunctionTanCuCuOnCu(in Circle2D c1, CurveAdaptor2D c2, in Line2D onLi, double rad)
    {
        _circ1 = c1;
        _curv2 = c2;
        _linon = onLi;
        _firstRad = rad;
        _theType = Type2.CiCuOnLi;
    }

    public FunctionTanCuCuOnCu(in Line2D l1, CurveAdaptor2D c2, in Line2D onLi, double rad)
    {
        _lin1 = l1;
        _curv2 = c2;
        _linon = onLi;
        _firstRad = rad;
        _theType = Type2.LiCuOnLi;
    }

    public FunctionTanCuCuOnCu(CurveAdaptor2D c1, in Point2D p2, in Line2D onLi, double rad)
    {
        _curv1 = c1;
        _pnt2 = p2;
        _linon = onLi;
        _firstRad = rad;
        _theType = Type2.CuPtOnLi;
    }

    public FunctionTanCuCuOnCu(CurveAdaptor2D c1, CurveAdaptor2D c2, in Circle2D onCi, double rad)
    {
        _curv1 = c1;
        _curv2 = c2;
        _circon = onCi;
        _firstRad = rad;
        _theType = Type2.CuCuOnCi;
    }

    public FunctionTanCuCuOnCu(in Circle2D c1, CurveAdaptor2D c2, in Circle2D onCi, double rad)
    {
        _circ1 = c1;
        _curv2 = c2;
        _circon = onCi;
        _firstRad = rad;
        _theType = Type2.CuCuOnCi; // OCCT: should be Geom2dGcc_CiCuOnCi; InitDerivative then evaluates the unset Curv1
    }

    public FunctionTanCuCuOnCu(in Line2D l1, CurveAdaptor2D c2, in Circle2D onCi, double rad)
    {
        _lin1 = l1;
        _curv2 = c2;
        _circon = onCi;
        _firstRad = rad;
        _theType = Type2.LiCuOnCi;
    }

    public FunctionTanCuCuOnCu(CurveAdaptor2D c1, in Point2D p2, in Circle2D onCi, double rad)
    {
        _curv1 = c1;
        _pnt2 = p2;
        _circon = onCi;
        _firstRad = rad;
        _theType = Type2.CuPtOnCi;
    }

    /// <summary>
    /// True for a point argument: the unknowns are the parameter on the curve, the centre's parameter and the radius,
    /// and the equations the two distances and the tangency with the curve (OCCT kept four unknowns, whose fourth
    /// equation divides by the point's null tangent, and its Iter gave it 3-vectors: Kerf decision 17106).
    /// </summary>
    private bool IsPoint => _theType is Type2.CuPtOnCu or Type2.CuPtOnCi or Type2.CuPtOnLi;

    public override int NbVariables()
    {
        return IsPoint ? 3 : 4;
    }

    public override int NbEquations()
    {
        return IsPoint ? 3 : 4;
    }

    /// <summary>The point case's equations and derivatives, x = (u1, uOn, R).</summary>
    private void PointValues(math_Vector x, math_Vector? fval, Matrix? deriv)
    {
        math_Vector y = new(1, 4);
        y[1] = x[1];
        y[3] = x[2];
        y[4] = x[3];
        InitDerivative(y, out Point2D point1, out Point2D point2, out Point2D point3, out Vector2D tan1, out _,
                       out Vector2D tan3, out Vector2D d21, out _, out _);
        Vector2D p2P3 = new(point2, point3);
        Vector2D p3P1 = new(point3, point1);
        double nnor1 = tan1.Magnitude;
        double r = x[3];
        double fr2 = _firstRad * _firstRad;
        if (fval is not null)
        {
            fval[1] = (p3P1.Dot(p3P1) - r * r) / fr2;
            fval[2] = (p2P3.Dot(p2P3) - r * r) / fr2;
            fval[3] = p3P1.Dot(tan1) / (nnor1 * _firstRad);
        }
        if (deriv is not null)
        {
            deriv[1, 1] = 2.0 * tan1.Dot(p3P1) / fr2;
            deriv[1, 2] = -2.0 * tan3.Dot(p3P1) / fr2;
            deriv[1, 3] = -2.0 * r / fr2;
            deriv[2, 1] = 0.0;
            deriv[2, 2] = 2.0 * tan3.Dot(p2P3) / fr2;
            deriv[2, 3] = -2.0 * r / fr2;
            deriv[3, 1] = (p3P1.Dot(d21) + tan1.Dot(tan1)) / (_firstRad * nnor1)
                          - (p3P1.Dot(tan1) * d21.Dot(tan1)) / (_firstRad * nnor1 * nnor1 * nnor1);
            deriv[3, 2] = -(tan3.Dot(tan1)) / (_firstRad * nnor1);
            deriv[3, 3] = 0.0;
        }
    }

    public override bool Value(math_Vector x, math_Vector fval)
    {
        if (IsPoint)
        {
            PointValues(x, fval, null);
            return true;
        }
        InitDerivative(x, out Point2D point1, out Point2D point2, out Point2D point3, out Vector2D tan1, out Vector2D tan2,
                       out Vector2D tan3, out Vector2D d21, out Vector2D d22, out Vector2D d23);
        // pipj (normes) et PiPj (non Normes).
        Vector2D p1P2 = new(point1, point2);
        Vector2D p2P3 = new(point2, point3);
        Vector2D p3P1 = new(point3, point1);
        Vector2D p1p2, p2p3, p3p1;
        //  if (FirstRad < 1.0) {FirstRad = 1.0; }
        p1p2 = p1P2 / _firstRad;
        p2p3 = p2P3 / _firstRad;
        p3p1 = p3P1 / _firstRad;
        // norme des Tani.
        double nnor1 = tan1.Magnitude;
        double nnor2 = tan2.Magnitude;
        // Fonctions Fui.
        // ==============
        fval[1] = (p3P1.Dot(p3P1) - x[4] * x[4]) / (_firstRad * _firstRad);
        fval[2] = (p2P3.Dot(p2P3) - x[4] * x[4]) / (_firstRad * _firstRad);
        fval[3] = p3P1.Dot(tan1) / (nnor1 * _firstRad);
        fval[4] = p2P3.Dot(tan2) / (nnor2 * _firstRad);
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix deriv)
    {
        if (IsPoint)
        {
            PointValues(x, null, deriv);
            return true;
        }
        InitDerivative(x, out Point2D point1, out Point2D point2, out Point2D point3, out Vector2D tan1, out Vector2D tan2,
                       out Vector2D tan3, out Vector2D d21, out Vector2D d22, out Vector2D d23);
        // pipj (normes) et PiPj (non Normes).
        Vector2D p1P2 = new(point1, point2);
        Vector2D p2P3 = new(point2, point3);
        Vector2D p3P1 = new(point3, point1);
        Vector2D p1p2, p2p3, p3p1;
        //  if (FirstRad < 1.0) {FirstRad = 1.0; }
        p1p2 = p1P2 / _firstRad;
        p2p3 = p2P3 / _firstRad;
        p3p1 = p3P1 / _firstRad;
        // normales au courbes normees Nori et non nromees nori et norme des nori.
        double nnor1 = tan1.Magnitude;
        double nnor2 = tan2.Magnitude;
        // Derivees dFui/uj  1 <= ui <= 3 , 1 <= uj <= 3
        // =============================================
        deriv[1, 1] = 2.0 * tan1.Dot(p3P1) / (_firstRad * _firstRad);
        deriv[1, 2] = 0.0;
        deriv[1, 3] = -2.0 * tan3.Dot(p3P1) / (_firstRad * _firstRad);
        deriv[1, 4] = -2.0 * x[4] / (_firstRad * _firstRad);

        deriv[2, 1] = 0.0;
        deriv[2, 2] = -2.0 * tan2.Dot(p2P3) / (_firstRad * _firstRad);
        deriv[2, 3] = 2.0 * tan3.Dot(p2P3) / (_firstRad * _firstRad);
        deriv[2, 4] = -2.0 * x[4] / (_firstRad * _firstRad);

        deriv[3, 1] = (p3P1.Dot(d21) + tan1.Dot(tan1)) / (_firstRad * nnor1)
                      - (p3P1.Dot(tan1) * d21.Dot(tan1)) / (_firstRad * nnor1 * nnor1 * nnor1);
        deriv[3, 2] = 0.0;
        deriv[3, 3] = -(tan3.Dot(tan1)) / (_firstRad * nnor1);
        deriv[3, 4] = 0.0;

        deriv[4, 1] = 0.0;
        deriv[4, 2] = (p2P3.Dot(d22) - tan2.Dot(tan2)) / (_firstRad * nnor2)
                      - p2P3.Dot(tan2) * tan2.Dot(d22) / (_firstRad * nnor2 * nnor2 * nnor2);
        deriv[4, 3] = tan3.Dot(tan2) / (_firstRad * nnor1);
        deriv[4, 4] = 0.0;
        return true;
    }

    public override bool Values(math_Vector x, math_Vector fval, Matrix deriv)
    {
        if (IsPoint)
        {
            PointValues(x, fval, deriv);
            return true;
        }
        InitDerivative(x, out Point2D point1, out Point2D point2, out Point2D point3, out Vector2D tan1, out Vector2D tan2,
                       out Vector2D tan3, out Vector2D d21, out Vector2D d22, out Vector2D d23);
        // pipj (normes) et PiPj (non Normes).
        Vector2D p1P2 = new(point1, point2);
        Vector2D p2P3 = new(point2, point3);
        Vector2D p3P1 = new(point3, point1);
        Vector2D p1p2, p2p3, p3p1;
        //  if (FirstRad < 1.0) {FirstRad = 1.0; }
        p1p2 = p1P2 / _firstRad;
        p2p3 = p2P3 / _firstRad;
        p3p1 = p3P1 / _firstRad;
        // normales au courbes normees Nori et non nromees nori et norme des nori.
        double nnor1 = tan1.Magnitude;
        double nnor2 = tan2.Magnitude;
        // Fonctions Fui.
        // ==============
        fval[1] = (p3P1.Dot(p3P1) - x[4] * x[4]) / (_firstRad * _firstRad);
        fval[2] = (p2P3.Dot(p2P3) - x[4] * x[4]) / (_firstRad * _firstRad);
        fval[3] = p3P1.Dot(tan1) / (nnor1 * _firstRad);
        fval[4] = p2P3.Dot(tan2) / (nnor2 * _firstRad);
        // Derivees dFui/uj  1 <= ui <= 3 , 1 <= uj <= 3
        // =============================================
        deriv[1, 1] = 2.0 * tan1.Dot(p3P1) / (_firstRad * _firstRad);
        deriv[1, 2] = 0.0;
        deriv[1, 3] = -2.0 * tan3.Dot(p3P1) / (_firstRad * _firstRad);
        deriv[1, 4] = -2.0 * x[4] / (_firstRad * _firstRad);

        deriv[2, 1] = 0.0;
        deriv[2, 2] = -2.0 * tan2.Dot(p2P3) / (_firstRad * _firstRad);
        deriv[2, 3] = 2.0 * tan3.Dot(p2P3) / (_firstRad * _firstRad);
        deriv[2, 4] = -2.0 * x[4] / (_firstRad * _firstRad);

        deriv[3, 1] = (p3P1.Dot(d21) + tan1.Dot(tan1)) / (_firstRad * nnor1)
                      - (p3P1.Dot(tan1) * d21.Dot(tan1)) / (_firstRad * nnor1 * nnor1 * nnor1);
        deriv[3, 2] = 0.0;
        deriv[3, 3] = -(tan3.Dot(tan1)) / (_firstRad * nnor1);
        deriv[3, 4] = 0.0;

        deriv[4, 1] = 0.0;
        deriv[4, 2] = (p2P3.Dot(d22) - tan2.Dot(tan2)) / (_firstRad * nnor2)
                      - p2P3.Dot(tan2) * tan2.Dot(d22) / (_firstRad * nnor2 * nnor2 * nnor2);
        deriv[4, 3] = tan3.Dot(tan2) / (_firstRad * nnor1);
        deriv[4, 4] = 0.0;
        return true;
    }
}
