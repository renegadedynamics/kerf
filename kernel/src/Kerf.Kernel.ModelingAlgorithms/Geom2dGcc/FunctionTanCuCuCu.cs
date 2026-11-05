// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuCuCu.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuCuCu.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Constraints.Type1;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// The function whose roots are the parameters (u1, u2, u3) of the tangency points of a circle tangent to three
/// curves (circles and lines among them).
/// </summary>
/// <remarks>
/// OCCT copies the adaptors; the port shares the solver's, which nothing writes (decision 797). OCCT declares three
/// more constructors (a curve and points) without defining them; they are left out.
/// </remarks>
internal sealed class FunctionTanCuCuCu : FunctionSetWithDerivatives
{
    private readonly CurveAdaptor2D _curv1 = new();
    private readonly CurveAdaptor2D _curv2 = new();
    private readonly CurveAdaptor2D _curv3 = new();
    private readonly Circle2D _circ1 = new();
    private readonly Circle2D _circ2 = new();
    private readonly Line2D _lin1 = new();
    private readonly Line2D _lin2 = new();
    private readonly Type1 _theType;

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
            case Type1.CiCuCu:
            {
                global::Kerf.Engine.Geometry.ElCLib.D2(x[1], _circ1, out point1, out tan1, out d21);
                ConstraintCurveTool.D2(_curv2, x[2], out point2, out tan2, out d22);
                ConstraintCurveTool.D2(_curv3, x[3], out point3, out tan3, out d23);
            }
            break;
            case Type1.CiCiCu:
            {
                global::Kerf.Engine.Geometry.ElCLib.D2(x[1], _circ1, out point1, out tan1, out d21);
                global::Kerf.Engine.Geometry.ElCLib.D2(x[2], _circ2, out point2, out tan2, out d22);
                ConstraintCurveTool.D2(_curv3, x[3], out point3, out tan3, out d23);
            }
            break;
            case Type1.CiLiCu:
            {
                global::Kerf.Engine.Geometry.ElCLib.D2(x[1], _circ1, out point1, out tan1, out d21);
                global::Kerf.Engine.Geometry.ElCLib.D1(x[2], _lin2, out point2, out tan2);
                d22 = new Vector2D(0.0, 0.0);
                ConstraintCurveTool.D2(_curv3, x[3], out point3, out tan3, out d23);
            }
            break;
            case Type1.LiCuCu:
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(x[1], _lin1, out point1, out tan1);
                d21 = new Vector2D(0.0, 0.0);
                ConstraintCurveTool.D2(_curv2, x[2], out point2, out tan2, out d22);
                ConstraintCurveTool.D2(_curv3, x[3], out point3, out tan3, out d23);
            }
            break;
            case Type1.LiLiCu:
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(x[1], _lin1, out point1, out tan1);
                d21 = new Vector2D(0.0, 0.0);
                global::Kerf.Engine.Geometry.ElCLib.D1(x[2], _lin2, out point2, out tan2);
                d22 = new Vector2D(0.0, 0.0);
                ConstraintCurveTool.D2(_curv3, x[3], out point3, out tan3, out d23);
            }
            break;
            case Type1.CuCuCu:
            {
                ConstraintCurveTool.D2(_curv1, x[1], out point1, out tan1, out d21);
                ConstraintCurveTool.D2(_curv2, x[2], out point2, out tan2, out d22);
                ConstraintCurveTool.D2(_curv3, x[3], out point3, out tan3, out d23);
            }
            break;
            default:
            {
                throw new ConstructionErrorException();
            }
        }
    }

    public FunctionTanCuCuCu(CurveAdaptor2D c1, CurveAdaptor2D c2, CurveAdaptor2D c3)
    {
        _curv1   = c1;
        _curv2   = c2;
        _curv3   = c3;
        _theType = Type1.CuCuCu;
    }

    public FunctionTanCuCuCu(in Circle2D c1, CurveAdaptor2D c2, CurveAdaptor2D c3)
    {
        _circ1   = c1;
        _curv2   = c2;
        _curv3   = c3;
        _theType = Type1.CiCuCu;
    }

    public FunctionTanCuCuCu(in Circle2D c1, in Circle2D c2, CurveAdaptor2D c3)
    {
        _circ1   = c1;
        _circ2   = c2;
        _curv3   = c3;
        _theType = Type1.CiCiCu;
    }

    public FunctionTanCuCuCu(in Circle2D c1, in Line2D l2, CurveAdaptor2D c3)
    {
        _circ1   = c1;
        _lin2    = l2;
        _curv3   = c3;
        _theType = Type1.CiLiCu;
    }

    public FunctionTanCuCuCu(in Line2D l1, in Line2D l2, CurveAdaptor2D c3)
    {
        _lin1    = l1;
        _lin2    = l2;
        _curv3   = c3;
        _theType = Type1.LiLiCu;
    }

    public FunctionTanCuCuCu(in Line2D l1, CurveAdaptor2D c2, CurveAdaptor2D c3)
    {
        _lin1    = l1;
        _curv2   = c2;
        _curv3   = c3;
        _theType = Type1.LiCuCu;
    }

    //==========================================================================

    public override int NbVariables()
    {
        return 3;
    }

    public override int NbEquations()
    {
        return 3;
    }

    public override bool Value(math_Vector x, math_Vector fval)
    {
        InitDerivative(x, out Point2D point1, out Point2D point2, out Point2D point3, out Vector2D tan1,
                       out Vector2D tan2, out Vector2D tan3, out _, out _, out _);
        // pipj (normes) et PiPj (non Normes).
        Xy  p1P2    = new Vector2D(point1, point2).XY();
        Xy  p2P3    = new Vector2D(point2, point3).XY();
        Xy  p3P1    = new Vector2D(point3, point1).XY();
        double norP1P2 = p1P2.Modulus();
        double norP2P3 = p2P3.Modulus();
        double norP3P1 = p3P1.Modulus();
        Xy  p1p2, p2p3, p3p1;
        if (norP1P2 >= Constants.Resolution())
        {
            p1p2 = p1P2 / norP1P2;
        }
        else
        {
            p1p2 = new Xy(0.0, 0.0);
        }
        if (norP2P3 >= Constants.Resolution())
        {
            p2p3 = p2P3 / norP2P3;
        }
        else
        {
            p2p3 = new Xy(0.0, 0.0);
        }
        if (norP3P1 >= Constants.Resolution())
        {
            p3p1 = p3P1 / norP3P1;
        }
        else
        {
            p3p1 = new Xy(0.0, 0.0);
        }
        // derivees premieres non normees Deriv1ui.
        Xy deriv1u1 = tan1.XY();
        Xy deriv1u2 = tan2.XY();
        Xy deriv1u3 = tan3.XY();
        // normales aux courbes.
        double nnor1 = deriv1u1.Modulus();
        double nnor2 = deriv1u2.Modulus();
        double nnor3 = deriv1u3.Modulus();
        Xy  Nor1  = new(-deriv1u1.Y(), deriv1u1.X());
        Xy  Nor2  = new(-deriv1u2.Y(), deriv1u2.X());
        Xy  Nor3  = new(-deriv1u3.Y(), deriv1u3.X());
        Xy  nor1, nor2, nor3;
        if (nnor1 >= Constants.Resolution())
        {
            nor1 = Nor1 / nnor1;
        }
        else
        {
            nor1 = new Xy(0.0, 0.0);
        }
        if (nnor2 >= Constants.Resolution())
        {
            nor2 = Nor2 / nnor2;
        }
        else
        {
            nor2 = new Xy(0.0, 0.0);
        }
        if (nnor3 >= Constants.Resolution())
        {
            nor3 = Nor3 / nnor3;
        }
        else
        {
            nor3 = new Xy(0.0, 0.0);
        }
        // determination des signes pour les produits scalaires.
        double   signe1  = 1.0;
        double   signe2  = 1.0;
        double   signe3  = 1.0;
        Point2D pcenter = new((point1.XY() + point2.XY() + point3.XY()) / 3.0);
        Xy    fic1    = pcenter.XY() - point1.XY();
        Xy    fic2    = pcenter.XY() - point2.XY();
        Xy    fic3    = pcenter.XY() - point3.XY();
        double   pscal11 = nor1.Dot(fic1);
        double   pscal22 = nor2.Dot(fic2);
        double   pscal33 = nor3.Dot(fic3);
        if (pscal11 <= 0.0)
        {
            signe1 = -1;
        }
        if (pscal22 <= 0.0)
        {
            signe2 = -1;
        }
        if (pscal33 <= 0.0)
        {
            signe3 = -1;
        }
        // Fonctions Fui.
        // ==============
        fval[1] = signe1 * nor1.Dot(p1p2) + signe2 * nor2.Dot(p1p2);
        fval[2] = signe2 * nor2.Dot(p2p3) + signe3 * nor3.Dot(p2p3);
        fval[3] = signe3 * nor3.Dot(p3p1) + signe1 * nor1.Dot(p3p1);
        return true;
    }

    // OCCT: computes nothing and leaves the derivatives as they are (occt-issues.md).
    public override bool Derivatives(math_Vector x, Matrix d)
    {
        return true;
    }

    // OCCT: computes nothing and leaves the values and derivatives as they are. Circ2d3TanIter solves with
    // math_FunctionSetRoot, which calls Values: its search starts from whatever F and DF hold, uninitialised natively
    // and zeros in the port, which therefore stops at once at the starting point (occt-issues.md, decision 802).
    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        return true;
    }
}
