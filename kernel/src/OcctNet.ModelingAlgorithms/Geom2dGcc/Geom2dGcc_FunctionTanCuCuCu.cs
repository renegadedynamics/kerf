// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuCuCu.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_FunctionTanCuCuCu.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.Geom2dGcc_Type1;

namespace Occt;

/// <summary>
/// The function whose roots are the parameters (u1, u2, u3) of the tangency points of a circle tangent to three
/// curves (circles and lines among them).
/// </summary>
/// <remarks>
/// OCCT copies the adaptors; the port shares the solver's, which nothing writes (decision 797). OCCT declares three
/// more constructors (a curve and points) without defining them; they are left out.
/// </remarks>
public sealed class Geom2dGcc_FunctionTanCuCuCu : FunctionSetWithDerivatives
{
    private readonly CurveAdaptor2D Curv1 = new();
    private readonly CurveAdaptor2D Curv2 = new();
    private readonly CurveAdaptor2D Curv3 = new();
    private readonly Circle2D Circ1 = new();
    private readonly Circle2D Circ2 = new();
    private readonly Line2D Lin1 = new();
    private readonly Line2D Lin2 = new();
    private readonly Geom2dGcc_Type1 TheType;

    public void InitDerivative(math_Vector X,
                               out Point2D Point1,
                               out Point2D Point2,
                               out Point2D Point3,
                               out Vector2D Tan1,
                               out Vector2D Tan2,
                               out Vector2D Tan3,
                               out Vector2D D21,
                               out Vector2D D22,
                               out Vector2D D23)
    {
        switch (TheType)
        {
            case Geom2dGcc_CiCuCu:
            {
                global::Kerf.Engine.Geometry.ElCLib.D2(X[1], Circ1, out Point1, out Tan1, out D21);
                Geom2dGcc_CurveTool.D2(Curv2, X[2], out Point2, out Tan2, out D22);
                Geom2dGcc_CurveTool.D2(Curv3, X[3], out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_CiCiCu:
            {
                global::Kerf.Engine.Geometry.ElCLib.D2(X[1], Circ1, out Point1, out Tan1, out D21);
                global::Kerf.Engine.Geometry.ElCLib.D2(X[2], Circ2, out Point2, out Tan2, out D22);
                Geom2dGcc_CurveTool.D2(Curv3, X[3], out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_CiLiCu:
            {
                global::Kerf.Engine.Geometry.ElCLib.D2(X[1], Circ1, out Point1, out Tan1, out D21);
                global::Kerf.Engine.Geometry.ElCLib.D1(X[2], Lin2, out Point2, out Tan2);
                D22 = new Vector2D(0.0, 0.0);
                Geom2dGcc_CurveTool.D2(Curv3, X[3], out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_LiCuCu:
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(X[1], Lin1, out Point1, out Tan1);
                D21 = new Vector2D(0.0, 0.0);
                Geom2dGcc_CurveTool.D2(Curv2, X[2], out Point2, out Tan2, out D22);
                Geom2dGcc_CurveTool.D2(Curv3, X[3], out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_LiLiCu:
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(X[1], Lin1, out Point1, out Tan1);
                D21 = new Vector2D(0.0, 0.0);
                global::Kerf.Engine.Geometry.ElCLib.D1(X[2], Lin2, out Point2, out Tan2);
                D22 = new Vector2D(0.0, 0.0);
                Geom2dGcc_CurveTool.D2(Curv3, X[3], out Point3, out Tan3, out D23);
            }
            break;
            case Geom2dGcc_CuCuCu:
            {
                Geom2dGcc_CurveTool.D2(Curv1, X[1], out Point1, out Tan1, out D21);
                Geom2dGcc_CurveTool.D2(Curv2, X[2], out Point2, out Tan2, out D22);
                Geom2dGcc_CurveTool.D2(Curv3, X[3], out Point3, out Tan3, out D23);
            }
            break;
            default:
            {
                throw new ConstructionErrorException();
            }
        }
    }

    public Geom2dGcc_FunctionTanCuCuCu(CurveAdaptor2D C1, CurveAdaptor2D C2, CurveAdaptor2D C3)
    {
        Curv1   = C1;
        Curv2   = C2;
        Curv3   = C3;
        TheType = Geom2dGcc_CuCuCu;
    }

    public Geom2dGcc_FunctionTanCuCuCu(in Circle2D C1, CurveAdaptor2D C2, CurveAdaptor2D C3)
    {
        Circ1   = C1;
        Curv2   = C2;
        Curv3   = C3;
        TheType = Geom2dGcc_CiCuCu;
    }

    public Geom2dGcc_FunctionTanCuCuCu(in Circle2D C1, in Circle2D C2, CurveAdaptor2D C3)
    {
        Circ1   = C1;
        Circ2   = C2;
        Curv3   = C3;
        TheType = Geom2dGcc_CiCiCu;
    }

    public Geom2dGcc_FunctionTanCuCuCu(in Circle2D C1, in Line2D L2, CurveAdaptor2D C3)
    {
        Circ1   = C1;
        Lin2    = L2;
        Curv3   = C3;
        TheType = Geom2dGcc_CiLiCu;
    }

    public Geom2dGcc_FunctionTanCuCuCu(in Line2D L1, in Line2D L2, CurveAdaptor2D C3)
    {
        Lin1    = L1;
        Lin2    = L2;
        Curv3   = C3;
        TheType = Geom2dGcc_LiLiCu;
    }

    public Geom2dGcc_FunctionTanCuCuCu(in Line2D L1, CurveAdaptor2D C2, CurveAdaptor2D C3)
    {
        Lin1    = L1;
        Curv2   = C2;
        Curv3   = C3;
        TheType = Geom2dGcc_LiCuCu;
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

    public override bool Value(math_Vector X, math_Vector Fval)
    {
        InitDerivative(X, out Point2D Point1, out Point2D Point2, out Point2D Point3, out Vector2D Tan1,
                       out Vector2D Tan2, out Vector2D Tan3, out _, out _, out _);
        // pipj (normes) et PiPj (non Normes).
        Xy  P1P2    = new Vector2D(Point1, Point2).XY();
        Xy  P2P3    = new Vector2D(Point2, Point3).XY();
        Xy  P3P1    = new Vector2D(Point3, Point1).XY();
        double NorP1P2 = P1P2.Modulus();
        double NorP2P3 = P2P3.Modulus();
        double NorP3P1 = P3P1.Modulus();
        Xy  p1p2, p2p3, p3p1;
        if (NorP1P2 >= Constants.Resolution())
        {
            p1p2 = P1P2 / NorP1P2;
        }
        else
        {
            p1p2 = new Xy(0.0, 0.0);
        }
        if (NorP2P3 >= Constants.Resolution())
        {
            p2p3 = P2P3 / NorP2P3;
        }
        else
        {
            p2p3 = new Xy(0.0, 0.0);
        }
        if (NorP3P1 >= Constants.Resolution())
        {
            p3p1 = P3P1 / NorP3P1;
        }
        else
        {
            p3p1 = new Xy(0.0, 0.0);
        }
        // derivees premieres non normees Deriv1ui.
        Xy Deriv1u1 = Tan1.XY();
        Xy Deriv1u2 = Tan2.XY();
        Xy Deriv1u3 = Tan3.XY();
        // normales aux courbes.
        double nnor1 = Deriv1u1.Modulus();
        double nnor2 = Deriv1u2.Modulus();
        double nnor3 = Deriv1u3.Modulus();
        Xy  Nor1  = new(-Deriv1u1.Y(), Deriv1u1.X());
        Xy  Nor2  = new(-Deriv1u2.Y(), Deriv1u2.X());
        Xy  Nor3  = new(-Deriv1u3.Y(), Deriv1u3.X());
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
        Point2D Pcenter = new((Point1.XY() + Point2.XY() + Point3.XY()) / 3.0);
        Xy    fic1    = Pcenter.XY() - Point1.XY();
        Xy    fic2    = Pcenter.XY() - Point2.XY();
        Xy    fic3    = Pcenter.XY() - Point3.XY();
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
        Fval[1] = signe1 * nor1.Dot(p1p2) + signe2 * nor2.Dot(p1p2);
        Fval[2] = signe2 * nor2.Dot(p2p3) + signe3 * nor3.Dot(p2p3);
        Fval[3] = signe3 * nor3.Dot(p3p1) + signe1 * nor1.Dot(p3p1);
        return true;
    }

    // OCCT: computes nothing and leaves the derivatives as they are (occt-issues.md).
    public override bool Derivatives(math_Vector X, Matrix D)
    {
        return true;
    }

    // OCCT: computes nothing and leaves the values and derivatives as they are. Circ2d3TanIter solves with
    // math_FunctionSetRoot, which calls Values: its search starts from whatever F and DF hold, uninitialised natively
    // and zeros in the port, which therefore stops at once at the starting point (occt-issues.md, decision 802).
    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        return true;
    }
}
