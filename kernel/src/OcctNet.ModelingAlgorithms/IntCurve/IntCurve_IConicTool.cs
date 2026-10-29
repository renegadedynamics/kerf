// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IConicTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IConicTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// A conic of gp seen as an implicit curve: OCCT's implementation of IntImpParGen's ImpTool for conics.
/// </summary>
/// <remarks>
/// A struct, copied as OCCT copies it. A working tool (as the result structs of decision 54), so it has no
/// equality. <c>default(IntCurve_IConicTool)</c> has a zero transformation; use the constructors.
/// </remarks>
public struct IntCurve_IConicTool : IIntImpParGen_ImpTool
{
    // OCCT's macros naming the parameters by conic: Elips_a/b/c, Hypr_a/b, Line_a/b/c, Circle_r/x0/y0,
    // Parab_f/2p are prm1, prm2, prm3.
    private double prm1;
    private double prm2;
    private double prm3;
    private gp_Ax22d Axis;
    private GeomAbs_CurveType type;
    private gp_Trsf2d Abs_To_Object;

    private readonly gp_Trsf2d Object_To_Abs => Abs_To_Object.Inverted();

    public IntCurve_IConicTool()
    {
        prm1 = 0.0;
        prm2 = 0.0;
        prm3 = 0.0;
        Axis = new gp_Ax22d();
        type = GeomAbs_OtherCurve;
        Abs_To_Object = new gp_Trsf2d();
        // ###### PLACER LE TYPE NON DEFINI ######
    }

    public IntCurve_IConicTool(in gp_Lin2d Line)
        : this()
    {
        Line.Coefficients(out prm1, out prm2, out prm3);
        Axis = new gp_Ax22d(Line.Position(), true);
        type = GeomAbs_Line;
    }

    public IntCurve_IConicTool(in gp_Elips2d Elips)
        : this()
    {
        prm1 = Elips.MajorRadius();
        prm2 = Elips.MinorRadius();
        prm3 = Math.Sqrt(prm1 * prm1 - prm2 * prm2);
        Axis = Elips.Axis();
        Abs_To_Object.SetTransformation(gp.OX2d(), Axis.XAxis());
        type = GeomAbs_Ellipse;
    }

    public IntCurve_IConicTool(in gp_Circ2d C)
        : this()
    {
        prm1 = C.Radius();
        Axis = C.Axis();
        prm2 = Axis.Location().X();
        prm3 = Axis.Location().Y();
        Abs_To_Object.SetTransformation(gp.OX2d(), Axis.XAxis());
        type = GeomAbs_Circle;
    }

    public IntCurve_IConicTool(in gp_Parab2d P)
        : this()
    {
        prm1 = P.Focal();
        prm2 = 4.0 * prm1;
        Axis = P.Axis();
        Abs_To_Object.SetTransformation(gp.OX2d(), Axis.XAxis());
        type = GeomAbs_Parabola;
    }

    public IntCurve_IConicTool(in gp_Hypr2d H)
        : this()
    {
        prm1 = H.MajorRadius();
        prm2 = H.MinorRadius();
        Axis = H.Axis();
        Abs_To_Object.SetTransformation(gp.OX2d(), Axis.XAxis());
        type = GeomAbs_Hyperbola;
    }

    public readonly gp_Pnt2d Value(double X)
    {
        switch (type)
        {
            case GeomAbs_Line:
                return (ElCLib.LineValue(X, Axis.XAxis()));
            case GeomAbs_Ellipse:
                return (ElCLib.EllipseValue(X, Axis, prm1, prm2));
            case GeomAbs_Circle:
                return (ElCLib.CircleValue(X, Axis, prm1));
            case GeomAbs_Parabola:
                return (ElCLib.ParabolaValue(X, Axis, prm1));
            case GeomAbs_Hyperbola:
                return (ElCLib.HyperbolaValue(X, Axis, prm1, prm2));
            default:
            {
                // OCCT prints "### Erreur sur le  type de la courbe ###" to std::cout.
                return (new gp_Pnt2d(0.0, 0.0));
            }
        }
    }

    public readonly void D1(double X, out gp_Pnt2d Pt, out gp_Vec2d Tan)
    {
        switch (type)
        {
            case GeomAbs_Line:
                ElCLib.LineD1(X, Axis.XAxis(), out Pt, out Tan);
                break;
            case GeomAbs_Ellipse:
                ElCLib.EllipseD1(X, Axis, prm1, prm2, out Pt, out Tan);
                break;
            case GeomAbs_Circle:
                ElCLib.CircleD1(X, Axis, prm1, out Pt, out Tan);
                break;
            case GeomAbs_Parabola:
                ElCLib.ParabolaD1(X, Axis, prm1, out Pt, out Tan);
                break;
            case GeomAbs_Hyperbola:
                ElCLib.HyperbolaD1(X, Axis, prm1, prm2, out Pt, out Tan);
                break;
            default:
            {
                // OCCT prints an error and leaves the outputs unchanged (decision 32: zeros).
                Pt = new gp_Pnt2d(0.0, 0.0);
                Tan = new gp_Vec2d(0.0, 0.0);
                break;
            }
        }
    }

    public readonly void D2(double X, out gp_Pnt2d Pt, out gp_Vec2d Tan, out gp_Vec2d Norm)
    {
        switch (type)
        {
            case GeomAbs_Line:
                ElCLib.LineD1(X, Axis.XAxis(), out Pt, out Tan);
                Norm = new gp_Vec2d(0.0, 0.0);
                break;
            case GeomAbs_Ellipse:
                ElCLib.EllipseD2(X, Axis, prm1, prm2, out Pt, out Tan, out Norm);
                break;
            case GeomAbs_Circle:
                ElCLib.CircleD2(X, Axis, prm1, out Pt, out Tan, out Norm);
                break;
            case GeomAbs_Parabola:
                ElCLib.ParabolaD2(X, Axis, prm1, out Pt, out Tan, out Norm);
                break;
            case GeomAbs_Hyperbola:
                ElCLib.HyperbolaD2(X, Axis, prm1, prm2, out Pt, out Tan, out Norm);
                break;
            default:
            {
                // OCCT prints an error and leaves the outputs unchanged (decision 32: zeros).
                Pt = new gp_Pnt2d(0.0, 0.0);
                Tan = new gp_Vec2d(0.0, 0.0);
                Norm = new gp_Vec2d(0.0, 0.0);
                break;
            }
        }
    }

    /// <summary>The signed distance between the point and the implicit curve.</summary>
    public readonly double Distance(in gp_Pnt2d ThePoint)
    {
        switch (type)
        {
            case GeomAbs_Line:
            {
                return (prm1 * ThePoint.X() + prm2 * ThePoint.Y() + prm3);
            }

            case GeomAbs_Ellipse:
            {
                // OCCT's AN_ELIPS is 0: the distance through the affinity on the minor axis.
                gp_Pnt2d P = ThePoint;
                P.Transform(Abs_To_Object);
                double x = P.X();
                double y = P.Y() * (prm1 / prm2);
                double d = Math.Sqrt(x * x + y * y) - prm1;
                return (d);
            }

            case GeomAbs_Circle:
            {
                double Dx = prm2 - ThePoint.X();
                double Dy = prm3 - ThePoint.Y();
                return (Math.Sqrt(Dx * Dx + Dy * Dy) - prm1);
            }

            case GeomAbs_Parabola:
            { //-- Distance(X,Y) = Y**2 - 2 P X
                gp_Pnt2d P = ThePoint;
                P.Transform(Abs_To_Object);
                return (P.Y() * P.Y() - prm2 * P.X());
            }

            case GeomAbs_Hyperbola:
            { //-- Distance(X,Y) = (X/a)**2 - (Y/b)**2 -1
                //--                 pour x>0
                //--     -(Y/b)**2 - 1  sinon ??
                //--     avec un gradient avec x -> std::abs(x)
                gp_Pnt2d P = ThePoint;
                P.Transform(Abs_To_Object);
                if (P.X() > 0.0)
                {
                    return ((P.X() * P.X()) / (prm1 * prm1) - (P.Y() * P.Y()) / (prm2 * prm2) - 1.0);
                }
                else
                {
                    return ((-P.X() * P.X()) / (prm1 * prm1) - (P.Y() * P.Y()) / (prm2 * prm2) - 1.0);
                }
            }
            default:
            {
                // OCCT prints "### Erreur sur le  type de la courbe ###" to std::cout.
                return (0.0);
            }
        }
    }

    /// <summary>The gradient of the signed distance between a point and the implicit curve, at the point.</summary>
    public readonly gp_Vec2d GradDistance(in gp_Pnt2d ThePoint)
    {
        switch (type)
        {
            case GeomAbs_Line:
                return (new gp_Vec2d(prm1, prm2));

            case GeomAbs_Circle:
            {
                gp_Pnt2d P = ThePoint;
                P.Transform(Abs_To_Object);
                double Gradx = 0.0;
                double Grady = 0.0;
                double x = P.X();
                double y = P.Y();
                double temp1 = Math.Sqrt(y * y + x * x);
                if (temp1 != 0)
                {
                    Gradx = x / temp1;
                    Grady = y / temp1;
                }
                gp_Vec2d Gradient = new(Gradx, Grady);
                Gradient.Transform(Object_To_Abs);
                return (Gradient);
            }
            case GeomAbs_Ellipse:
            {
                gp_Pnt2d P = ThePoint;
                P.Transform(Abs_To_Object);
                double Gradx = 0.0;
                double Grady = 0.0;
                double x = P.X();
                double y = P.Y() * (prm1 / prm2);
                double temp1 = Math.Sqrt(y * y + x * x);
                if (temp1 != 0)
                {
                    Gradx = x / temp1;
                    Grady = (y * (prm1 / prm2)) / temp1;
                }
                gp_Vec2d Gradient = new(Gradx, Grady);
                Gradient.Transform(Object_To_Abs);
                return (Gradient);
            }

            case GeomAbs_Parabola:
            { //-- Distance(X,Y) = Y**2 - 2 P X
                gp_Pnt2d P = ThePoint;
                P.Transform(Abs_To_Object);
                gp_Vec2d Gradient = new(-prm2, P.Y() + P.Y());
                Gradient.Transform(Object_To_Abs);
                return (Gradient);
            }
            case GeomAbs_Hyperbola:
            { //-- Distance(X,Y) = (X/a)**2 - (Y/b)**2 -1
                gp_Pnt2d P = ThePoint;
                P.Transform(Abs_To_Object);
                //--### la Branche a X negatif doit ramener vers les X positifs
                gp_Vec2d Gradient = new(2.0 * Math.Abs(P.X()) / (prm1 * prm1), -2.0 * P.Y() / (prm2 * prm2));
                Gradient.Transform(Object_To_Abs);
                return (Gradient);
            }
            default:
            {
                // OCCT prints "### Erreur sur le  type de la courbe ###" to std::cout.
                return (new gp_Vec2d(0.0, 0.0));
            }
        }
    }

    /// <summary>
    /// The parameter of the point of the implicit curve corresponding to <paramref name="P"/>, coherent with the
    /// signed distance.
    /// </summary>
    public readonly double FindParameter(in gp_Pnt2d P)
    {
        double Param = 0;

        switch (type)
        {
            case GeomAbs_Line:
                Param = ElCLib.LineParameter(Axis.XAxis(), P);
                break;

            case GeomAbs_Circle:
                Param = ElCLib.CircleParameter(Axis, P);
                if (Param < 0.0)
                {
                    Param += M_PI + M_PI;
                }
                break;

            case GeomAbs_Ellipse:
            {
                Param = ElCLib.EllipseParameter(Axis, prm1, prm2, P);
                if (Param < 0.0)
                {
                    Param += M_PI + M_PI;
                }
                break;
            }

            case GeomAbs_Parabola:
            {
                Param = ElCLib.ParabolaParameter(Axis, P);
                break;
            }
            case GeomAbs_Hyperbola:
            {
                Param = ElCLib.HyperbolaParameter(Axis, prm1, prm2, P);
                break;
            }
            default:
                break;
        }

        return (Param);
    }
}
