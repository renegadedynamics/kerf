// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IConicTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IConicTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

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
    private Frame2D Axis;
    private CurveKind type;
    private Transform2D Abs_To_Object;

    private readonly Transform2D Object_To_Abs => Abs_To_Object.Inverted();

    public IntCurve_IConicTool()
    {
        prm1 = 0.0;
        prm2 = 0.0;
        prm3 = 0.0;
        Axis = new Frame2D();
        type = OtherCurve;
        Abs_To_Object = new Transform2D();
        // ###### PLACER LE TYPE NON DEFINI ######
    }

    public IntCurve_IConicTool(in Line2D Line)
        : this()
    {
        Line.Coefficients(out prm1, out prm2, out prm3);
        Axis = new Frame2D(Line.Position(), true);
        type = global::Kerf.Geometry.CurveKind.Line;
    }

    public IntCurve_IConicTool(in Ellipse2D Elips)
        : this()
    {
        prm1 = Elips.MajorRadius();
        prm2 = Elips.MinorRadius();
        prm3 = Math.Sqrt(prm1 * prm1 - prm2 * prm2);
        Axis = Elips.Axis();
        Abs_To_Object.SetTransformation(Constants.OX2d(), Axis.XAxis());
        type = Ellipse;
    }

    public IntCurve_IConicTool(in Circle2D C)
        : this()
    {
        prm1 = C.Radius();
        Axis = C.Axis();
        prm2 = Axis.Location().X();
        prm3 = Axis.Location().Y();
        Abs_To_Object.SetTransformation(Constants.OX2d(), Axis.XAxis());
        type = Circle;
    }

    public IntCurve_IConicTool(in Parabola2D P)
        : this()
    {
        prm1 = P.Focal();
        prm2 = 4.0 * prm1;
        Axis = P.Axis();
        Abs_To_Object.SetTransformation(Constants.OX2d(), Axis.XAxis());
        type = Parabola;
    }

    public IntCurve_IConicTool(in Hyperbola2D H)
        : this()
    {
        prm1 = H.MajorRadius();
        prm2 = H.MinorRadius();
        Axis = H.Axis();
        Abs_To_Object.SetTransformation(Constants.OX2d(), Axis.XAxis());
        type = Hyperbola;
    }

    public readonly Point2D Value(double X)
    {
        switch (type)
        {
            case Line:
                return (global::Kerf.Engine.Geometry.ElCLib.LineValue(X, Axis.XAxis()));
            case Ellipse:
                return (global::Kerf.Engine.Geometry.ElCLib.EllipseValue(X, Axis, prm1, prm2));
            case Circle:
                return (global::Kerf.Engine.Geometry.ElCLib.CircleValue(X, Axis, prm1));
            case Parabola:
                return (global::Kerf.Engine.Geometry.ElCLib.ParabolaValue(X, Axis, prm1));
            case Hyperbola:
                return (global::Kerf.Engine.Geometry.ElCLib.HyperbolaValue(X, Axis, prm1, prm2));
            default:
            {
                // OCCT prints "### Erreur sur le  type de la courbe ###" to std::cout.
                return (new Point2D(0.0, 0.0));
            }
        }
    }

    public readonly void D1(double X, out Point2D Pt, out Vector2D Tan)
    {
        switch (type)
        {
            case Line:
                global::Kerf.Engine.Geometry.ElCLib.LineD1(X, Axis.XAxis(), out Pt, out Tan);
                break;
            case Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.EllipseD1(X, Axis, prm1, prm2, out Pt, out Tan);
                break;
            case Circle:
                global::Kerf.Engine.Geometry.ElCLib.CircleD1(X, Axis, prm1, out Pt, out Tan);
                break;
            case Parabola:
                global::Kerf.Engine.Geometry.ElCLib.ParabolaD1(X, Axis, prm1, out Pt, out Tan);
                break;
            case Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.HyperbolaD1(X, Axis, prm1, prm2, out Pt, out Tan);
                break;
            default:
            {
                // OCCT prints an error and leaves the outputs unchanged (decision 32: zeros).
                Pt = new Point2D(0.0, 0.0);
                Tan = new Vector2D(0.0, 0.0);
                break;
            }
        }
    }

    public readonly void D2(double X, out Point2D Pt, out Vector2D Tan, out Vector2D Norm)
    {
        switch (type)
        {
            case Line:
                global::Kerf.Engine.Geometry.ElCLib.LineD1(X, Axis.XAxis(), out Pt, out Tan);
                Norm = new Vector2D(0.0, 0.0);
                break;
            case Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.EllipseD2(X, Axis, prm1, prm2, out Pt, out Tan, out Norm);
                break;
            case Circle:
                global::Kerf.Engine.Geometry.ElCLib.CircleD2(X, Axis, prm1, out Pt, out Tan, out Norm);
                break;
            case Parabola:
                global::Kerf.Engine.Geometry.ElCLib.ParabolaD2(X, Axis, prm1, out Pt, out Tan, out Norm);
                break;
            case Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.HyperbolaD2(X, Axis, prm1, prm2, out Pt, out Tan, out Norm);
                break;
            default:
            {
                // OCCT prints an error and leaves the outputs unchanged (decision 32: zeros).
                Pt = new Point2D(0.0, 0.0);
                Tan = new Vector2D(0.0, 0.0);
                Norm = new Vector2D(0.0, 0.0);
                break;
            }
        }
    }

    /// <summary>The signed distance between the point and the implicit curve.</summary>
    public readonly double Distance(in Point2D ThePoint)
    {
        switch (type)
        {
            case Line:
            {
                return (prm1 * ThePoint.X() + prm2 * ThePoint.Y() + prm3);
            }

            case Ellipse:
            {
                // OCCT's AN_ELIPS is 0: the distance through the affinity on the minor axis.
                Point2D P = ThePoint;
                P.Transform(Abs_To_Object);
                double x = P.X();
                double y = P.Y() * (prm1 / prm2);
                double d = Math.Sqrt(x * x + y * y) - prm1;
                return (d);
            }

            case Circle:
            {
                double Dx = prm2 - ThePoint.X();
                double Dy = prm3 - ThePoint.Y();
                return (Math.Sqrt(Dx * Dx + Dy * Dy) - prm1);
            }

            case Parabola:
            { //-- Distance(X,Y) = Y**2 - 2 P X
                Point2D P = ThePoint;
                P.Transform(Abs_To_Object);
                return (P.Y() * P.Y() - prm2 * P.X());
            }

            case Hyperbola:
            { //-- Distance(X,Y) = (X/a)**2 - (Y/b)**2 -1
                //--                 pour x>0
                //--     -(Y/b)**2 - 1  sinon ??
                //--     avec un gradient avec x -> std::abs(x)
                Point2D P = ThePoint;
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
    public readonly Vector2D GradDistance(in Point2D ThePoint)
    {
        switch (type)
        {
            case Line:
                return (new Vector2D(prm1, prm2));

            case Circle:
            {
                Point2D P = ThePoint;
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
                Vector2D Gradient = new(Gradx, Grady);
                Gradient.Transform(Object_To_Abs);
                return (Gradient);
            }
            case Ellipse:
            {
                Point2D P = ThePoint;
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
                Vector2D Gradient = new(Gradx, Grady);
                Gradient.Transform(Object_To_Abs);
                return (Gradient);
            }

            case Parabola:
            { //-- Distance(X,Y) = Y**2 - 2 P X
                Point2D P = ThePoint;
                P.Transform(Abs_To_Object);
                Vector2D Gradient = new(-prm2, P.Y() + P.Y());
                Gradient.Transform(Object_To_Abs);
                return (Gradient);
            }
            case Hyperbola:
            { //-- Distance(X,Y) = (X/a)**2 - (Y/b)**2 -1
                Point2D P = ThePoint;
                P.Transform(Abs_To_Object);
                //--### la Branche a X negatif doit ramener vers les X positifs
                Vector2D Gradient = new(2.0 * Math.Abs(P.X()) / (prm1 * prm1), -2.0 * P.Y() / (prm2 * prm2));
                Gradient.Transform(Object_To_Abs);
                return (Gradient);
            }
            default:
            {
                // OCCT prints "### Erreur sur le  type de la courbe ###" to std::cout.
                return (new Vector2D(0.0, 0.0));
            }
        }
    }

    /// <summary>
    /// The parameter of the point of the implicit curve corresponding to <paramref name="P"/>, coherent with the
    /// signed distance.
    /// </summary>
    public readonly double FindParameter(in Point2D P)
    {
        double Param = 0;

        switch (type)
        {
            case Line:
                Param = global::Kerf.Engine.Geometry.ElCLib.LineParameter(Axis.XAxis(), P);
                break;

            case Circle:
                Param = global::Kerf.Engine.Geometry.ElCLib.CircleParameter(Axis, P);
                if (Param < 0.0)
                {
                    Param += M_PI + M_PI;
                }
                break;

            case Ellipse:
            {
                Param = global::Kerf.Engine.Geometry.ElCLib.EllipseParameter(Axis, prm1, prm2, P);
                if (Param < 0.0)
                {
                    Param += M_PI + M_PI;
                }
                break;
            }

            case Parabola:
            {
                Param = global::Kerf.Engine.Geometry.ElCLib.ParabolaParameter(Axis, P);
                break;
            }
            case Hyperbola:
            {
                Param = global::Kerf.Engine.Geometry.ElCLib.HyperbolaParameter(Axis, prm1, prm2, P);
                break;
            }
            default:
                break;
        }

        return (Param);
    }
}
