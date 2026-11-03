// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IConicTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IConicTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Intersection.Curves;

/// <summary>
/// A conic of gp seen as an implicit curve: OCCT's implementation of IntImpParGen's ImpTool for conics.
/// </summary>
/// <remarks>
/// A struct, copied as OCCT copies it. A working tool (as the result structs of decision 54), so it has no
/// equality. <c>default(IntCurve_IConicTool)</c> has a zero transformation; use the constructors.
/// </remarks>
public struct IConicTool : IImpTool
{
    // OCCT's macros naming the parameters by conic: Elips_a/b/c, Hypr_a/b, Line_a/b/c, Circle_r/x0/y0,
    // Parab_f/2p are prm1, prm2, prm3.
    private double _prm1;
    private double _prm2;
    private double _prm3;
    private Frame2D _axis;
    private CurveKind _type;
    private Transform2D _absToObject;

    private readonly Transform2D Object_To_Abs => _absToObject.Inverted();

    public IConicTool()
    {
        _prm1 = 0.0;
        _prm2 = 0.0;
        _prm3 = 0.0;
        _axis = new Frame2D();
        _type = OtherCurve;
        _absToObject = new Transform2D();
        // ###### PLACER LE TYPE NON DEFINI ######
    }

    public IConicTool(in Line2D line)
        : this()
    {
        line.Coefficients(out _prm1, out _prm2, out _prm3);
        _axis = new Frame2D(line.Position(), true);
        _type = global::Kerf.Geometry.CurveKind.Line;
    }

    public IConicTool(in Ellipse2D elips)
        : this()
    {
        _prm1 = elips.MajorRadius();
        _prm2 = elips.MinorRadius();
        _prm3 = Math.Sqrt(_prm1 * _prm1 - _prm2 * _prm2);
        _axis = elips.Axis();
        _absToObject.SetTransformation(Constants.OX2d(), _axis.XAxis());
        _type = Ellipse;
    }

    public IConicTool(in Circle2D c)
        : this()
    {
        _prm1 = c.Radius();
        _axis = c.Axis();
        _prm2 = _axis.Location().X();
        _prm3 = _axis.Location().Y();
        _absToObject.SetTransformation(Constants.OX2d(), _axis.XAxis());
        _type = Circle;
    }

    public IConicTool(in Parabola2D p)
        : this()
    {
        _prm1 = p.Focal();
        _prm2 = 4.0 * _prm1;
        _axis = p.Axis();
        _absToObject.SetTransformation(Constants.OX2d(), _axis.XAxis());
        _type = Parabola;
    }

    public IConicTool(in Hyperbola2D h)
        : this()
    {
        _prm1 = h.MajorRadius();
        _prm2 = h.MinorRadius();
        _axis = h.Axis();
        _absToObject.SetTransformation(Constants.OX2d(), _axis.XAxis());
        _type = Hyperbola;
    }

    public readonly Point2D Value(double x)
    {
        switch (_type)
        {
            case Line:
                return (global::Kerf.Engine.Geometry.ElCLib.LineValue(x, _axis.XAxis()));
            case Ellipse:
                return (global::Kerf.Engine.Geometry.ElCLib.EllipseValue(x, _axis, _prm1, _prm2));
            case Circle:
                return (global::Kerf.Engine.Geometry.ElCLib.CircleValue(x, _axis, _prm1));
            case Parabola:
                return (global::Kerf.Engine.Geometry.ElCLib.ParabolaValue(x, _axis, _prm1));
            case Hyperbola:
                return (global::Kerf.Engine.Geometry.ElCLib.HyperbolaValue(x, _axis, _prm1, _prm2));
            default:
            {
                // OCCT prints "### Erreur sur le  type de la courbe ###" to std::cout.
                return (new Point2D(0.0, 0.0));
            }
        }
    }

    public readonly void D1(double x, out Point2D pt, out Vector2D tan)
    {
        switch (_type)
        {
            case Line:
                global::Kerf.Engine.Geometry.ElCLib.LineD1(x, _axis.XAxis(), out pt, out tan);
                break;
            case Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.EllipseD1(x, _axis, _prm1, _prm2, out pt, out tan);
                break;
            case Circle:
                global::Kerf.Engine.Geometry.ElCLib.CircleD1(x, _axis, _prm1, out pt, out tan);
                break;
            case Parabola:
                global::Kerf.Engine.Geometry.ElCLib.ParabolaD1(x, _axis, _prm1, out pt, out tan);
                break;
            case Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.HyperbolaD1(x, _axis, _prm1, _prm2, out pt, out tan);
                break;
            default:
            {
                // OCCT prints an error and leaves the outputs unchanged (decision 32: zeros).
                pt = new Point2D(0.0, 0.0);
                tan = new Vector2D(0.0, 0.0);
                break;
            }
        }
    }

    public readonly void D2(double x, out Point2D pt, out Vector2D tan, out Vector2D norm)
    {
        switch (_type)
        {
            case Line:
                global::Kerf.Engine.Geometry.ElCLib.LineD1(x, _axis.XAxis(), out pt, out tan);
                norm = new Vector2D(0.0, 0.0);
                break;
            case Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.EllipseD2(x, _axis, _prm1, _prm2, out pt, out tan, out norm);
                break;
            case Circle:
                global::Kerf.Engine.Geometry.ElCLib.CircleD2(x, _axis, _prm1, out pt, out tan, out norm);
                break;
            case Parabola:
                global::Kerf.Engine.Geometry.ElCLib.ParabolaD2(x, _axis, _prm1, out pt, out tan, out norm);
                break;
            case Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.HyperbolaD2(x, _axis, _prm1, _prm2, out pt, out tan, out norm);
                break;
            default:
            {
                // OCCT prints an error and leaves the outputs unchanged (decision 32: zeros).
                pt = new Point2D(0.0, 0.0);
                tan = new Vector2D(0.0, 0.0);
                norm = new Vector2D(0.0, 0.0);
                break;
            }
        }
    }

    /// <summary>The signed distance between the point and the implicit curve.</summary>
    public readonly double Distance(in Point2D thePoint)
    {
        switch (_type)
        {
            case Line:
            {
                return (_prm1 * thePoint.X() + _prm2 * thePoint.Y() + _prm3);
            }

            case Ellipse:
            {
                // OCCT's AN_ELIPS is 0: the distance through the affinity on the minor axis.
                Point2D p = thePoint;
                p.Transform(_absToObject);
                double x = p.X();
                double y = p.Y() * (_prm1 / _prm2);
                double d = Math.Sqrt(x * x + y * y) - _prm1;
                return (d);
            }

            case Circle:
            {
                double dx = _prm2 - thePoint.X();
                double dy = _prm3 - thePoint.Y();
                return (Math.Sqrt(dx * dx + dy * dy) - _prm1);
            }

            case Parabola:
            { //-- Distance(X,Y) = Y**2 - 2 P X
                Point2D p = thePoint;
                p.Transform(_absToObject);
                return (p.Y() * p.Y() - _prm2 * p.X());
            }

            case Hyperbola:
            { //-- Distance(X,Y) = (X/a)**2 - (Y/b)**2 -1
                //--                 pour x>0
                //--     -(Y/b)**2 - 1  sinon ??
                //--     avec un gradient avec x -> std::abs(x)
                Point2D p = thePoint;
                p.Transform(_absToObject);
                if (p.X() > 0.0)
                {
                    return ((p.X() * p.X()) / (_prm1 * _prm1) - (p.Y() * p.Y()) / (_prm2 * _prm2) - 1.0);
                }
                else
                {
                    return ((-p.X() * p.X()) / (_prm1 * _prm1) - (p.Y() * p.Y()) / (_prm2 * _prm2) - 1.0);
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
    public readonly Vector2D GradDistance(in Point2D thePoint)
    {
        switch (_type)
        {
            case Line:
                return (new Vector2D(_prm1, _prm2));

            case Circle:
            {
                Point2D p = thePoint;
                p.Transform(_absToObject);
                double gradx = 0.0;
                double grady = 0.0;
                double x = p.X();
                double y = p.Y();
                double temp1 = Math.Sqrt(y * y + x * x);
                if (temp1 != 0)
                {
                    gradx = x / temp1;
                    grady = y / temp1;
                }
                Vector2D gradient = new(gradx, grady);
                gradient.Transform(Object_To_Abs);
                return (gradient);
            }
            case Ellipse:
            {
                Point2D p = thePoint;
                p.Transform(_absToObject);
                double gradx = 0.0;
                double grady = 0.0;
                double x = p.X();
                double y = p.Y() * (_prm1 / _prm2);
                double temp1 = Math.Sqrt(y * y + x * x);
                if (temp1 != 0)
                {
                    gradx = x / temp1;
                    grady = (y * (_prm1 / _prm2)) / temp1;
                }
                Vector2D gradient = new(gradx, grady);
                gradient.Transform(Object_To_Abs);
                return (gradient);
            }

            case Parabola:
            { //-- Distance(X,Y) = Y**2 - 2 P X
                Point2D p = thePoint;
                p.Transform(_absToObject);
                Vector2D gradient = new(-_prm2, p.Y() + p.Y());
                gradient.Transform(Object_To_Abs);
                return (gradient);
            }
            case Hyperbola:
            { //-- Distance(X,Y) = (X/a)**2 - (Y/b)**2 -1
                Point2D p = thePoint;
                p.Transform(_absToObject);
                //--### la Branche a X negatif doit ramener vers les X positifs
                Vector2D gradient = new(2.0 * Math.Abs(p.X()) / (_prm1 * _prm1), -2.0 * p.Y() / (_prm2 * _prm2));
                gradient.Transform(Object_To_Abs);
                return (gradient);
            }
            default:
            {
                // OCCT prints "### Erreur sur le  type de la courbe ###" to std::cout.
                return (new Vector2D(0.0, 0.0));
            }
        }
    }

    /// <summary>
    /// The parameter of the point of the implicit curve corresponding to <paramref name="p"/>, coherent with the
    /// signed distance.
    /// </summary>
    public readonly double FindParameter(in Point2D p)
    {
        double param = 0;

        switch (_type)
        {
            case Line:
                param = global::Kerf.Engine.Geometry.ElCLib.LineParameter(_axis.XAxis(), p);
                break;

            case Circle:
                param = global::Kerf.Engine.Geometry.ElCLib.CircleParameter(_axis, p);
                if (param < 0.0)
                {
                    param += M_PI + M_PI;
                }
                break;

            case Ellipse:
            {
                param = global::Kerf.Engine.Geometry.ElCLib.EllipseParameter(_axis, _prm1, _prm2, p);
                if (param < 0.0)
                {
                    param += M_PI + M_PI;
                }
                break;
            }

            case Parabola:
            {
                param = global::Kerf.Engine.Geometry.ElCLib.ParabolaParameter(_axis, p);
                break;
            }
            case Hyperbola:
            {
                param = global::Kerf.Engine.Geometry.ElCLib.HyperbolaParameter(_axis, _prm1, _prm2, p);
                break;
            }
            default:
                break;
        }

        return (param);
    }
}
