// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_ProjectOnPConicTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_ProjectOnPConicTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Implicit;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Intersection.Curves;

/// <summary>
/// The parameter of a point on an <see cref="PConic"/> (IntImpParGen's ProjectOnPCurveTool for conics).
/// </summary>
/// <remarks>A struct without state, so it can be a type argument of the generic intersectors.</remarks>
internal readonly struct ProjectOnPConicTool : IProjectOnPCurveTool<PConic>
{
    /// <summary>
    /// The parameter of the point of the conic corresponding to <paramref name="p"/>, clamped to the interval
    /// between the two parameters when they differ.
    /// </summary>
    public static double FindParameter(in PConic thePConic, in Point2D p, double lowParameter,
                                       double highParameter, double tol)
    {
        double paramSup, paramInf, param = 0;
        if (lowParameter > highParameter)
        {
            paramSup = lowParameter;
            paramInf = highParameter;
        }
        else
        {
            paramInf = lowParameter;
            paramSup = highParameter;
        }

        switch (thePConic.TypeCurve())
        {
            case Line:
                param = global::Kerf.Engine.Geometry.ElCLib.LineParameter(thePConic.Axis2().XAxis, p);
                break;

            case Circle:
                param = global::Kerf.Engine.Geometry.ElCLib.CircleParameter(thePConic.Axis2(), p);
                if (param < 0.0)
                {
                    param += M_PI + M_PI;
                }
                break;

            case Ellipse:
            {
                param = global::Kerf.Engine.Geometry.ElCLib.EllipseParameter(thePConic.Axis2(), thePConic.Param1(), thePConic.Param2(), p);
                if (param < 0.0)
                {
                    param += M_PI + M_PI;
                }
                break;
            }

            case Parabola:
            {
                param = global::Kerf.Engine.Geometry.ElCLib.ParabolaParameter(thePConic.Axis2(), p);
                break;
            }
            case Hyperbola:
            {
                param = global::Kerf.Engine.Geometry.ElCLib.HyperbolaParameter(thePConic.Axis2(), thePConic.Param1(), thePConic.Param2(), p);
                break;
            }
            default:
                break;
        }

        if (paramInf != paramSup)
        {
            if (param < paramInf)
            {
                return (paramInf);
            }
            if (param > paramSup)
            {
                return (paramSup);
            }
        }
        return (param);
    }

    /// <summary>The parameter of the point of the conic corresponding to <paramref name="p"/>.</summary>
    public static double FindParameter(in PConic thePConic, in Point2D p, double tol)
    {
        double param = 0;
        switch (thePConic.TypeCurve())
        {
            case Line:
                param = global::Kerf.Engine.Geometry.ElCLib.LineParameter(thePConic.Axis2().XAxis, p);
                break;

            case Circle:
                param = global::Kerf.Engine.Geometry.ElCLib.CircleParameter(thePConic.Axis2(), p);
                if (param < 0.0)
                {
                    param += M_PI + M_PI;
                }
                break;

            case Ellipse:
            {
                param = global::Kerf.Engine.Geometry.ElCLib.EllipseParameter(thePConic.Axis2(), thePConic.Param1(), thePConic.Param2(), p);
                if (param < 0.0)
                {
                    param += M_PI + M_PI;
                }
                break;
            }

            case Parabola:
            {
                param = global::Kerf.Engine.Geometry.ElCLib.ParabolaParameter(thePConic.Axis2(), p);
                break;
            }
            case Hyperbola:
            {
                param = global::Kerf.Engine.Geometry.ElCLib.HyperbolaParameter(thePConic.Axis2(), thePConic.Param1(), thePConic.Param2(), p);
                break;
            }
            default:
                break;
        }
        return (param);
    }
}
