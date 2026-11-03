// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_ProjectOnPConicTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_ProjectOnPConicTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// The parameter of a point on an <see cref="IntCurve_PConic"/> (IntImpParGen's ProjectOnPCurveTool for conics).
/// </summary>
/// <remarks>A struct without state, so it can be a type argument of the generic intersectors.</remarks>
public readonly struct IntCurve_ProjectOnPConicTool : IIntImpParGen_ProjectOnPCurveTool<IntCurve_PConic>
{
    /// <summary>
    /// The parameter of the point of the conic corresponding to <paramref name="P"/>, clamped to the interval
    /// between the two parameters when they differ.
    /// </summary>
    public static double FindParameter(in IntCurve_PConic ThePConic, in Point2D P, double LowParameter,
                                       double HighParameter, double Tol)
    {
        double ParamSup, ParamInf, Param = 0;
        if (LowParameter > HighParameter)
        {
            ParamSup = LowParameter;
            ParamInf = HighParameter;
        }
        else
        {
            ParamInf = LowParameter;
            ParamSup = HighParameter;
        }

        switch (ThePConic.TypeCurve())
        {
            case Line:
                Param = global::Kerf.Engine.Geometry.ElCLib.LineParameter(ThePConic.Axis2().XAxis(), P);
                break;

            case Circle:
                Param = global::Kerf.Engine.Geometry.ElCLib.CircleParameter(ThePConic.Axis2(), P);
                if (Param < 0.0)
                {
                    Param += M_PI + M_PI;
                }
                break;

            case Ellipse:
            {
                Param = global::Kerf.Engine.Geometry.ElCLib.EllipseParameter(ThePConic.Axis2(), ThePConic.Param1(), ThePConic.Param2(), P);
                if (Param < 0.0)
                {
                    Param += M_PI + M_PI;
                }
                break;
            }

            case Parabola:
            {
                Param = global::Kerf.Engine.Geometry.ElCLib.ParabolaParameter(ThePConic.Axis2(), P);
                break;
            }
            case Hyperbola:
            {
                Param = global::Kerf.Engine.Geometry.ElCLib.HyperbolaParameter(ThePConic.Axis2(), ThePConic.Param1(), ThePConic.Param2(), P);
                break;
            }
            default:
                break;
        }

        if (ParamInf != ParamSup)
        {
            if (Param < ParamInf)
            {
                return (ParamInf);
            }
            if (Param > ParamSup)
            {
                return (ParamSup);
            }
        }
        return (Param);
    }

    /// <summary>The parameter of the point of the conic corresponding to <paramref name="P"/>.</summary>
    public static double FindParameter(in IntCurve_PConic ThePConic, in Point2D P, double Tol)
    {
        double Param = 0;
        switch (ThePConic.TypeCurve())
        {
            case Line:
                Param = global::Kerf.Engine.Geometry.ElCLib.LineParameter(ThePConic.Axis2().XAxis(), P);
                break;

            case Circle:
                Param = global::Kerf.Engine.Geometry.ElCLib.CircleParameter(ThePConic.Axis2(), P);
                if (Param < 0.0)
                {
                    Param += M_PI + M_PI;
                }
                break;

            case Ellipse:
            {
                Param = global::Kerf.Engine.Geometry.ElCLib.EllipseParameter(ThePConic.Axis2(), ThePConic.Param1(), ThePConic.Param2(), P);
                if (Param < 0.0)
                {
                    Param += M_PI + M_PI;
                }
                break;
            }

            case Parabola:
            {
                Param = global::Kerf.Engine.Geometry.ElCLib.ParabolaParameter(ThePConic.Axis2(), P);
                break;
            }
            case Hyperbola:
            {
                Param = global::Kerf.Engine.Geometry.ElCLib.HyperbolaParameter(ThePConic.Axis2(), ThePConic.Param1(), ThePConic.Param2(), P);
                break;
            }
            default:
                break;
        }
        return (Param);
    }
}
