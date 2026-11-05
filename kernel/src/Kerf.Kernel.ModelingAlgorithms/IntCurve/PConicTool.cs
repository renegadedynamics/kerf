// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_PConicTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_PConicTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Implicit;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Intersection.Curves;

/// <summary>The evaluation of an <see cref="PConic"/> (IntImpParGen's ParTool for conics).</summary>
/// <remarks>A struct without state, so it can be a type argument of the generic intersectors.</remarks>
internal readonly struct PConicTool : IParTool<PConic>
{
    public static Point2D Value(in PConic pConic, double x)
    {
        switch (pConic.TypeCurve())
        {
            case Line:
                return (global::Kerf.Engine.Geometry.ElCLib.LineValue(x, pConic.Axis2().XAxis));
            case Circle:
                return (global::Kerf.Engine.Geometry.ElCLib.CircleValue(x, pConic.Axis2(), pConic.Param1()));
            case Ellipse:
                return (global::Kerf.Engine.Geometry.ElCLib.EllipseValue(x, pConic.Axis2(), pConic.Param1(), pConic.Param2()));
            case Parabola:
                return (global::Kerf.Engine.Geometry.ElCLib.ParabolaValue(x, pConic.Axis2(), pConic.Param1()));
            default: //-- case GeomAbs_Hyperbola:
                return (global::Kerf.Engine.Geometry.ElCLib.HyperbolaValue(x, pConic.Axis2(), pConic.Param1(), pConic.Param2()));
        }
    }

    public static void D1(in PConic pConic, double u, out Point2D pt, out Vector2D tan)
    {
        switch (pConic.TypeCurve())
        {
            case Line:
                global::Kerf.Engine.Geometry.ElCLib.LineD1(u, pConic.Axis2().XAxis, out pt, out tan);
                break;
            case Circle:
                global::Kerf.Engine.Geometry.ElCLib.CircleD1(u, pConic.Axis2(), pConic.Param1(), out pt, out tan);
                break;
            case Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.EllipseD1(u, pConic.Axis2(), pConic.Param1(), pConic.Param2(), out pt, out tan);
                break;
            case Parabola:
                global::Kerf.Engine.Geometry.ElCLib.ParabolaD1(u, pConic.Axis2(), pConic.Param1(), out pt, out tan);
                break;
            case Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.HyperbolaD1(u, pConic.Axis2(), pConic.Param1(), pConic.Param2(), out pt, out tan);
                break;
            default:
                // OCCT leaves the outputs unchanged (decision 32: zeros).
                pt = new Point2D(0.0, 0.0);
                tan = new Vector2D(0.0, 0.0);
                break;
        }
    }

    public static void D2(in PConic pConic, double u, out Point2D pt, out Vector2D tan, out Vector2D norm)
    {
        switch (pConic.TypeCurve())
        {
            case Line:
                // OCCT clears Tan (which LineD1 then writes) and leaves Norm unwritten: its callers' Norm stays
                // as default-constructed, (0, 0), which the port writes.
                norm = new Vector2D(0.0, 0.0);
                global::Kerf.Engine.Geometry.ElCLib.LineD1(u, pConic.Axis2().XAxis, out pt, out tan);
                break;
            case Circle:
                global::Kerf.Engine.Geometry.ElCLib.CircleD2(u, pConic.Axis2(), pConic.Param1(), out pt, out tan, out norm);
                break;
            case Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.EllipseD2(u, pConic.Axis2(), pConic.Param1(), pConic.Param2(), out pt, out tan, out norm);
                break;
            case Parabola:
                global::Kerf.Engine.Geometry.ElCLib.ParabolaD2(u, pConic.Axis2(), pConic.Param1(), out pt, out tan, out norm);
                break;
            case Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.HyperbolaD2(u, pConic.Axis2(), pConic.Param1(), pConic.Param2(), out pt, out tan, out norm);
                break;
            default:
                // OCCT leaves the outputs unchanged (decision 32: zeros).
                pt = new Point2D(0.0, 0.0);
                tan = new Vector2D(0.0, 0.0);
                norm = new Vector2D(0.0, 0.0);
                break;
        }
    }

    public static double EpsX(in PConic pConic) => (pConic.EpsX());

    public static int NbSamples(in PConic pConic) => (pConic.Accuracy());

    public static int NbSamples(in PConic pConic, double u0, double u1) => (pConic.Accuracy());
}
