// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_PConicTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_PConicTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>The evaluation of an <see cref="IntCurve_PConic"/> (IntImpParGen's ParTool for conics).</summary>
/// <remarks>A struct without state, so it can be a type argument of the generic intersectors.</remarks>
public readonly struct IntCurve_PConicTool : IIntImpParGen_ParTool<IntCurve_PConic>
{
    public static Point2D Value(in IntCurve_PConic PConic, double X)
    {
        switch (PConic.TypeCurve())
        {
            case Line:
                return (global::Kerf.Engine.Geometry.ElCLib.LineValue(X, PConic.Axis2().XAxis()));
            case Circle:
                return (global::Kerf.Engine.Geometry.ElCLib.CircleValue(X, PConic.Axis2(), PConic.Param1()));
            case Ellipse:
                return (global::Kerf.Engine.Geometry.ElCLib.EllipseValue(X, PConic.Axis2(), PConic.Param1(), PConic.Param2()));
            case Parabola:
                return (global::Kerf.Engine.Geometry.ElCLib.ParabolaValue(X, PConic.Axis2(), PConic.Param1()));
            default: //-- case GeomAbs_Hyperbola:
                return (global::Kerf.Engine.Geometry.ElCLib.HyperbolaValue(X, PConic.Axis2(), PConic.Param1(), PConic.Param2()));
        }
    }

    public static void D1(in IntCurve_PConic PConic, double U, out Point2D Pt, out Vector2D Tan)
    {
        switch (PConic.TypeCurve())
        {
            case Line:
                global::Kerf.Engine.Geometry.ElCLib.LineD1(U, PConic.Axis2().XAxis(), out Pt, out Tan);
                break;
            case Circle:
                global::Kerf.Engine.Geometry.ElCLib.CircleD1(U, PConic.Axis2(), PConic.Param1(), out Pt, out Tan);
                break;
            case Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.EllipseD1(U, PConic.Axis2(), PConic.Param1(), PConic.Param2(), out Pt, out Tan);
                break;
            case Parabola:
                global::Kerf.Engine.Geometry.ElCLib.ParabolaD1(U, PConic.Axis2(), PConic.Param1(), out Pt, out Tan);
                break;
            case Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.HyperbolaD1(U, PConic.Axis2(), PConic.Param1(), PConic.Param2(), out Pt, out Tan);
                break;
            default:
                // OCCT leaves the outputs unchanged (decision 32: zeros).
                Pt = new Point2D(0.0, 0.0);
                Tan = new Vector2D(0.0, 0.0);
                break;
        }
    }

    public static void D2(in IntCurve_PConic PConic, double U, out Point2D Pt, out Vector2D Tan, out Vector2D Norm)
    {
        switch (PConic.TypeCurve())
        {
            case Line:
                // OCCT clears Tan (which LineD1 then writes) and leaves Norm unwritten: its callers' Norm stays
                // as default-constructed, (0, 0), which the port writes.
                Norm = new Vector2D(0.0, 0.0);
                global::Kerf.Engine.Geometry.ElCLib.LineD1(U, PConic.Axis2().XAxis(), out Pt, out Tan);
                break;
            case Circle:
                global::Kerf.Engine.Geometry.ElCLib.CircleD2(U, PConic.Axis2(), PConic.Param1(), out Pt, out Tan, out Norm);
                break;
            case Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.EllipseD2(U, PConic.Axis2(), PConic.Param1(), PConic.Param2(), out Pt, out Tan, out Norm);
                break;
            case Parabola:
                global::Kerf.Engine.Geometry.ElCLib.ParabolaD2(U, PConic.Axis2(), PConic.Param1(), out Pt, out Tan, out Norm);
                break;
            case Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.HyperbolaD2(U, PConic.Axis2(), PConic.Param1(), PConic.Param2(), out Pt, out Tan, out Norm);
                break;
            default:
                // OCCT leaves the outputs unchanged (decision 32: zeros).
                Pt = new Point2D(0.0, 0.0);
                Tan = new Vector2D(0.0, 0.0);
                Norm = new Vector2D(0.0, 0.0);
                break;
        }
    }

    public static double EpsX(in IntCurve_PConic PConic) => (PConic.EpsX());

    public static int NbSamples(in IntCurve_PConic PConic) => (PConic.Accuracy());

    public static int NbSamples(in IntCurve_PConic PConic, double U0, double U1) => (PConic.Accuracy());
}
