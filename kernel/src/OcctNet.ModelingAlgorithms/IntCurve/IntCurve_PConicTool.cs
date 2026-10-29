// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_PConicTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_PConicTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>The evaluation of an <see cref="IntCurve_PConic"/> (IntImpParGen's ParTool for conics).</summary>
/// <remarks>A struct without state, so it can be a type argument of the generic intersectors.</remarks>
public readonly struct IntCurve_PConicTool : IIntImpParGen_ParTool<IntCurve_PConic>
{
    public static gp_Pnt2d Value(in IntCurve_PConic PConic, double X)
    {
        switch (PConic.TypeCurve())
        {
            case GeomAbs_Line:
                return (ElCLib.LineValue(X, PConic.Axis2().XAxis()));
            case GeomAbs_Circle:
                return (ElCLib.CircleValue(X, PConic.Axis2(), PConic.Param1()));
            case GeomAbs_Ellipse:
                return (ElCLib.EllipseValue(X, PConic.Axis2(), PConic.Param1(), PConic.Param2()));
            case GeomAbs_Parabola:
                return (ElCLib.ParabolaValue(X, PConic.Axis2(), PConic.Param1()));
            default: //-- case GeomAbs_Hyperbola:
                return (ElCLib.HyperbolaValue(X, PConic.Axis2(), PConic.Param1(), PConic.Param2()));
        }
    }

    public static void D1(in IntCurve_PConic PConic, double U, out gp_Pnt2d Pt, out gp_Vec2d Tan)
    {
        switch (PConic.TypeCurve())
        {
            case GeomAbs_Line:
                ElCLib.LineD1(U, PConic.Axis2().XAxis(), out Pt, out Tan);
                break;
            case GeomAbs_Circle:
                ElCLib.CircleD1(U, PConic.Axis2(), PConic.Param1(), out Pt, out Tan);
                break;
            case GeomAbs_Ellipse:
                ElCLib.EllipseD1(U, PConic.Axis2(), PConic.Param1(), PConic.Param2(), out Pt, out Tan);
                break;
            case GeomAbs_Parabola:
                ElCLib.ParabolaD1(U, PConic.Axis2(), PConic.Param1(), out Pt, out Tan);
                break;
            case GeomAbs_Hyperbola:
                ElCLib.HyperbolaD1(U, PConic.Axis2(), PConic.Param1(), PConic.Param2(), out Pt, out Tan);
                break;
            default:
                // OCCT leaves the outputs unchanged (decision 32: zeros).
                Pt = new gp_Pnt2d(0.0, 0.0);
                Tan = new gp_Vec2d(0.0, 0.0);
                break;
        }
    }

    public static void D2(in IntCurve_PConic PConic, double U, out gp_Pnt2d Pt, out gp_Vec2d Tan, out gp_Vec2d Norm)
    {
        switch (PConic.TypeCurve())
        {
            case GeomAbs_Line:
                // OCCT clears Tan (which LineD1 then writes) and leaves Norm unwritten: its callers' Norm stays
                // as default-constructed, (0, 0), which the port writes.
                Norm = new gp_Vec2d(0.0, 0.0);
                ElCLib.LineD1(U, PConic.Axis2().XAxis(), out Pt, out Tan);
                break;
            case GeomAbs_Circle:
                ElCLib.CircleD2(U, PConic.Axis2(), PConic.Param1(), out Pt, out Tan, out Norm);
                break;
            case GeomAbs_Ellipse:
                ElCLib.EllipseD2(U, PConic.Axis2(), PConic.Param1(), PConic.Param2(), out Pt, out Tan, out Norm);
                break;
            case GeomAbs_Parabola:
                ElCLib.ParabolaD2(U, PConic.Axis2(), PConic.Param1(), out Pt, out Tan, out Norm);
                break;
            case GeomAbs_Hyperbola:
                ElCLib.HyperbolaD2(U, PConic.Axis2(), PConic.Param1(), PConic.Param2(), out Pt, out Tan, out Norm);
                break;
            default:
                // OCCT leaves the outputs unchanged (decision 32: zeros).
                Pt = new gp_Pnt2d(0.0, 0.0);
                Tan = new gp_Vec2d(0.0, 0.0);
                Norm = new gp_Vec2d(0.0, 0.0);
                break;
        }
    }

    public static double EpsX(in IntCurve_PConic PConic) => (PConic.EpsX());

    public static int NbSamples(in IntCurve_PConic PConic) => (PConic.Accuracy());

    public static int NbSamples(in IntCurve_PConic PConic, double U0, double U1) => (PConic.Accuracy());
}
