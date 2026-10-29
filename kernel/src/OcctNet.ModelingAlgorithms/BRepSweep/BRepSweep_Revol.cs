// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Revol.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Revol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Builds the rotational sweep of a shape around an axis, by an angle or a whole turn.</summary>
internal sealed class BRepSweep_Revol
{
    private readonly BRepSweep_Rotation myRotation;

    /// <summary>The revolution of <paramref name="S"/> around <paramref name="Ax"/> by the angle <paramref name="D"/>.</summary>
    public BRepSweep_Revol(in TopoDS_Shape S, in gp_Ax1 Ax, double D, bool C = false)
    {
        myRotation = new BRepSweep_Rotation(S.Oriented(TopAbs_FORWARD), NumShape(D), Location(Ax, D), Axe(Ax, D), Angle(D), C);
        Standard_ConstructionError_Raise_if(Angle(D) <= Precision.Angular(), "BRepSweep_Revol::Constructor");
    }

    /// <summary>The whole revolution of <paramref name="S"/> around <paramref name="Ax"/>.</summary>
    public BRepSweep_Revol(in TopoDS_Shape S, in gp_Ax1 Ax, bool C = false)
    {
        myRotation = new BRepSweep_Rotation(S.Oriented(TopAbs_FORWARD), NumShape(2 * Math.PI), Location(Ax, 2 * Math.PI),
                                            Axe(Ax, 2 * Math.PI), Angle(2 * Math.PI), C);
    }

    public TopoDS_Shape Shape() => myRotation.Shape();

    public TopoDS_Shape Shape(in TopoDS_Shape aGenS) => myRotation.Shape(aGenS);

    public TopoDS_Shape FirstShape() => myRotation.FirstShape();

    public TopoDS_Shape FirstShape(in TopoDS_Shape aGenS) => myRotation.FirstShape(aGenS);

    public TopoDS_Shape LastShape() => myRotation.LastShape();

    public TopoDS_Shape LastShape(in TopoDS_Shape aGenS) => myRotation.LastShape(aGenS);

    private static Sweep_NumShape NumShape(double D)
    {
        var N = new Sweep_NumShape();
        if (Math.Abs(Angle(D) - 2 * Math.PI) <= Precision.Angular())
        {
            N.Init(2, TopAbs_EDGE, true, false, false);
        }
        else
        {
            N.Init(2, TopAbs_EDGE);
        }
        return N;
    }

    private static TopLoc_Location Location(in gp_Ax1 Ax, double D)
    {
        var gpt = new gp_Trsf();
        gpt.SetRotation(Axe(Ax, D), Angle(D));
        var L = new TopLoc_Location(gpt);
        return L;
    }

    private static gp_Ax1 Axe(in gp_Ax1 Ax, double D)
    {
        gp_Ax1 A = Ax;
        if (D < 0.0)
        {
            A.Reverse();
        }
        return A;
    }

    private static double Angle(double D)
    {
        double d = Math.Abs(D);
        while (d > (2 * Math.PI + Precision.Angular()))
        {
            d = d - 2 * Math.PI;
        }
        return d;
    }

    /// <summary>The angle of the revolution.</summary>
    public double Angle() => myRotation.Angle();

    /// <summary>The axis of the revolution.</summary>
    public gp_Ax1 Axe() => myRotation.Axe();

    /// <summary>True if the initial shape aGenS is used in the result shape.</summary>
    public bool IsUsed(in TopoDS_Shape aGenS) => myRotation.IsUsed(aGenS);
}
