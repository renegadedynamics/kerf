// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Revol.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Revol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>Builds the rotational sweep of a shape around an axis, by an angle or a whole turn.</summary>
internal sealed class BRepSweep_Revol
{
    private readonly BRepSweep_Rotation myRotation;

    /// <summary>The revolution of <paramref name="S"/> around <paramref name="Ax"/> by the angle <paramref name="D"/>.</summary>
    public BRepSweep_Revol(in Shape S, in Axis3D Ax, double D, bool C = false)
    {
        myRotation = new BRepSweep_Rotation(S.Oriented(Forward), NumShape(D), Location(Ax, D), Axe(Ax, D), Angle(D), C);
        Standard_ConstructionError_Raise_if(Angle(D) <= global::Kerf.Engine.Foundation.Precision.Angular(), "BRepSweep_Revol::Constructor");
    }

    /// <summary>The whole revolution of <paramref name="S"/> around <paramref name="Ax"/>.</summary>
    public BRepSweep_Revol(in Shape S, in Axis3D Ax, bool C = false)
    {
        myRotation = new BRepSweep_Rotation(S.Oriented(Forward), NumShape(2 * Math.PI), Location(Ax, 2 * Math.PI),
                                            Axe(Ax, 2 * Math.PI), Angle(2 * Math.PI), C);
    }

    public Shape Shape() => myRotation.Shape();

    public Shape Shape(in Shape aGenS) => myRotation.Shape(aGenS);

    public Shape FirstShape() => myRotation.FirstShape();

    public Shape FirstShape(in Shape aGenS) => myRotation.FirstShape(aGenS);

    public Shape LastShape() => myRotation.LastShape();

    public Shape LastShape(in Shape aGenS) => myRotation.LastShape(aGenS);

    private static Sweep_NumShape NumShape(double D)
    {
        var N = new Sweep_NumShape();
        if (Math.Abs(Angle(D) - 2 * Math.PI) <= global::Kerf.Engine.Foundation.Precision.Angular())
        {
            N.Init(2, ShapeType.Edge, true, false, false);
        }
        else
        {
            N.Init(2, ShapeType.Edge);
        }
        return N;
    }

    private static Placement Location(in Axis3D Ax, double D)
    {
        var gpt = new Transform3D();
        gpt.SetRotation(Axe(Ax, D), Angle(D));
        var L = new Placement(gpt);
        return L;
    }

    private static Axis3D Axe(in Axis3D Ax, double D)
    {
        Axis3D A = Ax;
        if (D < 0.0)
        {
            A.Reverse();
        }
        return A;
    }

    private static double Angle(double D)
    {
        double d = Math.Abs(D);
        while (d > (2 * Math.PI + global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            d = d - 2 * Math.PI;
        }
        return d;
    }

    /// <summary>The angle of the revolution.</summary>
    public double Angle() => myRotation.Angle();

    /// <summary>The axis of the revolution.</summary>
    public Axis3D Axe() => myRotation.Axe();

    /// <summary>True if the initial shape aGenS is used in the result shape.</summary>
    public bool IsUsed(in Shape aGenS) => myRotation.IsUsed(aGenS);
}
