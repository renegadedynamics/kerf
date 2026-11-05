// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Revol.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Revol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Sweeping;

/// <summary>Builds the rotational sweep of a shape around an axis, by an angle or a whole turn.</summary>
internal sealed class Revol
{
    private readonly Rotation _rotation;

    /// <summary>The revolution of <paramref name="s"/> around <paramref name="ax"/> by the angle <paramref name="d"/>.</summary>
    public Revol(in Shape s, in Axis3D ax, double d, bool c = false)
    {
        _rotation = new Rotation(s.Oriented(Forward), NumShape(d), Location(ax, d), Axe(ax, d), Angle(d), c);
        Standard_ConstructionError_Raise_if(Angle(d) <= global::Kerf.Engine.Foundation.Precision.Angular(), "BRepSweep_Revol::Constructor");
    }

    /// <summary>The whole revolution of <paramref name="s"/> around <paramref name="ax"/>.</summary>
    public Revol(in Shape s, in Axis3D ax, bool c = false)
    {
        _rotation = new Rotation(s.Oriented(Forward), NumShape(2 * Math.PI), Location(ax, 2 * Math.PI),
                                            Axe(ax, 2 * Math.PI), Angle(2 * Math.PI), c);
    }

    public Shape Shape() => _rotation.Shape();

    public Shape Shape(in Shape aGenS) => _rotation.Shape(aGenS);

    public Shape FirstShape() => _rotation.FirstShape();

    public Shape FirstShape(in Shape aGenS) => _rotation.FirstShape(aGenS);

    public Shape LastShape() => _rotation.LastShape();

    public Shape LastShape(in Shape aGenS) => _rotation.LastShape(aGenS);

    private static NumShape NumShape(double d)
    {
        var n = new NumShape();
        if (Math.Abs(Angle(d) - 2 * Math.PI) <= global::Kerf.Engine.Foundation.Precision.Angular())
        {
            n.Init(2, ShapeType.Edge, true, false, false);
        }
        else
        {
            n.Init(2, ShapeType.Edge);
        }
        return n;
    }

    private static Placement Location(in Axis3D ax, double d)
    {
        var gpt = new Transform3D();
        gpt = gpt.WithRotation(Axe(ax, d), Angle(d));
        var l = new Placement(gpt);
        return l;
    }

    private static Axis3D Axe(in Axis3D ax, double d)
    {
        Axis3D a = ax;
        if (d < 0.0)
        {
            a = a.Reversed();
        }
        return a;
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
    public double Angle() => _rotation.Angle();

    /// <summary>The axis of the revolution.</summary>
    public Axis3D Axe() => _rotation.Axe();

    /// <summary>True if the initial shape aGenS is used in the result shape.</summary>
    public bool IsUsed(in Shape aGenS) => _rotation.IsUsed(aGenS);
}
