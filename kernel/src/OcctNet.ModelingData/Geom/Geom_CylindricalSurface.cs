// Port of: src/ModelingData/TKG3d/Geom/Geom_CylindricalSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_CylindricalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// An infinite cylinder of given radius whose axis is the main axis of its local coordinate system.
/// P(U, V) = O + R * (cos(U) * XDir + sin(U) * YDir) + V * ZDir; U in [0, 2*PI], V infinite.
/// </summary>
public class Geom_CylindricalSurface : Geom_ElementarySurface
{
    private double radius;

    /// <summary>A cylinder of radius Radius whose local coordinate system is A3.</summary>
    /// <exception cref="ConstructionErrorException">Radius &lt; 0.</exception>
    public Geom_CylindricalSurface(in CoordinateSystem A3, double Radius)
    {
        radius = Radius;
        if (Radius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        pos = A3;
    }

    /// <summary>A cylinder from a gp cylinder.</summary>
    public Geom_CylindricalSurface(in Cylinder C)
    {
        radius = C.Radius();
        pos = C.Position();
    }

    public sealed override Geom_Geometry Copy()
    {
        Geom_CylindricalSurface Cs;
        Cs = new Geom_CylindricalSurface(pos, radius);
        return Cs;
    }

    /// <summary>2*PI - U.</summary>
    public sealed override double UReversedParameter(double U) => (2.0 * M_PI - U);

    /// <summary>-V.</summary>
    public sealed override double VReversedParameter(double V) => (-V);

    /// <summary>The radius.</summary>
    public double Radius() => radius;

    /// <summary>True.</summary>
    public sealed override bool IsUClosed() => true;

    /// <summary>False.</summary>
    public sealed override bool IsVClosed() => false;

    /// <summary>True (period 2*PI).</summary>
    public sealed override bool IsUPeriodic() => true;

    /// <summary>False.</summary>
    public sealed override bool IsVPeriodic() => false;

    internal void SetCylinder(in Cylinder C)
    {
        CheckMutable();
        radius = C.Radius();
        pos = C.Position();
    }

    /// <exception cref="ConstructionErrorException">R &lt; 0.</exception>
    internal void SetRadius(double R)
    {
        CheckMutable();
        if (R < 0.0)
        {
            throw new ConstructionErrorException();
        }
        radius = R;
    }

    /// <summary>A copy that is the cylinder C.</summary>
    public Geom_CylindricalSurface WithCylinder(in Cylinder C)
    {
        Geom_CylindricalSurface aCopy = (Geom_CylindricalSurface)Copy();
        aCopy.SetCylinder(C);
        return aCopy;
    }

    /// <summary>A copy of radius R.</summary>
    /// <exception cref="ConstructionErrorException">R &lt; 0.</exception>
    public Geom_CylindricalSurface WithRadius(double R)
    {
        Geom_CylindricalSurface aCopy = (Geom_CylindricalSurface)Copy();
        aCopy.SetRadius(R);
        return aCopy;
    }

    /// <summary>U in [0, 2*PI], V infinite.</summary>
    public sealed override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = 0.0;
        U2 = 2.0 * M_PI;
        V1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        V2 = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>
    /// The coefficients of the implicit equation in the absolute system:
    /// A1.X**2 + A2.Y**2 + A3.Z**2 + 2.(B1.X.Y + B2.X.Z + B3.Y.Z) + 2.(C1.X + C2.Y + C3.Z) + D = 0.
    /// </summary>
    public void Coefficients(out double A1,
                             out double A2,
                             out double A3,
                             out double B1,
                             out double B2,
                             out double B3,
                             out double C1,
                             out double C2,
                             out double C3,
                             out double D)
    {
        Transform3D T = new Transform3D();
        T.SetTransformation(pos);
        double T11 = T.Value(1, 1);
        double T12 = T.Value(1, 2);
        double T13 = T.Value(1, 3);
        double T14 = T.Value(1, 4);
        double T21 = T.Value(2, 1);
        double T22 = T.Value(2, 2);
        double T23 = T.Value(2, 3);
        double T24 = T.Value(2, 4);
        A1 = T11 * T11 + T21 * T21;
        A2 = T12 * T12 + T22 * T22;
        A3 = T13 * T13 + T23 * T23;
        B1 = T11 * T12 + T21 * T22;
        B2 = T11 * T13 + T21 * T23;
        B3 = T12 * T13 + T22 * T23;
        C1 = T11 * T14 + T21 * T24;
        C2 = T12 * T14 + T22 * T24;
        C3 = T13 * T14 + T23 * T24;
        D = T14 * T14 + T24 * T24 - radius * radius;
    }

    /// <summary>The non-persistent gp cylinder with the same geometry.</summary>
    public Cylinder Cylinder() => new Cylinder(pos, radius);

    public sealed override Point3D EvalD0(double U, double V)
    {
        Point3D aP;
        global::Kerf.Engine.Geometry.ElSLib.CylinderD0(U, V, pos, radius, out aP);
        return aP;
    }

    public sealed override ResD1 EvalD1(double U, double V)
    {
        ResD1 aResult;
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(U, V, pos, radius, out aResult.Point, out aResult.D1U, out aResult.D1V);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U, double V)
    {
        ResD2 aResult;
        global::Kerf.Engine.Geometry.ElSLib.CylinderD2(U,
                          V,
                          pos,
                          radius,
                          out aResult.Point,
                          out aResult.D1U,
                          out aResult.D1V,
                          out aResult.D2U,
                          out aResult.D2V,
                          out aResult.D2UV);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U, double V)
    {
        ResD3 aResult;
        global::Kerf.Engine.Geometry.ElSLib.CylinderD3(U,
                          V,
                          pos,
                          radius,
                          out aResult.Point,
                          out aResult.D1U,
                          out aResult.D1V,
                          out aResult.D2U,
                          out aResult.D2V,
                          out aResult.D2UV,
                          out aResult.D3U,
                          out aResult.D3V,
                          out aResult.D3UUV,
                          out aResult.D3UVV);
        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    public sealed override Vector3D EvalDN(double U, double V, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            throw new Geom_UndefinedDerivative();
        }
        if (Nv > 1)
        {
            return new Vector3D(0.0, 0.0, 0.0);
        }
        else
        {
            return global::Kerf.Engine.Geometry.ElSLib.CylinderDN(U, V, pos, radius, Nu, Nv);
        }
    }

    /// <summary>The U iso-parametric line (a Geom_Line).</summary>
    public sealed override Geom_Curve UIso(double U)
    {
        Geom_Line GL = new Geom_Line(global::Kerf.Engine.Geometry.ElSLib.CylinderUIso(pos, radius, U));
        return GL;
    }

    /// <summary>The V iso-parametric circle (a Geom_Circle).</summary>
    public sealed override Geom_Curve VIso(double V)
    {
        Geom_Circle GC = new Geom_Circle(global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(pos, radius, V));
        return GC;
    }

    protected internal sealed override void Transform(in Transform3D T)
    {
        CheckMutable();
        radius = radius * Math.Abs(T.ScaleFactor());
        pos.Transform(T);
    }

    /// <summary>V scaled by |T.ScaleFactor()| (unless infinite); U unchanged.</summary>
    public sealed override void TransformParameters(ref double U, ref double V, in Transform3D T)
    {
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(V))
        {
            V *= Math.Abs(T.ScaleFactor());
        }
    }

    /// <summary>An affinity of ratio |T.ScaleFactor()| along V.</summary>
    public sealed override GeneralTransform2D ParametricTransformation(in Transform3D T)
    {
        GeneralTransform2D T2 = new GeneralTransform2D();
        Axis2D Axis = new Axis2D(Constants.Origin2d(), Constants.DX2d());
        T2.SetAffinity(Axis, Math.Abs(T.ScaleFactor()));
        return T2;
    }
}
