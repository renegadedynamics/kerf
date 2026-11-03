// Port of: src/ModelingData/TKG3d/Geom/Geom_Plane.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Plane.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// An infinite plane, located in 3D space by a coordinate system (gp_Ax3) whose origin is the origin of the
/// parameters and whose X and Y directions are the U and V parametric directions. The main direction is the
/// normal. P(U, V) = O + U * XDir + V * YDir.
/// </summary>
public class Geom_Plane : Geom_ElementarySurface
{
    /// <summary>A plane located by A3: its origin, XDirection and YDirection, and normal.</summary>
    public Geom_Plane(in CoordinateSystem A3)
    {
        pos = A3;
    }

    /// <summary>A plane from a gp plane.</summary>
    public Geom_Plane(in Plane3D Pl)
    {
        pos = Pl.Position();
    }

    /// <summary>A plane through P, normal to V; X and Y directions computed as by gp_Pln.</summary>
    public Geom_Plane(in Point3D P, in Direction3D V)
    {
        Plane3D Pl = new Plane3D(P, V);
        pos = Pl.Position();
    }

    /// <summary>The plane A * X + B * Y + C * Z + D = 0.</summary>
    /// <exception cref="ConstructionErrorException">Sqrt(A*A + B*B + C*C) &lt;= gp::Resolution().</exception>
    public Geom_Plane(double A, double B, double C, double D)
    {
        Plane3D Pl = new Plane3D(A, B, C, D);
        pos = Pl.Position();
    }

    internal void SetPln(in Plane3D Pl)
    {
        CheckMutable();
        pos = Pl.Position();
    }

    /// <summary>A copy that is the plane Pl.</summary>
    public Geom_Plane WithPln(in Plane3D Pl)
    {
        Geom_Plane aCopy = (Geom_Plane)Copy();
        aCopy.SetPln(Pl);
        return aCopy;
    }

    /// <summary>The non-persistent gp plane with the same geometry.</summary>
    public Plane3D Pln() => new Plane3D(Position());

    /// <summary>Reverses the U direction (the X direction of the local coordinate system).</summary>
    protected internal sealed override void UReverse()
    {
        CheckMutable();
        pos.XReverse();
    }

    /// <summary>-U.</summary>
    public sealed override double UReversedParameter(double U) => (-U);

    /// <summary>Reverses the V direction (the Y direction of the local coordinate system).</summary>
    protected internal sealed override void VReverse()
    {
        CheckMutable();
        pos.YReverse();
    }

    /// <summary>-V.</summary>
    public sealed override double VReversedParameter(double V) => (-V);

    protected internal sealed override void Transform(in Transform3D T)
    {
        CheckMutable();
        pos.Transform(T);
    }

    /// <summary>False.</summary>
    public sealed override bool IsUClosed() => false;

    /// <summary>False.</summary>
    public sealed override bool IsVClosed() => false;

    /// <summary>False.</summary>
    public sealed override bool IsUPeriodic() => false;

    /// <summary>False.</summary>
    public sealed override bool IsVPeriodic() => false;

    /// <summary>Infinite bounds: U1 = V1 = -Precision::Infinite(), U2 = V2 = Precision::Infinite().</summary>
    public sealed override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        U2 = global::Kerf.Engine.Foundation.Precision.Infinite();
        V1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        V2 = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>The coefficients of the implicit equation A * X + B * Y + C * Z + D = 0 in the absolute system.</summary>
    public void Coefficients(out double A, out double B, out double C, out double D)
    {
        Plane3D Pl = new Plane3D(Position());
        Pl.Coefficients(out A, out B, out C, out D);
    }

    public sealed override Point3D EvalD0(double U, double V) => global::Kerf.Engine.Geometry.ElSLib.PlaneValue(U, V, pos);

    public sealed override ResD1 EvalD1(double U, double V)
    {
        ResD1 aResult;
        global::Kerf.Engine.Geometry.ElSLib.PlaneD1(U, V, pos, out aResult.Point, out aResult.D1U, out aResult.D1V);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U, double V)
    {
        ResD2 aResult;
        global::Kerf.Engine.Geometry.ElSLib.PlaneD1(U, V, pos, out aResult.Point, out aResult.D1U, out aResult.D1V);
        aResult.D2U = new Vector3D(0.0, 0.0, 0.0);
        aResult.D2V = new Vector3D(0.0, 0.0, 0.0);
        aResult.D2UV = new Vector3D(0.0, 0.0, 0.0);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U, double V)
    {
        ResD3 aResult;
        global::Kerf.Engine.Geometry.ElSLib.PlaneD1(U, V, pos, out aResult.Point, out aResult.D1U, out aResult.D1V);
        aResult.D2U = new Vector3D(0.0, 0.0, 0.0);
        aResult.D2V = new Vector3D(0.0, 0.0, 0.0);
        aResult.D2UV = new Vector3D(0.0, 0.0, 0.0);
        aResult.D3U = new Vector3D(0.0, 0.0, 0.0);
        aResult.D3V = new Vector3D(0.0, 0.0, 0.0);
        aResult.D3UUV = new Vector3D(0.0, 0.0, 0.0);
        aResult.D3UVV = new Vector3D(0.0, 0.0, 0.0);
        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    public sealed override Vector3D EvalDN(double U, double V, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            throw new Geom_UndefinedDerivative();
        }
        if (Nu == 0 && Nv == 1)
        {
            return new Vector3D(pos.YDirection());
        }
        else if (Nu == 1 && Nv == 0)
        {
            return new Vector3D(pos.XDirection());
        }
        return new Vector3D(0.0, 0.0, 0.0);
    }

    /// <summary>The U iso-parametric line (a Geom_Line).</summary>
    public sealed override Geom_Curve UIso(double U)
    {
        Geom_Line GL = new Geom_Line(global::Kerf.Engine.Geometry.ElSLib.PlaneUIso(pos, U));
        return GL;
    }

    /// <summary>The V iso-parametric line (a Geom_Line).</summary>
    public sealed override Geom_Curve VIso(double V)
    {
        Geom_Line GL = new Geom_Line(global::Kerf.Engine.Geometry.ElSLib.PlaneVIso(pos, V));
        return GL;
    }

    /// <summary>(U, V) scaled by |T.ScaleFactor()| (infinite values unchanged).</summary>
    public sealed override void TransformParameters(ref double U, ref double V, in Transform3D T)
    {
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(U))
        {
            U *= Math.Abs(T.ScaleFactor());
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(V))
        {
            V *= Math.Abs(T.ScaleFactor());
        }
    }

    /// <summary>A scale of |T.ScaleFactor()| about the parametric origin.</summary>
    public sealed override GeneralTransform2D ParametricTransformation(in Transform3D T)
    {
        Transform2D T2 = new Transform2D();
        T2.SetScale(Constants.Origin2d(), Math.Abs(T.ScaleFactor()));
        return new GeneralTransform2D(T2);
    }

    public sealed override Geom_Geometry Copy()
    {
        Geom_Plane Pl = new Geom_Plane(pos);
        return Pl;
    }
}
