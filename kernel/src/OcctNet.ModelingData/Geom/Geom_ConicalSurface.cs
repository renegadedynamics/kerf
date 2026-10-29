// Port of: src/ModelingData/TKG3d/Geom/Geom_ConicalSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_ConicalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An infinite cone whose axis is the main axis of its local coordinate system, defined by its semi-angle and
/// its radius in the reference plane (the XOY plane of the local system).
/// P(U, V) = O + (R + V * sin(Ang)) * (cos(U) * XDir + sin(U) * YDir) + V * cos(Ang) * ZDir.
/// </summary>
public class Geom_ConicalSurface : Geom_ElementarySurface
{
    private double radius;
    private double semiAngle;

    /// <summary>A cone of semi-angle Ang and reference radius Radius in the local coordinate system A3.</summary>
    /// <exception cref="Standard_ConstructionError">Radius &lt; 0, or |Ang| &lt;= Resolution, or |Ang| &gt;= PI/2 - Resolution.</exception>
    public Geom_ConicalSurface(in gp_Ax3 A3, double Ang, double Radius)
    {
        radius = Radius;
        semiAngle = Ang;
        if (Radius < 0.0 || Math.Abs(Ang) <= gp.Resolution()
            || Math.Abs(Ang) >= M_PI / 2.0 - gp.Resolution())
        {
            throw new Standard_ConstructionError();
        }
        pos = A3;
    }

    /// <summary>A cone from a gp cone.</summary>
    public Geom_ConicalSurface(in gp_Cone C)
    {
        radius = C.RefRadius();
        semiAngle = C.SemiAngle();
        pos = C.Position();
    }

    public sealed override Geom_Geometry Copy()
    {
        Geom_ConicalSurface Cs;
        Cs = new Geom_ConicalSurface(pos, semiAngle, radius);
        return Cs;
    }

    /// <summary>2*PI - U.</summary>
    public sealed override double UReversedParameter(double U) => (2.0 * M_PI - U);

    /// <summary>-V.</summary>
    public sealed override double VReversedParameter(double V) => (-V);

    /// <summary>Reverses the V direction: the main direction and the semi-angle change sign.</summary>
    protected internal sealed override void VReverse()
    {
        CheckMutable();
        semiAngle = -semiAngle;
        pos.ZReverse();
    }

    /// <summary>The reference radius (in the XOY plane of the local system).</summary>
    public double RefRadius() => radius;

    /// <summary>The semi-angle, in ]-PI/2, PI/2[, non-zero.</summary>
    public double SemiAngle() => semiAngle;

    /// <summary>True.</summary>
    public sealed override bool IsUClosed() => true;

    /// <summary>False.</summary>
    public sealed override bool IsVClosed() => false;

    /// <summary>True (period 2*PI).</summary>
    public sealed override bool IsUPeriodic() => true;

    /// <summary>False.</summary>
    public sealed override bool IsVPeriodic() => false;

    /// <summary>The non-persistent gp cone with the same geometry.</summary>
    public gp_Cone Cone() => new gp_Cone(pos, semiAngle, radius);

    internal void SetCone(in gp_Cone C)
    {
        CheckMutable();
        radius = C.RefRadius();
        semiAngle = C.SemiAngle();
        pos = C.Position();
    }

    /// <exception cref="Standard_ConstructionError">R &lt; 0.</exception>
    internal void SetRadius(double R)
    {
        CheckMutable();
        if (R < 0.0)
        {
            throw new Standard_ConstructionError();
        }
        radius = R;
    }

    /// <exception cref="Standard_ConstructionError">|Ang| &lt;= Resolution or |Ang| &gt;= PI/2 - Resolution.</exception>
    internal void SetSemiAngle(double Ang)
    {
        CheckMutable();
        if (Math.Abs(Ang) <= gp.Resolution() || Math.Abs(Ang) >= M_PI / 2.0 - gp.Resolution())
        {
            throw new Standard_ConstructionError();
        }
        semiAngle = Ang;
    }

    /// <summary>A copy that is the cone C.</summary>
    public Geom_ConicalSurface WithCone(in gp_Cone C)
    {
        Geom_ConicalSurface aCopy = (Geom_ConicalSurface)Copy();
        aCopy.SetCone(C);
        return aCopy;
    }

    /// <summary>A copy of reference radius R.</summary>
    /// <exception cref="Standard_ConstructionError">R &lt; 0.</exception>
    public Geom_ConicalSurface WithRadius(double R)
    {
        Geom_ConicalSurface aCopy = (Geom_ConicalSurface)Copy();
        aCopy.SetRadius(R);
        return aCopy;
    }

    /// <summary>A copy of semi-angle Ang.</summary>
    /// <exception cref="Standard_ConstructionError">|Ang| &lt;= Resolution or |Ang| &gt;= PI/2 - Resolution.</exception>
    public Geom_ConicalSurface WithSemiAngle(double Ang)
    {
        Geom_ConicalSurface aCopy = (Geom_ConicalSurface)Copy();
        aCopy.SetSemiAngle(Ang);
        return aCopy;
    }

    /// <summary>The apex of the cone, on its axis, on the side where the radius decreases.</summary>
    public gp_Pnt Apex()
    {
        gp_XYZ Coord = Position().Direction().XYZ();
        Coord.Multiply(-radius / Math.Tan(semiAngle));
        Coord.Add(Position().Location().XYZ());
        return new gp_Pnt(Coord);
    }

    /// <summary>U in [0, 2*PI], V infinite.</summary>
    public sealed override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = 0.0;
        U2 = 2.0 * M_PI;
        V1 = -Precision.Infinite();
        V2 = Precision.Infinite();
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
        gp_Trsf T = new gp_Trsf();
        T.SetTransformation(pos);
        double KAng = Math.Tan(semiAngle);
        double T11 = T.Value(1, 1);
        double T12 = T.Value(1, 2);
        double T13 = T.Value(1, 3);
        double T14 = T.Value(1, 4);
        double T21 = T.Value(2, 1);
        double T22 = T.Value(2, 2);
        double T23 = T.Value(2, 3);
        double T24 = T.Value(2, 4);
        double T31 = T.Value(3, 1) * KAng;
        double T32 = T.Value(3, 2) * KAng;
        double T33 = T.Value(3, 3) * KAng;
        double T34 = T.Value(3, 4) * KAng;
        A1 = T11 * T11 + T21 * T21 - T31 * T31;
        A2 = T12 * T12 + T22 * T22 - T32 * T32;
        A3 = T13 * T13 + T23 * T23 - T33 * T33;
        B1 = T11 * T12 + T21 * T22 - T31 * T32;
        B2 = T11 * T13 + T21 * T23 - T31 * T33;
        B3 = T12 * T13 + T22 * T23 - T32 * T33;
        // OCCT: C1-C3 and D are wrong unless the local origin is the global one and the radius is 0 (gp_Cone's
        // Coefficients has -T31 * (radius + T34) and -2.0 * radius * T34); kept as OCCT computes them.
        C1 = T11 * T14 + T21 * T24 + radius * T31;
        C2 = T12 * T14 + T22 * T24 + radius * T32;
        C3 = T13 * T14 + T23 * T24 + radius * T33;
        D = T14 * T14 + T24 * T24 - radius * radius - T34 * T34 + 2.0 * radius * T34;
    }

    public sealed override gp_Pnt EvalD0(double U, double V) => ElSLib.ConeValue(U, V, pos, radius, semiAngle);

    public sealed override ResD1 EvalD1(double U, double V)
    {
        ResD1 aResult;
        ElSLib.ConeD1(U, V, pos, radius, semiAngle, out aResult.Point, out aResult.D1U, out aResult.D1V);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U, double V)
    {
        ResD2 aResult;
        ElSLib.ConeD2(U,
                      V,
                      pos,
                      radius,
                      semiAngle,
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
        ElSLib.ConeD3(U,
                      V,
                      pos,
                      radius,
                      semiAngle,
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
    public sealed override gp_Vec EvalDN(double U, double V, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            throw new Geom_UndefinedDerivative();
        }
        if (Nv > 1)
        {
            return new gp_Vec(0.0, 0.0, 0.0);
        }
        else
        {
            return ElSLib.ConeDN(U, V, pos, radius, semiAngle, Nu, Nv);
        }
    }

    /// <summary>The U iso-parametric line (a Geom_Line).</summary>
    public sealed override Geom_Curve UIso(double U)
    {
        Geom_Line GL = new Geom_Line(ElSLib.ConeUIso(pos, radius, semiAngle, U));
        return GL;
    }

    /// <summary>The V iso-parametric circle (a Geom_Circle).</summary>
    public sealed override Geom_Curve VIso(double V)
    {
        Geom_Circle GC = new Geom_Circle(ElSLib.ConeVIso(pos, radius, semiAngle, V));
        return GC;
    }

    protected internal sealed override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        radius = radius * Math.Abs(T.ScaleFactor());
        pos.Transform(T);
    }

    /// <summary>V scaled by |T.ScaleFactor()| (unless infinite); U unchanged.</summary>
    public sealed override void TransformParameters(ref double U, ref double V, in gp_Trsf T)
    {
        if (!Precision.IsInfinite(V))
        {
            V *= Math.Abs(T.ScaleFactor());
        }
    }

    /// <summary>An affinity of ratio |T.ScaleFactor()| along V.</summary>
    public sealed override gp_GTrsf2d ParametricTransformation(in gp_Trsf T)
    {
        gp_GTrsf2d T2 = new gp_GTrsf2d();
        gp_Ax2d Axis = new gp_Ax2d(gp.Origin2d(), gp.DX2d());
        T2.SetAffinity(Axis, Math.Abs(T.ScaleFactor()));
        return T2;
    }
}
