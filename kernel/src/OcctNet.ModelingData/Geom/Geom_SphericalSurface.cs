// Port of: src/ModelingData/TKG3d/Geom/Geom_SphericalSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_SphericalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A sphere of given radius centred on the origin of its local coordinate system.
/// P(U, V) = O + R * cos(V) * (cos(U) * XDir + sin(U) * YDir) + R * sin(V) * ZDir;
/// U in [0, 2*PI], V in [-PI/2, PI/2].
/// </summary>
public class Geom_SphericalSurface : Geom_ElementarySurface
{
    private double radius;

    /// <summary>A sphere of radius Radius whose local coordinate system is A3.</summary>
    /// <exception cref="Standard_ConstructionError">Radius &lt; 0.</exception>
    public Geom_SphericalSurface(in gp_Ax3 A3, double Radius)
    {
        radius = Radius;
        if (Radius < 0.0)
        {
            throw new Standard_ConstructionError();
        }
        pos = A3;
    }

    /// <summary>A sphere from a gp sphere.</summary>
    public Geom_SphericalSurface(in gp_Sphere S)
    {
        radius = S.Radius();
        pos = S.Position();
    }

    public sealed override Geom_Geometry Copy()
    {
        Geom_SphericalSurface Cs;
        Cs = new Geom_SphericalSurface(pos, radius);
        return Cs;
    }

    /// <summary>2*PI - U.</summary>
    public sealed override double UReversedParameter(double U) => (2.0 * M_PI - U);

    /// <summary>-V.</summary>
    public sealed override double VReversedParameter(double V) => (-V);

    /// <summary>The area, 4 * PI * R * R.</summary>
    public double Area() => 4.0 * M_PI * radius * radius;

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

    internal void SetSphere(in gp_Sphere S)
    {
        CheckMutable();
        radius = S.Radius();
        pos = S.Position();
    }

    /// <summary>A copy of radius R.</summary>
    /// <exception cref="Standard_ConstructionError">R &lt; 0.</exception>
    public Geom_SphericalSurface WithRadius(double R)
    {
        Geom_SphericalSurface aCopy = (Geom_SphericalSurface)Copy();
        aCopy.SetRadius(R);
        return aCopy;
    }

    /// <summary>A copy that is the sphere S.</summary>
    public Geom_SphericalSurface WithSphere(in gp_Sphere S)
    {
        Geom_SphericalSurface aCopy = (Geom_SphericalSurface)Copy();
        aCopy.SetSphere(S);
        return aCopy;
    }

    /// <summary>U in [0, 2*PI], V in [-PI/2, PI/2].</summary>
    public sealed override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = 0.0;
        U2 = M_PI * 2.0;
        V1 = -M_PI / 2.0;
        V2 = M_PI / 2.0;
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
        double T11 = T.Value(1, 1);
        double T12 = T.Value(1, 2);
        double T13 = T.Value(1, 3);
        double T14 = T.Value(1, 4);
        double T21 = T.Value(2, 1);
        double T22 = T.Value(2, 2);
        double T23 = T.Value(2, 3);
        double T24 = T.Value(2, 4);
        double T31 = T.Value(3, 1);
        double T32 = T.Value(3, 2);
        double T33 = T.Value(3, 3);
        double T34 = T.Value(3, 4);
        A1 = T11 * T11 + T21 * T21 + T31 * T31;
        A2 = T12 * T12 + T22 * T22 + T32 * T32;
        A3 = T13 * T13 + T23 * T23 + T33 * T33;
        B1 = T11 * T12 + T21 * T22 + T31 * T32;
        B2 = T11 * T13 + T21 * T23 + T31 * T33;
        B3 = T12 * T13 + T22 * T23 + T32 * T33;
        C1 = T11 * T14 + T21 * T24 + T31 * T34;
        C2 = T12 * T14 + T22 * T24 + T32 * T34;
        C3 = T13 * T14 + T23 * T24 + T33 * T34;
        D = T14 * T14 + T24 * T24 + T34 * T34 - radius * radius;
    }

    public sealed override gp_Pnt EvalD0(double U, double V)
    {
        gp_Pnt aP;
        ElSLib.SphereD0(U, V, pos, radius, out aP);
        return aP;
    }

    public sealed override ResD1 EvalD1(double U, double V)
    {
        ResD1 aResult;
        ElSLib.SphereD1(U, V, pos, radius, out aResult.Point, out aResult.D1U, out aResult.D1V);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U, double V)
    {
        ResD2 aResult;
        ElSLib.SphereD2(U,
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
        ElSLib.SphereD3(U,
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
    public sealed override gp_Vec EvalDN(double U, double V, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            throw new Geom_UndefinedDerivative();
        }
        return ElSLib.SphereDN(U, V, pos, radius, Nu, Nv);
    }

    /// <summary>The non-persistent gp sphere with the same geometry.</summary>
    public gp_Sphere Sphere() => new gp_Sphere(pos, radius);

    /// <summary>The U iso-parametric half circle (a Geom_TrimmedCurve on a Geom_Circle, in [-PI/2, PI/2]).</summary>
    public sealed override Geom_Curve UIso(double U)
    {
        Geom_Circle GC = new Geom_Circle(ElSLib.SphereUIso(pos, radius, U));
        Geom_TrimmedCurve iso = new Geom_TrimmedCurve(GC, -M_PI / 2.0, M_PI / 2);
        return iso;
    }

    /// <summary>The V iso-parametric circle (a Geom_Circle).</summary>
    public sealed override Geom_Curve VIso(double V)
    {
        Geom_Circle GC = new Geom_Circle(ElSLib.SphereVIso(pos, radius, V));
        return GC;
    }

    /// <summary>The volume, 4 * PI * R**3 / 3.</summary>
    public double Volume() => (4.0 * M_PI * radius * radius * radius) / 3.0;

    protected internal sealed override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        radius = radius * Math.Abs(T.ScaleFactor());
        pos.Transform(T);
    }
}
