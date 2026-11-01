// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_Point.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_Point.lxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_Point.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A point of a blend's walking line: the section's extremities on the two supports (surfaces, curves or
/// restrictions), their parameters, the guide's parameter and, unless it is a tangency point, the tangents.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. <c>new Blend_Point()</c> is a tangency point, as OCCT's default constructor makes
/// it; <c>default</c> is not (its tangency flag is false). OCCT leaves the fields a constructor doesn't set
/// uninitialised; here they are zero.
/// </remarks>
public partial struct Blend_Point : IEquatable<Blend_Point>
{
    private gp_Pnt pt1;
    private gp_Pnt pt2;
    private gp_Vec tg1;
    private gp_Vec tg2;
    private double prm;
    private double u1;
    private double v1;
    private double u2;
    private double v2;
    private double pc1;
    private double pc2;
    private double utg12d;
    private double vtg12d;
    private double utg22d;
    private double vtg22d;
    private bool hass1;
    private bool hass2;
    private bool hasc1;
    private bool hasc2;
    private bool istgt;

    /// <summary>An empty tangency point.</summary>
    public Blend_Point()
    {
        istgt = true;
    }

    /// <summary>A point on two surfaces with its tangents.</summary>
    public Blend_Point(in gp_Pnt P1, in gp_Pnt P2, double Param, double U1, double V1, double U2, double V2,
                       in gp_Vec Tg1, in gp_Vec Tg2, in gp_Vec2d Tg12d, in gp_Vec2d Tg22d)
    {
        pt1 = P1;
        pt2 = P2;
        tg1 = Tg1;
        tg2 = Tg2;
        prm = Param;
        u1 = U1;
        v1 = V1;
        u2 = U2;
        v2 = V2;
        utg12d = Tg12d.X();
        vtg12d = Tg12d.Y();
        utg22d = Tg22d.X();
        vtg22d = Tg22d.Y();
        hass1 = true;
        hass2 = true;
        hasc1 = false;
        hasc2 = false;
        istgt = false;
    }

    /// <summary>A tangency point on two surfaces.</summary>
    public Blend_Point(in gp_Pnt P1, in gp_Pnt P2, double Param, double U1, double V1, double U2, double V2)
    {
        pt1 = P1;
        pt2 = P2;
        prm = Param;
        u1 = U1;
        v1 = V1;
        u2 = U2;
        v2 = V2;
        hass1 = true;
        hass2 = true;
        hasc1 = false;
        hasc2 = false;
        istgt = true;
    }

    /// <summary>A point on a surface and a curve with its tangents.</summary>
    public Blend_Point(in gp_Pnt Ps, in gp_Pnt Pc, double Param, double U, double V, double W, in gp_Vec Tgs,
                       in gp_Vec Tgc, in gp_Vec2d Tg2d)
    {
        pt1 = Ps;
        pt2 = Pc;
        tg1 = Tgs;
        tg2 = Tgc;
        prm = Param;
        u1 = U;
        v1 = V;
        pc2 = W;
        utg12d = Tg2d.X();
        vtg12d = Tg2d.Y();
        hass1 = true;
        hass2 = false;
        hasc1 = false;
        hasc2 = true;
        istgt = false;
    }

    /// <summary>A tangency point on a surface and a curve.</summary>
    public Blend_Point(in gp_Pnt Ps, in gp_Pnt Pc, double Param, double U, double V, double W)
    {
        pt1 = Ps;
        pt2 = Pc;
        prm = Param;
        u1 = U;
        v1 = V;
        pc2 = W;
        hass1 = true;
        hass2 = false;
        hasc1 = false;
        hasc2 = true;
        istgt = true;
    }

    /// <summary>A point on a surface and a restriction with its tangents.</summary>
    public Blend_Point(in gp_Pnt P1, in gp_Pnt P2, double Param, double U1, double V1, double U2, double V2,
                       double PC, in gp_Vec Tg1, in gp_Vec Tg2, in gp_Vec2d Tg12d, in gp_Vec2d Tg22d)
    {
        pt1 = P1;
        pt2 = P2;
        tg1 = Tg1;
        tg2 = Tg2;
        prm = Param;
        u1 = U1;
        v1 = V1;
        u2 = U2;
        v2 = V2;
        pc2 = PC;
        utg12d = Tg12d.X();
        vtg12d = Tg12d.Y();
        utg22d = Tg22d.X();
        vtg22d = Tg22d.Y();
        hass1 = true;
        hass2 = true;
        hasc1 = false;
        hasc2 = true;
        istgt = false;
    }

    /// <summary>A tangency point on a surface and a restriction.</summary>
    public Blend_Point(in gp_Pnt P1, in gp_Pnt P2, double Param, double U1, double V1, double U2, double V2,
                       double PC)
    {
        pt1 = P1;
        pt2 = P2;
        prm = Param;
        u1 = U1;
        v1 = V1;
        u2 = U2;
        v2 = V2;
        pc2 = PC;
        hass1 = true;
        hass2 = true;
        hasc1 = false;
        hasc2 = true;
        istgt = true;
    }

    /// <summary>A point on two restrictions with its tangents.</summary>
    public Blend_Point(in gp_Pnt P1, in gp_Pnt P2, double Param, double U1, double V1, double U2, double V2,
                       double PC1, double PC2, in gp_Vec Tg1, in gp_Vec Tg2, in gp_Vec2d Tg12d, in gp_Vec2d Tg22d)
    {
        pt1 = P1;
        pt2 = P2;
        tg1 = Tg1;
        tg2 = Tg2;
        prm = Param;
        u1 = U1;
        v1 = V1;
        u2 = U2;
        v2 = V2;
        pc1 = PC1;
        pc2 = PC2;
        utg12d = Tg12d.X();
        vtg12d = Tg12d.Y();
        utg22d = Tg22d.X();
        vtg22d = Tg22d.Y();
        hass1 = true;
        hass2 = true;
        hasc1 = true;
        hasc2 = true;
        istgt = false;
    }

    /// <summary>A tangency point on two restrictions.</summary>
    public Blend_Point(in gp_Pnt P1, in gp_Pnt P2, double Param, double U1, double V1, double U2, double V2,
                       double PC1, double PC2)
    {
        pt1 = P1;
        pt2 = P2;
        prm = Param;
        u1 = U1;
        v1 = V1;
        u2 = U2;
        v2 = V2;
        pc1 = PC1;
        pc2 = PC2;
        hass1 = true;
        hass2 = true;
        hasc1 = true;
        hasc2 = true;
        istgt = true;
    }

    internal void SetValue(in gp_Pnt P1, in gp_Pnt P2, double Param, double U1, double V1, double U2, double V2,
                           in gp_Vec Tg1, in gp_Vec Tg2, in gp_Vec2d Tg12d, in gp_Vec2d Tg22d)
    {
        pt1 = P1;
        pt2 = P2;
        prm = Param;
        u1 = U1;
        v1 = V1;
        hass1 = true;
        u2 = U2;
        v2 = V2;
        hass2 = true;
        hasc1 = false;
        hasc2 = false;
        istgt = false;
        tg1 = Tg1;
        tg2 = Tg2;
        utg12d = Tg12d.X();
        vtg12d = Tg12d.Y();
        utg22d = Tg22d.X();
        vtg22d = Tg22d.Y();
    }

    internal void SetValue(in gp_Pnt P1, in gp_Pnt P2, double Param, double U1, double V1, double U2, double V2)
    {
        pt1 = P1;
        pt2 = P2;
        prm = Param;
        u1 = U1;
        v1 = V1;
        hass1 = true;
        u2 = U2;
        v2 = V2;
        hass2 = true;
        hasc1 = false;
        hasc2 = false;
        istgt = true;
    }

    internal void SetValue(in gp_Pnt Ps, in gp_Pnt Pc, double Param, double U, double V, double W, in gp_Vec Tgs,
                           in gp_Vec Tgc, in gp_Vec2d Tg2d)
    {
        pt1 = Ps;
        pt2 = Pc;
        prm = Param;
        u1 = U;
        v1 = V;
        hass1 = true;
        hass2 = false;
        hasc1 = false;
        pc2 = W;
        hasc2 = true;
        istgt = false;
        tg1 = Tgs;
        tg2 = Tgc;
        utg12d = Tg2d.X();
        vtg12d = Tg2d.Y();
    }

    internal void SetValue(in gp_Pnt Ps, in gp_Pnt Pc, double Param, double U, double V, double W)
    {
        pt1 = Ps;
        pt2 = Pc;
        prm = Param;
        u1 = U;
        v1 = V;
        hass1 = true;
        hass2 = false;
        hasc1 = false;
        pc2 = W;
        hasc2 = true;
        istgt = true;
    }

    internal void SetValue(in gp_Pnt P1, in gp_Pnt P2, double Param, double U1, double V1, double U2, double V2,
                           double PC, in gp_Vec Tg1, in gp_Vec Tg2, in gp_Vec2d Tg12d, in gp_Vec2d Tg22d)
    {
        pt1 = P1;
        pt2 = P2;
        prm = Param;
        u1 = U1;
        v1 = V1;
        hass1 = true;
        u2 = U2;
        v2 = V2;
        hass2 = true;
        hasc1 = false;
        pc2 = PC;
        hasc2 = true;
        istgt = false;
        tg1 = Tg1;
        tg2 = Tg2;
        utg12d = Tg12d.X();
        vtg12d = Tg12d.Y();
        utg22d = Tg22d.X();
        vtg22d = Tg22d.Y();
    }

    internal void SetValue(in gp_Pnt P1, in gp_Pnt P2, double Param, double U1, double V1, double U2, double V2,
                           double PC)
    {
        pt1 = P1;
        pt2 = P2;
        prm = Param;
        u1 = U1;
        v1 = V1;
        hass1 = true;
        u2 = U2;
        v2 = V2;
        hass2 = true;
        hasc1 = false;
        pc2 = PC;
        hasc2 = true;
        istgt = true;
    }

    internal void SetValue(in gp_Pnt P1, in gp_Pnt P2, double Param, double U1, double V1, double U2, double V2,
                           double PC1, double PC2, in gp_Vec Tg1, in gp_Vec Tg2, in gp_Vec2d Tg12d,
                           in gp_Vec2d Tg22d)
    {
        pt1 = P1;
        pt2 = P2;
        prm = Param;
        u1 = U1;
        v1 = V1;
        hass1 = true;
        u2 = U2;
        v2 = V2;
        hass2 = true;
        pc1 = PC1;
        hasc1 = true;
        pc2 = PC2;
        hasc2 = true;
        istgt = false;
        tg1 = Tg1;
        tg2 = Tg2;
        utg12d = Tg12d.X();
        vtg12d = Tg12d.Y();
        utg22d = Tg22d.X();
        vtg22d = Tg22d.Y();
    }

    internal void SetValue(in gp_Pnt P1, in gp_Pnt P2, double Param, double U1, double V1, double U2, double V2,
                           double PC1, double PC2)
    {
        pt1 = P1;
        pt2 = P2;
        prm = Param;
        u1 = U1;
        v1 = V1;
        hass1 = true;
        u2 = U2;
        v2 = V2;
        hass2 = true;
        pc1 = PC1;
        hasc1 = true;
        pc2 = PC2;
        hasc2 = true;
        istgt = true;
    }

    internal void SetValue(in gp_Pnt P1, in gp_Pnt P2, double Param, double PC1, double PC2)
    {
        pt1 = P1;
        pt2 = P2;
        prm = Param;
        hass1 = false;
        hass2 = false;
        pc1 = PC1;
        hasc1 = true;
        pc2 = PC2;
        hasc2 = true;
        istgt = true;
    }

    internal void SetParameter(double Param)
    {
        prm = Param;
    }

    /// <summary>A copy with the guide's parameter <paramref name="Param"/>.</summary>
    public readonly Blend_Point WithParameter(double Param)
    {
        Blend_Point aCopy = this;
        aCopy.SetParameter(Param);
        return aCopy;
    }

    /// <summary>The parameter on the guide.</summary>
    public readonly double Parameter()
    {
        return prm;
    }

    /// <summary>True when the point carries no tangents.</summary>
    public readonly bool IsTangencyPoint()
    {
        return istgt;
    }

    public readonly gp_Pnt PointOnS1()
    {
        return pt1;
    }

    public readonly gp_Pnt PointOnS2()
    {
        return pt2;
    }

    /// <summary>The parameters on the first surface; throws <see cref="Standard_DomainError"/> if there are none.</summary>
    public readonly void ParametersOnS1(out double U1, out double V1)
    {
        if (!hass1)
        {
            throw new Standard_DomainError();
        }

        U1 = u1;
        V1 = v1;
    }

    /// <summary>The parameters on the second surface; throws <see cref="Standard_DomainError"/> if there are none.</summary>
    public readonly void ParametersOnS2(out double U2, out double V2)
    {
        if (!hass2)
        {
            throw new Standard_DomainError();
        }

        U2 = u2;
        V2 = v2;
    }

    public readonly gp_Vec TangentOnS1()
    {
        if (istgt)
        {
            throw new Standard_DomainError();
        }

        return tg1;
    }

    public readonly gp_Vec TangentOnS2()
    {
        if (istgt)
        {
            throw new Standard_DomainError();
        }

        return tg2;
    }

    public readonly gp_Vec2d Tangent2dOnS1()
    {
        if (istgt || !hass1)
        {
            throw new Standard_DomainError();
        }

        return new gp_Vec2d(utg12d, vtg12d);
    }

    public readonly gp_Vec2d Tangent2dOnS2()
    {
        if (istgt || !hass2)
        {
            throw new Standard_DomainError();
        }

        return new gp_Vec2d(utg22d, vtg22d);
    }

    public readonly gp_Pnt PointOnS()
    {
        return pt1;
    }

    public readonly gp_Pnt PointOnC()
    {
        return pt2;
    }

    public readonly void ParametersOnS(out double U1, out double V1)
    {
        if (!hass1)
        {
            throw new Standard_DomainError();
        }

        U1 = u1;
        V1 = v1;
    }

    public readonly double ParameterOnC()
    {
        if (!hasc2)
        {
            throw new Standard_DomainError();
        }

        return pc2;
    }

    public readonly gp_Vec TangentOnS()
    {
        if (istgt || !hass1)
        {
            throw new Standard_DomainError();
        }

        return tg1;
    }

    public readonly gp_Vec TangentOnC()
    {
        if (istgt)
        {
            throw new Standard_DomainError();
        }

        return tg2;
    }

    public readonly gp_Vec2d Tangent2d()
    {
        if (istgt || !hass1)
        {
            throw new Standard_DomainError();
        }

        return new gp_Vec2d(utg12d, vtg12d);
    }

    public readonly gp_Pnt PointOnC1()
    {
        return pt1;
    }

    public readonly gp_Pnt PointOnC2()
    {
        return pt2;
    }

    public readonly double ParameterOnC1()
    {
        if (!hasc1)
        {
            throw new Standard_DomainError();
        }

        return pc1;
    }

    public readonly double ParameterOnC2()
    {
        if (!hasc2)
        {
            throw new Standard_DomainError();
        }

        return pc2;
    }

    public readonly gp_Vec TangentOnC1()
    {
        if (istgt || !hass1)
        {
            throw new Standard_DomainError();
        }

        return tg1;
    }

    public readonly gp_Vec TangentOnC2()
    {
        if (istgt)
        {
            throw new Standard_DomainError();
        }

        return tg2;
    }

    public readonly bool Equals(Blend_Point other)
        => pt1.Equals(other.pt1) && pt2.Equals(other.pt2) && tg1.Equals(other.tg1) && tg2.Equals(other.tg2)
           && prm.Equals(other.prm) && u1.Equals(other.u1) && v1.Equals(other.v1) && u2.Equals(other.u2)
           && v2.Equals(other.v2) && pc1.Equals(other.pc1) && pc2.Equals(other.pc2) && utg12d.Equals(other.utg12d)
           && vtg12d.Equals(other.vtg12d) && utg22d.Equals(other.utg22d) && vtg22d.Equals(other.vtg22d)
           && hass1 == other.hass1 && hass2 == other.hass2 && hasc1 == other.hasc1 && hasc2 == other.hasc2
           && istgt == other.istgt;

    public override readonly bool Equals(object? obj) => obj is Blend_Point other && Equals(other);

    public override readonly int GetHashCode()
    {
        HashCode h = new();
        h.Add(pt1);
        h.Add(pt2);
        h.Add(tg1);
        h.Add(tg2);
        h.Add(prm);
        h.Add(u1);
        h.Add(v1);
        h.Add(u2);
        h.Add(v2);
        h.Add(pc1);
        h.Add(pc2);
        h.Add(istgt);
        return h.ToHashCode();
    }

    public static bool operator ==(in Blend_Point theLeft, in Blend_Point theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Blend_Point theLeft, in Blend_Point theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => gp.Format("Blend_Point", prm, pt1.X(), pt1.Y(), pt1.Z(), pt2.X(), pt2.Y(), pt2.Z(), u1, v1, u2, v2, pc1, pc2);
}
