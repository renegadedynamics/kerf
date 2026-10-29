// Port of: src/ModelingData/TKG3d/Geom/Geom_Line.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Line.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// An infinite line, defined by an axis (gp_Ax1): its origin is the origin of the parameter, its unit vector
/// the direction of the parametrization, P(u) = O + u * Dir. The parameter range is ]-infinite, +infinite[.
/// </summary>
public class Geom_Line : Geom_Curve
{
    private gp_Ax1 pos;

    /// <summary>A line on the axis A1.</summary>
    public Geom_Line(in gp_Ax1 A) => pos = A;

    /// <summary>A line equal to L.</summary>
    public Geom_Line(in gp_Lin L) => pos = L.Position();

    /// <summary>A line through P with direction V.</summary>
    public Geom_Line(in gp_Pnt P, in gp_Dir V) => pos = new gp_Ax1(P, V);

    /// <summary>A new, unfrozen line equal to this one.</summary>
    public sealed override Geom_Geometry Copy()
    {
        Geom_Line L;
        L = new Geom_Line(pos);
        return L;
    }

    /// <summary>Reverses the direction of the line.</summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        pos.Reverse();
    }

    /// <summary>-U.</summary>
    public sealed override double ReversedParameter(double U) => (-U);

    internal void SetDirection(in gp_Dir V)
    {
        CheckMutable();
        pos.SetDirection(V);
    }

    internal void SetLin(in gp_Lin L)
    {
        CheckMutable();
        pos = L.Position();
    }

    internal void SetLocation(in gp_Pnt P)
    {
        CheckMutable();
        pos.SetLocation(P);
    }

    internal void SetPosition(in gp_Ax1 A1)
    {
        CheckMutable();
        pos = A1;
    }

    /// <summary>The axis of the line.</summary>
    public gp_Ax1 Position() => pos;

    public sealed override bool IsClosed() => false;

    public sealed override bool IsPeriodic() => false;

    public sealed override GeomAbs_Shape Continuity() => GeomAbs_CN;

    /// <summary>-Precision::Infinite().</summary>
    public sealed override double FirstParameter() => -Precision.Infinite();

    /// <summary>Precision::Infinite().</summary>
    public sealed override double LastParameter() => Precision.Infinite();

    /// <summary>The non-persistent line equal to this one.</summary>
    public gp_Lin Lin() => new gp_Lin(pos);

    public sealed override bool IsCN(int N) => true;

    /// <summary>Applies T to this line.</summary>
    protected internal sealed override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        pos.Transform(T);
    }

    public sealed override gp_Pnt EvalD0(double U) => ElCLib.LineValue(U, pos);

    public sealed override ResD1 EvalD1(double U)
    {
        ResD1 aResult = new ResD1();
        ElCLib.LineD1(U, pos, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        ResD2 aResult = new ResD2();
        ElCLib.LineD1(U, pos, out aResult.Point, out aResult.D1);
        aResult.D2 = new gp_Vec(0.0, 0.0, 0.0);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        ResD3 aResult = new ResD3();
        ElCLib.LineD1(U, pos, out aResult.Point, out aResult.D1);
        aResult.D2 = new gp_Vec(0.0, 0.0, 0.0);
        aResult.D3 = new gp_Vec(0.0, 0.0, 0.0);
        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">N &lt; 1.</exception>
    public sealed override gp_Vec EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom_UndefinedDerivative();
        }
        if (N == 1)
        {
            return new gp_Vec(pos.Direction());
        }
        else
        {
            return new gp_Vec(0.0, 0.0, 0.0);
        }
    }

    /// <summary>U * |T.ScaleFactor()| (U itself when infinite).</summary>
    public sealed override double TransformedParameter(double U, in gp_Trsf T)
    {
        if (Precision.IsInfinite(U))
        {
            return U;
        }
        return U * Math.Abs(T.ScaleFactor());
    }

    /// <summary>|T.ScaleFactor()|.</summary>
    public sealed override double ParametricTransformation(in gp_Trsf T) => Math.Abs(T.ScaleFactor());

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom_Line WithDirection(in gp_Dir V)
    {
        Geom_Line aCopy = (Geom_Line)Copy();
        aCopy.SetDirection(V);
        return aCopy;
    }

    public Geom_Line WithLin(in gp_Lin L)
    {
        Geom_Line aCopy = (Geom_Line)Copy();
        aCopy.SetLin(L);
        return aCopy;
    }

    public Geom_Line WithLocation(in gp_Pnt P)
    {
        Geom_Line aCopy = (Geom_Line)Copy();
        aCopy.SetLocation(P);
        return aCopy;
    }

    public Geom_Line WithPosition(in gp_Ax1 A1)
    {
        Geom_Line aCopy = (Geom_Line)Copy();
        aCopy.SetPosition(A1);
        return aCopy;
    }
}
