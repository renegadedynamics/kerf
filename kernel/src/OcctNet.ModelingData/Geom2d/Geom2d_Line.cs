// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Line.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Line.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// An infinite line in the plane, placed by an axis (gp_Ax2d): its origin is the origin of the parameter and its
/// unit direction the positive sense, so the parameter of a point is its distance along the line from the origin.
/// The parameter range is ]-infinite, +infinite[.
/// </summary>
public class Geom2d_Line : Geom2d_Curve
{
    private gp_Ax2d pos;

    /// <summary>A new, unfrozen line equal to this one.</summary>
    public sealed override Geom2d_Geometry Copy()
    {
        Geom2d_Line L;
        L = new Geom2d_Line(pos);
        return L;
    }

    /// <summary>The line on the axis A: the origin and direction of A give the parametrization.</summary>
    public Geom2d_Line(in gp_Ax2d A) => pos = A;

    /// <summary>The line equal to L.</summary>
    public Geom2d_Line(in gp_Lin2d L) => pos = L.Position();

    /// <summary>The line through P with the direction V.</summary>
    public Geom2d_Line(in gp_Pnt2d P, in gp_Dir2d V) => pos = new gp_Ax2d(P, V);

    internal void SetDirection(in gp_Dir2d V)
    {
        CheckMutable();
        pos.SetDirection(V);
    }

    /// <summary>The unit direction of the line.</summary>
    public gp_Dir2d Direction() => pos.Direction();

    internal void SetLin2d(in gp_Lin2d L)
    {
        CheckMutable();
        pos = L.Position();
    }

    internal void SetLocation(in gp_Pnt2d P)
    {
        CheckMutable();
        pos.SetLocation(P);
    }

    /// <summary>The origin of the line.</summary>
    public gp_Pnt2d Location() => pos.Location();

    internal void SetPosition(in gp_Ax2d A)
    {
        CheckMutable();
        pos = A;
    }

    /// <summary>The axis of the line.</summary>
    public gp_Ax2d Position() => pos;

    /// <summary>The line as a gp_Lin2d.</summary>
    public gp_Lin2d Lin2d() => new gp_Lin2d(pos);

    /// <summary>Reverses the direction of the line.</summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        pos.Reverse();
    }

    /// <summary>-U.</summary>
    public sealed override double ReversedParameter(double U) => (-U);

    /// <summary>-Precision::Infinite().</summary>
    public sealed override double FirstParameter() => -Precision.Infinite();

    /// <summary>Precision::Infinite().</summary>
    public sealed override double LastParameter() => Precision.Infinite();

    /// <summary>False.</summary>
    public sealed override bool IsClosed() => false;

    /// <summary>False.</summary>
    public sealed override bool IsPeriodic() => false;

    /// <summary>CN.</summary>
    public sealed override GeomAbs_Shape Continuity() => GeomAbs_CN;

    /// <summary>The distance between P and the line.</summary>
    public double Distance(in gp_Pnt2d P)
    {
        gp_Lin2d L = new gp_Lin2d(pos);
        return L.Distance(P);
    }

    /// <summary>True.</summary>
    public sealed override bool IsCN(int N) => true;

    public sealed override gp_Pnt2d EvalD0(double U) => ElCLib.LineValue(U, pos);

    public sealed override ResD1 EvalD1(double U)
    {
        ResD1 aResult;
        ElCLib.LineD1(U, pos, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        ResD2 aResult = new ResD2();
        ElCLib.LineD1(U, pos, out aResult.Point, out aResult.D1);
        aResult.D2.SetCoord(0.0, 0.0);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        ResD3 aResult = new ResD3();
        ElCLib.LineD1(U, pos, out aResult.Point, out aResult.D1);
        aResult.D2.SetCoord(0.0, 0.0);
        aResult.D3.SetCoord(0.0, 0.0);
        return aResult;
    }

    /// <summary>The direction for N = 1, a null vector above.</summary>
    /// <exception cref="Geom2d_UndefinedDerivative">N &lt; 1.</exception>
    public sealed override gp_Vec2d EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom2d_UndefinedDerivative();
        }
        if (N == 1)
        {
            return new gp_Vec2d(pos.Direction());
        }
        else
        {
            return new gp_Vec2d(0.0, 0.0);
        }
    }

    /// <summary>Applies T to the line's axis.</summary>
    protected internal sealed override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        pos.Transform(T);
    }

    /// <summary>U * |T.ScaleFactor()|; an infinite U is returned unchanged.</summary>
    public sealed override double TransformedParameter(double U, in gp_Trsf2d T)
    {
        if (Precision.IsInfinite(U))
        {
            return U;
        }
        return U * Math.Abs(T.ScaleFactor());
    }

    /// <summary>|T.ScaleFactor()|.</summary>
    public sealed override double ParametricTransformation(in gp_Trsf2d T) => Math.Abs(T.ScaleFactor());

    // Non-mutating twins of the Set methods (OCCT.NET)

    /// <summary>A copy equal to L.</summary>
    public Geom2d_Line WithLin2d(in gp_Lin2d L)
    {
        Geom2d_Line aCopy = (Geom2d_Line)Copy();
        aCopy.SetLin2d(L);
        return aCopy;
    }

    /// <summary>A copy with the direction V.</summary>
    public Geom2d_Line WithDirection(in gp_Dir2d V)
    {
        Geom2d_Line aCopy = (Geom2d_Line)Copy();
        aCopy.SetDirection(V);
        return aCopy;
    }

    /// <summary>A copy with the origin P.</summary>
    public Geom2d_Line WithLocation(in gp_Pnt2d P)
    {
        Geom2d_Line aCopy = (Geom2d_Line)Copy();
        aCopy.SetLocation(P);
        return aCopy;
    }

    /// <summary>A copy on the axis A.</summary>
    public Geom2d_Line WithPosition(in gp_Ax2d A)
    {
        Geom2d_Line aCopy = (Geom2d_Line)Copy();
        aCopy.SetPosition(A);
        return aCopy;
    }
}
