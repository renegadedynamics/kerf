// Port of: src/ModelingData/TKG3d/Geom/Geom_Line.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Line.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// An infinite line, defined by an axis (gp_Ax1): its origin is the origin of the parameter, its unit vector
/// the direction of the parametrization, P(u) = O + u * Dir. The parameter range is ]-infinite, +infinite[.
/// </summary>
public class Geom_Line : Geom_Curve
{
    private Axis3D pos;

    /// <summary>A line on the axis A1.</summary>
    public Geom_Line(in Axis3D A) => pos = A;

    /// <summary>A line equal to L.</summary>
    public Geom_Line(in Line3D L) => pos = L.Position();

    /// <summary>A line through P with direction V.</summary>
    public Geom_Line(in Point3D P, in Direction3D V) => pos = new Axis3D(P, V);

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

    internal void SetDirection(in Direction3D V)
    {
        CheckMutable();
        pos.SetDirection(V);
    }

    internal void SetLin(in Line3D L)
    {
        CheckMutable();
        pos = L.Position();
    }

    internal void SetLocation(in Point3D P)
    {
        CheckMutable();
        pos.SetLocation(P);
    }

    internal void SetPosition(in Axis3D A1)
    {
        CheckMutable();
        pos = A1;
    }

    /// <summary>The axis of the line.</summary>
    public Axis3D Position() => pos;

    public sealed override bool IsClosed() => false;

    public sealed override bool IsPeriodic() => false;

    public sealed override Continuity Continuity() => CN;

    /// <summary>-Precision::Infinite().</summary>
    public sealed override double FirstParameter() => -global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>Precision::Infinite().</summary>
    public sealed override double LastParameter() => global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>The non-persistent line equal to this one.</summary>
    public Line3D Lin() => new Line3D(pos);

    public sealed override bool IsCN(int N) => true;

    /// <summary>Applies T to this line.</summary>
    protected internal sealed override void Transform(in Transform3D T)
    {
        CheckMutable();
        pos.Transform(T);
    }

    public sealed override Point3D EvalD0(double U) => global::Kerf.Engine.Geometry.ElCLib.LineValue(U, pos);

    public sealed override ResD1 EvalD1(double U)
    {
        ResD1 aResult = new ResD1();
        global::Kerf.Engine.Geometry.ElCLib.LineD1(U, pos, out aResult.Point, out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        ResD2 aResult = new ResD2();
        global::Kerf.Engine.Geometry.ElCLib.LineD1(U, pos, out aResult.Point, out aResult.D1);
        aResult.D2 = new Vector3D(0.0, 0.0, 0.0);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        ResD3 aResult = new ResD3();
        global::Kerf.Engine.Geometry.ElCLib.LineD1(U, pos, out aResult.Point, out aResult.D1);
        aResult.D2 = new Vector3D(0.0, 0.0, 0.0);
        aResult.D3 = new Vector3D(0.0, 0.0, 0.0);
        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">N &lt; 1.</exception>
    public sealed override Vector3D EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom_UndefinedDerivative();
        }
        if (N == 1)
        {
            return new Vector3D(pos.Direction());
        }
        else
        {
            return new Vector3D(0.0, 0.0, 0.0);
        }
    }

    /// <summary>U * |T.ScaleFactor()| (U itself when infinite).</summary>
    public sealed override double TransformedParameter(double U, in Transform3D T)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(U))
        {
            return U;
        }
        return U * Math.Abs(T.ScaleFactor());
    }

    /// <summary>|T.ScaleFactor()|.</summary>
    public sealed override double ParametricTransformation(in Transform3D T) => Math.Abs(T.ScaleFactor());

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom_Line WithDirection(in Direction3D V)
    {
        Geom_Line aCopy = (Geom_Line)Copy();
        aCopy.SetDirection(V);
        return aCopy;
    }

    public Geom_Line WithLin(in Line3D L)
    {
        Geom_Line aCopy = (Geom_Line)Copy();
        aCopy.SetLin(L);
        return aCopy;
    }

    public Geom_Line WithLocation(in Point3D P)
    {
        Geom_Line aCopy = (Geom_Line)Copy();
        aCopy.SetLocation(P);
        return aCopy;
    }

    public Geom_Line WithPosition(in Axis3D A1)
    {
        Geom_Line aCopy = (Geom_Line)Copy();
        aCopy.SetPosition(A1);
        return aCopy;
    }
}
