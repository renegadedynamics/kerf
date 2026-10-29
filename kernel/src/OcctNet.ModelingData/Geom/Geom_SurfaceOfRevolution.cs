// Port of: src/ModelingData/TKG3d/Geom/Geom_SurfaceOfRevolution.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_SurfaceOfRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A surface of revolution: a basis curve (the meridian) rotated about an axis. The U parameter is the angle of
/// rotation, in [0, 2*PI], and the V parameter is the parameter of the meridian.
/// P(U, V) = Rotation(Axis, U) * BasisCurve(V).
/// </summary>
/// <remarks>
/// The surface keeps its own copy of the basis curve; <see cref="Geom_SweptSurface.BasisCurve"/> returns that copy
/// (shared, as in OCCT; callers can't change it). The evaluation representation (<see cref="HasEvalRepresentation"/>)
/// is set by <see cref="WithEvalRepresentation"/>, which stores a copy of the descriptor; every modification drops it.
/// </remarks>
public class Geom_SurfaceOfRevolution : Geom_SweptSurface
{
    private GeomEval_RepSurfaceDesc.Base? myEvalRep;
    private gp_Pnt loc;

    /// <summary>
    /// The revolution of a copy of C about A1. C must be planar and lie in the same plane as A1 (not checked);
    /// the surface may be self-intersecting otherwise.
    /// </summary>
    public Geom_SurfaceOfRevolution(Geom_Curve C, in gp_Ax1 A1)
    {
        loc = A1.Location();
        direction = A1.Direction();
        SetBasisCurve(C);
    }

    /// <summary>True if an evaluation representation is attached.</summary>
    public bool HasEvalRepresentation() => myEvalRep is not null;

    /// <summary>The evaluation representation descriptor (may be null). Stored, not copied: don't modify.</summary>
    internal GeomEval_RepSurfaceDesc.Base? EvalRepresentation() => myEvalRep;

    /// <summary>Sets a new evaluation representation, after checking it (no circular reference).</summary>
    internal void SetEvalRepresentation(GeomEval_RepSurfaceDesc.Base? theDesc)
    {
        CheckMutable();
        GeomEval_RepUtils.ValidateSurfaceDesc(theDesc, this);
        myEvalRep = theDesc;
    }

    /// <summary>Removes the evaluation representation.</summary>
    internal void ClearEvalRepresentation()
    {
        CheckMutable();
        myEvalRep = null;
    }

    /// <summary>A copy evaluated through a copy of theDesc (null: without a representation).</summary>
    public Geom_SurfaceOfRevolution WithEvalRepresentation(GeomEval_RepSurfaceDesc.Base? theDesc)
    {
        GeomEval_RepUtils.ValidateSurfaceDesc(theDesc, this);
        Geom_SurfaceOfRevolution aCopy = (Geom_SurfaceOfRevolution)Copy();
        aCopy.SetEvalRepresentation(GeomEval_RepUtils.CloneSurfaceDesc(theDesc));
        return aCopy;
    }

    /// <summary>Changes the axis of revolution.</summary>
    internal void SetAxis(in gp_Ax1 A1)
    {
        CheckMutable();
        ClearEvalRepresentation();
        direction = A1.Direction();
        loc = A1.Location();
    }

    /// <summary>Changes the direction of the axis of revolution.</summary>
    internal void SetDirection(in gp_Dir V)
    {
        CheckMutable();
        ClearEvalRepresentation();
        direction = V;
    }

    /// <summary>Changes the revolved curve (to a copy of C).</summary>
    internal void SetBasisCurve(Geom_Curve C)
    {
        CheckMutable();
        ClearEvalRepresentation();
        basisCurve = (Geom_Curve)C.Copy();
        smooth = C.Continuity();
    }

    /// <summary>Changes the location point of the axis of revolution.</summary>
    internal void SetLocation(in gp_Pnt P)
    {
        CheckMutable();
        ClearEvalRepresentation();
        loc = P;
    }

    /// <summary>A copy revolved about A1.</summary>
    public Geom_SurfaceOfRevolution WithAxis(in gp_Ax1 A1)
    {
        Geom_SurfaceOfRevolution aCopy = (Geom_SurfaceOfRevolution)Copy();
        aCopy.SetAxis(A1);
        return aCopy;
    }

    /// <summary>A copy whose axis of revolution has the direction V.</summary>
    public Geom_SurfaceOfRevolution WithDirection(in gp_Dir V)
    {
        Geom_SurfaceOfRevolution aCopy = (Geom_SurfaceOfRevolution)Copy();
        aCopy.SetDirection(V);
        return aCopy;
    }

    /// <summary>A copy revolving (a copy of) C.</summary>
    public Geom_SurfaceOfRevolution WithBasisCurve(Geom_Curve C)
    {
        Geom_SurfaceOfRevolution aCopy = (Geom_SurfaceOfRevolution)Copy();
        aCopy.SetBasisCurve(C);
        return aCopy;
    }

    /// <summary>A copy whose axis of revolution passes through P.</summary>
    public Geom_SurfaceOfRevolution WithLocation(in gp_Pnt P)
    {
        Geom_SurfaceOfRevolution aCopy = (Geom_SurfaceOfRevolution)Copy();
        aCopy.SetLocation(P);
        return aCopy;
    }

    /// <summary>The axis of revolution.</summary>
    public gp_Ax1 Axis() => new gp_Ax1(loc, direction);

    /// <summary>The location point of the axis of revolution.</summary>
    public gp_Pnt Location() => loc;

    /// <summary>Not implemented in OCCT.</summary>
    /// <exception cref="Standard_NotImplemented">Always.</exception>
    public gp_Ax2 ReferencePlane()
    {
        throw new Standard_NotImplemented();
    }

    /// <summary>Reverses the U direction (the direction of the axis of revolution).</summary>
    protected internal sealed override void UReverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        direction.Reverse();
    }

    /// <summary>2*PI - U.</summary>
    public sealed override double UReversedParameter(double U) => (2.0 * M_PI - U);

    /// <summary>Reverses the V direction (the basis curve).</summary>
    protected internal sealed override void VReverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        basisCurve!.Reverse();
    }

    /// <summary>The reversed parameter of the basis curve.</summary>
    public sealed override double VReversedParameter(double V) => basisCurve!.ReversedParameter(V);

    /// <summary>V changed into the transformed parameter of the basis curve; U unchanged.</summary>
    public sealed override void TransformParameters(ref double U, ref double V, in gp_Trsf T)
    {
        V = basisCurve!.TransformedParameter(V, T);
    }

    /// <summary>An affinity along V of the basis curve's parametric transformation.</summary>
    public sealed override gp_GTrsf2d ParametricTransformation(in gp_Trsf T)
    {
        gp_GTrsf2d T2 = new gp_GTrsf2d();
        gp_Ax2d Axis = new gp_Ax2d(gp.Origin2d(), gp.DX2d());
        T2.SetAffinity(Axis, basisCurve!.ParametricTransformation(T));
        return T2;
    }

    /// <summary>U in [0, 2*PI]; V between the bounds of the basis curve.</summary>
    public sealed override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = 0.0;
        U2 = 2.0 * M_PI;
        V1 = basisCurve!.FirstParameter();
        V2 = basisCurve.LastParameter();
    }

    /// <summary>True.</summary>
    public sealed override bool IsUClosed() => true;

    /// <summary>True if the basis curve is closed.</summary>
    public sealed override bool IsVClosed() => basisCurve!.IsClosed();

    /// <summary>True.</summary>
    public sealed override bool IsCNu(int N) => true;

    /// <summary>True if the basis curve is CN.</summary>
    /// <exception cref="Standard_RangeError">N &lt; 0.</exception>
    public sealed override bool IsCNv(int N)
    {
        Standard_RangeError_Raise_if(N < 0, " ");
        return basisCurve!.IsCN(N);
    }

    /// <summary>True (period 2*PI).</summary>
    public sealed override bool IsUPeriodic() => true;

    /// <summary>True if the basis curve is periodic.</summary>
    public sealed override bool IsVPeriodic() => basisCurve!.IsPeriodic();

    /// <summary>The U iso-curve: a copy of the basis curve rotated by U.</summary>
    public sealed override Geom_Curve UIso(double U)
    {
        Geom_Curve C = (Geom_Curve)basisCurve!.Copy();
        gp_Ax1 RotAxis = new gp_Ax1(loc, direction);
        C.Rotate(RotAxis, U);
        return C;
    }

    /// <summary>The V iso-curve: the circle described by the point of parameter V of the basis curve.</summary>
    public sealed override Geom_Curve VIso(double V)
    {
        // TODO(1A): Geom_Circle is ported by unit 1A. OCCT:
        //   occ::handle<Geom_Circle> Circ;
        //   Pnt                      Pc = basisCurve->Value(V);
        //   gp_Lin                   L1(loc, direction);
        //   double                   Rad = L1.Distance(Pc);
        //   Ax2 Rep;
        //   if (Rad > gp::Resolution())
        //   {
        //     XYZ P = Pc.XYZ();
        //     XYZ C;
        //     C.SetLinearForm((P - loc.XYZ()).Dot(direction.XYZ()), direction.XYZ(), loc.XYZ());
        //     P = P - C;
        //     if (P.Modulus() > gp::Resolution())
        //     {
        //       gp_Dir D = P.Normalized();
        //       Rep      = gp_Ax2(C, direction, D);
        //     }
        //     else
        //     {
        //       Rep = gp_Ax2(C, direction);
        //     }
        //   }
        //   else
        //   {
        //     Rep = gp_Ax2(Pc, direction);
        //   }
        //   Circ = new Geom_Circle(Rep, Rad);
        //   return Circ;
        throw new Standard_NotImplemented("Geom_SurfaceOfRevolution::VIso needs Geom_Circle (unit 1A)");
    }

    public sealed override gp_Pnt EvalD0(double U, double V)
    {
        gp_Pnt aEvalRepResult;
        if (GeomEval_RepUtils.TryEvalSurfaceD0(myEvalRep, U, V, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        gp_Pnt aBasisD0 = basisCurve!.EvalD0(V);
        gp_Pnt aP;
        Geom_RevolutionUtils.CalculateD0(aBasisD0, U, new gp_Ax1(loc, direction), out aP);
        return aP;
    }

    public sealed override ResD1 EvalD1(double U, double V)
    {
        ResD1 aEvalRepResult;
        if (GeomEval_RepUtils.TryEvalSurfaceD1(myEvalRep, U, V, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        Geom_Curve.ResD1 aBasisD1 = basisCurve!.EvalD1(V);
        ResD1 aResult;
        Geom_RevolutionUtils.CalculateD1(aBasisD1.Point,
                                         aBasisD1.D1,
                                         U,
                                         new gp_Ax1(loc, direction),
                                         out aResult.Point,
                                         out aResult.D1U,
                                         out aResult.D1V);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U, double V)
    {
        ResD2 aEvalRepResult;
        if (GeomEval_RepUtils.TryEvalSurfaceD2(myEvalRep, U, V, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        Geom_Curve.ResD2 aBasisD2 = basisCurve!.EvalD2(V);
        ResD2 aResult;
        Geom_RevolutionUtils.CalculateD2(aBasisD2.Point,
                                         aBasisD2.D1,
                                         aBasisD2.D2,
                                         U,
                                         new gp_Ax1(loc, direction),
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
        ResD3 aEvalRepResult;
        if (GeomEval_RepUtils.TryEvalSurfaceD3(myEvalRep, U, V, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        Geom_Curve.ResD3 aBasisD3 = basisCurve!.EvalD3(V);
        ResD3 aResult;
        Geom_RevolutionUtils.CalculateD3(aBasisD3.Point,
                                         aBasisD3.D1,
                                         aBasisD3.D2,
                                         aBasisD3.D3,
                                         U,
                                         new gp_Ax1(loc, direction),
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

        gp_Vec aEvalRepResult;
        if (GeomEval_RepUtils.TryEvalSurfaceDN(myEvalRep, U, V, Nu, Nv, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        gp_Vec aCurvePtOrDN;
        if (Nu == 0)
        {
            aCurvePtOrDN = basisCurve!.EvalDN(V, Nv);
        }
        else if (Nv == 0)
        {
            gp_Pnt aD0 = basisCurve!.EvalD0(V);
            aCurvePtOrDN = new gp_Vec(aD0.XYZ() - loc.XYZ());
        }
        else
        {
            aCurvePtOrDN = basisCurve!.EvalDN(V, Nv);
        }
        return Geom_RevolutionUtils.CalculateDN(aCurvePtOrDN, U, new gp_Ax1(loc, direction), Nu, Nv);
    }

    /// <summary>Transforms the axis and the basis curve; a transformation reversing orientation also reverses U.</summary>
    protected internal sealed override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        ClearEvalRepresentation();
        loc.Transform(T);
        direction.Transform(T);
        basisCurve!.Transform(T);
        if (T.ScaleFactor() * T.HVectorialPart().Determinant() < 0.0)
        {
            UReverse();
        }
    }

    /// <summary>A new, unfrozen surface on a copy of the basis curve, with a copy of the evaluation representation.</summary>
    public sealed override Geom_Geometry Copy()
    {
        Geom_SurfaceOfRevolution aCopy = new Geom_SurfaceOfRevolution(basisCurve!, Axis());
        aCopy.myEvalRep = GeomEval_RepUtils.CloneSurfaceDesc(myEvalRep);
        return aCopy;
    }
}
