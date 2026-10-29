// Port of: src/ModelingData/TKG3d/Geom/Geom_SurfaceOfLinearExtrusion.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_SurfaceOfLinearExtrusion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A surface of linear extrusion: a basis curve swept along a direction. The U parameter is the parameter of
/// the basis curve, the V parameter (infinite) the distance along the direction.
/// P(U, V) = BasisCurve(U) + V * Direction.
/// </summary>
/// <remarks>
/// The surface keeps its own copy of the basis curve; <see cref="Geom_SweptSurface.BasisCurve"/> returns that copy
/// (shared, as in OCCT; callers can't change it). The evaluation representation (<see cref="HasEvalRepresentation"/>)
/// is set by <see cref="WithEvalRepresentation"/>, which stores a copy of the descriptor; every modification drops it.
/// </remarks>
public class Geom_SurfaceOfLinearExtrusion : Geom_SweptSurface
{
    private GeomEval_RepSurfaceDesc.Base? myEvalRep;

    /// <summary>The extrusion of a copy of C along V. Nothing is checked (C may be a line parallel to V).</summary>
    public Geom_SurfaceOfLinearExtrusion(Geom_Curve C, in gp_Dir V)
    {
        basisCurve = (Geom_Curve)C.Copy();
        direction = V;
        smooth = C.Continuity();
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
    public Geom_SurfaceOfLinearExtrusion WithEvalRepresentation(GeomEval_RepSurfaceDesc.Base? theDesc)
    {
        GeomEval_RepUtils.ValidateSurfaceDesc(theDesc, this);
        Geom_SurfaceOfLinearExtrusion aCopy = (Geom_SurfaceOfLinearExtrusion)Copy();
        aCopy.SetEvalRepresentation(GeomEval_RepUtils.CloneSurfaceDesc(theDesc));
        return aCopy;
    }

    /// <summary>Changes the direction of extrusion.</summary>
    internal void SetDirection(in gp_Dir V)
    {
        CheckMutable();
        ClearEvalRepresentation();
        direction = V;
    }

    /// <summary>Changes the extruded curve (to a copy of C).</summary>
    internal void SetBasisCurve(Geom_Curve C)
    {
        CheckMutable();
        ClearEvalRepresentation();
        smooth = C.Continuity();
        basisCurve = (Geom_Curve)C.Copy();
    }

    /// <summary>A copy extruded along V.</summary>
    public Geom_SurfaceOfLinearExtrusion WithDirection(in gp_Dir V)
    {
        Geom_SurfaceOfLinearExtrusion aCopy = (Geom_SurfaceOfLinearExtrusion)Copy();
        aCopy.SetDirection(V);
        return aCopy;
    }

    /// <summary>A copy extruding (a copy of) C.</summary>
    public Geom_SurfaceOfLinearExtrusion WithBasisCurve(Geom_Curve C)
    {
        Geom_SurfaceOfLinearExtrusion aCopy = (Geom_SurfaceOfLinearExtrusion)Copy();
        aCopy.SetBasisCurve(C);
        return aCopy;
    }

    /// <summary>Reverses the U direction (the basis curve).</summary>
    protected internal sealed override void UReverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        OwnBasisCurve();
        basisCurve!.Reverse();
    }

    /// <summary>The reversed parameter of the basis curve.</summary>
    public sealed override double UReversedParameter(double U) => basisCurve!.ReversedParameter(U);

    /// <summary>Reverses the V direction (the direction of extrusion).</summary>
    protected internal sealed override void VReverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        direction.Reverse();
    }

    /// <summary>-V.</summary>
    public sealed override double VReversedParameter(double V) => (-V);

    /// <summary>U between the bounds of the basis curve; V infinite.</summary>
    public sealed override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        V1 = -Precision.Infinite();
        V2 = Precision.Infinite();
        U1 = basisCurve!.FirstParameter();
        U2 = basisCurve.LastParameter();
    }

    public sealed override gp_Pnt EvalD0(double U, double V)
    {
        gp_Pnt aEvalRepResult;
        if (GeomEval_RepUtils.TryEvalSurfaceD0(myEvalRep, U, V, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        gp_Pnt aBasisD0 = basisCurve!.EvalD0(U);
        gp_Pnt aP;
        Geom_ExtrusionUtils.CalculateD0(aBasisD0, V, direction.XYZ(), out aP);
        return aP;
    }

    public sealed override ResD1 EvalD1(double U, double V)
    {
        ResD1 aEvalRepResult;
        if (GeomEval_RepUtils.TryEvalSurfaceD1(myEvalRep, U, V, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        Geom_Curve.ResD1 aBasisD1 = basisCurve!.EvalD1(U);
        ResD1 aResult;
        Geom_ExtrusionUtils.CalculateD1(aBasisD1.Point,
                                        aBasisD1.D1,
                                        V,
                                        direction.XYZ(),
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

        Geom_Curve.ResD2 aBasisD2 = basisCurve!.EvalD2(U);
        ResD2 aResult;
        Geom_ExtrusionUtils.CalculateD2(aBasisD2.Point,
                                        aBasisD2.D1,
                                        aBasisD2.D2,
                                        V,
                                        direction.XYZ(),
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

        Geom_Curve.ResD3 aBasisD3 = basisCurve!.EvalD3(U);
        ResD3 aResult;
        Geom_ExtrusionUtils.CalculateD3(aBasisD3.Point,
                                        aBasisD3.D1,
                                        aBasisD3.D2,
                                        aBasisD3.D3,
                                        V,
                                        direction.XYZ(),
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

        if (Nv == 0)
        {
            gp_Vec aDN = basisCurve!.EvalDN(U, Nu);
            return Geom_ExtrusionUtils.CalculateDN(aDN, direction.XYZ(), Nu, Nv);
        }
        else if (Nu == 0 && Nv == 1)
        {
            return new gp_Vec(direction.XYZ());
        }
        return new gp_Vec(0.0, 0.0, 0.0);
    }

    /// <summary>The U iso-curve: the line through the point of parameter U of the basis curve, along the direction.</summary>
    public sealed override Geom_Curve UIso(double U)
    {
        Geom_Line L;
        L = new Geom_Line(basisCurve!.Value(U), direction);
        return L;
    }

    /// <summary>The V iso-curve: a copy of the basis curve translated by V * Direction.</summary>
    public sealed override Geom_Curve VIso(double V)
    {
        gp_Vec Vdir = new gp_Vec(direction);
        Vdir.Multiply(V);
        Geom_Curve C;
        C = (Geom_Curve)basisCurve!.Translated(Vdir);
        return C;
    }

    /// <summary>True if the basis curve is CN.</summary>
    /// <exception cref="Standard_RangeError">N &lt; 0.</exception>
    public sealed override bool IsCNu(int N)
    {
        Standard_RangeError_Raise_if(N < 0, " ");
        return basisCurve!.IsCN(N);
    }

    /// <summary>True.</summary>
    public sealed override bool IsCNv(int N) => true;

    /// <summary>Transforms the direction and the basis curve.</summary>
    protected internal sealed override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        ClearEvalRepresentation();
        direction.Transform(T);
        OwnBasisCurve();
        basisCurve!.Transform(T);
    }

    /// <summary>True if the basis curve is closed.</summary>
    public sealed override bool IsUClosed() => basisCurve!.IsClosed();

    /// <summary>True if the basis curve is periodic.</summary>
    public sealed override bool IsUPeriodic() => basisCurve!.IsPeriodic();

    /// <summary>False.</summary>
    public sealed override bool IsVClosed() => false;

    /// <summary>False.</summary>
    public sealed override bool IsVPeriodic() => false;

    /// <summary>U changed into the transformed parameter of the basis curve; V scaled by |T.ScaleFactor()| unless infinite.</summary>
    public sealed override void TransformParameters(ref double U, ref double V, in gp_Trsf T)
    {
        U = basisCurve!.TransformedParameter(U, T);
        if (!Precision.IsInfinite(V))
        {
            V *= Math.Abs(T.ScaleFactor());
        }
    }

    /// <summary>The product of an affinity along U (the basis curve's) and one along V (the scale factor).</summary>
    public sealed override gp_GTrsf2d ParametricTransformation(in gp_Trsf T)
    {
        // transformation in the V Direction
        gp_GTrsf2d TV = new gp_GTrsf2d();
        gp_Ax2d Axis = new gp_Ax2d(gp.Origin2d(), gp.DX2d());
        TV.SetAffinity(Axis, Math.Abs(T.ScaleFactor()));
        // transformation in the U Direction
        gp_GTrsf2d TU = new gp_GTrsf2d();
        Axis = new gp_Ax2d(gp.Origin2d(), gp.DY2d());
        TU.SetAffinity(Axis, basisCurve!.ParametricTransformation(T));
        return TU * TV;
    }

    /// <summary>A new, unfrozen surface on a copy of the basis curve, with a copy of the evaluation representation.</summary>
    public sealed override Geom_Geometry Copy()
    {
        Geom_SurfaceOfLinearExtrusion aCopy = new Geom_SurfaceOfLinearExtrusion(basisCurve!, direction);
        aCopy.myEvalRep = GeomEval_RepUtils.CloneSurfaceDesc(myEvalRep);
        return aCopy;
    }
}
