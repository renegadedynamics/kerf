// Port of: src/ModelingData/TKG3d/Geom/Geom_OffsetSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_OffsetSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A surface offset from a basis surface: P(u, v) = p(u, v) + Offset * N / ||N||, where N = dp/du ^ dp/dv is the
/// normal of the basis surface. The basis surface must be at least C1 (G1 for a C0 B-spline or swept surface,
/// unless isNotCheckC0); it is copied, and nested offset and trimmed surfaces are flattened. The parametrization
/// is that of the basis surface.
/// </summary>
/// <remarks>
/// <para>Where the normal is undefined (a degenerate iso-curve of a B-spline or Bezier surface, or a cone apex), the
/// evaluation uses higher derivatives of the normal and the osculating surfaces built at construction, or
/// retries closer to the center of the parameter space; it throws if that fails.</para>
/// <para>When the offset of a plane, cylinder, cone, sphere or torus is again such a surface
/// (<see cref="Surface"/>), it is attached as the evaluation representation and evaluates the offset surface.
/// <see cref="BasisSurface"/> returns the stored basis surface frozen, as OCCT returns its handle; this surface's
/// internal mutators then copy it before changing it (decision 211).</para>
/// </remarks>
public class Geom_OffsetSurface : Geom_Surface
{
    private static readonly double MyAngularToleranceForG1 = Precision.Angular();

    private Geom_Surface basisSurf = null!;
    private GeomEval_RepSurfaceDesc.Base? myEvalRep;
    private double offsetValue;
    private Geom_OsculatingSurface? myOscSurf;
    private GeomAbs_Shape myBasisSurfContinuity;

    private static GeomEval_RepSurfaceDesc.Base? makeFullSurfaceRep(Geom_Surface? theSurface)
    {
        if (theSurface is null)
        {
            return null;
        }
        GeomEval_RepSurfaceDesc.Full aDesc = new GeomEval_RepSurfaceDesc.Full();
        aDesc.Representation = theSurface;
        return aDesc;
    }

    private static Geom_Surface? directRepSurface(Geom_OffsetSurface theSurface)
    {
        GeomEval_RepSurfaceDesc.Base? aDesc = theSurface.EvalRepresentation();
        if (aDesc is null)
        {
            return null;
        }
        if (aDesc.GetKind() != GeomEval_RepSurfaceDesc.Base.Kind.Full)
        {
            return null;
        }
        return aDesc.Representation;
    }

    /// <summary>
    /// The surface at distance theOffset from theSurf along its normal (a copy of theSurf; trimmed and offset
    /// surfaces are flattened, their offsets added).
    /// </summary>
    /// <exception cref="Standard_ConstructionError">
    /// The basis surface is C0 and not G1 (only checked for B-spline and swept surfaces), unless isNotCheckC0.
    /// </exception>
    public Geom_OffsetSurface(Geom_Surface theSurf, double theOffset, bool isNotCheckC0 = false)
    {
        offsetValue = theOffset;
        SetBasisSurface(theSurf, isNotCheckC0);
    }

    /// <summary>A copy of theOther (deep: the basis surface is copied), without validation.</summary>
    public Geom_OffsetSurface(Geom_OffsetSurface theOther)
    {
        basisSurf = (Geom_Surface)theOther.basisSurf.Copy();
        myEvalRep = GeomEval_RepUtils.CloneSurfaceDesc(theOther.myEvalRep);
        offsetValue = theOther.offsetValue;
        myOscSurf = theOther.myOscSurf is not null ? new Geom_OsculatingSurface(theOther.myOscSurf) : null;
        myBasisSurfContinuity = theOther.myBasisSurfContinuity;
        // Deep copy without validation - source surface is already validated.
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

    /// <summary>A new, unfrozen surface equal to this one (on a copy of the basis surface).</summary>
    public sealed override Geom_Geometry Copy() => new Geom_OffsetSurface(this);

    /// <summary>
    /// Changes the basis surface (a copy of S, flattened as in the constructor); the evaluation representation is
    /// recomputed, and so are the osculating surfaces of a B-spline or Bezier basis.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">As the constructor.</exception>
    internal void SetBasisSurface(Geom_Surface S, bool isNotCheckC0 = false)
    {
        CheckMutable();
        ClearEvalRepresentation();
        S.Bounds(out double aUf, out double aUl, out double aVf, out double aVl);

        Geom_Surface aCheckingSurf = (Geom_Surface)S.Copy();
        bool isTrimmed = false;

        while (aCheckingSurf is Geom_RectangularTrimmedSurface || aCheckingSurf is Geom_OffsetSurface)
        {
            if (aCheckingSurf is Geom_RectangularTrimmedSurface aTrimS)
            {
                aCheckingSurf = aTrimS.BasisSurface();
                isTrimmed = true;
            }

            if (aCheckingSurf is Geom_OffsetSurface aOS)
            {
                aCheckingSurf = aOS.BasisSurface();
                offsetValue += aOS.Offset();
            }
        }

        myBasisSurfContinuity = aCheckingSurf.Continuity();

        bool isC0 = !isNotCheckC0 && (myBasisSurfContinuity == GeomAbs_C0);

        // Basis surface must be at least C1
        if (isC0)
        {
            Geom_Curve? aCurve = null;

            if (aCheckingSurf is Geom_SurfaceOfRevolution aRevSurf)
            {
                aCurve = aRevSurf.BasisCurve();
            }
            else if (aCheckingSurf is Geom_SurfaceOfLinearExtrusion aLESurf)
            {
                aCurve = aLESurf.BasisCurve();
            }

            if (aCurve is not null)
            {
                while (aCurve is Geom_TrimmedCurve || aCurve is Geom_OffsetCurve)
                {
                    if (aCurve is Geom_TrimmedCurve aTrimC)
                    {
                        aCurve = aTrimC.BasisCurve();
                    }

                    if (aCurve is Geom_OffsetCurve aOC)
                    {
                        aCurve = aOC.BasisCurve();
                    }
                }
            }

            double aUIsoPar = (aUf + aUl) / 2.0, aVIsoPar = (aVf + aVl) / 2.0;
            bool isUG1, isVG1;

            Geom_Curve aCurv1 = aCurve is null ? aCheckingSurf.UIso(aUIsoPar) : aCurve;
            Geom_Curve aCurv2 = aCheckingSurf.VIso(aVIsoPar);
            isUG1 = aCurv1 is not Geom_BSplineCurve;
            isVG1 = aCurv2 is not Geom_BSplineCurve;

            if (!isUG1)
            {
                Geom_BSplineCurve aBC = (Geom_BSplineCurve)aCurv1;
                isUG1 = aBC.IsG1(aVf, aVl, MyAngularToleranceForG1);
            }
            //
            if (!isVG1)
            {
                Geom_BSplineCurve aBC = (Geom_BSplineCurve)aCurv2;
                isVG1 = aBC.IsG1(aUf, aUl, MyAngularToleranceForG1);
            }
            //
            if (isUG1 && isVG1)
            {
                myBasisSurfContinuity = GeomAbs_G1;
                isC0 = false;
            }

            // Raise exception if still C0
            if (isC0)
            {
                throw new Standard_ConstructionError("Offset with no C1 Surface");
            }
        }

        if (isTrimmed)
        {
            basisSurf = new Geom_RectangularTrimmedSurface(aCheckingSurf, aUf, aUl, aVf, aVl);
        }
        else
        {
            basisSurf = aCheckingSurf;
        }

        Geom_Surface? aSurfaceRep = Surface();
        SetEvalRepresentation(makeFullSurfaceRep(aSurfaceRep));

        if (aCheckingSurf is Geom_BSplineSurface || aCheckingSurf is Geom_BezierSurface)
        {
            // Hard-coded tolerance for now, but it should be offered in the constructor
            // and stored as a field. It could be used, for example, for iso-curve extraction
            // and also for singularity detection. For osculating surfaces, it is used to
            // detect whether an iso-curve is degenerate.
            double Tol = Precision.Confusion(); // 0.0001;
            myOscSurf = new Geom_OsculatingSurface(aCheckingSurf, Tol);
        }
    }

    /// <summary>Changes the offset value; the evaluation representation is recomputed.</summary>
    internal void SetOffsetValue(double D)
    {
        CheckMutable();
        ClearEvalRepresentation();
        offsetValue = D;
        SetEvalRepresentation(makeFullSurfaceRep(Surface()));
    }

    /// <summary>The offset value.</summary>
    public double Offset() => offsetValue;

    /// <summary>The basis surface, frozen (it is shared with this offset surface; decision 211).</summary>
    public Geom_Surface BasisSurface()
    {
        basisSurf.Freeze();
        return basisSurf;
    }

    // OCCT.NET: the basis surface may have been handed out (and frozen) by BasisSurface(); copy it before
    // changing it.
    private void ownBasisSurface()
    {
        if (basisSurf.IsFrozen())
        {
            basisSurf = (Geom_Surface)basisSurf.Copy();
        }
    }

    /// <summary>Reverses U on the basis surface and the sign of the offset (drops the evaluation representation).</summary>
    protected internal sealed override void UReverse()
    {
        CheckMutable();
        // OCCT: UReverse, VReverse and Transform drop the equivalent-surface representation without recomputing it
        // (SetOffsetValue and SetBasisSurface do recompute it): see occt-issues.md.
        ClearEvalRepresentation();
        ownBasisSurface();
        basisSurf.UReverse();
        offsetValue = -offsetValue;
    }

    /// <summary>The basis surface's reversed U parameter.</summary>
    public sealed override double UReversedParameter(double U) => basisSurf.UReversedParameter(U);

    /// <summary>Reverses V on the basis surface and the sign of the offset (drops the evaluation representation).</summary>
    protected internal sealed override void VReverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        ownBasisSurface();
        basisSurf.VReverse();
        offsetValue = -offsetValue;
    }

    /// <summary>The basis surface's reversed V parameter.</summary>
    public sealed override double VReversedParameter(double V) => basisSurf.VReversedParameter(V);

    /// <summary>The bounds of the basis surface.</summary>
    public sealed override void Bounds(out double U1, out double U2, out double V1, out double V2) =>
        basisSurf.Bounds(out U1, out U2, out V1, out V2);

    /// <summary>The continuity of the basis surface lowered by one (C0 below C2).</summary>
    public sealed override GeomAbs_Shape Continuity()
    {
        switch (myBasisSurfContinuity)
        {
            case GeomAbs_C2:
                return GeomAbs_C1;
            case GeomAbs_C3:
                return GeomAbs_C2;
            case GeomAbs_CN:
                return GeomAbs_CN;
            default:
                break;
        }
        return GeomAbs_C0;
    }

    /// <exception cref="Geom_UndefinedValue">The point is undefined at (U, V).</exception>
    public sealed override gp_Pnt EvalD0(double U, double V)
    {
        if (GeomEval_RepUtils.TryEvalSurfaceD0(myEvalRep, U, V, out gp_Pnt aEvalRepResult))
        {
            return aEvalRepResult;
        }

        if (!Geom_OffsetSurfaceUtils.EvaluateD0(U,
                                                V,
                                                new Geom_OffsetSurfaceUtils.GeomSurfaceBasis(basisSurf),
                                                offsetValue,
                                                myOscSurf,
                                                out gp_Pnt aP))
        {
            throw new Geom_UndefinedValue("Geom_OffsetSurface::EvalD0");
        }
        return aP;
    }

    /// <exception cref="Geom_UndefinedDerivative">The derivatives are undefined at (U, V).</exception>
    public sealed override ResD1 EvalD1(double U, double V)
    {
        if (GeomEval_RepUtils.TryEvalSurfaceD1(myEvalRep, U, V, out ResD1 aEvalRepResult))
        {
            return aEvalRepResult;
        }

        ResD1 aResult;
        if (!Geom_OffsetSurfaceUtils.EvaluateD1(U,
                                                V,
                                                new Geom_OffsetSurfaceUtils.GeomSurfaceBasis(basisSurf),
                                                offsetValue,
                                                myOscSurf,
                                                out aResult.Point,
                                                out aResult.D1U,
                                                out aResult.D1V))
        {
            throw new Geom_UndefinedDerivative("Geom_OffsetSurface::EvalD1");
        }
        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">The derivatives are undefined at (U, V).</exception>
    public sealed override ResD2 EvalD2(double U, double V)
    {
        if (GeomEval_RepUtils.TryEvalSurfaceD2(myEvalRep, U, V, out ResD2 aEvalRepResult))
        {
            return aEvalRepResult;
        }

        ResD2 aResult;
        if (!Geom_OffsetSurfaceUtils.EvaluateD2(U,
                                                V,
                                                new Geom_OffsetSurfaceUtils.GeomSurfaceBasis(basisSurf),
                                                offsetValue,
                                                myOscSurf,
                                                out aResult.Point,
                                                out aResult.D1U,
                                                out aResult.D1V,
                                                out aResult.D2U,
                                                out aResult.D2V,
                                                out aResult.D2UV))
        {
            throw new Geom_UndefinedDerivative("Geom_OffsetSurface::EvalD2");
        }
        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">The derivatives are undefined at (U, V).</exception>
    public sealed override ResD3 EvalD3(double U, double V)
    {
        if (GeomEval_RepUtils.TryEvalSurfaceD3(myEvalRep, U, V, out ResD3 aEvalRepResult))
        {
            return aEvalRepResult;
        }

        ResD3 aResult;
        if (!Geom_OffsetSurfaceUtils.EvaluateD3(U,
                                                V,
                                                new Geom_OffsetSurfaceUtils.GeomSurfaceBasis(basisSurf),
                                                offsetValue,
                                                myOscSurf,
                                                out aResult.Point,
                                                out aResult.D1U,
                                                out aResult.D1V,
                                                out aResult.D2U,
                                                out aResult.D2V,
                                                out aResult.D2UV,
                                                out aResult.D3U,
                                                out aResult.D3V,
                                                out aResult.D3UUV,
                                                out aResult.D3UVV))
        {
            throw new Geom_UndefinedDerivative("Geom_OffsetSurface::EvalD3");
        }
        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">
    /// Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0, or the derivative is undefined at (U, V).
    /// </exception>
    public sealed override gp_Vec EvalDN(double U, double V, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            throw new Geom_UndefinedDerivative("Geom_OffsetSurface::EvalDN");
        }
        if (GeomEval_RepUtils.TryEvalSurfaceDN(myEvalRep, U, V, Nu, Nv, out gp_Vec aEvalRepResult))
        {
            return aEvalRepResult;
        }

        if (!Geom_OffsetSurfaceUtils.EvaluateDN(U,
                                                V,
                                                Nu,
                                                Nv,
                                                new Geom_OffsetSurfaceUtils.GeomSurfaceBasis(basisSurf),
                                                offsetValue,
                                                myOscSurf,
                                                out gp_Vec aResult))
        {
            throw new Geom_UndefinedDerivative("Geom_OffsetSurface::EvalDN");
        }
        return aResult;
    }

    ////*************************************************
    ////
    ////   EVALUATOR FOR THE ISO-CURVE APPROXIMATION
    ////
    ////*************************************************

    private sealed class Geom_OffsetSurface_UIsoEvaluator : AdvApprox_EvaluatorFunction
    {
        private readonly Geom_Surface mySurface;
        private readonly double myIsoPar;

        public Geom_OffsetSurface_UIsoEvaluator(Geom_Surface theSurface, double theU)
        {
            mySurface = theSurface;
            myIsoPar = theU;
        }

        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Parameter,
                                      int DerivativeRequest, Span<double> Result, out int ReturnCode)
        {
            gp_Pnt P;
            if (DerivativeRequest == 0)
            {
                P = mySurface.Value(myIsoPar, Parameter);
                Result[0] = P.X();
                Result[1] = P.Y();
                Result[2] = P.Z();
            }
            else
            {
                mySurface.D1(myIsoPar, Parameter, out P, out gp_Vec DU, out gp_Vec DV);
                Result[0] = DV.X();
                Result[1] = DV.Y();
                Result[2] = DV.Z();
            }
            ReturnCode = 0;
        }
    }

    private sealed class Geom_OffsetSurface_VIsoEvaluator : AdvApprox_EvaluatorFunction
    {
        private readonly Geom_Surface mySurface;
        private readonly double myIsoPar;

        public Geom_OffsetSurface_VIsoEvaluator(Geom_Surface theSurface, double theV)
        {
            mySurface = theSurface;
            myIsoPar = theV;
        }

        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Parameter,
                                      int DerivativeRequest, Span<double> Result, out int ReturnCode)
        {
            gp_Pnt P;
            if (DerivativeRequest == 0)
            {
                P = mySurface.Value(Parameter, myIsoPar);
                Result[0] = P.X();
                Result[1] = P.Y();
                Result[2] = P.Z();
            }
            else
            {
                mySurface.D1(Parameter, myIsoPar, out P, out gp_Vec DU, out gp_Vec DV);
                Result[0] = DU.X();
                Result[1] = DU.Y();
                Result[2] = DU.Z();
            }
            ReturnCode = 0;
        }
    }

    /// <summary>
    /// The U iso-curve: the equivalent surface's, a translated iso-curve of an extrusion basis, or else a C1
    /// B-spline approximation (degree at most 14, 100 segments, tolerance Precision::Approximation()).
    /// </summary>
    /// <exception cref="Standard_ConstructionError">The approximation fails.</exception>
    public sealed override Geom_Curve UIso(double UU)
    {
        Geom_Surface? anEquivSurface = directRepSurface(this);
        if (anEquivSurface is null)
        {
            GeomAdaptor_Surface aGAsurf = new GeomAdaptor_Surface(basisSurf);
            if (aGAsurf.GetType() == GeomAbs_SurfaceType.GeomAbs_SurfaceOfExtrusion)
            {
                Geom_Curve aL = basisSurf.UIso(UU);

                basisSurf.D1(UU, 0.0, out gp_Pnt aP, out gp_Vec aD1U, out gp_Vec aD1V);
                gp_Vec aDir = aD1U.Crossed(aD1V);
                if (aDir.SquareMagnitude() < gp.Resolution())
                {
                    return aL;
                }
                aDir.Normalize();
                aDir *= offsetValue;

                aL.Translate(aDir);
                return aL;
            }
            const int Num1 = 0, Num2 = 0, Num3 = 1;
            NCollection_HArray1<double>? T1 = null, T2 = null;
            NCollection_HArray1<double> T3 = new(1, Num3);
            T3.Init(Precision.Approximation());
            Bounds(out double U1, out double U2, out double V1, out double V2);
            const GeomAbs_Shape Cont = GeomAbs_C1;
            const int MaxSeg = 100, MaxDeg = 14;

            Geom_OffsetSurface_UIsoEvaluator ev = new(this, UU);
            AdvApprox_ApproxAFunction Approx = new(Num1, Num2, Num3, T1, T2, T3, V1, V2, Cont, MaxDeg, MaxSeg, ev);

            Standard_ConstructionError_Raise_if(!Approx.IsDone(), " Geom_OffsetSurface : UIso");

            int NbPoles = Approx.NbPoles();

            NCollection_Array1<gp_Pnt> Poles = new(1, NbPoles);

            Approx.Poles(1, Poles);
            NCollection_Array1<double> Knots = new(Approx.Knots()!);
            NCollection_Array1<int> Mults = new(Approx.Multiplicities()!);

            Geom_BSplineCurve C = new Geom_BSplineCurve(Poles, Knots, Mults, Approx.Degree());
            return C;
        }
        return anEquivSurface.UIso(UU);
    }

    /// <summary>
    /// The V iso-curve: the equivalent surface's, or else a C1 B-spline approximation (degree at most 14, 100
    /// segments, tolerance Precision::Approximation()).
    /// </summary>
    /// <exception cref="Standard_ConstructionError">The approximation fails.</exception>
    public sealed override Geom_Curve VIso(double VV)
    {
        Geom_Surface? anEquivSurface = directRepSurface(this);
        if (anEquivSurface is null)
        {
            const int Num1 = 0, Num2 = 0, Num3 = 1;
            NCollection_HArray1<double>? T1 = null, T2 = null;
            NCollection_HArray1<double> T3 = new(1, Num3);
            T3.Init(Precision.Approximation());
            Bounds(out double U1, out double U2, out double V1, out double V2);
            const GeomAbs_Shape Cont = GeomAbs_C1;
            const int MaxSeg = 100, MaxDeg = 14;

            Geom_OffsetSurface_VIsoEvaluator ev = new(this, VV);
            AdvApprox_ApproxAFunction Approx = new(Num1, Num2, Num3, T1, T2, T3, U1, U2, Cont, MaxDeg, MaxSeg, ev);

            Standard_ConstructionError_Raise_if(!Approx.IsDone(), " Geom_OffsetSurface : VIso");

            NCollection_Array1<gp_Pnt> Poles = new(1, Approx.NbPoles());

            Approx.Poles(1, Poles);
            NCollection_Array1<double> Knots = new(Approx.Knots()!);
            NCollection_Array1<int> Mults = new(Approx.Multiplicities()!);

            Geom_BSplineCurve C = new Geom_BSplineCurve(Poles, Knots, Mults, Approx.Degree());
            return C;
        }
        return anEquivSurface.VIso(VV);
    }

    /// <summary>True if the basis surface is C(N + 1) in U.</summary>
    /// <exception cref="Standard_RangeError">N &lt; 0.</exception>
    public sealed override bool IsCNu(int N)
    {
        Standard_RangeError_Raise_if(N < 0, " ");
        return basisSurf.IsCNu(N + 1);
    }

    /// <summary>True if the basis surface is C(N + 1) in V.</summary>
    /// <exception cref="Standard_RangeError">N &lt; 0.</exception>
    public sealed override bool IsCNv(int N)
    {
        Standard_RangeError_Raise_if(N < 0, " ");
        return basisSurf.IsCNv(N + 1);
    }

    public sealed override bool IsUPeriodic() => basisSurf.IsUPeriodic();

    /// <summary>The U period of the basis surface.</summary>
    public sealed override double UPeriod() => basisSurf.UPeriod();

    public sealed override bool IsVPeriodic() => basisSurf.IsVPeriodic();

    /// <summary>The V period of the basis surface.</summary>
    public sealed override double VPeriod() => basisSurf.VPeriod();

    /// <summary>
    /// The U closure from the kind of basis surface (OCCT doesn't evaluate the surface): an elementary surface's
    /// (trimmed or not), a surface of revolution (its trimmed U closure), an extrusion of a circle or an ellipse;
    /// false otherwise.
    /// </summary>
    public sealed override bool IsUClosed()
    {
        bool UClosed;
        Geom_Surface SBasis = basisSurf;

        if (SBasis is Geom_RectangularTrimmedSurface St)
        {
            Geom_Surface S = St.BasisSurface();
            if (S is Geom_ElementarySurface)
            {
                UClosed = SBasis.IsUClosed();
            }
            else if (S is Geom_SurfaceOfLinearExtrusion Extru)
            {
                Geom_Curve C = Extru.BasisCurve();
                if (C is Geom_Circle || C is Geom_Ellipse)
                {
                    UClosed = SBasis.IsUClosed();
                }
                else
                {
                    UClosed = false;
                }
            }
            else if (S is Geom_SurfaceOfRevolution)
            {
                UClosed = SBasis.IsUClosed();
            }
            else
            {
                UClosed = false;
            }
        }
        else
        {
            if (SBasis is Geom_ElementarySurface)
            {
                UClosed = SBasis.IsUClosed();
            }
            else if (SBasis is Geom_SurfaceOfLinearExtrusion Extru)
            {
                Geom_Curve C = Extru.BasisCurve();
                UClosed = (C is Geom_Circle || C is Geom_Ellipse);
            }
            else if (SBasis is Geom_SurfaceOfRevolution)
            {
                UClosed = true;
            }
            else
            {
                UClosed = false;
            }
        }
        return UClosed;
    }

    /// <summary>The V closure of an elementary basis surface (trimmed or not); false otherwise.</summary>
    public sealed override bool IsVClosed()
    {
        bool VClosed;
        Geom_Surface SBasis = basisSurf;

        if (SBasis is Geom_RectangularTrimmedSurface St)
        {
            Geom_Surface S = St.BasisSurface();
            if (S is Geom_ElementarySurface)
            {
                VClosed = SBasis.IsVClosed();
            }
            else
            {
                VClosed = false;
            }
        }
        else
        {
            if (SBasis is Geom_ElementarySurface)
            {
                VClosed = SBasis.IsVClosed();
            }
            else
            {
                VClosed = false;
            }
        }
        return VClosed;
    }

    /// <summary>
    /// Applies T to the basis surface; the offset is scaled by T.ScaleFactor() (drops the evaluation
    /// representation, as OCCT does).
    /// </summary>
    protected internal sealed override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        ClearEvalRepresentation();
        ownBasisSurface();
        basisSurf.Transform(T);
        offsetValue *= T.ScaleFactor();
    }

    /// <summary>
    /// The basis surface's transformed parameters, transformed again by the equivalent surface when there is one
    /// (OCCT's error: see occt-issues.md).
    /// </summary>
    public sealed override void TransformParameters(ref double U, ref double V, in gp_Trsf T)
    {
        basisSurf.TransformParameters(ref U, ref V, T);
        Geom_Surface? anEquivSurface = directRepSurface(this);
        if (anEquivSurface is not null)
        {
            // OCCT: the parameters are transformed a second time by the equivalent surface (scaled twice for a
            // plane, a cylinder's V, ...), although its parametrization is the basis surface's.
            anEquivSurface.TransformParameters(ref U, ref V, T);
        }
    }

    /// <summary>The basis surface's parametric transformation.</summary>
    public sealed override gp_GTrsf2d ParametricTransformation(in gp_Trsf T) => basisSurf.ParametricTransformation(T);

    /// <summary>
    /// An equivalent non-offset surface if there is one: the basis surface itself (frozen) for a zero offset,
    /// and for an offset of a plane, cylinder, cone, sphere or (non-self-intersecting) torus, trimmed or not,
    /// the surface of that kind at the offset; null otherwise (or when the result degenerates).
    /// </summary>
    public Geom_Surface? Surface()
    {
        // Returns an equivalent non-offset surface if it exists for this offset surface.
        if (offsetValue == 0.0)
        {
            return BasisSurface(); // Direct case - no offset
        }

        double Tol = Precision.Confusion();
        Geom_Surface? Result = null, Base;
        Type TheType = basisSurf.DynamicType();
        bool IsTrimmed;
        double U1 = 0.0, V1 = 0.0, U2 = 0.0, V2 = 0.0;

        // Handle trimmed surfaces - extract the basis surface and bounds.
        if (TheType == typeof(Geom_RectangularTrimmedSurface))
        {
            Geom_RectangularTrimmedSurface S = (Geom_RectangularTrimmedSurface)basisSurf;
            Base = S.BasisSurface();
            TheType = Base.DynamicType();
            S.Bounds(out U1, out U2, out V1, out V2);
            IsTrimmed = true;
        }
        else
        {
            IsTrimmed = false;
            Base = basisSurf;
        }

        // Handle canonical surfaces - compute equivalent offset surface.
        // For direct orientation, offset is along outward normal; for indirect, it's reversed.
        if (TheType == typeof(Geom_Plane))
        {
            Geom_Plane P = (Geom_Plane)Base;
            // Plane normal is already available as Position().Direction().
            gp_Vec T = new gp_Vec(P.Position().Direction());
            T *= offsetValue;
            Result = (Geom_Plane)P.Translated(T);
        }
        else if (TheType == typeof(Geom_CylindricalSurface))
        {
            Geom_CylindricalSurface C = (Geom_CylindricalSurface)Base;
            gp_Ax3 Axis = C.Position();
            double aSign = Axis.Direct() ? 1.0 : -1.0;
            double Radius = C.Radius() + aSign * offsetValue;
            if (Radius >= Tol)
            {
                Result = new Geom_CylindricalSurface(Axis, Radius);
            }
            else if (Radius <= -Tol)
            {
                // Negative radius: flip X-axis to reverse normal orientation.
                Axis.XReverse();
                Result = new Geom_CylindricalSurface(Axis, -Radius);
            }
            // else: degenerate surface - radius is too small.
        }
        else if (TheType == typeof(Geom_ConicalSurface))
        {
            Geom_ConicalSurface C = (Geom_ConicalSurface)Base;
            gp_Ax3 anAxis = C.Position();
            double aSign = anAxis.Direct() ? 1.0 : -1.0;
            double anAlpha = C.SemiAngle();
            double aCos = Math.Cos(anAlpha);
            double aSin = Math.Sin(anAlpha);
            double aRadius = C.RefRadius() + aSign * offsetValue * aCos;
            if (aRadius >= 0.0)
            {
                // Translate apex along axis by offset component.
                gp_Vec aZ = new gp_Vec(anAxis.Direction());
                aZ *= -aSign * offsetValue * aSin;
                anAxis.Translate(aZ);
                Result = new Geom_ConicalSurface(anAxis, anAlpha, aRadius);
            }
            // else: degenerate surface - radius is negative.
        }
        else if (TheType == typeof(Geom_SphericalSurface))
        {
            Geom_SphericalSurface S = (Geom_SphericalSurface)Base;
            gp_Ax3 Axis = S.Position();
            double aSign = Axis.Direct() ? 1.0 : -1.0;
            double Radius = S.Radius() + aSign * offsetValue;
            if (Radius >= Tol)
            {
                Result = new Geom_SphericalSurface(Axis, Radius);
            }
            else if (Radius <= -Tol)
            {
                // Negative radius: flip both X and Z axes to reverse normal orientation.
                Axis.XReverse();
                Axis.ZReverse();
                Result = new Geom_SphericalSurface(Axis, -Radius);
            }
            // else: degenerate surface - radius is too small.
        }
        else if (TheType == typeof(Geom_ToroidalSurface))
        {
            Geom_ToroidalSurface S = (Geom_ToroidalSurface)Base;
            double MajorRadius = S.MajorRadius();
            gp_Ax3 Axis = S.Position();
            double aSign = Axis.Direct() ? 1.0 : -1.0;
            double MinorRadius = S.MinorRadius() + aSign * offsetValue;
            // Only handle non-self-intersecting torus (MinorRadius <= MajorRadius).
            if (MinorRadius >= Tol && MinorRadius <= MajorRadius)
            {
                Result = new Geom_ToroidalSurface(Axis, MajorRadius, MinorRadius);
            }
            else if (MinorRadius <= -Tol && -MinorRadius <= MajorRadius)
            {
                // Negative minor radius: flip X-axis to reverse normal orientation.
                Axis.XReverse();
                Result = new Geom_ToroidalSurface(Axis, MajorRadius, -MinorRadius);
            }
            // else: degenerate or self-intersecting torus - no equivalent surface.
        }

        // Trim the result if the basis surface was trimmed.
        if (IsTrimmed && Result is not null)
        {
            Base = Result;
            Result = new Geom_RectangularTrimmedSurface(Base, U1, U2, V1, V2);
        }

        return Result;
    }

    /// <summary>
    /// If true, UOsculSurf is the local osculating surface along U at (U, V) (frozen, shared), and IsOpposite
    /// tells whether its derivative is opposite to the basis surface's. Both are left unchanged when false.
    /// </summary>
    public bool UOsculatingSurface(double U, double V, ref bool IsOpposite, ref Geom_BSplineSurface? UOsculSurf)
    {
        return myOscSurf is not null && myOscSurf.UOsculatingSurface(U, V, ref IsOpposite, ref UOsculSurf);
    }

    /// <summary>
    /// If true, VOsculSurf is the local osculating surface along V at (U, V) (frozen, shared), and IsOpposite
    /// tells whether its derivative is opposite to the basis surface's. Both are left unchanged when false.
    /// </summary>
    public bool VOsculatingSurface(double U, double V, ref bool IsOpposite, ref Geom_BSplineSurface? VOsculSurf)
    {
        return myOscSurf is not null && myOscSurf.VOsculatingSurface(U, V, ref IsOpposite, ref VOsculSurf);
    }

    /// <summary>The osculating surfaces (null unless the basis is a B-spline or Bezier surface), for GeomAdaptor.</summary>
    internal Geom_OsculatingSurface? OsculatingSurface() => myOscSurf;

    /// <summary>The continuity of the basis surface (G1 for a G1 C0 basis).</summary>
    public GeomAbs_Shape GetBasisSurfContinuity() => myBasisSurfContinuity;

    // Non-mutating twins (OCCT.NET)

    /// <summary>A copy with the offset value D.</summary>
    public Geom_OffsetSurface WithOffsetValue(double D)
    {
        Geom_OffsetSurface aCopy = (Geom_OffsetSurface)Copy();
        aCopy.SetOffsetValue(D);
        return aCopy;
    }

    /// <summary>A copy on the basis surface S (see <see cref="SetBasisSurface"/>).</summary>
    public Geom_OffsetSurface WithBasisSurface(Geom_Surface S, bool isNotCheckC0 = false)
    {
        Geom_OffsetSurface aCopy = (Geom_OffsetSurface)Copy();
        aCopy.SetBasisSurface(S, isNotCheckC0);
        return aCopy;
    }

    /// <summary>A copy evaluated through a copy of theDesc (null: without a representation).</summary>
    public Geom_OffsetSurface WithEvalRepresentation(GeomEval_RepSurfaceDesc.Base? theDesc)
    {
        GeomEval_RepUtils.ValidateSurfaceDesc(theDesc, this);
        Geom_OffsetSurface aCopy = (Geom_OffsetSurface)Copy();
        aCopy.SetEvalRepresentation(GeomEval_RepUtils.CloneSurfaceDesc(theDesc));
        return aCopy;
    }

    /// <summary>A copy without an evaluation representation (the twin of ClearEvalRepresentation).</summary>
    public Geom_OffsetSurface WithoutEvalRepresentation()
    {
        Geom_OffsetSurface aCopy = (Geom_OffsetSurface)Copy();
        aCopy.ClearEvalRepresentation();
        return aCopy;
    }
}
