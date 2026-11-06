// Port of: src/ModelingData/TKG3d/Geom/Geom_OffsetSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_OffsetSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;

namespace Kerf.Geometry;

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
/// internal mutators then copy it before changing it.</para>
/// </remarks>
public sealed class OffsetSurface : Surface
{
    private static readonly double MyAngularToleranceForG1 = global::Kerf.Engine.Foundation.Precision.Angular();

    private Surface _basisSurf = null!;
    private RepSurfaceDesc.Base? _evalRep;
    private double _offsetValue;
    private OsculatingSurface? _oscSurf;
    private Continuity _basisSurfContinuity;

    private static RepSurfaceDesc.Base? MakeFullSurfaceRep(Surface? surface)
    {
        if (surface is null)
        {
            return null;
        }
        RepSurfaceDesc.Full desc = new RepSurfaceDesc.Full();
        desc.Representation = surface;
        return desc;
    }

    private static Surface? DirectRepSurface(OffsetSurface surface)
    {
        RepSurfaceDesc.Base? desc = surface.EvalRepresentation();
        if (desc is null)
        {
            return null;
        }
        if (desc.GetKind() != RepSurfaceDesc.Base.Kind.Full)
        {
            return null;
        }
        return desc.Representation;
    }

    /// <summary>
    /// The surface at distance theOffset from theSurf along its normal (a copy of theSurf; trimmed and offset
    /// surfaces are flattened, their offsets added).
    /// </summary>
    /// <exception cref="ConstructionErrorException">
    /// The basis surface is C0 and not G1 (only checked for B-spline and swept surfaces), unless isNotCheckC0.
    /// </exception>
    public OffsetSurface(Surface surface, double offset, bool skipContinuityCheck = false)
    {
        _offsetValue = offset;
        SetBasisSurface(surface, skipContinuityCheck);
    }

    /// <summary>A copy of theOther (deep: the basis surface is copied), without validation.</summary>
    public OffsetSurface(OffsetSurface other)
    {
        _basisSurf = (Surface)other._basisSurf.Copy();
        _evalRep = RepUtils.CloneSurfaceDesc(other._evalRep);
        _offsetValue = other._offsetValue;
        _oscSurf = other._oscSurf is not null ? new OsculatingSurface(other._oscSurf) : null;
        _basisSurfContinuity = other._basisSurfContinuity;
        // Deep copy without validation - source surface is already validated.
    }

    /// <summary>True if an evaluation representation is attached.</summary>
    internal bool HasEvalRepresentation => _evalRep is not null;

    /// <summary>The evaluation representation descriptor (may be null). Stored, not copied: don't modify.</summary>
    internal RepSurfaceDesc.Base? EvalRepresentation() => _evalRep;

    /// <summary>Sets a new evaluation representation, after checking it (no circular reference).</summary>
    internal void SetEvalRepresentation(RepSurfaceDesc.Base? desc)
    {
        CheckMutable();
        RepUtils.ValidateSurfaceDesc(desc, this);
        _evalRep = desc;
    }

    /// <summary>Removes the evaluation representation.</summary>
    internal void ClearEvalRepresentation()
    {
        CheckMutable();
        _evalRep = null;
    }

    /// <summary>A new, unfrozen surface equal to this one (on a copy of the basis surface).</summary>
    public sealed override Geometry3D Copy() => new OffsetSurface(this);

    /// <summary>
    /// Changes the basis surface (a copy of S, flattened as in the constructor); the evaluation representation is
    /// recomputed, and so are the osculating surfaces of a B-spline or Bezier basis.
    /// </summary>
    /// <exception cref="ConstructionErrorException">As the constructor.</exception>
    internal void SetBasisSurface(Surface s, bool isNotCheckC0 = false)
    {
        CheckMutable();
        ClearEvalRepresentation();
        s.ParameterBounds(out double uf, out double ul, out double vf, out double vl);

        Surface checkingSurf = (Surface)s.Copy();
        bool isTrimmed = false;

        while (checkingSurf is TrimmedSurface || checkingSurf is OffsetSurface)
        {
            if (checkingSurf is TrimmedSurface trimS)
            {
                checkingSurf = trimS.BasisSurface;
                isTrimmed = true;
            }

            if (checkingSurf is OffsetSurface os)
            {
                checkingSurf = os.BasisSurface;
                _offsetValue += os.Offset;
            }
        }

        _basisSurfContinuity = checkingSurf.Continuity;

        bool isC0 = !isNotCheckC0 && (_basisSurfContinuity == C0);

        // Basis surface must be at least C1
        if (isC0)
        {
            Curve3D? curve = null;

            if (checkingSurf is RevolutionSurface revSurf)
            {
                curve = revSurf.BasisCurve;
            }
            else if (checkingSurf is ExtrusionSurface leSurf)
            {
                curve = leSurf.BasisCurve;
            }

            if (curve is not null)
            {
                while (curve is TrimmedCurve3D || curve is OffsetCurve3D)
                {
                    if (curve is TrimmedCurve3D trimC)
                    {
                        curve = trimC.BasisCurve;
                    }

                    if (curve is OffsetCurve3D oc)
                    {
                        curve = oc.BasisCurve;
                    }
                }
            }

            double uIsoPar = (uf + ul) / 2.0, vIsoPar = (vf + vl) / 2.0;
            bool isUG1, isVG1;

            Curve3D curv1 = curve is null ? checkingSurf.UIso(uIsoPar) : curve;
            Curve3D curv2 = checkingSurf.VIso(vIsoPar);
            isUG1 = curv1 is not BSplineCurve3D;
            isVG1 = curv2 is not BSplineCurve3D;

            if (!isUG1)
            {
                BSplineCurve3D bc = (BSplineCurve3D)curv1;
                isUG1 = bc.IsG1(vf, vl, MyAngularToleranceForG1);
            }
            //
            if (!isVG1)
            {
                BSplineCurve3D bc = (BSplineCurve3D)curv2;
                isVG1 = bc.IsG1(uf, ul, MyAngularToleranceForG1);
            }
            //
            if (isUG1 && isVG1)
            {
                _basisSurfContinuity = G1;
                isC0 = false;
            }

            // Raise exception if still C0
            if (isC0)
            {
                throw Refusal.Argument(new ConstructionErrorException("Offset with no C1 Surface"));
            }
        }

        if (isTrimmed)
        {
            _basisSurf = new TrimmedSurface(checkingSurf, uf, ul, vf, vl);
        }
        else
        {
            _basisSurf = checkingSurf;
        }

        Surface? surfaceRep = Surface();
        SetEvalRepresentation(MakeFullSurfaceRep(surfaceRep));

        if (checkingSurf is BSplineSurface || checkingSurf is BezierSurface)
        {
            // Hard-coded tolerance for now, but it should be offered in the constructor
            // and stored as a field. It could be used, for example, for iso-curve extraction
            // and also for singularity detection. For osculating surfaces, it is used to
            // detect whether an iso-curve is degenerate.
            double tol = global::Kerf.Engine.Foundation.Precision.Confusion(); // 0.0001;
            _oscSurf = new OsculatingSurface(checkingSurf, tol);
        }
    }

    /// <summary>Changes the offset value; the evaluation representation is recomputed.</summary>
    internal void SetOffsetValue(double d)
    {
        CheckMutable();
        ClearEvalRepresentation();
        _offsetValue = d;
        SetEvalRepresentation(MakeFullSurfaceRep(Surface()));
    }

    /// <summary>The offset value.</summary>
    public double Offset => _offsetValue;

    /// <summary>The basis surface, frozen (it is shared with this offset surface; decision 211).</summary>
    public Surface BasisSurface
    {
        get
        {
            _basisSurf.Freeze();
            return _basisSurf;
        }
    }

    // Kerf: the basis surface may have been handed out (and frozen) by BasisSurface(); copy it before
    // changing it.
    private void OwnBasisSurface()
    {
        if (_basisSurf.IsFrozen)
        {
            _basisSurf = (Surface)_basisSurf.Copy();
        }
    }

    /// <summary>Reverses U on the basis surface and the sign of the offset (drops the evaluation representation).</summary>
    protected internal sealed override void UReverse()
    {
        CheckMutable();
        // OCCT: UReverse, VReverse and Transform drop the equivalent-surface representation without recomputing it
        // (SetOffsetValue and SetBasisSurface do recompute it): see occt-issues.md.
        ClearEvalRepresentation();
        OwnBasisSurface();
        _basisSurf.UReverse();
        _offsetValue = -_offsetValue;
    }

    /// <summary>The basis surface's reversed U parameter.</summary>
    public sealed override double UReversedParameter(double u) => _basisSurf.UReversedParameter(u);

    /// <summary>Reverses V on the basis surface and the sign of the offset (drops the evaluation representation).</summary>
    protected internal sealed override void VReverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        OwnBasisSurface();
        _basisSurf.VReverse();
        _offsetValue = -_offsetValue;
    }

    /// <summary>The basis surface's reversed V parameter.</summary>
    public sealed override double VReversedParameter(double v) => _basisSurf.VReversedParameter(v);

    /// <summary>The bounds of the basis surface.</summary>
    internal sealed override void ParameterBounds(out double u1, out double u2, out double v1, out double v2) =>
        _basisSurf.ParameterBounds(out u1, out u2, out v1, out v2);

    /// <summary>The continuity of the basis surface lowered by one (C0 below C2).</summary>
    public sealed override Continuity Continuity
    {
        get
        {
            switch (_basisSurfContinuity)
            {
                case C2:
                    return C1;
                case C3:
                    return C2;
                case CN:
                    return CN;
                default:
                    break;
            }
            return C0;
        }
    }

    /// <exception cref="UndefinedValue3DException">The point is undefined at (U, V).</exception>
    internal sealed override Point3D EvalD0(double u, double v)
    {
        if (RepUtils.TryEvalSurfaceD0(_evalRep, u, v, out Point3D evalRepResult))
        {
            return evalRepResult;
        }

        if (!OffsetSurfaceUtils.EvaluateD0(u,
                                                v,
                                                new OffsetSurfaceUtils.GeomSurfaceBasis(_basisSurf),
                                                _offsetValue,
                                                _oscSurf,
                                                out Point3D p))
        {
            throw Refusal.State(new UndefinedValue3DException("Geom_OffsetSurface::EvalD0"));
        }
        return p;
    }

    /// <exception cref="UndefinedDerivative3DException">The derivatives are undefined at (U, V).</exception>
    internal sealed override ResD1 EvalD1(double u, double v)
    {
        if (RepUtils.TryEvalSurfaceD1(_evalRep, u, v, out ResD1 evalRepResult))
        {
            return evalRepResult;
        }

        ResD1 result;
        if (!OffsetSurfaceUtils.EvaluateD1(u,
                                                v,
                                                new OffsetSurfaceUtils.GeomSurfaceBasis(_basisSurf),
                                                _offsetValue,
                                                _oscSurf,
                                                out result.Point,
                                                out result.D1U,
                                                out result.D1V))
        {
            throw Refusal.State(new UndefinedDerivative3DException("Geom_OffsetSurface::EvalD1"));
        }
        return result;
    }

    /// <exception cref="UndefinedDerivative3DException">The derivatives are undefined at (U, V).</exception>
    internal sealed override ResD2 EvalD2(double u, double v)
    {
        if (RepUtils.TryEvalSurfaceD2(_evalRep, u, v, out ResD2 evalRepResult))
        {
            return evalRepResult;
        }

        ResD2 result;
        if (!OffsetSurfaceUtils.EvaluateD2(u,
                                                v,
                                                new OffsetSurfaceUtils.GeomSurfaceBasis(_basisSurf),
                                                _offsetValue,
                                                _oscSurf,
                                                out result.Point,
                                                out result.D1U,
                                                out result.D1V,
                                                out result.D2U,
                                                out result.D2V,
                                                out result.D2UV))
        {
            throw Refusal.State(new UndefinedDerivative3DException("Geom_OffsetSurface::EvalD2"));
        }
        return result;
    }

    /// <exception cref="UndefinedDerivative3DException">The derivatives are undefined at (U, V).</exception>
    internal sealed override ResD3 EvalD3(double u, double v)
    {
        if (RepUtils.TryEvalSurfaceD3(_evalRep, u, v, out ResD3 evalRepResult))
        {
            return evalRepResult;
        }

        ResD3 result;
        if (!OffsetSurfaceUtils.EvaluateD3(u,
                                                v,
                                                new OffsetSurfaceUtils.GeomSurfaceBasis(_basisSurf),
                                                _offsetValue,
                                                _oscSurf,
                                                out result.Point,
                                                out result.D1U,
                                                out result.D1V,
                                                out result.D2U,
                                                out result.D2V,
                                                out result.D2UV,
                                                out result.D3U,
                                                out result.D3V,
                                                out result.D3UUV,
                                                out result.D3UVV))
        {
            throw Refusal.State(new UndefinedDerivative3DException("Geom_OffsetSurface::EvalD3"));
        }
        return result;
    }

    /// <exception cref="UndefinedDerivative3DException">
    /// Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0, or the derivative is undefined at (U, V).
    /// </exception>
    internal sealed override Vector3D EvalDN(double u, double v, int uOrder, int vOrder)
    {
        if (uOrder + vOrder < 1 || uOrder < 0 || vOrder < 0)
        {
            throw Refusal.State(new UndefinedDerivative3DException("Geom_OffsetSurface::EvalDN"));
        }
        if (RepUtils.TryEvalSurfaceDN(_evalRep, u, v, uOrder, vOrder, out Vector3D evalRepResult))
        {
            return evalRepResult;
        }

        if (!OffsetSurfaceUtils.EvaluateDN(u,
                                                v,
                                                uOrder,
                                                vOrder,
                                                new OffsetSurfaceUtils.GeomSurfaceBasis(_basisSurf),
                                                _offsetValue,
                                                _oscSurf,
                                                out Vector3D result))
        {
            throw Refusal.State(new UndefinedDerivative3DException("Geom_OffsetSurface::EvalDN"));
        }
        return result;
    }

    ////*************************************************
    ////
    ////   EVALUATOR FOR THE ISO-CURVE APPROXIMATION
    ////
    ////*************************************************

    private sealed class Geom_OffsetSurface_UIsoEvaluator : EvaluatorFunction
    {
        private readonly Surface _surface;
        private readonly double _isoPar;

        public Geom_OffsetSurface_UIsoEvaluator(Surface surface, double u)
        {
            _surface = surface;
            _isoPar = u;
        }

        public override void Evaluate(int dimension, ReadOnlySpan<double> startEnd, double parameter,
                                      int derivativeRequest, Span<double> result, out int returnCode)
        {
            Point3D p;
            if (derivativeRequest == 0)
            {
                p = _surface.Value(_isoPar, parameter);
                result[0] = p.X;
                result[1] = p.Y;
                result[2] = p.Z;
            }
            else
            {
                _surface.D1(_isoPar, parameter, out p, out Vector3D du, out Vector3D dv);
                result[0] = dv.X;
                result[1] = dv.Y;
                result[2] = dv.Z;
            }
            returnCode = 0;
        }
    }

    private sealed class Geom_OffsetSurface_VIsoEvaluator : EvaluatorFunction
    {
        private readonly Surface _surface;
        private readonly double _isoPar;

        public Geom_OffsetSurface_VIsoEvaluator(Surface surface, double v)
        {
            _surface = surface;
            _isoPar = v;
        }

        public override void Evaluate(int dimension, ReadOnlySpan<double> startEnd, double parameter,
                                      int derivativeRequest, Span<double> result, out int returnCode)
        {
            Point3D p;
            if (derivativeRequest == 0)
            {
                p = _surface.Value(parameter, _isoPar);
                result[0] = p.X;
                result[1] = p.Y;
                result[2] = p.Z;
            }
            else
            {
                _surface.D1(parameter, _isoPar, out p, out Vector3D du, out Vector3D dv);
                result[0] = du.X;
                result[1] = du.Y;
                result[2] = du.Z;
            }
            returnCode = 0;
        }
    }

    /// <summary>
    /// The U iso-curve: the equivalent surface's, a translated iso-curve of an extrusion basis, or else a C1
    /// B-spline approximation (degree at most 14, 100 segments, tolerance Precision::Approximation()).
    /// </summary>
    /// <exception cref="ConstructionErrorException">The approximation fails.</exception>
    public sealed override Curve3D UIso(double u)
    {
        Surface? equivSurface = DirectRepSurface(this);
        if (equivSurface is null)
        {
            SurfaceAdaptor gAsurf = new SurfaceAdaptor(_basisSurf);
            if (gAsurf.GetType() == SurfaceKind.SurfaceOfExtrusion)
            {
                Curve3D l = _basisSurf.UIso(u);

                _basisSurf.D1(u, 0.0, out Point3D p, out Vector3D d1U, out Vector3D d1V);
                Vector3D dir = d1U.Crossed(d1V);
                if (dir.SquareMagnitude < Constants.Resolution())
                {
                    return l;
                }
                dir = dir.Normalized();
                dir *= _offsetValue;

                l.Translate(dir);
                return l;
            }
            const int num1 = 0, num2 = 0, num3 = 1;
            NCollection_HArray1<double>? t1 = null, t2 = null;
            NCollection_HArray1<double> t3 = new(1, num3);
            t3.Init(global::Kerf.Engine.Foundation.Precision.Approximation());
            ParameterBounds(out double u1, out double u2, out double v1, out double v2);
            const Continuity cont = C1;
            const int maxSeg = 100, maxDeg = 14;

            Geom_OffsetSurface_UIsoEvaluator ev = new(this, u);
            ApproxAFunction approx = new(num1, num2, num3, t1, t2, t3, v1, v2, cont, maxDeg, maxSeg, ev);

            Refusal.ArgumentIf(!approx.IsDone, FailureKind.ConstructionError, " Geom_OffsetSurface : UIso");

            int nbPoles = approx.NbPoles();

            NCollection_Array1<Point3D> poles = new(1, nbPoles);

            approx.Poles(1, poles);
            NCollection_Array1<double> knots = new(approx.Knots()!);
            NCollection_Array1<int> mults = new(approx.Multiplicities()!);

            BSplineCurve3D c = new BSplineCurve3D(poles, knots, mults, approx.Degree());
            return c;
        }
        return equivSurface.UIso(u);
    }

    /// <summary>
    /// The V iso-curve: the equivalent surface's, or else a C1 B-spline approximation (degree at most 14, 100
    /// segments, tolerance Precision::Approximation()).
    /// </summary>
    /// <exception cref="ConstructionErrorException">The approximation fails.</exception>
    public sealed override Curve3D VIso(double v)
    {
        Surface? equivSurface = DirectRepSurface(this);
        if (equivSurface is null)
        {
            const int num1 = 0, num2 = 0, num3 = 1;
            NCollection_HArray1<double>? t1 = null, t2 = null;
            NCollection_HArray1<double> t3 = new(1, num3);
            t3.Init(global::Kerf.Engine.Foundation.Precision.Approximation());
            ParameterBounds(out double u1, out double u2, out double v1, out double v2);
            const Continuity cont = C1;
            const int maxSeg = 100, maxDeg = 14;

            Geom_OffsetSurface_VIsoEvaluator ev = new(this, v);
            ApproxAFunction approx = new(num1, num2, num3, t1, t2, t3, u1, u2, cont, maxDeg, maxSeg, ev);

            Refusal.ArgumentIf(!approx.IsDone, FailureKind.ConstructionError, " Geom_OffsetSurface : VIso");

            NCollection_Array1<Point3D> poles = new(1, approx.NbPoles());

            approx.Poles(1, poles);
            NCollection_Array1<double> knots = new(approx.Knots()!);
            NCollection_Array1<int> mults = new(approx.Multiplicities()!);

            BSplineCurve3D c = new BSplineCurve3D(poles, knots, mults, approx.Degree());
            return c;
        }
        return equivSurface.VIso(v);
    }

    /// <summary>True if the basis surface is C(N + 1) in U.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    internal sealed override bool IsCNu(int n)
    {
        Refusal.RangeIf(n < 0, FailureKind.RangeError, " ");
        return _basisSurf.IsCNu(n + 1);
    }

    /// <summary>True if the basis surface is C(N + 1) in V.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    internal sealed override bool IsCNv(int n)
    {
        Refusal.RangeIf(n < 0, FailureKind.RangeError, " ");
        return _basisSurf.IsCNv(n + 1);
    }

    public sealed override bool IsUPeriodic => _basisSurf.IsUPeriodic;

    /// <summary>The U period of the basis surface.</summary>
    public sealed override double UPeriod => _basisSurf.UPeriod;

    public sealed override bool IsVPeriodic => _basisSurf.IsVPeriodic;

    /// <summary>The V period of the basis surface.</summary>
    public sealed override double VPeriod => _basisSurf.VPeriod;

    /// <summary>
    /// The U closure from the kind of basis surface (OCCT doesn't evaluate the surface): an elementary surface's
    /// (trimmed or not), a surface of revolution (its trimmed U closure), an extrusion of a circle or an ellipse;
    /// false otherwise.
    /// </summary>
    public sealed override bool IsUClosed
    {
        get
        {
            bool uClosed;
            Surface sBasis = _basisSurf;

            if (sBasis is TrimmedSurface st)
            {
                Surface s = st.BasisSurface;
                if (s is ElementarySurface)
                {
                    uClosed = sBasis.IsUClosed;
                }
                else if (s is ExtrusionSurface extru)
                {
                    Curve3D c = extru.BasisCurve;
                    if (c is CircleCurve3D || c is EllipseCurve3D)
                    {
                        uClosed = sBasis.IsUClosed;
                    }
                    else
                    {
                        uClosed = false;
                    }
                }
                else if (s is RevolutionSurface)
                {
                    uClosed = sBasis.IsUClosed;
                }
                else
                {
                    uClosed = false;
                }
            }
            else
            {
                if (sBasis is ElementarySurface)
                {
                    uClosed = sBasis.IsUClosed;
                }
                else if (sBasis is ExtrusionSurface extru)
                {
                    Curve3D c = extru.BasisCurve;
                    uClosed = (c is CircleCurve3D || c is EllipseCurve3D);
                }
                else if (sBasis is RevolutionSurface)
                {
                    uClosed = true;
                }
                else
                {
                    uClosed = false;
                }
            }
            return uClosed;
        }
    }

    /// <summary>The V closure of an elementary basis surface (trimmed or not); false otherwise.</summary>
    public sealed override bool IsVClosed
    {
        get
        {
            bool vClosed;
            Surface sBasis = _basisSurf;

            if (sBasis is TrimmedSurface st)
            {
                Surface s = st.BasisSurface;
                if (s is ElementarySurface)
                {
                    vClosed = sBasis.IsVClosed;
                }
                else
                {
                    vClosed = false;
                }
            }
            else
            {
                if (sBasis is ElementarySurface)
                {
                    vClosed = sBasis.IsVClosed;
                }
                else
                {
                    vClosed = false;
                }
            }
            return vClosed;
        }
    }

    /// <summary>
    /// Applies T to the basis surface; the offset is scaled by T.ScaleFactor() (drops the evaluation
    /// representation, as OCCT does).
    /// </summary>
    protected internal sealed override void Transform(in Transform3D transform)
    {
        CheckMutable();
        ClearEvalRepresentation();
        OwnBasisSurface();
        _basisSurf.Transform(transform);
        _offsetValue *= transform.ScaleFactor;
    }

    /// <summary>
    /// The basis surface's transformed parameters: the offset surface has its parametrization (OCCT transformed them
    /// again by the equivalent surface when there is one; Kerf decision 1489).
    /// </summary>
    internal sealed override void TransformParameters(ref double u, ref double v, in Transform3D transform)
    {
        _basisSurf.TransformParameters(ref u, ref v, transform);
    }

    /// <summary>The basis surface's parametric transformation.</summary>
    internal sealed override GeneralTransform2D ParametricTransformation(in Transform3D transform) => _basisSurf.ParametricTransformation(transform);

    /// <summary>
    /// An equivalent non-offset surface if there is one: the basis surface itself (frozen) for a zero offset,
    /// and for an offset of a plane, cylinder, cone, sphere or (non-self-intersecting) torus, trimmed or not,
    /// the surface of that kind at the offset; null otherwise (or when the result degenerates).
    /// </summary>
    public Surface? Surface()
    {
        // Returns an equivalent non-offset surface if it exists for this offset surface.
        if (_offsetValue == 0.0)
        {
            return BasisSurface; // Direct case - no offset
        }

        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        Surface? result = null, Base;
        Type theType = _basisSurf.DynamicType();
        bool isTrimmed;
        double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;

        // Handle trimmed surfaces - extract the basis surface and bounds.
        if (theType == typeof(TrimmedSurface))
        {
            TrimmedSurface s = (TrimmedSurface)_basisSurf;
            Base = s.BasisSurface;
            theType = Base.DynamicType();
            s.ParameterBounds(out u1, out u2, out v1, out v2);
            isTrimmed = true;
        }
        else
        {
            isTrimmed = false;
            Base = _basisSurf;
        }

        // Handle canonical surfaces - compute equivalent offset surface.
        // For direct orientation, offset is along outward normal; for indirect, it's reversed.
        if (theType == typeof(PlaneSurface))
        {
            PlaneSurface p = (PlaneSurface)Base;
            // Plane normal is already available as Position().Direction().
            Vector3D t = new Vector3D(p.Position.Direction);
            t *= _offsetValue;
            result = (PlaneSurface)p.Translated(t);
        }
        else if (theType == typeof(CylindricalSurface))
        {
            CylindricalSurface c = (CylindricalSurface)Base;
            CoordinateSystem Axis = c.Position;
            double sign = Axis.IsDirect ? 1.0 : -1.0;
            double Radius = c.Radius + sign * _offsetValue;
            if (Radius >= tol)
            {
                result = new CylindricalSurface(Axis, Radius);
            }
            else if (Radius <= -tol)
            {
                // Negative radius: flip X-axis to reverse normal orientation.
                Axis = Axis.XReversed();
                result = new CylindricalSurface(Axis, -Radius);
            }
            // else: degenerate surface - radius is too small.
        }
        else if (theType == typeof(ConicalSurface))
        {
            ConicalSurface c = (ConicalSurface)Base;
            CoordinateSystem anAxis = c.Position;
            double sign = anAxis.IsDirect ? 1.0 : -1.0;
            double alpha = c.SemiAngle;
            double cos = Math.Cos(alpha);
            double sin = Math.Sin(alpha);
            double aRadius = c.RefRadius + sign * _offsetValue * cos;
            if (aRadius >= 0.0)
            {
                // Translate apex along axis by offset component.
                Vector3D z = new Vector3D(anAxis.Direction);
                z *= -sign * _offsetValue * sin;
                anAxis = anAxis.Translated(z);
                result = new ConicalSurface(anAxis, alpha, aRadius);
            }
            // else: degenerate surface - radius is negative.
        }
        else if (theType == typeof(SphericalSurface))
        {
            SphericalSurface s = (SphericalSurface)Base;
            CoordinateSystem axis = s.Position;
            double sign = axis.IsDirect ? 1.0 : -1.0;
            double radius = s.Radius + sign * _offsetValue;
            if (radius >= tol)
            {
                result = new SphericalSurface(axis, radius);
            }
            else if (radius <= -tol)
            {
                // Negative radius: flip both X and Z axes to reverse normal orientation.
                axis = axis.XReversed();
                axis = axis.ZReversed();
                result = new SphericalSurface(axis, -radius);
            }
            // else: degenerate surface - radius is too small.
        }
        else if (theType == typeof(ToroidalSurface))
        {
            ToroidalSurface s = (ToroidalSurface)Base;
            double majorRadius = s.MajorRadius;
            CoordinateSystem axis = s.Position;
            double sign = axis.IsDirect ? 1.0 : -1.0;
            double minorRadius = s.MinorRadius + sign * _offsetValue;
            // Only handle non-self-intersecting torus (MinorRadius <= MajorRadius).
            if (minorRadius >= tol && minorRadius <= majorRadius)
            {
                result = new ToroidalSurface(axis, majorRadius, minorRadius);
            }
            else if (minorRadius <= -tol && -minorRadius <= majorRadius)
            {
                // Negative minor radius: flip X-axis to reverse normal orientation.
                axis = axis.XReversed();
                result = new ToroidalSurface(axis, majorRadius, -minorRadius);
            }
            // else: degenerate or self-intersecting torus - no equivalent surface.
        }

        // Trim the result if the basis surface was trimmed.
        if (isTrimmed && result is not null)
        {
            Base = result;
            result = new TrimmedSurface(Base, u1, u2, v1, v2);
        }

        return result;
    }

    /// <summary>
    /// If true, UOsculSurf is the local osculating surface along U at (U, V) (frozen, shared), and IsOpposite
    /// tells whether its derivative is opposite to the basis surface's. Both are left unchanged when false.
    /// </summary>
    internal bool UOsculatingSurface(double u, double v, ref bool isOpposite, ref BSplineSurface? uOsculSurf)
    {
        return _oscSurf is not null && _oscSurf.UOsculatingSurface(u, v, ref isOpposite, ref uOsculSurf);
    }

    /// <summary>
    /// If true, VOsculSurf is the local osculating surface along V at (U, V) (frozen, shared), and IsOpposite
    /// tells whether its derivative is opposite to the basis surface's. Both are left unchanged when false.
    /// </summary>
    internal bool VOsculatingSurface(double u, double v, ref bool isOpposite, ref BSplineSurface? vOsculSurf)
    {
        return _oscSurf is not null && _oscSurf.VOsculatingSurface(u, v, ref isOpposite, ref vOsculSurf);
    }

    /// <summary>The osculating surfaces (null unless the basis is a B-spline or Bezier surface), for GeomAdaptor.</summary>
    internal OsculatingSurface? OsculatingSurface() => _oscSurf;

    /// <summary>The continuity of the basis surface (G1 for a G1 C0 basis).</summary>
    internal Continuity GetBasisSurfContinuity() => _basisSurfContinuity;

    // Non-mutating twins (Kerf)

    /// <summary>A copy with the offset value D.</summary>
    public OffsetSurface WithOffsetValue(double d)
    {
        OffsetSurface copy = (OffsetSurface)Copy();
        copy.SetOffsetValue(d);
        return copy;
    }

    /// <summary>A copy on the basis surface S (see <see cref="SetBasisSurface"/>).</summary>
    public OffsetSurface WithBasisSurface(Surface surface, bool skipContinuityCheck = false)
    {
        OffsetSurface copy = (OffsetSurface)Copy();
        copy.SetBasisSurface(surface, skipContinuityCheck);
        return copy;
    }

    /// <summary>A copy evaluated through a copy of theDesc (null: without a representation).</summary>
    internal OffsetSurface WithEvalRepresentation(RepSurfaceDesc.Base? desc)
    {
        RepUtils.ValidateSurfaceDesc(desc, this);
        OffsetSurface copy = (OffsetSurface)Copy();
        copy.SetEvalRepresentation(RepUtils.CloneSurfaceDesc(desc));
        return copy;
    }

    /// <summary>A copy without an evaluation representation (the twin of ClearEvalRepresentation).</summary>
    internal OffsetSurface WithoutEvalRepresentation()
    {
        OffsetSurface copy = (OffsetSurface)Copy();
        copy.ClearEvalRepresentation();
        return copy;
    }
}
