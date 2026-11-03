// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_RepCurveDesc.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// Descriptors of an evaluation representation of a 2D curve: another curve used to evaluate it (for instance an
/// exact analytic form of a B-spline), possibly limited in derivative order, domain and through an affine
/// parameter map. A C++ namespace in OCCT.
/// </summary>
public static class Geom2dEval_RepCurveDesc
{
    /// <summary>1D affine parameter map: uRep = Scale * u + Offset.</summary>
    public struct Map1d
    {
        public double Scale;
        public double Offset;

        /// <summary>The identity map, as OCCT's default member initializers.</summary>
        public Map1d()
        {
            Scale = 1.0;
            Offset = 0.0;
        }

        public readonly bool IsIdentity() =>
            Math.Abs(Scale - 1.0) < global::Kerf.Engine.Foundation.Precision.Confusion() && Math.Abs(Offset) < global::Kerf.Engine.Foundation.Precision.Confusion();

        public readonly bool IsValid() => Math.Abs(Scale) > global::Kerf.Engine.Foundation.Precision.Confusion();

        public readonly double Map(double theU) => Scale * theU + Offset;
    }

    /// <summary>1D parameter domain interval.</summary>
    public struct Domain1d
    {
        public double First;
        public double Last;

        /// <summary>The interval [0, 1], as OCCT's default member initializers.</summary>
        public Domain1d()
        {
            First = 0.0;
            Last = 1.0;
        }

        public readonly bool Contains(double theU) =>
            (theU >= First - global::Kerf.Engine.Foundation.Precision.Confusion()) && (theU <= Last + global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Abstract base descriptor: the representation and a kind tag for dispatch.</summary>
    public abstract class Base : Transient
    {
        /// <summary>Descriptor kind for switch-based dispatch.</summary>
        public enum Kind
        {
            /// <summary>Fully equivalent: no derivative limit, no domain, no map.</summary>
            Full,
            /// <summary>Full domain and identity map, limited to MaxDerivOrder.</summary>
            DerivBounded,
            /// <summary>MaxDerivOrder, an optional domain and a parameter map.</summary>
            Mapped,
        }

        /// <summary>The geometry used for evaluation.</summary>
        public Geom2d_Curve? Representation;

        /// <summary>The descriptor kind.</summary>
        public abstract Kind GetKind();

        protected Base() { }
    }

    /// <summary>Fully equivalent descriptor: evaluation delegates directly to the representation.</summary>
    public sealed class Full : Base
    {
        public override Kind GetKind() => Kind.Full;
    }

    /// <summary>Derivative-bounded descriptor: full domain, identity map, limited to MaxDerivOrder.</summary>
    public sealed class DerivBounded : Base
    {
        /// <summary>The maximum supported derivative order (OCCT: <c>std::size_t</c>).</summary>
        public int MaxDerivOrder = 3;

        public override Kind GetKind() => Kind.DerivBounded;
    }

    /// <summary>
    /// Mapped descriptor: an optional bounded domain and an affine parameter map. Evaluation checks the domain,
    /// maps the parameter, evaluates and scales the derivatives.
    /// </summary>
    public sealed class Mapped : Base
    {
        /// <summary>The maximum supported derivative order (OCCT: <c>std::size_t</c>).</summary>
        public int MaxDerivOrder = 3;

        /// <summary>The domain; null for the full domain.</summary>
        public Domain1d? Domain;

        /// <summary>The affine parameter transform.</summary>
        public Map1d ParamMap = new Map1d();

        public override Kind GetKind() => Kind.Mapped;
    }
}
