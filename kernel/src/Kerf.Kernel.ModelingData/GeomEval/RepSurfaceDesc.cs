// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_RepSurfaceDesc.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// Descriptors of an evaluation representation of a surface: another surface used to evaluate it, possibly
/// limited in derivative order, domain and through a diagonal affine parameter map with an optional UV swap.
/// A C++ namespace in OCCT.
/// </summary>
internal static class RepSurfaceDesc
{
    /// <summary>
    /// 2D diagonal affine parameter map with optional UV swap. Without swap: uRep = ScaleU*u + OffsetU,
    /// vRep = ScaleV*v + OffsetV. With swap: uRep = ScaleU*v + OffsetU, vRep = ScaleV*u + OffsetV.
    /// </summary>
    public struct Map2d
    {
        public double ScaleU;
        public double OffsetU;
        public double ScaleV;
        public double OffsetV;
        public bool SwapUV;

        /// <summary>The identity map, as OCCT's default member initializers.</summary>
        public Map2d()
        {
            ScaleU = 1.0;
            OffsetU = 0.0;
            ScaleV = 1.0;
            OffsetV = 0.0;
            SwapUV = false;
        }

        public readonly bool IsIdentity =>
            Math.Abs(ScaleU - 1.0) < global::Kerf.Engine.Foundation.Precision.Confusion()
            && Math.Abs(ScaleV - 1.0) < global::Kerf.Engine.Foundation.Precision.Confusion()
            && Math.Abs(OffsetU) < global::Kerf.Engine.Foundation.Precision.Confusion()
            && Math.Abs(OffsetV) < global::Kerf.Engine.Foundation.Precision.Confusion() && !SwapUV;

        public readonly bool IsValid =>
            Math.Abs(ScaleU) > global::Kerf.Engine.Foundation.Precision.Confusion() && Math.Abs(ScaleV) > global::Kerf.Engine.Foundation.Precision.Confusion();

        public readonly void Map(double u, double v, out double uRep, out double vRep)
        {
            if (SwapUV)
            {
                uRep = ScaleU * v + OffsetU;
                vRep = ScaleV * u + OffsetV;
            }
            else
            {
                uRep = ScaleU * u + OffsetU;
                vRep = ScaleV * v + OffsetV;
            }
        }
    }

    /// <summary>2D parameter domain.</summary>
    public struct Domain2d
    {
        public double UFirst;
        public double ULast;
        public double VFirst;
        public double VLast;

        /// <summary>The square [0, 1] × [0, 1], as OCCT's default member initializers.</summary>
        public Domain2d()
        {
            UFirst = 0.0;
            ULast = 1.0;
            VFirst = 0.0;
            VLast = 1.0;
        }

        public readonly bool Contains(double u, double v) =>
            (u >= UFirst - global::Kerf.Engine.Foundation.Precision.Confusion()) && (u <= ULast + global::Kerf.Engine.Foundation.Precision.Confusion())
            && (v >= VFirst - global::Kerf.Engine.Foundation.Precision.Confusion()) && (v <= VLast + global::Kerf.Engine.Foundation.Precision.Confusion());
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
        public Surface? Representation;

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
    /// Mapped descriptor: an optional bounded domain and a diagonal affine parameter map with optional UV swap.
    /// </summary>
    public sealed class Mapped : Base
    {
        /// <summary>The maximum supported derivative order (OCCT: <c>std::size_t</c>).</summary>
        public int MaxDerivOrder = 3;

        /// <summary>The domain; null for the full domain.</summary>
        public Domain2d? Domain;

        /// <summary>The affine parameter transform.</summary>
        public Map2d ParamMap = new Map2d();

        public override Kind GetKind() => Kind.Mapped;
    }
}
