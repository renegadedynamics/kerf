// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Domain.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine;

namespace Kerf.Engine.Numerics.Utilities;

/// <summary>Parameter domain types for curves and surfaces.</summary>
internal static partial class MathUtils
{
    /// <summary>1D parameter domain [Min, Max] for curves.</summary>
    /// <remarks>A C++ aggregate with public data members, which it keeps; `default` is the empty domain [0, 0], as
    /// OCCT's default constructor.</remarks>
    public struct Domain1D : IEquatable<Domain1D>
    {
        /// <summary>Lower bound of the domain.</summary>
        public double Min;
        /// <summary>Upper bound of the domain.</summary>
        public double Max;

        public Domain1D(double min, double max)
        {
            Min = min;
            Max = max;
        }

        public readonly double Length() => Max - Min;

        public readonly double Mid() => (Min + Max) * 0.5;

        /// <summary>True if theU is in [Min - theTol, Max + theTol].</summary>
        public readonly bool Contains(double u, double tol = 0.0)
        {
            return u >= Min - tol && u <= Max + tol;
        }

        /// <summary>Clamp value to domain bounds.</summary>
        public readonly double Clamp(double u) => u < Min ? Min : (u > Max ? Max : u);

        /// <summary>True if Length() &gt; theThreshold (effectively unbounded for optimization).</summary>
        public readonly bool IsLarge(double threshold = 1000.0) => Length() > threshold;

        /// <summary>True if the domain covers at least one full period.</summary>
        public readonly bool IsFullPeriod(double period, double tol = 1.0e-10)
        {
            return Length() >= period - tol;
        }

        /// <summary>Min + theT * Length().</summary>
        public readonly double Lerp(double t) => Min + t * (Max - Min);

        /// <summary>(theU - Min) / Length(), or 0.5 if Length() == 0.</summary>
        public readonly double Normalize(double u)
        {
            double len = Length();
            return len > 0.0 ? (u - Min) / len : 0.5;
        }

        /// <summary>True if both Min and Max are within (-theInfLimit, theInfLimit).</summary>
        public readonly bool IsFinite(double infLimit = 1.0e100)
        {
            return Min > -infLimit && Max < infLimit;
        }

        /// <summary>True if the domains are equal within tolerance.</summary>
        public readonly bool IsEqual(in Domain1D other, double tol = 1.0e-10)
        {
            return Math.Abs(Min - other.Min) <= tol && Math.Abs(Max - other.Max) <= tol;
        }

        public readonly bool Equals(Domain1D other) => Min.Equals(other.Min) && Max.Equals(other.Max);

        public override readonly bool Equals(object? obj) => obj is Domain1D other && Equals(other);

        public override readonly int GetHashCode() => HashCode.Combine(Min, Max);

        public static bool operator ==(Domain1D left, Domain1D right) => left.Equals(right);

        public static bool operator !=(Domain1D left, Domain1D right) => !left.Equals(right);

        public override readonly string ToString() => Constants.Format("MathUtils.Domain1D", Min, Max);
    }

    /// <summary>2D parameter domain [UMin, UMax] x [VMin, VMax] for surfaces.</summary>
    /// <remarks>A C++ aggregate with public data members, which it keeps; `default` is the empty domain, as OCCT's
    /// default constructor.</remarks>
    public struct Domain2D : IEquatable<Domain2D>
    {
        public double UMin;
        public double UMax;
        public double VMin;
        public double VMax;

        public Domain2D(double uMin, double uMax, double vMin, double vMax)
        {
            UMin = uMin;
            UMax = uMax;
            VMin = vMin;
            VMax = vMax;
        }

        public Domain2D(in Domain1D uDomain, in Domain1D vDomain)
        {
            UMin = uDomain.Min;
            UMax = uDomain.Max;
            VMin = vDomain.Min;
            VMax = vDomain.Max;
        }

        /// <summary>Returns the U subdomain.</summary>
        public readonly Domain1D U() => new(UMin, UMax);

        /// <summary>Returns the V subdomain.</summary>
        public readonly Domain1D V() => new(VMin, VMax);

        public readonly double ULength() => UMax - UMin;

        public readonly double VLength() => VMax - VMin;

        public readonly double UMid() => (UMin + UMax) * 0.5;

        public readonly double VMid() => (VMin + VMax) * 0.5;

        /// <summary>True if (theU, theV) is in the domain, extended by theTol.</summary>
        public readonly bool Contains(double u, double v, double tol = 0.0)
        {
            return u >= UMin - tol && u <= UMax + tol && v >= VMin - tol
                   && v <= VMax + tol;
        }

        /// <summary>Clamp UV to domain bounds, in place.</summary>
        public readonly void Clamp(ref double u, ref double v)
        {
            u = u < UMin ? UMin : (u > UMax ? UMax : u);
            v = v < VMin ? VMin : (v > VMax ? VMax : v);
        }

        /// <summary>True if both U and V lengths exceed theThreshold.</summary>
        public readonly bool IsLarge(double threshold = 1000.0)
        {
            return ULength() > threshold && VLength() > threshold;
        }

        public readonly bool IsUFullPeriod(double period, double tol = 1.0e-10)
        {
            return ULength() >= period - tol;
        }

        public readonly bool IsVFullPeriod(double period, double tol = 1.0e-10)
        {
            return VLength() >= period - tol;
        }

        /// <summary>True if all bounds are within (-theInfLimit, theInfLimit).</summary>
        public readonly bool IsFinite(double infLimit = 1.0e100)
        {
            return UMin > -infLimit && UMax < infLimit && VMin > -infLimit && VMax < infLimit;
        }

        public readonly bool Equals(Domain2D other)
            => UMin.Equals(other.UMin) && UMax.Equals(other.UMax) && VMin.Equals(other.VMin) && VMax.Equals(other.VMax);

        public override readonly bool Equals(object? obj) => obj is Domain2D other && Equals(other);

        public override readonly int GetHashCode() => HashCode.Combine(UMin, UMax, VMin, VMax);

        public static bool operator ==(Domain2D left, Domain2D right) => left.Equals(right);

        public static bool operator !=(Domain2D left, Domain2D right) => !left.Equals(right);

        public override readonly string ToString() => Constants.Format("MathUtils.Domain2D", UMin, UMax, VMin, VMax);
    }
}
