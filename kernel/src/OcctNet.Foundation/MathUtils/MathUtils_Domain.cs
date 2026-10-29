// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Domain.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Parameter domain types for curves and surfaces.</summary>
public static partial class MathUtils
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

        public Domain1D(double theMin, double theMax)
        {
            Min = theMin;
            Max = theMax;
        }

        public readonly double Length() => Max - Min;

        public readonly double Mid() => (Min + Max) * 0.5;

        /// <summary>True if theU is in [Min - theTol, Max + theTol].</summary>
        public readonly bool Contains(double theU, double theTol = 0.0)
        {
            return theU >= Min - theTol && theU <= Max + theTol;
        }

        /// <summary>Clamp value to domain bounds.</summary>
        public readonly double Clamp(double theU) => theU < Min ? Min : (theU > Max ? Max : theU);

        /// <summary>True if Length() &gt; theThreshold (effectively unbounded for optimization).</summary>
        public readonly bool IsLarge(double theThreshold = 1000.0) => Length() > theThreshold;

        /// <summary>True if the domain covers at least one full period.</summary>
        public readonly bool IsFullPeriod(double thePeriod, double theTol = 1.0e-10)
        {
            return Length() >= thePeriod - theTol;
        }

        /// <summary>Min + theT * Length().</summary>
        public readonly double Lerp(double theT) => Min + theT * (Max - Min);

        /// <summary>(theU - Min) / Length(), or 0.5 if Length() == 0.</summary>
        public readonly double Normalize(double theU)
        {
            double aLen = Length();
            return aLen > 0.0 ? (theU - Min) / aLen : 0.5;
        }

        /// <summary>True if both Min and Max are within (-theInfLimit, theInfLimit).</summary>
        public readonly bool IsFinite(double theInfLimit = 1.0e100)
        {
            return Min > -theInfLimit && Max < theInfLimit;
        }

        /// <summary>True if the domains are equal within tolerance.</summary>
        public readonly bool IsEqual(in Domain1D theOther, double theTol = 1.0e-10)
        {
            return Math.Abs(Min - theOther.Min) <= theTol && Math.Abs(Max - theOther.Max) <= theTol;
        }

        public readonly bool Equals(Domain1D theOther) => Min.Equals(theOther.Min) && Max.Equals(theOther.Max);

        public override readonly bool Equals(object? theObj) => theObj is Domain1D anOther && Equals(anOther);

        public override readonly int GetHashCode() => HashCode.Combine(Min, Max);

        public static bool operator ==(Domain1D theLeft, Domain1D theRight) => theLeft.Equals(theRight);

        public static bool operator !=(Domain1D theLeft, Domain1D theRight) => !theLeft.Equals(theRight);

        public override readonly string ToString() => gp.Format("MathUtils.Domain1D", Min, Max);
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

        public Domain2D(double theUMin, double theUMax, double theVMin, double theVMax)
        {
            UMin = theUMin;
            UMax = theUMax;
            VMin = theVMin;
            VMax = theVMax;
        }

        public Domain2D(in Domain1D theUDomain, in Domain1D theVDomain)
        {
            UMin = theUDomain.Min;
            UMax = theUDomain.Max;
            VMin = theVDomain.Min;
            VMax = theVDomain.Max;
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
        public readonly bool Contains(double theU, double theV, double theTol = 0.0)
        {
            return theU >= UMin - theTol && theU <= UMax + theTol && theV >= VMin - theTol
                   && theV <= VMax + theTol;
        }

        /// <summary>Clamp UV to domain bounds, in place.</summary>
        public readonly void Clamp(ref double theU, ref double theV)
        {
            theU = theU < UMin ? UMin : (theU > UMax ? UMax : theU);
            theV = theV < VMin ? VMin : (theV > VMax ? VMax : theV);
        }

        /// <summary>True if both U and V lengths exceed theThreshold.</summary>
        public readonly bool IsLarge(double theThreshold = 1000.0)
        {
            return ULength() > theThreshold && VLength() > theThreshold;
        }

        public readonly bool IsUFullPeriod(double thePeriod, double theTol = 1.0e-10)
        {
            return ULength() >= thePeriod - theTol;
        }

        public readonly bool IsVFullPeriod(double thePeriod, double theTol = 1.0e-10)
        {
            return VLength() >= thePeriod - theTol;
        }

        /// <summary>True if all bounds are within (-theInfLimit, theInfLimit).</summary>
        public readonly bool IsFinite(double theInfLimit = 1.0e100)
        {
            return UMin > -theInfLimit && UMax < theInfLimit && VMin > -theInfLimit && VMax < theInfLimit;
        }

        public readonly bool Equals(Domain2D theOther)
            => UMin.Equals(theOther.UMin) && UMax.Equals(theOther.UMax) && VMin.Equals(theOther.VMin) && VMax.Equals(theOther.VMax);

        public override readonly bool Equals(object? theObj) => theObj is Domain2D anOther && Equals(anOther);

        public override readonly int GetHashCode() => HashCode.Combine(UMin, UMax, VMin, VMax);

        public static bool operator ==(Domain2D theLeft, Domain2D theRight) => theLeft.Equals(theRight);

        public static bool operator !=(Domain2D theLeft, Domain2D theRight) => !theLeft.Equals(theRight);

        public override readonly string ToString() => gp.Format("MathUtils.Domain2D", UMin, UMax, VMin, VMax);
    }
}
