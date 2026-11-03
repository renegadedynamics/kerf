// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Regul.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Regul.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

namespace Occt;

/// <summary>
/// A curve of the data structure and its two supports, faces or surfaces (by index): the edges whose continuity
/// the fillet builders set.
/// </summary>
/// <remarks>A C++ value class, so a struct; <c>new()</c> and <c>default</c> are the same (all zero).</remarks>
public partial struct ChFiDS_Regul : IEquatable<ChFiDS_Regul>
{
    private int icurv;
    private int is1;
    private int is2;

    public ChFiDS_Regul()
    {
        icurv = 0;
        is1 = 0;
        is2 = 0;
    }

    internal void SetCurve(int IC)
    {
        icurv = Math.Abs(IC);
    }

    internal void SetS1(int IS1, bool IsFace = true)
    {
        if (IsFace)
        {
            is1 = Math.Abs(IS1);
        }
        else
        {
            is1 = -Math.Abs(IS1);
        }
    }

    internal void SetS2(int IS2, bool IsFace = true)
    {
        if (IsFace)
        {
            is2 = Math.Abs(IS2);
        }
        else
        {
            is2 = -Math.Abs(IS2);
        }
    }

    public readonly ChFiDS_Regul WithCurve(int IC)
    {
        ChFiDS_Regul aCopy = this;
        aCopy.SetCurve(IC);
        return aCopy;
    }

    public readonly ChFiDS_Regul WithS1(int IS1, bool IsFace = true)
    {
        ChFiDS_Regul aCopy = this;
        aCopy.SetS1(IS1, IsFace);
        return aCopy;
    }

    public readonly ChFiDS_Regul WithS2(int IS2, bool IsFace = true)
    {
        ChFiDS_Regul aCopy = this;
        aCopy.SetS2(IS2, IsFace);
        return aCopy;
    }

    /// <summary>True if the first support is a surface, not a face.</summary>
    public readonly bool IsSurface1()
    {
        return (is1 < 0);
    }

    /// <summary>True if the second support is a surface, not a face.</summary>
    public readonly bool IsSurface2()
    {
        return (is2 < 0);
    }

    public readonly int Curve()
    {
        return icurv;
    }

    public readonly int S1()
    {
        return Math.Abs(is1);
    }

    public readonly int S2()
    {
        return Math.Abs(is2);
    }

    public readonly bool Equals(ChFiDS_Regul other) => icurv == other.icurv && is1 == other.is1 && is2 == other.is2;

    public override readonly bool Equals(object? obj) => obj is ChFiDS_Regul other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(icurv, is1, is2);

    public static bool operator ==(in ChFiDS_Regul theLeft, in ChFiDS_Regul theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in ChFiDS_Regul theLeft, in ChFiDS_Regul theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => Constants.Format("ChFiDS_Regul", icurv, is1, is2);
}
