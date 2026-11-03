// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_FaceInterference.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_FaceInterference.lxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_FaceInterference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using System.Diagnostics.CodeAnalysis;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The interference of a fillet with one of its faces: the index of the curve of intersection in the data
/// structure, its transition, its pcurves on the face and on the fillet, and its parameter range.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. <see cref="ChFiDS_SurfData"/> holds two of them and the fillet builders change
/// them in place through its <c>Change*</c> accessors. The pcurves are the fillet's own working geometry.
/// </remarks>
public partial struct ChFiDS_FaceInterference : IEquatable<ChFiDS_FaceInterference>
{
    private double firstParam;
    private double lastParam;
    private Geom2d_Curve? pCurveOnFace;
    private Geom2d_Curve? pCurveOnSurf;
    private int lineindex;
    private TopAbs_Orientation LineTransition;

    public ChFiDS_FaceInterference()
    {
        firstParam = 0.0;
        lastParam = 0.0;
        lineindex = 0;
        LineTransition = TopAbs_FORWARD;
    }

    internal void SetInterference(int LineIndex, TopAbs_Orientation Trans, Geom2d_Curve? PCurv1, Geom2d_Curve? PCurv2)
    {
        lineindex = LineIndex;
        LineTransition = Trans;
        pCurveOnFace = PCurv1;
        pCurveOnSurf = PCurv2;
    }

    internal void SetTransition(TopAbs_Orientation Trans)
    {
        LineTransition = Trans;
    }

    internal void SetFirstParameter(double U1)
    {
        firstParam = U1;
    }

    internal void SetLastParameter(double U1)
    {
        lastParam = U1;
    }

    internal void SetParameter(double U1, bool IsFirst)
    {
        if (IsFirst)
        {
            SetFirstParameter(U1);
        }
        else
        {
            SetLastParameter(U1);
        }
    }

    internal void SetLineIndex(int I)
    {
        lineindex = I;
    }

    /// <summary>OCCT's <c>ChangePCurveOnFace</c>: the pcurve on the face, for reading and writing.</summary>
    [UnscopedRef]
    internal ref Geom2d_Curve? ChangePCurveOnFace() => ref pCurveOnFace;

    /// <summary>OCCT's <c>ChangePCurveOnSurf</c>: the pcurve on the fillet, for reading and writing.</summary>
    [UnscopedRef]
    internal ref Geom2d_Curve? ChangePCurveOnSurf() => ref pCurveOnSurf;

    public readonly ChFiDS_FaceInterference WithInterference(int LineIndex, TopAbs_Orientation Trans,
                                                             Geom2d_Curve? PCurv1, Geom2d_Curve? PCurv2)
    {
        ChFiDS_FaceInterference aCopy = this;
        aCopy.SetInterference(LineIndex, Trans, PCurv1, PCurv2);
        return aCopy;
    }

    public readonly ChFiDS_FaceInterference WithTransition(TopAbs_Orientation Trans)
    {
        ChFiDS_FaceInterference aCopy = this;
        aCopy.SetTransition(Trans);
        return aCopy;
    }

    public readonly ChFiDS_FaceInterference WithFirstParameter(double U1)
    {
        ChFiDS_FaceInterference aCopy = this;
        aCopy.SetFirstParameter(U1);
        return aCopy;
    }

    public readonly ChFiDS_FaceInterference WithLastParameter(double U1)
    {
        ChFiDS_FaceInterference aCopy = this;
        aCopy.SetLastParameter(U1);
        return aCopy;
    }

    public readonly ChFiDS_FaceInterference WithParameter(double U1, bool IsFirst)
    {
        ChFiDS_FaceInterference aCopy = this;
        aCopy.SetParameter(U1, IsFirst);
        return aCopy;
    }

    public readonly ChFiDS_FaceInterference WithLineIndex(int I)
    {
        ChFiDS_FaceInterference aCopy = this;
        aCopy.SetLineIndex(I);
        return aCopy;
    }

    /// <summary>The index of the curve of intersection in the data structure.</summary>
    public readonly int LineIndex()
    {
        return lineindex;
    }

    public readonly TopAbs_Orientation Transition()
    {
        return LineTransition;
    }

    public readonly Geom2d_Curve? PCurveOnFace()
    {
        return pCurveOnFace;
    }

    public readonly Geom2d_Curve? PCurveOnSurf()
    {
        return pCurveOnSurf;
    }

    public readonly double FirstParameter()
    {
        return firstParam;
    }

    public readonly double LastParameter()
    {
        return lastParam;
    }

    public readonly double Parameter(bool IsFirst)
    {
        if (IsFirst)
        {
            return FirstParameter();
        }
        else
        {
            return LastParameter();
        }
    }

    public readonly bool Equals(ChFiDS_FaceInterference other)
        => firstParam.Equals(other.firstParam) && lastParam.Equals(other.lastParam)
           && ReferenceEquals(pCurveOnFace, other.pCurveOnFace) && ReferenceEquals(pCurveOnSurf, other.pCurveOnSurf)
           && lineindex == other.lineindex && LineTransition == other.LineTransition;

    public override readonly bool Equals(object? obj) => obj is ChFiDS_FaceInterference other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(firstParam, lastParam, lineindex, LineTransition);

    public static bool operator ==(in ChFiDS_FaceInterference theLeft, in ChFiDS_FaceInterference theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in ChFiDS_FaceInterference theLeft, in ChFiDS_FaceInterference theRight)
        => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => Constants.Format("ChFiDS_FaceInterference", lineindex, (int)LineTransition, firstParam, lastParam);
}
