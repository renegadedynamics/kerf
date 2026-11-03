// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_TrihedronLaw.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_TrihedronLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Occt;

/// <summary>Defines a trihedron (tangent, normal, binormal) along a curve.</summary>
/// <remarks>
/// A working object (decision 21, as the adaptors, decision 101): <see cref="SetCurve"/> and
/// <see cref="SetInterval"/> are public and change the law, and evaluation may update caches. The law keeps the
/// caller's adaptor as OCCT keeps the handle and never changes it (<see cref="SetInterval"/> trims into a new
/// adaptor). A law is not safe to evaluate from several threads at once; <see cref="Copy"/> one per thread.
/// The evaluators' vectors are <c>ref</c>: some laws leave them unwritten on some paths, as in OCCT.
/// </remarks>
public abstract class GeomFill_TrihedronLaw : Transient
{
    protected CurveAdaptorBase? myCurve;
    protected CurveAdaptorBase? myTrimmed;

    /// <summary>Initializes the curve of the trihedron law; returns true.</summary>
    public virtual bool SetCurve(CurveAdaptorBase C)
    {
        myCurve = C;
        myTrimmed = myCurve;
        return true;
    }

    public abstract GeomFill_TrihedronLaw Copy();

    /// <summary>The status of the law; PipeOk by default.</summary>
    public virtual GeomFill_PipeError ErrorStatus() => GeomFill_PipeError.GeomFill_PipeOk;

    /// <summary>Computes the trihedron on the curve at Param.</summary>
    public abstract bool D0(double Param, ref Vector3D Tangent, ref Vector3D Normal, ref Vector3D BiNormal);

    /// <summary>Computes the trihedron and its derivative at Param (used only for C1 or C2 approximation).</summary>
    public virtual bool D1(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D Normal, ref Vector3D DNormal,
                           ref Vector3D BiNormal, ref Vector3D DBiNormal)
    {
        throw new MethodNotImplementedException(" GeomFill_TrihedronLaw::D2");
    }

    /// <summary>Computes the trihedron and its first and second derivatives at Param (used only for C2 approximation).</summary>
    public virtual bool D2(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D D2Tangent, ref Vector3D Normal,
                           ref Vector3D DNormal, ref Vector3D D2Normal, ref Vector3D BiNormal, ref Vector3D DBiNormal,
                           ref Vector3D D2BiNormal)
    {
        throw new MethodNotImplementedException(" GeomFill_TrihedronLaw::D2");
    }

    /// <summary>The number of intervals for continuity S; may be one if the law's continuity is at least S.</summary>
    public abstract int NbIntervals(Continuity S);

    /// <summary>Stores in T the parameters bounding the intervals of continuity S; T.Length() must exceed NbIntervals(S).</summary>
    public abstract void Intervals(NCollection_Array1<double> T, Continuity S);

    /// <summary>Sets the bounds of the parametric interval, which determines the derivatives there if the law isn't Cn.</summary>
    public virtual void SetInterval(double First, double Last)
    {
        myTrimmed = myCurve!.Trim(First, Last, 0);
    }

    /// <summary>The bounds of the parametric interval.</summary>
    public void GetInterval(out double First, out double Last)
    {
        First = myTrimmed!.FirstParameter();
        Last = myTrimmed.LastParameter();
    }

    /// <summary>The average trihedron, useful to make fast approximations of rational surfaces.</summary>
    public abstract void GetAverageLaw(ref Vector3D ATangent, ref Vector3D ANormal, ref Vector3D ABiNormal);

    /// <summary>Whether the law is constant.</summary>
    public virtual bool IsConstant() => false;

    /// <summary>Whether the law is defined only by the 3D geometry of the curve; false by default.</summary>
    public virtual bool IsOnlyBy3dCurve() => false;
}
