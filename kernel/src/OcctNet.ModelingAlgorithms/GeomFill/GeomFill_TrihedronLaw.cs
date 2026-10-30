// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_TrihedronLaw.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_TrihedronLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Defines a trihedron (tangent, normal, binormal) along a curve.</summary>
/// <remarks>
/// A working object (decision 21, as the adaptors, decision 101): <see cref="SetCurve"/> and
/// <see cref="SetInterval"/> are public and change the law, and evaluation may update caches. The law keeps the
/// caller's adaptor as OCCT keeps the handle and never changes it (<see cref="SetInterval"/> trims into a new
/// adaptor). A law is not safe to evaluate from several threads at once; <see cref="Copy"/> one per thread.
/// The evaluators' vectors are <c>ref</c>: some laws leave them unwritten on some paths, as in OCCT.
/// </remarks>
public abstract class GeomFill_TrihedronLaw : Standard_Transient
{
    protected Adaptor3d_Curve? myCurve;
    protected Adaptor3d_Curve? myTrimmed;

    /// <summary>Initializes the curve of the trihedron law; returns true.</summary>
    public virtual bool SetCurve(Adaptor3d_Curve C)
    {
        myCurve = C;
        myTrimmed = myCurve;
        return true;
    }

    public abstract GeomFill_TrihedronLaw Copy();

    /// <summary>The status of the law; PipeOk by default.</summary>
    public virtual GeomFill_PipeError ErrorStatus() => GeomFill_PipeError.GeomFill_PipeOk;

    /// <summary>Computes the trihedron on the curve at Param.</summary>
    public abstract bool D0(double Param, ref gp_Vec Tangent, ref gp_Vec Normal, ref gp_Vec BiNormal);

    /// <summary>Computes the trihedron and its derivative at Param (used only for C1 or C2 approximation).</summary>
    public virtual bool D1(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec Normal, ref gp_Vec DNormal,
                           ref gp_Vec BiNormal, ref gp_Vec DBiNormal)
    {
        throw new Standard_NotImplemented(" GeomFill_TrihedronLaw::D2");
    }

    /// <summary>Computes the trihedron and its first and second derivatives at Param (used only for C2 approximation).</summary>
    public virtual bool D2(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec D2Tangent, ref gp_Vec Normal,
                           ref gp_Vec DNormal, ref gp_Vec D2Normal, ref gp_Vec BiNormal, ref gp_Vec DBiNormal,
                           ref gp_Vec D2BiNormal)
    {
        throw new Standard_NotImplemented(" GeomFill_TrihedronLaw::D2");
    }

    /// <summary>The number of intervals for continuity S; may be one if the law's continuity is at least S.</summary>
    public abstract int NbIntervals(GeomAbs_Shape S);

    /// <summary>Stores in T the parameters bounding the intervals of continuity S; T.Length() must exceed NbIntervals(S).</summary>
    public abstract void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S);

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
    public abstract void GetAverageLaw(ref gp_Vec ATangent, ref gp_Vec ANormal, ref gp_Vec ABiNormal);

    /// <summary>Whether the law is constant.</summary>
    public virtual bool IsConstant() => false;

    /// <summary>Whether the law is defined only by the 3D geometry of the curve; false by default.</summary>
    public virtual bool IsOnlyBy3dCurve() => false;
}
