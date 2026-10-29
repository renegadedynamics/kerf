// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_Surface.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The root of surfaces on which geometric algorithms work: an interface between the services a surface
/// provides and those algorithms need. GeomAdaptor_Surface adapts a Geom surface. The surface can be divided
/// into intervals of any continuity in U and V.
/// </summary>
/// <remarks>
/// Every virtual method throws <see cref="Standard_NotImplemented"/> here, so a derived class need only redefine
/// what its algorithms use. Evaluation goes through the <c>Eval</c> methods; <see cref="D0"/> to
/// <see cref="DN"/> and <see cref="Value"/> call them. B-spline coefficients are cached for evaluation, so
/// evaluating one adaptor from several threads at once is not safe.
/// </remarks>
public class Adaptor3d_Surface : Standard_Transient
{
    /// <summary>A shallow copy of the adaptor.</summary>
    public virtual Adaptor3d_Surface ShallowCopy() => throw new Standard_NotImplemented("Adaptor3d_Surface::ShallowCopy");

    public virtual double FirstUParameter() => throw new Standard_NotImplemented("Adaptor3d_Surface::FirstUParameter");

    public virtual double LastUParameter() => throw new Standard_NotImplemented("Adaptor3d_Surface::LastUParameter");

    public virtual double FirstVParameter() => throw new Standard_NotImplemented("Adaptor3d_Surface::FirstVParameter");

    public virtual double LastVParameter() => throw new Standard_NotImplemented("Adaptor3d_Surface::LastVParameter");

    public virtual GeomAbs_Shape UContinuity() => throw new Standard_NotImplemented("Adaptor3d_Surface::UContinuity");

    public virtual GeomAbs_Shape VContinuity() => throw new Standard_NotImplemented("Adaptor3d_Surface::VContinuity");

    /// <summary>The number of U intervals of continuity S; may be one if UContinuity() is at least S.</summary>
    public virtual int NbUIntervals(GeomAbs_Shape S) => throw new Standard_NotImplemented("Adaptor3d_Surface::NbUIntervals");

    /// <summary>The number of V intervals of continuity S; may be one if VContinuity() is at least S.</summary>
    public virtual int NbVIntervals(GeomAbs_Shape S) => throw new Standard_NotImplemented("Adaptor3d_Surface::NbVIntervals");

    /// <summary>The bounds of the U intervals of continuity S.</summary>
    public virtual void UIntervals(NCollection_Array1<double> T, GeomAbs_Shape S) =>
        throw new Standard_NotImplemented("Adaptor3d_Surface::UIntervals");

    /// <summary>The bounds of the V intervals of continuity S.</summary>
    public virtual void VIntervals(NCollection_Array1<double> T, GeomAbs_Shape S) =>
        throw new Standard_NotImplemented("Adaptor3d_Surface::VIntervals");

    /// <summary>The surface trimmed in U between First and Last; Tol is used to test for confusion of points.</summary>
    public virtual Adaptor3d_Surface UTrim(double First, double Last, double Tol) =>
        throw new Standard_NotImplemented("Adaptor3d_Surface::UTrim");

    /// <summary>The surface trimmed in V between First and Last; Tol is used to test for confusion of points.</summary>
    public virtual Adaptor3d_Surface VTrim(double First, double Last, double Tol) =>
        throw new Standard_NotImplemented("Adaptor3d_Surface::VTrim");

    public virtual bool IsUClosed() => throw new Standard_NotImplemented("Adaptor3d_Surface::IsUClosed");

    public virtual bool IsVClosed() => throw new Standard_NotImplemented("Adaptor3d_Surface::IsVClosed");

    public virtual bool IsUPeriodic() => throw new Standard_NotImplemented("Adaptor3d_Surface::IsUPeriodic");

    public virtual double UPeriod() => throw new Standard_NotImplemented("Adaptor3d_Surface::UPeriod");

    public virtual bool IsVPeriodic() => throw new Standard_NotImplemented("Adaptor3d_Surface::IsVPeriodic");

    public virtual double VPeriod() => throw new Standard_NotImplemented("Adaptor3d_Surface::VPeriod");

    /// <summary>The point of parameters (U, V). (GeomLib::NormEstim computes the normal.)</summary>
    public gp_Pnt Value(double theU, double theV) => EvalD0(theU, theV);

    /// <summary>The point of parameters (U, V).</summary>
    public void D0(double theU, double theV, out gp_Pnt theP) => theP = EvalD0(theU, theV);

    /// <summary>The point and first derivatives. Throws if the current intervals are not C1.</summary>
    public void D1(double theU, double theV, out gp_Pnt theP, out gp_Vec theD1U, out gp_Vec theD1V)
    {
        Geom_Surface.ResD1 aRes = EvalD1(theU, theV);
        theP = aRes.Point;
        theD1U = aRes.D1U;
        theD1V = aRes.D1V;
    }

    /// <summary>The point and the first and second derivatives. Throws if the current intervals are not C2.</summary>
    public void D2(double theU, double theV, out gp_Pnt theP, out gp_Vec theD1U, out gp_Vec theD1V,
                   out gp_Vec theD2U, out gp_Vec theD2V, out gp_Vec theD2UV)
    {
        Geom_Surface.ResD2 aRes = EvalD2(theU, theV);
        theP = aRes.Point;
        theD1U = aRes.D1U;
        theD1V = aRes.D1V;
        theD2U = aRes.D2U;
        theD2V = aRes.D2V;
        theD2UV = aRes.D2UV;
    }

    /// <summary>The point and the derivatives up to the third order. Throws if the current intervals are not C3.</summary>
    public void D3(double theU, double theV, out gp_Pnt theP, out gp_Vec theD1U, out gp_Vec theD1V,
                   out gp_Vec theD2U, out gp_Vec theD2V, out gp_Vec theD2UV,
                   out gp_Vec theD3U, out gp_Vec theD3V, out gp_Vec theD3UUV, out gp_Vec theD3UVV)
    {
        Geom_Surface.ResD3 aRes = EvalD3(theU, theV);
        theP = aRes.Point;
        theD1U = aRes.D1U;
        theD1V = aRes.D1V;
        theD2U = aRes.D2U;
        theD2V = aRes.D2V;
        theD2UV = aRes.D2UV;
        theD3U = aRes.D3U;
        theD3V = aRes.D3V;
        theD3UUV = aRes.D3UUV;
        theD3UVV = aRes.D3UVV;
    }

    /// <summary>
    /// The derivative of order Nu in U and Nv in V at (U, V). Throws if the current intervals are not CNu and CNv,
    /// or if Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.
    /// </summary>
    public gp_Vec DN(double theU, double theV, int theNu, int theNv) => EvalDN(theU, theV, theNu, theNv);

    /// <summary>The U resolution corresponding to the real space resolution R3d.</summary>
    public virtual double UResolution(double R3d) => throw new Standard_NotImplemented("Adaptor3d_Surface::UResolution");

    /// <summary>The V resolution corresponding to the real space resolution R3d.</summary>
    public virtual double VResolution(double R3d) => throw new Standard_NotImplemented("Adaptor3d_Surface::VResolution");

    /// <summary>The type of the surface.</summary>
    /// <remarks>Hides <see cref="object.GetType"/>; use <see cref="Standard_Transient.DynamicType"/> for the .NET type.</remarks>
    public new virtual GeomAbs_SurfaceType GetType() => throw new Standard_NotImplemented("Adaptor3d_Surface::GetType");

    public virtual gp_Pln Plane() => throw new Standard_NotImplemented("Adaptor3d_Surface::Plane");

    public virtual gp_Cylinder Cylinder() => throw new Standard_NotImplemented("Adaptor3d_Surface::Cylinder");

    public virtual gp_Cone Cone() => throw new Standard_NotImplemented("Adaptor3d_Surface::Cone");

    public virtual gp_Sphere Sphere() => throw new Standard_NotImplemented("Adaptor3d_Surface::Sphere");

    public virtual gp_Torus Torus() => throw new Standard_NotImplemented("Adaptor3d_Surface::Torus");

    public virtual int UDegree() => throw new Standard_NotImplemented("Adaptor3d_Surface::UDegree");

    public virtual int NbUPoles() => throw new Standard_NotImplemented("Adaptor3d_Surface::NbUPoles");

    public virtual int VDegree() => throw new Standard_NotImplemented("Adaptor3d_Surface::VDegree");

    public virtual int NbVPoles() => throw new Standard_NotImplemented("Adaptor3d_Surface::NbVPoles");

    public virtual int NbUKnots() => throw new Standard_NotImplemented("Adaptor3d_Surface::NbUKnots");

    public virtual int NbVKnots() => throw new Standard_NotImplemented("Adaptor3d_Surface::NbVKnots");

    public virtual bool IsURational() => throw new Standard_NotImplemented("Adaptor3d_Surface::IsURational");

    public virtual bool IsVRational() => throw new Standard_NotImplemented("Adaptor3d_Surface::IsVRational");

    public virtual Geom_BezierSurface Bezier() => throw new Standard_NotImplemented("Adaptor3d_Surface::Bezier");

    public virtual Geom_BSplineSurface BSpline() => throw new Standard_NotImplemented("Adaptor3d_Surface::BSpline");

    public virtual gp_Ax1 AxeOfRevolution() => throw new Standard_NotImplemented("Adaptor3d_Surface::AxeOfRevolution");

    public virtual gp_Dir Direction() => throw new Standard_NotImplemented("Adaptor3d_Surface::Direction");

    public virtual Adaptor3d_Curve BasisCurve() => throw new Standard_NotImplemented("Adaptor3d_Surface::BasisCurve");

    public virtual Adaptor3d_Surface BasisSurface() => throw new Standard_NotImplemented("Adaptor3d_Surface::BasisSurface");

    public virtual double OffsetValue() => throw new Standard_NotImplemented("Adaptor3d_Surface::OffsetValue");

    /// <summary>The point of parameters (U, V). Throws on failure.</summary>
    public virtual gp_Pnt EvalD0(double theU, double theV) => throw new Standard_NotImplemented("Adaptor3d_Surface::EvalD0");

    /// <summary>The point and first partial derivatives at (U, V). Throws on failure.</summary>
    public virtual Geom_Surface.ResD1 EvalD1(double theU, double theV) =>
        throw new Standard_NotImplemented("Adaptor3d_Surface::EvalD1");

    /// <summary>The point and partial derivatives up to the second order at (U, V). Throws on failure.</summary>
    public virtual Geom_Surface.ResD2 EvalD2(double theU, double theV) =>
        throw new Standard_NotImplemented("Adaptor3d_Surface::EvalD2");

    /// <summary>The point and partial derivatives up to the third order at (U, V). Throws on failure.</summary>
    public virtual Geom_Surface.ResD3 EvalD3(double theU, double theV) =>
        throw new Standard_NotImplemented("Adaptor3d_Surface::EvalD3");

    /// <summary>The derivative of order Nu in U and Nv in V at (U, V). Throws on failure.</summary>
    public virtual gp_Vec EvalDN(double theU, double theV, int theNu, int theNv) =>
        throw new Standard_NotImplemented("Adaptor3d_Surface::EvalDN");
}
