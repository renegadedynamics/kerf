// Port of: src/ModelingData/TKBRep/BRepLProp/BRepLProp_SLProps.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The local properties of a face's surface (<see cref="BRepAdaptor_Surface"/>) at a pair of parameters: point,
/// derivatives, tangents, normal and curvatures.
/// </summary>
/// <remarks>
/// OCCT's alias <c>GeomLProp_SLPropsBase&lt;BRepAdaptor_Surface&gt;</c>: the body of <see cref="LProp_SLProps3d"/>,
/// which this class derives from (decision 229; the bounds are the adaptor's first and last parameters, as
/// BRepLProp_SurfaceTool's). OCCT keeps the adaptor by value, so the constructors and
/// <see cref="SetSurface(BRepAdaptor_Surface)"/> keep a shallow copy of it.
/// </remarks>
public class BRepLProp_SLProps : LProp_SLProps3d
{
    /// <summary>
    /// The properties of the surface S at (U, V), computing derivatives up to order N (0 to 2), with the linear
    /// tolerance Resolution.
    /// </summary>
    public BRepLProp_SLProps(BRepAdaptor_Surface S, double U, double V, int N, double Resolution)
        : base((BRepAdaptor_Surface)S.ShallowCopy(), U, V, N, Resolution)
    {
    }

    /// <summary>As the previous constructor, without setting the parameters.</summary>
    public BRepLProp_SLProps(BRepAdaptor_Surface S, int N, double Resolution)
        : base((BRepAdaptor_Surface)S.ShallowCopy(), N, Resolution)
    {
    }

    /// <summary>Properties without a surface yet: <see cref="SetSurface(BRepAdaptor_Surface)"/> gives it.</summary>
    public BRepLProp_SLProps(int N, double Resolution)
        : base(N, Resolution)
    {
    }

    /// <summary>Sets the surface (a shallow copy of it); the parameters must then be set again.</summary>
    public void SetSurface(BRepAdaptor_Surface S) => base.SetSurface((BRepAdaptor_Surface)S.ShallowCopy());
}
