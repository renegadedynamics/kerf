// Port of: src/ModelingData/TKBRep/BRepLProp/BRepLProp_SLProps.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Properties;

using Kerf.Engine.Topology.Adaptors;

namespace Kerf.Engine.Topology.Properties;

/// <summary>
/// The local properties of a face's surface (<see cref="FaceSurfaceAdaptor"/>) at a pair of parameters: point,
/// derivatives, tangents, normal and curvatures.
/// </summary>
/// <remarks>
/// OCCT's alias <c>GeomLProp_SLPropsBase&lt;BRepAdaptor_Surface&gt;</c>: the body of <see cref="SLProps3d"/>,
/// which this class derives from (decision 229; the bounds are the adaptor's first and last parameters, as
/// BRepLProp_SurfaceTool's). OCCT keeps the adaptor by value, so the constructors and
/// <see cref="SetSurface(FaceSurfaceAdaptor)"/> keep a shallow copy of it.
/// </remarks>
internal class SLProps : SLProps3d
{
    /// <summary>
    /// The properties of the surface S at (U, V), computing derivatives up to order N (0 to 2), with the linear
    /// tolerance Resolution.
    /// </summary>
    public SLProps(FaceSurfaceAdaptor s, double u, double v, int n, double resolution)
        : base((FaceSurfaceAdaptor)s.ShallowCopy(), u, v, n, resolution)
    {
    }

    /// <summary>As the previous constructor, without setting the parameters.</summary>
    public SLProps(FaceSurfaceAdaptor s, int n, double resolution)
        : base((FaceSurfaceAdaptor)s.ShallowCopy(), n, resolution)
    {
    }

    /// <summary>Properties without a surface yet: <see cref="SetSurface(FaceSurfaceAdaptor)"/> gives it.</summary>
    public SLProps(int n, double resolution)
        : base(n, resolution)
    {
    }

    /// <summary>Sets the surface (a shallow copy of it); the parameters must then be set again.</summary>
    public void SetSurface(FaceSurfaceAdaptor s) => base.SetSurface((FaceSurfaceAdaptor)s.ShallowCopy());
}
