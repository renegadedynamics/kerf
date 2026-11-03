// Port of: src/ModelingData/TKBRep/BRepLProp/BRepLProp_CLProps.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Properties;

using Kerf.Engine.Topology.Adaptors;

using Occt;

namespace Kerf.Engine.Topology.Properties;

/// <summary>
/// The local properties of an edge's curve (<see cref="EdgeCurveAdaptor"/>) at a parameter: point, derivatives,
/// tangent, curvature, normal and centre of curvature.
/// </summary>
/// <remarks>
/// OCCT's alias <c>GeomLProp_CLPropsBase&lt;gp_Pnt, gp_Vec, gp_Dir, BRepAdaptor_Curve&gt;</c>: the body of
/// <see cref="CLProps3d"/>, which this class derives from (decision 229). OCCT keeps the adaptor by value, so
/// the constructors and <see cref="SetCurve(EdgeCurveAdaptor)"/> keep a shallow copy of it: changing the caller's
/// adaptor afterwards changes nothing here.
/// </remarks>
public class CLProps : CLProps3d
{
    /// <summary>
    /// The properties of the curve C, computing derivatives up to order N (0 to 3) when a parameter is set, with the
    /// linear tolerance Resolution.
    /// </summary>
    public CLProps(EdgeCurveAdaptor c, int n, double resolution)
        : base((EdgeCurveAdaptor)c.ShallowCopy(), n, resolution)
    {
    }

    /// <summary>As the previous constructor, and sets the parameter U.</summary>
    public CLProps(EdgeCurveAdaptor c, double u, int n, double resolution)
        : base((EdgeCurveAdaptor)c.ShallowCopy(), u, n, resolution)
    {
    }

    /// <summary>Properties without a curve yet: <see cref="SetCurve(EdgeCurveAdaptor)"/> gives it.</summary>
    public CLProps(int n, double resolution)
        : base(n, resolution)
    {
    }

    /// <summary>Sets the curve (a shallow copy of it); the parameter must then be set again.</summary>
    public void SetCurve(EdgeCurveAdaptor c) => base.SetCurve((EdgeCurveAdaptor)c.ShallowCopy());
}
