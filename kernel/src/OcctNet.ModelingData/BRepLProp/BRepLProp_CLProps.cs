// Port of: src/ModelingData/TKBRep/BRepLProp/BRepLProp_CLProps.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The local properties of an edge's curve (<see cref="BRepAdaptor_Curve"/>) at a parameter: point, derivatives,
/// tangent, curvature, normal and centre of curvature.
/// </summary>
/// <remarks>
/// OCCT's alias <c>GeomLProp_CLPropsBase&lt;gp_Pnt, gp_Vec, gp_Dir, BRepAdaptor_Curve&gt;</c>: the body of
/// <see cref="LProp_CLProps3d"/>, which this class derives from (decision 229). OCCT keeps the adaptor by value, so
/// the constructors and <see cref="SetCurve(BRepAdaptor_Curve)"/> keep a shallow copy of it: changing the caller's
/// adaptor afterwards changes nothing here.
/// </remarks>
public class BRepLProp_CLProps : LProp_CLProps3d
{
    /// <summary>
    /// The properties of the curve C, computing derivatives up to order N (0 to 3) when a parameter is set, with the
    /// linear tolerance Resolution.
    /// </summary>
    public BRepLProp_CLProps(BRepAdaptor_Curve C, int N, double Resolution)
        : base((BRepAdaptor_Curve)C.ShallowCopy(), N, Resolution)
    {
    }

    /// <summary>As the previous constructor, and sets the parameter U.</summary>
    public BRepLProp_CLProps(BRepAdaptor_Curve C, double U, int N, double Resolution)
        : base((BRepAdaptor_Curve)C.ShallowCopy(), U, N, Resolution)
    {
    }

    /// <summary>Properties without a curve yet: <see cref="SetCurve(BRepAdaptor_Curve)"/> gives it.</summary>
    public BRepLProp_CLProps(int N, double Resolution)
        : base(N, Resolution)
    {
    }

    /// <summary>Sets the curve (a shallow copy of it); the parameter must then be set again.</summary>
    public void SetCurve(BRepAdaptor_Curve C) => base.SetCurve((BRepAdaptor_Curve)C.ShallowCopy());
}
