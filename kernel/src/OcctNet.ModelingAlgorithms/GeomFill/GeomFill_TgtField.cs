// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_TgtField.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_TgtField.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>The root of the tangency fields of the constrained filling (GeomFill_TgtOnCoons).</summary>
/// <remarks>A working object (decision 21); <see cref="Scale"/> changes a scalable field.</remarks>
public abstract class GeomFill_TgtField : Transient
{
    /// <summary>Whether the field can be scaled; false by default.</summary>
    public virtual bool IsScalable() => false;

    /// <summary>Scales the field by the law Func; does nothing by default.</summary>
    public virtual void Scale(Law_BSpline Func)
    {
    }

    /// <summary>The value of the tangency field at W.</summary>
    public abstract Vector3D Value(double W);

    /// <summary>The derivative of the tangency field at W.</summary>
    public abstract Vector3D D1(double W);

    /// <summary>The value and the derivative of the tangency field at W.</summary>
    public abstract void D1(double W, out Vector3D V, out Vector3D DV);
}
