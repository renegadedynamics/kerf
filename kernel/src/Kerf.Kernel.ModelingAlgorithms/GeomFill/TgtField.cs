// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_TgtField.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_TgtField.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>The root of the tangency fields of the constrained filling (GeomFill_TgtOnCoons).</summary>
/// <remarks>A working object (decision 21); <see cref="Scale"/> changes a scalable field.</remarks>
internal abstract class TgtField : Transient
{
    /// <summary>Whether the field can be scaled; false by default.</summary>
    public virtual bool IsScalable => false;

    /// <summary>Scales the field by the law Func; does nothing by default.</summary>
    public virtual void Scale(BSpline func)
    {
    }

    /// <summary>The value of the tangency field at W.</summary>
    public abstract Vector3D Value(double w);

    /// <summary>The derivative of the tangency field at W.</summary>
    public abstract Vector3D D1(double w);

    /// <summary>The value and the derivative of the tangency field at W.</summary>
    public abstract void D1(double w, out Vector3D v, out Vector3D dv);
}
