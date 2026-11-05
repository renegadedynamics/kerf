// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HGPPConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HGPPConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// A generic constraint of a non-linear plate (<see cref="NLPlate"/>) at a UV point: a target position and/or
/// target derivatives, up to an active order.
/// </summary>
/// <remarks>
/// A working object (decision 21), as OCCT's handle class: the setters are public, and <see cref="NLPlate"/>
/// moves the UV point of a sliding constraint (<c>SetUV</c>). The base's targets are zero and its setters, but
/// <c>SetActiveOrder</c> and <c>SetUV</c>, do nothing, as in OCCT.
/// </remarks>
internal abstract class HgppConstraint : Transient
{
    private static readonly Xyz XyZnull = new(0.0, 0.0, 0.0);
    private static readonly D1 D1null = new(XyZnull, XyZnull);
    private static readonly D2 D2null = new(XyZnull, XyZnull, XyZnull);
    private static readonly D3 D3null = new(XyZnull, XyZnull, XyZnull, XyZnull);

    protected Xy _uv;
    protected int _activeOrder;

    protected HgppConstraint()
    {
        _activeOrder = 0;
    }

    public virtual void SetUVFreeSliding(bool uvFree)
    {
    }

    public virtual void SetIncrementalLoadAllowed(bool ila)
    {
    }

    public virtual void SetActiveOrder(int activeOrder)
    {
        _activeOrder = activeOrder;
    }

    public virtual void SetUV(in Xy uv)
    {
        _uv = uv;
    }

    public virtual void SetOrientation(int orient = 0)
    {
    }

    public virtual void SetG0Criterion(double tolDist)
    {
    }

    public virtual void SetG1Criterion(double tolAng)
    {
    }

    public virtual void SetG2Criterion(double tolCurv)
    {
    }

    public virtual void SetG3Criterion(double tolG3)
    {
    }

    public virtual bool UVFreeSliding() => false;

    public virtual bool IncrementalLoadAllowed() => false;

    public abstract int ActiveOrder();

    public virtual Xy UV() => _uv;

    public virtual int Orientation() => 0;

    public abstract bool IsG0 { get; }

    /// <summary>The target position (zero in the base: OCCT says it should not be called).</summary>
    public virtual Xyz G0Target() => XyZnull;

    /// <summary>The target first derivatives (zero in the base: OCCT says it should not be called).</summary>
    public virtual D1 G1Target() => D1null;

    /// <summary>The target second derivatives (zero in the base: OCCT says it should not be called).</summary>
    public virtual D2 G2Target() => D2null;

    /// <summary>The target third derivatives (zero in the base: OCCT says it should not be called).</summary>
    public virtual D3 G3Target() => D3null;

    public virtual double G0Criterion() => 0.0;

    public virtual double G1Criterion() => 0.0;

    public virtual double G2Criterion() => 0.0;

    public virtual double G3Criterion() => 0.0;
}
