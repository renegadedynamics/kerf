// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HGPPConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/NLPlate/NLPlate_HGPPConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A generic constraint of a non-linear plate (<see cref="NLPlate_NLPlate"/>) at a UV point: a target position and/or
/// target derivatives, up to an active order.
/// </summary>
/// <remarks>
/// A working object (decision 21), as OCCT's handle class: the setters are public, and <see cref="NLPlate_NLPlate"/>
/// moves the UV point of a sliding constraint (<c>SetUV</c>). The base's targets are zero and its setters, but
/// <c>SetActiveOrder</c> and <c>SetUV</c>, do nothing, as in OCCT.
/// </remarks>
public abstract class NLPlate_HGPPConstraint : Standard_Transient
{
    private static readonly gp_XYZ XYZnull = new(0.0, 0.0, 0.0);
    private static readonly Plate_D1 D1null = new(XYZnull, XYZnull);
    private static readonly Plate_D2 D2null = new(XYZnull, XYZnull, XYZnull);
    private static readonly Plate_D3 D3null = new(XYZnull, XYZnull, XYZnull, XYZnull);

    protected gp_XY myUV;
    protected int myActiveOrder;

    protected NLPlate_HGPPConstraint()
    {
        myActiveOrder = 0;
    }

    public virtual void SetUVFreeSliding(bool UVFree)
    {
    }

    public virtual void SetIncrementalLoadAllowed(bool ILA)
    {
    }

    public virtual void SetActiveOrder(int ActiveOrder)
    {
        myActiveOrder = ActiveOrder;
    }

    public virtual void SetUV(in gp_XY UV)
    {
        myUV = UV;
    }

    public virtual void SetOrientation(int Orient = 0)
    {
    }

    public virtual void SetG0Criterion(double TolDist)
    {
    }

    public virtual void SetG1Criterion(double TolAng)
    {
    }

    public virtual void SetG2Criterion(double TolCurv)
    {
    }

    public virtual void SetG3Criterion(double TolG3)
    {
    }

    public virtual bool UVFreeSliding() => false;

    public virtual bool IncrementalLoadAllowed() => false;

    public abstract int ActiveOrder();

    public virtual gp_XY UV() => myUV;

    public virtual int Orientation() => 0;

    public abstract bool IsG0();

    /// <summary>The target position (zero in the base: OCCT says it should not be called).</summary>
    public virtual gp_XYZ G0Target() => XYZnull;

    /// <summary>The target first derivatives (zero in the base: OCCT says it should not be called).</summary>
    public virtual Plate_D1 G1Target() => D1null;

    /// <summary>The target second derivatives (zero in the base: OCCT says it should not be called).</summary>
    public virtual Plate_D2 G2Target() => D2null;

    /// <summary>The target third derivatives (zero in the base: OCCT says it should not be called).</summary>
    public virtual Plate_D3 G3Target() => D3null;

    public virtual double G0Criterion() => 0.0;

    public virtual double G1Criterion() => 0.0;

    public virtual double G2Criterion() => 0.0;

    public virtual double G3Criterion() => 0.0;
}
