// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_Tool.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_Tool.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// The root of ShapeUpgrade's splitting tools: a context (<see cref="ShapeBuild_ReShape"/>) recording the changes, the
/// basic precision and the minimal and maximal tolerances allowed.
/// </summary>
/// <remarks>A working object (decision 21). The context is the caller's working object, shared by reference.</remarks>
public class ShapeUpgrade_Tool : Transient
{
    private ShapeBuild_ReShape? myContext;
    private double myPrecision;
    private double myMinTol;
    private double myMaxTol;

    /// <summary>Empty constructor (precision and minimal tolerance Confusion, maximal tolerance 1).</summary>
    public ShapeUpgrade_Tool()
    {
        myPrecision = myMinTol = Kerf.Engine.Foundation.Precision.Confusion();
        myMaxTol = 1; // Precision::Infinite() ?? pdn
    }

    /// <summary>Copies all fields from another tool.</summary>
    public void Set(ShapeUpgrade_Tool tool)
    {
        myContext = tool.myContext;
        myPrecision = tool.myPrecision;
        myMinTol = tool.myMinTol;
        myMaxTol = tool.myMaxTol;
    }

    /// <summary>Sets the context.</summary>
    public void SetContext(ShapeBuild_ReShape? context)
    {
        myContext = context;
    }

    /// <summary>Returns the context.</summary>
    public ShapeBuild_ReShape? Context() => myContext;

    /// <summary>Sets the basic precision value (and widens the tolerance limits to it).</summary>
    public void SetPrecision(double preci)
    {
        myPrecision = preci;
        if (myMaxTol < myPrecision)
        {
            myMaxTol = myPrecision;
        }
        if (myMinTol > myPrecision)
        {
            myMinTol = myPrecision;
        }
    }

    /// <summary>Returns the basic precision value.</summary>
    public double Precision() => myPrecision;

    /// <summary>Sets the minimal allowed tolerance.</summary>
    public void SetMinTolerance(double mintol)
    {
        myMinTol = mintol;
    }

    /// <summary>Returns the minimal allowed tolerance.</summary>
    public double MinTolerance() => myMinTol;

    /// <summary>Sets the maximal allowed tolerance.</summary>
    public void SetMaxTolerance(double maxtol)
    {
        myMaxTol = maxtol;
    }

    /// <summary>Returns the maximal allowed tolerance.</summary>
    public double MaxTolerance() => myMaxTol;

    /// <summary>Returns the tolerance limited by the maximal tolerance (only the maximal restriction is implemented).</summary>
    public double LimitTolerance(double toler) => Math.Min(myMaxTol, toler);
}
