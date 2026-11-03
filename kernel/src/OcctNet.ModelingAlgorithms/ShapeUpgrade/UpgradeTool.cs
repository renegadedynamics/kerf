// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_Tool.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_Tool.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Build;
using Occt;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// The root of ShapeUpgrade's splitting tools: a context (<see cref="ReShapeBuilder"/>) recording the changes, the
/// basic precision and the minimal and maximal tolerances allowed.
/// </summary>
/// <remarks>A working object (decision 21). The context is the caller's working object, shared by reference.</remarks>
public class UpgradeTool : Transient
{
    private ReShapeBuilder? _context;
    private double _precision;
    private double _minTol;
    private double _maxTol;

    /// <summary>Empty constructor (precision and minimal tolerance Confusion, maximal tolerance 1).</summary>
    public UpgradeTool()
    {
        _precision = _minTol = Kerf.Engine.Foundation.Precision.Confusion();
        _maxTol = 1; // Precision::Infinite() ?? pdn
    }

    /// <summary>Copies all fields from another tool.</summary>
    public void Set(UpgradeTool tool)
    {
        _context = tool._context;
        _precision = tool._precision;
        _minTol = tool._minTol;
        _maxTol = tool._maxTol;
    }

    /// <summary>Sets the context.</summary>
    public void SetContext(ReShapeBuilder? context)
    {
        _context = context;
    }

    /// <summary>Returns the context.</summary>
    public ReShapeBuilder? Context() => _context;

    /// <summary>Sets the basic precision value (and widens the tolerance limits to it).</summary>
    public void SetPrecision(double preci)
    {
        _precision = preci;
        if (_maxTol < _precision)
        {
            _maxTol = _precision;
        }
        if (_minTol > _precision)
        {
            _minTol = _precision;
        }
    }

    /// <summary>Returns the basic precision value.</summary>
    public double Precision() => _precision;

    /// <summary>Sets the minimal allowed tolerance.</summary>
    public void SetMinTolerance(double mintol)
    {
        _minTol = mintol;
    }

    /// <summary>Returns the minimal allowed tolerance.</summary>
    public double MinTolerance() => _minTol;

    /// <summary>Sets the maximal allowed tolerance.</summary>
    public void SetMaxTolerance(double maxtol)
    {
        _maxTol = maxtol;
    }

    /// <summary>Returns the maximal allowed tolerance.</summary>
    public double MaxTolerance() => _maxTol;

    /// <summary>Returns the tolerance limited by the maximal tolerance (only the maximal restriction is implemented).</summary>
    public double LimitTolerance(double toler) => Math.Min(_maxTol, toler);
}
