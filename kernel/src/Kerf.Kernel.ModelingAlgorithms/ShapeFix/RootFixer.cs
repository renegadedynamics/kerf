// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Root.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Root.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Root.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// The root of Shape Healing's fixing tools: a context (<see cref="ReShapeBuilder"/>) recording the replacements
/// made, a message registrator, the basic precision and the minimal and maximal tolerances allowed.
/// </summary>
/// <remarks>A working object (decision 21). The context is the caller's working object, shared by reference.</remarks>
public class RootFixer : Transient
{
    /// <summary>The shape the messages are attached to by default.</summary>
    protected Shape _shape;

    private ReShapeBuilder?              _context;
    private BasicMsgRegistrator? _msgReg;
    private double                           _precision;
    private double                           _minTol;
    private double                           _maxTol;

    /// <summary>Empty constructor (no context is created).</summary>
    public RootFixer()
    {
        _shape     = new Shape();
        _precision = _minTol = _maxTol = Kerf.Engine.Foundation.Precision.Confusion();
        _msgReg    = new BasicMsgRegistrator();
    }

    /// <summary>Copies all fields from another Root object.</summary>
    public virtual void Set(RootFixer root)
    {
        _context   = root._context;
        _msgReg    = root._msgReg;
        _precision = root._precision;
        _minTol    = root._minTol;
        _maxTol    = root._maxTol;
        _shape     = root._shape;
    }

    /// <summary>Sets the context.</summary>
    public virtual void SetContext(ReShapeBuilder? context)
    {
        _context = context;
    }

    /// <summary>Returns the context.</summary>
    public ReShapeBuilder? Context() => _context;

    /// <summary>Sets the message registrator.</summary>
    public virtual void SetMsgRegistrator(BasicMsgRegistrator? msgreg)
    {
        _msgReg = msgreg;
    }

    /// <summary>Returns the message registrator.</summary>
    public BasicMsgRegistrator? MsgRegistrator() => _msgReg;

    /// <summary>Sets the basic precision value (and widens the tolerance limits to it).</summary>
    public virtual void SetPrecision(double preci)
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
    public virtual void SetMinTolerance(double mintol)
    {
        _minTol = mintol;
    }

    /// <summary>Returns the minimal allowed tolerance.</summary>
    public double MinTolerance() => _minTol;

    /// <summary>Sets the maximal allowed tolerance.</summary>
    public virtual void SetMaxTolerance(double maxtol)
    {
        _maxTol = maxtol;
    }

    /// <summary>Returns the maximal allowed tolerance.</summary>
    public double MaxTolerance() => _maxTol;

    /// <summary>Returns the tolerance limited by the maximal tolerance (only the maximal restriction is implemented).</summary>
    public double LimitTolerance(double toler) => Min(_maxTol, toler);

    /// <summary>Sends a message to be attached to the shape, through the message registrator.</summary>
    public void SendMsg(in Shape shape, Msg message, Gravity gravity = Info)
    {
        _msgReg?.Send(shape, message, gravity);
    }

    /// <summary>Sends a message to be attached to the shape the tool works on.</summary>
    public void SendMsg(Msg message, Gravity gravity = Info)
    {
        SendMsg(_shape, message, gravity);
    }

    /// <summary>Sends a warning to be attached to the shape.</summary>
    public void SendWarning(in Shape shape, Msg message)
    {
        SendMsg(shape, message, Warning);
    }

    /// <summary>Sends a warning to be attached to the shape the tool works on.</summary>
    public void SendWarning(Msg message)
    {
        SendWarning(_shape, message);
    }

    /// <summary>Sends a fail to be attached to the shape.</summary>
    public void SendFail(in Shape shape, Msg message)
    {
        SendMsg(shape, message, Fail);
    }

    /// <summary>Sends a fail to be attached to the shape the tool works on.</summary>
    public void SendFail(Msg message)
    {
        SendFail(_shape, message);
    }

    /// <summary>
    /// For three-position flags (on, off, default) of ShapeFix's modes: a negative flag gives the default
    /// <paramref name="need"/>, else whether the flag is positive.
    /// </summary>
    protected static bool NeedFix(int flag, bool need = true) => flag < 0 ? need : (flag > 0);
}
