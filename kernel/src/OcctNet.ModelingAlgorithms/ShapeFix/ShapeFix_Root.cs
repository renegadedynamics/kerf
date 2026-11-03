// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Root.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Root.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Root.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Kerf.Engine.Foundation.Gravity;

namespace Occt;

/// <summary>
/// The root of Shape Healing's fixing tools: a context (<see cref="ShapeBuild_ReShape"/>) recording the replacements
/// made, a message registrator, the basic precision and the minimal and maximal tolerances allowed.
/// </summary>
/// <remarks>A working object (decision 21). The context is the caller's working object, shared by reference.</remarks>
public class ShapeFix_Root : Transient
{
    /// <summary>The shape the messages are attached to by default.</summary>
    protected TopoDS_Shape myShape;

    private ShapeBuild_ReShape?              myContext;
    private ShapeExtend_BasicMsgRegistrator? myMsgReg;
    private double                           myPrecision;
    private double                           myMinTol;
    private double                           myMaxTol;

    /// <summary>Empty constructor (no context is created).</summary>
    public ShapeFix_Root()
    {
        myShape     = new TopoDS_Shape();
        myPrecision = myMinTol = myMaxTol = Kerf.Engine.Foundation.Precision.Confusion();
        myMsgReg    = new ShapeExtend_BasicMsgRegistrator();
    }

    /// <summary>Copies all fields from another Root object.</summary>
    public virtual void Set(ShapeFix_Root Root)
    {
        myContext   = Root.myContext;
        myMsgReg    = Root.myMsgReg;
        myPrecision = Root.myPrecision;
        myMinTol    = Root.myMinTol;
        myMaxTol    = Root.myMaxTol;
        myShape     = Root.myShape;
    }

    /// <summary>Sets the context.</summary>
    public virtual void SetContext(ShapeBuild_ReShape? context)
    {
        myContext = context;
    }

    /// <summary>Returns the context.</summary>
    public ShapeBuild_ReShape? Context() => myContext;

    /// <summary>Sets the message registrator.</summary>
    public virtual void SetMsgRegistrator(ShapeExtend_BasicMsgRegistrator? msgreg)
    {
        myMsgReg = msgreg;
    }

    /// <summary>Returns the message registrator.</summary>
    public ShapeExtend_BasicMsgRegistrator? MsgRegistrator() => myMsgReg;

    /// <summary>Sets the basic precision value (and widens the tolerance limits to it).</summary>
    public virtual void SetPrecision(double preci)
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
    public virtual void SetMinTolerance(double mintol)
    {
        myMinTol = mintol;
    }

    /// <summary>Returns the minimal allowed tolerance.</summary>
    public double MinTolerance() => myMinTol;

    /// <summary>Sets the maximal allowed tolerance.</summary>
    public virtual void SetMaxTolerance(double maxtol)
    {
        myMaxTol = maxtol;
    }

    /// <summary>Returns the maximal allowed tolerance.</summary>
    public double MaxTolerance() => myMaxTol;

    /// <summary>Returns the tolerance limited by the maximal tolerance (only the maximal restriction is implemented).</summary>
    public double LimitTolerance(double toler) => Min(myMaxTol, toler);

    /// <summary>Sends a message to be attached to the shape, through the message registrator.</summary>
    public void SendMsg(in TopoDS_Shape shape, Msg message, Gravity gravity = Info)
    {
        myMsgReg?.Send(shape, message, gravity);
    }

    /// <summary>Sends a message to be attached to the shape the tool works on.</summary>
    public void SendMsg(Msg message, Gravity gravity = Info)
    {
        SendMsg(myShape, message, gravity);
    }

    /// <summary>Sends a warning to be attached to the shape.</summary>
    public void SendWarning(in TopoDS_Shape shape, Msg message)
    {
        SendMsg(shape, message, Warning);
    }

    /// <summary>Sends a warning to be attached to the shape the tool works on.</summary>
    public void SendWarning(Msg message)
    {
        SendWarning(myShape, message);
    }

    /// <summary>Sends a fail to be attached to the shape.</summary>
    public void SendFail(in TopoDS_Shape shape, Msg message)
    {
        SendMsg(shape, message, Fail);
    }

    /// <summary>Sends a fail to be attached to the shape the tool works on.</summary>
    public void SendFail(Msg message)
    {
        SendFail(myShape, message);
    }

    /// <summary>
    /// For three-position flags (on, off, default) of ShapeFix's modes: a negative flag gives the default
    /// <paramref name="need"/>, else whether the flag is positive.
    /// </summary>
    protected static bool NeedFix(int Flag, bool need = true) => Flag < 0 ? need : (Flag > 0);
}
