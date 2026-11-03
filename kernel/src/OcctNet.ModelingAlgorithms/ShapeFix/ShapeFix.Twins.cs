// OCCT.NET: the functional twins of ShapeFix's in-place statics (docs/phase5-6-plan.md, section 2.7; decision 583).
// Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;

namespace Occt;

public static partial class ShapeFix
{
    /// <summary>
    /// The functional twin of <see cref="SameParameter"/>: the shape with its edges made SameParameter by
    /// ShapeFix_Edge and the tolerances of its edges on planes updated, frozen. A frozen shape is copied first, the
    /// sub-shapes left unchanged are the caller's own, and a shape needing nothing is returned itself.
    /// <paramref name="theIsDone"/> is OCCT's result (false if an edge couldn't be made SameParameter, or on a user
    /// break).
    /// </summary>
    public static Shape SameParametered(in Shape shape, bool enforce, out bool theIsDone, double preci = 0.0,
                                               ProgressRange theProgress = default,
                                               ShapeExtend_BasicMsgRegistrator? theMsgReg = null)
    {
        var anEntry = new CopyAtEntry(shape);
        theIsDone = SameParameter(anEntry.Shape(), enforce, preci, theProgress, theMsgReg);
        return frozen(anEntry.Finish(anEntry.Shape()));
    }

    /// <summary>
    /// The functional twin of <see cref="EncodeRegularity"/>: the shape with the continuity of its edges between
    /// their faces encoded, frozen; see <see cref="SameParametered"/>.
    /// </summary>
    public static Shape RegularityEncoded(in Shape shape, double tolang = 1.0e-10)
    {
        var anEntry = new CopyAtEntry(shape);
        EncodeRegularity(anEntry.Shape(), tolang);
        return frozen(anEntry.Finish(anEntry.Shape()));
    }

    /// <summary>
    /// The functional twin of <see cref="FixVertexPosition"/>: the shape with its vertices more tolerant than
    /// <paramref name="theTolerance"/> moved to the curves' ends (the replacements recorded in
    /// <paramref name="thecontext"/>, which answers for the caller's sub-shapes), frozen; the shape itself when
    /// nothing was done (<paramref name="theIsDone"/> false).
    /// </summary>
    public static Shape VertexPositionFixed(in Shape theshape, double theTolerance, ShapeBuild_ReShape thecontext,
                                                   out bool theIsDone)
    {
        var          anEntry = new CopyAtEntry(theshape, thecontext);
        Shape aWork   = anEntry.Shape();
        theIsDone = FixVertexPosition(ref aWork, theTolerance, thecontext);
        return frozen(anEntry.Finish(aWork));
    }

    private static Shape frozen(in Shape theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }
}
