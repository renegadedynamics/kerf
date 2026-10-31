// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeEvolved.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_MakeEvolved.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using static Occt.GeomAbs_JoinType;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Builds evolved shapes: the unlooped sweep of a profile (a wire) along a planar spine (a face or a wire).
/// </summary>
/// <remarks>
/// <para>
/// The profile is defined in a coordinate system R placed at each point of the spine: R is the global system if
/// theIsAxeProf is true; otherwise its origin is the point of the spine closest to the profile, its X axis the
/// spine's tangent there and its Z axis the normal to the spine's plane (<see cref="BRepFill.Axe"/>). Only the
/// GeomAbs_Arc join is implemented (as in OCCT). With theIsSolid the result is completed to a solid or a compound of
/// solids. With theIsProfOnSpine the profile must touch the spine.
/// </para>
/// <para>
/// Not available in this unit: the volume mode (theIsVolume, BRepFill_AdvancedEvolved, which removes
/// self-intersections with the Boolean operations; phase 7) throws <see cref="Standard_NotImplemented"/>.
/// </para>
/// <para>
/// A maker (decisions 414 and 454): <see cref="BRepBuilderAPI_MakeShape.Shape"/>, <see cref="Top"/>,
/// <see cref="Bottom"/> and <see cref="GeneratedShapes"/> hand out frozen shapes.
/// </para>
/// </remarks>
public class BRepOffsetAPI_MakeEvolved : BRepBuilderAPI_MakeShape
{
    private readonly BRepFill_Evolved myEvolved = new();
    private readonly bool myIsVolume;

    /// <summary>An empty maker.</summary>
    public BRepOffsetAPI_MakeEvolved()
    {
    }

    /// <summary>
    /// Builds the evolved shape of <paramref name="theProfile"/> swept along <paramref name="theSpine"/> (a wire or a
    /// face).
    /// </summary>
    /// <exception cref="Standard_TypeMismatch">The spine is neither a wire nor a face.</exception>
    /// <exception cref="Standard_NotImplemented">The volume mode, or a join other than GeomAbs_Arc.</exception>
    public BRepOffsetAPI_MakeEvolved(in TopoDS_Shape theSpine,
                                     in TopoDS_Wire theProfile,
                                     GeomAbs_JoinType theJoinType = GeomAbs_Arc,
                                     bool theIsAxeProf = true,
                                     bool theIsSolid = false,
                                     bool theIsProfOnSpine = false,
                                     double theTol = 0.0000001,
                                     bool theIsVolume = false,
                                     bool theRunInParallel = false)
    {
        myIsVolume = theIsVolume;
        if (theSpine.ShapeType() != TopAbs_WIRE && theSpine.ShapeType() != TopAbs_FACE)
        {
            throw new Standard_TypeMismatch("BRepOffsetAPI_MakeEvolved: face or wire is expected as a spine");
        }
        if (theIsVolume)
        {
            // OCCT.NET: BRepFill_AdvancedEvolved (the Boolean operations' removal of self-intersections) is phase 7's
            _ = theRunInParallel;
            throw new Standard_NotImplemented("BRepOffsetAPI_MakeEvolved: the volume mode needs BRepFill_AdvancedEvolved (phase 7)");
        }
        else
        {
            gp_Ax3 Axis = new(gp.Origin(), gp.DZ(), gp.DX());

            if (!theIsAxeProf)
            {
                BRepFill.Axe(theSpine, theProfile, out Axis, out bool POS, Max(theTol, Precision.Confusion()));
                if (theIsProfOnSpine && !POS)
                {
                    return;
                }
            }
            if (theSpine.ShapeType() == TopAbs_WIRE)
            {
                myEvolved.Perform(TopoDS.Wire(theSpine), theProfile, Axis, theJoinType, theIsSolid);
            }
            else
            {
                myEvolved.Perform(TopoDS.Face(theSpine), theProfile, Axis, theJoinType, theIsSolid);
            }
        }

        Build();
    }

    /// <summary>The underlying algorithm.</summary>
    /// <exception cref="Standard_TypeMismatch">In volume mode.</exception>
    public BRepFill_Evolved Evolved()
    {
        if (myIsVolume)
        {
            throw new Standard_TypeMismatch("BRepOffsetAPI_MakeEvolved: myEvolved is accessed while in volume mode");
        }
        return myEvolved;
    }

    /// <summary>Takes the result of the algorithm.</summary>
    public override void Build(Message_ProgressRange theRange = default)
    {
        if (myEvolved.IsDone())
        {
            myShape = myEvolved.InternalShape();
        }

        Done();
    }

    /// <summary>
    /// The shapes generated from a sub-shape <paramref name="SpineShape"/> of the spine and a sub-shape
    /// <paramref name="ProfShape"/> of the profile (frozen, with the result).
    /// </summary>
    public IBentoList<TopoDS_Shape> GeneratedShapes(in TopoDS_Shape SpineShape, in TopoDS_Shape ProfShape)
    {
        if (!myEvolved.IsDone())
        {
            return PackedList<TopoDS_Shape>.Empty;
        }

        return myEvolved.GeneratedShapes(SpineShape, ProfShape);
    }

    /// <summary>The top face if the result was built solid (frozen, with the result).</summary>
    public TopoDS_Shape Top() => myEvolved.Top();

    /// <summary>The bottom face if the result was built solid (frozen, with the result).</summary>
    public TopoDS_Shape Bottom() => myEvolved.Bottom();
}
