// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// The functional twins of the fillet and chamfer makers (plan 2.8, decision 1139): one call from a shape, its edges
/// and the options to a frozen result and its history, for callers that hold values (Bento's cells).
/// </summary>
/// <remarks>
/// <para>
/// Each twin runs OCCT's maker (<see cref="BRepFilletAPI_MakeFillet"/>, <see cref="BRepFilletAPI_MakeChamfer"/>) with
/// the options, as the Boolean twins do (decision 579). The shape is never written (the makers copy it at entry) and
/// the result shares its unchanged sub-shapes. The history is OCCT's <c>BRepTools_History(shapes, maker)</c> over the
/// shape: the faces modified and generated, and, as OCCT's makers answer <c>IsDeleted</c> from the result's faces, every
/// edge and vertex removed.
/// </para>
/// <para>
/// A fillet or chamfer that is not done throws <see cref="NotDoneException"/>, as <c>Shape()</c> does (without building a second time); its diagnostics
/// (the faulty contours and their statuses) are the maker's to read.
/// </para>
/// </remarks>
public static class BRepFilletAPI
{
    /// <summary>The fillet of radius <paramref name="theRadius"/> on the edges <paramref name="theEdges"/> of <paramref name="theShape"/>.</summary>
    /// <exception cref="NotDoneException">The fillet could not be built.</exception>
    public static TopoDS_Shape Fillet(in TopoDS_Shape theShape, IReadOnlyList<TopoDS_Edge> theEdges, double theRadius,
                                      in BRepFilletAPI_FilletOptions theOptions, out BRepTools_History theHistory)
    {
        BRepFilletAPI_MakeFillet aMaker = new(theShape, theOptions.FShape);
        aMaker.SetParams(theOptions.Tang, theOptions.Tesp, theOptions.T2d, theOptions.TApp3d, theOptions.TolApp2d,
                         theOptions.Fleche);
        aMaker.SetContinuity(theOptions.InternalContinuity, theOptions.AngularTolerance);
        for (int i = 0; i < theEdges.Count; i++)
        {
            aMaker.Add(theRadius, theEdges[i]);
        }
        return Run(aMaker, theShape, out theHistory);
    }

    /// <inheritdoc cref="Fillet(in TopoDS_Shape, IReadOnlyList{TopoDS_Edge}, double, in BRepFilletAPI_FilletOptions, out BRepTools_History)"/>
    public static TopoDS_Shape Fillet(in TopoDS_Shape theShape, IReadOnlyList<TopoDS_Edge> theEdges, double theRadius,
                                      in BRepFilletAPI_FilletOptions theOptions) =>
        Fillet(theShape, theEdges, theRadius, theOptions, out _);

    /// <summary>The symmetric chamfer of distance <paramref name="theDistance"/> on the edges <paramref name="theEdges"/>.</summary>
    /// <exception cref="NotDoneException">The chamfer could not be built.</exception>
    public static TopoDS_Shape Chamfer(in TopoDS_Shape theShape, IReadOnlyList<TopoDS_Edge> theEdges, double theDistance,
                                       in BRepFilletAPI_ChamferOptions theOptions, out BRepTools_History theHistory)
    {
        BRepFilletAPI_MakeChamfer aMaker = new(theShape);
        aMaker.SetMode(theOptions.Mode);
        for (int i = 0; i < theEdges.Count; i++)
        {
            aMaker.Add(theDistance, theEdges[i]);
        }
        return Run(aMaker, theShape, out theHistory);
    }

    /// <inheritdoc cref="Chamfer(in TopoDS_Shape, IReadOnlyList{TopoDS_Edge}, double, in BRepFilletAPI_ChamferOptions, out BRepTools_History)"/>
    public static TopoDS_Shape Chamfer(in TopoDS_Shape theShape, IReadOnlyList<TopoDS_Edge> theEdges, double theDistance,
                                       in BRepFilletAPI_ChamferOptions theOptions) =>
        Chamfer(theShape, theEdges, theDistance, theOptions, out _);

    // Build, the frozen result (Shape() throws StdFail_NotDone when not done) and OCCT's history over the shape.
    private static TopoDS_Shape Run(BRepFilletAPI_LocalOperation theMaker, in TopoDS_Shape theShape,
                                    out BRepTools_History theHistory)
    {
        theMaker.Build();
        if (!theMaker.IsDone())
        {
            throw new NotDoneException("BRep_API: command not done");
        }
        TopoDS_Shape aResult = theMaker.Shape();
        NCollection_List<TopoDS_Shape> anArgs = new();
        anArgs.Append(theShape);
        theHistory = new BRepTools_History(anArgs, theMaker);
        return aResult;
    }
}
