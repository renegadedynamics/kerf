// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Fillets.Api;

/// <summary>
/// The functional twins of the fillet and chamfer makers (plan 2.8, decision 1139): one call from a shape, its edges
/// and the options to a frozen result and its history, for callers that hold values (Bento's cells).
/// </summary>
/// <remarks>
/// <para>
/// Each twin runs OCCT's maker (<see cref="MakeFillet"/>, <see cref="MakeChamfer"/>) with
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
internal static class BRepFilletApi
{
    /// <summary>The fillet of radius <paramref name="radius"/> on the edges <paramref name="edges"/> of <paramref name="shape"/>.</summary>
    /// <exception cref="NotDoneException">The fillet could not be built.</exception>
    public static Shape Fillet(in Shape shape, IReadOnlyList<Edge> edges, double radius,
                                      in MakeFilletOptions options, out History history)
    {
        MakeFillet maker = new(shape, options.FShape);
        maker.SetParams(options.Tang, options.Tesp, options.T2d, options.TApp3d, options.TolApp2d,
                         options.Fleche);
        maker.SetContinuity(options.InternalContinuity, options.AngularTolerance);
        for (int i = 0; i < edges.Count; i++)
        {
            maker.Add(radius, edges[i]);
        }
        return Run(maker, shape, out history);
    }

    /// <inheritdoc cref="Fillet(in Shape, IReadOnlyList{Edge}, double, in MakeFilletOptions, out History)"/>
    public static Shape Fillet(in Shape shape, IReadOnlyList<Edge> edges, double radius,
                                      in MakeFilletOptions options) =>
        Fillet(shape, edges, radius, options, out _);

    /// <summary>The symmetric chamfer of distance <paramref name="distance"/> on the edges <paramref name="edges"/>.</summary>
    /// <exception cref="NotDoneException">The chamfer could not be built.</exception>
    public static Shape Chamfer(in Shape shape, IReadOnlyList<Edge> edges, double distance,
                                       in MakeChamferOptions options, out History history)
    {
        MakeChamfer maker = new(shape);
        maker.SetMode(options.Mode);
        for (int i = 0; i < edges.Count; i++)
        {
            maker.Add(distance, edges[i]);
        }
        return Run(maker, shape, out history);
    }

    /// <inheritdoc cref="Chamfer(in Shape, IReadOnlyList{Edge}, double, in MakeChamferOptions, out History)"/>
    public static Shape Chamfer(in Shape shape, IReadOnlyList<Edge> edges, double distance,
                                       in MakeChamferOptions options) =>
        Chamfer(shape, edges, distance, options, out _);

    // Build, the frozen result (Shape() throws StdFail_NotDone when not done) and OCCT's history over the shape.
    private static Shape Run(LocalOperation maker, in Shape shape,
                                    out History history)
    {
        maker.Build();
        if (!maker.IsDone)
        {
            throw new NotDoneException("BRep_API: command not done");
        }
        Shape result = maker.Shape();
        NCollection_List<Shape> anArgs = new();
        anArgs.Append(shape);
        history = new History(anArgs, maker);
        return result;
    }
}
