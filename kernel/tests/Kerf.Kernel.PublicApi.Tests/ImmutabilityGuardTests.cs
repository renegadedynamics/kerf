// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Tests.Shared;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// The kernel's public types can't be changed once made (design-rules.md, section 3; the owner, decision 2039; unit
/// 13P, decision 2043): no public settable property, mutable field, mutable collection held or returned, event, mutable
/// static state or in-place mutator, except what <see cref="Accepted"/> lists, each with its reason. KERF0003 catches
/// the first few as the code is compiled; this reads what was built, and the rest.
/// </summary>
public class ImmutabilityGuardTests
{
    /// <summary>What the guard finds and is accepted, by key (<c>Type.Member</c>), each with the reason.</summary>
    private static readonly Dictionary<string, string> Accepted = new()
    {
        ["Kerf.Booleans.BooleanVolumes.s_pointsAsked"] =
            "Internal and [ThreadStatic]: the points this thread's checks have classified, read by the tests that hold the checks' "
            + "cost down (unit 13A). Nothing reads it to decide a result.",
        ["Kerf.Booleans.BooleanVolumes.s_pointChecks"] =
            "Internal and [ThreadStatic]: the checks of decision 1924's points this thread has made, read by the tests that hold "
            + "their cost down (unit 13K). Nothing reads it to decide a result.",
        ["Kerf.Booleans.BooleanVolumes.s_proofsInParallel"] =
            "Internal and [ThreadStatic]: the inputs'-faces proofs this thread has classified in parallel, read by the test that "
            + "they give what one at a time gives (unit 15AR). Nothing reads it to decide a result.",
        ["Kerf.Topology.Placement.s_productItems"] =
            "Private and [ThreadStatic]: scratch storage for the product of two placements' chains, which never escapes the call.",
        ["Kerf.Drawing.Png.Encode"] = "The PNG file's bytes: a new array, the caller's own.",
        ["Kerf.Geometry.Grid.ToArray"] = "A copy of the grid's values, as its name says.",
        ["Kerf.Geometry.OrientedBox.Corners"] = "A new array of the eight corners on each call.",
        ["Kerf.Meshing.FaceMesh.GetTriangles"] = "A copy for a renderer or a file; the mesh's own buffers are ReadOnlyMemory.",
        ["Kerf.Meshing.FaceMesh.GetNodes"] = "A copy for a renderer or a file; the mesh's own buffers are ReadOnlyMemory.",
        ["Kerf.Meshing.FaceMesh.GetSurfaceParameters"] = "A copy for a renderer or a file; the mesh's own buffers are ReadOnlyMemory.",
        ["Kerf.Meshing.ShapeMesh.GetTriangles"] = "A copy for a renderer or a file; the mesh's own buffers are ReadOnlyMemory.",
        ["Kerf.Topology.SubShapes.Enumerator.MoveNext"] = "An enumerator moves by contract, as List<T>.Enumerator does; it isn't a value anyone shares.",
    };

    [Fact]
    public void Nothing_public_can_change_but_what_is_accepted()
    {
        var found = ApiSurface.Mutability(PublicApiBaselineTests.Assemblies()).ToList();
        string[] unexpected = [.. found.Where(f => !Accepted.ContainsKey(f.Key)).Select(f => f.Key + ": " + f.Why)];
        Assert.True(unexpected.Length == 0,
                    "Make these immutable (init, readonly, an immutable collection, a returning form), or accept each here with its reason:\n"
                    + string.Join('\n', unexpected));
        string[] stale = [.. Accepted.Keys.Where(k => !found.Any(f => f.Key == k))];
        Assert.True(stale.Length == 0, "No longer found, so no longer accepted: " + string.Join(", ", stale));
    }
}
