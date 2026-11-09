namespace Kerf.Modeling.Tests.Immutability;

/// <summary>
/// The modelling layer's public types can't be changed once made (design-rules.md, section 3; the owner, decision 2039;
/// unit 13P, decision 26401): no public settable property, mutable field, mutable collection held or returned, event,
/// mutable static state or in-place mutator, except what <see cref="Accepted"/> lists, each with its reason. KERF0003
/// catches the first few as the code is compiled; this reads what was built, and the rest.
/// </summary>
public class ImmutabilityGuardTests
{
    /// <summary>What the guard finds and is accepted, by key (<c>Type.Member</c>), each with the reason.</summary>
    private static readonly Dictionary<string, string> Accepted = new()
    {
        ["Kerf.Modeling.TransformExtensions.TransposedList"] = "build123d's transposed_list: a new array of the 16 values on each call.",
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
