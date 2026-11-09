namespace Kerf.Modeling.Tests.Immutability;

/// <summary>
/// The modelling layer's public API against its baseline, modeling/docs/public-api.txt (unit 13P, decision 26401): any
/// change to Kerf.Modeling's public surface fails here until the file is regenerated and committed with it, so every
/// change to the API is seen at review. Regenerate with <c>KERF_API_UPDATE=1</c>, running this test. The kernel's
/// surface has its own baseline and test, in Kerf.Kernel.PublicApi.Tests.
/// </summary>
public class PublicApiBaselineTests
{
    internal static IEnumerable<System.Reflection.Assembly> Assemblies() => [typeof(Kerf.Modeling.Part).Assembly];

    [Fact]
    public void The_public_API_is_its_baseline()
    {
        string? difference = ApiSurface.CompareWithBaseline(
            Assemblies(), Path.Combine(ApiSurface.RepositoryRoot(), "modeling", "docs", "public-api.txt"));
        Assert.True(difference is null, difference);
    }
}
