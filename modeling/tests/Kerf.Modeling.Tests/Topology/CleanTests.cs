// Ported from build123d tests/test_direct_api/test_clean_method.py. Omitted:
// - test_clean_warning_on_exception: it patches ShapeUpgrade_UnifySameDomain with a mock to make Build fail.
namespace Kerf.Modeling.Tests.Topology;

public class CleanMethodTests
{
    [Fact]
    public void CleanWithNoneWrapped()
    {
        // build123d sets wrapped to None; the port's equivalent is the empty shape, whose clean returns itself
        var solid = new Solid();
        var result = solid.Clean();
        Assert.Same(solid, result);
    }
}
