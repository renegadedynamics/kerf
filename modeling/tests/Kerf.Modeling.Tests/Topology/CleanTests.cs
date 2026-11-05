// Ported from build123d tests/test_direct_api/test_clean_method.py. Omitted:
// - test_clean_warning_on_exception: it patches ShapeUpgrade_UnifySameDomain with a mock to make Build fail.
namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class CleanMethodTests
{
    [Fact]
    public void CleanWithNoneWrapped()
    {
        // build123d sets wrapped to None; the port's equivalent is the empty shape, whose clean returns itself
        var solid = default(Solid);
        var result = solid.Clean();
        Check.Same(solid, result);
    }
}
