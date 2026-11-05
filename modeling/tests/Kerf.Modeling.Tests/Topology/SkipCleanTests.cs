// Ported from build123d tests/test_direct_api/test_skip_clean.py. SkipClean.clean is read-only here (SkipClean.Clean),
// so setUp's reset to True has no counterpart: each test starts outside any scope.
namespace Kerf.Modeling.Tests.Topology;

using Kerf.Modeling;

public class SkipCleanTests
{
    [Fact]
    public void ContextManagerSetsCleanFalse()
    {
        // Clean is initially true
        Assert.True(SkipClean.Clean);

        using (new SkipClean())
        {
            // Within the scope, Clean is false
            Assert.False(SkipClean.Clean);
        }

        // After leaving the scope, Clean reverts to true
        Assert.True(SkipClean.Clean);
    }

    [Fact]
    public void ExceptionHandlingDoesNotAffectClean()
    {
        Assert.True(SkipClean.Clean);

        // Use the scope and throw an exception
        try
        {
            using (new SkipClean())
            {
                Assert.False(SkipClean.Clean);
                throw new ArgumentException("Test exception");
            }
        }
        catch (ArgumentException)
        {
        }

        // Clean is restored to true after an exception
        Assert.True(SkipClean.Clean);
    }

    [Fact]
    public void BooleanOpLeavesSplitterFaces()
    {
        // Fusing two boxes face to face normally merges the shared faces; inside SkipClean they survive
        var left = Solid.MakeBox(1, 1, 1);
        var right = Solid.MakeBox(1, 1, 1, new Plane(new Point3D(1, 0, 0)));

        var cleaned = left.Fuse(right);
        Shape uncleaned;
        using (new SkipClean())
            uncleaned = left.Fuse(right);

        Assert.Equal(6, cleaned.Faces().Count);
        Assert.True(uncleaned.Faces().Count > 6);
        Check.AlmostEqual(cleaned.Volume, uncleaned.Volume, 6);
    }
}
