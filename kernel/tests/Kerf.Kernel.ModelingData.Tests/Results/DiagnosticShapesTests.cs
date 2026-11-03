// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Tests of a diagnostic's shapes, Built<T> and the bridge from a report's alerts (unit 10F).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingData.Tests.Results;

public class DiagnosticShapesTests
{
    internal static Vertex MakeVertex(double x)
    {
        var vertex = new Vertex();
        new BrepBuilder().MakeVertex(ref vertex, new Point3D(x, 0, 0), 1e-7);
        return vertex;
    }

    [Fact]
    public void ADiagnosticNamesNoShapesUntilGivenSome()
    {
        var diagnostic = Diagnostic.Error(DiagnosticCode.InvalidInput, "Bad.");
        Assert.Empty(diagnostic.Shapes);

        Shape a = MakeVertex(1), b = MakeVertex(2);
        var withShapes = diagnostic.WithShapes(a, b);
        Assert.Equal([a, b], withShapes.Shapes.AsEnumerable());
        Assert.Empty(diagnostic.Shapes);          // the original is unchanged
        Assert.Equal(diagnostic.Message, withShapes.Message);

        Assert.Equal([b], withShapes.WithShapes(new List<Shape> { b }).Shapes.AsEnumerable());
        Assert.Empty(withShapes.WithShapes().Shapes);
    }

    [Fact]
    public void BuiltConvertsToItsShape()
    {
        var vertex = MakeVertex(3);
        var history = new History();
        var built = new Built<Vertex>(vertex, history);
        Vertex shape = built;
        Assert.True(shape.IsSame(vertex));
        Assert.Same(history, built.History);
        Assert.Throws<ArgumentNullException>(() => new Built<Vertex>(vertex, null!));
        Assert.Throws<ArgumentNullException>(() => built with { History = null! });
    }

    [Fact]
    public void BuiltSucceedsAsAResult()
    {
        var vertex = MakeVertex(4);
        Result<Built<Vertex>> result = new Built<Vertex>(vertex, new History());
        Vertex shape = result.Value;
        Assert.True(shape.IsSame(vertex));
    }
}

/// <summary>Alerts as the algorithms define them, for the tests.</summary>
public sealed class TestAlertNothingToDo : Alert;

public sealed class TestAlertBadShape(in Shape shape) : AlertWithShape(shape);

public class AlertDiagnosticsTests
{
    [Fact]
    public void GravitiesBecomeSeverities()
    {
        var report = new Report();
        var vertex = DiagnosticShapesTests.MakeVertex(1);
        report.AddAlert(Gravity.Fail, new TestAlertBadShape(vertex));
        report.AddAlert(Gravity.Info, new TestAlertNothingToDo());
        report.AddAlert(Gravity.Warning, new TestAlertNothingToDo());

        var diagnostics = AlertDiagnostics.ToDiagnostics(report);
        Assert.Equal(2, diagnostics.Length);
        Assert.Equal(Severity.Warning, diagnostics[0].Severity);
        Assert.Equal("engine.test-alert-nothing-to-do", diagnostics[0].Code.Value);
        Assert.Empty(diagnostics[0].Shapes);
        Assert.Equal(Severity.Error, diagnostics[1].Severity);
        Assert.Equal("engine.test-alert-bad-shape", diagnostics[1].Code.Value);
        Assert.True(diagnostics[1].Shapes.Single().IsSame(vertex));
    }

    [Fact]
    public void AMessageComesFromTheTableWhereItHasOne()
    {
        // No text for the test's alerts: the code's words
        Assert.Equal("Test alert nothing to do", AlertDiagnostics.ToDiagnostic(new TestAlertNothingToDo(), Gravity.Alarm)!.Message);
        MsgFile.AddMsg(nameof(TestAlertBadShape), "The shape is bad");
        Assert.Equal("The shape is bad", AlertDiagnostics.ToDiagnostic(new TestAlertBadShape(new Shape()), Gravity.Fail)!.Message);
    }

    [Fact]
    public void InformationAndTracesAreLeftOut()
    {
        Assert.Null(AlertDiagnostics.ToDiagnostic(new TestAlertNothingToDo(), Gravity.Info));
        Assert.Null(AlertDiagnostics.ToDiagnostic(new TestAlertNothingToDo(), Gravity.Trace));
        Assert.Empty(AlertDiagnostics.ToDiagnostics(new Report()));
    }

    [Fact]
    public void ANullShapeIsntNamed()
    {
        Assert.Empty(AlertDiagnostics.ToDiagnostic(new TestAlertBadShape(new Shape()), Gravity.Fail)!.Shapes);
    }
}
