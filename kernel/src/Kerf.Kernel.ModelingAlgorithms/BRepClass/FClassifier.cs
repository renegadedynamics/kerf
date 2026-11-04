// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FClassifier.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification.Topological;
using Kerf.Engine.Foundation;

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Classification;

/// <summary>Classifies a 2D point in a face explored by a <see cref="FaceExplorer"/>.</summary>
/// <remarks>A working object (decision 21). The body is TopClass_FaceClassifier's (decision 285).</remarks>
internal class FClassifier
{
    protected readonly FClass2dOfFClassifier _classifier = new();
    protected ClassifierEdge _edge = new();
    protected double _edgeParameter;
    protected Position2D _position;
    protected bool _rejected;
    protected bool _nowires;

    /// <summary>An undefined classifier.</summary>
    public FClassifier()
    {
        _edgeParameter = 0.0;
        _rejected = false;
        _nowires = true;
    }

    /// <summary>Classifies <paramref name="p"/> with the tolerance <paramref name="tol"/> in the face explored by <paramref name="f"/>.</summary>
    public FClassifier(FaceExplorer f, in Point2D p, double tol)
    {
        _edgeParameter = 0.0;
        _rejected = false;
        _nowires = true;
        Perform(f, p, tol);
    }

    /// <summary>Classifies <paramref name="p"/> with the tolerance <paramref name="tol"/> in the face explored by <paramref name="f"/>.</summary>
    public void Perform(FaceExplorer f, in Point2D p, double tol)
    {
        TopologicalFaceClassifier.Perform(out _rejected,
                                        ref _nowires,
                                        _classifier,
                                        ref _edge,
                                        ref _position,
                                        ref _edgeParameter,
                                        f,
                                        p,
                                        tol);
    }

    /// <summary>The result of the classification.</summary>
    public Containment State() => TopologicalFaceClassifier.State(_rejected, _nowires, _classifier);

    /// <summary>True when the state was found by a rejection: it is OUT.</summary>
    public bool Rejected() => _rejected;

    /// <summary>True if the face has no wire: the state is IN.</summary>
    public bool NoWires() => _nowires;

    /// <summary>The edge that decided the classification; when the state is ON, the edge holding the point.</summary>
    /// <exception cref="DomainErrorException">The point was rejected.</exception>
    public ClassifierEdge Edge() => TopologicalFaceClassifier.Edge(_rejected, _edge);

    /// <summary>The parameter on <see cref="Edge"/> that decided the classification.</summary>
    /// <exception cref="DomainErrorException">The point was rejected.</exception>
    public double EdgeParameter() => TopologicalFaceClassifier.EdgeParameter(_rejected, _edgeParameter);

    /// <summary>The position of the point on <see cref="Edge"/>.</summary>
    public Position2D Position() => _position;
}
