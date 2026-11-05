// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Classifier.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Classifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification.Topological;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Classification.Hatching;

/// <summary>Classifies a 2D point against the elements of a hatcher (<see cref="Elements2D"/>).</summary>
/// <remarks>
/// A working object (decision 21). The body is TopClass_FaceClassifier's (decision 285), as for BRepClass_FClassifier.
/// <see cref="Edge"/> is a shallow copy of the deciding element's curve, as OCCT copies it.
/// </remarks>
internal class Classifier2D
{
    protected readonly FClass2dOfClassifier2D _classifier = new();
    protected CurveAdaptor2D _edge = new();
    protected double _edgeParameter;
    protected Position2D _position;
    protected bool _rejected;
    protected bool _nowires;

    /// <summary>An undefined classifier.</summary>
    public Classifier2D()
    {
        _edgeParameter = 0.0;
        _rejected = false;
        _nowires = true;
    }

    /// <summary>Classifies <paramref name="p"/> with the tolerance <paramref name="tol"/> against the elements <paramref name="f"/>.</summary>
    public Classifier2D(Elements2D f, in Point2D p, double tol)
    {
        _edgeParameter = 0.0;
        _rejected = false;
        _nowires = true;
        Perform(f, p, tol);
    }

    /// <summary>Classifies <paramref name="p"/> with the tolerance <paramref name="tol"/> against the elements <paramref name="f"/>.</summary>
    public void Perform(Elements2D f, in Point2D p, double tol)
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

    /// <summary>True if there is no element: the state is IN.</summary>
    public bool NoWires() => _nowires;

    /// <summary>The element curve that decided the classification; when the state is ON, the one holding the point.</summary>
    /// <exception cref="DomainErrorException">The point was rejected.</exception>
    public CurveAdaptor2D Edge() => TopologicalFaceClassifier.Edge(_rejected, _edge);

    /// <summary>The parameter on <see cref="Edge"/> that decided the classification.</summary>
    /// <exception cref="DomainErrorException">The point was rejected.</exception>
    public double EdgeParameter() => TopologicalFaceClassifier.EdgeParameter(_rejected, _edgeParameter);

    /// <summary>The position of the point on <see cref="Edge"/>.</summary>
    public Position2D Position() => _position;
}
