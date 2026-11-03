// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FClassifier.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Topology;

namespace Occt;

/// <summary>Classifies a 2D point in a face explored by a <see cref="BRepClass_FaceExplorer"/>.</summary>
/// <remarks>A working object (decision 21). The body is TopClass_FaceClassifier's (decision 285).</remarks>
public class BRepClass_FClassifier
{
    protected readonly BRepClass_FClass2dOfFClassifier myClassifier = new();
    protected BRepClass_Edge myEdge = new();
    protected double myEdgeParameter;
    protected IntRes2d_Position myPosition;
    protected bool rejected;
    protected bool nowires;

    /// <summary>An undefined classifier.</summary>
    public BRepClass_FClassifier()
    {
        myEdgeParameter = 0.0;
        rejected = false;
        nowires = true;
    }

    /// <summary>Classifies <paramref name="P"/> with the tolerance <paramref name="Tol"/> in the face explored by <paramref name="F"/>.</summary>
    public BRepClass_FClassifier(BRepClass_FaceExplorer F, in Point2D P, double Tol)
    {
        myEdgeParameter = 0.0;
        rejected = false;
        nowires = true;
        Perform(F, P, Tol);
    }

    /// <summary>Classifies <paramref name="P"/> with the tolerance <paramref name="Tol"/> in the face explored by <paramref name="F"/>.</summary>
    public void Perform(BRepClass_FaceExplorer F, in Point2D P, double Tol)
    {
        TopClass_FaceClassifier.Perform(out rejected,
                                        ref nowires,
                                        myClassifier,
                                        ref myEdge,
                                        ref myPosition,
                                        ref myEdgeParameter,
                                        F,
                                        P,
                                        Tol);
    }

    /// <summary>The result of the classification.</summary>
    public Containment State() => TopClass_FaceClassifier.State(rejected, nowires, myClassifier);

    /// <summary>True when the state was found by a rejection: it is OUT.</summary>
    public bool Rejected() => rejected;

    /// <summary>True if the face has no wire: the state is IN.</summary>
    public bool NoWires() => nowires;

    /// <summary>The edge that decided the classification; when the state is ON, the edge holding the point.</summary>
    /// <exception cref="DomainErrorException">The point was rejected.</exception>
    public BRepClass_Edge Edge() => TopClass_FaceClassifier.Edge(rejected, myEdge);

    /// <summary>The parameter on <see cref="Edge"/> that decided the classification.</summary>
    /// <exception cref="DomainErrorException">The point was rejected.</exception>
    public double EdgeParameter() => TopClass_FaceClassifier.EdgeParameter(rejected, myEdgeParameter);

    /// <summary>The position of the point on <see cref="Edge"/>.</summary>
    public IntRes2d_Position Position() => myPosition;
}
