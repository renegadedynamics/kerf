// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Classifier.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Classifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Classifies a 2D point against the elements of a hatcher (<see cref="Geom2dHatch_Elements"/>).</summary>
/// <remarks>
/// A working object (decision 21). The body is TopClass_FaceClassifier's (decision 285), as for BRepClass_FClassifier.
/// <see cref="Edge"/> is a shallow copy of the deciding element's curve, as OCCT copies it.
/// </remarks>
public class Geom2dHatch_Classifier
{
    protected readonly Geom2dHatch_FClass2dOfClassifier myClassifier = new();
    protected Geom2dAdaptor_Curve myEdge = new();
    protected double myEdgeParameter;
    protected IntRes2d_Position myPosition;
    protected bool rejected;
    protected bool nowires;

    /// <summary>An undefined classifier.</summary>
    public Geom2dHatch_Classifier()
    {
        myEdgeParameter = 0.0;
        rejected = false;
        nowires = true;
    }

    /// <summary>Classifies <paramref name="P"/> with the tolerance <paramref name="Tol"/> against the elements <paramref name="F"/>.</summary>
    public Geom2dHatch_Classifier(Geom2dHatch_Elements F, in gp_Pnt2d P, double Tol)
    {
        myEdgeParameter = 0.0;
        rejected = false;
        nowires = true;
        Perform(F, P, Tol);
    }

    /// <summary>Classifies <paramref name="P"/> with the tolerance <paramref name="Tol"/> against the elements <paramref name="F"/>.</summary>
    public void Perform(Geom2dHatch_Elements F, in gp_Pnt2d P, double Tol)
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
    public TopAbs_State State() => TopClass_FaceClassifier.State(rejected, nowires, myClassifier);

    /// <summary>True when the state was found by a rejection: it is OUT.</summary>
    public bool Rejected() => rejected;

    /// <summary>True if there is no element: the state is IN.</summary>
    public bool NoWires() => nowires;

    /// <summary>The element curve that decided the classification; when the state is ON, the one holding the point.</summary>
    /// <exception cref="Standard_DomainError">The point was rejected.</exception>
    public Geom2dAdaptor_Curve Edge() => TopClass_FaceClassifier.Edge(rejected, myEdge);

    /// <summary>The parameter on <see cref="Edge"/> that decided the classification.</summary>
    /// <exception cref="Standard_DomainError">The point was rejected.</exception>
    public double EdgeParameter() => TopClass_FaceClassifier.EdgeParameter(rejected, myEdgeParameter);

    /// <summary>The position of the point on <see cref="Edge"/>.</summary>
    public IntRes2d_Position Position() => myPosition;
}
