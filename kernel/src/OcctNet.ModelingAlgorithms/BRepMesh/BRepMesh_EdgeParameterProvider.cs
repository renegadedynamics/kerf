// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_EdgeParameterProvider.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The parameters on the edge's pcurve of the discretization points, as the edge's SameParameter flag asks: the
/// stored parameters as they are, or scaled to the pcurve's range and refined by projecting the 3D points on it.
/// </summary>
/// <typeparam name="ParametersCollection">
/// The array of the stored parameters: OCCT instantiates the template over
/// <c>IMeshData::ICurveArrayAdaptorHandle</c> (<see cref="IMeshData_ParametersListArrayAdaptor{T}"/>) and
/// <c>NCollection_HArray1&lt;double&gt;</c>; both are <see cref="INCollection_ArrayRead{T}"/>.
/// </typeparam>
/// <remarks>
/// A working object: <see cref="Parameter"/> must be called for the points in order, as it follows the previous
/// one (OCCT's <c>mutable</c> fields).
/// </remarks>
public sealed class BRepMesh_EdgeParameterProvider<ParametersCollection> : Transient
    where ParametersCollection : class, INCollection_ArrayRead<double>
{
    private ParametersCollection? myParameters;

    private bool myIsSameParam;
    private double myFirstParam;

    private double myOldFirstParam;
    private double myScale;

    private double myCurParam;
    private double myFoundParam;

    private readonly BRepAdaptor_Curve myCurveAdaptor = new();

    private readonly Extrema_LocateExtPC myProjector = new();

    /// <summary>An empty provider.</summary>
    public BRepMesh_EdgeParameterProvider()
    {
        myIsSameParam = false;
        myFirstParam = 0.0;
        myOldFirstParam = 0.0;
        myScale = 0.0;
        myCurParam = 0.0;
        myFoundParam = 0.0;
    }

    /// <summary>A provider for the edge with the orientation on the face, and the stored parameters.</summary>
    public BRepMesh_EdgeParameterProvider(IMeshData_Edge theEdge,
                                          TopAbs_Orientation theOrientation,
                                          IMeshData_Face theFace,
                                          ParametersCollection theParameters)
    {
        Init(theEdge, theOrientation, theFace, theParameters);
    }

    /// <summary>Initializes the provider with the given data.</summary>
    public void Init(IMeshData_Edge theEdge,
                     TopAbs_Orientation theOrientation,
                     IMeshData_Face theFace,
                     ParametersCollection theParameters)
    {
        myParameters = theParameters;
        myIsSameParam = theEdge.GetSameParam();
        myScale = 1.0;

        // Extract actual parametric values
        TopoDS_Edge aEdge = TopoDS.Edge(theEdge.GetEdge().Oriented(theOrientation));

        myCurveAdaptor.Initialize(aEdge, theFace.GetFace());
        if (myIsSameParam)
        {
            return;
        }

        myFirstParam = myCurveAdaptor.FirstParameter();
        double aLastParam = myCurveAdaptor.LastParameter();

        myFoundParam = myCurParam = myFirstParam;

        // Extract parameters stored in polygon
        myOldFirstParam = myParameters.Value(myParameters.Lower());
        double aOldLastParam = myParameters.Value(myParameters.Upper());

        // Calculate scale factor between actual and stored parameters
        if ((myOldFirstParam != myFirstParam || aOldLastParam != aLastParam)
            && myOldFirstParam != aOldLastParam)
        {
            myScale = (aLastParam - myFirstParam) / (aOldLastParam - myOldFirstParam);
        }

        myProjector.Initialize(myCurveAdaptor,
                               myCurveAdaptor.FirstParameter(),
                               myCurveAdaptor.LastParameter(),
                               global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    /// <summary>
    /// The parameter of the point with the given index: the stored value if the edge is SameParameter, otherwise the
    /// scaled value refined by projecting <paramref name="thePoint3d"/> on the pcurve.
    /// </summary>
    public double Parameter(int theIndex, in Point3D thePoint3d)
    {
        if (myIsSameParam)
        {
            return myParameters!.Value(theIndex);
        }

        // Use scaled
        double aParam = myParameters!.Value(theIndex);

        double aPrevParam = myCurParam;
        myCurParam = myFirstParam + myScale * (aParam - myOldFirstParam);

        double aPrevFoundParam = myFoundParam;
        myFoundParam += (myCurParam - aPrevParam);

        myProjector.Perform(thePoint3d, myFoundParam);
        if (myProjector.IsDone())
        {
            double aFoundParam = myProjector.Point().Parameter();
            if ((aPrevFoundParam < myFoundParam && aPrevFoundParam < aFoundParam)
                || (aPrevFoundParam > myFoundParam && aPrevFoundParam > aFoundParam))
            {
                // Rude protection against case when amplified parameter goes before
                // previous one due to period or other reason occurred in projector.
                // Using parameter returned by projector as is can produce self-intersections.
                myFoundParam = aFoundParam;
            }
        }

        return myFoundParam;
    }

    /// <summary>The pcurve used to compute the parameters.</summary>
    public Adaptor2d_Curve2d? GetPCurve() => myCurveAdaptor.CurveOnSurface().GetCurve();
}
