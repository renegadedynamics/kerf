// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_EdgeParameterProvider.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The parameters on the edge's pcurve of the discretization points, as the edge's SameParameter flag asks: the
/// stored parameters as they are, or scaled to the pcurve's range and refined by projecting the 3D points on it.
/// </summary>
/// <typeparam name="ParametersCollection">
/// The array of the stored parameters: OCCT instantiates the template over
/// <c>IMeshData::ICurveArrayAdaptorHandle</c> (<see cref="ParametersListArrayAdaptorModel{T}"/>) and
/// <c>NCollection_HArray1&lt;double&gt;</c>; both are <see cref="INCollection_ArrayRead{T}"/>.
/// </typeparam>
/// <remarks>
/// A working object: <see cref="Parameter"/> must be called for the points in order, as it follows the previous
/// one (OCCT's <c>mutable</c> fields).
/// </remarks>
public sealed class EdgeParameterProvider<ParametersCollection> : Transient
    where ParametersCollection : class, INCollection_ArrayRead<double>
{
    private ParametersCollection? _parameters;

    private bool _isSameParam;
    private double _firstParam;

    private double _oldFirstParam;
    private double _scale;

    private double _curParam;
    private double _foundParam;

    private readonly EdgeCurveAdaptor _curveAdaptor = new();

    private readonly LocateExtPC _projector = new();

    /// <summary>An empty provider.</summary>
    public EdgeParameterProvider()
    {
        _isSameParam = false;
        _firstParam = 0.0;
        _oldFirstParam = 0.0;
        _scale = 0.0;
        _curParam = 0.0;
        _foundParam = 0.0;
    }

    /// <summary>A provider for the edge with the orientation on the face, and the stored parameters.</summary>
    public EdgeParameterProvider(EdgeModel edge,
                                          Orientation orientation,
                                          FaceModel face,
                                          ParametersCollection parameters)
    {
        Init(edge, orientation, face, parameters);
    }

    /// <summary>Initializes the provider with the given data.</summary>
    public void Init(EdgeModel edge,
                     Orientation orientation,
                     FaceModel face,
                     ParametersCollection parameters)
    {
        _parameters = parameters;
        _isSameParam = edge.GetSameParam();
        _scale = 1.0;

        // Extract actual parametric values
        Edge aEdge = ShapeCasts.Edge(edge.GetEdge().Oriented(orientation));

        _curveAdaptor.Initialize(aEdge, face.GetFace());
        if (_isSameParam)
        {
            return;
        }

        _firstParam = _curveAdaptor.FirstParameter();
        double lastParam = _curveAdaptor.LastParameter();

        _foundParam = _curParam = _firstParam;

        // Extract parameters stored in polygon
        _oldFirstParam = _parameters.Value(_parameters.Lower());
        double oldLastParam = _parameters.Value(_parameters.Upper());

        // Calculate scale factor between actual and stored parameters
        if ((_oldFirstParam != _firstParam || oldLastParam != lastParam)
            && _oldFirstParam != oldLastParam)
        {
            _scale = (lastParam - _firstParam) / (oldLastParam - _oldFirstParam);
        }

        _projector.Initialize(_curveAdaptor,
                               _curveAdaptor.FirstParameter(),
                               _curveAdaptor.LastParameter(),
                               global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    /// <summary>
    /// The parameter of the point with the given index: the stored value if the edge is SameParameter, otherwise the
    /// scaled value refined by projecting <paramref name="point3d"/> on the pcurve.
    /// </summary>
    public double Parameter(int index, in Point3D point3d)
    {
        if (_isSameParam)
        {
            return _parameters!.Value(index);
        }

        // Use scaled
        double param = _parameters!.Value(index);

        double prevParam = _curParam;
        _curParam = _firstParam + _scale * (param - _oldFirstParam);

        double prevFoundParam = _foundParam;
        _foundParam += (_curParam - prevParam);

        _projector.Perform(point3d, _foundParam);
        if (_projector.IsDone)
        {
            double foundParam = _projector.Point().Parameter();
            if ((prevFoundParam < _foundParam && prevFoundParam < foundParam)
                || (prevFoundParam > _foundParam && prevFoundParam > foundParam))
            {
                // Rude protection against case when amplified parameter goes before
                // previous one due to period or other reason occurred in projector.
                // Using parameter returned by projector as is can produce self-intersections.
                _foundParam = foundParam;
            }
        }

        return _foundParam;
    }

    /// <summary>The pcurve used to compute the parameters.</summary>
    public Curve2DAdaptorBase? GetPCurve() => _curveAdaptor.CurveOnSurface().GetCurve();
}
