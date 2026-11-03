// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_FaceDiscret.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_FaceDiscret.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Occt;
using static Kerf.Engine.Meshing.Model.StatusModel;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Meshes the model's faces, each with an algorithm the factory chooses by the type of its surface; in parallel when
/// the parameters ask for it.
/// </summary>
/// <remarks>
/// <para>
/// Faces that are reused (their triangulation fits) or failed are skipped. Each loop body creates its own algorithm
/// and writes only its own discrete face (its status and deflection, the indices of its pcurves' points) and its
/// face's triangulation; it evaluates only its face's surface adaptor, and reads the edges' 3D points, which no body
/// writes. So a parallel run gives the serial run's meshes (decision 490).
/// </para>
/// <para>
/// A face whose meshing throws a failure (<see cref="ModelAlgo.IsCaughtFailure"/>) gets
/// <see cref="Failure"/>, as OCCT's <c>catch (Standard_Failure)</c>.
/// </para>
/// </remarks>
public class FaceDiscret : ModelAlgo
{
    private readonly MeshAlgoFactory? _algoFactory;
    private ModelModel? _model;
    private ToolParameters _parameters = new();

    /// <summary>A face discretizer with the factory of meshing algorithms.</summary>
    public FaceDiscret(MeshAlgoFactory? algoFactory)
    {
        _algoFactory = algoFactory;
    }

    /// <summary>Meshes the faces of the model; false without a model or on a user break.</summary>
    protected override bool PerformInternal(ModelModel? model,
                                            in ToolParameters parameters,
                                            ProgressRange range)
    {
        _model = model;
        _parameters = parameters;
        if (_model is null)
        {
            return false;
        }

        FaceListFunctor functor = new(this, range);
        OsdParallel.For(0,
                         _model.FacesNb(),
                         functor.Process,
                         !_parameters.InParallel || _model.FacesNb() <= 1);
        functor.Close(); // the C++ functor's scope is closed by its destructor
        if (!range.More())
        {
            return false;
        }

        _model = null; // Do not hold link to model.
        return true;
    }

    // Auxiliary functor for parallel processing of Faces.
    private sealed class FaceListFunctor
    {
        private readonly FaceDiscret _algo;
        private readonly ProgressScope _scope;
        private readonly ProgressRange[] _ranges;

        public FaceListFunctor(FaceDiscret algo, ProgressRange range)
        {
            _algo = algo;
            _scope = new ProgressScope(range, "Face Discret", algo._model!.FacesNb());
            _ranges = new ProgressRange[algo._model.FacesNb()];
            for (int faceIter = 0; faceIter < algo._model.FacesNb(); ++faceIter)
            {
                _ranges[faceIter] = _scope.Next();
            }
        }

        public void Process(int faceIndex)
        {
            if (!_scope.More())
            {
                return;
            }
            using ProgressScope faceScope = new(_ranges[faceIndex], null, 1);
            _algo.Process(faceIndex, faceScope.Next());
        }

        public void Close() => _scope.Close();
    }

    // Checks existing discretization of the face and updates data model.
    private void Process(int faceIndex, ProgressRange range)
    {
        FaceModel dFace = _model!.GetFace(faceIndex);
        if (dFace.IsSet(StatusModel.Failure) || dFace.IsSet(StatusModel.Reused))
        {
            return;
        }

        try
        {
            MeshAlgo? meshingAlgo =
                _algoFactory!.GetAlgo(dFace.GetSurface().GetType(), _parameters);

            if (meshingAlgo is null)
            {
                dFace.SetStatus(StatusModel.Failure);
                return;
            }

            if (!range.More())
            {
                dFace.SetStatus(StatusModel.UserBreak);
                return;
            }

            meshingAlgo.Perform(dFace, _parameters, range);
        }
        catch (Exception exception) when (IsCaughtFailure(exception))
        {
            dFace.SetStatus(StatusModel.Failure);
        }
    }
}
