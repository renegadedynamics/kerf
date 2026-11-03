// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_FaceDiscret.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_FaceDiscret.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.IMeshData_Status;

namespace Occt;

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
/// A face whose meshing throws a failure (<see cref="IMeshTools_ModelAlgo.IsCaughtFailure"/>) gets
/// <see cref="IMeshData_Failure"/>, as OCCT's <c>catch (Standard_Failure)</c>.
/// </para>
/// </remarks>
public class BRepMesh_FaceDiscret : IMeshTools_ModelAlgo
{
    private readonly IMeshTools_MeshAlgoFactory? myAlgoFactory;
    private IMeshData_Model? myModel;
    private IMeshTools_Parameters myParameters = new();

    /// <summary>A face discretizer with the factory of meshing algorithms.</summary>
    public BRepMesh_FaceDiscret(IMeshTools_MeshAlgoFactory? theAlgoFactory)
    {
        myAlgoFactory = theAlgoFactory;
    }

    /// <summary>Meshes the faces of the model; false without a model or on a user break.</summary>
    protected override bool performInternal(IMeshData_Model? theModel,
                                            in IMeshTools_Parameters theParameters,
                                            ProgressRange theRange)
    {
        myModel = theModel;
        myParameters = theParameters;
        if (myModel is null)
        {
            return false;
        }

        FaceListFunctor aFunctor = new(this, theRange);
        OsdParallel.For(0,
                         myModel.FacesNb(),
                         aFunctor.Process,
                         !myParameters.InParallel || myModel.FacesNb() <= 1);
        aFunctor.Close(); // the C++ functor's scope is closed by its destructor
        if (!theRange.More())
        {
            return false;
        }

        myModel = null; // Do not hold link to model.
        return true;
    }

    // Auxiliary functor for parallel processing of Faces.
    private sealed class FaceListFunctor
    {
        private readonly BRepMesh_FaceDiscret myAlgo;
        private readonly ProgressScope myScope;
        private readonly ProgressRange[] myRanges;

        public FaceListFunctor(BRepMesh_FaceDiscret theAlgo, ProgressRange theRange)
        {
            myAlgo = theAlgo;
            myScope = new ProgressScope(theRange, "Face Discret", theAlgo.myModel!.FacesNb());
            myRanges = new ProgressRange[theAlgo.myModel.FacesNb()];
            for (int aFaceIter = 0; aFaceIter < theAlgo.myModel.FacesNb(); ++aFaceIter)
            {
                myRanges[aFaceIter] = myScope.Next();
            }
        }

        public void Process(int theFaceIndex)
        {
            if (!myScope.More())
            {
                return;
            }
            using ProgressScope aFaceScope = new(myRanges[theFaceIndex], null, 1);
            myAlgo.process(theFaceIndex, aFaceScope.Next());
        }

        public void Close() => myScope.Close();
    }

    // Checks existing discretization of the face and updates data model.
    private void process(int theFaceIndex, ProgressRange theRange)
    {
        IMeshData_Face aDFace = myModel!.GetFace(theFaceIndex);
        if (aDFace.IsSet(IMeshData_Failure) || aDFace.IsSet(IMeshData_Reused))
        {
            return;
        }

        try
        {
            IMeshTools_MeshAlgo? aMeshingAlgo =
                myAlgoFactory!.GetAlgo(aDFace.GetSurface().GetType(), myParameters);

            if (aMeshingAlgo is null)
            {
                aDFace.SetStatus(IMeshData_Failure);
                return;
            }

            if (!theRange.More())
            {
                aDFace.SetStatus(IMeshData_UserBreak);
                return;
            }

            aMeshingAlgo.Perform(aDFace, myParameters, theRange);
        }
        catch (Exception anException) when (IsCaughtFailure(anException))
        {
            aDFace.SetStatus(IMeshData_Failure);
        }
    }
}
