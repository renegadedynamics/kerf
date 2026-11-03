// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakerVolume.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakerVolume.lxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_MakerVolume.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Builds solids from a set of shapes (faces, shells, solids; edges and vertices become internal parts): the shapes
/// are intersected (General Fuse), their split faces are put into a box enclosing them all, the solids are built from
/// the faces by <see cref="BOPAlgo_BuilderSolid"/>, and the solid holding the box is removed.
/// </summary>
/// <remarks>
/// With <see cref="SetIntersect"/> false the arguments are not intersected: they must already make closed volumes
/// that share their faces. The result is a solid, or a compound of solids. <see cref="Box"/> and <see cref="Faces"/>
/// return the box and the faces the solids were built from (OCCT's working data; the box is published frozen).
/// </remarks>
public class BOPAlgo_MakerVolume : BOPAlgo_Builder
{
    protected bool myIntersect;
    protected Box myBBox = new();
    protected Solid mySBox = new();
    protected NCollection_List<Shape> myFaces = new();
    protected bool myAvoidInternalShapes;

    /// <summary>Empty constructor.</summary>
    public BOPAlgo_MakerVolume()
    {
        myIntersect = true;
        myAvoidInternalShapes = false;
    }

    /// <summary>Clears the data.</summary>
    public override void Clear()
    {
        base.Clear();
        myIntersect = true;
        myBBox = new Box();
        mySBox.Nullify();
        myFaces.Clear();
        myAvoidInternalShapes = false;
    }

    /// <summary>Sets whether the arguments are intersected (true by default).</summary>
    public void SetIntersect(bool bIntersect) => myIntersect = bIntersect;

    /// <summary>Whether the arguments are intersected.</summary>
    public bool IsIntersect() => myIntersect;

    /// <summary>The solid box enclosing the faces (frozen).</summary>
    public Solid Box()
    {
        mySBox.TShape()?.FreezeGraph();
        return mySBox;
    }

    /// <summary>The solid box, unfrozen, for ported code.</summary>
    internal Solid InternalBox() => mySBox;

    /// <summary>The faces the solids are built from (each face in both orientations, and the box's faces).</summary>
    public NCollection_List<Shape> Faces() => myFaces;

    /// <summary>Prevents the addition of internal parts into the result's solids (they are added by default).</summary>
    public void SetAvoidInternalShapes(bool theAvoidInternal) => myAvoidInternalShapes = theAvoidInternal;

    /// <summary>Whether the internal parts are left out of the result.</summary>
    public bool IsAvoidInternalShapes() => myAvoidInternalShapes;

    protected override void CheckData()
    {
        if (myArguments.IsEmpty())
        {
            AddError(new BOPAlgo_AlertTooFewArguments()); // no arguments to process
            return;
        }
        //
        CheckFiller();
    }

    /// <summary>Performs the operation.</summary>
    public override void Perform(ProgressRange theRange = default)
    {
        using var aPS = new ProgressScope(theRange, "Performing MakeVolume operation", 10);
        double anInterPart = myIntersect ? 9 : 0.5;
        double aBuildPart = 10.0 - anInterPart;

        GetReport().Clear();
        //
        if (myEntryPoint == 1)
        {
            myPaveFiller = null;
        }
        //
        BOPAlgo_PaveFiller pPF = new();
        //
        if (!myIntersect)
        {
            // if there is no need to intersect the arguments, then it is necessary
            // to create the compound of them and use it as one argument
            Compound anArgs = new();
            BrepBuilder aBB = new();
            NCollection_List<Shape>.Iterator aIt = new();
            NCollection_List<Shape> aLS = new();
            //
            aBB.MakeCompound(ref anArgs);
            aIt.Initialize(myArguments);
            for (; aIt.More(); aIt.Next())
            {
                Shape aS = aIt.Value();
                aBB.Add(anArgs, aS);
            }
            aLS.Append(anArgs);
            //
            pPF.SetArguments(aLS);
        }
        else
        {
            pPF.SetArguments(myArguments);
        }
        //
        pPF.SetRunParallel(myRunParallel);
        pPF.SetFuzzyValue(myFuzzyValue);
        pPF.SetNonDestructive(myNonDestructive);
        pPF.SetGlue(myGlue);
        pPF.SetUseOBB(myUseOBB);
        pPF.Perform(aPS.Next(anInterPart));
        //
        myEntryPoint = 1;
        PerformInternal(pPF, aPS.Next(aBuildPart));
    }

    protected override void PerformInternal1(BOPAlgo_PaveFiller theFiller, ProgressRange theRange)
    {
        using var aPS = new ProgressScope(theRange, "Building volumes", 100);
        myPaveFiller = theFiller;
        myDS = myPaveFiller.PDS();
        myContext = myPaveFiller.Context();
        //
        // 1. CheckData
        CheckData();
        if (HasErrors())
        {
            return;
        }
        //
        // 2. Prepare
        Prepare();
        if (HasErrors())
        {
            return;
        }
        //
        BOPAlgo_PISteps aSteps = new((int)BOPAlgo_PIOperation.PIOperation_Last);
        analyzeProgress(100.0, aSteps);

        // 3. Fill Images
        if (myIntersect)
        {
            // 3.1. Vertices
            FillImagesVertices(aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatVertices)));
            if (HasErrors())
            {
                return;
            }
            // 3.2. Edges
            FillImagesEdges(aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatEdges)));
            if (HasErrors())
            {
                return;
            }
            // 3.3. Wires
            FillImagesContainers(ShapeType.Wire, aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatWires)));
            if (HasErrors())
            {
                return;
            }
            // 3.4. Faces
            FillImagesFaces(aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatFaces)));
            if (HasErrors())
            {
                return;
            }
        }
        //
        // 4. Collect faces
        CollectFaces();
        if (HasErrors())
        {
            return;
        }
        //
        NCollection_Map<Shape> aBoxFaces = new(ShapeHasher.Instance);
        NCollection_List<Shape> aLSR = new();
        //
        // 5. Create bounding box
        MakeBox(aBoxFaces);
        //
        // 6. Make volumes
        BuildSolids(aLSR, aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_BuildSolids)));
        if (HasErrors())
        {
            return;
        }
        //
        // 7. Treat the result
        RemoveBox(aLSR, aBoxFaces);
        //
        // 8. Fill internal shapes
        FillInternalShapes(aLSR);
        //
        // 9. Build Result
        BuildShape(aLSR);
        //
        // 10. History
        PrepareHistory(aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_FillHistory)));
        if (HasErrors())
        {
            return;
        }
        //
        // 11. Post-treatment
        PostTreat(aPS.Next(aSteps.GetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_PostTreat)));
    }

    /// <summary>The steps of the operation (OCCT's nested enum, after the Builder's).</summary>
    protected new enum BOPAlgo_PIOperation
    {
        PIOperation_BuildSolids = (int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_Last,
        PIOperation_Last,
    }

    protected override void fillPISteps(BOPAlgo_PISteps theSteps)
    {
        NbShapes aNbShapes = getNbShapes();
        if (myIntersect)
        {
            theSteps.SetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatVertices, aNbShapes.NbVertices());
            theSteps.SetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatEdges, aNbShapes.NbEdges());
            theSteps.SetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatWires, aNbShapes.NbWires());
            theSteps.SetStep((int)BOPAlgo_Builder.BOPAlgo_PIOperation.PIOperation_TreatFaces, 50 * aNbShapes.NbFaces());
        }
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_BuildSolids, 50 * aNbShapes.NbFaces());
    }

    /// <summary>Collects all faces.</summary>
    protected void CollectFaces()
    {
        //
        int i, aNbShapes;
        NCollection_List<Shape>.Iterator aIt = new();
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
        //
        aNbShapes = myDS!.NbSourceShapes();
        for (i = 0; i < aNbShapes; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() != ShapeType.Face)
            {
                continue;
            }
            //
            Box aB = aSI.Box();
            myBBox.Add(aB);
            //
            Shape aF = aSI.Shape();
            if (myImages.IsBound(aF))
            {
                NCollection_List<Shape> aLFIm = myImages.Find(aF);
                aIt.Initialize(aLFIm);
                for (; aIt.More(); aIt.Next())
                {
                    Shape aFIm = aIt.Value();
                    if (aMFence.Add(aFIm))
                    {
                        AddFace(aFIm, myFaces);
                    }
                }
            }
            else
            {
                AddFace(aF, myFaces);
            }
        }
    }

    /// <summary>Makes the solid box.</summary>
    protected void MakeBox(NCollection_Map<Shape> theBoxFaces)
    {
        //
        double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax, anExt;
        //
        anExt = Math.Sqrt(myBBox.SquareExtent()) * 0.5;
        myBBox.Enlarge(anExt);
        myBBox.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);
        //
        Point3D aPMin = new(aXmin, aYmin, aZmin), aPMax = new(aXmax, aYmax, aZmax);
        //
        mySBox = new BRepPrimAPI_MakeBox(aPMin, aPMax).InternalSolid();
        //
        Explorer aExp = new(mySBox, ShapeType.Face);
        for (; aExp.More(); aExp.Next())
        {
            Shape aF = aExp.Current();
            myFaces.Append(aF);
            theBoxFaces.Add(aF);
        }
    }

    /// <summary>Builds the solids.</summary>
    protected void BuildSolids(NCollection_List<Shape> theLSR, ProgressRange theRange)
    {
        BOPAlgo_BuilderSolid aBS = new();
        //
        aBS.SetShapes(myFaces);
        aBS.SetRunParallel(myRunParallel);
        aBS.SetAvoidInternalShapes(myAvoidInternalShapes);
        aBS.Perform(theRange);
        if (aBS.HasErrors())
        {
            AddError(new BOPAlgo_AlertSolidBuilderFailed()); // SolidBuilder failed
            return;
        }
        //
        myReport.Merge(aBS.GetReport());
        //
        theLSR.Assign(aBS.Areas());
    }

    /// <summary>Removes the covering box.</summary>
    protected void RemoveBox(NCollection_List<Shape> theLSR, NCollection_Map<Shape> theBoxFaces)
    {
        //
        NCollection_List<Shape>.Iterator aIt = new();
        Explorer aExp = new();
        bool bFound;
        //
        bFound = false;
        aIt.Initialize(theLSR);
        for (; aIt.More(); aIt.Next())
        {
            Shape aSR = aIt.Value();
            //
            aExp.Init(aSR, ShapeType.Face);
            for (; aExp.More(); aExp.Next())
            {
                Shape aF = aExp.Current();
                if (theBoxFaces.Contains(aF))
                {
                    bFound = true;
                    theLSR.Remove(aIt);
                    break;
                }
            }
            if (bFound)
            {
                break;
            }
        }
    }

    /// <summary>Builds the result.</summary>
    protected void BuildShape(NCollection_List<Shape> theLSR)
    {
        if (theLSR.Extent() == 1)
        {
            myShape = theLSR.First();
        }
        else
        {
            BrepBuilder aBB = new();
            NCollection_List<Shape>.Iterator aIt = new();
            //
            aIt.Initialize(theLSR);
            for (; aIt.More(); aIt.Next())
            {
                Shape aSol = aIt.Value();
                aBB.Add(myShape, aSol);
            }
        }
    }

    /// <summary>Fills the solids with the internal shapes (the arguments' edges and vertices).</summary>
    protected void FillInternalShapes(NCollection_List<Shape> theLSR)
    {
        if (myAvoidInternalShapes)
        {
            return;
        }

        // Get all non-compound shapes
        NCollection_List<Shape> aLSC = new();
        // Fence map
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);

        NCollection_List<Shape>.Iterator itLA = new(myDS!.Arguments());
        for (; itLA.More(); itLA.Next())
        {
            BOPTools_AlgoTools.TreatCompound(itLA.Value(), aLSC, aMFence);
        }

        // Get only edges and vertices from arguments
        NCollection_List<Shape> aLVE = new();

        itLA.Initialize(aLSC);
        for (; itLA.More(); itLA.Next())
        {
            Shape aS = itLA.Value();
            ShapeType aType = aS.ShapeType();
            if (aType == ShapeType.Wire)
            {
                for (Iterator it = new(aS); it.More(); it.Next())
                {
                    Shape aSS = it.Value();
                    if (aMFence.Add(aSS))
                    {
                        aLVE.Append(aSS);
                    }
                }
            }
            else if (aType == ShapeType.Vertex || aType == ShapeType.Edge)
            {
                aLVE.Append(aS);
            }
        }

        BOPAlgo_Tools.FillInternals(theLSR, aLVE, myImages, myContext!);
    }

    private static void AddFace(in Shape theF, NCollection_List<Shape> theLF)
    {
        Shape aFF = theF;
        aFF.Orientation(Forward);
        theLF.Append(aFF);
        aFF.Orientation(Reversed);
        theLF.Append(aFF);
    }
}
