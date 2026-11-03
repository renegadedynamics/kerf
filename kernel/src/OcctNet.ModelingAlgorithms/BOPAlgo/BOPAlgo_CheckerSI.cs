// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CheckerSI.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CheckerSI.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>A face intersected with itself (OCCT's file-local BOPAlgo_FaceSelfIntersect).</summary>
/// <remarks>
/// OCCT derives it from IntTools_FaceFace and BOPAlgo_ParallelAlgo; C# has one base, so it holds a
/// <see cref="BOPAlgo_Options"/> for the run-parallel flag and the user break, as the pave filler's BOPAlgo_FaceFace
/// (decision 786). As in OCCT, an exception isn't caught here: it ends the checker with
/// <see cref="BOPAlgo_AlertIntersectionFailed"/>.
/// </remarks>
internal sealed class BOPAlgo_FaceSelfIntersect : IntTools_FaceFace, BOPTools_Parallel.ISolver
{
    private readonly BOPAlgo_Options myOptions = new();
    private ProgressRange myProgressRange;
    private int myIF;
    private double myTolF;
    private TopoDS_Face myF = new();

    public BOPAlgo_FaceSelfIntersect()
    {
        myIF = -1;
        myTolF = 1.0e-7;
    }

    //
    public void SetIndex(int nF) => myIF = nF;

    //
    public int IndexOfFace() => myIF;

    //
    public void SetFace(in TopoDS_Face aF) => myF = aF;

    //
    public TopoDS_Face Face() => myF;

    //
    public void SetTolF(double aTolF) => myTolF = aTolF;

    //
    public double TolF() => myTolF;

    public void SetProgressRange(ProgressRange theRange) => myProgressRange = theRange;

    public void SetRunParallel(bool theFlag) => myOptions.SetRunParallel(theFlag);

    //
    public void Perform()
    {
        using var aPS = new ProgressScope(myProgressRange, null, 1);
        if (myOptions.UserBreak(aPS))
        {
            return;
        }
        Perform(myF, myF, myOptions.RunParallel());
    }
}

/// <summary>
/// Checks a shape on self-interference: the pave filler intersects the sub-shapes of its single argument with each
/// other, faces are intersected with themselves, and vertices, edges, faces and solids are tested against solids. The
/// pairs of interfering sub-shapes are the data structure's <see cref="BOPDS_DS.Interferences"/>.
/// </summary>
/// <remarks>
/// <para>
/// The level of check (<see cref="SetLevelOfCheck"/>) limits the types of interference: 0 V/V; 1 and V/E; 2 and E/E;
/// 3 and V/F; 4 and E/F; 5 and F/F (with the faces' self-intersection); 6 and V/S; 7 and E/S; 8 and F/S; 9 and S/S
/// (the default, all).
/// </para>
/// <para>
/// The checker is non-destructive (OCCT sets the mode in its constructor and its <c>Init</c> doesn't reset it), so the
/// argument is never written; a frozen argument is the usual case. The class is partial over OCCT's two files.
/// </para>
/// </remarks>
public partial class BOPAlgo_CheckerSI : BOPAlgo_PaveFiller
{
    protected int myLevelOfCheck;

    public BOPAlgo_CheckerSI()
    {
        myLevelOfCheck = BOPDS_DS.NbInterfTypes() - 1;
        myNonDestructive = true;
        SetAvoidBuildPCurve(true);
    }

    /// <summary>Sets the level of checking the shape on self-interference (0 to 9; other values are ignored).</summary>
    public void SetLevelOfCheck(int theLevel)
    {
        int aNbLists;
        //
        aNbLists = BOPDS_DS.NbInterfTypes();
        if (theLevel >= 0 && theLevel < aNbLists)
        {
            myLevelOfCheck = theLevel;
        }
    }

    protected override void Init(ProgressRange theRange)
    {
        Clear();
        //
        // 1. myDS
        myDS = new BOPDS_DS();
        myDS.SetArguments(myArguments);
        myDS.Init(myFuzzyValue);
        //
        // 2 myContext
        myContext = new IntTools_Context();
        //
        // 3.myIterator
        BOPDS_IteratorSI theIterSI = new();
        theIterSI.SetDS(myDS);
        theIterSI.Prepare(myContext, myUseOBB, myFuzzyValue);
        theIterSI.UpdateByLevelOfCheck(myLevelOfCheck);
        //
        myIterator = theIterSI;
    }

    /// <summary>Checks the single argument; an exception becomes <see cref="BOPAlgo_AlertIntersectionFailed"/>.</summary>
    public override void Perform(ProgressRange theRange = default)
    {
        try
        {
            //
            if (myArguments.Extent() != 1)
            {
                AddError(new BOPAlgo_AlertMultipleArguments());
                return;
            }
            //
            // OCCT.NET: OCCT's non-destructive filler still raises the tolerances of argument vertices that a section
            // curve passes through (PutPaveOnCurve, occt-issues.md), so the check of a shape holding frozen TShapes
            // runs on an exact copy of it (decision 882); the data structure then holds the copy's sub-shapes, and
            // Original maps them back to the argument's.
            TopoDS_Shape anInput = myArguments.First();
            myCopyAtEntry = new BRepTools_CopyAtEntry(anInput);
            if (myCopyAtEntry.IsCopied())
            {
                myArguments = new NCollection_List<TopoDS_Shape>();
                myArguments.Append(myCopyAtEntry.Shape());
            }
            try
            {
                PerformChecks(theRange);
            }
            finally
            {
                if (myCopyAtEntry.IsCopied())
                {
                    myArguments = new NCollection_List<TopoDS_Shape>();
                    myArguments.Append(anInput);
                }
            }
        }
        //
        catch (FailureException)
        {
            AddError(new BOPAlgo_AlertIntersectionFailed());
        }
    }

    private BRepTools_CopyAtEntry? myCopyAtEntry;

    /// <summary>
    /// The argument's own sub-shape for a shape of the data structure: the shape itself, unless the argument was
    /// checked on a copy (it held frozen TShapes), when it is the original of the copy's sub-shape.
    /// </summary>
    internal TopoDS_Shape Original(in TopoDS_Shape theShape) =>
        myCopyAtEntry is not null && myCopyAtEntry.IsCopied() ? myCopyAtEntry.Original(theShape) : theShape;

    // OCCT's Perform from its progress scope on (the body of its try block).
    private void PerformChecks(ProgressRange theRange)
    {
        {
            using var aPS = new ProgressScope(theRange, "Checking shape on self-intersection", 10);
            // Perform intersection of sub shapes
            base.Perform(aPS.Next(8));
            if (UserBreak(aPS))
            {
                return;
            }
            //
            CheckFaceSelfIntersection(aPS.Next());

            using var aPSZZ = new ProgressScope(aPS.Next(), null, 4);
            // Perform intersection with solids
            if (!HasErrors())
            {
                PerformVZ(aPSZZ.Next());
            }
            //
            if (!HasErrors())
            {
                PerformEZ(aPSZZ.Next());
            }
            //
            if (!HasErrors())
            {
                PerformFZ(aPSZZ.Next());
            }
            //
            if (!HasErrors())
            {
                PerformZZ(aPSZZ.Next());
            }
            //
            if (HasErrors())
            {
                return;
            }

            // Treat the intersection results
            PostTreat();
        }
    }

    /// <summary>Puts the interferences of the source shapes into the table of interferences.</summary>
    protected void PostTreat()
    {
        int i, aNb, n1, n2;
        BOPDS_Pair aPK = new();
        //
        BOPDS_MapOfPair aMPK = myDS!.Interferences();

        // 0
        NCollection_DynamicArray<BOPDS_InterfVV> aVVs = myDS.InterfVV();
        aNb = aVVs.Length();
        for (i = 0; i != aNb; ++i)
        {
            BOPDS_InterfVV aVV = aVVs.Value(i);
            aVV.Indices(out n1, out n2);
            if (myDS.IsNewShape(n1) || myDS.IsNewShape(n2))
            {
                continue;
            }
            aPK.SetIndices(n1, n2);
            aMPK.Add(aPK);
        }
        //
        // 1
        NCollection_DynamicArray<BOPDS_InterfVE> aVEs = myDS.InterfVE();
        aNb = aVEs.Length();
        for (i = 0; i != aNb; ++i)
        {
            BOPDS_InterfVE aVE = aVEs.Value(i);
            aVE.Indices(out n1, out n2);
            if (myDS.IsNewShape(n1) || myDS.IsNewShape(n2))
            {
                continue;
            }
            aPK.SetIndices(n1, n2);
            aMPK.Add(aPK);
        }
        //
        // 2
        NCollection_DynamicArray<BOPDS_InterfEE> aEEs = myDS.InterfEE();
        aNb = aEEs.Length();
        for (i = 0; i != aNb; ++i)
        {
            BOPDS_InterfEE aEE = aEEs.Value(i);
            aEE.Indices(out n1, out n2);
            if (myDS.IsNewShape(n1) || myDS.IsNewShape(n2))
            {
                continue;
            }
            aPK.SetIndices(n1, n2);
            aMPK.Add(aPK);
        }
        //
        // 3
        NCollection_DynamicArray<BOPDS_InterfVF> aVFs = myDS.InterfVF();
        aNb = aVFs.Length();
        for (i = 0; i != aNb; ++i)
        {
            BOPDS_InterfVF aVF = aVFs.Value(i);
            aVF.Indices(out n1, out n2);
            if (myDS.IsNewShape(n1) || myDS.IsNewShape(n2))
            {
                continue;
            }
            aPK.SetIndices(n1, n2);
            aMPK.Add(aPK);
        }
        //
        // 4
        NCollection_DynamicArray<BOPDS_InterfEF> aEFs = myDS.InterfEF();
        aNb = aEFs.Length();
        for (i = 0; i != aNb; ++i)
        {
            BOPDS_InterfEF aEF = aEFs.Value(i);
            if (aEF.CommonPart().Type() == TopAbs_SHAPE)
            {
                continue;
            }
            aEF.Indices(out n1, out n2);
            if (myDS.IsNewShape(n1) || myDS.IsNewShape(n2))
            {
                continue;
            }
            aPK.SetIndices(n1, n2);
            aMPK.Add(aPK);
        }
        //
        // 5
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS.InterfFF();
        aNb = aFFs.Length();
        for (i = 0; i != aNb; ++i)
        {
            bool bTangentFaces, bFlag;
            int aNbC, aNbP, j, iFound;
            //
            BOPDS_InterfFF aFF = aFFs.Value(i);
            aFF.Indices(out n1, out n2);
            //
            bTangentFaces = aFF.TangentFaces();
            aNbP = aFF.Points().Length();
            NCollection_DynamicArray<BOPDS_Curve> aVC = aFF.Curves();
            aNbC = aVC.Length();
            if (aNbP == 0 && aNbC == 0 && !bTangentFaces)
            {
                continue;
            }
            //
            iFound = (n1 == n2) ? 1 : 0;
            // case of self-intersection inside one face
            if (iFound == 0)
            {
                if (bTangentFaces)
                {
                    TopoDS_Face aF1 = TopoDS.Face(myDS.Shape(n1));
                    TopoDS_Face aF2 = TopoDS.Face(myDS.Shape(n2));
                    bFlag = BOPTools_AlgoTools.AreFacesSameDomain(aF1, aF2, myContext!, myFuzzyValue);
                    if (bFlag)
                    {
                        ++iFound;
                    }
                }
                else
                {
                    for (j = 0; j != aNbC; ++j)
                    {
                        BOPDS_Curve aNC = aVC.Value(j);
                        NCollection_List<BOPDS_PaveBlock> aLPBC = aNC.PaveBlocks();
                        if (aLPBC.Extent() != 0)
                        {
                            ++iFound;
                            break;
                        }
                    }
                }
            }
            //
            if (iFound == 0)
            {
                continue;
            }
            //
            aPK.SetIndices(n1, n2);
            aMPK.Add(aPK);
        }
        //
        //
        // 6
        NCollection_DynamicArray<BOPDS_InterfVZ> aVZs = myDS.InterfVZ();
        aNb = aVZs.Length();
        for (i = 0; i != aNb; ++i)
        {
            //
            BOPDS_InterfVZ aVZ = aVZs.Value(i);
            aVZ.Indices(out n1, out n2);
            if (myDS.IsNewShape(n1) || myDS.IsNewShape(n2))
            {
                continue;
            }
            aPK.SetIndices(n1, n2);
            aMPK.Add(aPK);
        }
        //
        // 7
        NCollection_DynamicArray<BOPDS_InterfEZ> aEZs = myDS.InterfEZ();
        aNb = aEZs.Length();
        for (i = 0; i != aNb; ++i)
        {
            //
            BOPDS_InterfEZ aEZ = aEZs.Value(i);
            aEZ.Indices(out n1, out n2);
            aPK.SetIndices(n1, n2);
            aMPK.Add(aPK);
        }
        //
        // 8
        NCollection_DynamicArray<BOPDS_InterfFZ> aFZs = myDS.InterfFZ();
        aNb = aFZs.Length();
        for (i = 0; i != aNb; ++i)
        {
            //
            BOPDS_InterfFZ aFZ = aFZs.Value(i);
            aFZ.Indices(out n1, out n2);
            aPK.SetIndices(n1, n2);
            aMPK.Add(aPK);
        }
        //
        // 9
        NCollection_DynamicArray<BOPDS_InterfZZ> aZZs = myDS.InterfZZ();
        aNb = aZZs.Length();
        for (i = 0; i != aNb; ++i)
        {
            //
            BOPDS_InterfZZ aZZ = aZZs.Value(i);
            aZZ.Indices(out n1, out n2);
            aPK.SetIndices(n1, n2);
            aMPK.Add(aPK);
        }
    }

    /// <summary>Intersects the faces that may self-intersect (not planes, cylinders, cones, spheres or ring tori) with themselves.</summary>
    protected void CheckFaceSelfIntersection(ProgressRange theRange)
    {
        if (myLevelOfCheck < 5)
        {
            return;
        }

        BOPDS_Pair aPK = new();

        BOPDS_MapOfPair aMPK = myDS!.Interferences();
        aMPK.Clear();

        List<BOPAlgo_FaceSelfIntersect> aVFace = [];

        int aNbS = myDS.NbSourceShapes();

        using var aPSOuter = new ProgressScope(theRange, null, 1);

        //
        for (int i = 0; i < aNbS; i++)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() != TopAbs_FACE)
            {
                continue;
            }
            //
            TopoDS_Face aF = TopoDS.Face(aSI.Shape());
            BRepAdaptor_Surface BAsurf = new(aF, false);
            SurfaceKind aSurfType = BAsurf.GetType();
            if (aSurfType == Plane || aSurfType == SurfaceKind.Cylinder || aSurfType == SurfaceKind.Cone
                || aSurfType == SurfaceKind.Sphere)
            {
                continue;
            }

            if (aSurfType == SurfaceKind.Torus)
            {
                Torus aTorus = BAsurf.Torus();
                double aMajorRadius = aTorus.MajorRadius();
                double aMinorRadius = aTorus.MinorRadius();
                if (aMajorRadius > aMinorRadius + global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    continue;
                }
            }

            double aTolF = BRep_Tool.Tolerance(aF);

            BOPAlgo_FaceSelfIntersect aFaceSelfIntersect = new();
            aVFace.Add(aFaceSelfIntersect);
            //
            aFaceSelfIntersect.SetRunParallel(myRunParallel);
            aFaceSelfIntersect.SetIndex(i);
            aFaceSelfIntersect.SetFace(aF);
            aFaceSelfIntersect.SetTolF(aTolF);
        }

        int aNbFace = aVFace.Count;
        using var aPSParallel = new ProgressScope(aPSOuter.Next(), "Checking surface on self-intersection", aNbFace);
        for (int iF = 0; iF < aNbFace; ++iF)
        {
            aVFace[iF].SetProgressRange(aPSParallel.Next());
        }
        //======================================================
        BOPTools_Parallel.Perform(myRunParallel, aVFace);
        //======================================================
        if (UserBreak(aPSOuter))
        {
            return;
        }
        //
        for (int k = 0; k < aNbFace; k++)
        {
            BOPAlgo_FaceSelfIntersect aFaceSelfIntersect = aVFace[k];
            //
            int nF = aFaceSelfIntersect.IndexOfFace();

            bool bIsDone = aFaceSelfIntersect.IsDone();
            if (bIsDone)
            {
                NCollection_Sequence<IntTools_Curve> aCvsX = aFaceSelfIntersect.Lines();
                NCollection_Sequence<IntTools_PntOn2Faces> aPntsX = aFaceSelfIntersect.Points();
                //
                int aNbCurves = aCvsX.Length();
                int aNbPoints = aPntsX.Length();
                //
                if (aNbCurves != 0 || aNbPoints != 0)
                {
                    aPK.SetIndices(nF, nF);
                    aMPK.Add(aPK);
                }
            }
        }
    }
}
