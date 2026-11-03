// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Solid.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Solid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepCheck_Status;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// The checks of a solid on its own (<see cref="Minimum"/>): shells sharing faces
/// (<see cref="BRepCheck_InvalidImbricationOfShells"/>), parts that aren't shells and aren't internal, or only internal
/// shells (<see cref="BRepCheck_BadOrientationOfSubshape"/>), closed shells outside one another
/// (<see cref="BRepCheck_SubshapeNotInShape"/>) and more than one closed shell that isn't a hole
/// (<see cref="BRepCheck_EnclosedRegion"/>).
/// </summary>
/// <remarks>
/// Each closed shell is classified in a solid of its own, built for the check with the internal builder and never
/// handed out; the checked solid is only read.
/// </remarks>
public class BRepCheck_Solid : BRepCheck_Result
{
    // OCCT's BRepCheck_ToolSolid (a class local to the .cxx). Its BRepCheck_HSC, a handle holding the classifier, is
    // the classifier itself.
    private sealed class BRepCheck_ToolSolid
    {
        private bool myIsHole;
        private Point3D myPnt;
        private double myPntTol;
        private Solid mySolid;
        private BRepClass3d_SolidClassifier? myHSC;

        public BRepCheck_ToolSolid()
        {
            myIsHole = false;
            myPntTol = global::Kerf.Engine.Foundation.Precision.Confusion();
            myPnt = new Point3D(-1.0, -1.0, -1.0);
            mySolid = new Solid();
        }

        public void SetSolid(in Solid aZ) => mySolid = aZ;

        public Solid Solid() => mySolid;

        public bool IsHole() => myIsHole;

        public Point3D InnerPoint() => myPnt;

        public double CheckTol() => myPntTol;

        public bool IsOut(BRepCheck_ToolSolid aOther)
        {
            bool bFlag;
            Containment aState;
            //
            BRepClass3d_SolidClassifier aSC = myHSC!;
            //
            aSC.Perform(aOther.InnerPoint(), aOther.CheckTol());
            aState = aSC.State();
            bFlag = (aState == Out);
            //
            return bFlag;
        }

        public void Init()
        {
            double aT, aPAR_T;
            var aExp = new Explorer();
            //
            // 0.myHSC
            myHSC = new BRepClass3d_SolidClassifier();
            //
            BRepClass3d_SolidClassifier aSC = myHSC;
            // 1. Load
            aSC.Load(mySolid);
            //
            // 2. myIsHole
            aSC.PerformInfinitePoint(RealSmall());
            myIsHole = (aSC.State() == In);
            //
            // 3. myPnt
            aPAR_T = 0.43213918; // 10*e^(-PI)
            aExp.Init(mySolid, ShapeType.Edge);
            for (; aExp.More(); aExp.Next())
            {
                Edge aE = ShapeCasts.Edge(aExp.Current());
                if (!Tool.Degenerated(aE))
                {
                    Curve3D aC3D = Tool.Curve(aE, out double aT1, out double aT2)!;
                    aT = (1.0 - aPAR_T) * aT1 + aPAR_T * aT2;
                    myPnt = aC3D.Value(aT);
                    myPntTol = Tool.Tolerance(aE);
                    break;
                }
            }
        }
    }

    /// <summary>Checks the solid on its own (<see cref="Minimum"/>).</summary>
    public BRepCheck_Solid(in Solid S)
    {
        Init(S);
    }

    /// <summary>Nothing more than the minimum.</summary>
    public override void Blind()
    {
        if (!myBlind)
        {
            // nothing more than in the minimum
            myBlind = true;
        }
    }

    /// <summary>Nothing: a solid has no checks in a context.</summary>
    public override void InContext(in Shape S)
    {
    }

    /// <summary>
    /// Checks the solid per se: shells that overlap, detached non-internal parts, and the closed non-internal shells
    /// (outside one another, or more than one enclosing a region).
    /// </summary>
    public override void Minimum()
    {
        if (myMin)
        {
            return;
        }

        myMin = true;
        //
        bool bFound, bIsHole, bFlag;
        int i, j, aNbVTS, aNbVTS1, iCntSh, iCntShInt;
        Solid aZ = new();
        var aIt = new Iterator();
        var aItF = new Iterator();
        var aBB = new Builder();
        var aExp = new Explorer();
        var aMSS = new NCollection_Map<Shape>(ShapeHasher.Instance);
        Orientation aOr;
        var aVTS = new List<BRepCheck_ToolSolid>();

        var aNewList = new NCollection_List<BRepCheck_Status>();
        NCollection_List<BRepCheck_Status> aLST = myMap.Bound(myShape, aNewList);
        aLST.Append(BRepCheck_NoError);
        //
        //-------------------------------------------------
        // 1. InvalidImbricationOfShells
        bFound = false;
        aExp.Init(myShape, ShapeType.Face);
        for (; !bFound && aExp.More(); aExp.Next())
        {
            Shape aF = aExp.Current();
            if (!aMSS.Add(aF))
            {
                BRepCheck.Add(aLST, BRepCheck_InvalidImbricationOfShells);
                bFound = !bFound;
            }
        }

        //
        //-------------------------------------------------
        // 2.
        //    - Too many growths,
        //    - There is smt of the solid that is out of solid
        iCntSh = 0;
        iCntShInt = 0;
        aIt.Initialize(myShape);
        for (; aIt.More(); aIt.Next())
        {
            Shape aSx = aIt.Value();
            //
            if (aSx.ShapeType() != ShapeType.Shell)
            {
                aOr = aSx.Orientation();
                if (aOr != Internal)
                {
                    BRepCheck.Add(aLST, BRepCheck_BadOrientationOfSubshape);
                }

                continue;
            }

            //
            Shell aSh = ShapeCasts.Shell(aSx);
            //
            // Skip internal shells
            bFound = false;
            aItF.Initialize(aSh);
            for (; !bFound && aItF.More(); aItF.Next())
            {
                Shape aF = aItF.Value();
                aOr = aF.Orientation();
                if (aOr == Internal)
                {
                    bFound = !bFound;
                }
            }

            if (bFound)
            {
                ++iCntShInt;
                continue;
            }

            //
            ++iCntSh;
            //
            // Skip not closed shells
            if (!Tool.IsClosed(aSh))
            {
                continue;
            }

            //
            // OCCT.NET: a new unfrozen solid holding the (frozen) shell, never handed out
            aBB.MakeSolid(ref aZ);
            aBB.Add(aZ, aSh);
            //
            var aTS = new BRepCheck_ToolSolid();
            //
            aTS.SetSolid(aZ);
            aVTS.Add(aTS);
        } // for (; aIt.More(); aIt.Next()) {

        //
        if (iCntSh == 0 && iCntShInt != 0)
        {
            // all shells in the solid are internal
            BRepCheck.Add(aLST, BRepCheck_BadOrientationOfSubshape);
        }

        //
        aNbVTS = aVTS.Count;
        if (aNbVTS < 2)
        {
            return;
        }

        //
        aNbVTS1 = 0;
        for (i = 0; i < aNbVTS; ++i)
        {
            BRepCheck_ToolSolid aTS = aVTS[i];
            //
            aTS.Init();
            bIsHole = aTS.IsHole();
            if (!bIsHole)
            {
                ++aNbVTS1;
                if (aNbVTS1 > 1)
                {
                    // Too many growths
                    BRepCheck.Add(aLST, BRepCheck_EnclosedRegion);
                    break;
                }
            }
        }

        //
        // OCCT: after the break above, the remaining tools are never initialised, and IsOut below dereferences
        // their null classifier (the analyzer doesn't catch exceptions from Minimum)
        bFound = false;
        aNbVTS1 = aNbVTS - 1;
        for (i = 0; !bFound && i < aNbVTS1; ++i)
        {
            BRepCheck_ToolSolid aTSi = aVTS[i];
            //
            for (j = i + 1; !bFound && j < aNbVTS; ++j)
            {
                BRepCheck_ToolSolid aTSj = aVTS[j];
                //
                bFlag = aTSi.IsOut(aTSj);
                if (bFlag)
                {
                    // smt of solid is out of solid
                    BRepCheck.Add(aLST, BRepCheck_SubshapeNotInShape);
                    bFound = !bFound;
                }
            }
        }

        //
        // myMin = true;
    }
}
