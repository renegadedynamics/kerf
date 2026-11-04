// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CheckerSI.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CheckerSI.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>A face intersected with itself (OCCT's file-local BOPAlgo_FaceSelfIntersect).</summary>
/// <remarks>
/// OCCT derives it from IntTools_FaceFace and BOPAlgo_ParallelAlgo; C# has one base, so it holds a
/// <see cref="Options"/> for the run-parallel flag and the user break, as the pave filler's BOPAlgo_FaceFace
/// (decision 786). As in OCCT, an exception isn't caught here: it ends the checker with
/// <see cref="AlertIntersectionFailed"/>.
/// </remarks>
internal sealed class FaceSelfIntersect : FaceFace, BooleanParallel.ISolver
{
    private readonly Options _options = new();
    private ProgressRange _progressRange;
    private int _if;
    private double _tolF;
    private Face _f = new();

    public FaceSelfIntersect()
    {
        _if = -1;
        _tolF = 1.0e-7;
    }

    //
    public void SetIndex(int nF) => _if = nF;

    //
    public int IndexOfFace() => _if;

    //
    public void SetFace(in Face aF) => _f = aF;

    //
    public Face Face() => _f;

    //
    public void SetTolF(double aTolF) => _tolF = aTolF;

    //
    public double TolF() => _tolF;

    public void SetProgressRange(ProgressRange range) => _progressRange = range;

    public void SetRunParallel(bool flag) => _options.SetRunParallel(flag);

    //
    public void Perform()
    {
        using var ps = new ProgressScope(_progressRange, null, 1);
        if (_options.UserBreak(ps))
        {
            return;
        }
        Perform(_f, _f, _options.RunParallel());
    }
}

/// <summary>
/// Checks a shape on self-interference: the pave filler intersects the sub-shapes of its single argument with each
/// other, faces are intersected with themselves, and vertices, edges, faces and solids are tested against solids. The
/// pairs of interfering sub-shapes are the data structure's <see cref="DS.Interferences"/>.
/// </summary>
/// <remarks>
/// <para>
/// The level of check (<see cref="SetLevelOfCheck"/>) limits the types of interference: 0 V/V; 1 and V/E; 2 and E/E;
/// 3 and V/F; 4 and E/F; 5 and F/F (with the faces' self-intersection); 6 and V/S; 7 and E/S; 8 and F/S; 9 and S/S
/// (the default, all).
/// </para>
/// <para>
/// The checker is non-destructive (OCCT sets the mode in its constructor and its <c>Init</c> doesn't reset it), yet
/// OCCT's filler still raises the tolerances of argument vertices that a section curve passes through
/// (<c>PutPaveOnCurve</c>, occt-issues.md). The check runs in a working overlay: those writes go to working copies,
/// which the check reads and then drops, so the argument (frozen, the usual case) is never written and the data
/// structure holds its own sub-shapes. The class is partial over OCCT's two files.
/// </para>
/// </remarks>
internal partial class CheckerSI : PaveFiller
{
    protected int _levelOfCheck;

    public CheckerSI()
    {
        _levelOfCheck = global::Kerf.Engine.Booleans.Data.DS.InterfTypeCount - 1;
        _nonDestructive = true;
        SetAvoidBuildPCurve(true);
    }

    /// <summary>Sets the level of checking the shape on self-interference (0 to 9; other values are ignored).</summary>
    public void SetLevelOfCheck(int level)
    {
        int nbLists;
        //
        nbLists = global::Kerf.Engine.Booleans.Data.DS.InterfTypeCount;
        if (level >= 0 && level < nbLists)
        {
            _levelOfCheck = level;
        }
    }

    protected override void Init(ProgressRange range)
    {
        Clear();
        //
        // 1. myDS
        _ds = new DS();
        _ds.SetArguments(_arguments);
        _ds.Init(_fuzzyValue);
        //
        // 2 myContext
        _context = new IntersectionContext();
        //
        // 3.myIterator
        IteratorSI theIterSI = new();
        theIterSI.SetDS(_ds);
        theIterSI.Prepare(_context, _useOBB, _fuzzyValue);
        theIterSI.UpdateByLevelOfCheck(_levelOfCheck);
        //
        _iterator = theIterSI;
    }

    /// <summary>Checks the single argument; an exception becomes <see cref="AlertIntersectionFailed"/>.</summary>
    public override void Perform(ProgressRange range = default)
    {
        try
        {
            //
            if (_arguments.Extent() != 1)
            {
                AddError(new AlertMultipleArguments());
                return;
            }
            //
            // Kerf: the filler's writes into the argument go to the overlay's working copies, dropped at the end
            // (the checker hands out no shape built from them)
            using (WorkingOverlay.Begin(_arguments.First()))
            {
                PerformChecks(range);
            }
        }
        //
        catch (FailureException)
        {
            AddError(new AlertIntersectionFailed());
        }
    }

    // OCCT's Perform from its progress scope on (the body of its try block).
    private void PerformChecks(ProgressRange range)
    {
        {
            using var ps = new ProgressScope(range, "Checking shape on self-intersection", 10);
            // Perform intersection of sub shapes
            base.Perform(ps.Next(8));
            if (UserBreak(ps))
            {
                return;
            }
            //
            CheckFaceSelfIntersection(ps.Next());

            using var pszz = new ProgressScope(ps.Next(), null, 4);
            // Perform intersection with solids
            if (!HasErrors)
            {
                PerformVZ(pszz.Next());
            }
            //
            if (!HasErrors)
            {
                PerformEZ(pszz.Next());
            }
            //
            if (!HasErrors)
            {
                PerformFZ(pszz.Next());
            }
            //
            if (!HasErrors)
            {
                PerformZZ(pszz.Next());
            }
            //
            if (HasErrors)
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
        int i, nb, n1, n2;
        Pair pk = new();
        //
        MapOfPair mpk = _ds!.Interferences();

        // 0
        List<InterfVV> vVs = _ds.InterfVV();
        nb = vVs.Count;
        for (i = 0; i != nb; ++i)
        {
            InterfVV vv = vVs[i];
            vv.Indices(out n1, out n2);
            if (_ds.IsNewShape(n1) || _ds.IsNewShape(n2))
            {
                continue;
            }
            pk.SetIndices(n1, n2);
            mpk.Add(pk);
        }
        //
        // 1
        List<InterfVE> vEs = _ds.InterfVE();
        nb = vEs.Count;
        for (i = 0; i != nb; ++i)
        {
            InterfVE ve = vEs[i];
            ve.Indices(out n1, out n2);
            if (_ds.IsNewShape(n1) || _ds.IsNewShape(n2))
            {
                continue;
            }
            pk.SetIndices(n1, n2);
            mpk.Add(pk);
        }
        //
        // 2
        List<InterfEE> eEs = _ds.InterfEE();
        nb = eEs.Count;
        for (i = 0; i != nb; ++i)
        {
            InterfEE ee = eEs[i];
            ee.Indices(out n1, out n2);
            if (_ds.IsNewShape(n1) || _ds.IsNewShape(n2))
            {
                continue;
            }
            pk.SetIndices(n1, n2);
            mpk.Add(pk);
        }
        //
        // 3
        List<InterfVF> vFs = _ds.InterfVF();
        nb = vFs.Count;
        for (i = 0; i != nb; ++i)
        {
            InterfVF vf = vFs[i];
            vf.Indices(out n1, out n2);
            if (_ds.IsNewShape(n1) || _ds.IsNewShape(n2))
            {
                continue;
            }
            pk.SetIndices(n1, n2);
            mpk.Add(pk);
        }
        //
        // 4
        List<InterfEF> eFs = _ds.InterfEF();
        nb = eFs.Count;
        for (i = 0; i != nb; ++i)
        {
            InterfEF ef = eFs[i];
            if (ef.CommonPart().Type() == ShapeType.Shape)
            {
                continue;
            }
            ef.Indices(out n1, out n2);
            if (_ds.IsNewShape(n1) || _ds.IsNewShape(n2))
            {
                continue;
            }
            pk.SetIndices(n1, n2);
            mpk.Add(pk);
        }
        //
        // 5
        List<InterfFF> fFs = _ds.InterfFF();
        nb = fFs.Count;
        for (i = 0; i != nb; ++i)
        {
            bool bTangentFaces, bFlag;
            int nbC, nbP, j, iFound;
            //
            InterfFF ff = fFs[i];
            ff.Indices(out n1, out n2);
            //
            bTangentFaces = ff.TangentFaces();
            nbP = ff.Points().Count;
            List<SectionCurve> vc = ff.Curves();
            nbC = vc.Count;
            if (nbP == 0 && nbC == 0 && !bTangentFaces)
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
                    Face f1 = ShapeCasts.Face(_ds.Shape(n1));
                    Face f2 = ShapeCasts.Face(_ds.Shape(n2));
                    bFlag = AlgoTools.AreFacesSameDomain(f1, f2, _context!, _fuzzyValue);
                    if (bFlag)
                    {
                        ++iFound;
                    }
                }
                else
                {
                    for (j = 0; j != nbC; ++j)
                    {
                        SectionCurve nc = vc[j];
                        NCollection_List<PaveBlock> lpbc = nc.PaveBlocks();
                        if (lpbc.Extent() != 0)
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
            pk.SetIndices(n1, n2);
            mpk.Add(pk);
        }
        //
        //
        // 6
        List<InterfVZ> vZs = _ds.InterfVZ();
        nb = vZs.Count;
        for (i = 0; i != nb; ++i)
        {
            //
            InterfVZ vz = vZs[i];
            vz.Indices(out n1, out n2);
            if (_ds.IsNewShape(n1) || _ds.IsNewShape(n2))
            {
                continue;
            }
            pk.SetIndices(n1, n2);
            mpk.Add(pk);
        }
        //
        // 7
        List<InterfEZ> eZs = _ds.InterfEZ();
        nb = eZs.Count;
        for (i = 0; i != nb; ++i)
        {
            //
            InterfEZ ez = eZs[i];
            ez.Indices(out n1, out n2);
            pk.SetIndices(n1, n2);
            mpk.Add(pk);
        }
        //
        // 8
        List<InterfFZ> fZs = _ds.InterfFZ();
        nb = fZs.Count;
        for (i = 0; i != nb; ++i)
        {
            //
            InterfFZ fz = fZs[i];
            fz.Indices(out n1, out n2);
            pk.SetIndices(n1, n2);
            mpk.Add(pk);
        }
        //
        // 9
        List<InterfZZ> zZs = _ds.InterfZZ();
        nb = zZs.Count;
        for (i = 0; i != nb; ++i)
        {
            //
            InterfZZ zz = zZs[i];
            zz.Indices(out n1, out n2);
            pk.SetIndices(n1, n2);
            mpk.Add(pk);
        }
    }

    /// <summary>Intersects the faces that may self-intersect (not planes, cylinders, cones, spheres or ring tori) with themselves.</summary>
    protected void CheckFaceSelfIntersection(ProgressRange range)
    {
        if (_levelOfCheck < 5)
        {
            return;
        }

        Pair pk = new();

        MapOfPair mpk = _ds!.Interferences();
        mpk.Clear();

        List<FaceSelfIntersect> vFace = [];

        int nbS = _ds.SourceShapeCount;

        using var psOuter = new ProgressScope(range, null, 1);

        //
        for (int i = 0; i < nbS; i++)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() != ShapeType.Face)
            {
                continue;
            }
            //
            Face f = ShapeCasts.Face(si.Shape());
            FaceSurfaceAdaptor bAsurf = new(f, false);
            SurfaceKind surfType = bAsurf.GetType();
            if (surfType == Plane || surfType == SurfaceKind.Cylinder || surfType == SurfaceKind.Cone
                || surfType == SurfaceKind.Sphere)
            {
                continue;
            }

            if (surfType == SurfaceKind.Torus)
            {
                Torus torus = bAsurf.Torus();
                double majorRadius = torus.MajorRadius;
                double minorRadius = torus.MinorRadius;
                if (majorRadius > minorRadius + global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    continue;
                }
            }

            double tolF = Tool.Tolerance(f);

            FaceSelfIntersect faceSelfIntersect = new();
            vFace.Add(faceSelfIntersect);
            //
            faceSelfIntersect.SetRunParallel(_runParallel);
            faceSelfIntersect.SetIndex(i);
            faceSelfIntersect.SetFace(f);
            faceSelfIntersect.SetTolF(tolF);
        }

        int nbFace = vFace.Count;
        using var psParallel = new ProgressScope(psOuter.Next(), "Checking surface on self-intersection", nbFace);
        for (int iF = 0; iF < nbFace; ++iF)
        {
            vFace[iF].SetProgressRange(psParallel.Next());
        }
        //======================================================
        BooleanParallel.Perform(_runParallel, vFace);
        //======================================================
        if (UserBreak(psOuter))
        {
            return;
        }
        //
        for (int k = 0; k < nbFace; k++)
        {
            FaceSelfIntersect faceSelfIntersect = vFace[k];
            //
            int nF = faceSelfIntersect.IndexOfFace();

            bool bIsDone = faceSelfIntersect.IsDone;
            if (bIsDone)
            {
                NCollection_Sequence<IntersectionCurve> cvsX = faceSelfIntersect.Lines();
                NCollection_Sequence<PntOn2Faces> pntsX = faceSelfIntersect.Points();
                //
                int nbCurves = cvsX.Length();
                int nbPoints = pntsX.Length();
                //
                if (nbCurves != 0 || nbPoints != 0)
                {
                    pk.SetIndices(nF, nF);
                    mpk.Add(pk);
                }
            }
        }
    }
}
