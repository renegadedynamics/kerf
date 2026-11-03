// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Section.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Section.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Booleans.Operation;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Api;

/// <summary>
/// The section of two shapes (or planes, or surfaces): the edges and vertices where they meet, as a compound.
/// </summary>
/// <remarks>
/// The options: <see cref="Approximation"/> (approximate the section curves as B-splines),
/// <see cref="ComputePCurveOn1"/> and <see cref="ComputePCurveOn2"/> (attach p-curves on the first or second
/// argument's faces). A plane or surface argument becomes a face (C2 or smoother surfaces) or a shell
/// (BRepBuilderAPI_MakeShell), as in OCCT.
/// </remarks>
public class SectionOperation : BooleanOperation
{
    private bool _approx;
    private bool _computePCurve1;
    private bool _computePCurve2;

    /// <summary>An empty section.</summary>
    public SectionOperation()
    {
        Init(false);
    }

    /// <summary>A section using the intersections already computed by <paramref name="aPF"/>.</summary>
    public SectionOperation(PaveFiller aPF)
        : base(aPF)
    {
        Init(false);
    }

    /// <summary>The section of two shapes, built at once unless <paramref name="performNow"/> is false.</summary>
    public SectionOperation(in Shape sh1, in Shape sh2, bool performNow = true)
        : base(sh1, sh2, global::Kerf.Engine.Booleans.Operation.Section)
    {
        Init(performNow);
    }

    /// <summary>The section of two shapes with the intersections already computed by <paramref name="aDSF"/>.</summary>
    public SectionOperation(in Shape aS1, in Shape aS2, PaveFiller aDSF, bool performNow = true)
        : base(aS1, aS2, aDSF, global::Kerf.Engine.Booleans.Operation.Section)
    {
        Init(performNow);
    }

    /// <summary>The section of a shape by a plane.</summary>
    public SectionOperation(in Shape sh, in Plane3D pl, bool performNow = true)
        : base(sh, MakeShape(new PlaneSurface(pl)), global::Kerf.Engine.Booleans.Operation.Section)
    {
        Init(performNow);
    }

    /// <summary>The section of a shape by a surface.</summary>
    public SectionOperation(in Shape sh, Surface sf, bool performNow = true)
        : base(sh, MakeShape(sf), global::Kerf.Engine.Booleans.Operation.Section)
    {
        Init(performNow);
    }

    /// <summary>The section of a surface by a shape.</summary>
    public SectionOperation(Surface sf, in Shape sh, bool performNow = true)
        : base(MakeShape(sf), sh, global::Kerf.Engine.Booleans.Operation.Section)
    {
        Init(performNow);
    }

    /// <summary>The section of two surfaces.</summary>
    public SectionOperation(Surface sf1, Surface sf2, bool performNow = true)
        : base(MakeShape(sf1), MakeShape(sf2), global::Kerf.Engine.Booleans.Operation.Section)
    {
        Init(performNow);
    }

    /// <summary>Sets the operation and the default options, and builds if asked.</summary>
    protected void Init(bool bFlag)
    {
        _operation = global::Kerf.Engine.Booleans.Operation.Section;
        _approx = false;
        _computePCurve1 = false;
        _computePCurve2 = false;
        //
        if (bFlag)
        {
            Build();
        }
    }

    /// <summary>Sets the first argument (replacing the objects).</summary>
    public void Init1(in Shape s1)
    {
        _arguments.Clear();
        _arguments.Append(s1);
    }

    /// <summary>Sets the first argument to a plane.</summary>
    public void Init1(in Plane3D pl) => Init1(MakeShape(new PlaneSurface(pl)));

    /// <summary>Sets the first argument to a surface.</summary>
    public void Init1(Surface sf) => Init1(MakeShape(sf));

    /// <summary>Sets the second argument (replacing the tools).</summary>
    public void Init2(in Shape s2)
    {
        _tools.Clear();
        _tools.Append(s2);
    }

    /// <summary>Sets the second argument to a plane.</summary>
    public void Init2(in Plane3D pl) => Init2(MakeShape(new PlaneSurface(pl)));

    /// <summary>Sets the second argument to a surface.</summary>
    public void Init2(Surface sf) => Init2(MakeShape(sf));

    /// <summary>Approximates the section curves as B-splines.</summary>
    public void Approximation(bool b) => _approx = b;

    /// <summary>Attaches to the section edges their p-curves on the faces of the first argument.</summary>
    public void ComputePCurveOn1(bool b) => _computePCurve1 = b;

    /// <summary>Attaches to the section edges their p-curves on the faces of the second argument.</summary>
    public void ComputePCurveOn2(bool b) => _computePCurve2 = b;

    /// <summary>Gives the section options to the intersection tool.</summary>
    protected override void SetAttributes()
    {
        SectionAttribute theSecAttr = new(_approx, _computePCurve1, _computePCurve2);
        _dsFiller!.SetSectionAttribute(theSecAttr);
    }

    /// <summary>Performs the section.</summary>
    public override void Build(ProgressRange range = default) => base.Build(range);

    /// <summary>
    /// The face of the first argument on which the section edge <paramref name="e"/> lies, and true; false (and a null
    /// shape) if there is none. OCCT leaves the output untouched when it returns false. The face is published (frozen).
    /// </summary>
    public bool HasAncestorFaceOn1(in Shape e, out Shape f)
    {
        bool bRes;
        f = new Shape();
        bRes = HasAncestorFace(_dsFiller, 1, e, ref f);
        f.TShape()?.FreezeGraph();
        return bRes;
    }

    /// <summary>The face of the second argument on which the section edge <paramref name="e"/> lies (see the first form).</summary>
    public bool HasAncestorFaceOn2(in Shape e, out Shape f)
    {
        bool bRes;
        f = new Shape();
        bRes = HasAncestorFace(_dsFiller, 2, e, ref f);
        f.TShape()?.FreezeGraph();
        return bRes;
    }

    private static bool HasAncestorFace(PaveFiller? pPF, int aIndex, in Shape aE, ref Shape aF)
    {
        bool bRes;
        //
        bRes = false;
        if (aE.IsNull)
        {
            return bRes;
        }
        if (aE.ShapeType() != ShapeType.Edge)
        {
            return bRes;
        }
        //
        Shape f1 = new(), f2 = new();
        //
        bRes = HasAncestorFaces(pPF!, aE, ref f1, ref f2);
        if (!bRes)
        {
            return bRes;
        }
        //
        aF = (aIndex == 1) ? f1 : f2;
        return bRes;
    }

    private static bool HasAncestorFaces(PaveFiller pPF, in Shape aEx, ref Shape aF1, ref Shape aF2)
    {
        int nbFF, i, j, nE, nbVC;
        //
        DS pDS = pPF.PDS()!;
        List<InterfFF> fFs = pDS.InterfFF();
        //
        nbFF = fFs.Count;
        for (i = 0; i < nbFF; ++i)
        {
            InterfFF fFi = fFs[i];
            fFi.Indices(out int nF1, out int nF2);
            //
            List<SectionCurve> vc = fFi.Curves();
            nbVC = vc.Count;
            for (j = 0; j < nbVC; j++)
            {
                SectionCurve bc = vc[j];
                //
                for (NCollection_List<PaveBlock>.Iterator itLPB = new(bc.PaveBlocks()); itLPB.More(); itLPB.Next())
                {
                    PaveBlock pb = itLPB.Value();
                    nE = pb.Edge();
                    if (nE < 0)
                    {
                        continue;
                    }
                    //
                    Shape e = pDS.Shape(nE);
                    if (aEx.IsSame(e))
                    {
                        aF1 = pDS.Shape(nF1);
                        aF2 = pDS.Shape(nF2);
                        return true;
                    }
                }
            }
        }
        return false;
    }

    // A face (C2 or smoother surfaces) or a shell on the whole surface, unfrozen (made here, as in OCCT).
    private static Shape MakeShape(Surface s)
    {
        Continuity c = s.Continuity();
        if (c >= C2)
        {
            return new MakeFace(s, global::Kerf.Engine.Foundation.Precision.Confusion()).InternalShape();
        }
        return new MakeShell(s).InternalShape();
    }
}
