// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ClosedFaceDivide.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ClosedFaceDivide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Divides a closed face (one with a seam, or on a geometrically closed surface) into the given number of parts plus
/// one, across the seam's direction.
/// </summary>
/// <remarks>A working object (decision 21). The face given is not modified (see <see cref="ShapeUpgrade_FaceDivide"/>).</remarks>
public class ShapeUpgrade_ClosedFaceDivide : ShapeUpgrade_FaceDivide
{
    private int myNbSplit;

    /// <summary>Empty constructor (one splitting line).</summary>
    public ShapeUpgrade_ClosedFaceDivide()
    {
        myNbSplit = 1;
    }

    /// <summary>Initializes by a face.</summary>
    public ShapeUpgrade_ClosedFaceDivide(in Face F) : base(F)
    {
        myNbSplit = 1;
    }

    /// <summary>Splits the surface and computes the shell from the face; true if it was split.</summary>
    public override bool SplitSurface(double theArea = 0.0)
    {
        ShapeUpgrade_SplitSurface? SplitSurf = GetSplitSurfaceTool();
        if (SplitSurf is null)
        {
            return false;
        }

        if (myResult.IsNull || myResult.ShapeType() != ShapeType.Face)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
            return false;
        }
        Face face = ShapeCasts.Face(myResult);

        ShapeAnalysis.GetFaceUVBounds(myFace, out double Uf, out double Ul, out double Vf, out double Vl);
        // 01.10.99 pdn Porting on DEC
        if (Kerf.Engine.Foundation.Precision.IsInfinite(Uf) || Kerf.Engine.Foundation.Precision.IsInfinite(Ul) || Kerf.Engine.Foundation.Precision.IsInfinite(Vf)
            || Kerf.Engine.Foundation.Precision.IsInfinite(Vl))
        {
            return false;
        }

        Surface surf = Tool.Surface(face, out Placement L)!;

        bool isUSplit = false;
        bool doSplit = false;
        var split = new NCollection_HSequence<double>();

        for (var iter = new Iterator(face); iter.More() && !doSplit; iter.Next())
        {
            if (iter.Value().ShapeType() != ShapeType.Wire)
            {
                continue;
            }
            Wire wire = ShapeCasts.Wire(iter.Value());
            var sewd = new ShapeExtend_WireData(wire);
            for (int i = 1; i <= sewd.NbEdges() && !doSplit; i++)
            {
                if (sewd.IsSeam(i))
                {
                    doSplit = true;
                    Edge edge = sewd.Edge(i);
                    var sae = new ShapeAnalysis_Edge();
                    if (!sae.PCurve(edge, face, out Curve2D? c1, out double f1, out double l1, false))
                    {
                        continue;
                    }
                    Shape tmpE = edge.Reversed();
                    if (!sae.PCurve(ShapeCasts.Edge(tmpE), face, out Curve2D? c2, out double f2, out double l2, false))
                    {
                        continue;
                    }
                    if (ReferenceEquals(c2, c1))
                    {
                        continue;
                    }
                    // splitting
                    var sac = new ShapeAnalysis_Curve();
                    var B1 = new Box2d();
                    var B2 = new Box2d();
                    sac.FillBndBox(c1!, f1, l1, 20, true, ref B1);
                    sac.FillBndBox(c2!, f2, l2, 20, true, ref B2);
                    B1.Get(out double x1min, out double y1min, out double x1max, out double y1max);
                    B2.Get(out double x2min, out double y2min, out double x2max, out double y2max);
                    double xf, xl, yf, yl;
                    if (x1min < x2min)
                    {
                        xf = x1max;
                        xl = x2min;
                    }
                    else
                    {
                        xf = x2max;
                        xl = x1min;
                    }
                    if (y1min < y2min)
                    {
                        yf = y1max;
                        yl = y2min;
                    }
                    else
                    {
                        yf = y2max;
                        yl = y1min;
                    }

                    double dU = xl - xf;
                    double dV = yl - yf;
                    if (dU > dV)
                    {
                        double step = dU / (myNbSplit + 1);
                        double val = xf + step;
                        for (int j = 1; j <= myNbSplit; j++, val += step)
                        {
                            split.Append(val);
                        }
                        isUSplit = true;
                    }
                    else
                    {
                        double step = dV / (myNbSplit + 1);
                        double val = yf + step;
                        for (int j = 1; j <= myNbSplit; j++, val += step)
                        {
                            split.Append(val);
                        }
                        isUSplit = false;
                    }
                }
            }
        }

        if (!doSplit)
        {
            // pdn try to define geometric closure.
            var sas = new ShapeAnalysis_Surface(surf);
            bool uclosed = sas.IsUClosed(Precision());
            bool vclosed = sas.IsVClosed(Precision());
            double U1, U2, V1, V2;
            if (uclosed)
            {
                surf.Bounds(out U1, out U2, out V1, out V2);
                var GAS = new SurfaceAdaptor(surf);
                double toler = GAS.UResolution(Precision());
                if ((U2 - U1) - (Ul - Uf) < toler)
                {
                    var rts = new TrimmedSurface(surf, U1, (U2 + U1) / 2, true);
                    var sast = new ShapeAnalysis_Surface(rts);
                    if (!sast.IsUClosed(Precision()))
                    {
                        doSplit = true;
                        double step = (Ul - Uf) / (myNbSplit + 1);
                        double val = Uf + step;
                        for (int i = 1; i <= myNbSplit; i++, val += step)
                        {
                            split.Append(val);
                        }
                        isUSplit = true;
                    }
                    // OCCT_DEBUG output left out: thin face, not split
                }
            }

            if (vclosed && !doSplit)
            {
                surf.Bounds(out U1, out U2, out V1, out V2);
                var GAS = new SurfaceAdaptor(surf);
                double toler = GAS.VResolution(Precision());
                if ((V2 - V1) - (Vl - Vf) < toler)
                {
                    var rts = new TrimmedSurface(surf, V1, (V2 + V1) / 2, false);
                    var sast = new ShapeAnalysis_Surface(rts);
                    if (!sast.IsVClosed(Precision()))
                    {
                        doSplit = true;
                        double step = (Vl - Vf) / (myNbSplit + 1);
                        double val = Vf + step;
                        for (int i = 1; i <= myNbSplit; i++, val += step)
                        {
                            split.Append(val);
                        }
                        isUSplit = false;
                    }
                    // OCCT_DEBUG output left out: thin face, not split
                }
            }
        }

        if (!doSplit)
        {
            return false;
        }

        SplitSurf.Init(surf, Uf, Ul, Vf, Vl);
        if (isUSplit)
        {
            SplitSurf.SetUSplitValues(split);
        }
        else
        {
            SplitSurf.SetVSplitValues(split);
        }

        SplitSurf.Perform(mySegmentMode);
        if (!SplitSurf.Status(ShapeExtend_DONE))
        {
            return false;
        }

        ShapeExtend_CompositeSurface Grid = SplitSurf.ResSurfaces()!;

        var CompShell = new ShapeFix_ComposeShell();
        CompShell.Init(Grid, L, face, Precision());
        CompShell.SetMaxTolerance(MaxTolerance());
        CompShell.SetContext(Context());
        CompShell.Perform();
        if (CompShell.Status(ShapeExtend_FAIL) || !CompShell.Status(ShapeExtend_DONE))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
        }

        // OCCT.NET: the result unfrozen, the caller's sub-shapes given back where the face was copied (decision 999)
        Shape res = CompShell.InternalFinishedResult();
        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        for (var exp = new Explorer(res, ShapeType.Face); exp.More(); exp.Next())
        {
            Shape tempf = Context()!.InternalApply(exp.Current());
            Face f = ShapeCasts.Face(tempf);
            myResult = f;
            if (SplitSurface())
            {
                Context()!.Replace(f, myResult);
            }
        }
        myResult = Context()!.InternalApply(res);
        return true;
    }

    /// <summary>Sets the number of cutting lines the closed face is split by (the resulting faces are num + 1).</summary>
    public void SetNbSplitPoints(int num)
    {
        if (num > 0)
        {
            myNbSplit = num;
        }
    }

    /// <summary>Returns the number of splitting points.</summary>
    public int GetNbSplitPoints() => myNbSplit;
}
