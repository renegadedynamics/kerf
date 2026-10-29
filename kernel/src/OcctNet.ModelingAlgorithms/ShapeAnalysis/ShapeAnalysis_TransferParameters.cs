// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_TransferParameters.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_TransferParameters.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Transfers parameters between an edge's 3D curve and its pcurve on a face, by the linear map between their ranges
/// (the edge is taken as SameParameter). Subclasses (ShapeAnalysis_TransferParametersProj) compute it by projection.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="TransferRange"/> writes the ranges of a new edge in place, so it is
/// internal: ported code calls it on edges it is building.
/// </remarks>
public class ShapeAnalysis_TransferParameters : Standard_Transient
{
    protected double myFirst;
    protected double myLast;
    protected TopoDS_Edge myEdge = new TopoDS_Edge();
    // OCCT leaves it uninitialised until SetMaxTolerance; 0 here
    protected double myMaxTolerance;

    private double myShift;
    private double myScale;
    private double myFirst2d;
    private double myLast2d;
    private TopoDS_Face myFace = new TopoDS_Face();

    /// <summary>An empty tool: the identity map.</summary>
    public ShapeAnalysis_TransferParameters()
    {
        myScale = 1.0;
        myShift = 0.0;
    }

    /// <summary>A tool for the edge on the face (see <see cref="Init"/>).</summary>
    public ShapeAnalysis_TransferParameters(in TopoDS_Edge E, in TopoDS_Face F)
    {
        Init(E, F);
    }

    /// <summary>
    /// Loads the edge and the face, and computes the linear map from the 3D curve's range to the pcurve's (the
    /// identity if either is missing; a null face is allowed, for free edges).
    /// </summary>
    public virtual void Init(in TopoDS_Edge E, in TopoDS_Face F)
    {
        myScale = 1.0;
        myShift = 0.0;
        double l, f, l2d = 0.0, f2d = 0.0;
        myEdge = E;
        var sae = new ShapeAnalysis_Edge();
        sae.Curve3d(E, out Geom_Curve? curve3d, out f, out l, false);
        myFirst = f;
        myLast = l;
        Geom2d_Curve? curve2d = null;
        if (!F.IsNull())
        { // process free edges
            sae.PCurve(E, F, out curve2d, out f2d, out l2d, false);
        }

        myFirst2d = f2d;
        myLast2d = l2d;
        myFace = F;
        if (curve3d is null || curve2d is null)
        {
            return;
        }

        double ln2d = l2d - f2d;
        double ln3d = l - f;
        myScale = (ln3d <= gp.Resolution() ? 1.0 : ln2d / ln3d);
        myShift = f2d - f * myScale;
    }

    /// <summary>Sets the maximal tolerance to use in the computations.</summary>
    public void SetMaxTolerance(double maxtol)
    {
        myMaxTolerance = maxtol;
    }

    /// <summary>Transfers the parameters (to the pcurve if <paramref name="To2d"/>, else to the 3D curve).</summary>
    public virtual NCollection_HSequence<double> Perform(NCollection_HSequence<double> Params, bool To2d)
    {
        var res = new NCollection_HSequence<double>();
        for (int i = 1; i <= Params.Length(); i++)
        {
            res.Append(Perform(Params.Value(i), To2d));
        }

        return res;
    }

    /// <summary>Transfers one parameter (to the pcurve if <paramref name="To2d"/>, else to the 3D curve).</summary>
    public virtual double Perform(double Param, bool To2d)
    {
        double NewParam;
        if (To2d)
        {
            NewParam = myShift + Param * myScale;
        }
        else
        {
            NewParam = -myShift / myScale + Param * 1.0 / myScale;
        }

        return NewParam;
    }

    /// <summary>
    /// Sets on <paramref name="newEdge"/> (a part of the loaded edge) the ranges of the loaded edge's curves between
    /// the parameters <paramref name="prevPar"/> and <paramref name="currPar"/>, of the pcurve if
    /// <paramref name="Is2d"/>, else of the 3D curve (ShapeBuild_Edge::CopyRanges).
    /// </summary>
    /// <remarks>Internal: it writes into the edge, which throws on a frozen one.</remarks>
    internal virtual void TransferRange(in TopoDS_Edge newEdge, double prevPar, double currPar, bool Is2d)
    {
        var sbe = new ShapeBuild_Edge();
        if (Is2d)
        {
            double span2d = myLast2d - myFirst2d;
            double tmp1, tmp2;
            if (prevPar > currPar)
            {
                tmp1 = currPar;
                tmp2 = prevPar;
            }
            else
            {
                tmp1 = prevPar;
                tmp2 = currPar;
            }

            double alpha = (tmp1 - myFirst2d) / span2d;
            double beta = (tmp2 - myFirst2d) / span2d;
            sbe.CopyRanges(newEdge, myEdge, alpha, beta);
        }
        else
        {
            double alpha = (prevPar - myFirst) / (myLast - myFirst);
            double beta = (currPar - myFirst) / (myLast - myFirst);
            sbe.CopyRanges(newEdge, myEdge, alpha, beta);
        }
    }

    /// <summary>Tells if the map is the identity (no shift, scale 1).</summary>
    public virtual bool IsSameRange()
    {
        return myShift == 0.0 && myScale == 1.0;
    }
}
