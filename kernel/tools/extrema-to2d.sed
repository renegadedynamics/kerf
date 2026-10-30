# Generates the 2D curve extrema from the 3D ones (decision 218): for each of Extrema_PCFOfEPCOfExtPC, EPCOfExtPC,
# ExtPC, LocEPCOfLocateExtPC, LocateExtPC, CCLocFOfLocECC, ECC and LocECC in src/OcctNet.ModelingData/Extrema,
#   sed -E -f tools/extrema-to2d.sed Extrema_X.cs > Extrema_X2d.cs
# The 2D files are its output with no hand edits.
s/\b(Extrema_PCFOfEPCOfExtPC|Extrema_EPCOfExtPC|Extrema_ExtPC|Extrema_LocEPCOfLocateExtPC|Extrema_LocateExtPC|Extrema_CCLocFOfLocECC|Extrema_ECC|Extrema_LocECC|Extrema_ELPCOfLocateExtPC|Extrema_EPCOfELPCOfLocateExtPC|Extrema_PCFOfEPCOfELPCOfLocateExtPC|Extrema_PCLocFOfLocEPCOfLocateExtPC|Extrema_ExtPElC|Extrema_POnCurv)\b/\12d/g
s/\bExtrema_CurveTool\b/Extrema_Curve2dTool/g
s/\bAdaptor3d_Curve\b/Adaptor2d_Curve2d/g
s/\bgp_Pnt\b/gp_Pnt2d/g
s/\bgp_Vec\b/gp_Vec2d/g
s/\bGeom_BSplineCurve\b/Geom2d_BSplineCurve/g
s/\bGeom_BezierCurve\b/Geom2d_BezierCurve/g
s/3D/2D/g
